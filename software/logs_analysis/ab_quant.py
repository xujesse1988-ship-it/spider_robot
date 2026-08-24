#!/usr/bin/env python3
"""零力交接 A/B 量化管线（HANDOVER-DESIGN §9.3；协议 docs/HANDOVER-AB-PROTOCOL.md）。

08-20 基线量化（html/vent-snap-20260820.html）的管线入库参数化版：原脚本
（ncc_track/decompose/vent_zoom）散在会话 scratchpad，已抢救合并至此。NCC
打分公式、亚像素抛物线、分解窗口口径与原版逐字相同（本文件曾用 08-20 原
视频+日志回归验证：off/比例/逐腿弹跳/每轮下滑与报告一致），改动只有三点：
  - 事件表从黑匣子日志自动提取（原为手抄 12 行，曾抄错一行返工）；
  - 比例改参数：B 组全程位移 <10mm，不能再按"总位移人工读数"标比例——
    A 组用 --calib 标出 mm/px 后**相机锁死不动**，B 组 --mmppx 沿用；
  - 对时优先用"倾身对时标记"（协议要求 A/B 都在首轮前做 ↑↓ 各一档，
    B 组抬落身体几乎不动，原"活动对比度"法会失锁），无标记自动回退原法。

依赖：ffmpeg/ffprobe、numpy、Pillow（开发机跑，不上树莓派）。帧序列/轨迹按
--out 目录缓存并带参数指纹（cache_meta.json）：换视频/fps/模板矩形/对时会
自动作废重算，上次被 Ctrl-C 截断的帧序列也会被抓住重抽——旧缓存静默沿用
过一次就是一份错数。⚠ 缓存勿放 /tmp：帧序列多则数 GB，/tmp 配额一满整机
Bash 静默失败（项目已知事故），放 home 下目录（如 ~/ab_cache/A）。

用法：
  # 先只抽帧，在 OUT/seq/f00001.jpg 上量模板矩形（Y0:Y1,X0:X1）
  python ab_quant.py --video A.mp4 --log A.log --out ~/ab_cache/A --frames-only
  # A 组（基线对照）：比例按玻璃上胶带尺人工读数标定（T0/T1 为日志时刻）
  python ab_quant.py --video A.mp4 --log A.log --out ~/ab_cache/A \
      --body 280:372,340:480 --ref 640:730,250:380 --calib 142.3,327.5,131
  # B 组（--handover）：沿用 A 组比例
  python ab_quant.py --video B.mp4 --log B.log --out ~/ab_cache/B \
      --body ... --ref ... --mmppx 0.881
  # 第 N 次抬落（序号见分解表）的原生帧率破裂瞬间剖面 + 释放脚回弹
  python ab_quant.py ... --zoom N

日志兼容：body_lean 与 climb_walk 都认——climb_walk 连续行走不经 HOVER、
落地不打"落地："（单步打"单步落地（后半步）："），缺的事件从相位转移行
（lift→transfer / transfer→descend）推导，§9.4 的净前进率复测同用本管线。

模板选择：body=机身绿 PCB 纹理块（含安装孔等高对比特征）；ref=画面里保证
不动的静物块（窗台花盆等，用于相机漂移修正）。两块都取第 0 帧坐标。
"""
import argparse
import glob
import json
import math
import os
import re
import subprocess

import numpy as np
from PIL import Image

BODY_RY, BODY_RX = 25, 12   # 逐帧搜索半径 px（身体主要沿画面 y 走）——基准值，
                            # 按 BASE_FPS/BASE_W/BASE_H 口径标定；track() 按实际
                            # fps 与分辨率缩放：半径的物理含义是"帧间最大可跟踪
                            # 位移"（基线尺度 25px≈22mm/帧，弹跳 20.9 过冲 22.9
                            # 已贴顶），抽帧慢/画幅大时不缩会静默钉在窗边低报
REF_RY, REF_RX = 8, 8
BASE_FPS = 3.0              # 半径基准口径 = 08-20 基线（3fps、544×960 竖幅）
BASE_W, BASE_H = 544, 960


def gray(path):
    return np.asarray(Image.open(path).convert("L"), dtype=np.float64)


def _ncc_once(img, tpl, cy, cx, ry, rx):
    """单次 NCC 搜索。返回 (y, x, score, 峰贴搜索窗边)——贴边只算窗还能向外
    扩的方向（被图像边界钳住的不算，扩窗也无帧外数据）。"""
    th, tw = tpl.shape
    y0 = max(0, int(cy) - ry)
    y1 = min(img.shape[0] - th, int(cy) + ry)
    x0 = max(0, int(cx) - rx)
    x1 = min(img.shape[1] - tw, int(cx) + rx)
    if y1 < y0 or x1 < x0:
        return cy, cx, -1.0, False
    region = img[y0:y1 + th, x0:x1 + tw]
    win = np.lib.stride_tricks.sliding_window_view(region, (th, tw))
    t0 = tpl - tpl.mean()
    tn = np.sqrt((t0 * t0).sum())
    n = float(th * tw)
    ws = win.sum((-2, -1))
    w2 = np.einsum("abij,abij->ab", win, win)
    cc = np.einsum("abij,ij->ab", win, t0)
    sc = cc / (np.sqrt(np.maximum(w2 - ws * ws / n, 0.0)) * tn + 1e-9)
    iy, ix = np.unravel_index(np.argmax(sc), sc.shape)
    s = float(sc[iy, ix])
    edge = (iy == 0 and y0 > 0) \
        or (iy == sc.shape[0] - 1 and y1 < img.shape[0] - th) \
        or (ix == 0 and x0 > 0) \
        or (ix == sc.shape[1] - 1 and x1 < img.shape[1] - tw)

    def para(sm1, s0, sp1):
        den = sm1 - 2 * s0 + sp1
        return 0.0 if abs(den) < 1e-12 else 0.5 * (sm1 - sp1) / den

    dy = para(sc[iy - 1, ix], s, sc[iy + 1, ix]) \
        if 0 < iy < sc.shape[0] - 1 else 0.0
    dx = para(sc[iy, ix - 1], s, sc[iy, ix + 1]) \
        if 0 < ix < sc.shape[1] - 1 else 0.0
    return y0 + iy + dy, x0 + ix + dx, s, edge


def ncc_search(img, tpl, cy, cx, ry, rx):
    """以 (cy,cx) 为中心 ±(ry,rx) 搜索 tpl：归一化互相关峰 + 抛物线亚像素。
    打分与 08-20 原脚本逐点式完全同式（Σt0=0 ⇒ 去均值交叉项自然并入），
    sliding_window_view 向量化提速。峰钉在搜索窗边=帧间位移超半径（真实
    弹跳会这样），自动 ×2/×4 扩窗重搜而不是静默低报；返回
    (y, x, score, 是否扩过窗)，越界返回 score=-1。"""
    for mul in (1, 2, 4):
        y, x, s, edge = _ncc_once(img, tpl, cy, cx, ry * mul, rx * mul)
        if not edge:
            return y, x, s, mul > 1
    return y, x, s, True


def parse_rect(s):
    ys, xs = s.split(",")
    y0, y1 = (int(v) for v in ys.split(":"))
    x0, x1 = (int(v) for v in xs.split(":"))
    return y0, y1, x0, x1


def probe_video(video):
    """ffprobe 探源视频真实参数（时间轴/半径缩放/缓存指纹都用它——30fps、
    544×960 是基线口径不是普适事实：手机默认 60fps，硬编码直接把时间轴
    打对折）。返回 dict(duration, fps, w, h)。"""
    r = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries",
         "stream=width,height,avg_frame_rate:format=duration",
         "-of", "json", video], capture_output=True, text=True, check=True)
    j = json.loads(r.stdout)
    st = j["streams"][0]
    num, _, den = st["avg_frame_rate"].partition("/")
    fps = float(num) / float(den or 1)
    if not 1.0 <= fps <= 240.0:
        raise SystemExit(f"ffprobe 帧率异常 {st['avg_frame_rate']}（VFR 花活？"
                         "先 ffmpeg 转恒定帧率再喂）")
    return dict(duration=float(j["format"]["duration"]), fps=fps,
                w=int(st["width"]), h=int(st["height"]),
                path=os.path.abspath(video))


def _meta(out):
    """OUT/cache_meta.json：各段缓存的参数指纹。没有/坏了当空表。"""
    try:
        with open(os.path.join(out, "cache_meta.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


def _meta_save(out, meta):
    with open(os.path.join(out, "cache_meta.json"), "w") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)


def extract_frames(video, out, fps, vinfo):
    """抽帧到 OUT/seq。缓存带指纹：换视频/换 fps/上次抽帧被 Ctrl-C 截断
    （帧数≠时长×fps）都作废重抽——截断的 seq 静默沿用会把全程尾段丢掉、
    改 fps 重跑会给旧帧盖错时间戳（t=i/fps 按当前 fps 算）。"""
    seq = os.path.join(out, "seq")
    os.makedirs(seq, exist_ok=True)
    want = dict(video=os.path.abspath(video), mtime=os.path.getmtime(video),
                fps=fps, n_expect=int(vinfo["duration"] * fps))
    meta = _meta(out)
    paths = sorted(glob.glob(os.path.join(seq, "f*.jpg")))
    stale = None
    if paths:
        if meta.get("seq") != want:
            stale = "视频/fps 与上次不一致（或缓存无指纹）"
        elif abs(len(paths) - want["n_expect"]) > 2:
            stale = f"帧数 {len(paths)} ≠ 预期 {want['n_expect']}（上次抽帧被中断？）"
    if stale:
        print(f"⚠ 帧缓存作废重抽：{stale}")
        for p in paths:
            os.remove(p)
        paths = []
    if not paths:
        subprocess.run(["ffmpeg", "-loglevel", "error", "-i", video,
                        "-vf", f"fps={fps}", "-q:v", "4",
                        os.path.join(seq, "f%05d.jpg")], check=True)
        paths = sorted(glob.glob(os.path.join(seq, "f*.jpg")))
        if abs(len(paths) - want["n_expect"]) > 2:
            raise SystemExit(f"抽帧数 {len(paths)} 对不上时长×fps="
                             f"{want['n_expect']}（视频截断/ffmpeg 出错？）")
        meta["seq"] = want
        _meta_save(out, meta)
    return paths


def track(paths, body_rect, ref_rect, out, fps, vinfo):
    """全程 NCC 追踪 body/ref 两模板 → OUT/traj.csv（指纹一致才用缓存：
    改过 --body/--ref/fps 的重跑必须重算，旧 traj 按错模板打的分静默沿用
    会让操作者误判'视频不可用'）。搜索半径按 fps/分辨率相对基线缩放，
    ref 是静物只随分辨率缩。"""
    csv = os.path.join(out, "traj.csv")
    ry_b = max(6, round(BODY_RY * (BASE_FPS / fps) * (vinfo["h"] / BASE_H)))
    rx_b = max(4, round(BODY_RX * (BASE_FPS / fps) * (vinfo["w"] / BASE_W)))
    ry_r = max(4, round(REF_RY * (vinfo["h"] / BASE_H)))
    rx_r = max(4, round(REF_RX * (vinfo["w"] / BASE_W)))
    want = dict(video=os.path.abspath(vinfo["path"]), fps=fps,
                body=list(body_rect), ref=list(ref_rect),
                radii=[ry_b, rx_b, ry_r, rx_r], n=len(paths))
    meta = _meta(out)
    if os.path.exists(csv) and meta.get("traj") != want:
        print("⚠ 轨迹缓存作废重算：--body/--ref/fps/视频与上次不一致"
              "（或缓存无指纹）")
        os.remove(csv)
    if not os.path.exists(csv):
        f0 = gray(paths[0])
        by0, by1, bx0, bx1 = body_rect
        ry0, ry1, rx0, rx1 = ref_rect
        body_t = f0[by0:by1, bx0:bx1].copy()
        ref_t = f0[ry0:ry1, rx0:rx1].copy()
        by, bx, ry, rx = float(by0), float(bx0), float(ry0), float(rx0)
        rows = []
        n_exp = 0
        for i, p in enumerate(paths):
            img = gray(p)
            by, bx, bs, e1 = ncc_search(img, body_t, by, bx, ry_b, rx_b)
            ry, rx, rs, e2 = ncc_search(img, ref_t, ry, rx, ry_r, rx_r)
            n_exp += e1 or e2
            rows.append((i, i / fps, bx, by, bs, rx, ry, rs))
        if n_exp:
            print(f"⚠ {n_exp} 帧 NCC 峰贴搜索窗边，已扩窗重搜（帧间位移逼近"
                  f"/超过半径 {ry_b}px——集中在弹跳帧属预期，散布全程说明"
                  "fps 太低或模板追丢）")
        with open(csv, "w") as f:
            f.write("frame,t_video,body_x,body_y,body_score,"
                    "ref_x,ref_y,ref_score\n")
            for r in rows:
                f.write(f"{r[0]},{r[1]:.3f},{r[2]:.2f},{r[3]:.2f},{r[4]:.3f},"
                        f"{r[5]:.2f},{r[6]:.2f},{r[7]:.3f}\n")
        meta["traj"] = want
        _meta_save(out, meta)
    return np.genfromtxt(csv, delimiter=",", names=True)


def parse_log(path):
    """黑匣子日志 → 事件表 + TLM 电流序列 + 交接配置（时间都是日志相对秒）。
    body_lean 与 climb_walk 两种词汇都认：climb_walk 连续行走不进 HOVER、
    落地不打"落地："（单步打"单步落地（后半步）："），缺的事件由 build_lifts
    用相位转移行 lift→transfer / transfer→descend 推导。交接配置从参数行
    handover=… 提取（逐腿 δ 表；关/老日志=空表），供 build_lifts 校验
    "该有交接事件却没有"的抬落。"""
    ev = {k: [] for k in ("vent", "rel", "att", "hover", "ho", "land",
                          "lean", "lt", "td")}
    tlm = []
    ho_cfg = {}
    pat = re.compile(r"^\[\s*([0-9.]+)\] (EVT|TLM) (.*)$")
    rxs = (("vent", re.compile(r"吸附 (\w+) attached→venting")),
           ("rel", re.compile(r"吸附 (\w+) venting→released")),
           ("att", re.compile(r"吸附 (\w+) sucking→attached")),
           ("hover", re.compile(r"相位 (\w+) transfer→hover")),
           ("ho", re.compile(r"相位 (\w+) stance→handover")),
           ("lt", re.compile(r"相位 (\w+) lift→transfer")),
           ("td", re.compile(r"相位 (\w+) transfer→descend")),
           # 对步第二只腿（--dual，DUAL-SWING-DESIGN §4-6）经落地错峰队列走
           # hover→descend，且"落地：L1+R2"的 \w+ 只捕获首腿——没有本行，
           # 每对的第二只腿会因"缺落地"被整条丢弃出标定表。单足日志本转移
           # 与"落地："文本同刻共存，pick 先命中谁都同语义
           ("td", re.compile(r"相位 (\w+) hover→descend")),
           ("land", re.compile(r"落地：(\w+)")),
           ("land", re.compile(r"落地（后半步）：(\w+)")))
    rx_lean = re.compile(r"倾身 ([+-][0-9.]+)mm")
    rx_cur = re.compile(r"电=([0-9.]+)V/([0-9.]+)A")
    rx_ho = re.compile(r"(?<![\w_])handover=(\S+)")
    for line in open(path, encoding="utf-8"):
        m2 = rx_ho.search(line)
        if m2 and not ho_cfg:            # 参数行（# 头/note），关=留空表
            for tok in m2.group(1).split(","):
                name, _, val = tok.partition(":")
                try:
                    ho_cfg[name] = float(val)
                except ValueError:
                    pass
        m = pat.match(line.strip())
        if not m:
            continue
        t, kind, rest = float(m.group(1)), m.group(2), m.group(3)
        if kind == "TLM":
            m2 = rx_cur.search(rest)
            if m2:
                tlm.append((t, float(m2.group(1)), float(m2.group(2))))
            continue
        for key, rx in rxs:
            m2 = rx.search(rest)
            if m2:
                ev[key].append((t, m2.group(1)))
        m2 = rx_lean.search(rest)
        if m2:
            ev["lean"].append((t, float(m2.group(1))))
    return ev, np.array(tlm), ho_cfg


def build_lifts(ev, ho_cfg=None):
    """按放气事件串出每次抬落 {leg, ho, vent, rel, hover, land, att}。
    配对边界=该腿下一次放气事件，不设固定秒数窗——旧版 60s 窗被一次 >60s
    的冻结拆散配对后，该抬落静默消失、后面全部错轮。悬停/落地缺失时退化
    用相位行（climb_walk 连续行走词汇：lift→transfer≈悬停点、
    transfer→descend＝落地下探开始，与 body_lean '落地：' 同语义）。
    退出序列/取机的放气没有摆动，汇总一行滤掉；缺事件的真抬落逐条报告。
    交接回看同样到该腿上一次落地为界（旧 6s 窗：δ≥11 或漏气暂停/冻结把
    Z 段拖长就找不到，卸载坡道被错记进 vent 跳变列反向污染 δ 修正）；
    配置说该腿 δ>0 却没有交接事件 ⇒ 大声告警 + 标 ho_missing（该次
    vent 跳变不得用于 δ 修正）。"""
    ho_cfg = ho_cfg or {}
    lifts = []
    silent = 0
    vents = ev["vent"]
    for k, (tv, leg) in enumerate(vents):
        nxt = next((t for t, l in vents[k + 1:] if l == leg), float("inf"))

        def pick(key, lo, hi):
            return next((t for t, l in ev[key] if l == leg and lo < t < hi),
                        None)

        hov = pick("hover", tv, nxt) or pick("lt", tv, nxt)
        if hov is None:
            silent += 1
            continue
        rel = pick("rel", tv, hov)
        land = pick("land", hov, nxt) or pick("td", hov, nxt)
        att = pick("att", land or hov, nxt)
        if None in (rel, land, att):
            miss = "/".join(k_ for k_, v in (("released", rel),
                                             ("落地/descend", land),
                                             ("attached", att)) if v is None)
            print(f"⚠ 丢弃抬落：{leg} vent@{tv:.1f}s 缺 {miss} 事件"
                  "（日志截断/中断退出？）——轮次按逐腿序数推，其余不受牵连")
            continue
        prev = max((t for t, l in ev["att"] if l == leg and t < tv),
                   default=0.0)
        ho = max((t for t, l in ev["ho"] if l == leg and prev < t < tv),
                 default=None)
        missing = ho is None and ho_cfg.get(leg, 0.0) > 0.0
        if missing:
            print(f"⚠ {leg} vent@{tv:.1f}s：参数行说 handover δ="
                  f"{ho_cfg[leg]:g} 但没找到 stance→handover 事件——该次 "
                  "vent 跳变窗会咬进未记录的卸载坡道，已从 δ 修正口径剔除")
        lifts.append(dict(leg=leg, ho=ho, vent=tv, rel=rel, hover=hov,
                          land=land, att=att, ho_missing=missing))
    if silent:
        print(f"（另 {silent} 次放气无对应摆动——退出序列/取机，未计入）")
    return lifts


def find_offset(lifts, leans, tv, y, sync):
    """对时 off（video_t + off = log_t）。三种途径：
    - auto：日志里有倾身对时标记（协议要求首轮前 ↑×2/↓×2）→ 已知授予时刻
      与铺设速率（LEAN_SPEED=10mm/s）构造位移模板，逐 off 最小二乘拟合幅值
      选 R² 最大——B 组抬落身体几乎不动，活动对比度法会失锁；"窗口跨度
      最大化"也不行（任何包住整个鼓包的 off 跨度相同，合成数据实测退化
      8s），必须锁形状边沿；
    - 显式 --sync T0,T1：给定日志窗口内做同款拟合（无倾身则退化为跨度）；
    - 无标记回退 08-20 原法：粗扫抬落活动能量 + 活动/静置对比度细化。"""
    def yat(t_log, off):
        return np.interp(t_log - off, tv, y)

    def lean_fit(t0, t1):
        """R² 最大的 off：y ≈ ȳ + a·模板；方差门槛滤掉平窗（平窗残差
        天然为零，纯拼残差会赢）。返回 (off, R²) 或 None。"""
        def tmpl(tl):
            v = np.zeros_like(tl)
            for tg, mm in leans:
                dur = max(abs(mm) / 10.0, 1e-6)
                v += np.sign(mm) * np.clip((tl - tg) / dur, 0.0, 1.0)
            return v

        def r2(off):
            m = (tv >= t0 - off) & (tv <= t1 - off)
            if m.sum() < 8:
                return None
            yy = y[m] - y[m].mean()
            tot = (yy * yy).sum()
            if tot / m.sum() < 1.2 ** 2:          # 窗内位移 std ≤1.2px：没动
                return None
            ff = tmpl(tv[m] + off)
            ff = ff - ff.mean()
            den = (ff * ff).sum()
            if den < 1e-9:
                return None
            a = (yy * ff).sum() / den
            return a * a * den / tot

        best = (None, -1.0)
        # off 搜索界从数据跨度导出：窗口 [t0−off, t1−off] 整段落进视频
        # ⇔ off ∈ [t1−tv[-1], t0−tv[0]]。负 off（先开录像后起脚本——协议
        # 规定的正是这个顺序）天然在界内；旧版 max(0.0,…) 表示不了负值，
        # 静默给出错 off 再经 --calib/--mmppx 把两组所有毫米数全部带偏
        lo, hi = t1 - tv[-1], t0 - tv[0]
        if hi < lo:
            return None
        for off in np.arange(math.ceil(lo * 10.0) / 10.0, hi + 0.1, 0.1):
            r = r2(off)
            if r is not None and r > best[1]:
                best = (off, r)
        if best[0] is not None:
            for off in np.arange(best[0] - 0.5, best[0] + 0.5, 0.02):
                r = r2(off)
                if r is not None and r > best[1]:
                    best = (off, r)
        return best if best[0] is not None else None

    if sync not in ("auto", "contrast"):
        t0, t1 = (float(v) for v in sync.split(","))
        got = lean_fit(t0, t1) if leans else None
        if got is None:
            raise SystemExit(f"--sync 窗口 {t0}~{t1} 拟合失败（窗口放不进"
                             "视频/窗内没动静/日志无倾身标记）")
        print(f"对时（--sync 窗口拟合）：off={got[0]:.2f}s R²={got[1]:.2f}")
        return got[0]
    if sync == "auto" and leans:
        t0 = leans[0][0] - 3.0
        t1 = leans[-1][0] + abs(leans[-1][1]) / 10.0 + 6.0
        got = lean_fit(t0, t1)
        if got is None:
            raise SystemExit(f"倾身标记窗口 {t0:.1f}~{t1:.1f} 拟合失败"
                             "（视频没拍到标记段？改用 --sync contrast 试）")
        off, r2v = got
        print(f"对时（倾身标记形状拟合 {t0:.1f}~{t1:.1f}）："
              f"off={off:.2f}s R²={r2v:.2f}"
              + ("" if r2v > 0.7 else " ⚠ 拟合弱，人工核对 off"))
        return off

    # 08-20 原法：粗扫（覆盖的抬落活动能量最大）+ 对比度细化。
    # 扫描界同样从数据跨度导出（至少一次抬落能整段落进视频），负 off 可达；
    # 起点取 0.1 格点对齐——与旧版网格重合处逐点同值，基线回归不动
    best = (None, -1.0)
    lo_c = math.ceil((lifts[0]["att"] + 3.0 - tv[-1]) * 10.0) / 10.0
    hi_c = lifts[-1]["vent"] - 2.0 - tv[0]
    for off in np.arange(lo_c, hi_c, 0.1):
        cov = [L for L in lifts
               if L["vent"] - 2.0 - off >= tv[0] and L["att"] + 3.0 - off <= tv[-1]]
        if len(cov) < max(3, 2 * len(lifts) // 3):
            continue
        e = sum(abs(yat(L["att"], off) - yat(L["vent"] - 1.0, off))
                for L in cov) / len(cov)
        if e > best[1]:
            best = (off, e)
    off = best[0]
    if off is None:
        raise SystemExit("活动对比度法找不到对时（视频没盖住足够多的抬落？）")

    def contrast(off):
        act = qui = 1e-9
        aq = qn = 0.0
        for i, L in enumerate(lifts):
            if L["vent"] - off < 1.0 or L["att"] - off > tv[-1] - 3.0:
                continue
            aq += abs(yat(L["att"], off) - yat(L["vent"], off))
            act += L["att"] - L["vent"]
            nv = lifts[i + 1]["vent"] if i + 1 < len(lifts) else L["att"] + 15
            qn += abs(yat(nv - 0.5, off) - yat(L["att"] + 2.0, off))
            qui += nv - 2.5 - L["att"]
        return (aq / act) / (qn / qui + 1e-9)

    offs = np.arange(off - 3.0, off + 3.0, 0.05)
    cs = [contrast(o) for o in offs]
    off = float(offs[int(np.argmax(cs))])
    print(f"对时（活动对比度法）：off={off:.2f}s 对比度 {max(cs):.1f}")
    return off


def main():
    ap = argparse.ArgumentParser(
        description="零力交接 A/B 量化（协议 docs/HANDOVER-AB-PROTOCOL.md）")
    ap.add_argument("--video", required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--out", required=True, help="缓存/输出目录（帧序列很大，"
                    "放 home 下如 ~/ab_cache/A；勿放 /tmp——配额一满整机 "
                    "Bash 静默失败。参数指纹自动校验，换参不必换目录）")
    ap.add_argument("--fps", type=float, default=3.0, help="全程追踪抽帧率")
    ap.add_argument("--frames-only", action="store_true",
                    help="只抽帧（去 OUT/seq 里量模板矩形坐标）")
    ap.add_argument("--body", help="机身模板矩形 Y0:Y1,X0:X1（第 0 帧坐标）")
    ap.add_argument("--ref", help="静物参照模板矩形 Y0:Y1,X0:X1")
    ap.add_argument("--mmppx", type=float, default=None,
                    help="比例 mm/px（B 组沿用 A 组标定值；相机不许动过）")
    ap.add_argument("--calib", default=None,
                    help="A 组标比例：T0,T1,MM——日志 T0→T1 身体位移人工读数"
                         " MM mm（08-20 口径：胶带尺读数差）")
    ap.add_argument("--sync", default="auto",
                    help="对时：auto=优先倾身标记（无则活动对比度）；"
                         "contrast=强制原法；或显式日志窗口 T0,T1")
    ap.add_argument("--zoom", type=int, default=None,
                    help="对第 N 次抬落（分解表序号）抽 30fps 剖面")
    args = ap.parse_args()

    if os.path.realpath(args.out).startswith("/tmp/"):
        ap.error("--out 在 /tmp 下：帧缓存数百 MB 起，/tmp 配额一满整机 Bash "
                 "静默失败（项目已知事故）——放 home 下（如 ~/ab_cache/A）")
    vinfo = probe_video(args.video)
    if abs(vinfo["fps"] - 30.0) > 1.0:
        print(f"⚠ 源视频 {vinfo['fps']:.1f}fps（非基线 30）：时间轴/剖面按实测"
              "帧率算，但快门/果冻效应口径与基线未必可比")
    paths = extract_frames(args.video, args.out, args.fps, vinfo)
    print(f"帧序列 {len(paths)} 帧 @{args.fps:g}fps（{args.out}/seq；源 "
          f"{vinfo['w']}×{vinfo['h']}@{vinfo['fps']:g}fps {vinfo['duration']:.1f}s）")
    if args.frames_only:
        print("只抽帧模式：在 f00001.jpg 上量 --body/--ref 矩形后重跑")
        return
    if not (args.body and args.ref):
        ap.error("需要 --body 与 --ref 模板矩形（先 --frames-only 量坐标）")
    if (args.mmppx is None) == (args.calib is None):
        ap.error("--mmppx（B 组沿用）与 --calib（A 组标定）二选一")

    d = track(paths, parse_rect(args.body), parse_rect(args.ref),
              args.out, args.fps, vinfo)
    tv = d["t_video"]
    y = d["body_y"] - (d["ref_y"] - d["ref_y"][0])   # 相机漂移修正
    x = d["body_x"] - (d["ref_x"] - d["ref_x"][0])
    print(f"追踪健康：body_score min/中位 {d['body_score'].min():.3f}/"
          f"{np.median(d['body_score']):.3f}  ref_score min "
          f"{d['ref_score'].min():.3f}  相机抖动(ref_y 范围) "
          f"{d['ref_y'].max() - d['ref_y'].min():.2f}px")

    ev, tlm, ho_cfg = parse_log(args.log)
    lifts = build_lifts(ev, ho_cfg)
    print(f"日志抬落事件 {len(lifts)} 次"
          + ("（含 HANDOVER 相位=B 组）" if any(L["ho"] for L in lifts)
             else "（无 HANDOVER=A 组/基线）"))
    if not lifts:
        raise SystemExit("日志里没有完整抬落事件")

    off = find_offset(lifts, ev["lean"], tv, y, args.sync)

    def yat(t_log):
        return float(np.interp(t_log - off, tv, y))

    if args.calib:
        t0, t1, mm = (float(v) for v in args.calib.split(","))
        for tc in (t0, t1):
            # 钳位 interp 会拿视频边界值当真：一端出覆盖 ⇒ 像素分母偏小 ⇒
            # mm/px 静默偏大，再经 --mmppx 把 A/B 两组全部毒化——硬报错
            if not tv[0] + 0.5 <= tc - off <= tv[-1] - 0.5:
                raise SystemExit(
                    f"--calib 时刻 {tc:g}s 在视频覆盖外（视频对应日志 "
                    f"{tv[0] + off:.1f}~{tv[-1] + off:.1f}s，off={off:.2f}）"
                    "——换视频里拍到的安静时刻重标")
        mmppx = mm / abs(yat(t1) - yat(t0))
        print(f"比例标定：|y({t1:.1f})−y({t0:.1f})| = "
              f"{abs(yat(t1) - yat(t0)):.1f}px = {mm:g}mm → {mmppx:.3f} mm/px"
              f"（B 组沿用请传 --mmppx {mmppx:.3f}，期间相机不许动）")
    else:
        mmppx = args.mmppx

    # ---- 逐事件分解（窗口口径与 08-20 报告一致；+=沿墙向下）----
    has_ho = any(L["ho"] for L in lifts)
    head = "  # 腿    | " + ("交接段 | " if has_ho else "") \
        + "vent跳变 | 摆动段 | 落地压入 | 静置 | 全周期 (mm, +=向下)"
    print("\n" + head)
    tot = np.zeros(5)
    per_round = {}               # 轮 -> [位移和, 计入的腿集合]
    bounce = {}
    seen = {}                    # 轮次=该腿第几次抬落：幸存下标 i//6 在任何
    all_legs = set()             # 一次丢弃/跳过后会把后面全部错轮（旧版实锤）
    pre_ts = [(L["ho"] - 0.5) if L["ho"] else (L["vent"] - 1.0) for L in lifts]
    for i, L in enumerate(lifts):
        seen[L["leg"]] = seen.get(L["leg"], 0) + 1
        rnd = L["rnd"] = seen[L["leg"]]
        all_legs.add(L["leg"])
        per_round.setdefault(rnd, [0.0, set()])
        pre_t = pre_ts[i]
        # 静置窗收在下一次抬落的 pre 点（B 组若用 vent−1.0 会咬进下一腿
        # 交接段的头 0.7s，把交接期位移错记进本腿静置列）
        nxt_t = pre_ts[i + 1] if i + 1 < len(lifts) else L["att"] + 12.0
        if pre_t - off < tv[0] + 0.5 or nxt_t - off > tv[-1] - 0.5:
            print(f" {i:2d} {L['leg']}#{rnd} | ⚠ 视频没盖住本次抬落，"
                  "跳过（插值会拿边界值当真）")
            continue
        y_pre = yat(pre_t)
        y_v0 = yat(L["vent"]) if L["ho"] else y_pre
        seg = np.array([y_v0 - y_pre,                 # 交接段（A 组恒 0）
                        yat(L["rel"] + 0.4) - y_v0,   # vent 跳变（密封破裂）
                        yat(L["land"] - 0.3) - yat(L["rel"] + 0.4),
                        yat(L["att"] + 1.5) - yat(L["land"] - 0.3),
                        yat(nxt_t) - yat(L["att"] + 1.5)]) * mmppx
    # 说明：交接段窗含 VENT 前的排队帧（3fps 分辨率 0.33s），细看用 --zoom
        tot += seg
        per_round[rnd][0] += seg.sum()
        per_round[rnd][1].add(L["leg"])
        if not L["ho_missing"]:      # 无交接事件的抬落不进 δ 反馈（见 build_lifts）
            bounce.setdefault(L["leg"], []).append(seg[1])
        cells = ([f"{seg[0]:+6.1f} | "] if has_ho else []) \
            + [f"{seg[1]:+7.1f} | {seg[2]:+6.1f} | {seg[3]:+7.1f} | "
               f"{seg[4]:+5.1f} | {seg.sum():+6.1f}"] \
            + (["  ⚠ 无交接事件，勿用于 δ 修正"] if L["ho_missing"] else [])
        print(f" {i:2d} {L['leg']}#{rnd} | " + "".join(cells))
    print(f"合计：交接 {tot[0]:+.1f}  vent {tot[1]:+.1f}  摆动 {tot[2]:+.1f}  "
          f"落地 {tot[3]:+.1f}  静置 {tot[4]:+.1f}  总 {tot.sum():+.1f}mm")
    for rnd, (s, legs) in sorted(per_round.items()):
        miss = sorted(all_legs - legs)
        print(f"第 {rnd} 轮下滑：{s:+.1f}mm"
              + (f"（缺 {'/'.join(miss)}——丢弃/没盖住，勿与整轮判据线直接比）"
                 if miss else ""))
    print("\n逐腿 vent 跳变（δ 反馈；末轮值为准，+=向下坠）:")
    for leg in ("L1", "R1", "L3", "R3", "R2", "L2"):
        if leg in bounce:
            vals = " ".join(f"{v:+.1f}" for v in bounce[leg])
            print(f"  {leg}: {vals}")
    print(f"横向漂移全程：{(x[-1] - x[0]) * mmppx:+.1f}mm（+=画面右）")

    # ---- 电流棘轮（0.5s TLM）----
    if len(tlm):
        t_c, v_c, a_c = tlm[:, 0], tlm[:, 1], tlm[:, 2]
        quiet = a_c[(t_c > lifts[0]["vent"] - 12) & (t_c < lifts[0]["vent"] - 2)]
        print(f"\n电流：首抬前静置均值 {quiet.mean():.2f}A"
              f"（电压起点 {v_c[0]:.2f}V）" if len(quiet) else "\n电流：")
        for i, L in enumerate(lifts):
            nv = lifts[i + 1]["vent"] if i + 1 < len(lifts) else L["att"] + 10
            m = (t_c > L["vent"] - 1) & (t_c < nv - 1)
            if m.any():
                print(f"  {i:2d} {L['leg']}#{L['rnd']}: "
                      f"均值 {a_c[m].mean():.2f}A 峰 {a_c[m].max():.2f}A")
        m_last = (t_c > lifts[-1]["att"] + 2) & (t_c < lifts[-1]["att"] + 12)
        if m_last.any():
            print(f"  末轮后静置均值 {a_c[m_last].mean():.2f}A"
                  "（不回落=内应力仍在累积）")

    # ---- 原生帧率破裂剖面（可选）----
    if args.zoom is not None:
        if not 0 <= args.zoom < len(lifts):
            raise SystemExit(f"--zoom {args.zoom} 越界：本日志共 {len(lifts)} 次"
                             "抬落（序号见分解表首列）")
        zoom(args.zoom, lifts[args.zoom], args.video, args.out, off, mmppx,
             tv, d, parse_rect(args.body), vinfo)


def zoom(idx, L, video, out, off, mmppx, tv, d, body_rect, vinfo):
    """原生帧率细看第 idx 次抬落：身体 y(t) 剖面 + 释放脚回弹（08-20 原法）。
    时间轴用 ffprobe 实测帧率——30 硬编码对 60fps 手机录像把剖面对折；
    片段起点钳到视频 0（负 -ss ffmpeg 退出码 0 写零帧或钳起点，旧版在此
    IndexError/整列标记错位）；模板与释放脚掩膜按 --body 矩形与分辨率
    缩放（92×140、bx+70 等是基线 544×960 下该矩形的派生量，别的机位下
    按老常数切的是任意图块）。"""
    fps_v = vinfo["fps"]
    by0, by1, bx0, bx1 = body_rect
    th_b, tw_b = by1 - by0, bx1 - bx0
    sy, sx = vinfo["h"] / BASE_H, vinfo["w"] / BASE_W
    s_av = (sy + sx) / 2.0
    t0 = (L["ho"] or L["vent"]) - 1.2
    t1 = L["vent"] + 2.6
    if t0 - off >= vinfo["duration"] - 0.5 or t1 - off <= 0.5:
        print(f"⚠ --zoom {idx}：抬落窗 {t0:.1f}~{t1:.1f}s 不在视频覆盖内"
              f"（视频对应日志 {off:.1f}~{vinfo['duration'] + off:.1f}s），跳过")
        return
    if t0 - off < 0.0:
        print(f"⚠ --zoom {idx}：片段头 {off - t0:.1f}s 在开录像之前，"
              "从视频 0 点起标")
        t0 = off
    dir_ = os.path.join(out, f"clip_{idx}_{L['leg']}")
    os.makedirs(dir_, exist_ok=True)
    key = f"clip_{idx}_{L['leg']}"
    want = dict(video=vinfo["path"], off=round(off, 3),
                t0=round(t0, 3), t1=round(t1, 3))
    meta = _meta(out)
    paths = sorted(glob.glob(f"{dir_}/c*.jpg"))
    if paths and meta.get(key) != want:
        print("⚠ 剖面片段缓存作废重抽：off/窗口/视频与上次不一致")
        for p in paths:
            os.remove(p)
        paths = []
    if not paths:
        subprocess.run(["ffmpeg", "-loglevel", "error",
                        "-ss", f"{t0 - off:.3f}", "-i", video,
                        "-t", f"{t1 - t0:.3f}", "-q:v", "4",
                        "-y", f"{dir_}/c%04d.jpg"], check=True)
        paths = sorted(glob.glob(f"{dir_}/c*.jpg"))
        if not paths:
            raise SystemExit(f"--zoom 抽帧得 0 帧（-ss {t0 - off:.3f} 出视频"
                             "范围？ffmpeg 对负/越界 -ss 退出码照样是 0）")
        meta[key] = want
        _meta_save(out, meta)
    f0 = gray(paths[0])
    if f0.shape != (vinfo["h"], vinfo["w"]):
        raise SystemExit(f"剖面帧 {f0.shape[1]}×{f0.shape[0]} ≠ 源视频 "
                         f"{vinfo['w']}×{vinfo['h']}（缓存串目录？删 {dir_} 重跑）")
    by = float(np.interp(t0 - off, tv, d["body_y"]))
    bx = float(np.interp(t0 - off, tv, d["body_x"]))
    if int(by) + th_b > f0.shape[0] or int(bx) + tw_b > f0.shape[1]:
        raise SystemExit("身体模板在剖面首帧越界（--body 矩形与追踪位置对不上）")
    tpl = f0[int(by):int(by) + th_b, int(bx):int(bx) + tw_b].copy()
    ty, tx = by, bx
    ry_z = max(6, round(14 * sy * 30.0 / fps_v))
    rx_z = max(4, round(8 * sx * 30.0 / fps_v))
    traj = []
    for p in paths:
        img = gray(p)
        ty, tx, s, _ = ncc_search(img, tpl, ty, tx, ry_z, rx_z)
        traj.append((ty, tx, s))
    traj = np.array(traj)
    yrel = (traj[:, 0] - traj[0, 0]) * mmppx
    print(f"\n=== #{idx} {L['leg']} {fps_v:g}fps 剖面（相对首帧 mm，t=日志时刻）===")
    marks = [(L["vent"], "<-- 阀开(request_release)"),
             (L["rel"], "<-- 盘压归零")]
    if L["ho"]:
        marks.append((L["ho"], "<-- 交接开始"))
    step = max(1, round(fps_v / 15.0))         # ~15 行/s（30fps=隔 2 帧，同旧版）
    rows = list(range(0, len(yrel), step))
    tags = {}                                  # 标记吸附到最近打印行（阈值法在
    for tm, m in marks:                        # 非整帧率下会把同一标记打两行）
        i_r = min(rows, key=lambda i: abs(t0 + i / fps_v - tm))
        if abs(t0 + i_r / fps_v - tm) < step * 1.02 / fps_v:
            tags[i_r] = tags.get(i_r, "") + m
    for i in rows:
        print(f"  {t0 + i / fps_v:7.2f}  {yrel[i]:+6.2f} {tags.get(i, '')}")
    # 释放脚：首尾帧差找机身外运动簇，NCC 追踪其回弹（掩膜=08-20 参数按
    # --body 矩形/分辨率换算：150px 排除圈、120/180px 上下带均随画幅缩放）
    fa, fb = gray(paths[0]), gray(paths[-1])
    df = np.abs(fb - fa)
    h, w = df.shape
    yy, xx = np.mgrid[0:h, 0:w]
    keep = (np.hypot(xx - (bx + tw_b / 2.0), yy - (by - 15 * sy)) > 150 * s_av) \
        & (yy > by - 120 * sy) & (yy < h - 180 * sy)
    df = df * keep
    d4 = df[:h // 4 * 4, :w // 4 * 4].reshape(h // 4, 4, w // 4, 4).mean((1, 3))
    iy, ix = np.unravel_index(np.argmax(d4), d4.shape)
    fy, fx = iy * 4, ix * 4
    half = round(35 * s_av)
    ft = f0[max(0, fy - half):fy + half, max(0, fx - half):fx + half].copy()
    fyy, fxx = float(max(0, fy - half)), float(max(0, fx - half))
    r_f = max(6, round(12 * s_av * 30.0 / fps_v))
    ff = []
    for p in paths:
        img = gray(p)
        fyy, fxx, s, _ = ncc_search(img, ft, fyy, fxx, r_f, r_f)
        ff.append((fyy, fxx, s))
    ff = np.array(ff)
    dyf = (ff[:, 0] - ff[0, 0]) * mmppx
    print(f"  释放脚簇 @({fx},{fy})：终点 dy={dyf[-1]:+.1f}mm "
          f"最大|dy|={np.max(np.abs(dyf)):.1f}mm score终 {ff[-1, 2]:.2f}")
    stp = max(1, round(fps_v / 10.0))
    print("  脚块 dy 剖面:", " ".join(f"{v:+.1f}" for v in dyf[::stp]))


if __name__ == "__main__":
    main()
