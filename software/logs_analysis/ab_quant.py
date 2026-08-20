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

依赖：ffmpeg、numpy、Pillow（开发机跑，不上树莓派）。帧序列/轨迹按 --out
目录缓存，重跑只算增量；换模板/比例请换 --out 或删缓存。

用法：
  # 先只抽帧，在 OUT/seq/f00001.jpg 上量模板矩形（Y0:Y1,X0:X1）
  python ab_quant.py --video A.mp4 --log A.log --out /tmp/ab/A --frames-only
  # A 组（基线对照）：比例按玻璃上胶带尺人工读数标定（T0/T1 为日志时刻）
  python ab_quant.py --video A.mp4 --log A.log --out /tmp/ab/A \
      --body 280:372,340:480 --ref 640:730,250:380 --calib 142.3,327.5,131
  # B 组（--handover）：沿用 A 组比例
  python ab_quant.py --video B.mp4 --log B.log --out /tmp/ab/B \
      --body ... --ref ... --mmppx 0.881
  # 第 N 次抬落（序号见分解表）的 30fps 破裂瞬间剖面 + 释放脚回弹
  python ab_quant.py ... --zoom N

模板选择：body=机身绿 PCB 纹理块（含安装孔等高对比特征）；ref=画面里保证
不动的静物块（窗台花盆等，用于相机漂移修正）。两块都取第 0 帧坐标。
"""
import argparse
import glob
import os
import re
import subprocess

import numpy as np
from PIL import Image

BODY_RY, BODY_RX = 25, 12   # 逐帧搜索半径 px（身体主要沿画面 y 走）
REF_RY, REF_RX = 8, 8


def gray(path):
    return np.asarray(Image.open(path).convert("L"), dtype=np.float64)


def ncc_search(img, tpl, cy, cx, ry, rx):
    """以 (cy,cx) 为中心 ±(ry,rx) 搜索 tpl：归一化互相关峰 + 抛物线亚像素。
    打分与 08-20 原脚本逐点式完全同式（Σt0=0 ⇒ 去均值交叉项自然并入），
    sliding_window_view 向量化提速。返回 (y, x, score)，越界返回 score=-1。"""
    th, tw = tpl.shape
    y0 = max(0, int(cy) - ry)
    y1 = min(img.shape[0] - th, int(cy) + ry)
    x0 = max(0, int(cx) - rx)
    x1 = min(img.shape[1] - tw, int(cx) + rx)
    if y1 < y0 or x1 < x0:
        return cy, cx, -1.0
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

    def para(sm1, s0, sp1):
        den = sm1 - 2 * s0 + sp1
        return 0.0 if abs(den) < 1e-12 else 0.5 * (sm1 - sp1) / den

    dy = para(sc[iy - 1, ix], s, sc[iy + 1, ix]) \
        if 0 < iy < sc.shape[0] - 1 else 0.0
    dx = para(sc[iy, ix - 1], s, sc[iy, ix + 1]) \
        if 0 < ix < sc.shape[1] - 1 else 0.0
    return y0 + iy + dy, x0 + ix + dx, s


def parse_rect(s):
    ys, xs = s.split(",")
    y0, y1 = (int(v) for v in ys.split(":"))
    x0, x1 = (int(v) for v in xs.split(":"))
    return y0, y1, x0, x1


def extract_frames(video, out, fps):
    seq = os.path.join(out, "seq")
    os.makedirs(seq, exist_ok=True)
    paths = sorted(glob.glob(os.path.join(seq, "f*.jpg")))
    if not paths:
        subprocess.run(["ffmpeg", "-loglevel", "error", "-i", video,
                        "-vf", f"fps={fps}", "-q:v", "4",
                        os.path.join(seq, "f%05d.jpg")], check=True)
        paths = sorted(glob.glob(os.path.join(seq, "f*.jpg")))
    return paths


def track(paths, body_rect, ref_rect, out, fps):
    """全程 NCC 追踪 body/ref 两模板 → OUT/traj.csv（有缓存直接读）。"""
    csv = os.path.join(out, "traj.csv")
    if not os.path.exists(csv):
        f0 = gray(paths[0])
        by0, by1, bx0, bx1 = body_rect
        ry0, ry1, rx0, rx1 = ref_rect
        body_t = f0[by0:by1, bx0:bx1].copy()
        ref_t = f0[ry0:ry1, rx0:rx1].copy()
        by, bx, ry, rx = float(by0), float(bx0), float(ry0), float(rx0)
        rows = []
        for i, p in enumerate(paths):
            img = gray(p)
            by, bx, bs = ncc_search(img, body_t, by, bx, BODY_RY, BODY_RX)
            ry, rx, rs = ncc_search(img, ref_t, ry, rx, REF_RY, REF_RX)
            rows.append((i, i / fps, bx, by, bs, rx, ry, rs))
        with open(csv, "w") as f:
            f.write("frame,t_video,body_x,body_y,body_score,"
                    "ref_x,ref_y,ref_score\n")
            for r in rows:
                f.write(f"{r[0]},{r[1]:.3f},{r[2]:.2f},{r[3]:.2f},{r[4]:.3f},"
                        f"{r[5]:.2f},{r[6]:.2f},{r[7]:.3f}\n")
    return np.genfromtxt(csv, delimiter=",", names=True)


def parse_log(path):
    """黑匣子日志 → 事件表 + TLM 电流序列（时间都是日志相对秒）。"""
    ev = {k: [] for k in ("vent", "rel", "att", "hover", "ho", "land", "lean")}
    tlm = []
    pat = re.compile(r"^\[\s*([0-9.]+)\] (EVT|TLM) (.*)$")
    rxs = (("vent", re.compile(r"吸附 (\w+) attached→venting")),
           ("rel", re.compile(r"吸附 (\w+) venting→released")),
           ("att", re.compile(r"吸附 (\w+) sucking→attached")),
           ("hover", re.compile(r"相位 (\w+) transfer→hover")),
           ("ho", re.compile(r"相位 (\w+) stance→handover")),
           ("land", re.compile(r"落地：(\w+)")))
    rx_lean = re.compile(r"倾身 ([+-][0-9.]+)mm")
    rx_cur = re.compile(r"电=([0-9.]+)V/([0-9.]+)A")
    for line in open(path, encoding="utf-8"):
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
    return ev, np.array(tlm)


def build_lifts(ev):
    """按放气事件串出每次抬落 {leg, ho, vent, rel, hover, land, att}。
    退出序列的放气没有后续 hover，自然滤掉。"""
    lifts = []
    for tv, leg in ev["vent"]:
        hov = next((t for t, l in ev["hover"] if l == leg and tv < t < tv + 60),
                   None)
        if hov is None:
            continue
        rel = next((t for t, l in ev["rel"] if l == leg and tv < t < hov), None)
        land = next((t for t, l in ev["land"] if l == leg and t > hov), None)
        att = next((t for t, l in ev["att"]
                    if l == leg and land is not None and t > land), None)
        if None in (rel, land, att):
            continue
        ho = max((t for t, l in ev["ho"] if l == leg and tv - 6.0 < t < tv),
                 default=None)
        lifts.append(dict(leg=leg, ho=ho, vent=tv, rel=rel, hover=hov,
                          land=land, att=att))
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
        lo = max(0.0, t1 - tv[-1])
        for off in np.arange(lo, max(lo, t0) + 0.1, 0.1):
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

    # 08-20 原法：粗扫（覆盖的抬落活动能量最大）+ 对比度细化
    best = (None, -1.0)
    for off in np.arange(0.0, lifts[-1]["att"], 0.1):
        cov = [L for L in lifts
               if L["vent"] - 2.0 - off >= 0.0 and L["att"] + 3.0 - off <= tv[-1]]
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
                    "放 /tmp 或已 gitignore 的路径）")
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

    paths = extract_frames(args.video, args.out, args.fps)
    print(f"帧序列 {len(paths)} 帧 @{args.fps:g}fps（{args.out}/seq）")
    if args.frames_only:
        print("只抽帧模式：在 f00001.jpg 上量 --body/--ref 矩形后重跑")
        return
    if not (args.body and args.ref):
        ap.error("需要 --body 与 --ref 模板矩形（先 --frames-only 量坐标）")
    if (args.mmppx is None) == (args.calib is None):
        ap.error("--mmppx（B 组沿用）与 --calib（A 组标定）二选一")

    d = track(paths, parse_rect(args.body), parse_rect(args.ref),
              args.out, args.fps)
    tv = d["t_video"]
    y = d["body_y"] - (d["ref_y"] - d["ref_y"][0])   # 相机漂移修正
    x = d["body_x"] - (d["ref_x"] - d["ref_x"][0])
    print(f"追踪健康：body_score min/中位 {d['body_score'].min():.3f}/"
          f"{np.median(d['body_score']):.3f}  ref_score min "
          f"{d['ref_score'].min():.3f}  相机抖动(ref_y 范围) "
          f"{d['ref_y'].max() - d['ref_y'].min():.2f}px")

    ev, tlm = parse_log(args.log)
    lifts = build_lifts(ev)
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
    per_round = {}
    bounce = {}
    pre_ts = [(L["ho"] - 0.5) if L["ho"] else (L["vent"] - 1.0) for L in lifts]
    for i, L in enumerate(lifts):
        pre_t = pre_ts[i]
        # 静置窗收在下一次抬落的 pre 点（B 组若用 vent−1.0 会咬进下一腿
        # 交接段的头 0.7s，把交接期位移错记进本腿静置列）
        nxt_t = pre_ts[i + 1] if i + 1 < len(lifts) else L["att"] + 12.0
        if pre_t - off < tv[0] + 0.5 or nxt_t - off > tv[-1] - 0.5:
            print(f" {i:2d} {L['leg']}#{i // 6 + 1} | ⚠ 视频没盖住本次抬落，"
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
        tot += np.array([seg[0], seg[1], seg[2], seg[3], seg[4]])
        rnd = i // 6 + 1
        per_round.setdefault(rnd, 0.0)
        per_round[rnd] += seg.sum()
        bounce.setdefault(L["leg"], []).append(seg[1])
        cells = ([f"{seg[0]:+6.1f} | "] if has_ho else []) \
            + [f"{seg[1]:+7.1f} | {seg[2]:+6.1f} | {seg[3]:+7.1f} | "
               f"{seg[4]:+5.1f} | {seg.sum():+6.1f}"]
        print(f" {i:2d} {L['leg']}#{rnd} | " + "".join(cells))
    print(f"合计：交接 {tot[0]:+.1f}  vent {tot[1]:+.1f}  摆动 {tot[2]:+.1f}  "
          f"落地 {tot[3]:+.1f}  静置 {tot[4]:+.1f}  总 {tot.sum():+.1f}mm")
    for rnd, s in sorted(per_round.items()):
        print(f"第 {rnd} 轮下滑：{s:+.1f}mm")
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
                print(f"  {i:2d} {L['leg']}#{i // 6 + 1}: "
                      f"均值 {a_c[m].mean():.2f}A 峰 {a_c[m].max():.2f}A")
        m_last = (t_c > lifts[-1]["att"] + 2) & (t_c < lifts[-1]["att"] + 12)
        if m_last.any():
            print(f"  末轮后静置均值 {a_c[m_last].mean():.2f}A"
                  "（不回落=内应力仍在累积）")

    # ---- 30fps 破裂剖面（可选）----
    if args.zoom is not None:
        if not 0 <= args.zoom < len(lifts):
            raise SystemExit(f"--zoom {args.zoom} 越界：本日志共 {len(lifts)} 次"
                             "抬落（序号见分解表首列）")
        zoom(args.zoom, lifts[args.zoom], args.video, args.out, off, mmppx,
             tv, d)


def zoom(idx, L, video, out, off, mmppx, tv, d):
    """30fps 细看第 idx 次抬落：身体 y(t) 剖面 + 释放脚回弹（08-20 原法）。"""
    t0 = (L["ho"] or L["vent"]) - 1.2
    t1 = L["vent"] + 2.6
    dir_ = os.path.join(out, f"clip_{idx}_{L['leg']}")
    os.makedirs(dir_, exist_ok=True)
    if not glob.glob(f"{dir_}/c*.jpg"):
        subprocess.run(["ffmpeg", "-loglevel", "error",
                        "-ss", f"{t0 - off:.3f}", "-i", video,
                        "-t", f"{t1 - t0:.3f}", "-q:v", "4",
                        "-y", f"{dir_}/c%04d.jpg"], check=True)
    paths = sorted(glob.glob(f"{dir_}/c*.jpg"))
    f0 = gray(paths[0])
    by = float(np.interp(t0 - off, tv, d["body_y"]))
    bx = float(np.interp(t0 - off, tv, d["body_x"]))
    tpl = f0[int(by):int(by) + 92, int(bx):int(bx) + 140].copy()
    ty, tx = by, bx
    traj = []
    for p in paths:
        img = gray(p)
        ty, tx, s = ncc_search(img, tpl, ty, tx, 14, 8)
        traj.append((ty, tx, s))
    traj = np.array(traj)
    yrel = (traj[:, 0] - traj[0, 0]) * mmppx
    print(f"\n=== #{idx} {L['leg']} 30fps 剖面（相对首帧 mm，t=日志时刻）===")
    marks = [(L["vent"], "<-- 阀开(request_release)"),
             (L["rel"], "<-- 盘压归零")]
    if L["ho"]:
        marks.append((L["ho"], "<-- 交接开始"))
    for i in range(0, len(yrel), 2):
        t = t0 + i / 30.0
        tag = "".join(m for tm, m in marks if abs(t - tm) < 0.034)
        print(f"  {t:7.2f}  {yrel[i]:+6.2f} {tag}")
    # 释放脚：首尾帧差找机身外运动簇，NCC 追踪其回弹（掩膜参数沿用 08-20）
    fa, fb = gray(paths[0]), gray(paths[-1])
    df = np.abs(fb - fa)
    h, w = df.shape
    yy, xx = np.mgrid[0:h, 0:w]
    keep = (np.hypot(xx - (bx + 70), yy - (by - 15)) > 150) \
        & (yy > by - 120) & (yy < h - 180)
    df = df * keep
    d4 = df[:h // 4 * 4, :w // 4 * 4].reshape(h // 4, 4, w // 4, 4).mean((1, 3))
    iy, ix = np.unravel_index(np.argmax(d4), d4.shape)
    fy, fx = iy * 4, ix * 4
    ft = f0[max(0, fy - 35):fy + 35, max(0, fx - 35):fx + 35].copy()
    fyy, fxx = float(max(0, fy - 35)), float(max(0, fx - 35))
    ff = []
    for p in paths:
        img = gray(p)
        fyy, fxx, s = ncc_search(img, ft, fyy, fxx, 12, 12)
        ff.append((fyy, fxx, s))
    ff = np.array(ff)
    dyf = (ff[:, 0] - ff[0, 0]) * mmppx
    print(f"  释放脚簇 @({fx},{fy})：终点 dy={dyf[-1]:+.1f}mm "
          f"最大|dy|={np.max(np.abs(dyf)):.1f}mm score终 {ff[-1, 2]:.2f}")
    print("  脚块 dy 剖面:", " ".join(f"{v:+.1f}" for v in dyf[::3]))


if __name__ == "__main__":
    main()
