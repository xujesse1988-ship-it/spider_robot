#!/usr/bin/env python3
"""裁剪窗标记对时:录像开晚、倾身标记只拍到尾巴时的 off 救援。

背景(08-25 C2 实案,n=3 报告偏差①):ab_quant 标记法 lean_fit 要求标记
窗口**整段**落在视频内(off ≤ t0−tv[0]),录像开晚 2.3s 就把真值排除出
搜索界,自动对时找了假峰;随后的阶跃基底回归(step_off)在 vent 梳齿轮内
间距高度周期化(17.5~18.5s)时错位一格 R² 仅差 <0.001,B2 亚秒校准辨不了
齿——off 锁到 +1 齿,逐腿归因整体平移一槽(伪"R2/L2 大坠"),总量守恒
所以不易察觉。教训:①阶跃救援前先查梳齿周期性(打印 vent 间距);
②"标记没拍到"须区分整段缺失与边缘裁剪——后者用本工具。

本工具 = ab_quant.find_offset.lean_fit 的裁剪窗变体,口径唯一差异:
掩膜允许窗口与视频取交集,但要求 ①覆盖 ≥60% 窗口样本;②每个 lean 的
棱边(授予时刻−0.7s ~ 授予+铺设时长+0.7s)完整在画内——锁形状靠棱边,
棱边不缺就能定 off。拟合幅值顺带给出该组传递比,可与其余标记互证
(C2: off=59.34 R²=0.99,幅值 4.23mm=42.3%,与 8 标记 42.2±3.3% 咬合)。

用法:
  python marker_off_clip.py <lean_日志> <traj.csv> [lo hi]
  lo/hi = off 粗扫范围(缺省从数据跨度导出,全范围扫描并列出各局部峰)。
定准后把 off 传给 ab_quant --off 重跑分解。
"""
import sys

import numpy as np

sys.path.insert(0, "/home/shaopeng/spider/software/logs_analysis")
import ab_quant as q


def clip_lean_fit(leans, tv, y, lo=None, hi=None):
    """返回 [(R², off, a_px)] 的局部峰列表(降序)。a_px=每 10mm 档的
    像素幅值(×mm/px÷10mm=传递比)。"""
    t0 = leans[0][0] - 3.0
    t1 = leans[-1][0] + abs(float(leans[-1][1])) / 10.0 + 6.0
    span = t1 - t0

    def tmpl(tl):
        v = np.zeros_like(tl)
        for tg, mm in leans:
            mm = float(mm)
            dur = max(abs(mm) / 10.0, 1e-6)
            v += np.sign(mm) * np.clip((tl - tg) / dur, 0.0, 1.0)
        return v

    fps_est = (len(tv) - 1) / max(tv[-1] - tv[0], 1e-6)

    def fit(off):
        m = (tv >= t0 - off) & (tv <= t1 - off)
        if m.sum() < max(8, 0.6 * span * fps_est):
            return None
        for tg, mm in leans:                      # 棱边必须完整在画内
            e0 = tg - 0.7 - off
            e1 = tg + abs(float(mm)) / 10.0 + 0.7 - off
            if e0 < tv[0] or e1 > tv[-1]:
                return None
        yy = y[m] - y[m].mean()
        tot = (yy * yy).sum()
        if tot / m.sum() < 1.2 ** 2:              # 平窗滤除(同 ab_quant)
            return None
        ff = tmpl(tv[m] + off)
        ff = ff - ff.mean()
        den = (ff * ff).sum()
        if den < 1e-9:
            return None
        a = (yy * ff).sum() / den
        return a * a * den / tot, a

    if lo is None:
        lo = t1 - tv[-1] - span                   # 裁剪窗:界外各放宽一个窗
    if hi is None:
        hi = t0 - tv[0] + span
    grid = np.arange(np.floor(lo), hi, 0.1)
    vals = {}
    for off in grid:
        r = fit(off)
        if r is not None:
            vals[round(float(off), 4)] = r[0]
    peaks = []
    for off, r in vals.items():
        near = [v for o, v in vals.items() if abs(o - off) <= 1.5]
        if r >= max(near) - 1e-12:
            best_r, best_o = r, off
            for o2 in np.arange(off - 0.3, off + 0.3, 0.02):
                rr = fit(o2)
                if rr is not None and rr[0] > best_r:
                    best_r, best_o = rr[0], o2
            if not any(abs(best_o - p[1]) < 0.5 for p in peaks):
                a = fit(best_o)[1]
                peaks.append((best_r, best_o, a))
    peaks.sort(reverse=True)
    return peaks


def main():
    log, traj = sys.argv[1], sys.argv[2]
    lo = float(sys.argv[3]) if len(sys.argv) > 3 else None
    hi = float(sys.argv[4]) if len(sys.argv) > 4 else None
    ev, _, ho_cfg = q.parse_log(log)
    lifts = q.build_lifts(ev, ho_cfg)
    rels = [L["rel"] for L in lifts]
    print("倾身标记(t, mm):", ev["lean"])
    print("vent 梳齿间距 s(周期化=阶跃法有错齿风险):",
          [round(b - a, 2) for a, b in zip(rels, rels[1:])])
    d = np.genfromtxt(traj, delimiter=",", names=True)
    tv = d["t_video"]
    y = d["body_y"] - (d["ref_y"] - d["ref_y"][0])
    peaks = clip_lean_fit(ev["lean"], tv, y, lo, hi)
    if not peaks:
        raise SystemExit("裁剪窗拟合无峰:标记棱边不在画内(整段缺失)——"
                         "只能退回阶跃法,并用本打印的梳齿间距评估错齿风险")
    print("裁剪窗标记拟合峰(降序):")
    for r, off, a in peaks[:5]:
        print(f"  off={off:7.2f}  R²={r:.3f}  幅值 {a:+.1f}px/10mm 档")
    r, off, a = peaks[0]
    print(f"→ 采用 off={off:.2f}(传给 ab_quant --off);"
          f"幅值×mm/px÷10 即该组传递比,应与其余标记互证")


if __name__ == "__main__":
    main()
