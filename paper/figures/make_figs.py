#!/usr/bin/env python3
"""论文插图生成(paper/main.tex 的 Fig.2-5)。在仓库根目录跑:
    .venv/bin/python paper/figures/make_figs.py

数据来源(与 paper/README.md 溯源表一致):
  Fig 2  阶梯轨迹+破裂剖面 = 重复#1 A 组(~/ab_cache/0825/A 缓存 + 日志
         lean_20260825_103942.log,off=125.84/mmppx=0.2951 与 report.txt 同);
         剖面 = ab_quant --zoom 7 的帧缓存(clip_7_L1)全帧率重追踪。
  Fig 3  弹跳-δ 线性 = html/handover-delta-calib-20260824.html 第 2 轮表。
  Fig 4  分解瀑布/逐腿弹跳 = html/handover-ab-n3-20260826.html §1/§3。
  Fig 5  D′ 对齐剖面 = html/dprime-20260831.html 内嵌 PNG 原样提取。

配色:六腿分类色经 dataviz validate_palette 全项通过(直接标注为次级编码);
事件线/辅助一律灰阶,文字用文字色不用系列色。
"""
import base64
import glob
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "software/logs_analysis")
import ab_quant  # noqa: E402

OUT = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.expanduser("~/ab_cache/0825/A")
VIDEO = "images/lean_20260825_103942.MOV"
LOG = "software/logs_analysis/lean_20260825_103942.log"
OFF, MMPPX = 125.84, 0.2951           # report.txt 同源(标记法 R²=0.98)

PAL = {"L1": "#2457b0", "R1": "#c05621", "L3": "#0f8a5f",
       "R3": "#8a4fc8", "R2": "#a06c10", "L2": "#c2366e"}
INK, MUT, LIGHT = "#1a1a1a", "#555555", "#c4c4c4"

plt.rcParams.update({
    "font.size": 8, "axes.labelsize": 8, "xtick.labelsize": 7,
    "ytick.labelsize": 7, "axes.linewidth": 0.6,
    "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "axes.spines.top": False, "axes.spines.right": False,
    "pdf.fonttype": 42,
})


def lifts_and_traj():
    ev, tlm, ho_cfg = ab_quant.parse_log(LOG)
    lifts = ab_quant.build_lifts(ev, ho_cfg)
    tr = np.genfromtxt(os.path.join(CACHE, "traj.csv"),
                       names=True, delimiter=",")
    y = (tr["body_y"] - tr["body_y"][0]) - (tr["ref_y"] - tr["ref_y"][0])
    return lifts, tr["t_video"] + OFF, y * MMPPX, tr


def fig_staircase_zoom():
    lifts, t, y, tr = lifts_and_traj()
    rels = [L["rel"] for L in lifts]
    t0, t1 = min(L["vent"] for L in lifts) - 12, max(L["att"] for L in lifts) + 25
    m = (t >= t0) & (t <= t1)
    yb = np.interp(t0, t, y)                 # 起点归零(截窗前的标记段不计)

    fig, (ax, az) = plt.subplots(
        2, 1, figsize=(3.45, 3.6), height_ratios=[1, 1],
        constrained_layout=True)

    # (a) 全程阶梯
    for r in rels:
        ax.axvline(r, color=LIGHT, lw=0.5, zorder=1)
    ax.plot(t[m], y[m] - yb, color=PAL["L1"], lw=1.1, zorder=3)
    ax.invert_yaxis()
    ax.set_xlabel("time (s)")
    ax.set_ylabel("descent (mm)")
    for i, lab in enumerate(["round 1", "round 2", "round 3"]):
        seg = rels[6 * i:6 * i + 6]
        ax.text((seg[0] + seg[-1]) / 2, -6, lab, ha="center",
                va="bottom", fontsize=7, color=MUT)
    ax.text(t1 - 2, np.interp(t1, t, y) - yb - 8, "185 mm", ha="right",
            va="bottom", fontsize=7, color=INK)
    ax.text(0.02, 0.04, "(a)", transform=ax.transAxes, fontsize=8,
            color=INK, weight="bold")

    # (b) 30fps 剖面(clip_7_L1 全帧率重追踪,复刻 ab_quant.zoom 内核)
    L = lifts[7]
    vinfo = ab_quant.probe_video(VIDEO)
    fps = vinfo["fps"]
    tz0 = (L["ho"] or L["vent"]) - 1.2
    paths = sorted(glob.glob(os.path.join(CACHE, "clip_7_L1", "c*.jpg")))
    if not paths:
        raise SystemExit("clip_7_L1 帧缓存缺失——先跑 ab_quant --zoom 7")
    f0 = ab_quant.gray(paths[0])
    by0_, by1_, bx0_, bx1_ = 2253, 2384, 751, 1018
    th, tw = by1_ - by0_, bx1_ - bx0_
    tv = tr["t_video"]
    by = float(np.interp(tz0 - OFF, tv, tr["body_y"]))
    bx = float(np.interp(tz0 - OFF, tv, tr["body_x"]))
    tpl = f0[int(by):int(by) + th, int(bx):int(bx) + tw].copy()
    sy, sx = vinfo["h"] / ab_quant.BASE_H, vinfo["w"] / ab_quant.BASE_W
    ry = max(6, round(14 * sy * 30.0 / fps))
    rx = max(4, round(8 * sx * 30.0 / fps))
    ty, tx = by, bx
    ys = []
    for p in paths:
        img = ab_quant.gray(p)
        ty, tx, s, _ = ab_quant.ncc_search(img, tpl, ty, tx, ry, rx)
        ys.append(ty)
    ys = (np.array(ys) - ys[0]) * MMPPX
    tt = tz0 + np.arange(len(ys)) / fps
    i_rel = int(np.argmax(ys > 5.0))          # 机械释放=首个 >5mm 帧
    t_snap = tt[i_rel]

    az.axvline(L["vent"] - t_snap, color=LIGHT, lw=0.7)
    az.axvline(L["rel"] - t_snap, color=LIGHT, lw=0.7, ls=(0, (3, 2)))
    az.plot(tt - t_snap, ys, color=PAL["L1"], lw=1.0,
            marker="o", ms=2.0, mew=0, zorder=3)
    az.invert_yaxis()
    az.set_xlabel("time from seal release (s)")
    az.set_ylabel("descent (mm)")
    az.set_xlim(-1.25, 2.3)
    az.text(L["vent"] - t_snap - 0.06, 6.5, "valve\ncommand", ha="right",
            va="top", fontsize=6.5, color=MUT)
    az.text(L["rel"] - t_snap + 0.06, 6.5, "cup pressure\nreads zero",
            ha="left", va="top", fontsize=6.5, color=MUT)
    az.annotate("falls in $\\leq$2 frames ($\\leq$67 ms)",
                xy=(0.06, 14), xytext=(0.62, 14.5), fontsize=6.5, color=INK,
                va="center", arrowprops=dict(arrowstyle="-", color=MUT, lw=0.6))
    az.text(1.5, 19.6, "settles 21.5 mm", fontsize=6.5, color=MUT,
            va="bottom")
    az.text(0.02, 0.04, "(b)", transform=az.transAxes, fontsize=8,
            color=INK, weight="bold")

    fig.savefig(os.path.join(OUT, "fig_staircase_zoom.pdf"))
    plt.close(fig)
    print(f"fig_staircase_zoom: rupture at valve+{t_snap-L['vent']:.2f}s, "
          f"pressure-zero at valve+{L['rel']-L['vent']:.2f}s, "
          f"peak {ys.max():.1f}, settle {ys[-8:].mean():.1f} mm")


def fig_bounce_delta():
    # html/handover-delta-calib-20260824.html 第 2 轮表(δ=0/12/24, mm)
    data = {"L1": (22.5, 14.4, 8.9), "R1": (16.3, 11.3, 6.9),
            "L3": (7.7, 5.0, 2.7), "R3": (7.2, 3.4, 1.2),
            "R2": (3.5, 2.0, 0.5), "L2": (4.1, 3.0, 0.7)}
    dx = np.array([0.0, 12.0, 24.0])
    lab_dy = {"L3": +0.65, "R3": -0.65, "L2": +0.55, "R2": -0.55}
    fig, ax = plt.subplots(figsize=(3.45, 2.35), constrained_layout=True)
    ax.axhline(0, color=LIGHT, lw=0.6, zorder=1)
    stars = []
    for leg, b in data.items():
        c = PAL[leg]
        k, b0 = np.polyfit(dx, b, 1)
        dstar = -b0 / k
        stars.append(dstar)
        ax.plot(dx, b, "o", color=c, ms=3.2, mew=0, zorder=4)
        ax.plot([0, 24], [b0, b0 + 24 * k], color=c, lw=1.1, zorder=3)
        ax.plot([24, dstar], [b0 + 24 * k, 0], color=c, lw=0.8,
                ls=(0, (3, 2)), zorder=2)
        ax.plot([dstar], [0], marker="v", color=c, ms=4, mew=0, zorder=4)
        ax.text(-1.2, b0 + lab_dy.get(leg, 0.0), leg, ha="right",
                va="center", fontsize=7, color=c)
    ax.text(np.mean(stars), -2.0, "$\\delta^{*}=28$–$42$ mm", ha="center",
            va="top", fontsize=7, color=MUT)
    ax.set_xlim(-4.5, 44)
    ax.set_ylim(-3.6, 24)
    ax.set_xlabel("pre-laid handover displacement $\\delta$ (mm)")
    ax.set_ylabel("rupture bounce (mm)")
    fig.savefig(os.path.join(OUT, "fig_bounce_delta.pdf"))
    plt.close(fig)
    print("fig_bounce_delta done")


def fig_waterfall():
    # n=3 主表(html/handover-ab-n3-20260826.html §1/§3)
    conds = ["A", "B", "C"]
    vent = [172.9, 31.6, 20.1]
    ho = [0.0, 60.2, 39.7]
    tot = [185.5, 105.6, 72.9]
    other = [t - v - h for t, v, h in zip(tot, vent, ho)]
    seg_c = {"rupture": "#b3452c", "handover": "#2457b0", "other": "#9a9a9a"}
    legs = ["L1", "R1", "L3", "R3", "R2", "L2"]
    bA = [20.1, 16.2, 7.5, 6.3, 6.7, 3.5]
    bC = [0.9, 0.9, 1.8, 1.9, 2.4, 0.7]

    fig, (ax, ar) = plt.subplots(
        1, 2, figsize=(3.45, 2.1), width_ratios=[1, 1.35],
        constrained_layout=True)

    x = np.arange(3)
    bot = np.zeros(3)
    for name, vals in (("rupture", vent), ("handover", ho), ("other", other)):
        ax.bar(x, vals, 0.62, bottom=bot, color=seg_c[name],
               edgecolor="white", linewidth=1.0, label=name)
        bot += np.array(vals)
    for i, (t, pct) in enumerate(zip(tot, ["", "$-43\\%$", "$-61\\%$"])):
        ax.text(i, t + 4, f"{t:.1f}\n{pct}".strip(), ha="center",
                va="bottom", fontsize=6.5, color=INK)
    ax.set_xticks(x, conds)
    ax.set_ylim(0, 232)
    ax.set_ylabel("3-round slip (mm)")
    ax.legend(frameon=False, fontsize=6, loc="upper right",
              handlelength=1.0, borderaxespad=0.1)
    ax.text(0.03, 0.94, "(a)", transform=ax.transAxes, fontsize=8,
            color=INK, weight="bold")

    xw = np.arange(6)
    ar.bar(xw - 0.19, bA, 0.34, color="#9a9a9a", edgecolor="white",
           linewidth=0.6, label="A")
    ar.bar(xw + 0.19, bC, 0.34, color="#2457b0", edgecolor="white",
           linewidth=0.6, label="C")
    ar.set_xticks(xw, legs)
    ar.set_ylim(0, 23)
    ar.set_yticks([0, 5, 10, 15, 20])
    ar.set_ylabel("round-2 bounce (mm)")
    ar.legend(frameon=False, fontsize=6, loc="upper right",
              handlelength=1.0, borderaxespad=0.1)
    ar.text(0.03, 0.94, "(b)", transform=ar.transAxes, fontsize=8,
            color=INK, weight="bold")

    fig.savefig(os.path.join(OUT, "fig_waterfall.pdf"))
    plt.close(fig)
    print("fig_waterfall done")


def fig_dprime():
    html = open("html/dprime-20260831.html", encoding="utf-8").read()
    m = re.search(r'src="data:image/png;base64,([A-Za-z0-9+/=\s]+)"', html)
    if not m:
        raise SystemExit("dprime html 里没找到内嵌 PNG")
    png = base64.b64decode(m.group(1))
    with open(os.path.join(OUT, "dprime_profile.png"), "wb") as f:
        f.write(png)
    print(f"dprime_profile.png extracted ({len(png)/1024:.0f} KB)")


if __name__ == "__main__":
    fig_dprime()
    fig_bounce_delta()
    fig_waterfall()
    fig_staircase_zoom()
