#!/usr/bin/env python3
"""阶跃基底回归精修对时(08-24 救援法):y(t) ≈ 线性漂移 + Σ 18 个 vent
时刻的阶跃,扫 off 使最小二乘 R² 最大。用法:step_off.py 组目录 日志戳 lo hi"""
import sys

import numpy as np

sys.path.insert(0, "/home/shaopeng/spider/software/logs_analysis")
from ab_quant import parse_log, build_lifts

outdir, stamp, lo, hi = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
point = sys.argv[5] if len(sys.argv) > 5 else "rel"   # 阶跃点:rel|vent
d = np.genfromtxt(f"/home/shaopeng/ab_cache/{outdir}/traj.csv",
                  delimiter=",", names=True)
tv = d["t_video"]
y = d["body_y"] - (d["ref_y"] - d["ref_y"][0])
ev, _, cfg = parse_log(
    f"/home/shaopeng/spider/software/logs_analysis/lean_{stamp}.log")
lifts = build_lifts(ev, cfg)
vents = [L[point] + 0.2 for L in lifts]

best = (None, -1.0)
for off in np.arange(lo, hi, 0.05):
    cols = [np.ones_like(tv), tv]
    for t_s in vents:
        cols.append((tv >= t_s - off).astype(float))
    X = np.stack(cols, 1)
    beta, res, *_ = np.linalg.lstsq(X, y, rcond=None)
    r = y - X @ beta
    r2 = 1.0 - (r * r).sum() / ((y - y.mean()) ** 2).sum()
    if r2 > best[1]:
        best = (off, r2)
off, r2 = best
print(f"阶跃基底回归:off={off:.2f}s R²={r2:.4f}(扫描 {lo}~{hi})")
# 阶跃系数概览(应全为正=向下坠,量级对得上 vent 弹跳)
cols = [np.ones_like(tv), tv] + [(tv >= t_s - off).astype(float) for t_s in vents]
X = np.stack(cols, 1)
beta = np.linalg.lstsq(X, y, rcond=None)[0]
print("18 阶跃系数(px):", " ".join(f"{b:+.0f}" for b in beta[2:]))
