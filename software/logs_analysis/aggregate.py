#!/usr/bin/env python3
"""九组(3 重复 × A/B/C)汇总:解析各 report.txt + 标记幅值,出 n=3 统计。"""
import re
import sys

import numpy as np

sys.path.insert(0, "/home/shaopeng/spider/software/logs_analysis")
from ab_quant import parse_log

# (重复, 条件, out 目录, 日志戳, mm/px, 组内顺位)
RUNS = [
    (1, "A", "0825/A",   "20260825_103942", 0.2951, 1),
    (1, "B", "0825/B",   "20260825_105000", 0.2957, 2),
    (1, "C", "0825/C",   "20260825_105827", 0.2954, 3),
    (2, "B", "0825pm/B", "20260825_154921", 0.2917, 1),
    (2, "C", "0825pm/C", "20260825_155727", 0.2925, 2),
    (2, "A", "0825pm/A", "20260825_160551", 0.2949, 3),
    (3, "C", "0826/C",   "20260826_101200", 0.2883, 1),
    (3, "A", "0826/A",   "20260826_102037", 0.2906, 2),
    (3, "B", "0826/B",   "20260826_102805", 0.2912, 3),
]
LEGS = ("L1", "R1", "L3", "R3", "R2", "L2")


def parse_report(path):
    txt = open(path, encoding="utf-8").read()
    d = {}
    d["rounds"] = [float(m) for m in re.findall(
        r"第 \d 轮下滑：([+-][0-9.]+)mm", txt)]
    m = re.search(r"合计：交接 ([+-][0-9.]+)  vent ([+-][0-9.]+)  摆动 "
                  r"([+-][0-9.]+)  落地 ([+-][0-9.]+)  静置 ([+-][0-9.]+)  "
                  r"总 ([+-][0-9.]+)mm", txt)
    d["seg"] = [float(v) for v in m.groups()]
    d["vent2"] = {}
    for leg in LEGS:
        m = re.search(rf"^  {leg}: ([+-][0-9.]+) ([+-][0-9.]+) ([+-][0-9.]+)",
                      txt, re.M)
        d["vent2"][leg] = float(m.group(2)) if m else float("nan")
    m = re.search(r"首抬前静置均值 ([0-9.]+)A（电压起点 ([0-9.]+)V）", txt)
    d["i0"], d["v0"] = float(m.group(1)), float(m.group(2))
    m = re.search(r"末轮后静置均值 ([0-9.]+)A", txt)
    d["i1"] = float(m.group(1))
    m = re.search(r"off=([0-9.+-]+)s(?: R²=([0-9.]+))?", txt)
    d["off"] = float(m.group(1))
    d["r2"] = float(m.group(2)) if m.group(2) else float("nan")
    m = re.search(r"body_score min/中位 ([0-9.]+)/([0-9.]+)  ref_score min "
                  r"([0-9.]+)  相机抖动\(ref_y 范围\) ([0-9.]+)px", txt)
    d["bs_med"], d["ref_jit"] = float(m.group(2)), float(m.group(4))
    return d


def marker_amp(outdir, logstamp, off):
    t = np.genfromtxt(f"/home/shaopeng/ab_cache/{outdir}/traj.csv",
                      delimiter=",", names=True)
    tv = t["t_video"]
    y = t["body_y"] - (t["ref_y"] - t["ref_y"][0])
    ev, _, _ = parse_log("/home/shaopeng/spider/software/logs_analysis/"
                         f"lean_{logstamp}.log")
    leans = ev["lean"]
    t0 = leans[0][0] - 3.0
    t1 = leans[-1][0] + abs(leans[-1][1]) / 10.0 + 6.0
    m = (tv >= t0 - off) & (tv <= t1 - off)
    v = np.zeros(m.sum())
    tl = tv[m] + off
    for tg, mm in leans:
        v += np.sign(mm) * np.clip((tl - tg) / max(abs(mm) / 10.0, 1e-6), 0, 1)
    yy = y[m] - y[m].mean()
    ff = v - v.mean()
    return abs((yy * ff).sum() / (ff * ff).sum())


rows = []
print("重复 顺位 条件 | mm/px | off R² | body中位 | 第2轮 | 总 | 交接 | vent |"
      " 电流Δ | 标记实位移mm(指令10)")
for rep, cond, outdir, stamp, mmppx, pos in RUNS:
    p = f"/home/shaopeng/ab_cache/{outdir}/report.txt"
    try:
        d = parse_report(p)
    except Exception as e:
        print(f"  #{rep} {cond}: 缺/坏 report.txt({e})——先跑 ab_quant")
        continue
    amp_px = marker_amp(outdir, stamp, d["off"])
    scale = mmppx or float("nan")
    amp_mm = amp_px * scale
    rows.append(dict(rep=rep, cond=cond, pos=pos, r2=d["r2"], sag2=d["rounds"][1],
                     tot=d["seg"][5], ho=d["seg"][0], vent=d["seg"][1],
                     di=d["i1"] - d["i0"], amp=amp_mm, v0=d["v0"],
                     vent2=d["vent2"], bs=d["bs_med"]))
    print(f"  #{rep} 位{pos} {cond} | {scale:.4f} | {d['off']:+7.2f} {d['r2']:.2f}"
          f" | {d['bs_med']:.2f} | {d['rounds'][1]:6.1f} | {d['seg'][5]:6.1f}"
          f" | {d['seg'][0]:6.1f} | {d['seg'][1]:6.1f} | +{d['i1']-d['i0']:.2f}A"
          f" | {amp_mm:.2f}")

print("\n=== n=3 统计(均值 ± 样本标准差,mm)===")
print("条件 | 第2轮下滑 | 三轮总 | 交接段 | vent段 | 电流Δ | 标记实位移")
for cond in ("A", "B", "C"):
    g = [r for r in rows if r["cond"] == cond]
    if len(g) < 2:
        print(f"  {cond}: 只有 {len(g)} 组,不出统计")
        continue

    def ms(key):
        v = np.array([r[key] for r in g])
        return f"{v.mean():6.1f} ± {v.std(ddof=1):4.1f}"

    v = np.array([r["amp"] for r in g])
    print(f"  {cond}({len(g)}) | {ms('sag2')} | {ms('tot')} | {ms('ho')} |"
          f" {ms('vent')} | {ms('di')}A | {v.mean():.2f} ± {v.std(ddof=1):.2f}")

print("\n=== 相对效应(逐重复配对,比值用同重复内算)===")
for rep in (1, 2, 3):
    g = {r["cond"]: r for r in rows if r["rep"] == rep}
    if len(g) == 3:
        a, b, c = g["A"]["tot"], g["B"]["tot"], g["C"]["tot"]
        print(f"  #{rep}: 总下滑 A {a:.0f} → B {b:.0f}({(b/a-1)*100:+.0f}%)"
              f" → C {c:.0f}({(c/a-1)*100:+.0f}%;对 B {(c/b-1)*100:+.0f}%)")

print("\n=== 第 2 轮逐腿 vent(mm,按条件 n=3 均值±std)===")
hdr = "条件 | " + " | ".join(LEGS)
print(hdr)
for cond in ("A", "B", "C"):
    g = [r for r in rows if r["cond"] == cond]
    if len(g) < 2:
        continue
    cells = []
    for leg in LEGS:
        v = np.array([r["vent2"][leg] for r in g])
        cells.append(f"{v.mean():+5.1f}±{v.std(ddof=1):.1f}")
    print(f"  {cond} | " + " | ".join(cells))
