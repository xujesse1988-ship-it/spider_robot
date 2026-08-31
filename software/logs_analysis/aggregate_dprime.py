#!/usr/bin/env python3
"""D′ 判别实验(协议 §9)六组配对统计:C(铺设 10mm/s)vs D′(20mm/s),
08-31 同日三对顺序交替(D′C / CD′ / D′C)。读 ~/ab_cache/0831/{组}/report.txt,
输出:逐组表 → 分条件 n=3 统计 → 逐对配对差与 t 检验(df=2)→ 判读表
(时间蠕变 vs 逐 mm 损耗:交接段实测 vs M8 预测)→ 附带健全性检查
(逐腿 vent/传递比/电流增量应不变)。用法:.venv/bin/python aggregate_dprime.py"""
import re
import sys

import numpy as np

sys.path.insert(0, "/home/shaopeng/spider/software/logs_analysis")
from ab_quant import parse_log, build_lifts

LEGS = ("L1", "R1", "L3", "R3", "R2", "L2")
# (对, 条件, 顺位, 缓存目录, 日志戳, mm/px 板长边实测——跑完逐组填)
RUNS = [
    (1, "D", 1, "0831/D1", "20260831_164136", 0.29232),
    (1, "C", 2, "0831/C1", "20260831_170013", 0.29160),
    (2, "C", 1, "0831/C2", "20260831_171453", 0.28956),
    (2, "D", 2, "0831/D2", "20260831_172720", 0.28897),
    (3, "D", 1, "0831/D3", "20260831_174013", 0.29079),
    (3, "C", 2, "0831/C3", "20260831_175215", 0.29024),
]
# T1 联合拟合 M8 参数(html/stiffness-fit-20260826.html):共模=ρ_C·(1+β′(r−1))·T窗;
# C 组 08-25/26 实测交接段 39.7±4.3、再分配项 ~7.7(份额不变→D′ 同)
RHO_C, BETA_P, REDIS_C, C_REF = 0.434, 0.21, 7.7, 39.7
ROW = re.compile(r"^\s*\d+\s+(\w\d)#(\d) \|\s*([+-][0-9.]+) \|")


def parse_report(path):
    txt = open(path, encoding="utf-8").read()
    d = {"ho": {}}
    for line in txt.splitlines():
        m = ROW.match(line)
        if m:
            d["ho"].setdefault(m.group(1), {})[int(m.group(2))] = float(m.group(3))
    d["rounds"] = [float(m) for m in re.findall(r"第 \d 轮下滑：([+-][0-9.]+)mm", txt)]
    m = re.search(r"合计：交接 ([+-][0-9.]+)  vent ([+-][0-9.]+)  摆动 ([+-][0-9.]+)"
                  r"  落地 ([+-][0-9.]+)  静置 ([+-][0-9.]+)  总 ([+-][0-9.]+)mm", txt)
    d["seg"] = [float(v) for v in m.groups()]
    d["vent2"] = {}
    for leg in LEGS:
        m = re.search(rf"^  {leg}: ([+-][0-9.]+) ([+-][0-9.]+) ([+-][0-9.]+)", txt, re.M)
        d["vent2"][leg] = float(m.group(2)) if m else float("nan")
    m = re.search(r"首抬前静置均值 ([0-9.]+)A（电压起点 ([0-9.]+)V）", txt)
    d["i0"], d["v0"] = float(m.group(1)), float(m.group(2))
    d["i1"] = float(re.search(r"末轮后静置均值 ([0-9.]+)A", txt).group(1))
    m = re.search(r"off=([0-9.+-]+)s(?: R²=([0-9.]+))?", txt)
    d["off"] = float(m.group(1)); d["r2"] = float(m.group(2)) if m.group(2) else float("nan")
    m = re.search(r"body_score min/中位 ([0-9.]+)/([0-9.]+)  ref_score min ([0-9.]+)"
                  r"  相机抖动\(ref_y 范围\) ([0-9.]+)px", txt)
    d["bs_med"], d["ref_min"], d["ref_jit"] = float(m.group(2)), float(m.group(3)), float(m.group(4))
    return d


def marker_amp(outdir, stamp, off):
    t = np.genfromtxt(f"/home/shaopeng/ab_cache/{outdir}/traj.csv", delimiter=",", names=True)
    tv = t["t_video"]; y = t["body_y"] - (t["ref_y"] - t["ref_y"][0])
    ev, _, _ = parse_log(f"/home/shaopeng/spider/software/logs_analysis/lean_{stamp}.log")
    leans = ev["lean"]
    t0 = leans[0][0] - 3.0; t1 = leans[-1][0] + abs(leans[-1][1]) / 10.0 + 6.0
    m = (tv >= t0 - off) & (tv <= t1 - off)
    v = np.zeros(m.sum()); tl = tv[m] + off
    for tg, mm in leans:
        v += np.sign(mm) * np.clip((tl - tg) / max(abs(mm) / 10.0, 1e-6), 0, 1)
    yy = y[m] - y[m].mean(); ff = v - v.mean()
    return abs((yy * ff).sum() / (ff * ff).sum())


def twin(stamp):
    ev, _, ho = parse_log(f"/home/shaopeng/spider/software/logs_analysis/lean_{stamp}.log")
    T = {}
    for L in build_lifts(ev, ho):
        T.setdefault(L["leg"], []).append(L["vent"] - (L["ho"] - 0.5))
    return T


def paired_t(d):
    d = np.asarray(d, float); n = len(d)
    sd = d.std(ddof=1); t = d.mean() / (sd / np.sqrt(n)) if sd > 0 else float("inf")
    try:
        from scipy import stats
        p = 2 * stats.t.sf(abs(t), n - 1)
    except Exception:
        p = float("nan")
    return d.mean(), sd, t, p


def main():
    rows = []
    print("对 顺位 条件 | mm/px | off R² | body中位/ref最低 | 第2轮 | 总 | 交接 | vent | 电流Δ | 标记mm(指令10) | T窗均值s")
    for pair, cond, pos, outdir, stamp, mmppx in RUNS:
        try:
            d = parse_report(f"/home/shaopeng/ab_cache/{outdir}/report.txt")
        except Exception as e:
            print(f"  对{pair} {cond}: 缺/坏 report.txt({e})"); continue
        amp = marker_amp(outdir, stamp, d["off"]) * (mmppx or float("nan"))
        T = twin(stamp); tw = np.mean([t for v in T.values() for t in v])
        rows.append(dict(pair=pair, cond=cond, pos=pos, outdir=outdir, stamp=stamp, mmppx=mmppx,
                         r2=d["r2"], sag2=d["rounds"][1], tot=d["seg"][5], ho=d["seg"][0],
                         vent=d["seg"][1], di=d["i1"] - d["i0"], amp=amp, tw=tw,
                         holeg=d["ho"], vent2=d["vent2"], bs=d["bs_med"], rm=d["ref_min"]))
        print(f"  对{pair} 位{pos} {cond} | {mmppx or float('nan'):.4f} | {d['off']:+7.2f} {d['r2']:.2f} | "
              f"{d['bs_med']:.2f}/{d['ref_min']:.3f} | {d['rounds'][1]:6.1f} | {d['seg'][5]:6.1f} | "
              f"{d['seg'][0]:6.1f} | {d['seg'][1]:6.1f} | +{d['i1']-d['i0']:.2f}A | {amp:.2f} | {tw:.2f}")
    if len(rows) < 6:
        print("\n六组未齐,以下统计按已有组"); 
    print("\n=== 分条件统计(均值 ± 样本标准差)===")
    print("条件 | 第2轮 | 总 | 交接段 | vent段 | 电流Δ | 标记mm | T窗s")
    for c in ("C", "D"):
        R = [r for r in rows if r["cond"] == c]
        if not R: continue
        f = lambda k: f"{np.mean([r[k] for r in R]):6.1f} ± {np.std([r[k] for r in R], ddof=1) if len(R)>1 else 0:4.1f}"
        print(f"  {c}({len(R)}) | {f('sag2')} | {f('tot')} | {f('ho')} | {f('vent')} | "
              f"{np.mean([r['di'] for r in R]):.2f}±{np.std([r['di'] for r in R], ddof=1) if len(R)>1 else 0:.2f}A | "
              f"{np.mean([r['amp'] for r in R]):.2f}±{np.std([r['amp'] for r in R], ddof=1) if len(R)>1 else 0:.2f} | "
              f"{np.mean([r['tw'] for r in R]):.2f}")
    pairs = {}
    for r in rows:
        pairs.setdefault(r["pair"], {})[r["cond"]] = r
    full = [p for p in sorted(pairs) if len(pairs[p]) == 2]
    if full:
        print("\n=== 逐对配对差 D′−C(同日同对)===")
        for r in rows:
            r["hv"] = r["ho"] + r["vent"]          # 交接+vent 合并段:铺设加快只挪窗口归因时,此列才是判据
        for k, nm in (("ho", "交接段"), ("vent", "vent段"), ("hv", "交接+vent 合并"), ("tot", "三轮总"), ("sag2", "第2轮"), ("di", "电流Δ")):
            diffs = [pairs[p]["D"][k] - pairs[p]["C"][k] for p in full]
            mean, sd, t, pv = paired_t(diffs)
            unit = "A" if k == "di" else "mm"
            print(f"  {nm}: " + " / ".join(f"{x:+.1f}" for x in diffs) +
                  f"  → 均值 {mean:+.1f}±{sd:.1f}{unit}  t={t:.2f} p={pv:.3f}")
        # 判读:时间蠕变预测
        twC = np.mean([pairs[p]["C"]["tw"] for p in full]); twD = np.mean([pairs[p]["D"]["tw"] for p in full])
        hoC = np.mean([pairs[p]["C"]["ho"] for p in full]); hoD = np.mean([pairs[p]["D"]["ho"] for p in full])
        common_C = hoC - REDIS_C
        pred_time = REDIS_C + common_C * (twD / twC)
        print("\n=== 判读(协议 §9)===")
        print(f"  C 实测交接段 {hoC:.1f}(08-25/26 口径 {C_REF});共模≈{common_C:.1f}+再分配 {REDIS_C}")
        print(f"  时间蠕变假说预测 D′ = {REDIS_C} + {common_C:.1f}×({twD:.2f}/{twC:.2f}) = {pred_time:.1f}mm")
        print(f"  逐 mm 损耗假说预测 D′ ≈ C = {hoC:.1f}mm")
        print(f"  D′ 实测 {hoD:.1f}mm → 落点比例 (C−D′)/(C−预测时间蠕变) = "
              f"{(hoC - hoD) / max(hoC - pred_time, 1e-9):.2f}(1=纯时间蠕变,0=纯逐 mm 损耗)")
        hvC = np.mean([pairs[p]["C"]["hv"] for p in full]); hvD = np.mean([pairs[p]["D"]["hv"] for p in full])
        print(f"  ⚠ 窗口归因检验:交接+vent 合并段 C {hvC:.1f} vs D′ {hvD:.1f}mm——若交接段降而合并段不变,"
              "=下沉只是从交接窗挪进 vent 窗(弛豫过程跟随载荷转移事件、有自身时间常数),提速无净收益")
        print("\n=== 逐腿交接段(三轮均值,mm;两条件对照)===")
        print("腿 | " + " | ".join(f"C{p}/D{p}" for p in full))
        for leg in LEGS:
            cells = []
            for p in full:
                c = np.mean(list(pairs[p]["C"]["holeg"][leg].values()))
                dd = np.mean(list(pairs[p]["D"]["holeg"][leg].values()))
                cells.append(f"{c:+5.1f}/{dd:+5.1f}")
            print(f"{leg} | " + " | ".join(cells))
        print("\n=== 健全性检查(两条件应不变)===")
        print("第2轮逐腿 vent(C 均值 / D′ 均值):")
        for leg in LEGS:
            vc = np.mean([pairs[p]["C"]["vent2"][leg] for p in full])
            vd = np.mean([pairs[p]["D"]["vent2"][leg] for p in full])
            print(f"  {leg}: {vc:+.1f} / {vd:+.1f}")


if __name__ == "__main__":
    main()
