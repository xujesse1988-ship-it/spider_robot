#!/usr/bin/env python3
"""T1 第 1 步:从 B 组交接段实测超定拟合逐腿相对刚度 k̂(PAPER-PLAN §4 T1)。

模型(HANDOVER-DESIGN 附录 A 的并联弹簧,交接期身体位移):
  Δb_{j,r} = c0·δ_j·(k_j − Σ_{i≠j} k_i·s_i^{(j)})·g_r + α + β·(r−1)
  归一 Σk=1;B 组均分 s_i=1/5 ⇒ 再分配因子 F_j = δ_j·(6k_j−1)/5。
  c0   = 有效指令比例(吸收传递比/单位账,理想=1);
  g_r  = 1+γ(r−1) 稳态储能爬坡的乘性项(逐轮增长压在再分配项上——
         数据显示增长集中于最硬腿,均匀蠕变式加性增长对不上);
  α,β  = 每次交接的共模下沉及其逐轮增长(份额无法消除的部分,
         直接决定 D 条件的改进上限——比份额解本身更要紧的输出)。
模型族 M1(c0)/M2(+γ)/M3(+γ,α,β)/M4(c0,α,β) 用 AICc + 留一重复交叉
验证选型;拟合 k̂ 与 08-24 斜率折算 k̂ 交叉验证;再用 B 拟合参数
样本外预测 C 组(轮转权重份额 d=1..5 → 0/0.05/0.10/0.25/0.60,
_share_now 口径)检验模型可迁移性。最后按拟合 k̂ 预解 D 条件份额
(L1 可行性/残余预测),给 PAPER-PLAN T1 第 2 步的门控数字。

用法:.venv/bin/python software/logs_analysis/fit_stiffness.py
输入:~/ab_cache/{0825,0825pm,0826}/{B,C}/report.txt + 对应 lean 日志(δ 表)。
"""
import itertools
import re
import sys

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, "/home/shaopeng/spider/software/logs_analysis")
import ab_quant as q

LEGS = ("L1", "R1", "L3", "R3", "R2", "L2")          # 报告口径的列序
SLOTS = ("R3", "L1", "R2", "L3", "R1", "L2")         # 轮转槽序(_share_now)
RUNS = {  # (重复, 条件) -> (缓存目录, 日志戳)
    (1, "B"): ("0825/B", "20260825_105000"),
    (2, "B"): ("0825pm/B", "20260825_154921"),
    (3, "B"): ("0826/B", "20260826_102805"),
    (1, "C"): ("0825/C", "20260825_105827"),
    (2, "C"): ("0825pm/C", "20260825_155727"),
    (3, "C"): ("0826/C", "20260826_101200"),
    # 08-31 D′ 判别实验(协议 §9):C31=对照(10mm/s),D31=铺设 20mm/s,份额同 C(轮转权重)
    (1, "C31"): ("0831/C1", "20260831_170013"),
    (2, "C31"): ("0831/C2", "20260831_171453"),
    (3, "C31"): ("0831/C3", "20260831_175215"),
    (1, "D31"): ("0831/D1", "20260831_164136"),
    (2, "D31"): ("0831/D2", "20260831_172720"),
    (3, "D31"): ("0831/D3", "20260831_174013"),
}
GROUP = {"B": "B", "C": "C", "C31": "31", "D31": "31"}   # 共模系数分组:同日同工况共享
# 08-24 斜率表(协议 §6)折算 k̂ ∝ 斜率/(1+斜率) 归一(PAPER-PLAN §3)
K_SLOPE = {"L1": 0.29, "R1": 0.22, "R3": 0.16, "L3": 0.14,
           "L2": 0.10, "R2": 0.09}
W_C = {1: 0.0, 2: 0.05, 3: 0.10, 4: 0.25, 5: 0.60}   # C 组前向槽距→份额

ROW = re.compile(r"^\s*\d+\s+(\w\d)#(\d) \|\s*([+-][0-9.]+) \|")
CUR = re.compile(r"^\s*\d+ (\w\d)#(\d): 均值 ([0-9.]+)A")
I0 = re.compile(r"首抬前静置均值 ([0-9.]+)A")


def load(rep, cond):
    """→ (Δb[leg][rnd] mm, δ 表, ΔI[leg][rnd] 窗均电流−基线 A,
    T[leg][rnd] 交接窗时长 s = vent −(ho−0.5),与分解窗同口径)。"""
    outdir, stem = RUNS[(rep, cond)]
    db, cw, i0 = {}, {}, None
    for line in open(f"/home/shaopeng/ab_cache/{outdir}/report.txt",
                     encoding="utf-8"):
        m = ROW.match(line)
        if m:
            db.setdefault(m.group(1), {})[int(m.group(2))] = float(m.group(3))
            continue
        m = CUR.match(line)
        if m:
            cw.setdefault(m.group(1), {})[int(m.group(2))] = float(m.group(3))
            continue
        m = I0.search(line)
        if m:
            i0 = float(m.group(1))
    ev, _, ho_cfg = q.parse_log(
        f"/home/shaopeng/spider/software/logs_analysis/lean_{stem}.log")
    lifts = q.build_lifts(ev, ho_cfg)
    tw, seen = {}, {}
    for L in lifts:
        seen[L["leg"]] = seen.get(L["leg"], 0) + 1
        tw.setdefault(L["leg"], {})[seen[L["leg"]]] = \
            L["vent"] - (L["ho"] - 0.5)
    assert set(db) == set(LEGS) and all(len(v) == 3 for v in db.values()), \
        f"{outdir} 分解表不全"
    di = {n: {r: cw[n][r] - i0 for r in (1, 2, 3)} for n in LEGS}
    return db, ho_cfg, di, tw


def shares(cond, leg):
    """抬 leg 时 5 支撑腿的份额 {腿:s}(B=均分;C=轮转距离权重)。"""
    if cond == "B":
        return {n: 0.2 for n in LEGS if n != leg}
    # C / C31 / D31 均为轮转距离权重 0.6/0.25/0.1/0.05/0(08-31 两组命令行同开 --handover-weights)
    k0 = SLOTS.index(leg)
    raw = {n: W_C[(SLOTS.index(n) - k0) % 6] for n in LEGS if n != leg}
    tot = sum(raw.values())
    return {n: v / tot for n, v in raw.items()}


def redis_factor(kvec, delta, cond):
    """再分配因子 F_j = δ_j·(k_j − Σ k_i s_i)(Σk=1)→ {leg: mm 系数}。"""
    k = dict(zip(LEGS, kvec))
    out = {}
    for j in LEGS:
        s = shares(cond, j)
        out[j] = delta[j] * (k[j] - sum(k[i] * s[i] for i in s))
    return out


def predict(params, model, delta, cond, legs_rounds):
    kvec, c0, gam, alp, bet = params
    F = redis_factor(kvec, delta, cond)
    out = []
    for j, r in legs_rounds:
        g = 1.0 + (gam if model in ("M2", "M3") else 0.0) * (r - 1)
        add = (alp + bet * (r - 1)) if model in ("M3", "M4") else 0.0
        out.append(c0 * F[j] * g + add)
    return np.array(out)


def unpack(x, model):
    k = np.abs(x[:6])
    k = k / k.sum()
    c0 = x[6]
    gam = x[7] if model in ("M2", "M3") else 0.0
    alp = x[8] if model in ("M3", "M4") else 0.0
    bet = x[9] if model in ("M3", "M4") else 0.0
    return k, c0, gam, alp, bet


def fit(data, model, k0=None):
    """data = [(delta, legs_rounds, y)] 逐重复。返回 (params, sse)。"""
    def sse(x):
        p = unpack(x, model)
        s = 0.0
        for delta, lr, y in data:
            s += ((predict(p, model, delta, "B", lr) - y) ** 2).sum()
        return s

    best = None
    starts = []
    base = [K_SLOPE[n] for n in LEGS]
    for kini in (base, [1 / 6.0] * 6):
        for c0ini in (1.0, 1.5):
            starts.append(np.array(kini + [c0ini, 0.3, 1.0, 0.5]))
    if k0 is not None:
        starts.append(np.array(list(k0) + [1.0, 0.3, 1.0, 0.5]))
    for x0 in starts:
        r = minimize(sse, x0, method="Nelder-Mead",
                     options={"maxiter": 20000, "xatol": 1e-6, "fatol": 1e-9})
        if best is None or r.fun < best.fun:
            best = r
    return unpack(best.x, model), best.fun


def npar(model):
    return {"M1": 6, "M2": 7, "M3": 9, "M4": 8}[model]  # k 自由 5 + 其余


def aicc(sse, n, p):
    return n * np.log(sse / n) + 2 * p + 2 * p * (p + 1) / max(n - p - 1, 1)


def fit_joint(jdata, model, k0):
    """联合拟合(任意条件集)。jdata=[(cond, delta, lr, y, dI, tw)]。共模项按 GROUP 分组
    共享系数(B / C / 31——08-31 的 C31 与 D31 同日同工况,系数共享,唯一差别是 D31 的
    窗时长减半:M8 预测其共模减半、M10 预测不变,数据裁决)。
    M5 : Δb = c0·F·g_r + α_g + β_g·(r−1)          (分组常数,2 参/组)
    M7 : Δb = c0·F·g_r + kI·ΔI_win                 (电流回归,1 参)
    M8 : Δb = c0·F·g_r + ρ_g·(1+β′(r−1))·T_win     (蠕变率×窗时长,1 参/组+β′)
    M9 : Δb = c0·F·g_r + kI·ΔI_win·T_win           (电流×时长,1 参)
    M10: Δb = c0·F·g_r + κ_g·(1+β′(r−1))·δ_j       (逐 mm 损耗∝铺设量,1 参/组+β′)"""
    groups = sorted({GROUP[t[0]] for t in jdata})
    gi = {g: i for i, g in enumerate(groups)}
    ng = len(groups)
    NP = {"M5": 7 + 2 * ng, "M7": 8, "M8": 8 + ng, "M9": 8, "M10": 8 + ng}

    def unpack(x):
        k = np.abs(x[:6]); k = k / k.sum()
        c0, gam = x[6], x[7]
        ex = x[8:]
        return k, c0, gam, ex

    def pred(x, cond, delta, lr, dI, tw):
        k, c0, gam, ex = unpack(x)
        F = redis_factor(k, delta, cond)
        g_ = gi[GROUP[cond]]
        out = []
        for (j, r), di, t in zip(lr, dI, tw):
            g = 1.0 + gam * (r - 1)
            if model == "M5":
                add = ex[2 * g_] + ex[2 * g_ + 1] * (r - 1)
            elif model == "M7":
                add = ex[0] * di
            elif model == "M8":
                add = ex[g_] * (1.0 + ex[ng] * (r - 1)) * t
            elif model == "M9":
                add = ex[0] * di * t
            else:  # M10
                add = ex[g_] * (1.0 + ex[ng] * (r - 1)) * delta[j]
            out.append(c0 * F[j] * g + add)
        return np.array(out)

    def sse(x):
        return sum(((pred(x, c, d, lr, dI, tw) - y) ** 2).sum()
                   for c, d, lr, y, dI, tw in jdata)

    base = list(k0)
    ex0 = {"M5": [2.0, 0.4] * ng, "M7": [1.2], "M8": [0.5] * ng + [0.2],
           "M9": [0.15], "M10": [0.06] * ng + [0.2]}[model]
    starts = [base + [1.0, 0.12] + ex0, base + [1.7, 0.16] + [v * 0.5 for v in ex0]]
    best = None
    for x0 in starts:
        r = minimize(sse, np.array(x0), method="Nelder-Mead",
                     options={"maxiter": 60000, "xatol": 1e-6, "fatol": 1e-9})
        if best is None or r.fun < best.fun:
            best = r
    k, c0, gam, ex = unpack(best.x)
    extra = {"groups": groups, "ex": ex, "ng": ng}
    return (k, c0, gam, extra), best.fun, NP[model], \
        (lambda cond, delta, lr, dI, tw, x=best.x: pred(x, cond, delta, lr, dI, tw))


def main():
    # ---- 数据 ----
    bdata, ball = [], []
    for rep in (1, 2, 3):
        db, delta, di, tw = load(rep, "B")
        lr = [(j, r) for j in LEGS for r in (1, 2, 3)]
        y = np.array([db[j][r] for j, r in lr])
        bdata.append((delta, lr, y))
        ball.append((rep, db, di, tw))
    n_obs = sum(len(d[2]) for d in bdata)
    delta = bdata[0][0]
    print("δ 表(mm,来自日志):", {n: delta[n] for n in LEGS})
    print(f"B 组观测 {n_obs} 点(3 重复 × 6 腿 × 3 轮)\n")

    # ---- 模型族拟合 + AICc + 留一重复 CV ----
    print("模型 | SSE | RMSE | AICc | 留一重复 CV-RMSE | 参数")
    results = {}
    for model in ("M1", "M2", "M3", "M4"):
        p, sse = fit(bdata, model)
        cv = []
        for hold in range(3):
            tr = [d for i, d in enumerate(bdata) if i != hold]
            ph, _ = fit(tr, model, k0=p[0])
            d0 = bdata[hold]
            e = predict(ph, model, d0[0], "B", d0[1]) - d0[2]
            cv.append(np.sqrt((e ** 2).mean()))
        k, c0, gam, alp, bet = p
        results[model] = p
        tag = (f"c0={c0:.2f}" + (f" γ={gam:.2f}" if model in ("M2", "M3")
               else "") + (f" α={alp:.2f} β={bet:.2f}"
               if model in ("M3", "M4") else ""))
        print(f"{model} | {sse:7.1f} | {np.sqrt(sse/n_obs):.2f}"
              f" | {aicc(sse, n_obs, npar(model)):7.1f}"
              f" | {np.mean(cv):.2f} | {tag}")
    print()

    # ---- 选型(AICc 最小)与 k̂ 交叉验证 ----
    sel = min(results, key=lambda m: aicc(
        sum(((predict(results[m], m, d, "B", lr) - y) ** 2).sum()
            for d, lr, y in bdata), n_obs, npar(m)))
    k, c0, gam, alp, bet = results[sel]
    kd = dict(zip(LEGS, k))
    print(f"选型 {sel}:c0={c0:.3f} γ={gam:.3f} α={alp:.3f} β={bet:.3f}")
    print("腿 | 拟合 k̂ | 斜率 k̂ | 差")
    for n in LEGS:
        print(f"{n} | {kd[n]:.3f} | {K_SLOPE[n]:.3f}"
              f" | {kd[n]-K_SLOPE[n]:+.3f}")
    ks = np.array([K_SLOPE[n] for n in LEGS])
    r = np.corrcoef(k, ks)[0, 1]
    print(f"Pearson r(拟合 vs 斜率)={r:.3f}\n")

    # ---- 预测 vs 实测逐腿表(模型闭环素材)----
    print("B 组逐腿逐轮:实测(三重复均值) vs 拟合预测 (mm)")
    print("腿 | 轮1 实/预 | 轮2 实/预 | 轮3 实/预")
    for j in LEGS:
        cells = []
        for rr in (1, 2, 3):
            meas = np.mean([db[j][rr] for _, db, _, _ in ball])
            pred = predict(results[sel], sel, delta, "B", [(j, rr)])[0]
            cells.append(f"{meas:+5.1f}/{pred:+5.1f}")
        print(f"{j} | " + " | ".join(cells))
    print()

    # ---- 样本外:预测 C 组(权重份额,同 δ 表/同参数)----
    print("C 组样本外预测(B 拟合参数 + 轮转权重份额):")
    err, rows = [], {}
    cdata = []
    for rep in (1, 2, 3):
        db, dC, diC, twC = load(rep, "C")
        lr = [(j, r) for j in LEGS for r in (1, 2, 3)]
        cdata.append((dC, lr, np.array([db[j][r] for j, r in lr]), diC, twC))
        for j in LEGS:
            for rr in (1, 2, 3):
                F = redis_factor(k, dC, "C")[j]
                g = 1.0 + (gam if sel in ("M2", "M3") else 0.0) * (rr - 1)
                add = (alp + bet * (rr - 1)) if sel in ("M3", "M4") else 0.0
                pv = c0 * F * g + add
                rows.setdefault((j, rr), []).append((db[j][rr], pv))
                err.append(db[j][rr] - pv)
    err = np.array(err)
    print(f"  RMSE={np.sqrt((err**2).mean()):.2f}mm"
          f"(B 组内 RMSE 对照见上表);逐腿(实测均值/预测):")
    for j in LEGS:
        cells = [f"{np.mean([m for m, _ in rows[(j, rr)]]):+5.1f}/"
                 f"{rows[(j, rr)][0][1]:+5.1f}" for rr in (1, 2, 3)]
        print(f"  {j} | " + " | ".join(cells))
    mC = sum(np.mean([m for m, _ in rows[(j, rr)]])
             for j in LEGS for rr in (1, 2, 3))
    pC = sum(rows[(j, rr)][0][1] for j in LEGS for rr in (1, 2, 3))
    print(f"  C 交接段总量:实测均值 {mC:+.1f} vs 预测 {pC:+.1f}mm")
    print("  → 若差距大:'共模项与份额无关'被 C 证伪,进入联合拟合。\n")

    # ---- 联合拟合:共模项机制赛马(B+C;--d31 时并入 08-31 的 C31/D31)----
    def pack(cond, d, lr, y, di, tw):
        return (cond, d, lr, y, np.array([di[j][r] for j, r in lr]),
                np.array([tw[j][r] for j, r in lr]))
    jdata = [pack("B", d, lr, y, ball[i][2], ball[i][3]) for i, (d, lr, y) in enumerate(bdata)]
    jdata += [pack("C", d, lr, y, di, tw) for d, lr, y, di, tw in cdata]
    with31 = "--d31" in sys.argv
    if with31:
        for cond in ("C31", "D31"):
            for rep in (1, 2, 3):
                try:
                    db, dd, di, tw = load(rep, cond)
                except Exception as e:
                    print(f"  {cond}#{rep} 缺:{e}"); continue
                lr = [(j, r) for j in LEGS for r in (1, 2, 3)]
                jdata.append(pack(cond, dd, lr, np.array([db[j][r] for j, r in lr]), di, tw))
    n_j = sum(len(t[3]) for t in jdata)
    conds = sorted({t[0] for t in jdata}, key=lambda c: ("B", "C", "C31", "D31").index(c))
    print(f"联合拟合({n_j} 点;条件 {'/'.join(conds)})。交接窗时长:" + "  ".join(
        f"{c} {np.concatenate([t[5] for t in jdata if t[0]==c]).mean():.2f}s" for c in conds))
    fits = {}
    models = ("M5", "M7", "M8", "M9", "M10")
    for model in models:
        (kJ, c0J, gamJ, extra), sseJ, p_n, predJ = fit_joint(jdata, model, k)
        kdJ = dict(zip(LEGS, kJ))
        ex, groups = extra["ex"], extra["groups"]
        if model == "M5":
            tag = " ".join(f"α_{g}={ex[2*i]:.2f} β_{g}={ex[2*i+1]:.2f}" for i, g in enumerate(groups))
        elif model == "M8":
            tag = " ".join(f"ρ_{g}={ex[i]:.3f}mm/s" for i, g in enumerate(groups)) + f" β'={ex[len(groups)]:.2f}"
        elif model == "M10":
            tag = " ".join(f"κ_{g}={ex[i]:.4f}mm/mm" for i, g in enumerate(groups)) + f" β'={ex[len(groups)]:.2f}"
        elif model == "M7":
            tag = f"kI={ex[0]:.2f}mm/A"
        else:
            tag = f"kIT={ex[0]:.3f}mm/(A·s)"
        a = aicc(sseJ, n_j, p_n)
        fits[model] = (a, kJ, c0J, gamJ, extra, predJ)
        print(f"  {model}: SSE={sseJ:6.1f} RMSE={np.sqrt(sseJ/n_j):.2f} AICc={a:6.1f} | "
              f"c0={c0J:.2f} γ={gamJ:.2f} | " + tag)
        print("    k̂:", {n: round(kdJ[n], 3) for n in LEGS})
    best = min(fits, key=lambda m: fits[m][0])
    print(f"  联合选型:{best}(AICc 最小)")
    if with31:
        print("\n  判别核心——各条件交接段三轮总量:实测 vs 各模型预测(mm;M8=∝窗时长,M10=∝铺设量):")
        for c in conds:
            rows_c = [t for t in jdata if t[0] == c]
            meas = np.mean([t[3].sum() for t in rows_c])
            preds = {m: np.mean([fits[m][5](t[0], t[1], t[2], t[4], t[5]).sum() for t in rows_c])
                     for m in ("M5", "M8", "M10")}
            print(f"    {c:>4}: 实测 {meas:5.1f} | " + " ".join(f"{m} {v:5.1f}" for m, v in preds.items()))
    print()

    # ---- T1 第 2 步预解:刚度感知份额 + D 条件门控账 ----
    _, kJ, c0J, gamJ, extra, _ = fits[best]
    kdJ = dict(zip(LEGS, kJ))
    print(f"D 条件门控账(按联合选型 {best} 的 k̂):")
    redis_B = redis_D = 0.0
    for j in LEGS:
        sup = [n for n in LEGS if n != j]
        kmax = max(kdJ[n] for n in sup)
        feas = kmax >= kdJ[j] - 1e-9
        coef = 0.0 if feas else kdJ[j] - kmax
        holder = max(sup, key=lambda n: kdJ[n])
        for rr in (1, 2, 3):
            g = 1.0 + gamJ * (rr - 1)
            redis_D += c0J * delta[j] * coef * g
            redis_B += c0J * redis_factor(kJ, delta, "B")[j] * g
        print(f"  抬 {j}(k̂={kdJ[j]:.3f}):支撑最硬 {holder}={kmax:.3f} → "
              + ("可精确归零" if feas else f"无解,残余系数 {coef:.3f}"))
    common_B = 60.2 - redis_B
    print(f"\n  账目:B 交接段实测 60.2 = 再分配项 ~{redis_B:.1f}"
          f" + 共模项 ~{common_B:.1f}mm。")
    print(f"  刚度感知份额(原 D 设计)只动再分配项:{redis_B:.1f}→"
          f"{redis_D:.1f}mm——**对 B 总账收益 ~{redis_B-redis_D:.0f}mm,"
          f"对 C 更少,'交接段 60→10'的预期不成立,按门控不上墙**。")
    if best == "M10":
        print("  共模项机制=逐 mm 损耗(∝铺设量 δ,与铺设用时无关):提速无净收益,"
              "杠杆只剩 δ 本身与份额分配(权重已证明能压 κ);见 08-31 D′ 报告。")
    elif best in ("M8", "M9"):
        print("  共模项机制=蠕变率×窗时长:缩短交接窗是纯软件杠杆(D′ 实验验证)。")
    else:
        print("  共模项分条件常数即权重已在砍共模——机制指向内应力/蠕变,见报告。")


if __name__ == "__main__":
    main()
