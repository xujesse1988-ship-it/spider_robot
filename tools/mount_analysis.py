#!/usr/bin/env python3
"""地-墙过渡运动学可行性（三维，含 coxa 偏摆与身体俯仰）。2026-09-06 探索稿。

世界系：地面 z=0，墙面 x=0（机器人在 x<0 一侧，头朝墙），墙法线 -x。
身体系：x 前 y 左 z 上；身体位姿 = (xb, zb, phi)，phi 抬头为正（绕 y）。
吸盘轴 = 腿平面内 a_t + cup_delta，与压入方向（地面 -z_w / 墙面 +x_w）的夹角
= 总倾角；分解为腿平面内分量和垂直腿平面（coxa 偏摆造成）的分量。

运行（项目根）：.venv/bin/python tools/mount_analysis.py        # 一、二
            .venv/bin/python tools/mount_analysis.py 2b 3b 4  # 自由构型搜索/中后腿落墙带/静力
            .venv/bin/python tools/mount_analysis.py 6:55,60  # 最少换步路径（腹面深55, 目标φ≥60）
假设待实机核：COXA_MAX、BELLY（髋面以下舱体厚度）、机身外廓 ±100、重心=机身中心髋面下 10mm。
"""
import math, sys, itertools
sys.path.insert(0, "/home/shaopeng/spider/software")
from hexapod.config import DEFAULT_CONFIG as CFG, LEG_NAMES
from hexapod.kinematics import leg_ik, WorkspaceError

ALPHA_LIM = (-40.3, 139.7)
THETA_LIM = (0.0, 176.4)
DELTA = CFG.cup_delta_deg          # -25.2
TOL = 12.0                         # climb.py TILT_BAND_DEG
IK_MARGIN = 3.0                    # climb.py D_SAFE_MARGIN
COXA_MAX = 60.0                    # coxa 相对中性的最大偏摆（前腿指正前/后腿指正后需 55°，待实机核）
BELLY = 40.0                       # 髋平面以下舱体厚度：09-08 实测约 35，取 40 留余量
X_WALL = 0.0
d2r, r2d = math.radians, math.degrees


def rot_b2w(phi):
    c, s = math.cos(phi), math.sin(phi)
    return ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))   # 行=世界轴, 列=身体轴


def b2w(v, pose):
    xb, zb, phi = pose
    R = rot_b2w(phi)
    return tuple(sum(R[i][j] * v[j] for j in range(3)) + (xb, 0.0, zb)[i] for i in range(3))


def w2b(v, pose):
    xb, zb, phi = pose
    R = rot_b2w(phi)
    d = (v[0] - xb, v[1], v[2] - zb)
    return tuple(sum(R[j][i] * d[j] for j in range(3)) for i in range(3))


def w2b_dir(v, pose):
    R = rot_b2w(pose[2])
    return tuple(sum(R[j][i] * v[j] for j in range(3)) for i in range(3))


def ang(u, v):
    d = sum(a * b for a, b in zip(u, v))
    n = math.sqrt(sum(a * a for a in u)) * math.sqrt(sum(a * a for a in v))
    return r2d(math.acos(max(-1.0, min(1.0, d / n))))


def solve(leg, p_body, press_dir_body):
    """单腿：身体系足端点 + 身体系压入方向 -> 解与倾角。None = IK 不可达/超行程。"""
    psi = d2r(leg.mount_angle_deg)
    x, y = p_body[0] - leg.mount_x, p_body[1] - leg.mount_y
    xl = x * math.cos(psi) + y * math.sin(psi)
    yl = -x * math.sin(psi) + y * math.cos(psi)
    z = p_body[2]
    try:
        g, a, th = leg_ik(CFG, xl, yl, z)
    except WorkspaceError:
        return None
    # IK 包络余量
    r = math.hypot(xl, yl) - CFG.coxa_len
    d = math.hypot(r, z)
    if d > CFG.femur_len + CFG.tibia_len - IK_MARGIN:
        return None
    ad, thd = r2d(a), r2d(th)
    if not (ALPHA_LIM[0] <= ad <= ALPHA_LIM[1] and THETA_LIM[0] <= thd <= THETA_LIM[1]):
        return None
    a_c = a + th - math.pi + d2r(DELTA)          # 腿平面内吸盘轴角（自外向水平起，向上为正）
    cg, sg = math.cos(g), math.sin(g)
    axis_leg = (math.cos(a_c) * cg, math.cos(a_c) * sg, math.sin(a_c))
    # 腿系 -> 身体系（转 psi）
    cp, sp = math.cos(psi), math.sin(psi)
    axis_b = (axis_leg[0] * cp - axis_leg[1] * sp, axis_leg[0] * sp + axis_leg[1] * cp, axis_leg[2])
    # 腿平面法线（身体系）
    beta = psi + g
    n_plane = (-math.sin(beta), math.cos(beta), 0.0)
    oop = r2d(math.asin(max(-1, min(1, sum(a * b for a, b in zip(n_plane, press_dir_body))))))
    tilt = ang(axis_b, press_dir_body)
    # 膝点（身体系）
    kx = CFG.coxa_len * cg + CFG.femur_len * math.cos(a) * cg
    ky = CFG.coxa_len * sg + CFG.femur_len * math.cos(a) * sg
    kz = CFG.femur_len * math.sin(a)
    knee_b = (kx * cp - ky * sp + leg.mount_x, kx * sp + ky * cp + leg.mount_y, kz)
    return dict(gamma=r2d(g), alpha=ad, theta=thd, tilt=tilt, oop=abs(oop), knee_b=knee_b)


def check_foot(leg, p_world, surface, pose, tol=TOL):
    """surface: 'floor' | 'wall'。返回 (ok, info)。"""
    press_w = (0.0, 0.0, -1.0) if surface == "floor" else (1.0, 0.0, 0.0)
    s = solve(leg, w2b(p_world, pose), w2b_dir(press_w, pose))
    if s is None:
        return False, "IK"
    if abs(s["gamma"]) > COXA_MAX:
        return False, f"coxa{s['gamma']:+.0f}°"
    kw = b2w(s["knee_b"], pose)
    if kw[2] < 8.0:
        return False, "膝触地"
    if kw[0] > X_WALL - 10.0:
        return False, "膝撞墙"
    if s["tilt"] > tol:
        return False, f"倾角{s['tilt']:.0f}°(面外{s['oop']:.0f}°)"
    return True, s


def body_ok(pose):
    """机身四角与腹面不撞地/墙（粗略：机身外廓 ±100 x ±100，腹面深 BELLY）。"""
    for bx, by in ((100, 100), (100, -100), (-100, 100), (-100, -100)):
        for bz in (0.0, -BELLY):
            w = b2w((bx, by, bz), pose)
            if w[2] < 5.0:
                return False, "腹面/机身触地"
            if w[0] > X_WALL - 10.0:
                return False, "机身撞墙"
    return True, ""


def feasible(contacts, pose, tol=TOL):
    """contacts: {leg: (p_world, surface)}。全部可行返回 True。"""
    ok, why = body_ok(pose)
    if not ok:
        return False, why
    for name, (p, surf) in contacts.items():
        ok, info = check_foot(CFG.leg(name), p, surf, pose, tol)
        if not ok:
            return False, f"{name}:{info}"
    return True, ""


# ---------------------------------------------------------------- 一、面外倾角（解析）
def part1():
    print("=" * 78)
    print("一、coxa 偏摆造成的面外倾角（与位置无关，只看腿平面方向 β 与俯仰 φ）")
    print("   地面足：asin(sinφ·|sinβ|)   墙面足：asin(cosφ·|sinβ|)   β=腿平面相对机身 x 轴")
    print("=" * 78)
    rows = [("前腿 L1/R1 中性 β=55°", 55), ("前腿 coxa 内摆 55° → β=0（指向正前）", 0),
            ("中腿 L2/R2 中性 β=90°", 90), ("中腿 coxa 前摆 45° → β=45°", 45),
            ("后腿 L3/R3 中性 β=125°", 125), ("后腿 coxa 后摆 55° → β=180°（指向正后）", 180)]
    phis = (0, 15, 30, 45, 60, 70, 75, 80, 90)
    print(f"{'':44s}" + "".join(f"φ={p:>3d}°" for p in phis))
    for label, beta in rows:
        sb = abs(math.sin(d2r(beta)))
        fl = "".join(f"{r2d(math.asin(math.sin(d2r(p)) * sb)):6.0f} " for p in phis)
        wl = "".join(f"{r2d(math.asin(math.cos(d2r(p)) * sb)):6.0f} " for p in phis)
        print(f"{label:44s}")
        print(f"{'   在地面':40s} {fl}")
        print(f"{'   在墙面':40s} {wl}")
    print("→ 容差 12~15°：中腿在地面只能撑到 φ≈15°，上墙要等 φ≥70°；前腿指正前/后腿指正后")
    print("  时面外角恒为 0，随便什么俯仰都能对正各自的面。")


# ---------------------------------------------------------------- 二、四足过渡阶段可达的俯仰角
def scan_pitch(contacts, x_range, z_range, phis, tol=TOL):
    """给定接触点集合，扫身体位姿；返回 {phi: [(xb,zb),...]}。"""
    out = {}
    for phi in phis:
        cells = []
        for xb in x_range:
            for zb in z_range:
                ok, _ = feasible(contacts, (xb, zb, d2r(phi)), tol)
                if ok:
                    cells.append((xb, zb))
        out[phi] = cells
    return out


def part2():
    print()
    print("=" * 78)
    print("二、四足阶段：前两足吸在墙上（β=0）、后两足踩地（β=180°）、中腿悬空")
    print("   扫身体位姿 (xb, zb, φ)，条件：四腿 IK 可达(余量3mm)+行程内+吸盘总倾角≤12°")
    print("   +膝不触地/撞墙+机身腹面(髋下40mm)不触地不撞墙")
    print("=" * 78)
    # 起始：平身站高 90，前髋距墙 D，前吸盘高 hw（离地），后足在后髋正后方 rb 处
    for D, hw, rb in ((140, 200, 150), (140, 230, 150), (160, 230, 170), (120, 200, 130)):
        xb0 = X_WALL - D - 83.5
        contacts = {
            "L1": ((X_WALL, 63.0, hw), "wall"), "R1": ((X_WALL, -63.0, hw), "wall"),
            "L3": ((xb0 - 83.5 - rb, 63.0, 0.0), "floor"), "R3": ((xb0 - 83.5 - rb, -63.0, 0.0), "floor"),
        }
        phis = list(range(0, 91, 5))
        res = scan_pitch(contacts, [xb0 + dx for dx in range(-60, 121, 5)],
                         list(range(40, 261, 5)), phis)
        ok0, why0 = feasible(contacts, (xb0, 90.0, 0.0))
        print(f"\n  前髋距墙 D={D} 前吸盘离地 hw={hw} 后足距后髋 rb={rb}：起始位姿(平身,高90) "
              f"{'可行' if ok0 else '不可行:' + why0}")
        line = []
        for phi in phis:
            c = res[phi]
            if c:
                xs = [a for a, _ in c]; zs = [b for _, b in c]
                line.append(f"φ={phi:2d}°: xb∈[{min(xs) - xb0:+4.0f},{max(xs) - xb0:+4.0f}] "
                            f"zb∈[{min(zs):3.0f},{max(zs):3.0f}] ({len(c)}格)")
            else:
                line.append(f"φ={phi:2d}°: ——")
        for l in line:
            print("    " + l)


# ---------------------------------------------------------------- 三、中/后腿上墙可落足带
def part3():
    print()
    print("=" * 78)
    print("三、中腿 / 后腿 在给定俯仰下能否落到墙上（扫落点高度与 coxa 偏摆，倾角≤12°）")
    print("   身体位姿取：中髋距墙 Dm（身体系 x 方向量到墙面的距离由 xb、φ 决定）")
    print("=" * 78)
    for phi in (60, 70, 75, 80, 85, 90):
        print(f"\n  φ={phi}°")
        for name in ("L2", "L3", "L1"):
            leg = CFG.leg(name)
            best = None
            hits = 0
            for dwall in range(60, 201, 10):     # 该髋沿身体 -z... 用世界：髋到墙的水平距离
                # 让该腿髋轴在世界系距墙 dwall、离地 300（离地足够高，地面不参与）
                # 由 pose 反推：髋世界 x = xb + mount_x cosφ ... 直接搜 xb
                for h in range(-200, 260, 10):   # 落点高度相对髋（世界 z）
                    # 取 xb 使得该髋 x = X_WALL - dwall
                    R = rot_b2w(d2r(phi))
                    hx = R[0][0] * leg.mount_x + R[0][2] * 0.0
                    xb = X_WALL - dwall - hx
                    zb = 300.0 - (R[2][0] * leg.mount_x)
                    pose = (xb, zb, d2r(phi))
                    hip_w = b2w((leg.mount_x, leg.mount_y, 0.0), pose)
                    p = (X_WALL, hip_w[1], hip_w[2] + h)
                    ok, info = check_foot(leg, p, "wall", pose)
                    if ok:
                        hits += 1
                        if best is None or info["tilt"] < best[0]:
                            best = (info["tilt"], dwall, h, info["gamma"], info["oop"])
            if best:
                print(f"    {name}: 可行格 {hits:3d}；最正一格 倾角{best[0]:4.1f}° 髋距墙{best[1]} "
                      f"落点相对髋高{best[2]:+d} coxa偏摆{best[3]:+.0f}° 面外{best[4]:.1f}°")
            else:
                print(f"    {name}: 无可行落点")


if __name__ == "__main__" and len(sys.argv) == 1:
    part1()
    part2()


# ---------------------------------------------------------------- 二b、四足阶段：每个俯仰角下是否存在任何可行的四足构型
def part2b(coxa_max=55.0):
    print()
    print("=" * 78)
    print(f"二b、四足阶段：对每个 φ 搜 (前吸盘高 hw, 后足距 rb, xb, zb)，存在即可行；coxa 摆幅≤{coxa_max:.0f}°")
    print("   前腿指向 β=max(0,55-coxa_max)，后腿 β=min(180,125+coxa_max)")
    print("=" * 78)
    bf = max(0.0, 55.0 - coxa_max); bb = min(180.0, 125.0 + coxa_max)
    from collections import Counter
    for phi in range(0, 91, 10):
        best = None; fails = Counter(); n = 0
        for hw in range(120, 421, 20):
            for rb in range(80, 241, 20):
                for xb_rel in range(-120, 121, 10):     # xb 相对"前髋距墙 140"的基准
                    xb = X_WALL - 140.0 - 83.5 + xb_rel
                    for zb in range(30, 301, 10):
                        pose = (xb, float(zb), d2r(phi))
                        # 前吸盘：世界 y 使 β=bf；足在 (0, y, hw)。先算髋世界 y=±63，
                        # β 由足相对髋的身体系 xy 方向决定 → 用身体系直接放：足身体系方向 bf
                        contacts = {}
                        okall = True
                        for name, sgn in (("L1", 1), ("R1", -1)):
                            leg = CFG.leg(name)
                            # 足在世界 x=0, z=hw；求 y：身体系里足相对髋的方向角 = sgn*bf
                            # 身体系 dx = w2b 之后的 x 分量（与 y 无关），dy = dx*tan(bf)
                            pb0 = w2b((X_WALL, 0.0, float(hw)), pose)
                            dx = pb0[0] - leg.mount_x
                            if dx <= 0: okall = False; break
                            dy = dx * math.tan(d2r(sgn * bf))
                            y_w = leg.mount_y + dy          # 身体 y 与世界 y 相同（绕 y 转）
                            contacts[name] = ((X_WALL, y_w, float(hw)), "wall")
                        if not okall:
                            fails["前足在髋后"] += 1; continue
                        for name, sgn in (("L3", 1), ("R3", -1)):
                            leg = CFG.leg(name)
                            pb0 = w2b((xb - 200.0, 0.0, 0.0), pose)   # 仅为取 z 无关，需解 x
                            # 后足：世界 z=0，世界 x = 髋世界 x - rb
                            hip_w = b2w((leg.mount_x, leg.mount_y, 0.0), pose)
                            xf = hip_w[0] - rb
                            pbf = w2b((xf, 0.0, 0.0), pose)
                            dx = pbf[0] - leg.mount_x
                            if dx >= 0: okall = False; break
                            dy = -dx * math.tan(d2r(sgn * (180.0 - bb)))
                            contacts[name] = ((xf, leg.mount_y + dy, 0.0), "floor")
                        if not okall:
                            fails["后足在髋前"] += 1; continue
                        ok, why = feasible(contacts, pose)
                        n += 1
                        if ok:
                            # 余量：取四腿最大倾角最小者
                            worst = 0.0
                            for name, (p, surf) in contacts.items():
                                _, s = check_foot(CFG.leg(name), p, surf, pose)
                                worst = max(worst, s["tilt"])
                            if best is None or worst < best[0]:
                                best = (worst, hw, rb, xb_rel, zb, contacts)
                        else:
                            fails[why.split(":")[0] + ":" + why.split(":")[-1][:4]] += 1
        if best:
            w, hw, rb, xr, zb, c = best
            print(f"  φ={phi:2d}°: 可行  最正构型 最大倾角{w:4.1f}° 前吸盘离地{hw} 后足距后髋{rb} "
                  f"身高zb={zb} 前髋距墙≈{140 - xr - (0)}(平身口径)")
        else:
            top = ", ".join(f"{k}×{v}" for k, v in fails.most_common(4))
            print(f"  φ={phi:2d}°: —— 无可行构型（{n} 组；失败原因 {top}）")


# ---------------------------------------------------------------- 三b、中/后腿上墙：coxa 摆幅受限
def part3b(coxa_max=40.0):
    print()
    print("=" * 78)
    print(f"三b、中腿/后腿落墙可行带（coxa 摆幅≤{coxa_max:.0f}°，倾角≤12°；扫髋距墙、落点高、落点侧向）")
    print("=" * 78)
    for phi in (50, 60, 70, 75, 80, 90):
        print(f"\n  φ={phi}°")
        for name in ("L2", "L3"):
            leg = CFG.leg(name)
            cells = []
            for dwall in range(40, 221, 10):
                for h in range(-220, 221, 10):
                    for dy in range(-120, 121, 10):
                        R = rot_b2w(d2r(phi))
                        xb = X_WALL - dwall - R[0][0] * leg.mount_x
                        zb = 400.0 - R[2][0] * leg.mount_x
                        pose = (xb, zb, d2r(phi))
                        hip_w = b2w((leg.mount_x, leg.mount_y, 0.0), pose)
                        p = (X_WALL, hip_w[1] + dy, hip_w[2] + h)
                        ok, info = check_foot(leg, p, "wall", pose)
                        if ok and abs(info["gamma"]) <= coxa_max:
                            cells.append((info["tilt"], dwall, h, dy, info["gamma"]))
            if cells:
                cells.sort()
                ds = sorted({c[1] for c in cells}); hs = [c[2] for c in cells]
                t0 = cells[0]
                print(f"    {name}: {len(cells):4d} 格；髋距墙 {min(ds)}~{max(ds)}，落点相对髋高 "
                      f"{min(hs):+d}~{max(hs):+d}；最正: 倾角{t0[0]:.1f}° 髋距墙{t0[1]} 高{t0[2]:+d} "
                      f"侧偏{t0[3]:+d} coxa{t0[4]:+.0f}°")
            else:
                print(f"    {name}: 无可行落点")


# ---------------------------------------------------------------- 四、四足阶段吸盘剪切载荷估算
def part4():
    print()
    print("=" * 78)
    print("四、四足阶段静力估算（刚体、准静态）：W=34.7N，重心取机身中心髋面下 10mm")
    print("   力矩平衡（绕后足触地线）：xc·W = h·Fn + D·Fs；两极端：Fn=0（全靠墙盘剪切）/ Fs=0（全靠地面摩擦）")
    print("=" * 78)
    W = 34.7
    for phi, xb_rel, zb, hw, rb in ((0, 0, 90, 230, 150), (20, -40, 85, 230, 150),
                                    (40, -20, 120, 300, 120), (60, 0, 160, 360, 100)):
        pose = (X_WALL - 140.0 - 83.5 + xb_rel, float(zb), d2r(phi))
        com = b2w((0.0, 0.0, -10.0), pose)
        hipL3 = b2w((-83.5, 63.0, 0.0), pose)
        xr = hipL3[0] - rb                      # 后足触地 x
        xc = com[0] - xr; D = X_WALL - xr; h = hw
        Fs_max = xc * W / D; Fn_max = xc * W / h
        print(f"  φ={phi:2d}° 重心距后足水平{xc:5.0f}mm 后足距墙{D:4.0f} 墙盘高{h:3.0f}: "
              f"Fn=0→每盘剪切{Fs_max / 2:4.1f}N(容量15N,SF{15 / (Fs_max / 2):.1f}) | "
              f"Fs=0→地面需摩擦{Fn_max:4.1f}N(μ≥{Fn_max / W:.2f})")


if __name__ == "__main__" and len(sys.argv) > 1:
    for a in sys.argv[1:]:
        f = {"2b": part2b, "2b40": lambda: part2b(40.0), "3b": part3b, "4": part4}.get(a)
        if f: f()


# ---------------------------------------------------------------- 五、贪心阶梯：固定接触转到极限 → 换步 → 再转
def four_contacts(hw, rb, pose_for_rear=None, rear_x=None):
    c = {"L1": ((X_WALL, 63.0, float(hw)), "wall"), "R1": ((X_WALL, -63.0, float(hw)), "wall")}
    if rear_x is not None:
        c["L3"] = ((rear_x, 63.0, 0.0), "floor"); c["R3"] = ((rear_x, -63.0, 0.0), "floor")
    return c


def max_pitch_fixed(contacts, pose, dphi=2.5, step=10):
    """从 pose 出发，固定接触，逐 dphi 抬头；每级在 (xb,zb) 邻域 ±40 内找可行点（连通近似）。
    返回 (phi_max_deg, pose_at_max, trace)。"""
    xb, zb, phi = pose
    trace = [pose]
    from collections import Counter
    while True:
        nphi = phi + d2r(dphi)
        found = None
        fails = Counter()
        for dx in range(-40, 41, step):
            for dz in range(-40, 41, step):
                cand = (xb + dx, zb + dz, nphi)
                ok, why = feasible(contacts, cand)
                if ok:
                    # 取离原位最近的
                    if found is None or abs(dx) + abs(dz) < abs(found[0] - xb) + abs(found[1] - zb):
                        found = cand
                else:
                    fails[why.split(":")[0]] += 1
        if found is None:
            return r2d(phi), (xb, zb, phi), trace, fails
        xb, zb, phi = found
        trace.append(found)
        if r2d(phi) >= 89.9:
            return r2d(phi), (xb, zb, phi), trace, fails


def restep(contacts, pose, which):
    """在当前位姿下把 which（'front'/'rear'）换到"最有利"的新落点：前足取最高可行 hw，
    后足取离墙最近可行 rear_x（保守：要求新旧都在同一位姿可行）。"""
    xb, zb, phi = pose
    if which == "front":
        best = None
        for hw in range(100, 601, 10):
            c = dict(contacts); c.update(four_contacts(hw, None))
            ok, _ = feasible(c, pose)
            if ok:
                best = hw
        if best is None:
            return None
        c = dict(contacts); c.update(four_contacts(best, None)); return c, f"前足→墙高{best}"
    else:
        best = None
        for rx in range(int(xb) - 400, int(X_WALL) - 20, 10):
            c = dict(contacts)
            c["L3"] = ((float(rx), 63.0, 0.0), "floor"); c["R3"] = ((float(rx), -63.0, 0.0), "floor")
            ok, _ = feasible(c, pose)
            if ok:
                best = rx
        if best is None:
            return None
        c = dict(contacts)
        c["L3"] = ((float(best), 63.0, 0.0), "floor"); c["R3"] = ((float(best), -63.0, 0.0), "floor")
        return c, f"后足→距墙{X_WALL - best:.0f}"


def part5(D=140, hw=230, rb=150):
    print()
    print("=" * 78)
    print("五、贪心阶梯（四足：前 β=0 后 β=180，中腿悬空）：固定接触抬头到极限→换步→再抬")
    print(f"   起点：平身高 90，前髋距墙 {D}，前吸盘离地 {hw}，后足距后髋 {rb}")
    print("=" * 78)
    xb0 = X_WALL - D - 83.5
    pose = (xb0, 90.0, 0.0)
    contacts = four_contacts(hw, rb, rear_x=xb0 - 83.5 - rb)
    ok, why = feasible(contacts, pose)
    print(f"  起点 {'可行' if ok else '不可行 ' + why}")
    stage = 0
    while stage < 14:
        pm, pose, trace, fails = max_pitch_fixed(contacts, pose)
        top = ", ".join(f"{k}×{v}" for k, v in fails.most_common(3))
        print(f"  阶段{stage:2d}: 固定接触抬到 φ={pm:4.1f}°  身体 xb={pose[0] - xb0:+5.0f} zb={pose[1]:4.0f}"
              f"  （再抬受阻: {top}）")
        if pm >= 89.9:
            break
        # 换步：先前后各试，选能让下一段抬更多的
        opts = []
        for which in ("front", "rear"):
            r = restep(contacts, pose, which)
            if r:
                c2, desc = r
                pm2, _, _, _ = max_pitch_fixed(c2, pose)
                opts.append((pm2, desc, c2))
        if not opts:
            print("  没有可换的步，停止"); break
        opts.sort(key=lambda t: -t[0])
        pm2, desc, c2 = opts[0]
        if pm2 <= pm + 0.1:
            # 试连换两次
            r2 = restep(c2, pose, "rear" if "前" in desc else "front")
            if r2:
                c3, desc3 = r2
                pm3, _, _, _ = max_pitch_fixed(c3, pose)
                if pm3 > pm + 0.1:
                    print(f"          换步: {desc} + {desc3}")
                    contacts = c3; stage += 1; continue
            print(f"          换步后仍无法继续抬头（最好 {desc} → {pm2:.1f}°），停止"); break
        print(f"          换步: {desc}")
        contacts = c2
        stage += 1
    # 报告终态接触
    for n, (p, s) in sorted(contacts.items()):
        print(f"  终态 {n}: {s} ({p[0]:.0f},{p[1]:.0f},{p[2]:.0f})")


if __name__ == "__main__" and len(sys.argv) > 1:
    for a in sys.argv[1:]:
        if a == "5":
            part5()
        elif a.startswith("5:"):
            D, hw, rb = (int(v) for v in a[2:].split(","))
            part5(D, hw, rb)


# ---------------------------------------------------------------- 六、最少换步路径搜索（四足阶段）
def part6(belly=None, goal_phi=60.0, tol=TOL):
    global BELLY
    if belly is not None:
        BELLY = belly
    import heapq, time as _t
    t0 = _t.time()
    PHIS = [2.5 * i for i in range(0, 37)]            # 0..90
    XBS = list(range(-380, -40, 10))                    # 身体 x（墙在 0）
    ZBS = list(range(40, 271, 10))
    HWS = list(range(150, 481, 10))                     # 前吸盘离地高
    RXS = list(range(-560, -60, 10))                    # 后足世界 x
    cells = [(x, z) for x in XBS for z in ZBS]
    L1, L3 = CFG.leg("L1"), CFG.leg("L3")
    body = {}; front = {}; rear = {}
    for pi, phi in enumerate(PHIS):
        ph = d2r(phi)
        body[pi] = frozenset(c for c in cells if body_ok((c[0], c[1], ph))[0])
        for hi, hw in enumerate(HWS):
            front[pi, hi] = frozenset(c for c in body[pi]
                                      if check_foot(L1, (X_WALL, 63.0, float(hw)), "wall", (c[0], c[1], ph), tol)[0])
        for ri, rx in enumerate(RXS):
            rear[pi, ri] = frozenset(c for c in body[pi]
                                     if check_foot(L3, (float(rx), 63.0, 0.0), "floor", (c[0], c[1], ph), tol)[0])
    print(f"  预计算 {_t.time() - t0:.0f}s")

    def feas(pi, hi, ri):
        return front[pi, hi] & rear[pi, ri]

    def near(A, B):
        """两位姿集合是否相邻（存在相距 ≤1 格的点）。"""
        if A & B:
            return True
        for x, z in A:
            for dx in (-10, 0, 10):
                for dz in (-10, 0, 10):
                    if (x + dx, z + dz) in B:
                        return True
        return False

    def rear_can_wall(pi, S):
        """存在位姿使 L3 能落墙（倾角≤tol，coxa≤40°）。"""
        ph = d2r(PHIS[pi])
        for x, z in S:
            pose = (x, z, ph)
            hip = b2w((L3.mount_x, L3.mount_y, 0.0), pose)
            for h in range(-200, 21, 10):
                if hip[2] + h < 20.0:        # 09-06 修正：落点不能低于地面（原先漏查，把
                    continue                 # "后腿 60~70° 可落墙"算得过早；mount_plan 实为 72.5°）
                for dy in (20, 40, 60):
                    ok, info = check_foot(L3, (X_WALL, hip[1] + dy, hip[2] + h), "wall", pose, tol)
                    if ok and abs(info["gamma"]) <= 40.0:
                        return True
        return False

    # 起点：φ=0，任意 (hw, rx) 可行且含平身高 90、前髋距墙 120~180 的位姿
    start = []
    for hi in range(len(HWS)):
        for ri in range(len(RXS)):
            S = feas(0, hi, ri)
            if any(z == 90 and -263.5 <= x <= -203.5 for x, z in S):
                start.append((hi, ri))
    print(f"  起点候选 {len(start)} 组")
    # Dijkstra：代价=换步次数（一次换步=换一对腿）
    INF = 10 ** 9
    dist = {}; prev = {}
    pq = []
    for hi, ri in start:
        dist[(0, hi, ri)] = 0; heapq.heappush(pq, (0, (0, hi, ri)))
    goal = None
    gi = int(goal_phi / 2.5)
    while pq:
        d, (pi, hi, ri) = heapq.heappop(pq)
        if d > dist.get((pi, hi, ri), INF):
            continue
        S = feas(pi, hi, ri)
        if pi >= gi and rear_can_wall(pi, S):
            goal = (pi, hi, ri); break
        # 抬头一级
        if pi + 1 < len(PHIS):
            S2 = feas(pi + 1, hi, ri)
            if S2 and near(S, S2):
                k = (pi + 1, hi, ri)
                if d < dist.get(k, INF):
                    dist[k] = d; prev[k] = ((pi, hi, ri), "抬头"); heapq.heappush(pq, (d, k))
        # 换前足 / 换后足（同位姿下新旧都可行）
        for hi2 in range(len(HWS)):
            if hi2 != hi:
                S2 = feas(pi, hi2, ri)
                if S2 and (S & S2):
                    k = (pi, hi2, ri)
                    if d + 1 < dist.get(k, INF):
                        dist[k] = d + 1; prev[k] = ((pi, hi, ri), f"换前足→墙高{HWS[hi2]}"); heapq.heappush(pq, (d + 1, k))
        for ri2 in range(len(RXS)):
            if ri2 != ri:
                S2 = feas(pi, hi, ri2)
                if S2 and (S & S2):
                    k = (pi, hi, ri2)
                    if d + 1 < dist.get(k, INF):
                        dist[k] = d + 1; prev[k] = ((pi, hi, ri), f"换后足→距墙{-RXS[ri2]}"); heapq.heappush(pq, (d + 1, k))
    print(f"  搜索 {_t.time() - t0:.0f}s；腹面深 {BELLY}mm，目标 φ≥{goal_phi}° 且后腿可落墙")
    if goal is None:
        # 报告能到的最大 φ
        mx = max(dist, key=lambda k: (k[0], -dist[k]))
        print(f"  ✗ 未达目标；可达最大 φ={PHIS[mx[0]]}°（换步 {dist[mx]} 次，前吸盘高 {HWS[mx[1]]}，后足距墙 {-RXS[mx[2]]}）")
        goal = mx
    # 回放路径
    path = []; k = goal
    while k in prev:
        pk, act = prev[k]; path.append((k, act)); k = pk
    path.reverse()
    print(f"  ✓ 换步 {dist[goal]} 次到 φ={PHIS[goal[0]]}°；起点 前吸盘高 {HWS[k[1]]} 后足距墙 {-RXS[k[2]]}")
    W = 34.7
    def load(pi, hi, ri, S):
        # 取位姿集合中位格，绕后足触地线力矩平衡：xc·W = h·Fn + D·Fs
        x, z = sorted(S)[len(S) // 2]
        pose = (x, z, d2r(PHIS[pi]))
        com = b2w((0.0, 0.0, -10.0), pose)
        rx = RXS[ri]; xc = com[0] - rx; D = X_WALL - rx; h = HWS[hi]
        return xc * W / D, xc * W / h, (x, z)
    for (pi, hi, ri), act in path:
        if act == "抬头":
            continue
        S = feas(pi, hi, ri)
        xs = sorted({x for x, _ in S}); zs = sorted({z for _, z in S})
        Fs, Fn, (x, z) = load(pi, hi, ri, S)
        print(f"    φ={PHIS[pi]:5.1f}°  {act:16s} 身体位姿 x∈[{xs[0]},{xs[-1]}] z∈[{zs[0]},{zs[-1]}]；"
              f"换步瞬间墙盘剪切上界 {'单盘' if '前' in act else '双盘各'}{Fs / (1 if '前' in act else 2):4.1f}N / 地面摩擦上界{Fn:4.1f}N")
    pi, hi, ri = goal
    S = feas(pi, hi, ri)
    print(f"    φ={PHIS[pi]:5.1f}°  终态：前吸盘高 {HWS[hi]}，后足距墙 {-RXS[ri]}，位姿格 {len(S)}")


if __name__ == "__main__" and len(sys.argv) > 1:
    for a in sys.argv[1:]:
        if a == "6":
            part6()
        elif a.startswith("6:"):
            b, g = (float(v) for v in a[2:].split(","))
            part6(b, g)
