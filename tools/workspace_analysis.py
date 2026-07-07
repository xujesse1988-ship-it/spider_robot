#!/usr/bin/env python3
"""活动度分析：六足机器人上垂直墙面的运动学可行性。

回答两个问题（结论写入 docs/CLIMBING-DESIGN.md §7）：
  A. 在墙上行走时，吸盘轴线相对墙面法线的偏角有多大（波纹吸盘容差 ~15°）
  B. 地-墙过渡时，前腿能把吸盘以可接受偏角按到墙上的"可落足区"有多大，
     身体俯仰（绕后髋抬头）能改善多少

模型（前腿竖直平面 2D，忽略 coxa 偏摆，保守）：
  髋 = 原点；coxa 43mm 水平 + femur 80 + tibia 134
  关节电气行程（舵机 ±90° × 安装偏角）:
    femur α ∈ [-55°, +125°]，膝内角 θ ∈ [22°, 180°]（k=180-θ ∈ [0°,158°]）
  吸盘轴 = tibia 末段方向 a_t = α + θ - 180°
运行:  .venv/bin/python tools/workspace_analysis.py
"""
import math
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "software"))

from hexapod.config import DEFAULT_CONFIG as CFG
from hexapod.kinematics import leg_ik, WorkspaceError

ALPHA_LIM = (-55.0, 125.0)   # femur 电气行程（含 35° 安装偏角）
THETA_LIM = (22.0, 180.0)    # 膝内角（含 68° 安装偏角；>180°=反关节，不用）
H_HIP = 90.0                 # 站立时髋轴离地高度 mm
BODY_LEN = 167.0             # 前后髋距（官方 L1_TO_L3）


def ik2d(x, z):
    """竖直平面 IK；返回 (alpha_deg, theta_deg, a_t_deg) 或 None（不可达/超行程）。"""
    try:
        _, a, th = leg_ik(CFG, x, 0.0, z)
    except WorkspaceError:
        return None
    a, th = math.degrees(a), math.degrees(th)
    if not (ALPHA_LIM[0] <= a <= ALPHA_LIM[1] and THETA_LIM[0] <= th <= THETA_LIM[1]):
        return None
    return a, th, a + th - 180.0


def knee_xz(a_deg):
    a = math.radians(a_deg)
    return CFG.coxa_len + CFG.femur_len * math.cos(a), CFG.femur_len * math.sin(a)


print("=" * 72)
print("A. 墙面行走：支撑相内吸盘轴线偏离墙面法线的角度")
print("   （足端在腿平面 (reach±步幅/2, -身高)；理想=0°，波纹吸盘容差≈15°）")
print("=" * 72)
for h in (70, 90):
    for step in (0, 30, 40, 60):
        angs = []
        for dx in (-step / 2, 0, step / 2):
            r = ik2d(CFG.foot_reach + dx, -h)
            if r:
                angs.append(abs(r[2] + 90.0))  # 距竖直(-90°=垂直入墙)的偏角
        if len(angs) == 3:
            print(f"  身高{h}mm 步幅{step:2d}mm: 偏角 {min(angs):4.1f}°~{max(angs):4.1f}°")
        else:
            print(f"  身高{h}mm 步幅{step:2d}mm: 超出行程!")

print()
print("=" * 72)
print("B. 地-墙过渡：前腿在竖直墙上的可落足区（相对前髋的高度范围）")
print("   条件：IK 可解 + 关节行程内 + 吸盘轴偏离水平 ≤tol + 膝不撞墙 + 足不落地")
print("=" * 72)
for tol in (15.0, 20.0):
    print(f"\n  --- 吸盘偏角容差 {tol:.0f}° ---")
    print(f"  {'俯仰':>4} {'前髋高':>6} | " + " ".join(f"D={d:<3d}" for d in range(80, 201, 20)))
    for phi in (0.0, 15.0, 30.0, 45.0):
        h_hip = H_HIP + BODY_LEN * math.sin(math.radians(phi))  # 绕后髋线抬头
        cphi, sphi = math.cos(math.radians(phi)), math.sin(math.radians(phi))
        cells = []
        for D in range(80, 201, 20):
            zs = []
            for zw in range(int(-h_hip) + 15, 220, 2):   # 墙上点（世界系，相对前髋）
                # 世界 -> 腿平面（身体抬头 phi，腿平面随身体转 -phi）
                x_leg = cphi * D + sphi * zw
                z_leg = -sphi * D + cphi * zw
                r = ik2d(x_leg, z_leg)
                if r is None:
                    continue
                a, th, a_t = r
                if abs(a_t + phi) > tol:        # 世界系吸盘轴偏离水平
                    continue
                kx, kz = knee_xz(a)             # 膝在世界系的水平位置
                if cphi * kx - sphi * kz > D - 10:
                    continue                    # 膝撞墙
                zs.append(zw)
            cells.append(f"{min(zs):+4d}~{max(zs):+4d}" if zs else "  ——   ")
        print(f"  {phi:4.0f}° {h_hip:5.0f}mm | " + " ".join(f"{c:<9s}" for c in cells))

print("""
  说明：D=前髋到墙的水平距离(mm)；表格值=可落足高度范围(相对前髋,mm)；
  "——"=不存在可行落足点。前髋高=该俯仰角下前髋离地高度（绕后髋抬头）。""")
