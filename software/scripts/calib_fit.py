#!/usr/bin/env python3
"""±45° 脉宽 + attach 一体化标定：逐舵机记录 (脉宽, 关节角) 样本，线性拟合出
us_m45 / us_p45 / attach_deg，打印 config.py 可直接粘贴的 ServoCal 行。

原理：ServoCal 是"关节角-脉宽"线性模型，us_m45/us_p45/attach_deg 只是直线的
三个参数。逐舵机在 2~4 个脉宽下用几何法实测关节角，最小二乘拟合：
  attach_deg = 拟合线在 1500µs（新腿统一中位）处的关节角
  us_m45/us_p45 = 1500 ∓ 45/斜率——左右腿方向差天然在斜率符号里
角度实测（记录命令见 REPL）：
  femur 高差法  rech <Δh>      sinα = Δh/80；Δh = 膝轴螺丝心高 − 髋轴螺丝心高(mm)
  tibia 三边法  recd <AP>      AP = femur 舵盘螺丝心→吸盘唇口圆心直线距离(mm)，
                               余弦定理解 θ，k = 180−θ（l3 默认 120，--l3 可改）
  coxa 投点法   recp <x> <y>   唇口圆心垂直投点的纸上坐标；先 base/hip 建基准
  通用          rec <角度>      自行算好的关节角（度）

前提：机身垫高在稳固支架上压重、六腿悬空全程可自由摆；方格纸与基线沿用
γ 基线法的布置。使能后全舵机回 1500 中位，悬空无风险。q / Ctrl-C 断电退出。

用法:
  python calib_fit.py --mock     # 干跑熟悉流程
  python calib_fit.py            # 真机（默认 /dev/ttyACM0）
"""
import argparse
import json
import math
import os
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.config import DEFAULT_CONFIG as CFG
from hexapod.driver import Servo2040Driver, MockDriver, NUM_SERVOS

CENTER_US = 1500.0
US_LO, US_HI = 600.0, 2400.0    # 软限位：防误敲极端值撞结构
RAMP_STEP, RAMP_DT = 10.0, 0.02  # 缓动：10µs/20ms ≈ 全程 2s
JOINTS = ("coxa", "femur", "tibia")

HELP = """命令（角度单位度，坐标用方格纸格子数一致即可）:
  s L2 femur | s 10      选舵机（腿名+关节 或 通道号；关节可缩写 c/f/t）
  1520 | +10 | -25       绝对/相对脉宽，缓动到位
  base x1 y1 x2 y2       基线 C_L1→C_R1 两投点坐标（全局记一次）
  hip x y                当前腿 coxa 轴投点（每腿一次）
  rech <Δh> / recd <AP> / recp <x> <y> / rec <角度>   记录当前脉宽的角度样本
  list / drop            看样本 / 删最近一条
  fit                    拟合当前舵机，缓存结果
  save                   全部结果写 docs/data/ JSON + 打印 config 粘贴块
  h / q                  帮助 / 断电退出"""


def norm_deg(a):
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def fit_line(pairs):
    """最小二乘 θ = a·us + b。返回 (a, b, 最大残差)。"""
    n = len(pairs)
    sx = sum(u for u, _ in pairs)
    sy = sum(d for _, d in pairs)
    sxx = sum(u * u for u, _ in pairs)
    sxy = sum(u * d for u, d in pairs)
    den = n * sxx - sx * sx
    a = (n * sxy - sx * sy) / den
    b = (sy - a * sx) / n
    resid = max(abs(d - (a * u + b)) for u, d in pairs)
    return a, b, resid


class Session:
    def __init__(self, drv, real, l3):
        self.drv, self.real, self.l3 = drv, real, l3
        self.cur = {ch: CENTER_US for ch in range(NUM_SERVOS)}
        self.leg = None            # LegConfig
        self.joint = None          # "coxa"/"femur"/"tibia"
        self.samples = {}          # (leg, joint) -> [(us, deg)]
        self.fits = {}             # (leg, joint) -> dict
        self.base_az = None        # 基线方位角（俯视逆时针，度）
        self.hips = {}             # 腿名 -> (x, y)

    # ---- 运动 ----
    @property
    def channel(self):
        return getattr(self.leg, self.joint).channel

    def ramp(self, target):
        target = min(US_HI, max(US_LO, target))
        ch = self.channel
        while abs(self.cur[ch] - target) > 1e-6:
            step = max(-RAMP_STEP, min(RAMP_STEP, target - self.cur[ch]))
            self.cur[ch] += step
            self.drv.set_pulses_us(ch, [self.cur[ch]])
            if self.real:
                time.sleep(RAMP_DT)
        return target

    # ---- 记录 ----
    def key(self):
        return (self.leg.name, self.joint)

    def add(self, deg):
        us = self.cur[self.channel]
        self.samples.setdefault(self.key(), []).append((us, deg))
        n = len(self.samples[self.key()])
        print(f"  样本{n}: {us:.0f}µs -> {deg:.2f}°")

    def angle_from(self, cmd, args):
        if cmd == "rec":
            return float(args[0])
        if cmd == "rech":
            if self.joint != "femur":
                raise ValueError("rech 只用于 femur")
            dh = float(args[0])
            if abs(dh) > CFG.femur_len:
                raise ValueError(f"|Δh| 不能超过 femur 长 {CFG.femur_len}")
            return math.degrees(math.asin(dh / CFG.femur_len))
        if cmd == "recd":
            if self.joint != "tibia":
                raise ValueError("recd 只用于 tibia")
            ap = float(args[0])
            l2, l3 = CFG.femur_len, self.l3
            c = (l2 * l2 + l3 * l3 - ap * ap) / (2 * l2 * l3)
            if not -1.0 <= c <= 1.0:
                raise ValueError(f"AP={ap} 与 80/{l3} 构不成三角形")
            return 180.0 - math.degrees(math.acos(c))   # k = 180 - θ
        if cmd == "recp":
            if self.joint != "coxa":
                raise ValueError("recp 只用于 coxa")
            if self.base_az is None:
                raise ValueError("先 base x1 y1 x2 y2 记基线")
            hip = self.hips.get(self.leg.name)
            if hip is None:
                raise ValueError(f"先 hip x y 记 {self.leg.name} 的 coxa 投点")
            px, py = float(args[0]), float(args[1])
            az = math.degrees(math.atan2(py - hip[1], px - hip[0]))
            return norm_deg(az - self.base_az - 90.0 - self.leg.mount_angle_deg)
        raise ValueError(cmd)

    # ---- 拟合 ----
    def fit(self):
        pairs = self.samples.get(self.key(), [])
        if len(pairs) < 2:
            print("⚠ 至少 2 个样本（建议 3~4 个）")
            return
        du = max(u for u, _ in pairs) - min(u for u, _ in pairs)
        dd = max(d for _, d in pairs) - min(d for _, d in pairs)
        if du < 100 or dd < 15:
            print(f"⚠ 样本跨度太小（脉宽 {du:.0f}µs / 角度 {dd:.1f}°），"
                  "拉开到 ≥100µs 且 ≥15° 再拟合")
            return
        a, b, resid = fit_line(pairs)
        attach = a * CENTER_US + b
        us_m45 = CENTER_US - 45.0 / a
        us_p45 = CENTER_US + 45.0 / a
        self.fits[self.key()] = dict(
            channel=self.channel, attach_deg=round(attach, 2),
            us_m45=round(us_m45, 1), us_p45=round(us_p45, 1),
            slope_deg_per_us=round(a, 5), resid_deg=round(resid, 2),
            samples=pairs)
        print(f"  {self.leg.name}.{self.joint}: attach={attach:.2f}°  "
              f"us_m45={us_m45:.0f}  us_p45={us_p45:.0f}  "
              f"斜率 {a:.4f}°/µs  最大残差 {resid:.2f}°"
              + ("  ⚠ 残差>1.5°，建议复测" if resid > 1.5 else ""))

    def save(self):
        if not self.fits:
            print("还没有拟合结果")
            return
        ddir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            "..", "..", "docs", "data")
        path = os.path.normpath(os.path.join(
            ddir, f"calib_pm45_{time.strftime('%Y%m%d')}.json"))
        with open(path, "w") as f:
            json.dump({f"{l}.{j}": v for (l, j), v in self.fits.items()},
                      f, ensure_ascii=False, indent=1)
        print(f"已存 {path}\n--- config.py 粘贴块（中位=1500）---")
        for name in [l.name for l in CFG.legs]:
            lines = []
            for j in JOINTS:
                v = self.fits.get((name, j))
                if v:
                    lines.append(
                        f"{j}_cal=ServoCal(channel={v['channel']}, "
                        f"attach_deg={v['attach_deg']}, "
                        f"us_m45={v['us_m45']}, us_p45={v['us_p45']})")
            if lines:
                print(f"  # {name}\n  " + ",\n  ".join(lines) + ",")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    ap.add_argument("--l3", type=float, default=CFG.tibia_len,
                    help="tibia 有效长 K→唇口 (mm)，116/120 之争裁决后改这里")
    args = ap.parse_args()

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    s = Session(drv, not args.mock, args.l3)
    drv.set_all_pulses_us([CENTER_US] * NUM_SERVOS)
    drv.enable(True)
    print(f"全舵机 1500µs 使能。l3={args.l3}。腿必须全程悬空。\n{HELP}")

    try:
        while True:
            tag = (f"[{s.leg.name}.{s.joint} ch{s.channel} "
                   f"@{s.cur[s.channel]:.0f}] " if s.leg else "[未选舵机] ")
            try:
                line = input(tag + "> ").strip()
            except EOFError:
                break
            if not line:
                continue
            t = line.split()
            cmd = t[0].lower()
            try:
                if cmd in ("q", "quit"):
                    break
                elif cmd == "h":
                    print(HELP)
                elif cmd == "s":
                    if len(t) == 2 and t[1].isdigit():
                        ch = int(t[1])
                        s.leg, s.joint = next(
                            (l, j) for l in CFG.legs for j in JOINTS
                            if getattr(l, j).channel == ch)
                    else:
                        s.leg = CFG.leg(t[1].upper())
                        s.joint = {"c": "coxa", "f": "femur", "t": "tibia"}[
                            t[2][0].lower()]
                    print(f"  选中 {s.leg.name}.{s.joint} 通道 {s.channel}，"
                          f"当前 {s.cur[s.channel]:.0f}µs")
                elif cmd == "base":
                    x1, y1, x2, y2 = map(float, t[1:5])
                    s.base_az = math.degrees(math.atan2(y2 - y1, x2 - x1))
                    print(f"  基线方位 {s.base_az:.2f}°（纸坐标系）")
                elif cmd == "hip":
                    if not s.leg:
                        raise ValueError("先 s 选腿")
                    s.hips[s.leg.name] = (float(t[1]), float(t[2]))
                    print(f"  {s.leg.name} hip = {s.hips[s.leg.name]}")
                elif cmd in ("rec", "rech", "recd", "recp"):
                    if not s.leg:
                        raise ValueError("先 s 选舵机")
                    s.add(s.angle_from(cmd, t[1:]))
                elif cmd == "list":
                    for u, d in s.samples.get(s.key(), []):
                        print(f"  {u:.0f}µs -> {d:.2f}°")
                elif cmd == "drop":
                    if s.samples.get(s.key()):
                        print(f"  已删 {s.samples[s.key()].pop()}")
                elif cmd == "fit":
                    s.fit()
                elif cmd == "save":
                    s.save()
                elif cmd[0] in "+-0123456789":
                    if not s.leg:
                        raise ValueError("先 s 选舵机")
                    v = float(line)
                    tgt = s.cur[s.channel] + v if cmd[0] in "+-" else v
                    got = s.ramp(tgt)
                    if got != tgt:
                        print(f"  ⚠ 已限位到 {got:.0f}（软限 {US_LO:.0f}~{US_HI:.0f}）")
                else:
                    print("  ？未知命令，h 看帮助")
            except (ValueError, KeyError, IndexError, StopIteration) as e:
                print(f"  ⚠ {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if s.fits and s.real:
            s.save()             # 防手滑退出丢数据
        drv.close()
        print("\n已断舵机电。")


if __name__ == "__main__":
    main()
