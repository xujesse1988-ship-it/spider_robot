#!/usr/bin/env python3
"""六足空中舞蹈：身体架在垫块上、六脚全部悬空时的编排动作。

前提：把机器人身体搁在一个垫块（书堆/泡沫砖/纸箱）上，六条腿全部离地悬空。
这样腿不承重、不受支撑多边形约束，可以大幅快速地舞动，也不会翻倒。

足端用每条腿自己的极坐标描述，六腿对称，编排代码很短：
  out   径向——沿腿中性方向往外伸为正（伸展/蜷缩）
  tang  切向——绕髋轴水平摆，逆时针为正（前后划）
  up    垂直——抬起为正（拍打/人浪）

悬空带来两个新风险，--check 会逐帧算：
  · 踢地——足端最低点决定垫块至少要多高，脚本直接把这个数算给你；
  · 撞腿——相邻腿大幅切向摆会打架，逐帧检查所有腿对的膝点/足端间距。

用法:
  python dance.py --check              # 只算不动：IK/垫块高度/腿间距/脉宽，先跑这个
  python dance.py --check --prop 130   # 已知垫块高 130mm，校验够不够
  python dance.py --mock               # 无硬件按真实节奏干跑
  python dance.py --list               # 列出动作
  python dance.py --move ripple        # 单跑一个动作
  python dance.py --prop 130           # 真机（会先确认身体已架空）
  python dance.py --bpm 112 --scale 0.6
"""
import argparse
import math
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver
from hexapod.kinematics import WorkspaceError, leg_joint_points

# ---- 节奏 ----
BPM = 104.0
BRIDGE_BEATS = 0.8    # 动作之间的过渡时长（拍）

# ---- 悬空中位姿态（相对髋轴平面）----
REACH0 = 125.0        # 足端到髋轴的水平距离 mm（站立默认 130）。别调太小：腿越收，
                      # 抬腿时膝盖折得越狠——115 时 bounce 就能把膝弯顶到 143°
Z0 = -75.0            # 足端垂直位置 mm（负 = 髋轴平面以下）

# ---- 幅度（--scale 统一缩放）----
OUT = 55.0            # 径向伸缩幅度 mm
TANG = 65.0           # 切向摆幅 mm
UP = 60.0             # 垂直摆幅 mm
CURL_IN = 55.0        # 蜷缩时收回量 mm
SWIM_BIAS = 25.0      # swim 画圆的圆心外偏 mm——不外偏的话圆会扫进"腿收到最里
                      # 又抬高"的角落，膝盖折到 157°，小腿有撞大腿的风险
SWIM_UP = 10.0        # swim 圆心上抬 mm——整套里踩得最低的就是它，抬一点少要垫块高度

# ---- 安全余量 ----
GROUND_CLEAR = 20.0   # 足端离地最小余量 mm（决定垫块高度要求）
LEG_GAP = 45.0        # 腿之间最小间距 mm（吸盘足 30mm + 余量）
KNEE_MAX = 145.0      # 膝弯角上限 deg（站立时 108）——再大小腿要贴上大腿了
KNEE_MIN = 35.0       # 膝弯角下限 deg——再小整条腿接近完全伸直
POWER_S = 0.5

# 绕身体一圈的顺序（左前->左后->右后->右前），人浪/追逐用
RING = ("L1", "L2", "L3", "R3", "R2", "R1")
LEFT = ("L1", "L2", "L3")
RIGHT = ("R1", "R2", "R3")
TRI_A = ("L1", "R2", "L3")
TRI_B = ("R1", "L2", "R3")
# 扑腾用的固定相位错开（写死而不是随机，保证每次跑一样、check 有意义）
FLUTTER_PH = {"L1": 0.00, "L2": 0.37, "L3": 0.71, "R1": 0.18, "R2": 0.55, "R3": 0.89}


def _ss(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3 - 2 * u)


def _env(u, r=0.22):
    """幅度包络 0->1->0，中段满幅。动作起收都不突兀。"""
    if u < r:
        return _ss(u / r)
    if u > 1 - r:
        return _ss((1 - u) / r)
    return 1.0


def _raw_us(cal, joint_deg):
    """未夹紧的脉宽——joint_deg_to_us 撞 min_us/max_us 时是静默夹住的，这里要看真值。"""
    servo_deg = cal.sign * (joint_deg - cal.attach_deg)
    center = (cal.us_m45 + cal.us_p45) / 2
    return center + servo_deg * (cal.us_p45 - cal.us_m45) / 90.0


class Dancer:
    """按拍驱动六条悬空腿的动作播放器。

    动作是 fn(u) -> {腿名: (out, tang, up)}，u 为该动作内 0..1 的进度；
    缺的腿自动补中位。极坐标 -> 身体系的换算在 self.foot 里。
    """

    def __init__(self, bot, bpm=BPM, scale=1.0, check=False, z0=Z0):
        self.bot = bot
        self.beat = 60.0 / bpm
        self.k = scale
        self.check = check
        self.z0 = z0
        self.dt = 1.0 / bot.cfg.update_hz
        self.t = 0.0
        self.dirs = {}          # 腿名 -> (mount_x, mount_y, cos a, sin a)
        for leg in bot.cfg.legs:
            a = math.radians(leg.mount_angle_deg)
            self.dirs[leg.name] = (leg.mount_x, leg.mount_y, math.cos(a), math.sin(a))
        self.home = self.feet({})
        self.last = dict(self.home)
        self._last_power = -1e9
        self.peak_a = 0.0
        # 干跑统计
        self.span = {}          # 通道 -> [us_lo, us_hi, deg_lo, deg_hi]
        self.z_min = 0.0        # 足端最低点（相对髋轴平面）
        self.z_at = ""
        self.gap_min = 1e9      # 最小腿间距
        self.gap_at = ""
        self.frames = 0

    # ---------- 坐标 ----------
    def foot(self, name, out=0.0, tang=0.0, up=0.0):
        mx, my, ca, sa = self.dirs[name]
        r = REACH0 + out
        return (mx + r * ca - tang * sa,
                my + r * sa + tang * ca,
                self.z0 + up)

    def feet(self, moves):
        """{腿名: (out,tang,up)} -> 身体系足端目标；未给的腿回中位。"""
        out = {}
        for name in self.dirs:
            o, t, u = moves.get(name, (0.0, 0.0, 0.0))
            out[name] = self.foot(name, o, t, u)
        return out

    # ---------- 一帧 ----------
    def frame(self, feet):
        if self.check:
            self._audit(feet)
        else:
            self.bot.move_feet(feet)
        self.last = feet
        self.frames += 1

    def _audit(self, feet):
        """IK + 脉宽 + 触地 + 腿间距。IK 无解直接抛 WorkspaceError。"""
        angles = self.bot.joint_angles(feet)
        pts = {}
        for name, (g, a, th) in angles.items():
            leg = self.bot.cfg.leg(name)
            for cal, deg in ((leg.coxa, math.degrees(g)),
                             (leg.femur, math.degrees(a)),
                             (leg.tibia, 180.0 - math.degrees(th))):
                us = _raw_us(cal, deg)
                s = self.span.setdefault(cal.channel, [us, us, deg, deg])
                s[0], s[1] = min(s[0], us), max(s[1], us)
                s[2], s[3] = min(s[2], deg), max(s[3], deg)
            if feet[name][2] < self.z_min:
                self.z_min, self.z_at = feet[name][2], name
            # 膝点/足端转到身体系，供撞腿检查
            mx, my, ca, sa = self.dirs[name]
            pts[name] = [(mx + px * ca - py * sa, my + px * sa + py * ca, pz)
                         for px, py, pz in leg_joint_points(self.bot.cfg, g, a, th)[2:]]
        names = list(pts)
        for i, n1 in enumerate(names):
            for n2 in names[i + 1:]:
                for p in pts[n1]:
                    for q in pts[n2]:
                        d = math.dist(p, q)
                        if d < self.gap_min:
                            self.gap_min, self.gap_at = d, f"{n1}-{n2}"

    def tick(self):
        self.t += self.dt
        if self.check:
            return
        time.sleep(self.dt)
        if self.t - self._last_power > POWER_S:
            v, c = self.bot.check_power()       # 欠压直接抛异常停机
            self.peak_a = max(self.peak_a, c)
            warn = " ⚠过流" if c > self.bot.cfg.curr_warn else ""
            print(f"\r  电压 {v:.2f}V  电流 {c:5.2f}A  峰值 {self.peak_a:5.2f}A{warn}   ",
                  end="", flush=True)
            self._last_power = self.t

    # ---------- 播放 ----------
    def run(self, beats, fn, name=""):
        f0 = self.feet(fn(0.0))
        if any(math.dist(self.last[n], f0[n]) > 5.0 for n in f0):
            self._bridge(f0)
        n = max(2, int(round(beats * self.beat / self.dt)))
        for i in range(n + 1):
            u = i / n
            try:
                self.frame(self.feet(fn(u)))
            except WorkspaceError as e:
                raise WorkspaceError(f"[{name}] 第 {u * beats:.1f} 拍 IK 无解：{e}") from None
            self.tick()

    def _bridge(self, target):
        """平滑过渡到目标首帧——动作衔接与单跑都靠它，避免舵机瞬跳。"""
        a = self.last
        n = max(2, int(round(BRIDGE_BEATS * self.beat / self.dt)))
        for i in range(1, n + 1):
            s = _ss(i / n)
            self.frame({k: tuple(p + (q - p) * s for p, q in zip(a[k], target[k]))
                        for k in target})
            self.tick()


# ================= 动作 =================
# 每个动作返回 {腿名: (out, tang, up)}，未列出的腿停在中位。

def spread(d, beats=6):
    """伸懒腰：六腿同时向外伸展到最远再收回，像花开合。"""
    def f(u):
        e = math.sin(math.pi * u) * d.k
        return {n: (OUT * e, 0.0, UP * 0.25 * e) for n in d.dirs}
    d.run(beats, f, "spread")


def ripple(d, beats=12, rounds=3):
    """人浪：一道波绕身体转圈，波峰的腿抬得最高、伸得最远。"""
    m = len(RING)

    def f(u):
        pos = u * rounds * m
        mv = {}
        for i, n in enumerate(RING):
            ph = (pos - i) % m
            w = math.sin(math.pi * ph) if ph < 1.0 else 0.0     # 只在波峰经过时抬
            mv[n] = (OUT * 0.5 * w * d.k, 0.0, UP * 1.1 * w * d.k)
        return mv
    d.run(beats, f, "ripple")


def swim(d, beats=8, cycles=3):
    """划水：每条腿在自己的径向-垂直平面里画圆，六腿同相，像一起爬行。

    圆心整体外偏 SWIM_BIAS，否则圆的左上象限会把膝盖折到 150° 以上。
    """
    def f(u):
        a = 2 * math.pi * cycles * u
        e = _env(u) * d.k
        return {n: (SWIM_BIAS * e + OUT * 0.8 * e * math.cos(a),
                    0.0, SWIM_UP * e + UP * 0.8 * e * math.sin(a))
                for n in d.dirs}
    d.run(beats, f, "swim")


def scissor(d, beats=8, cycles=4):
    """剪刀：左三腿与右三腿切向反相前后摆，同时上下错开，交叉感很强。"""
    def f(u):
        a = 2 * math.pi * cycles * u
        e = _env(u) * d.k
        mv = {}
        for n in d.dirs:
            s = 1.0 if n in LEFT else -1.0
            mv[n] = (0.0, TANG * e * math.sin(a) * s, UP * 0.45 * e * math.cos(a) * s)
        return mv
    d.run(beats, f, "scissor")


def pinwheel(d, beats=8, cycles=2):
    """风车：六腿切向同向大幅扫，身体不动但看着像在旋转。"""
    def f(u):
        a = 2 * math.pi * cycles * u
        e = _env(u) * d.k
        return {n: (0.0, TANG * 1.15 * e * math.sin(a), UP * 0.3 * e * math.cos(a))
                for n in d.dirs}
    d.run(beats, f, "pinwheel")


def chase(d, beats=10, rounds=2):
    """追逐：一个虚拟目标点绕身体转，离它最近的腿伸出去够，其余收着。"""
    m = len(RING)

    def f(u):
        pos = u * rounds * m
        mv = {}
        for i, n in enumerate(RING):
            dph = (pos - i + m / 2) % m - m / 2          # 到目标的环上距离 [-3,3)
            w = max(0.0, 1.0 - abs(dph))                 # 只有最近的一两条腿响应
            s = _ss(w)
            mv[n] = (OUT * 1.0 * s * d.k,
                     -TANG * 0.6 * dph / m * 4 * s * d.k,   # 朝目标方向偏一点
                     UP * 0.5 * s * d.k)
        return mv
    d.run(beats, f, "chase")


def flutter(d, beats=8, cycles=6):
    """扑腾：高频小幅上下抖，六腿相位固定错开，像振翅。"""
    def f(u):
        e = _env(u) * d.k
        mv = {}
        for n, ph in FLUTTER_PH.items():
            a = 2 * math.pi * (cycles * u + ph)
            mv[n] = (OUT * 0.15 * e * math.cos(a), 0.0, UP * 0.5 * e * math.sin(a))
        return mv
    d.run(beats, f, "flutter")


def wiggle(d, beats=6, cycles=8):
    """动手指：六腿小幅高频切向抖动，蜘蛛感最强的一个动作。"""
    def f(u):
        e = _env(u) * d.k
        mv = {}
        for i, n in enumerate(RING):
            a = 2 * math.pi * (cycles * u + i / len(RING))
            mv[n] = (0.0, TANG * 0.4 * e * math.sin(a), UP * 0.12 * e * math.cos(a))
        return mv
    d.run(beats, f, "wiggle")


def typewriter(d, beats=8, hits=12):
    """点戳：六腿按环序依次快速下压回弹，像手指敲桌面。"""
    m = len(RING)

    def f(u):
        k = u * hits
        i = int(k) % m
        ph = k % 1.0
        n = RING[i]
        return {n: (OUT * 0.2 * d.k * math.sin(math.pi * ph),
                    0.0, -UP * 0.45 * d.k * math.sin(math.pi * ph))}
    d.run(beats, f, "typewriter")


def bounce(d, beats=8, hits=8):
    """踩点：三角两组交替上下，全套里最踩拍子的动作。

    抬起的同时往外送一点——腿是绕髋轴转的，不外送的话抬得越高膝盖折得越狠。
    """
    def f(u):
        k = u * hits
        grp = TRI_A if int(k) % 2 == 0 else TRI_B
        h = math.sin(math.pi * (k % 1.0))
        return {n: (OUT * 0.3 * d.k * h if n in grp else 0.0, 0.0,
                    (UP * 0.85 * d.k * h if n in grp else -UP * 0.25 * d.k * h))
                for n in d.dirs}
    d.run(beats, f, "bounce")


def curl(d, beats=8):
    """蜷缩：六腿全部收到最里侧屏住，再展开——很有戏剧性的收尾前铺垫。"""
    def f(u):
        e = math.sin(math.pi * u) ** 0.6 * d.k      # 收进去后保持一会儿再放
        return {n: (-CURL_IN * e, 0.0, -UP * 0.2 * e) for n in d.dirs}
    d.run(beats, f, "curl")


def home(d, beats=3):
    """回悬空中位。"""
    d.run(beats, lambda u: {}, "home")


# ================= 编排 =================
MOVES = {
    "spread": lambda d: spread(d, 6),
    "ripple": lambda d: ripple(d, 12, 3),
    "swim": lambda d: swim(d, 8, 3),
    "scissor": lambda d: scissor(d, 8, 4),
    "pinwheel": lambda d: pinwheel(d, 8, 2),
    "chase": lambda d: chase(d, 10, 2),
    "flutter": lambda d: flutter(d, 8, 6),
    "wiggle": lambda d: wiggle(d, 6, 8),
    "typewriter": lambda d: typewriter(d, 8, 12),
    "bounce": lambda d: bounce(d, 8, 8),
    "curl": lambda d: curl(d, 8),
    "home": lambda d: home(d, 3),
}

ROUTINE = ["spread", "ripple", "swim", "scissor", "pinwheel", "chase",
           "flutter", "wiggle", "typewriter", "bounce", "curl", "spread", "home"]


def report(d, prop):
    need = -d.z_min + GROUND_CLEAR
    print(f"\n干跑 {d.frames} 帧，约 {d.frames * d.dt:.1f}s。\n")
    print("  通道  关节         关节角范围        脉宽范围         软限余量")
    print("  " + "-" * 62)
    print("        （coxa=水平摆角  femur=大腿仰角  tibia=膝弯角，站立时 108°）")
    by_ch = {}
    for leg in d.bot.cfg.legs:
        for jn, cal in (("coxa", leg.coxa), ("femur", leg.femur), ("tibia", leg.tibia)):
            by_ch[cal.channel] = (f"{leg.name}.{jn}", cal)
    bad = []
    for ch in sorted(d.span):
        lo_us, hi_us, lo_d, hi_d = d.span[ch]
        label, cal = by_ch[ch]
        margin = min(lo_us - cal.min_us, cal.max_us - hi_us)
        flag = ""
        if margin < 0:
            flag, _ = "  ✗ 撞限位", bad.append((label, ch, margin))
        elif margin < 150:
            flag = "  ⚠ 余量小"
        print(f"  {ch:>4}  {label:<11}  {lo_d:>7.1f}~{hi_d:>6.1f}°  "
              f"{lo_us:>7.0f}~{hi_us:>7.0f}µs  {margin:>7.0f}µs{flag}")

    ok = True
    knees = [d.span[leg.tibia.channel] for leg in d.bot.cfg.legs
             if leg.tibia.channel in d.span]
    k_lo, k_hi = min(s[2] for s in knees), max(s[3] for s in knees)
    print(f"\n  膝弯角范围  {k_lo:.1f}~{k_hi:.1f}°（站立 108°，阈值 "
          f"{KNEE_MIN:.0f}~{KNEE_MAX:.0f}°）")
    if k_hi > KNEE_MAX or k_lo < KNEE_MIN:
        print("  ⚠ 超出阈值——小腿可能撞大腿。这个阈值是保守估计，不是实测值：")
        print("    断电手动把膝盖掰到极限量一下，确认真实可弯范围再决定要不要放宽。")
        ok = False
    print(f"\n  足端最低点  {d.z_min:.1f}mm（髋轴平面以下，{d.z_at}）")
    print(f"  垫块高度要求 ≥ {need:.0f}mm —— 指髋轴（coxa 转轴）平面离地，"
          f"含 {GROUND_CLEAR:.0f}mm 余量")
    if prop:
        if prop >= need:
            print(f"  你的垫块 {prop:.0f}mm ✓ 富余 {prop - need:.0f}mm")
        else:
            print(f"  你的垫块 {prop:.0f}mm ✗ 差 {need - prop:.0f}mm，会踢地——垫高或用 --scale 缩幅度")
            ok = False
    print(f"\n  最小腿间距  {d.gap_min:.1f}mm（{d.gap_at}，阈值 {LEG_GAP:.0f}mm）")
    if d.gap_min < LEG_GAP:
        print(f"  ✗ 相邻腿可能打架 —— 缩小 TANG 或用 --scale")
        ok = False
    if bad:
        print("\n✗ 以下通道会被静默夹到软限位，实际姿态和期望不符：")
        for label, ch, m in bad:
            print(f"    {label}(ch{ch}) 超出 {-m:.0f}µs —— 用 --scale 缩小幅度")
        ok = False
    if ok:
        print("\n✓ 全部帧 IK 可解，无撞限位、无踢地、无撞腿。")
        print("  提醒：软限位只是 500~2500µs，不代表机械上不会撞结构。")
        print("  第一次上真机用 --scale 0.5 起步，看着腿逐步加大。")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件按真实节奏干跑")
    ap.add_argument("--check", action="store_true", help="只算不动：安全校验报告")
    ap.add_argument("--list", action="store_true", help="列出所有动作")
    ap.add_argument("--move", action="append", help="只跑指定动作（可多次）")
    ap.add_argument("--prop", type=float, default=0.0,
                    help="垫块把身体垫起后，髋轴平面离地高度 mm（校验会不会踢地）")
    ap.add_argument("--z0", type=float, default=Z0,
                    help=f"悬空中位的足端高度 mm，默认 {Z0:.0f}；垫块矮就往上调（如 -55）")
    ap.add_argument("--bpm", type=float, default=BPM)
    ap.add_argument("--scale", type=float, default=1.0, help="幅度缩放，0.5 = 打对折")
    ap.add_argument("--loop", type=int, default=1, help="整套重复几遍")
    ap.add_argument("--yes", action="store_true", help="跳过架空确认")
    args = ap.parse_args()

    if args.list:
        for n in MOVES:
            print(f"  {n}")
        return 0

    names = args.move or ROUTINE
    unknown = [n for n in names if n not in MOVES]
    if unknown:
        print(f"未知动作 {unknown}，--list 看全部")
        return 2

    drv = MockDriver() if (args.mock or args.check) else Servo2040Driver(args.port)
    bot = Hexapod(drv)
    d = Dancer(bot, bpm=args.bpm, scale=args.scale, check=args.check, z0=args.z0)

    if args.check:
        try:
            for _ in range(args.loop):
                for n in names:
                    MOVES[n](d)
            home(d)
        except WorkspaceError as e:
            print(f"\n✗ {e}\n  用 --scale 缩小幅度，或改小文件顶部的幅度常量。")
            return 1
        return 0 if report(d, args.prop) else 1

    if not args.mock and not args.yes:
        if input("身体已架在垫块上、六只脚全部悬空了吗？[y/N] ").strip().lower() != "y":
            print("先架空再跑——腿在地上乱甩会把机器人掀翻。")
            return 1

    # 先把中位脉宽发好再使能，避免上电乱跳
    bot.move_feet(d.home)
    drv.enable(True)
    bot.glide_to(d.home, 1.5)
    d.last = dict(d.home)

    print(f"♪ {args.bpm:.0f} BPM，幅度 ×{args.scale:.2f}，"
          f"{len(names)} 个动作 ×{args.loop} 遍。Ctrl-C 随时停。")
    undervolt = False
    try:
        for _ in range(args.loop):
            for n in names:
                print(f"\n▶ {n}")
                MOVES[n](d)
    except KeyboardInterrupt:
        print("\n中断。")
    except RuntimeError as e:              # check_power 欠压
        undervolt = True
        print(f"\n⚠ {e}")
    finally:
        try:
            if not undervolt:              # 欠压就别再动了，直接断电
                home(d, 2)
        except (KeyboardInterrupt, RuntimeError):
            pass
        drv.close()
        print(f"\n已断舵机电（腿会垂下，身体还在垫块上）。峰值电流 {d.peak_a:.2f}A。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
