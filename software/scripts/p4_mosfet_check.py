#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P4 气路电气(YYNMOS-8 + 6 阀 + 泵 A)接线自检(树莓派 5 上跑)。

对应图纸:
  html/p4-pneumatic-electrical.html   端子速查表 12~18 行(GPIO 分配)、图 3、调试顺序
  html/p4-pi-wiring.html              Pi 物理脚位与线色

干什么:逐路点动 GPIO,由你确认"**对的负载动了、且只有它动**",
把接错脚位、IN 串位、OUT 接错路、虚接这几类接线错误逐一定位。
带负载跑(阀咔哒/泵转)和空载跑(只看每路 LED 翻转)都行,判词一样。

安全边界:
  - 阀通电 = 吸盘通大气(断电才通真空)。所以**只能空载在台架上跑**,
    机器人吸在墙上时点动 = 放气 = 掉下来。
  - 泵每次只点动 0.8s(可调),膜片泵干转这点时间无碍。
  - 通道 8(泵 B)V0 整路空置,脚本不碰 GPIO26。
  - 退出时全部拉低并释放 GPIO。释放后 GPIO5/6 回到复位默认上拉,
    全靠端子上的 6.2k 下拉兜底——**退出后哪路 LED 还亮,就是哪路下拉有问题**。

用法:
  python3 scripts/p4_mosfet_check.py            # 逐路交互点动(通道 1~7)
  python3 scripts/p4_mosfet_check.py --ch 3     # 只测通道 3
  python3 scripts/p4_mosfet_check.py --sweep    # 不提问,1~7 连扫两轮,眼睛盯 LED 排
  python3 scripts/p4_mosfet_check.py --hold 7   # 通道 7 常开,配合万用表量,回车关断
  python3 scripts/p4_mosfet_check.py --list     # 只打印映射表,不碰硬件
"""
import argparse
import subprocess
import sys
import time

# ---- 通道映射:与 html/p4-pneumatic-electrical.html 速查表 12~15 行一致 ----
# 改这里必须同步改 hexapod/adhesion.py 的 VALVE_PINS / PUMP_PIN(反之亦然)
# (通道, IN 端子, GPIO(BCM), Pi 物理脚, 负载, 控制线色, 类别)
CHANNELS = [
    (1, "IN1+", 5,  29, "阀 L1", "绿", "valve"),
    (2, "IN2+", 6,  31, "阀 L2", "绿", "valve"),
    (3, "IN3+", 13, 33, "阀 L3", "绿", "valve"),
    (4, "IN4+", 16, 36, "阀 R1", "白", "valve"),
    (5, "IN5+", 19, 35, "阀 R2", "白", "valve"),
    (6, "IN6+", 21, 40, "阀 R3", "白", "valve"),
    (7, "IN7+", 20, 38, "泵 A", "紫", "pump"),
    (8, "IN8+", 26, 37, "泵 B", "紫", "spare"),   # V0 空置:线没装,GPIO26 不碰
]

VALVE_ON_S = 0.40       # 阀点动:通电时长
VALVE_OFF_S = 0.35      # 阀点动:两次之间的断电时长
PUMP_ON_S = 0.8         # 泵点动时长(--pump-secs 可改)


class MosfetIO:
    """lgpio 输出封装。所有非空置通道上电即驱动为低(全关)。"""
    GPIOCHIP = 4        # 树莓派 5 旧内核;新内核/其他板子落回 0

    def __init__(self):
        try:
            import lgpio
        except ImportError:
            sys.exit("缺 lgpio。树莓派上:sudo apt install python3-lgpio,"
                     "或 venv 里 pip install lgpio(本仓库:pip install -e '.[pi]')")
        self._lg = lgpio
        try:
            self._h = lgpio.gpiochip_open(self.GPIOCHIP)
        except Exception:
            self._h = lgpio.gpiochip_open(0)
        self._pins = []
        for _n, _t, gpio, _p, name, _c, kind in CHANNELS:
            if kind == "spare":
                continue
            try:
                lgpio.gpio_claim_output(self._h, gpio, 0)
            except Exception as e:
                self.close()
                sys.exit(f"占不到 GPIO{gpio}({name}):{e}\n"
                         "是不是别的程序(adhesion / 步态)还在跑?先停掉它。")
            self._pins.append(gpio)

    def write(self, gpio, level):
        self._lg.gpio_write(self._h, gpio, level)

    def all_off(self):
        for p in self._pins:
            self._lg.gpio_write(self._h, p, 0)

    def close(self):
        try:
            self.all_off()
        except Exception:
            pass
        self._lg.gpiochip_close(self._h)


def ask(prompt):
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return "q"


def print_table():
    print("  通道  IN端子  GPIO    物理脚  负载    线色   备注")
    print("  " + "-" * 56)
    for num, term, gpio, pin, name, color, kind in CHANNELS:
        note = "V0 空置,不测" if kind == "spare" else ""
        print(f"   {num}    {term:5s} GPIO{gpio:<3d} Pin{pin:<4d} {name:5s}  {color}     {note}")
    print()


def pulse(io, ch, pump_secs):
    """点动一路:阀两次通断(咔哒两组),泵单次短转。"""
    _n, _t, gpio, _p, _name, _c, kind = ch
    if kind == "pump":
        io.write(gpio, 1)
        time.sleep(pump_secs)
        io.write(gpio, 0)
    else:
        for _ in range(2):
            io.write(gpio, 1)
            time.sleep(VALVE_ON_S)
            io.write(gpio, 0)
            time.sleep(VALVE_OFF_S)


def diagnose_no_action(ch):
    """负载没动 → 追问 LED,把故障切到 IN 侧或 OUT 侧。返回结果码。"""
    num, term, gpio, pin, name, _c, kind = ch
    led = ask("       └ 刚才该路 LED(板上通道指示灯)亮了吗?[y/n/没看清=回车重放] ")
    if led not in ("y", "n"):
        return "retry"
    if led == "y":
        print(f"       ❌ LED 亮而负载不动 → 问题在 OUT 侧:")
        if kind == "pump":
            print(f"          · 泵「−」是否拧进了 OUT7−(速查表第 10 行)")
            print(f"          · 泵「+」是否接在泵轨(XL6009 #1 OUT+),该轨保险/接头是否到位")
        else:
            print(f"          · {name} 两根线是否拧紧在 OUT{num}+ / OUT{num}−(会不会串到隔壁路)")
            print(f"          · 断电量阀线圈电阻,几十~几百 Ω 为正常,∞ = 线圈/线断")
        return "fail-out"
    print(f"       ❌ LED 也不亮 → 问题在 IN 侧:")
    print(f"          · Pi Pin{pin}(GPIO{gpio})→ {term} 这根线断/插错脚位(对照速查表 12~14 行)")
    print(f"          · 该路 IN− 跳线没并进信号地母线(7 根短跳线缺了它,速查表第 16 行)")
    print(f"          · 信号地总线 Pin39 → IN− 那根断了(那样应该是 8 路全不亮)")
    print(f"          · 6.2k 下拉误装成小阻值把信号分掉了(量 {term} 对 IN{num}− 点动时应 ≈3.3V)")
    return "fail-in"


def test_channel(io, ch, pump_secs):
    num, term, gpio, pin, name, _c, kind = ch
    expect = ("泵应短转一下(约 %.1fs)" % pump_secs) if kind == "pump" \
        else "该阀应咔哒两组(通/断电各一响),LED 同步闪两下"
    print(f"\n  ▶ 通道 {num}  {name}  (GPIO{gpio} / Pin{pin} → {term})   {expect}")
    while True:
        pulse(io, ch, pump_secs)
        a = ask("     结果?[回车=对的负载动了且只有它 / n=没动 / o=动的不对或不止一路 / "
                "r=重放 / s=跳过 / q=退出] ")
        if a in ("", "y"):
            print("     ✅ 通过")
            return "ok"
        if a == "r":
            continue
        if a == "s":
            return "skip"
        if a == "q":
            return "quit"
        if a == "o":
            other = ask("       └ 实际动的是哪个负载?(随手记一下,回车跳过) ")
            print("       ❌ 串位:这一路的 IN 线(Pi 端或板端)或 OUT 负载线接到了别的通道。")
            if other:
                print(f"          实际动作:{other} —— 对照速查表把两路对调即可。")
            print("          注意串位一般成对出现,后面测到对家那路会再错一次,一起换。")
            return "cross"
        if a == "n":
            r = diagnose_no_action(ch)
            if r == "retry":
                continue
            return r


def check_throttled():
    """调试顺序最后一步:vcgencmd get_throttled 应回 0x0。"""
    try:
        out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                             text=True, timeout=5).stdout.strip()
        val = int(out.split("=")[1], 16)
    except (FileNotFoundError, subprocess.TimeoutExpired, IndexError, ValueError):
        print("  (vcgencmd 不可用,跳过 Pi 供电健康检查)")
        return
    if val == 0:
        print("  ✅ get_throttled = 0x0,点动全程 Pi 供电健康")
        return
    print(f"  ⚠ get_throttled = {out.split('=')[1]}")
    if val & 0x1:
        print("     bit0:**此刻正欠压** —— Pi 的 5V 源被拖垮,先查供电")
    if val & 0xF0000:
        print("     高位:开机以来出现过欠压/降频。若恰好发生在泵点动瞬间,"
              "说明泵浪涌串扰到了 Pi 供电,查共地走线与 5V 来源")


def run_interactive(io, only_ch, pump_secs):
    targets = [c for c in CHANNELS if c[6] != "spare" and (only_ch is None or c[0] == only_ch)]
    if not targets:
        sys.exit(f"通道 {only_ch} 不可测(1~7;通道 8 V0 空置)。")

    print("开始前确认三件事:")
    print("  ① 12V 已上电(两块 XL6009 输出正常)")
    print("  ② 机器人空载在台架上——阀通电会把吸盘放到大气,在墙上跑等于松手")
    print("  ③ 现在所有通道都已被驱动为低:8 颗 LED 应全灭、无阀嗡嗡响、泵不转")
    a = ask("  是这样吗?[回车=是 / n=不是] ")
    if a == "n":
        print("\n  ❌ 静态下就有通道导通,先修再点动。逐条排查亮/响的那路:")
        print("     · IN+ 线是否插错到 3.3V/5V 脚(对照速查表 12~15 行的物理脚位)")
        print("     · 该路 IN+ / IN− 是否接反")
        print("     · 万用表量该路 INn+ 对 INn−:此刻应 ≈0V")
        sys.exit(1)

    results = {}
    for ch in targets:
        r = test_channel(io, ch, pump_secs)
        if r == "quit":
            print("\n  已退出,全部通道拉低。")
            break
        results[ch[0]] = r

    # ---- 小结 ----
    print("\n" + "=" * 60)
    print("  小结")
    print("=" * 60)
    tag = {"ok": "✅ 通过", "skip": "○ 跳过", "cross": "❌ 串位",
           "fail-in": "❌ IN 侧不通", "fail-out": "❌ OUT 侧不通"}
    for ch in targets:
        num, _t, gpio, _p, name, _c, _k = ch
        if num in results:
            print(f"   通道 {num}  {name:5s} (GPIO{gpio:<2d})  {tag[results[num]]}")
    bad = [n for n, r in results.items() if r.startswith("fail") or r == "cross"]
    print()
    check_throttled()

    if bad:
        print(f"\n  ❌ 有问题的通道:{'、'.join(map(str, bad))} —— 断电按上面的判词逐条修,"
              "修完重跑:")
        print("     python3 scripts/p4_mosfet_check.py --ch " +
              " / --ch ".join(map(str, bad)))
        return 1
    if len(results) < len(targets) or any(r == "skip" for r in results.values()):
        print("\n  ○ 还有通道没测完/被跳过,补完再收口。")
        return 1
    print("\n  ✅ 点动全过。剩下的收口动作:")
    print("     1. 泵抽真空实测:python3 scripts/p4_sensor_check.py --live")
    print("        (泵转、罐压/足压往负走,气路方向同时确认:断电吸真空、通电放大气)")
    print("     2. 一项软件测不出来的,断电用万用表补:泵「+」对板 DC+ 应**不导通**。")
    print("        导通 = OUT7+ 误接,泵被并回阀轨,分轨失效(速查表第 11 行)")
    print("     3. 全过后回填 hexapod/adhesion.py:")
    print("        VALVE_PINS = [5, 6, 13, 16, 19, 21]   # L1 L2 L3 R1 R2 R3")
    print("        PUMP_PIN   = 20                       # 不变")
    return 0


def run_sweep(io, rounds, pump_secs):
    """不提问连扫:眼睛盯着 LED 排/听声音,快速过一遍。"""
    targets = [c for c in CHANNELS if c[6] != "spare"]
    print(f"连扫 {rounds} 轮,顺序通道 1→7。盯住 LED 排:应逐颗依次闪、一次只亮一颗。\n")
    for r in range(1, rounds + 1):
        for ch in targets:
            num, _t, gpio, _p, name, _c, kind = ch
            print(f"  第 {r} 轮  通道 {num}  {name}  (GPIO{gpio})")
            if kind == "pump":
                io.write(gpio, 1); time.sleep(pump_secs); io.write(gpio, 0)
            else:
                io.write(gpio, 1); time.sleep(VALVE_ON_S); io.write(gpio, 0)
            time.sleep(0.4)
    print("\n扫完。哪路 LED 没闪、闪错位、或负载没跟着动,就单测那路:--ch N")


def run_hold(io, chnum):
    ch = next((c for c in CHANNELS if c[0] == chnum), None)
    if ch is None or ch[6] == "spare":
        sys.exit(f"通道 {chnum} 不可测(1~7;通道 8 V0 空置)。")
    num, term, gpio, _p, name, _c, kind = ch
    print(f"通道 {num}({name})常开。万用表预期:")
    print(f"  · 黑笔功率地(DC−)、红笔 OUT{num}−:导通中 ≈0V,关断后经负载回到 ≈12V")
    print(f"  · {term} 对 IN{num}−:≈3.3V")
    if kind == "pump":
        print("  · 泵持续转。别挂着不管,量完就关。")
    else:
        print("  · 阀保持通电(吸盘侧通大气)。线圈会发热,量完就关。")
    io.write(gpio, 1)
    try:
        ask("\n[回车] 关断并退出 ")
    finally:
        io.write(gpio, 0)


def main():
    ap = argparse.ArgumentParser(description="P4 YYNMOS-8 + 阀/泵 接线自检")
    ap.add_argument("--ch", type=int, metavar="N", help="只测通道 N(1~7)")
    ap.add_argument("--sweep", action="store_true", help="不提问连扫两轮,肉眼看 LED")
    ap.add_argument("--rounds", type=int, default=2, help="--sweep 扫几轮(默认 2)")
    ap.add_argument("--hold", type=int, metavar="N", help="通道 N 常开配合万用表,回车关断")
    ap.add_argument("--pump-secs", type=float, default=PUMP_ON_S,
                    help=f"泵每次点动秒数(默认 {PUMP_ON_S})")
    ap.add_argument("--list", action="store_true", help="只打印通道映射表,不碰硬件")
    a = ap.parse_args()

    print("=" * 60)
    print("P4 气路电气接线自检(YYNMOS-8 + 6 阀 + 泵 A)")
    print("=" * 60)
    print_table()
    if a.list:
        return 0

    io = MosfetIO()
    code = 0
    try:
        if a.hold is not None:
            run_hold(io, a.hold)
        elif a.sweep:
            run_sweep(io, a.rounds, a.pump_secs)
        else:
            code = run_interactive(io, a.ch, a.pump_secs)
    except KeyboardInterrupt:
        print("\n中断,全部通道拉低。")
        code = 130
    finally:
        io.close()
    print("\nGPIO 已释放(GPIO5/6 回到复位上拉)。再看一眼板子:此刻哪路 LED 还亮,"
          "就是哪路的 6.2k 下拉没兜住,查它。")
    return code


if __name__ == "__main__":
    sys.exit(main())
