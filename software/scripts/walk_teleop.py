#!/usr/bin/env python3
"""键盘遥控行走（SSH 到树莓派运行）。

  w/s  前进/后退      a/d  左移/右移
  q/e  左转/右转      空格 停
  1/2  三角/波浪步态   v    切阀策略 auto→on→off（对照用）
  m    进/出原地踏步   ESC  退出

用法: python walk_teleop.py [--port /dev/ttyACM0] [--mock] [--vent auto|on|off]
                            [--march-lift 70]

原地踏步 m（手动逐脚）：按 m 进入，机器人保持站位不动，六只脚各是一个开关——
按一下抬起、再按踩下，抬哪只、抬多久全看人按键；再按 m 回到普通遥控（可以走路
或摆别的姿势）。按键照俯视布局，键盘上就是一个 2×3 块：
    t=L1 左前   y=R1 右前
    g=L2 左中   h=R2 右中
    b=L3 左后   n=R3 右后
抬起高度 --march-lift 默认 70mm（步行抬脚才 40mm，这里特意抬高一截；再高要当心
femur 过竖直撞身体，改之前先架空试）。抬落 0.5s 走完、两端速度为零不甩腿，抬到
一半再按会平滑折返。
**踏步状态下只认这六个键 + m（出去）+ ESC（退出程序）**：w/s/a/d/q/e、空格、
1/2、v 一律忽略，免得手一抖在单脚悬空时把身体走了。出踏步时抬着的脚先滑回站位
再交还遥控，不会砸地。同时抬 4 只以上会有警告——三条腿撑不住整机，自己掂量。
踏步全程六阀通电排气（吸盘通大气）：身体没走不等于脚不抬，断了电脚就被被动真空
吸住。按 m 时若阀策略是 off 会自动切 on 并提示（踏步中 v 键不生效，要看被吸住的
对照，先出踏步再按 v 切 off）。代价是六线圈≈25W 持续发热，久留在踏步状态留意阀
温；单脚长时间悬空时那条腿的舵机也在持续扛载出力，留意舵机温度。

阀策略 --vent（默认 auto）：地面行走不吸附，但本机阀断电=通罐位，吸盘经每足
单向阀接歧管——身体一压把盘里空气挤进歧管，抬腿时单向阀不让空气回流，盘内
形成被动真空把脚吸在地上（09-02 实机：装气路后抬腿幅度极低、吸盘不离地）。
  auto  站起时和走动时六阀通电（排气位，吸盘通大气），停走落稳 0.5s 后断电，
        站着不动不耗六线圈≈25W。起步时线圈按足串行上电约 1s，**通电完成前
        脚不抬**（状态行显示"排气中"），之后才开始走
  on    整段常通（对照/兜底）
  off   不碰阀 GPIO/I2C（装气路前旧行为，对照：脚会被吸住）
运行中按 v 轮换三种策略。退出时先断舵机电再断六线圈；线圈断电后吸盘回到
通罐位，搬机时若脚被轻微吸住属正常，关 12V 即放开。

黑匣子（09-04）：每跑一次落 `logs/walk_<时间>.log`——启动步骤逐步落盘（每路阀
线圈通电前、舵机继电器合闸前后/固件使能前后的母线电压）、Pi 5 的 5V 输入电压每
0.1s 一行（就位后 0.5s）、遥控中每 0.5s 一行母线电压电流。09-04 实机：本脚本
启动也死过机（灯绿→红），当时没有黑匣子只能空手验尸，故与 climb_walk/body_lean
同口径。死机后 `bash scripts/pi_forensics.sh check` 会带出最新一份尾巴。
"""
import argparse
import os
import select
import sys
import termios
import time
import tty

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver, TRIPOD, WAVE
from hexapod.adhesion import GroundVent, MockVacuumIO, Pi5VacuumIO
from hexapod.gait import GaitEngine, MarchEngine
from hexapod.powerlog import PowerWatch, startup_marker, servo_power_on
from hexapod.runlog import RunLog

SPEED = 40.0    # mm/s
TURN = 0.3      # rad/s
POWER_PRINT_S = 0.5   # 电压/电流状态行刷新间隔（原地覆写，不刷屏）


def read_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--vent", choices=GroundVent.MODES, default="auto",
                    help="六阀策略：auto=站起/走动通电排气、静止断电（默认）；"
                         "on=常通；off=不碰阀（脚会被被动真空吸住，对照用）。运行中 v 轮换")
    ap.add_argument("--march-lift", type=float, default=MarchEngine.LIFT_MM,
                    help=f"原地踏步（m 键）单脚抬起高度 mm，默认 {MarchEngine.LIFT_MM:.0f}"
                         "（步行抬脚 40）")
    args = ap.parse_args()
    mode = args.vent

    # 黑匣子：建在碰任何硬件之前，硬件初始化失败也要留痕（excepthook 兜底）；
    # 与 climb_walk 同口径（hexapod/runlog.py + hexapod/powerlog.py）
    log = RunLog(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs"), tag="walk")
    print(f"黑匣子日志: {log.path}")
    log.note(f"模式={'mock' if args.mock else '实机'} port={args.port} vent={mode}"
             f" SPEED={SPEED:g} TURN={TURN} march_lift={args.march_lift:g}")
    pwr = PowerWatch(log).start()
    if pwr.uv_ever_at_start:
        print("⚠ 本次开机以来 Pi 已出现过欠压（get_throttled 粘滞位）——供电有问题，"
              "先查 5V 降压再跑")
    step = startup_marker(log, pwr)   # 启动步骤标记：一步一行当场落盘
    _prev_hook = sys.excepthook

    def _crash_hook(tp, val, tb):
        # 任何未捕获异常（含硬件初始化失败/欠压停机）落盘后再走默认打印
        pwr.stop()
        log.exc(val)
        log.close("uncaught")
        _prev_hook(tp, val, tb)
    sys.excepthook = _crash_hook

    # 阀先于舵机：站起时吸盘就已通大气，压下去不攒被动真空
    vent = GroundVent(io_factory=((lambda: MockVacuumIO(6)) if args.mock
                                  else (lambda: Pi5VacuumIO(6, on_step=step))),
                      stagger_s=0.0 if args.mock else 0.2)
    if mode != "off":
        step("阀板初始化：六阀线圈按足串行通电（排气位，0.2s 间隔）")
        vent.set(True)
        step("六阀已到排气位（线圈全通电）")
        print(f"阀策略 {mode}：站起前六阀已拉到排气位（吸盘通大气）")
    else:
        print("阀策略 off：不碰阀（通罐位），吸盘可能被被动真空吸在地上，按 v 切策略对照")

    step("打开舵机串口 + 继电器 GPIO17（保持断开）")
    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    bot = Hexapod(drv)
    # 缓慢站起：使能前先发蹲姿（离断电趴姿最近，使能跳变小），再慢滑到站姿
    bot.move_feet(bot.crouch_feet())
    servo_power_on(drv, log, pwr)   # 合闸→0.4s→固件使能，两段各采母线电压，替代原 sleep(1)
    log.event("缓慢站起（4s）")
    bot.stand(duration=4.0)
    log.event("站姿就位，遥控就绪")
    pwr.relax()                     # 大电流启动段过了，Pi 电源采样放慢到 0.5s

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = vy = wz = 0.0
    t = 0.0
    dt = 1.0 / bot.cfg.update_hz
    last_power_check = 0.0
    peak_a = 0.0
    march = None          # 原地踏步引擎（None=普通遥控）
    print("遥控就绪 (w/s/a/d/q/e, 空格停, 1/2 步态, v 阀策略, "
          "m 原地踏步, ESC 退出)")
    try:
        while True:
            k = read_key(0)
            if k == "\x1b":
                break
            elif k == "m":
                if march is None:               # 进踏步：先站稳，六脚都在站位不动
                    vx = vy = wz = 0.0
                    march = MarchEngine(bot.cfg, lift_mm=args.march_lift)
                    if mode == "off":   # 踏步全程要六阀通电排气，off 不通电=脚被吸住
                        mode = "on"
                        print("\n阀策略 off → on：踏步要六阀全程通电排气"
                              "（踏步中 v 不生效，要看被吸住的对照先按 m 出来）")
                    bot.glide_to(dict(bot.engine.default_feet), 0.5)
                    print(f"\n进入原地踏步（只认这六个键，m 退出）：{march.key_hint()}"
                          f"\n  按一下抬起 {march.lift_mm:.0f}mm，再按踩下")
                else:                           # 出踏步：抬着的脚先滑回站位再交还遥控
                    march = None
                    bot.glide_to(dict(bot.engine.default_feet), MarchEngine.MOVE_S)
                    print("\n退出原地踏步，回到遥控（已停住，w/s/a/d/q/e 可走）")
            elif march is not None:
                leg = march.leg_of_key(k)       # 踏步状态只认六个脚键，其余一律忽略
                if leg:
                    up = march.toggle(leg)
                    n_up = len(march.up_legs())
                    print(f"\n{leg} {'抬起' if up else '踩下'}"
                          f"（抬着：{'/'.join(march.up_legs()) or '无'}）")
                    if n_up >= 4:
                        print(f"⚠ 已经抬着 {n_up} 只脚，剩下 {6 - n_up} 条腿撑不住整机，小心翻")
            elif k == "w":
                vx, vy = SPEED, 0
            elif k == "s":
                vx, vy = -SPEED, 0
            elif k == "a":
                vx, vy = 0, SPEED
            elif k == "d":
                vx, vy = 0, -SPEED
            elif k == "q":
                wz = TURN
            elif k == "e":
                wz = -TURN
            elif k == " ":
                vx = vy = wz = 0.0
            elif k == "1":
                bot.engine = GaitEngine(bot.cfg, TRIPOD)
            elif k == "2":
                bot.engine = GaitEngine(bot.cfg, WAVE)
            elif k == "v":
                mode = GroundVent.MODES[(GroundVent.MODES.index(mode) + 1) % len(GroundVent.MODES)]
                print(f"\n阀策略 → {mode}")

            moving = bool(vx or vy or wz)
            # auto：走动/踏步全程通电、只有普通遥控站着不动才断电；线圈没全部通电前
            # 脚不抬（起步原地等 ≈1s，踏步里抬脚指令也先攒着，等阀通电完再动）
            if vent.drive(mode, moving or march is not None):
                if march is not None:
                    march.update(dt)
                    bot.move_feet(march.foot_targets())
                else:
                    bot.move_feet(bot.engine.foot_targets(t, vx, vy, wz))
            else:
                bot.move_feet(bot.engine.foot_targets(t, 0.0, 0.0, 0.0))
            if t - last_power_check > POWER_PRINT_S:
                v, c = bot.check_power()  # 欠压直接抛异常停机
                peak_a = max(peak_a, c)
                line = (f"电压 {v:.2f}V  电流 {c:5.2f}A  峰值 {peak_a:5.2f}A  "
                        f"阀[{mode}] {vent.state_text()}")
                if march is not None:
                    line += f"  踏步[{march.state_text()}]"
                print("\r" + line.ljust(78), end="", flush=True)  # 补空格抹掉上一行残留
                log.row(f"t={t:6.1f} 电={v:.2f}V/{c:.2f}A/峰{peak_a:.2f}"
                        f" 速={vx:+.0f},{vy:+.0f},{wz:+.2f} 阀[{mode}]"
                        f"{' 踏步' if march is not None else ''}")
                last_power_check = t
            time.sleep(dt)
            t += dt
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        log.event("退出：断舵机电 → 断六线圈")
        try:
            drv.close()
        finally:
            vent.close()    # 六线圈断电再释放，绝不拉高着退（阀会一直通电发热）
        pwr.stop()
        log.close("正常退出")
        print("\n已断舵机电、阀线圈已断电，退出。")


if __name__ == "__main__":
    main()
