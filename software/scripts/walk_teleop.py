#!/usr/bin/env python3
"""键盘遥控行走（SSH 到树莓派运行）。

  w/s  前进/后退      a/d  左移/右移
  q/e  左转/右转      空格 停
  1/2  三角/波浪步态   v    切阀策略 auto→on→off（对照用）
  ESC  退出

用法: python walk_teleop.py [--port /dev/ttyACM0] [--mock] [--vent auto|on|off]

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
"""
import argparse
import select
import sys
import termios
import time
import tty

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver, TRIPOD, WAVE
from hexapod.adhesion import GroundVent, MockVacuumIO
from hexapod.gait import GaitEngine

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
    args = ap.parse_args()
    mode = args.vent

    # 阀先于舵机：站起时吸盘就已通大气，压下去不攒被动真空
    vent = GroundVent(io_factory=(lambda: MockVacuumIO(6)) if args.mock else None,
                      stagger_s=0.0 if args.mock else 0.2)
    if mode != "off":
        vent.set(True)
        print(f"阀策略 {mode}：站起前六阀已拉到排气位（吸盘通大气）")
    else:
        print("阀策略 off：不碰阀（通罐位），吸盘可能被被动真空吸在地上，按 v 切策略对照")

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    bot = Hexapod(drv)
    # 缓慢站起：使能前先发蹲姿（离断电趴姿最近，使能跳变小），再慢滑到站姿
    bot.move_feet(bot.crouch_feet())
    drv.enable(True)
    time.sleep(0 if args.mock else 1.0)
    bot.stand(duration=4.0)

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = vy = wz = 0.0
    t = 0.0
    dt = 1.0 / bot.cfg.update_hz
    last_power_check = 0.0
    peak_a = 0.0
    print("遥控就绪 (w/s/a/d/q/e, 空格停, 1/2 步态, v 阀策略, ESC 退出)")
    try:
        while True:
            k = read_key(0)
            if k == "\x1b":
                break
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
            # auto：走动通电/静止断电；线圈没全部通电前脚不抬（原地等 ≈1s）
            if vent.drive(mode, moving):
                bot.move_feet(bot.engine.foot_targets(t, vx, vy, wz))
            else:
                bot.move_feet(bot.engine.foot_targets(t, 0.0, 0.0, 0.0))
            if t - last_power_check > POWER_PRINT_S:
                v, c = bot.check_power()  # 欠压直接抛异常停机
                peak_a = max(peak_a, c)
                print(f"\r电压 {v:.2f}V  电流 {c:5.2f}A  峰值 {peak_a:5.2f}A  "
                      f"阀[{mode}] {vent.state_text()}  ", end="", flush=True)
                last_power_check = t
            time.sleep(dt)
            t += dt
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        try:
            drv.close()
        finally:
            vent.close()    # 六线圈断电再释放，绝不拉高着退（阀会一直通电发热）
        print("\n已断舵机电、阀线圈已断电，退出。")


if __name__ == "__main__":
    main()
