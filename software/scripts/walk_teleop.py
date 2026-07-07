#!/usr/bin/env python3
"""键盘遥控行走（SSH 到树莓派运行）。

  w/s  前进/后退      a/d  左移/右移
  q/e  左转/右转      空格 停
  1/2  三角/波浪步态   ESC  退出

用法: python walk_teleop.py [--port /dev/ttyACM0] [--mock]
"""
import argparse
import select
import sys
import termios
import time
import tty

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver, TRIPOD, WAVE
from hexapod.gait import GaitEngine

SPEED = 40.0    # mm/s
TURN = 0.3      # rad/s


def read_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true")
    args = ap.parse_args()

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    bot = Hexapod(drv)
    bot.move_feet(bot.engine.default_feet)
    drv.enable(True)
    bot.stand()

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = vy = wz = 0.0
    t = 0.0
    dt = 1.0 / bot.cfg.update_hz
    last_power_check = 0.0
    print("遥控就绪 (w/s/a/d/q/e, 空格停, ESC 退出)")
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

            bot.move_feet(bot.engine.foot_targets(t, vx, vy, wz))
            if t - last_power_check > 2.0:
                bot.check_power()  # 欠压直接抛异常停机
                last_power_check = t
            time.sleep(dt)
            t += dt
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        drv.close()
        print("已断舵机电，退出。")


if __name__ == "__main__":
    main()
