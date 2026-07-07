#!/usr/bin/env python3
"""站立测试：上电 -> 平滑过渡到默认站姿 -> 打印足底开关和电源状态。

用法: python stand_up.py [--port /dev/ttyACM0] [--mock]
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    args = ap.parse_args()

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    bot = Hexapod(drv)
    # 先把目标脉宽发好再使能——固件会让舵机直接到位（避免上电乱跳）
    bot.move_feet(bot.engine.default_feet)
    drv.enable(True)
    bot.stand(duration=2.0)
    print("站立完成。Ctrl-C 退出（退出即断舵机电）。")
    try:
        while True:
            v, c = bot.check_power()
            print(f"电压 {v:.2f}V  电流 {c:.1f}A  触地 {bot.touch_states()}")
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        drv.close()


if __name__ == "__main__":
    main()
