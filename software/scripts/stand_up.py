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
    # 缓慢站起：使能前先发蹲姿（离断电趴姿最近，使能跳变小），再慢滑到站姿。
    # 舵机无位置回读，使能瞬间会满速跳到目标——跳变必须安排在蹲姿这一步。
    bot.move_feet(bot.crouch_feet())
    drv.enable(True)
    time.sleep(0 if args.mock else 1.0)
    bot.stand(duration=4.0)
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
