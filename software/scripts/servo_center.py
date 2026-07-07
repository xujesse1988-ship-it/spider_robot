#!/usr/bin/env python3
"""装配标定第一步：全部 18 路舵机回中位(1500µs)。

装舵盘前运行本脚本，保持上电状态按官方视频的角度装舵盘。
⚠️ 已装好整机时慎用：全体回中会让机器人突然改变姿态，先把机身架空。

用法: python servo_center.py [--port /dev/ttyACM0] [--channel N]
"""
import argparse
import sys

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.driver import Servo2040Driver, NUM_SERVOS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--channel", type=int, default=None,
                    help="只回中某一通道 0-17（默认全部）")
    args = ap.parse_args()

    drv = Servo2040Driver(args.port)
    if args.channel is None:
        drv.set_all_pulses_us([1500] * NUM_SERVOS)
    else:
        drv.set_pulses_us(args.channel, [1500])
    drv.enable(True)
    v = drv.read_voltage_v()
    print(f"舵机已使能并回中。舵机电源电压 {v:.2f}V")
    print("按回车断电退出...")
    input()
    drv.close()


if __name__ == "__main__":
    main()
