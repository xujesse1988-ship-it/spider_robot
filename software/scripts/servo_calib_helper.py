#!/usr/bin/env python3
"""交互式舵机标定辅助脚本。

循环输入舵机通道号和脉宽值，方便寻找 ±45° 位置。
"""

import sys
import argparse

# 确保能找到 hexapod 包
sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.driver import Servo2040Driver, NUM_SERVOS

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    args = ap.parse_args()

    print("初始化舵机驱动...")
    try:
        drv = Servo2040Driver(args.port)
    except Exception as e:
        print(f"打开串口失败: {e}")
        return

    # 为了安全，上电前先将所有舵机脉宽设为 1500 (中位)
    drv.set_all_pulses_us([1500] * NUM_SERVOS)
    drv.enable(True)
    
    v = drv.read_voltage_v()
    print(f"舵机已使能 (初始均回到 1500µs)。当前电压 {v:.2f}V")
    print("-" * 50)
    print("使用说明: 输入 [通道号] [脉宽值] (用空格隔开)，例如: 16 1520")
    print("输入 q 退出并断电")
    print("-" * 50)

    try:
        while True:
            cmd = input("请输入 > ").strip().lower()
            if cmd == 'q' or cmd == 'quit':
                break
            
            if not cmd:
                continue

            parts = cmd.split()
            if len(parts) != 2:
                print("⚠️ 输入格式错误！请用空格隔开通道号和脉宽值，例如: 16 1520")
                continue
                
            try:
                ch = int(parts[0])
                pw = float(parts[1])
            except ValueError:
                print("⚠️ 通道号和脉宽值必须是数字！")
                continue
                
            if ch < 0 or ch >= NUM_SERVOS:
                print(f"⚠️ 通道号超出范围！必须在 0 到 {NUM_SERVOS - 1} 之间。")
                continue
                
            if pw < 500 or pw > 2500:
                print("⚠️ 警告: 脉宽值一般在 500 ~ 2500 µs 之间，请确认数值是否合理。")
                
            drv.set_pulses_us(ch, [pw])
            print(f"✅ 已发送: 通道 {ch} 移动至 {pw} µs")
            
    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，准备退出...")
    finally:
        print("正在断开舵机电源并退出...")
        try:
            drv.enable(False)
            drv.close()
        except:
            pass

if __name__ == "__main__":
    main()
