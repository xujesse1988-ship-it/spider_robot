import time
from hexapod.driver import Servo2040Driver
from hexapod.config import DEFAULT_CONFIG

def main():
    print("=== 交互式关节摆动测试 ===")
    print("支持的通道: 0~17")
    print("按 Ctrl+C 随时退出程序。\n")
    
    d = Servo2040Driver()
    try:
        print("回中所有通道并开启驱动...")
        d.set_all_pulses_us([1500] * 18)
        d.enable(True)
        time.sleep(1.0)
        
        while True:
            try:
                ch_input = input("\n请输入指定的舵机通道号 (0~17): ")
                ch = int(ch_input.strip())
                if ch < 0 or ch > 17:
                    print("错误: 本测试支持 0~17 通道，请重新输入。")
                    continue
                    
                deg_input = input("请输入要转动的角度 (度数, 例如 0, 20, -20): ")
                deg = float(deg_input.strip())
                
                # 找到对应通道的舵机配置以进行角度到脉宽的转换
                servo_cal = None
                name = ""
                leg_name = ""
                for leg in DEFAULT_CONFIG.legs:
                    if leg.coxa.channel == ch:
                        servo_cal = leg.coxa
                        name = "Coxa"
                        leg_name = leg.name
                        break
                    elif leg.femur.channel == ch:
                        servo_cal = leg.femur
                        name = "Femur"
                        leg_name = leg.name
                        break
                    elif leg.tibia.channel == ch:
                        servo_cal = leg.tibia
                        name = "Tibia"
                        leg_name = leg.name
                        break
                        
                if not servo_cal:
                    print(f"错误: 找不到通道 {ch} 对应的舵机配置，请重新输入。")
                    continue
                    
                # 计算该关节支持的最小和最大理论角度 (基于物理舵机 ±90 度行程)
                min_deg = servo_cal.attach_deg - 90.0
                max_deg = servo_cal.attach_deg + 90.0
                
                if deg < min_deg or deg > max_deg:
                    print(f"\n[保护拦截] 错误: 目标角度 {deg}° 超出 {leg_name} {name} 关节的物理可达范围 [{min_deg}°, {max_deg}°]！请重新输入。")
                    continue
                    
                # 根据官方配置计算微秒(us)脉宽
                us = servo_cal.joint_deg_to_us(deg)
                print(f"\n准备转动: 通道 {ch} ({leg_name} {name}), 关节目标角度 {deg}°, 对应脉宽 {us:.1f}us")
                
                print(f"转动通道 {ch} 到 {deg}°...")
                d.set_pulses_us(ch, [int(us)])
                time.sleep(0.5)  # 稍作延时以便观测动作
                
            except ValueError:
                print("错误: 请输入有效的数字。")
                
    except KeyboardInterrupt:
        print("\n用户取消，正在退出。")
    except Exception as e:
        print(f"驱动发生错误: {e}")
    finally:
        print("关闭连接...")
        d.close()

if __name__ == "__main__":
    main()
