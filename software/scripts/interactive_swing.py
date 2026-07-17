import time
from hexapod.driver import Servo2040Driver
from hexapod.config import DEFAULT_CONFIG

def main():
    print("=== 交互式关节摆动测试 ===")
    print("支持的通道: 15 (Coxa), 16 (Femur), 17 (Tibia)")
    
    try:
        ch_input = input("请输入指定的舵机通道号 (15/16/17): ")
        ch = int(ch_input.strip())
        if ch not in (15, 16, 17):
            print("错误: 本测试仅支持 L1 腿的 15, 16, 17 通道。")
            return
            
        deg_input = input("请输入要转动的角度 (度数, 例如 0, 20, -20): ")
        deg = float(deg_input.strip())
        
        # 找到 L1 腿对应的舵机配置以进行角度到脉宽的转换
        leg_cfg = DEFAULT_CONFIG.leg("L1")
        if ch == 15:
            servo_cal = leg_cfg.coxa
            name = "Coxa"
        elif ch == 16:
            servo_cal = leg_cfg.femur
            name = "Femur"
        elif ch == 17:
            servo_cal = leg_cfg.tibia
            name = "Tibia"
            
        # 计算该关节支持的最小和最大理论角度 (基于物理舵机 ±90 度行程)
        min_deg = servo_cal.attach_deg - 90.0
        max_deg = servo_cal.attach_deg + 90.0
        
        if deg < min_deg or deg > max_deg:
            print(f"\n[保护拦截] 错误: 目标角度 {deg}° 超出 {name} 关节的物理可达范围 [{min_deg}°, {max_deg}°]！为了保护硬件，已取消执行。")
            return
            
        # 根据官方配置计算微秒(us)脉宽
        us = servo_cal.joint_deg_to_us(deg)
        print(f"\n准备转动: 通道 {ch} ({name}), 关节目标角度 {deg}°, 对应脉宽 {us:.1f}us")
        
        d = Servo2040Driver()
        try:
            print("回中所有通道...")
            d.set_all_pulses_us([1500] * 18)
            d.enable(True)
            time.sleep(1.0)
            
            print(f"转动通道 {ch} 到 {deg}°...")
            d.set_pulses_us(ch, [int(us)])
            time.sleep(2.0)  # 保持2秒钟让用户观察
            
        except Exception as e:
            print(f"驱动发生错误: {e}")
        finally:
            print("关闭连接...")
            d.close()
            
    except ValueError:
        print("错误: 请输入有效的数字。")
    except KeyboardInterrupt:
        print("\n用户取消。")

if __name__ == "__main__":
    main()
