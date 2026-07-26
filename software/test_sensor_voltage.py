import time
from hexapod.adhesion import Pi5VacuumIO

def test_sensor_voltage():
    print(">>> 气压传感器电压读取测试 (大气压状态下)")
    io = Pi5VacuumIO(n_feet=1)
    
    try:
        while True:
            # 读取 ADS1115 引脚的电压
            adc_v = io._read_v(0)
            
            # 计算传感器实际输出的电压 (乘以分压系数)
            sensor_v = adc_v * io.V_DIV
            
            # 顺便读取一下换算后的气压值
            kpa = io.read_foot_kpa(0)
            
            print(f"ADC测得电压(经过分压): {adc_v:.3f} V | 传感器实际输出: {sensor_v:.3f} V | 当前气压计算值: {kpa:5.1f} kPa", end='\r')
            time.sleep(0.2)
            
    except KeyboardInterrupt:
        print("\n>>> 测试结束")
    except Exception as e:
        print(f"\n>>> 发生错误: {e}")
    finally:
        io.close()

if __name__ == '__main__':
    test_sensor_voltage()
