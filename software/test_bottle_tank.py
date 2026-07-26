import time
from hexapod.adhesion import Pi5VacuumIO

def test_tank():
    print(">>> 储气罐负压测试开始")
    print(">>> 初始化硬件 (ADS1115 + GPIO 20 控制泵)...")
    
    # 实例化现有的树莓派控制 IO
    io = Pi5VacuumIO(n_feet=1)
    
    try:
        start_p = io.read_foot_kpa(0)
        print(f">>> 当前初始气压: {start_p:.2f} kPa")
        print(">>> 启动真空泵，目标真空度: -70 kPa")
        
        # 启动真空泵
        io.set_pump(True)
        
        t0 = time.time()
        # 持续循环直到气压小于等于 -70 kPa
        while True:
            current_p = io.read_foot_kpa(0)
            elapsed = time.time() - t0
            
            # 实时打印气压信息 (回车符覆盖上一行)
            print(f"正在抽气... 用时: {elapsed:.1f}s | 当前气压: {current_p:5.2f} kPa", end='\r')
            
            if current_p <= -70.0:
                break
                
            time.sleep(0.05) # 20Hz 采样更新频率
            
        print(f"\n\n>>> 达到目标负压 (-70 kPa)，已自动停泵！")
        
    except KeyboardInterrupt:
        print("\n>>> 用户手动终止测试")
    except Exception as e:
        print(f"\n>>> 发生错误: {e}")
    finally:
        print(">>> 关闭真空泵，释放资源")
        io.set_pump(False)
        io.close()

if __name__ == '__main__':
    test_tank()
