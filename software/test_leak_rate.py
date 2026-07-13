import time, csv
from hexapod.adhesion import Pi5VacuumIO

print(">>> [第8步] 60秒纯物理保压漏气率测试")
io = Pi5VacuumIO(n_feet=1)

print("🚨 请把吸盘死死按在干净的玻璃表面上！3秒后开始抽气...")
time.sleep(3)

print(">>> 开始抽气...")
io.set_valve(0, True)   # 阀门置于抽气/保压死路 (0)
io.set_pump(True)       # 启动真空泵

# 抽气直到低于 -50 kPa（或者抽满 2 秒）
t_suck = time.time()
while io.read_foot_kpa(0) > -50.0 and time.time() - t_suck < 2.0:
    time.sleep(0.05)

start_p = io.read_foot_kpa(0)
print(f"\n>>> 已抽到初始真空度: {start_p:.1f} kPa")
print(">>> 停泵！现在全靠阀门断电死锁和气路本身的密封性来保压！")
io.set_pump(False)

csv_filename = 'leak_rate_log.csv'
print(f">>> 开始 60 秒倒计时，数据将以 20Hz 频率实时保存至 {csv_filename} ...\n")

with open(csv_filename, 'w') as f:
    writer = csv.writer(f)
    writer.writerow(['Time_s', 'Pressure_kPa'])
    
    t0 = time.time()
    last_print_t = 0
    while time.time() - t0 < 60.0:
        t = time.time() - t0
        p = io.read_foot_kpa(0)
        writer.writerow([f"{t:.3f}", f"{p:.2f}"])
        
        # 控制台每秒打印一次，免得刷屏太快
        if t - last_print_t >= 1.0:
            print(f"静默保压中 | 时间: {t:04.1f}s / 60s | 气压: {p:5.1f} kPa", end='\r')
            last_print_t = t
            
        time.sleep(0.05) # 20Hz 采样率

end_p = io.read_foot_kpa(0)
print("\n\n>>> 60秒测试结束！通电泄压...")
io.set_valve(0, False) # 释放吸盘
time.sleep(1)
io.close()

print(f"✅ 测试完成！数据已保存至 {csv_filename}")
print("==================================================")
print(f"初始压力: {start_p:.1f} kPa")
print(f"最终压力: {end_p:.1f} kPa")
print(f"60秒总漏气量: {abs(end_p - start_p):.1f} kPa")
print("==================================================")
if end_p < -20.0:
    print("🏆 恭喜！最终气压未超过 -20kPa，保压气密性完美达标！")
else:
    print("⚠️ 最终气压高于 -20kPa，存在较明显的漏气，请重点排查三通接头或吸盘边缘！")
