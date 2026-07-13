import time
from hexapod.adhesion import AdhesionController, FootState, Pi5VacuumIO

print(">>> [极限挂重测试模式] 初始化硬件...")
io = Pi5VacuumIO(n_feet=1)
ctl = AdhesionController(io, n_feet=1)

print("🚨 警告：请确保测试时重物的正下方没有您的脚！")
print("🚨 建议在地面垫好软枕头或纸箱，以防重物掉落砸坏地板！")
print(">>> 3秒后启动抽气，请把吸盘按在水平架起的玻璃下表面...")
io.set_pump(True)
time.sleep(3)

ctl.request_attach(0)
while not ctl.is_attached(0):
    ctl.update(0.02)
    time.sleep(0.02)
    if ctl.state[0] is FootState.FAULT:
        io.set_pump(False)
        io.close()
        raise SystemExit("\n❌ 没吸住，请检查是否按平了！")

print(f"\n✅ 吸附成功！初始压力: {io.read_foot_kpa(0):.1f} kPa")
print("==================================================")
print(">>> 现在系统已进入【超长保压闭环模式】 (时长: 60 秒) <<<")
print(">>> 请开始慢慢往吸盘上挂矿泉水或其他重物！")
print(">>> 如果吸盘崩脱，随时可以按 Ctrl+C 紧急结束脚本。")
print("==================================================")

try:
    t_hold = time.time()
    while time.time() - t_hold < 60.0:
        ctl.update(0.02)
        time.sleep(0.02)
        # 实时打印压力，方便您在脱落的瞬间看一眼临界气压值
        print(f"保压中 | 剩余倒计时: {60 - (time.time()-t_hold):.1f}s | 当前气压: {io.read_foot_kpa(0):.1f} kPa   ", end='\r')
except KeyboardInterrupt:
    print("\n\n⚠️ 检测到 Ctrl+C，提前结束挂重测试！")

print("\n\n>>> 测试结束，通电泄放真空...")
ctl.request_release(0)

while ctl.state[0] is not FootState.RELEASED:
    ctl.update(0.02)
    time.sleep(0.02)

print("✅ 已经安全泄气释放！")
io.set_pump(False)
io.close()
