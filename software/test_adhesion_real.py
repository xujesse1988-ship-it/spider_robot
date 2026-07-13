import time
from hexapod.adhesion import AdhesionController, FootState, Pi5VacuumIO

print(">>> 初始化硬件...")
io = Pi5VacuumIO(n_feet=1)
ctl = AdhesionController(io, n_feet=1)

print(">>> 启动真空泵，提前抽真空 (预留3秒准备时间)")
print("🚨 请立刻把吸盘平按在玻璃上！")
io.set_pump(True)
time.sleep(3)

print(">>> 发送吸附指令 (request_attach)...")
t0 = time.time()
ctl.request_attach(0)

# 循环等待，直到状态变成 ATTACHED
while not ctl.is_attached(0):
    ctl.update(0.02)
    time.sleep(0.02)
    if ctl.state[0] is FootState.FAULT:
        io.set_pump(False)
        io.close()
        raise SystemExit("❌ SUCKING 超时——没吸住，请检查密封或是否按紧了！")

ta = time.time() - t0
print(f"✅ 吸附成功！确认耗时: {ta:.2f}s, 当前压力: {io.read_foot_kpa(0):.1f} kPa")

print(">>> 保持吸附状态 3 秒钟 (此时依然通电，您可以稍微体验一下吸力)...")
time.sleep(3)

print(">>> 发送释放指令 (request_release)...")
t1 = time.time()
ctl.request_release(0)

# 循环等待，直到状态变成 RELEASED
while ctl.state[0] is not FootState.RELEASED:
    ctl.update(0.02)
    time.sleep(0.02)

tr = time.time() - t1
print(f"✅ 释放成功！脱落耗时: {tr:.2f}s")
print(f"🎉 完整循环合计: {ta+tr:.2f}s (验收目标要求 < 2s)")

io.set_pump(False)
io.close()
