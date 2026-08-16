"""ClimbEngine 步态-吸附联动测试（P4-GUIDE 4.6.1：正常循环 / SUCKING 超时重试 /
ATTACHED 漏气 / 互锁拒抬，外加垂直落地与相位暂停的专项断言）。全部跑在
MockVacuumIO + MockDriver 上，仿真时间与控制周期同步推进。"""
import math

from hexapod.adhesion import AdhesionController, MockVacuumIO, FootState
from hexapod.climb import ClimbEngine, LegPhase
from hexapod.config import DEFAULT_CONFIG as CFG, LEG_NAMES
from hexapod.driver import MockDriver
from hexapod.robot import Hexapod

DT = 0.02
SWING = (LegPhase.LIFT, LegPhase.TRANSFER, LegPhase.DESCEND,
         LegPhase.PRESS, LegPhase.RETRY_LIFT, LegPhase.WAIT)


def make_engine(**kw):
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    return io, ctl, ClimbEngine(CFG, ctl, **kw)


def run(eng, seconds, vx=0.0, wz=0.0):
    for _ in range(int(seconds / DT)):
        eng.update(DT, vx, 0.0, wz)


def start(eng):
    """跑完启动全吸附序列。"""
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.started:
            return
    raise AssertionError(f"启动序列 20s 未完成: {eng.status()} frozen={eng.frozen}")


def test_startup_attaches_all_before_clock_runs():
    io, ctl, eng = make_engine()
    start(eng)
    assert ctl.attached_count() == 6
    assert eng.t == 0.0                        # 没吸满 6 足前相位钟不走
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())
    # 全部足端目标都在压入位
    for n in LEG_NAMES:
        assert math.isclose(eng.foot[n][2],
                            -CFG.stand_height - CFG.leg(n).press_delta_mm)


def test_stationary_command_never_lifts():
    io, ctl, eng = make_engine()
    start(eng)
    run(eng, 5.0)                              # 速度 0：空转过窗，不抬腿
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())
    assert ctl.attached_count() == 6
    assert abs(eng.t - 5.0) < 1e-3             # 钟正常走（窗被跳过而不是暂停）


def test_walk_cycle_invariants():
    """正常行走 30s：每腿完整走完分段摆动；全程互锁、竖直下探、放气确认、
    落点即压入位等硬规则成立；所有目标可解 IK。"""
    io, ctl, eng = make_engine()
    start(eng)
    bot = Hexapod(MockDriver())
    seen = {n: set() for n in LEG_NAMES}
    last_z = {}
    for _ in range(int(30 / DT)):
        targets = eng.update(DT, 30.0, 0.0, 0.0)
        assert eng.frozen is None
        bot.pulses(targets)                    # 全部目标必须在工作空间内
        assert ctl.attached_count() >= 5       # 互锁：任意时刻至少 5 足吸附
        swinging = [n for n in LEG_NAMES if eng.phase_of[n] in SWING]
        assert len(swinging) <= 1              # 一次至多一腿离面
        for n in LEG_NAMES:
            ph = eng.phase_of[n]
            seen[n].add(ph)
            x, y, z = targets[n]
            if ph == LegPhase.TRANSFER:
                # 放气确认（RELEASED）之前不许横移
                assert ctl.state[LEG_NAMES.index(n)] == FootState.RELEASED
            if ph in (LegPhase.DESCEND, LegPhase.PRESS, LegPhase.WAIT):
                # 竖直落地：XY 冻结在落点正上方
                lx, ly = eng.landing[n]
                assert abs(x - lx) < 1e-9 and abs(y - ly) < 1e-9
            if ph == LegPhase.DESCEND and n in last_z \
                    and last_z[n][0] == LegPhase.DESCEND:
                # 下探限速：消灭"拍地"
                assert last_z[n][1] - z <= CFG.descend_speed * DT + 1e-9
            last_z[n] = (ph, z)
    for n in LEG_NAMES:
        missing = {LegPhase.LIFT, LegPhase.TRANSFER, LegPhase.DESCEND,
                   LegPhase.PRESS, LegPhase.WAIT} - seen[n]
        assert not missing, f"{n} 没走完整摆动分段，缺 {missing}"
    assert eng.t > 0.0


def test_suck_timeout_retries_then_succeeds():
    io, ctl, eng = make_engine()
    start(eng)
    io.sealed[0] = False                       # L1 下一步落脚必然吸不上
    saw_retry = False
    for _ in range(int(30 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.retries["L1"] >= 1:
            saw_retry = True
            io.sealed[0] = True                # 重试时"贴好了"
        if saw_retry and eng.phase_of["L1"] == LegPhase.STANCE \
                and ctl.is_attached(0):
            break
    assert saw_retry and ctl.is_attached(0)
    assert eng.frozen is None


def test_retry_exhausted_freezes_then_clear_resumes():
    io, ctl, eng = make_engine()
    start(eng)
    io.sealed[0] = False
    run(eng, 30.0, vx=30.0)
    assert eng.frozen is not None and "L1" in eng.frozen
    t_frozen = eng.t
    run(eng, 2.0, vx=30.0)                     # 冻结态：目标与钟都不动
    assert eng.t == t_frozen
    io.sealed[0] = True                        # 人工处理（擦唇口/挪落点）后解除
    eng.clear_freeze()
    for _ in range(int(20 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if ctl.is_attached(0) and eng.phase_of["L1"] == LegPhase.STANCE:
            break
    assert eng.frozen is None and ctl.is_attached(0)


def test_leak_pauses_clock_and_recovers():
    io, ctl, eng = make_engine()
    start(eng)
    run(eng, 1.0)
    io.sealed[5] = False                       # R3 支撑中开始漏气
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if ctl.is_leaking(5):
            break
    assert ctl.is_leaking(5)
    t0 = eng.t
    run(eng, 0.5)                              # 挽救窗内：钟暂停、不冻结
    assert eng.t == t0 and eng.frozen is None
    io.sealed[5] = True                        # 挽救成功（阀一直开着重抽）
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if not ctl.is_leaking(5):
            break
    assert not ctl.is_leaking(5)
    run(eng, 0.5)
    assert eng.t > t0                          # 钟恢复推进


def test_leak_rescue_timeout_freezes():
    io, ctl, eng = make_engine()
    start(eng)
    io.sealed[5] = False
    run(eng, CFG.leak_rescue_s + 2.0)
    assert eng.frozen is not None and "R3" in eng.frozen and "漏气" in eng.frozen


def test_interlock_refuses_lift():
    io, ctl, eng = make_engine()
    start(eng)
    ctl.state[3] = FootState.FAULT             # 硬注入：R1 支撑中异常失附
    run(eng, CFG.interlock_timeout_s + 1.5, vx=30.0)
    assert eng.frozen is not None and "互锁" in eng.frozen
    # 拒抬生效：没有任何腿离开支撑、也没放气
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())
    assert ctl.state[0] == FootState.ATTACHED


def test_leak_blocks_new_swing_while_moving():
    """漏气挽救期间即使有速度指令也不放行抬腿。"""
    io, ctl, eng = make_engine()
    start(eng)
    io.sealed[5] = False
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if ctl.is_leaking(5):
            break
    assert ctl.is_leaking(5)
    t0 = eng.t
    run(eng, 1.5, vx=30.0)                     # 挽救窗（2s）内推着走
    assert eng.frozen is None
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())
    assert eng.t == t0
    io.sealed[5] = True                        # 挽救成功后恢复行走
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if not ctl.is_leaking(5):
            break
    lifted = False
    for _ in range(int(10 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        lifted = lifted or any(p != LegPhase.STANCE
                               for p in eng.phase_of.values())
    assert lifted and eng.frozen is None


def test_vent_stall_freezes_instead_of_silent_wait():
    """LIFT 抬到位但放气确认不了（排气堵/传感器漂移）：必须冻结报警。"""
    class StuckVentIO(MockVacuumIO):
        def __init__(self):
            super().__init__(6)
            self.stuck = set()

        def step(self, dt):
            super().step(dt)
            for i in self.stuck:
                self.foot_kpa[i] = -20.0       # 高于 RELEASE_KPA，放气永不确认

    io = StuckVentIO()
    ctl = AdhesionController(io)
    eng = ClimbEngine(CFG, ctl)
    start(eng)
    io.stuck = {0}                             # L1 第一个摆动
    run(eng, 8.0, vx=30.0)
    assert eng.frozen is not None and "放气" in eng.frozen


def test_startup_waits_for_tank_and_times_out():
    """冷罐上电：罐压没建立不抽第一只脚；泵坏了超时冻结而不是重试穷尽。"""
    class DeadPumpIO(MockVacuumIO):
        def step(self, dt):
            super().step(dt)
            self.tank_kpa = 0.0                # 泵坏：罐压永远建不起来

    io = DeadPumpIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(CFG, ctl)
    run_s = 0.0
    while run_s < 35.0 and eng.frozen is None:
        eng.update(DT)
        run_s += DT
    assert eng.frozen is not None and "罐压" in eng.frozen
    # 从没对任何脚抽过气（没有重试穷尽的误报）
    assert all(s == FootState.RELEASED for s in ctl.state)


def test_tank_sensor_fault_stops_pump_and_freezes():
    class DeadTankIO(MockVacuumIO):
        def read_tank_kpa(self):
            return -112.0                      # 未接：分压点被拉到地

    io = DeadTankIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(CFG, ctl)
    eng.update(DT)
    assert ctl.tank_fault and not io.pump
    assert eng.frozen is not None and "罐压" in eng.frozen
    # 联调旁路：ignore_tank_fault 不冻结
    io2 = DeadTankIO(6)
    eng2 = ClimbEngine(CFG, AdhesionController(io2), ignore_tank_fault=True)
    for _ in range(int(1.0 / DT)):
        eng2.update(DT)
    assert eng2.frozen is None


def test_air_mode_keeps_walking_without_adhesion():
    """实机架空路径（4.6.3）：全部吸不上也要把步态走下去，统计放弃次数。"""
    io, ctl, eng = make_engine(air_mode=True)
    io.sealed = [False] * 6                    # 吸盘悬空
    for _ in range(int(60 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.started and eng.t > 1.0:
            break
    assert eng.frozen is None
    assert eng.started and eng.air_giveups >= 6
    assert eng.t > 1.0
