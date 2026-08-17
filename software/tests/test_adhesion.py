from hexapod.adhesion import (AdhesionController, MockVacuumIO, FootState,
                              ATTACH_KPA)


def run(ctl, seconds, dt=0.02):
    for _ in range(int(seconds / dt)):
        ctl.update(dt)


def test_attach_release_cycle():
    io = MockVacuumIO()
    ctl = AdhesionController(io)
    run(ctl, 3.0)                      # 先让泵把储气罐抽起来
    assert io.tank_kpa < -40

    ctl.request_attach(0)
    run(ctl, 1.5)
    assert ctl.state[0] == FootState.ATTACHED
    assert io.read_foot_kpa(0) <= ATTACH_KPA
    assert ctl.attached_count() == 1

    ctl.request_release(0)
    run(ctl, 1.0)
    assert ctl.state[0] == FootState.RELEASED
    assert io.read_foot_kpa(0) > -5


def test_bad_seal_goes_fault_and_recovers():
    io = MockVacuumIO()
    ctl = AdhesionController(io)
    run(ctl, 3.0)
    io.sealed[2] = False               # 模拟吸盘没贴上
    ctl.request_attach(2)
    run(ctl, 2.0)
    assert ctl.state[2] == FootState.FAULT
    # 重试：贴好了
    ctl.clear_fault(2)
    io.sealed[2] = True
    ctl.request_attach(2)
    run(ctl, 1.5)
    assert ctl.state[2] == FootState.ATTACHED


def test_pump_hysteresis():
    io = MockVacuumIO()
    ctl = AdhesionController(io)
    run(ctl, 5.0)
    assert io.tank_kpa < -50           # 稳态维持在工作区间
    assert not io.pump or io.tank_kpa > -80


class _LateSealIO:
    """脚本化压力曲线：SUCKING 后期（超时线前一点）才达标。
    回归用例：确认窗不能挤占超时预算——压力已达标、只是还在确认窗里的脚，
    不许被 SUCK_TIMEOUT 打成 FAULT 放掉（那是物理上已密封的脚）。"""

    def __init__(self):
        self.t = 0.0
        self.valve = [False]
        self.pump = False

    def set_valve(self, i, on):
        self.valve[i] = bool(on)

    def set_pump(self, on):
        self.pump = bool(on)

    def read_tank_kpa(self):
        return -60.0

    def read_foot_kpa(self, i):
        # PRESSING 0.3s + SUCKING 0.75s 后达标（离 0.8s 超时线仅 0.05s）
        return -35.0 if self.t >= 1.05 else -10.0

    def step(self, dt):
        self.t += dt


def test_confirm_window_does_not_eat_suck_timeout():
    io = _LateSealIO()
    ctl = AdhesionController(io, n_feet=1)
    ctl.request_attach(0)
    run(ctl, 2.0)
    assert ctl.state[0] == FootState.ATTACHED   # 而不是被超时打成 FAULT


def test_pump_without_tank_follows_suck_demand():
    class DeadTankIO(MockVacuumIO):
        def read_tank_kpa(self):
            return -112.0                       # 罐压未接

    io = DeadTankIO(6)
    ctl = AdhesionController(io, pump_without_tank=True)
    ctl.update(0.02)
    assert ctl.tank_fault and not io.pump       # 没抽气需求：泵不转
    ctl.request_attach(0)
    run(ctl, 0.45)
    assert ctl.state[0] == FootState.SUCKING and io.pump   # 抽气中：泵跟上


class _NoTankIO(MockVacuumIO):
    """无罐模式禁读罐压：读了就炸（罐压传感器根本不在链路上）。"""

    def read_tank_kpa(self):
        raise AssertionError("无罐模式不许读罐压传感器")


def test_tankless_pump_policy_and_never_reads_tank():
    io = _NoTankIO(6)
    ctl = AdhesionController(io, tankless=True)
    ctl.update(0.02)
    assert not io.pump and not ctl.tank_fault   # 空闲：泵停、无罐压报警
    ctl.request_attach(0)
    run(ctl, 2.0)                               # 抽气需求驱动泵 -> 吸附成功
    assert ctl.state[0] == FootState.ATTACHED
    # 已吸附足压滞环维持：人为放掉压力，泵应重新启动
    run(ctl, 1.0)
    io.foot_kpa[0] = -40.0                      # 高于 PUMP_ON(-55)：该补抽了
    io.tank_kpa = -40.0                         # mock 内部物理（≠传感器读数）
    ctl.update(0.02)
    assert io.pump


def test_tankless_climb_engine_full_cycle():
    """无罐 + ClimbEngine：启动全吸附与行走全程不碰罐压传感器、不冻结。"""
    from hexapod.climb import ClimbEngine, LegPhase
    from hexapod.config import DEFAULT_CONFIG

    io = _NoTankIO(6)
    ctl = AdhesionController(io, tankless=True, suck_timeout_s=2.5)
    eng = ClimbEngine(DEFAULT_CONFIG, ctl)
    for _ in range(int(1.0 / 0.02)):
        eng.update(0.02)
    # 盲抽预抽期：泵已在转、但还没对任何脚开始抽气
    assert io.pump and ctl.attached_count() == 0
    assert all(s == FootState.RELEASED for s in ctl.state)
    for _ in range(int(20 / 0.02)):
        eng.update(0.02)
        if eng.started:
            break
    assert eng.started and ctl.attached_count() == 6
    lifted = False
    for _ in range(int(10 / 0.02)):
        eng.update(0.02, 30.0, 0.0, 0.0)
        lifted = lifted or any(p != LegPhase.STANCE
                               for p in eng.phase_of.values())
    assert lifted and eng.frozen is None
