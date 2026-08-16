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
