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
