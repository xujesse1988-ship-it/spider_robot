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


def test_pump_inhibit_overrides_hysteresis():
    """取机窗口（climb_walk 'oo' 放吸盘保持站立）：pump_inhibit 置位后
    泵不许再开——罐压浅时滞环本该开泵，也要被压住。"""
    io = MockVacuumIO()
    ctl = AdhesionController(io)
    run(ctl, 1.0)
    assert io.pump                     # 冷罐：滞环正在要求开泵
    ctl.pump_inhibit = True
    run(ctl, 1.0)
    assert not io.pump                 # 置位后停泵，且不再被滞环重启


def test_force_release_from_fault_and_sucking():
    """退出收尾接口：request_release 只认 ATTACHED；force_release 要把
    FAULT/SUCKING 的足也真正放气到 RELEASED（审核发现：从冻结态或无罐
    SUCKING 中途 ESC 退出的足从未排气，白烧预算落假日志）。"""
    io = MockVacuumIO()
    ctl = AdhesionController(io)
    run(ctl, 3.0)
    io.sealed[0] = False               # 足 0：吸不上 → FAULT
    ctl.request_attach(0)
    run(ctl, 2.0)
    assert ctl.state[0] == FootState.FAULT
    ctl.request_attach(1)              # 足 1：停在 SUCKING 中途
    run(ctl, 0.35)
    assert ctl.state[1] == FootState.SUCKING
    ctl.request_release(0)             # 旧接口对 FAULT 纹丝不动（口径不变）
    assert ctl.state[0] == FootState.FAULT
    for i in (0, 1):
        ctl.force_release(i)
        assert ctl.state[i] == FootState.VENTING
    ctl.force_release(5)               # RELEASED 足：no-op
    assert ctl.state[5] == FootState.RELEASED
    run(ctl, 1.0)
    assert ctl.state[0] == FootState.RELEASED
    assert ctl.state[1] == FootState.RELEASED


def test_abandon_release_stops_state_machine_valve_writes():
    """放气确认超时的弃疗口径：abandon_release 后状态机不得再碰该阀——
    否则退出序列刚断电的阀会被 VENTING 分支每周期反手重新通电，黑匣子
    "线圈已断"与实际电平相反（审核发现：退出序列超时足）。"""
    class StuckVentIO(MockVacuumIO):
        def step(self, dt):
            super().step(dt)
            if not self.valve[0]:      # 排气位却排不动（排气堵）
                self.foot_kpa[0] = -20.0

    io = StuckVentIO()
    ctl = AdhesionController(io)
    run(ctl, 3.0)
    ctl.request_attach(0)
    run(ctl, 1.5)
    assert ctl.state[0] == FootState.ATTACHED
    ctl.force_release(0)
    run(ctl, 1.5)                      # 预算耗尽仍确认不了
    assert ctl.state[0] == FootState.VENTING and not io.valve[0]
    ctl.abandon_release(0)             # 弃疗：先迁出 VENTING
    assert ctl.state[0] == FootState.RELEASED
    io.set_valve(0, True)              # 调用方断阀电（通→断）
    run(ctl, 0.5)
    assert io.valve[0]                 # 不再被状态机反手翻回排气位


def test_pump_inhibit_keeps_tank_telemetry_fresh():
    """退出/取机窗口（pump_inhibit）只禁泵不禁采样：last_tank_kpa 镜像必须
    继续刷新——退出窗正是 08-18 事故窗口，TLM 整段记 ESC 前的陈旧罐压
    会害验尸（审核发现）。读失败只丢遥测，不打断收尾期状态机。"""
    io = MockVacuumIO()
    ctl = AdhesionController(io)
    run(ctl, 5.0)                      # 罐压建立
    ctl.pump_inhibit = True
    run(ctl, 0.1)
    frozen_val = ctl.last_tank_kpa
    run(ctl, 10.0)                     # 泵停后罐慢泄漏，镜像应跟着走
    assert not io.pump
    assert ctl.last_tank_kpa > frozen_val + 1.0

    io2 = MockVacuumIO()
    ctl2 = AdhesionController(io2)
    run(ctl2, 3.0)
    ctl2.request_attach(0)
    run(ctl2, 1.5)
    assert ctl2.state[0] == FootState.ATTACHED
    ctl2.pump_inhibit = True

    def _boom():
        raise IOError("I2C 降级")
    io2.read_tank_kpa = _boom
    ctl2.force_release(0)
    run(ctl2, 1.0)
    assert ctl2.state[0] == FootState.RELEASED   # 罐压读挂了，放气照常收口

    ctl3 = AdhesionController(_NoTankIO(6), tankless=True)
    ctl3.pump_inhibit = True
    run(ctl3, 0.2)                     # 无罐模式 inhibit 下也绝不读罐压


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
