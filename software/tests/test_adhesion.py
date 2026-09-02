import time

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


def _bare_pi5io():
    """不碰硬件构造 Pi5VacuumIO（绕开 __init__ 的 lgpio/smbus 导入）：只测
    _kpa 的读失败容忍逻辑。08-23 实测泵/阀通电瞬态致 Errno 121 且 <1s 自愈，
    单次 NACK 直接上抛会把毛刺放大成全机冻结——本组测试固化"重试一次→
    0.5s 旧值→持续才上抛"的梯子。"""
    from hexapod.adhesion import Pi5VacuumIO
    io = object.__new__(Pi5VacuumIO)
    io._cache = {}
    io._burst = []
    io.read_faults = 0
    return io


def test_bus_error_rides_glitch_then_raises_when_sustained():
    io = _bare_pi5io()
    adc = (0x48, 1)
    calls = []
    io._read_v = lambda addr, ch: (calls.append("ok"), 2.0)[1]
    k0 = io._kpa(adc)                      # 灌一个新鲜缓存

    def read_fail(addr, ch):
        calls.append("fail")
        raise OSError(121, "Remote I/O error")
    io._read_v = read_fail
    time.sleep(0.06)                       # 过 CACHE_S/突发窗，走真实读路径
    assert io._kpa(adc) == k0              # 瞬态：重试后用旧值顶，不上抛
    assert io.read_faults == 1
    assert calls.count("fail") == 2        # 首读 + 重试各一次
    # 旧值做旧 >0.5s = 持续失联：必须上抛，报文点名总线失联 + 芯片地址
    io._cache[adc] = (k0, time.time() - 0.6)
    io._burst = []
    try:
        io._kpa(adc)
    except IOError as e:
        assert "总线失联" in str(e) and "0x48" in str(e)
    else:
        raise AssertionError("持续总线失联未上抛")


def test_bus_error_single_glitch_recovers_on_retry():
    io = _bare_pi5io()
    adc = (0x49, 2)
    seq = [OSError(121, "Remote I/O error"), 1.9]

    def read_flaky(addr, ch):
        v = seq.pop(0)
        if isinstance(v, Exception):
            raise v
        return v
    io._read_v = read_flaky
    kpa = io._kpa(adc)                     # 首读炸、重试成功：拿到新鲜值
    assert not seq and io.read_faults == 0
    assert abs(kpa - io.KPA_PER_V * (1.9 * io.V_DIV - io.V_ATM)) < 1e-9


# ---------- GroundVent：地面不吸附行走的六阀排气开关 ----------

class _SpyIO(MockVacuumIO):
    def __init__(self):
        super().__init__(6)
        self.writes = []
        self.closed = False

    def set_valve(self, i, on):
        super().set_valve(i, on)
        self.writes.append((i, bool(on)))

    def close(self):
        self.closed = True


def test_ground_vent_lazy_toggle_and_close():
    from hexapod.adhesion import GroundVent
    made = []

    def factory():
        io = _SpyIO()
        made.append(io)
        return io

    gv = GroundVent(io_factory=factory, stagger_s=0.0)
    # --no-vent：不开排气也从不切换 → 完全不碰阀 IO
    gv.set(False)
    assert not made and not gv.on and gv.io is None
    # 首次开排气才构造 IO；IO 初态已是排气位（valve 全 False=不通真空）→ 不再重复写
    gv.set(True)
    assert len(made) == 1 and gv.on
    io = made[0]
    assert not any(io.valve) and io.writes == []
    # v 键切回通罐：六阀 set_valve(True)=线圈断电
    assert gv.toggle() is False
    assert all(io.valve) and io.writes == [(i, True) for i in range(6)]
    # 再切排气：六阀 set_valve(False)=线圈通电
    assert gv.toggle() is True
    assert not any(io.valve) and len(made) == 1
    # 收尾：六线圈断电 + 泵停 + 释放；之后 set(False) 不会再建 IO
    gv.close()
    assert io.closed and all(io.valve) and not io.pump
    assert gv.io is None and not gv.on
    gv.set(False)
    assert len(made) == 1


def test_ground_vent_close_without_io_is_noop():
    from hexapod.adhesion import GroundVent
    gv = GroundVent(io_factory=lambda: (_ for _ in ()).throw(AssertionError("不该建 IO")))
    gv.close()
    assert gv.io is None and not gv.on
