"""ClimbEngine 步态-吸附联动测试（P4-GUIDE 4.6.1：正常循环 / SUCKING 超时重试 /
ATTACHED 漏气 / 互锁拒抬，外加垂直落地与相位暂停的专项断言）。全部跑在
MockVacuumIO + MockDriver 上，仿真时间与控制周期同步推进。"""
import math
from dataclasses import replace

from hexapod.adhesion import AdhesionController, MockVacuumIO, FootState
from hexapod.climb import (ClimbEngine, LegPhase, PRESS_DEPTH_MAX,
                           TILT_BAND_DEG, _press_tilt)
from hexapod.config import DEFAULT_CONFIG as CFG, LEG_NAMES
from hexapod.driver import MockDriver
from hexapod.kinematics import leg_ik
from hexapod.robot import Hexapod

DT = 0.02
SWING = (LegPhase.VENT, LegPhase.LIFT, LegPhase.TRANSFER, LegPhase.HOVER,
         LegPhase.DESCEND, LegPhase.PRESS, LegPhase.RETRY_LIFT, LegPhase.WAIT)


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


def test_vent_before_lift():
    """抬腿先通气（08-19 实机：边放气边抬会被残余真空拽起）：窗头进 VENT
    原地保持 lift_vent_s——z 不抬、XY 随支撑场平移（贴面的脚在墙面系不动），
    期间放气已在进行（VENTING+阀断真空），到时才进 LIFT。"""
    io, ctl, eng = make_engine()
    start(eng)
    for _ in range(int(5 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.phase_of["L1"] == LegPhase.VENT:
            break
    assert eng.phase_of["L1"] == LegPhase.VENT
    vx_eff = eng._clamp_speed(30.0, 0.0, 0.0)[0]
    z0, x0 = eng.foot["L1"][2], eng.foot["L1"][0]
    t_vent = 0.0
    valve_vented = False
    while eng.phase_of["L1"] == LegPhase.VENT:
        assert ctl.state[0] == FootState.VENTING       # 放气先行于抬腿
        assert math.isclose(eng.foot["L1"][2], z0)     # 原地：z 不抬
        valve_vented = valve_vented or not io.valve[0]
        eng.update(DT, 30.0, 0.0, 0.0)
        t_vent += DT
    assert valve_vented                                # 阀真断了真空
    assert eng.phase_of["L1"] == LegPhase.LIFT
    assert abs(t_vent - CFG.lift_vent_s) < 3 * DT
    # 贴面段随支撑场：XY 跟着整体平移（不随场 = 墙面系被拖着划）
    dx = eng.foot["L1"][0] - x0
    assert abs(dx + vx_eff * t_vent) <= 2 * vx_eff * DT + 1e-9


def test_startup_sequential_press():
    """启动逐足压入（08-19 实机：六腿同时压没有反力座，机身被整体顶起，
    杯压不实）：任意时刻至多一腿离开等待位，按队列顺序逐足"压入->吸住->
    下一腿"，未轮到的腿 z 停在站位不预压。"""
    io, ctl, eng = make_engine()
    order = []
    for _ in range(int(30 / DT)):
        eng.update(DT)
        moving = [n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE]
        assert len(moving) <= 1, f"同时压入: {moving}"
        for n in moving:
            if n not in order:
                order.append(n)
        for n in eng._attach_queue[1:]:        # 未轮到：不预压
            assert math.isclose(eng.foot[n][2], -CFG.stand_height)
        if eng.started:
            break
    assert eng.started and ctl.attached_count() == 6
    assert order == list(LEG_NAMES)


def test_retry_presses_deeper_each_time():
    """重试加深：FAULT 后不是原深度重压（几何缺口不变必然同败），每次比
    上次加深 retry_deeper_mm；吸上后保持段停在实际深度不回弹。"""
    io, ctl, eng = make_engine()
    start(eng)
    io.sealed[0] = False
    min_z, deep = 0.0, False
    for _ in range(int(30 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.phase_of["L1"] in (LegPhase.PRESS, LegPhase.WAIT):
            min_z = min(min_z, eng.foot["L1"][2])
        if eng.retries["L1"] >= 2:
            deep = True
            io.sealed[0] = True                # 第二次加深后"贴好了"
        if deep and ctl.is_attached(0) \
                and eng.phase_of["L1"] == LegPhase.STANCE:
            break
    assert ctl.is_attached(0) and eng.frozen is None
    z_nom = -CFG.stand_height - CFG.leg("L1").press_delta_mm
    assert min_z <= z_nom - 2 * CFG.retry_deeper_mm + 1e-6   # 真压深了
    # 保持段 = 实际吸附深度（回名义深度会把刚吸上的盘拔回去）
    assert eng.foot["L1"][2] <= z_nom - 2 * CFG.retry_deeper_mm + 1e-6


def test_press_posture_cup_axis_perpendicular():
    """爬墙站位的核心几何：压入位**物理吸盘轴**（= a_t + cup_delta，不是
    K→唇心连线）必须垂直于吸附面（html/tibia-three-view.html 口径）。"""
    io, ctl, eng = make_engine()
    for leg in CFG.legs:
        x, y, _ = eng.default_feet[leg.name]
        r = math.hypot(x - leg.mount_x, y - leg.mount_y)
        z_press = -CFG.stand_height - leg.press_delta_mm
        _, a, th = leg_ik(CFG, r, 0.0, z_press)
        axis_deg = math.degrees(a + th - math.pi) + CFG.cup_delta_deg
        assert abs(axis_deg + 90.0) < 0.5, \
            f"{leg.name} 吸盘轴 {axis_deg:.1f}°（应 -90°），r={r:.1f}"
        # 旧默认站位（130）不满足该约束，站位必须是解出来的
        assert r > 150.0


def test_landing_clamped_to_tilt_band():
    """任意方向的极限步长，落点压入位唇口倾角都不越 ±TILT_BAND_DEG。"""
    io, ctl, eng = make_engine()
    for n in LEG_NAMES:
        leg = CFG.leg(n)
        z_press = -CFG.stand_height - leg.press_delta_mm
        for k in range(8):                     # 8 个方向的最大速度指令
            ang = math.pi * k / 4
            lx, ly = eng._landing_xy(n, 300 * math.cos(ang),
                                     300 * math.sin(ang), 0.0)
            r = math.hypot(lx - leg.mount_x, ly - leg.mount_y)
            tilt = math.degrees(_press_tilt(CFG, r, z_press))
            assert abs(tilt) <= TILT_BAND_DEG + 0.1, \
                f"{n} 方向{k}: 落点倾角 {tilt:.1f}°"


def test_crouch_to_climb_stance_vertical_path_reachable():
    """climb_walk 启动路径：爬墙站位 XY 的蹲姿 -> 站姿纯竖直插值全程 IK 可解
    （吸盘不横拖的一步到位站起）。"""
    io, ctl, eng = make_engine()
    bot = Hexapod(MockDriver())
    crouch = bot.crouch_feet(feet=eng.default_feet)
    for s in range(11):
        mix = {n: tuple(a + (b - a) * s / 10 for a, b in
                        zip(crouch[n], eng.default_feet[n]))
               for n in crouch}
        bot.pulses(mix)
        for n in crouch:                   # 纯竖直：XY 与爬墙站位完全一致
            assert mix[n][:2] == eng.default_feet[n][:2]


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


def _run_one_comp_event(eng, vx, expect_comp):
    """跑完一次抬腿事件，断言支撑足位移账目：离面段（LIFT/TRANSFER）=
    速度场积分（随钟）+ expect_comp 补偿（随真实时间铺完）；贴面段
    （DESCEND 起）只有速度场——不横拖正在压入的吸盘。y 不受扰、z 恒在
    压入位。"""
    vx_eff = eng._clamp_speed(vx, 0.0, 0.0)[0]     # 步幅限幅后的真实速度
    # 起步过渡周期不装填补偿（稳态账未建立）：先走满一轮六腿都摆过、
    # 且当前摆动收尾，再量下一个稳态事件
    for _ in range(int(90 / DT)):
        eng.update(DT, vx, 0.0, 0.0)
        if len(eng._swung_since_go) >= 6 and all(
                p == LegPhase.STANCE for p in eng.phase_of.values()):
            break
    else:
        raise AssertionError("90s 未完成首轮热身")
    swing = t0 = f0 = None
    for _ in range(int(10 / DT)):                  # 等一条腿进摆动
        t_pre = eng.t
        feet_pre = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
        eng.update(DT, vx, 0.0, 0.0)
        lifted = [n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE]
        if lifted:
            swing, t0, f0 = lifted[0], t_pre, feet_pre
            break
    assert swing is not None
    obs = next(n for n in LEG_NAMES if n != swing)  # 任取一条支撑腿观察
    for _ in range(int(10 / DT)):                  # 跑到 DESCEND 入口
        if eng.phase_of[swing] not in (LegPhase.VENT, LegPhase.LIFT,
                                       LegPhase.TRANSFER):
            break
        eng.update(DT, vx, 0.0, 0.0)
    assert eng.phase_of[swing] == LegPhase.DESCEND
    dx = eng.foot[obs][0] - f0[obs][0]
    exp = -vx_eff * (eng.t - t0) - expect_comp
    assert math.isclose(dx, exp, abs_tol=1e-6), f"离面段 {dx:.4f} 应 {exp:.4f}"
    assert math.isclose(eng.foot[obs][1], f0[obs][1], abs_tol=1e-9)
    t1, x1 = eng.t, eng.foot[obs][0]
    for _ in range(int(20 / DT)):                  # 跑到摆动腿回支撑
        if eng.phase_of[swing] == LegPhase.STANCE:
            break
        eng.update(DT, vx, 0.0, 0.0)
    assert eng.phase_of[swing] == LegPhase.STANCE
    dx2 = eng.foot[obs][0] - x1                    # 贴面段：无补偿注入
    assert math.isclose(dx2, -vx_eff * (eng.t - t1), abs_tol=1e-6), \
        f"贴面段被注入了补偿: {dx2:.4f} vs {-vx_eff * (eng.t - t1):.4f}"
    assert math.isclose(eng.foot[obs][2],
                        -CFG.stand_height - CFG.leg(obs).press_delta_mm)
    assert eng.frozen is None


def test_sag_comp_injects_during_lift_transfer_only():
    """满速直行：半步幅 20，平面尾预算随压深反解（press 18 ≈36.8 →
    限额 ≈3.4mm/事件），设定 6 被动态截住；注入只在 LIFT/TRANSFER 离面段。"""
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(replace(CFG, climb_sag_comp_mm=6.0), ctl)
    start(eng)
    cap = (eng.comp_tail - CFG.climb_max_step / 2.0) / 5.0
    assert 0.0 < cap < 6.0                 # 设定 6 真被限额截住才算数
    _run_one_comp_event(eng, vx=30.0, expect_comp=cap)


def test_sag_comp_full_delivery_at_low_speed():
    """低速直行步幅小，限额放宽，设定值全额交付：vx=5 → 半步幅 7.5，
    限额 (comp_tail-7.5)/5（press 18 ≈5.9）≥ 设定 5 → 每事件足额 5mm。
    降速换大补偿的操作口径靠这条成立。"""
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(replace(CFG, climb_sag_comp_mm=5.0), ctl)
    start(eng)
    assert (eng.comp_tail - 7.5) / 5.0 >= 5.0   # 前提：低速限额确实放得下
    _run_one_comp_event(eng, vx=5.0, expect_comp=5.0)


def test_sag_comp_capped_walk_stays_solvable():
    """补偿顶到 CLI 上限 8mm/事件 + 满速走 60s：动态限额（comp_tail 总账，
    随压深反解）截住每事件量，全程 IK 可解（出工作空间 pulses 会抛
    WorkspaceError 炸控制环）、互锁不破、不冻结。贴边界的原最紧工况
    （起步过渡瞬态，press 13 时实测 d≈201.7）已被过渡期门控消灭——直行
    稳态最紧 d 离边界 ~12mm，贴边工况由 deep-attach 回归+上界断言看护；
    这里另断言热身后补偿确实被反复装填（门控没把补偿整个关死）。"""
    cfg = replace(CFG, climb_sag_comp_mm=8.0)
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(cfg, ctl)
    start(eng)
    bot = Hexapod(MockDriver(), cfg)
    d_max = cfg.femur_len + cfg.tibia_len
    worst_d = 0.0
    events = 0
    comp_loads = 0
    prev = dict(eng.phase_of)
    prev_left = 0.0
    for _ in range(int(60 / DT)):
        targets = eng.update(DT, 30.0, 0.0, 0.0)
        assert eng.frozen is None
        bot.pulses(targets)
        assert ctl.attached_count() >= 5
        if eng._comp_left > prev_left + 1e-9:   # 本窗装填了非零补偿
            comp_loads += 1
        prev_left = eng._comp_left
        for n in LEG_NAMES:
            if eng.phase_of[n] == LegPhase.STANCE:
                leg = cfg.leg(n)
                x, y, z = targets[n]
                r = math.hypot(x - leg.mount_x, y - leg.mount_y) - cfg.coxa_len
                worst_d = max(worst_d, math.hypot(r, z))
            if prev[n] == LegPhase.STANCE and eng.phase_of[n] != LegPhase.STANCE:
                events += 1
        prev = dict(eng.phase_of)
    assert events >= 5, f"60s 只发生 {events} 次抬腿事件，补偿没被反复运用"
    assert comp_loads >= 5, \
        f"热身后仅 {comp_loads} 次装填补偿，过渡门控疑似把补偿关死"
    assert worst_d <= d_max - 1.5, \
        f"支撑目标 d={worst_d:.1f} 贴死 IK 边界 {d_max:.1f}，限额总账失守"


def test_deep_attach_plus_sag_comp_stays_in_envelope():
    """审核发现 #1 回归：R3 启动期 3 次加深（extra=15）才吸上 + --sag-comp 3
    满速直行——旧账目下支撑尾 ~15s 后 d 出 IK 包络（WorkspaceError 冻结，
    f 解冻即复冻，墙上死局）。修复 = 补偿平面预算按等 d 圆随深度缩表。"""
    cfg = replace(CFG, climb_sag_comp_mm=3.0)
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(cfg, ctl)
    io.sealed[5] = False                      # R3 落脚吸不紧（08-19 症状）
    for _ in range(int(60 / DT)):
        eng.update(DT)
        if eng.retries["R3"] >= 3:
            io.sealed[5] = True               # 第三级加深后才贴住
        if eng.started:
            break
    assert eng.started
    # 双封顶：press 18 时深度封顶(28-18=10)先于数量封顶(15)生效
    assert eng._press_extra["R3"] == min(
        3 * cfg.retry_deeper_mm,
        PRESS_DEPTH_MAX - cfg.legs[0].press_delta_mm)
    bot = Hexapod(MockDriver(), cfg)
    d_max = cfg.femur_len + cfg.tibia_len
    worst = 0.0
    for _ in range(int(60 / DT)):             # 满速直行：全程可解、不冻结
        targets = eng.update(DT, 30.0, 0.0, 0.0)
        assert eng.frozen is None, f"冻结: {eng.frozen}"
        bot.pulses(targets)
        for n in LEG_NAMES:
            leg = cfg.leg(n)
            x, y, z = targets[n]
            r = math.hypot(x - leg.mount_x, y - leg.mount_y) - cfg.coxa_len
            worst = max(worst, math.hypot(r, z))
    assert worst <= d_max - 1.0, f"支撑 d={worst:.1f} 贴死边界 {d_max:.1f}"


def test_freeze_cancels_pending_single_step():
    """审核发现 #2 回归：单步等窗期间冻结，f 解冻后不得自行抬腿
    （挂起单步与'带旧速度恢复行走'同性质，必须随解冻取消）。"""
    io, ctl, eng = make_engine()
    start(eng)
    assert eng.request_step(30.0, 0.0, 0.0) is None
    eng.frozen = "测试注入"
    run(eng, 1.0)                             # 冻结期：单步不得启动
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())
    assert eng.step_pending                   # 冻结期挂起保留（可诊断）
    eng.clear_freeze()
    assert not eng.step_pending               # 解冻即取消
    run(eng, 5.0)                             # 速度 0：不得自行抬腿
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())


def test_sag_comp_downhill_rotates_with_heading():
    """_down（下坡方向）与支撑足同一旋转口径随积分航向转：
    转过 θ 后应为 (-cosθ, sinθ)——补偿方向不因转向而失准。"""
    io, ctl, eng = make_engine()
    start(eng)
    wz_eff = eng._clamp_speed(0.0, 0.0, 0.5)[2]
    t0 = eng.t
    run(eng, 15.0, wz=0.5)                         # press 18 摆动更长，多跑一会
    th = (eng.t - t0) * wz_eff
    assert th > 0.1                                # 真转了角度才算数
    assert math.isclose(eng._down[0], -math.cos(th), abs_tol=1e-9)
    assert math.isclose(eng._down[1], math.sin(th), abs_tol=1e-9)


def test_cmd_mirror_records_clamped_speed():
    """状态行/黑匣子 TLM 取 eng.cmd = 步幅限幅后的实际下发速度：记键盘原始
    指令会让日志速度恒不等于下发速度，--sag-comp 标定按日志推算预期位移
    时被系统性带偏（审核发现）。冻结/启动期不下发速度，镜像应为零。"""
    io, ctl, eng = make_engine()
    assert eng.cmd == (0.0, 0.0, 0.0)
    start(eng)
    assert eng.cmd == (0.0, 0.0, 0.0)          # 启动序列期间无速度下发
    eng.update(DT, 300.0, 0.0, 0.0)
    assert eng.cmd == eng._clamp_speed(300.0, 0.0, 0.0)
    assert 0.0 < eng.cmd[0] < 300.0            # 真被限幅了才算数
    eng.frozen = "测试注入"
    eng.update(DT, 300.0, 0.0, 0.0)
    assert eng.cmd == (0.0, 0.0, 0.0)          # 冻结：目标保持，无速度下发


def _run_to_hover(eng):
    """受理单步第一段并跑到悬停，返回途中摆动过的腿集合。主环速度恒 0。"""
    assert eng.request_step(30.0, 0.0, 0.0) is None
    swung = set()
    for _ in range(int(20 / DT)):
        eng.update(DT)
        swung |= {n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE}
        if eng.step_hover_leg():
            return swung
    raise AssertionError(f"单步 20s 未悬停: {eng.status()} frozen={eng.frozen}")


def _run_single_step(eng):
    """跑完一次分段单步（抬→悬停→落地→吸附），返回摆动过的腿集合。"""
    swung = _run_to_hover(eng)
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        swung |= {n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE}
        if not eng.step_pending and not eng.step_active:
            return swung
    raise AssertionError(f"单步落地 20s 未完成: {eng.status()} frozen={eng.frozen}")


def test_single_step_walks_one_leg_and_records_turn():
    """单步：只走"当前轮到"的一条腿，走完自动停；轮次按窗序推进且持久
    （中间静止多久都不乱），连续行走的抬腿同样推进轮次（两模式互通）。"""
    io, ctl, eng = make_engine()
    start(eng)
    assert eng.step_leg == "L1"                # 首窗轮到 L1
    assert eng.request_step(0.0, 0.0, 0.0) == "单步速度为零"
    assert _run_single_step(eng) == {"L1"}
    assert eng.step_leg == "L2" and ctl.attached_count() == 6
    eng.update(DT)
    assert eng.cmd == (0.0, 0.0, 0.0)          # 完成自动停，无残留速度
    run(eng, 3.0)                              # 静止晾一段：轮次不许漂
    assert eng.step_leg == "L2"
    assert _run_single_step(eng) == {"L2"}     # 第二步轮到 L2
    assert eng.step_leg == "L3"
    # 连续行走也推进同一份轮次记录
    seen = None
    for _ in range(int(10 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        lifted = [n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE]
        if lifted:
            seen = lifted[0]
            break
    assert seen is not None
    assert eng.step_leg == LEG_NAMES[(LEG_NAMES.index(seen) + 1) % 6]


def test_single_step_hovers_until_land_confirm():
    """分段单步：第一次 i 抬腿平移到落点上方悬停（HOVER，离面
    lift_clearance），不落地、无超时；期间支撑足纹丝不动（相位钟停在窗尾）；
    step_land 才放行落地，落完轮次推进、不能重复落。"""
    io, ctl, eng = make_engine()
    start(eng)
    assert eng.step_land() == "没有悬停中的腿"     # 没抬腿不能落
    assert _run_to_hover(eng) == {"L1"}
    assert eng.step_hover_leg() == "L1" and eng.step_active
    lx, ly = eng.landing["L1"]
    support0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES if n != "L1"}
    run(eng, 5.0)                                  # 晾着：不自行落地不超时
    assert eng.step_hover_leg() == "L1" and eng.frozen is None
    x, y, z = eng.foot["L1"]
    assert abs(x - lx) < 1e-9 and abs(y - ly) < 1e-9
    assert math.isclose(z, -CFG.stand_height + CFG.lift_clearance)
    for n, p in support0.items():                  # 悬停期支撑足纹丝不动
        assert tuple(eng.foot[n]) == p
    assert eng.request_step(30.0, 0.0, 0.0) == "已有单步在途"
    assert eng.step_land() is None                 # 第二段：落地
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if not eng.step_active and eng.phase_of["L1"] == LegPhase.STANCE:
            break
    assert ctl.is_attached(0) and ctl.attached_count() == 6
    assert eng.step_leg == "L2"                    # 轮次已推进
    assert eng.step_land() == "没有悬停中的腿"     # 落完不能重复落


def test_single_step_hover_survives_freeze():
    """悬停期冻结（如支撑足漏气/罐压）：目标保持、step_land 拒绝；解冻不
    取消悬停（与 clear_freeze"摆动中的单步保留"同口径，且悬停腿无残留
    动作），之后 step_land 正常收口。"""
    io, ctl, eng = make_engine()
    start(eng)
    _run_to_hover(eng)
    eng.frozen = "测试注入"
    run(eng, 1.0)
    assert eng.step_hover_leg() == "L1"            # 冻结：悬停保持
    assert eng.step_land() == "冻结中"
    eng.clear_freeze()
    assert eng.step_active and eng.step_hover_leg() == "L1"
    run(eng, 5.0)                                  # 解冻后无残留动作
    assert eng.step_hover_leg() == "L1"
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if not eng.step_active and eng.phase_of["L1"] == LegPhase.STANCE:
            break
    assert ctl.is_attached(0) and eng.frozen is None


def test_air_mode_keeps_walking_without_adhesion():
    """实机架空路径（4.6.3）：全部吸不上也要把步态走下去，统计放弃次数。
    预算 120s：逐足启动 + 每腿 3 次加深重试（穷尽才放弃）本身就要 ~50s。"""
    io, ctl, eng = make_engine(air_mode=True)
    io.sealed = [False] * 6                    # 吸盘悬空
    for _ in range(int(120 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.started and eng.t > 1.0:
            break
    assert eng.frozen is None
    assert eng.started and eng.air_giveups >= 6
    assert eng.t > 1.0
