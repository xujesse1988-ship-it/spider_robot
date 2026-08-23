"""ClimbEngine 步态-吸附联动测试（P4-GUIDE 4.6.1：正常循环 / SUCKING 超时重试 /
ATTACHED 漏气 / 互锁拒抬，外加垂直落地与相位暂停的专项断言）。全部跑在
MockVacuumIO + MockDriver 上，仿真时间与控制周期同步推进。"""
import math
from dataclasses import replace

from hexapod.adhesion import AdhesionController, MockVacuumIO, FootState
from hexapod.climb import (ClimbEngine, LegPhase, PRESS_DEPTH_MAX,
                           TILT_BAND_DEG, _press_tilt, max_straight_step,
                           parse_handover, HANDOVER_SPEED_MMS)
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
    leg = eng.slot_order[0]                            # 首窗腿（现 R3）
    fi = LEG_NAMES.index(leg)
    for _ in range(int(5 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.phase_of[leg] == LegPhase.VENT:
            break
    assert eng.phase_of[leg] == LegPhase.VENT
    vx_eff = eng._clamp_speed(30.0, 0.0, 0.0)[0]
    z0, x0 = eng.foot[leg][2], eng.foot[leg][0]
    t_vent = 0.0
    valve_vented = False
    while eng.phase_of[leg] == LegPhase.VENT:
        assert ctl.state[fi] == FootState.VENTING      # 放气先行于抬腿
        assert math.isclose(eng.foot[leg][2], z0)      # 原地：z 不抬
        valve_vented = valve_vented or not io.valve[fi]
        eng.update(DT, 30.0, 0.0, 0.0)
        t_vent += DT
    assert valve_vented                                # 阀真断了真空
    assert eng.phase_of[leg] == LegPhase.LIFT
    assert abs(t_vent - CFG.lift_vent_s) < 3 * DT
    # 贴面段随支撑场：XY 跟着整体平移（不随场 = 墙面系被拖着划）
    dx = eng.foot[leg][0] - x0
    assert abs(dx + vx_eff * t_vent) <= 2 * vx_eff * DT + 1e-9


def test_startup_sequential_press():
    """启动逐足压入（08-19 实机：六腿同时压没有反力座，机身被整体顶起，
    杯压不实）：任意时刻至多一腿离开等待位，按队列顺序逐足"压入->吸住->
    下一腿"，未轮到的腿 z 停在站位不预压。吸附序=窗序 R3→L1→R2→L3→R1→L2
    （原 LEG_NAMES 序）：落地新旧与抬腿轮转从启动起对齐、首抬腿 R3 驻留
    最久、压入左右交替。"""
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
    assert order == list(eng.slot_order)
    assert order[0] == "R3"                    # 窗序首腿（对角波浪）


def test_retry_presses_deeper_each_time():
    """重试加深：FAULT 后不是原深度重压（几何缺口不变必然同败），每次比
    上次加深 retry_deeper_mm；吸上后保持段停在实际深度不回弹。"""
    io, ctl, eng = make_engine()
    start(eng)
    leg = eng.slot_order[0]                    # 首窗腿（现 R3）
    fi = LEG_NAMES.index(leg)
    io.sealed[fi] = False
    min_z, deep = 0.0, False
    for _ in range(int(30 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.phase_of[leg] in (LegPhase.PRESS, LegPhase.WAIT):
            min_z = min(min_z, eng.foot[leg][2])
        if eng.retries[leg] >= 2:
            deep = True
            io.sealed[fi] = True               # 第二次加深后"贴好了"
        if deep and ctl.is_attached(fi) \
                and eng.phase_of[leg] == LegPhase.STANCE:
            break
    assert ctl.is_attached(fi) and eng.frozen is None
    z_nom = -CFG.stand_height - CFG.leg(leg).press_delta_mm
    assert min_z <= z_nom - 2 * CFG.retry_deeper_mm + 1e-6   # 真压深了
    # 保持段 = 实际吸附深度（回名义深度会把刚吸上的盘拔回去）
    assert eng.foot[leg][2] <= z_nom - 2 * CFG.retry_deeper_mm + 1e-6


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
    leg = eng.slot_order[0]                    # 首窗腿（现 R3）
    fi = LEG_NAMES.index(leg)
    io.sealed[fi] = False                      # 它下一步落脚必然吸不上
    saw_retry = False
    for _ in range(int(30 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.retries[leg] >= 1:
            saw_retry = True
            io.sealed[fi] = True               # 重试时"贴好了"
        if saw_retry and eng.phase_of[leg] == LegPhase.STANCE \
                and ctl.is_attached(fi):
            break
    assert saw_retry and ctl.is_attached(fi)
    assert eng.frozen is None


def test_retry_exhausted_freezes_then_clear_resumes():
    io, ctl, eng = make_engine()
    start(eng)
    leg = eng.slot_order[0]
    fi = LEG_NAMES.index(leg)
    io.sealed[fi] = False
    run(eng, 30.0, vx=30.0)
    assert eng.frozen is not None and leg in eng.frozen
    t_frozen = eng.t
    run(eng, 2.0, vx=30.0)                     # 冻结态：目标与钟都不动
    assert eng.t == t_frozen
    io.sealed[fi] = True                       # 人工处理（擦唇口/挪落点）后解除
    eng.clear_freeze()
    for _ in range(int(20 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if ctl.is_attached(fi) and eng.phase_of[leg] == LegPhase.STANCE:
            break
    assert eng.frozen is None and ctl.is_attached(fi)


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
    （中间静止多久都不乱），连续行走的抬腿同样推进轮次（两模式互通）。
    窗序规格 = 对角交替波浪 R3→L1→R2→L3→R1→L2（每侧后->前、两侧错
    半周期：相邻抬腿全对角/交叉，同排搭档间隔恒半周期）。"""
    io, ctl, eng = make_engine()
    assert eng.slot_order == ("R3", "L1", "R2", "L3", "R1", "L2")
    start(eng)
    o = eng.slot_order
    assert eng.step_leg == o[0]                # 首窗轮到窗序首腿
    assert eng.request_step(0.0, 0.0, 0.0) == "单步速度为零"
    assert _run_single_step(eng) == {o[0]}
    assert eng.step_leg == o[1] and ctl.attached_count() == 6
    eng.update(DT)
    assert eng.cmd == (0.0, 0.0, 0.0)          # 完成自动停，无残留速度
    run(eng, 3.0)                              # 静止晾一段：轮次不许漂
    assert eng.step_leg == o[1]
    assert _run_single_step(eng) == {o[1]}     # 第二步轮到窗序第二腿
    assert eng.step_leg == o[2]
    # 连续行走也推进同一份轮次记录
    seen = None
    for _ in range(int(10 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        lifted = [n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE]
        if lifted:
            seen = lifted[0]
            break
    assert seen is not None
    assert eng.step_leg == o[(o.index(seen) + 1) % 6]


def test_single_step_hovers_until_land_confirm():
    """分段单步：第一次 i 抬腿平移到落点上方悬停（HOVER，离面
    lift_clearance），不落地、无超时；期间支撑足纹丝不动（相位钟停在窗尾）；
    step_land 才放行落地，落完轮次推进、不能重复落。"""
    io, ctl, eng = make_engine()
    start(eng)
    leg = eng.slot_order[0]                        # 首窗腿（现 R3）
    fi = LEG_NAMES.index(leg)
    assert eng.step_land() == "没有悬停中的腿"     # 没抬腿不能落
    assert _run_to_hover(eng) == {leg}
    assert eng.step_hover_leg() == leg and eng.step_active
    lx, ly = eng.landing[leg]
    support0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES if n != leg}
    run(eng, 5.0)                                  # 晾着：不自行落地不超时
    assert eng.step_hover_leg() == leg and eng.frozen is None
    x, y, z = eng.foot[leg]
    assert abs(x - lx) < 1e-9 and abs(y - ly) < 1e-9
    assert math.isclose(z, -CFG.stand_height + CFG.lift_clearance)
    for n, p in support0.items():                  # 悬停期支撑足纹丝不动
        assert tuple(eng.foot[n]) == p
    assert eng.request_step(30.0, 0.0, 0.0) == "已有单步在途"
    assert eng.step_land() is None                 # 第二段：落地
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if not eng.step_active and eng.phase_of[leg] == LegPhase.STANCE:
            break
    assert ctl.is_attached(fi) and ctl.attached_count() == 6
    assert eng.step_leg == eng.slot_order[1]       # 轮次已推进
    assert eng.step_land() == "没有悬停中的腿"     # 落完不能重复落


def test_single_step_hover_survives_freeze():
    """悬停期冻结（如支撑足漏气/罐压）：目标保持、step_land 拒绝；解冻不
    取消悬停（与 clear_freeze"摆动中的单步保留"同口径，且悬停腿无残留
    动作），之后 step_land 正常收口。"""
    io, ctl, eng = make_engine()
    start(eng)
    leg = eng.slot_order[0]                        # 首窗腿（现 R3）
    fi = LEG_NAMES.index(leg)
    _run_to_hover(eng)
    eng.frozen = "测试注入"
    run(eng, 1.0)
    assert eng.step_hover_leg() == leg             # 冻结：悬停保持
    assert eng.step_land() == "冻结中"
    eng.clear_freeze()
    assert eng.step_active and eng.step_hover_leg() == leg
    run(eng, 5.0)                                  # 解冻后无残留动作
    assert eng.step_hover_leg() == leg
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if not eng.step_active and eng.phase_of[leg] == LegPhase.STANCE:
            break
    assert ctl.is_attached(fi) and eng.frozen is None


def _first_lifted(eng, vx=30.0, budget=10.0):
    """按住 w 直到有腿离开支撑相，返回首抬腿名。"""
    for _ in range(int(budget / DT)):
        eng.update(DT, vx, 0.0, 0.0)
        lifted = [n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE]
        if lifted:
            return lifted[0]
    raise AssertionError(f"{budget}s 未抬腿: {eng.status()} frozen={eng.frozen}")


def test_go_from_rest_starts_at_turn_pointer():
    """起步对轮次（修前实锤：静止 0.1/0.8/1.5/2.2/2.9/3.4s 后按 w 首抬分别
    是 L2/L3/R1/R2/R3/L1——完全随按键时刻在周期里的落点漂移）：任意静止
    时长后起步，首抬恒为轮次指针 step_leg；单步与连续行走续接同一份轮次
    （单步走完 L1 再按 w，首抬必是 L2）；单步受理即当拍启动不再等窗。"""
    for idle in (0.1, 1.5, 2.9):
        io, ctl, eng = make_engine()
        start(eng)
        run(eng, idle)                         # 静止晾着（钟空转过窗）
        first = eng.slot_order[0]
        assert _first_lifted(eng) == first, f"idle={idle}s 首抬非 {first}"
    # 单步走完首腿后连续起步：续接轮次，首抬窗序第二腿
    io, ctl, eng = make_engine()
    start(eng)
    assert _run_single_step(eng) == {eng.slot_order[0]}
    run(eng, 2.2)                              # 静止晾着：轮次不许漂
    assert _first_lifted(eng) == eng.slot_order[1]
    # 单步受理即当拍启动（旧口径等窗空转最多 ~3.6s）
    io, ctl, eng = make_engine()
    start(eng)
    leg = eng.slot_order[0]
    assert eng.request_step(30.0, 0.0, 0.0) is None
    for _ in range(int(0.5 / DT)):
        eng.update(DT)
        if eng.phase_of[leg] != LegPhase.STANCE:
            break
    assert eng.phase_of[leg] != LegPhase.STANCE


class _ShallowIO(MockVacuumIO):
    """罐压/盘压被钳在 floor 之上的仿真：吸得上（-30 可过、-40 罐压就绪线
    可过）但压不深——复现"唇口微漏/泵弱，盘压压不进门槛"的实机工况。"""

    def __init__(self, n, floor):
        super().__init__(n)
        self.floor = floor

    def step(self, dt):
        super().step(dt)
        self.tank_kpa = max(self.tank_kpa, self.floor)
        for i in range(self.n):
            self.foot_kpa[i] = max(self.foot_kpa[i], self.floor)


def test_lift_gate_holds_until_supports_deep():
    """抬腿门槛（08-19 实测 R2 刚过 -30 R3 就抬、R2/R3 抬腿整机下坠的回归）：
    互锁（ATTACHED）过了但支撑盘压未深于 lift_gate_kpa 时窗头不放行、整机
    静止等泵，gate_wait 外显；压深后当即放行抬腿，全程不冻结。"""
    io = _ShallowIO(6, -41.0)
    ctl = AdhesionController(io)
    eng = ClimbEngine(CFG, ctl)
    start(eng)
    run(eng, 3.0, vx=30.0)
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())   # 拒抬
    assert eng.gate_wait is not None and eng.frozen is None
    lifted_by = eng.gate_wait[0]
    assert eng.gate_wait[1]                    # 点名软腿非空
    io.floor = -100.0                          # 泵拽下去了
    for _ in range(int(5 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if any(p != LegPhase.STANCE for p in eng.phase_of.values()):
            break
    swinging = [n for n in LEG_NAMES if eng.phase_of[n] != LegPhase.STANCE]
    assert swinging == [lifted_by]             # 压深后放行的正是被拦的那条
    assert eng.frozen is None and eng.gate_wait is None


def test_lift_gate_timeout_freezes_naming_soft_leg():
    """门槛等待超时 -> 冻结报警点名压不下去的腿（不能静默停摆）。"""
    io = _ShallowIO(6, -41.0)
    ctl = AdhesionController(io)
    eng = ClimbEngine(replace(CFG, lift_gate_timeout_s=2.0), ctl)
    start(eng)
    run(eng, 4.0, vx=30.0)
    assert eng.frozen and "门槛" in eng.frozen and "kPa" in eng.frozen
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())


def test_cup_tilt_trim_pulls_station_inward():
    """吸盘轴垂直度实测修正（08-19 地面观察：模型垂直站位上实机轴仍外倾，
    落点内收更垂直）：正修正角把站位半径内收（~2mm/°），修正后模型轴向角
    在新站位上恰为零（垂直解自洽）；落点带与步幅上限同步内收。"""
    cfg3 = replace(CFG, cup_tilt_trim_deg=3.0)
    io = MockVacuumIO(6)
    eng0 = ClimbEngine(CFG, AdhesionController(io))
    eng3 = ClimbEngine(cfg3, AdhesionController(io))
    leg = CFG.leg("L1")
    r0 = math.hypot(eng0.default_feet["L1"][0] - leg.mount_x,
                    eng0.default_feet["L1"][1] - leg.mount_y)
    r3 = math.hypot(eng3.default_feet["L1"][0] - leg.mount_x,
                    eng3.default_feet["L1"][1] - leg.mount_y)
    assert r0 - 12.0 < r3 < r0 - 3.0           # 3° 收 ~6mm 量级
    z_press = -cfg3.stand_height - leg.press_delta_mm
    assert abs(_press_tilt(cfg3, r3, z_press)) < 1e-4   # 新站位垂直自洽
    lo0, hi0 = eng0._r_band["L1"]
    lo3, hi3 = eng3._r_band["L1"]
    assert hi3 < hi0 and lo3 < lo0             # 落点带整体内移
    # 步幅上限随正修正微涨而非缩：倾角-半径曲线站位附近平缓（~0.5°/mm）、
    # 带边陡（~0.63°/mm），同样 -3° 平移下站位内收得比带边多，外摆余量净增
    cap0, cap3 = max_straight_step(CFG), max_straight_step(cfg3)
    assert cap0 < cap3 < cap0 + 8.0, (cap0, cap3)


def test_max_straight_step_geometry():
    """直行步幅几何上限（--max-step 验证口径，落点唇口 ≤11.5° 工作带 +
    支撑尾端包络双口径联合解）：站高 90 ≈66，站高 62 ≈79——站矮唇口角随
    步幅涨得慢且包络余量大，上限显著抬高。"""
    cap90 = max_straight_step(CFG)
    assert 63.0 <= cap90 <= 68.0, cap90
    cap62 = max_straight_step(replace(CFG, stand_height=62.0))
    assert 76.0 <= cap62 <= 82.0, cap62
    assert cap62 > cap90 + 8.0


def test_big_step_low_stand_worst_case_stays_safe():
    """大步幅实验工况（--stand-height 62 --max-step 78 --sag-comp 4）最坏
    路径：R3 三级加深启动 + 满速直行 60s——全程不冻结、支撑 d 不贴 IK
    边界；角腿落点半径越过旧书写上限 190（证明 _R_BRACKET 拓宽真实生效、
    落点没再被假边界裁短）且不超 12° 裁剪带。"""
    cfg = replace(CFG, stand_height=62.0, climb_max_step=78.0,
                  climb_sag_comp_mm=4.0)
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(cfg, ctl)
    io.sealed[5] = False                      # R3 落脚吸不紧
    for _ in range(int(60 / DT)):
        eng.update(DT)
        if eng.retries["R3"] >= 3:
            io.sealed[5] = True               # 第三级加深后才贴住
        if eng.started:
            break
    assert eng.started
    bot = Hexapod(MockDriver(), cfg)
    d_max = cfg.femur_len + cfg.tibia_len
    worst_d = worst_land = 0.0
    for _ in range(int(60 / DT)):
        targets = eng.update(DT, 30.0, 0.0, 0.0)
        assert eng.frozen is None, f"冻结: {eng.frozen}"
        bot.pulses(targets)                   # 全部目标必须可解
        for n in LEG_NAMES:
            leg = cfg.leg(n)
            x, y, z = targets[n]
            r = math.hypot(x - leg.mount_x, y - leg.mount_y)
            worst_d = max(worst_d, math.hypot(r - cfg.coxa_len, z))
            if eng.phase_of[n] == LegPhase.DESCEND:
                lx, ly = eng.landing[n]
                worst_land = max(worst_land, math.hypot(lx - leg.mount_x,
                                                        ly - leg.mount_y))
    assert worst_d <= d_max - 1.0, f"d={worst_d:.1f} 贴死 {d_max:.1f}"
    r_lo, r_hi = eng._r_band["L1"]
    assert worst_land > 190.0, f"落点 r={worst_land:.1f} 没越过旧书写上限"
    assert worst_land <= r_hi + 1e-6


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


def test_lean_moves_body_and_respects_envelope():
    """倾身（body_lean）：请求量匀速铺完（LEAN_SPEED_MMS），支撑足整体
    -X 平移=身体前移；授予量按 IK 包络（D_SAFE_MARGIN 同口径）截短，
    到边拒绝；极限位形全部 IK 可解；反向仍可退回。"""
    from hexapod.climb import LEAN_SPEED_MMS
    io, ctl, eng = make_engine()
    start(eng)
    x0 = {n: eng.foot[n][0] for n in LEG_NAMES}
    granted, deny = eng.request_lean(30.0)
    assert deny is None and granted == 30.0
    # 匀速铺设：半程时刻应约走一半（不是阶跃）
    run(eng, 15.0 / LEAN_SPEED_MMS)
    assert abs(eng.lean_mm - 15.0) < 2.0
    run(eng, 15.0 / LEAN_SPEED_MMS + 1.0)
    assert math.isclose(eng.lean_mm, 30.0, abs_tol=1e-6)
    assert abs(eng.lean_pending) < 1e-9
    for n in LEG_NAMES:
        assert math.isclose(eng.foot[n][0], x0[n] - 30.0, abs_tol=1e-6)
    # 贪心要一大截：授予到包络余量为止，铺完后全部在包络内且 IK 可解
    g2, d2 = eng.request_lean(500.0)
    assert d2 is None and 0.0 < g2 < 500.0
    run(eng, g2 / LEAN_SPEED_MMS + 1.0)
    d_safe = CFG.femur_len + CFG.tibia_len - 3.0
    for n in LEG_NAMES:
        leg = CFG.leg(n)
        r = math.hypot(eng.foot[n][0] - leg.mount_x,
                       eng.foot[n][1] - leg.mount_y)
        assert math.hypot(r - CFG.coxa_len, eng.foot[n][2]) <= d_safe + 1e-6
    Hexapod(MockDriver(), CFG).pulses(eng.targets())   # 极限位形可解
    g3, d3 = eng.request_lean(5.0)
    assert g3 == 0.0 and d3                            # 到边拒绝
    g4, d4 = eng.request_lean(-20.0)                   # 反向仍可
    assert d4 is None and g4 == -20.0
    run(eng, 20.0 / LEAN_SPEED_MMS + 1.0)
    assert math.isclose(eng.lean_mm, 30.0 + g2 - 20.0, abs_tol=1e-6)


def test_lift_in_place_hover_and_land_back():
    """原地抬起（request_lift 零速单步）：互锁照走、VENT 先通气、落点=
    默认站位、其余支撑足纹丝不动（零速无支撑场平移）；step_land 落回
    压入位并重新吸附。"""
    io, ctl, eng = make_engine()
    start(eng)
    others0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES if n != "L2"}
    assert eng.request_lift("L2") is None
    assert eng.request_lift("L1") == "已有单步在途"
    seen_vent = False
    for _ in range(int(20 / DT)):
        eng.update(DT)
        seen_vent = seen_vent or eng.phase_of["L2"] == LegPhase.VENT
        if eng.phase_of["L2"] == LegPhase.HOVER:
            break
    assert eng.phase_of["L2"] == LegPhase.HOVER and seen_vent
    x0, y0, _ = eng.default_feet["L2"]
    assert math.isclose(eng.foot["L2"][0], x0, abs_tol=1e-6)
    assert math.isclose(eng.foot["L2"][1], y0, abs_tol=1e-6)
    assert math.isclose(eng.foot["L2"][2], -CFG.stand_height + CFG.lift_clearance,
                        abs_tol=1e-6)
    for n, p in others0.items():                     # 零速：支撑足不动
        assert all(math.isclose(a, b, abs_tol=1e-9)
                   for a, b in zip(eng.foot[n], p)), n
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L2"] == LegPhase.STANCE and not eng.step_active:
            break
    assert ctl.is_attached(LEG_NAMES.index("L2"))
    assert math.isclose(eng.foot["L2"][2],
                        -CFG.stand_height - CFG.leg("L2").press_delta_mm)
    assert eng.select_step_leg("L9") == "未知腿 L9"


def test_lean_pauses_during_swing_resumes_after():
    """倾身×抬落互斥：摆动/落压期间（非 STANCE/HOVER）倾身暂停不丢，
    悬停中恢复铺设（悬停腿不平移），落回支撑后铺完剩余量。"""
    io, ctl, eng = make_engine()
    start(eng)
    assert eng.request_lift("R1") is None
    g, d = eng.request_lean(20.0)
    assert d is None and g == 20.0
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["R1"] == LegPhase.HOVER:
            break
        assert eng.lean_mm == 0.0            # 摆动中倾身冻结
    assert eng.phase_of["R1"] == LegPhase.HOVER
    hover_xy = tuple(eng.foot["R1"][:2])
    run(eng, 1.0)                            # 悬停中恢复铺设 ~10mm
    h = eng.lean_mm
    assert h > 5.0
    assert tuple(eng.foot["R1"][:2]) == hover_xy   # 悬停腿不随支撑平移
    assert eng.step_land() is None
    while eng.phase_of["R1"] != LegPhase.STANCE:
        assert math.isclose(eng.lean_mm, h, abs_tol=1e-9)  # 落压期间冻结
        eng.update(DT)
    run(eng, 3.0)
    assert math.isclose(eng.lean_mm, 20.0, abs_tol=1e-6)
    # R1 落回站位后只吃到后半段倾身，其余腿吃满 20
    assert eng.foot["R1"][0] > eng.default_feet["R1"][0] - 20.0
    assert math.isclose(eng.foot["L2"][0], eng.default_feet["L2"][0] - 20.0,
                        abs_tol=1e-6)


def test_clear_freeze_cancels_pending_lean():
    """冻结解除时未铺完的倾身必须取消（与挂起单步同口径）：解冻后不许
    自行续倾。"""
    io, ctl, eng = make_engine()
    start(eng)
    eng.request_lean(30.0)
    run(eng, 0.5)
    done = eng.lean_mm
    assert 0.0 < done < 30.0
    eng.frozen = "测试注入"
    run(eng, 1.0)                            # 冻结中不推进
    assert eng.lean_mm == done
    eng.clear_freeze()
    assert eng.lean_pending == 0.0
    before = {n: eng.foot[n][0] for n in LEG_NAMES}
    run(eng, 2.0)
    assert all(math.isclose(eng.foot[n][0], before[n]) for n in LEG_NAMES)
    g, d = eng.request_lean(-10.0)           # 解冻后新请求照常受理
    assert d is None and g == -10.0


# ---------- vent 前零力交接（docs/HANDOVER-DESIGN.md，08-20 弹跳棘轮治法） ----------

HO = {"L1": 17.0, "R1": 15.0, "L3": 11.0, "R3": 9.0, "R2": 5.0, "L2": 5.0}


def make_ho_engine(deltas=None):
    """带零力交接的引擎（默认 δ=HANDOVER-DESIGN §5 起标表）。"""
    cfg = replace(CFG, legs=tuple(
        replace(l, handover_mm=(deltas or HO).get(l.name, 0.0))
        for l in CFG.legs))
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    return cfg, io, ctl, ClimbEngine(cfg, ctl)


def test_handover_holds_attached_then_vents_on_schedule():
    """相位序列（§7.2）：δ>0 时窗头决策后进 HANDOVER，期间吸附保持
    ATTACHED、阀不动（通罐位）；δ/HANDOVER_SPEED_MMS 秒（±3 DT）铺完
    才 request_release 进 VENT（照抄 test_vent_before_lift 的结构）。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    leg = eng.slot_order[0]                            # 首窗腿（现 R3）
    fi = LEG_NAMES.index(leg)
    for _ in range(int(5 / DT)):
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.phase_of[leg] == LegPhase.HANDOVER:
            break
    assert eng.phase_of[leg] == LegPhase.HANDOVER
    t_ho = 0.0
    while eng.phase_of[leg] == LegPhase.HANDOVER:
        assert ctl.state[fi] == FootState.ATTACHED     # 交接期保持密封吸附
        assert io.valve[fi]                            # 阀不动（通罐位）
        eng.update(DT, 30.0, 0.0, 0.0)
        t_ho += DT
    assert eng.phase_of[leg] == LegPhase.VENT          # 铺完才放气
    assert ctl.state[fi] == FootState.VENTING
    assert abs(t_ho - HO[leg] / HANDOVER_SPEED_MMS) < 3 * DT


def test_handover_displacement_account_zero_sum():
    """位移账（§7.3）：交接完成时被抬腿 = 起点 + 上坡 δ，每条支撑腿 =
    起点 + 下坡 δ/5（头朝上贴墙 _down=(-1,0)：上坡=+X），y 不受扰、z 恒在
    压入位；六腿位移矢量和 = 0（均值不变 = 身体指令不动的代数断言）。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    f0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    assert eng.request_lift("L1") is None              # δ=17，零速原地
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.VENT:
            break
    assert eng.phase_of["L1"] == LegPhase.VENT         # 交接刚完成、还没抬
    d = HO["L1"]
    assert math.isclose(eng.foot["L1"][0], f0["L1"][0] + d, abs_tol=1e-6)
    assert math.isclose(eng.foot["L1"][1], f0["L1"][1], abs_tol=1e-9)
    assert math.isclose(eng.foot["L1"][2], f0["L1"][2], abs_tol=1e-9)
    vec = [eng.foot["L1"][0] - f0["L1"][0], eng.foot["L1"][1] - f0["L1"][1]]
    for n in LEG_NAMES:
        if n == "L1":
            continue
        assert math.isclose(eng.foot[n][0], f0[n][0] - d / 5, abs_tol=1e-6)
        assert math.isclose(eng.foot[n][1], f0[n][1], abs_tol=1e-9)
        assert math.isclose(eng.foot[n][2], f0[n][2], abs_tol=1e-9)  # z 恒压入位
        vec[0] += eng.foot[n][0] - f0[n][0]
        vec[1] += eng.foot[n][1] - f0[n][1]
    assert abs(vec[0]) < 1e-6 and abs(vec[1]) < 1e-9


def test_handover_follows_support_field_while_walking():
    """行走叠加（§7.4）：带速度走时 HANDOVER 期间被抬腿跟随支撑场（不随
    场 = 墙面系被拖着划、蹭移还密封着的唇口），交接分量叠加在场平移之上：
    被抬腿 = 随场 −vΔt + 上坡 δ；支撑腿 = 随场 −vΔt + 下坡 δ/5。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    leg = eng.slot_order[0]                            # R3，δ=9
    vx_eff = eng._clamp_speed(30.0, 0.0, 0.0)[0]
    t0 = f0 = None
    for _ in range(int(10 / DT)):
        t_pre = eng.t
        feet_pre = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
        eng.update(DT, 30.0, 0.0, 0.0)
        if eng.phase_of[leg] == LegPhase.HANDOVER:
            t0, f0 = t_pre, feet_pre
            break
    assert t0 is not None
    obs = next(n for n in LEG_NAMES if n != leg)
    for _ in range(int(10 / DT)):
        if eng.phase_of[leg] != LegPhase.HANDOVER:
            break
        eng.update(DT, 30.0, 0.0, 0.0)
    assert eng.phase_of[leg] == LegPhase.VENT
    d = HO[leg]
    dt_clock = eng.t - t0                              # 场只随钟走（钟停在窗尾）
    assert dt_clock > 0.0
    assert math.isclose(eng.foot[leg][0] - f0[leg][0],
                        -vx_eff * dt_clock + d, abs_tol=1e-6)
    assert math.isclose(eng.foot[obs][0] - f0[obs][0],
                        -vx_eff * dt_clock - d / 5, abs_tol=1e-6)
    assert math.isclose(eng.foot[obs][1], f0[obs][1], abs_tol=1e-9)


def test_handover_pauses_on_support_leak_and_resumes():
    """漏气暂停（§7.5）：交接中支撑腿漏气，_ho_left 冻结不推进、目标全
    保持、不放气（漏着的盘摩擦余量低不该被推）；挽救成功后续铺完成。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    assert eng.request_lift("L1") is None
    for _ in range(int(10 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HANDOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HANDOVER
    run(eng, 0.3)                                      # 先铺一小段
    ri = LEG_NAMES.index("R2")
    io.sealed[ri] = False                              # 支撑腿漏气
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if ctl.is_leaking(ri):
            break
    assert ctl.is_leaking(ri)
    left0 = eng._ho_left
    assert 0.0 < left0 < HO["L1"]
    feet0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    run(eng, 0.5)                                      # 挽救窗（2s）内
    assert eng.frozen is None
    assert eng._ho_left == left0                       # 交接暂停不丢
    assert eng.phase_of["L1"] == LegPhase.HANDOVER
    assert ctl.state[LEG_NAMES.index("L1")] == FootState.ATTACHED  # 没放气
    assert all(tuple(eng.foot[n]) == feet0[n] for n in LEG_NAMES)
    io.sealed[ri] = True                               # 挽救成功
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if not ctl.is_leaking(ri):
            break
    assert not ctl.is_leaking(ri)
    for _ in range(int(30 / DT)):                      # 续铺完成、走到悬停
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HOVER and eng.frozen is None


def test_handover_survives_freeze_and_resumes():
    """冻结恢复（§7.6）：交接中 frozen 注入，目标与 _ho_left 保持；
    clear_freeze 不取消在途交接（与摆动中 step_active 保留同口径），解冻
    续铺、完成、正常走完抬落（对比 test_single_step_hover_survives_freeze）。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    assert eng.request_lift("L1") is None
    for _ in range(int(10 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HANDOVER:
            break
    run(eng, 0.4)
    assert eng.phase_of["L1"] == LegPhase.HANDOVER
    left0 = eng._ho_left
    assert 0.0 < left0 < HO["L1"]
    eng.frozen = "测试注入"
    feet0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    run(eng, 1.0)
    assert eng._ho_left == left0                       # 冻结：交接保持不丢
    assert all(tuple(eng.foot[n]) == feet0[n] for n in LEG_NAMES)
    eng.clear_freeze()
    assert eng.step_active                             # 在途抬起不被取消
    for _ in range(int(30 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HOVER
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.STANCE and not eng.step_active:
            break
    assert ctl.is_attached(LEG_NAMES.index("L1")) and eng.frozen is None


def test_handover_direction_rotates_with_heading():
    """航向旋转（§7.7）：wz 转过 θ 后交接方向随 _down 旋转（与 sag_comp
    同一套航向推算）——被抬腿沿 −_down（上坡）δ、支撑沿 +_down δ/5，
    照抄 test_sag_comp_downhill_rotates_with_heading 的口径。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    run(eng, 15.0, wz=0.5)                             # 步步带交接地转航向
    for _ in range(int(15 / DT)):                      # 停走，等在途摆动收口
        eng.update(DT)
        if all(p == LegPhase.STANCE for p in eng.phase_of.values()):
            break
    assert all(p == LegPhase.STANCE for p in eng.phase_of.values())
    dxn, dyn = eng._down
    assert abs(dyn) > 0.03                             # 真转了角度才算数
    f0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    assert eng.request_lift("L1") is None
    for _ in range(int(30 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.VENT:
            break
    assert eng.phase_of["L1"] == LegPhase.VENT
    d = HO["L1"]
    assert math.isclose(eng.foot["L1"][0] - f0["L1"][0], -dxn * d, abs_tol=1e-6)
    assert math.isclose(eng.foot["L1"][1] - f0["L1"][1], -dyn * d, abs_tol=1e-6)
    obs = "R2"
    assert math.isclose(eng.foot[obs][0] - f0[obs][0], dxn * d / 5, abs_tol=1e-6)
    assert math.isclose(eng.foot[obs][1] - f0[obs][1], dyn * d / 5, abs_tol=1e-6)


def test_handover_lift_in_place_solvable_at_max_delta():
    """极限 δ 全程 IK 可解（§7.8）：request_lift + δ=20（>21 实测封顶前的
    最大标定值）走完整抬落，全程 pulses 可解、支撑足除交接分量（下坡 δ/5）
    外纹丝不动（对比 test_lift_in_place_hover_and_land_back）。"""
    cfg, io, ctl, eng = make_ho_engine({n: 20.0 for n in LEG_NAMES})
    start(eng)
    bot = Hexapod(MockDriver(), cfg)
    others0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES if n != "L2"}
    assert eng.request_lift("L2") is None
    for _ in range(int(30 / DT)):
        bot.pulses(eng.update(DT))                     # 全程可解
        assert eng.frozen is None
        if eng.phase_of["L2"] == LegPhase.HOVER:
            break
    assert eng.phase_of["L2"] == LegPhase.HOVER
    x0, y0, _ = eng.default_feet["L2"]                 # 落点=默认站位
    assert math.isclose(eng.foot["L2"][0], x0, abs_tol=1e-6)
    assert math.isclose(eng.foot["L2"][1], y0, abs_tol=1e-6)
    for n, p in others0.items():                       # 支撑足只动交接分量
        assert math.isclose(eng.foot[n][0], p[0] - 20.0 / 5, abs_tol=1e-6), n
        assert math.isclose(eng.foot[n][1], p[1], abs_tol=1e-9)
        assert math.isclose(eng.foot[n][2], p[2], abs_tol=1e-9)
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        bot.pulses(eng.update(DT))
        if eng.phase_of["L2"] == LegPhase.STANCE and not eng.step_active:
            break
    assert ctl.is_attached(LEG_NAMES.index("L2"))
    assert math.isclose(eng.foot["L2"][2],
                        -CFG.stand_height - CFG.leg("L2").press_delta_mm)


def test_parse_handover_formats_and_bounds():
    """δ 解析（§7.9）：统一值/逐腿两种格式（腿名不分大小写、未给的腿 0）、
    越界与非法输入拒绝。"""
    assert parse_handover("8") == {n: 8.0 for n in LEG_NAMES}
    got = parse_handover("L1:17, r3:9")
    assert got["L1"] == 17.0 and got["R3"] == 9.0
    assert all(got[n] == 0.0 for n in LEG_NAMES if n not in ("L1", "R3"))
    assert parse_handover("0") == {n: 0.0 for n in LEG_NAMES}   # 0=关，合法
    for bad in ("26", "-1", "L1:26", "L1:-1", "X9:5", "L1:x", "L1:", "abc", ""):
        try:
            parse_handover(bad)
        except ValueError:
            continue
        raise AssertionError(f"{bad!r} 应被拒绝")


def _lift_and_land(eng, leg, timeout_s=30.0):
    """request_lift 走完整抬落回支撑（测试助手）。"""
    assert eng.request_lift(leg) is None
    for _ in range(int(timeout_s / DT)):
        eng.update(DT)
        if eng.phase_of[leg] == LegPhase.HOVER:
            break
    assert eng.phase_of[leg] == LegPhase.HOVER, eng.frozen
    assert eng.step_land() is None
    for _ in range(int(timeout_s / DT)):
        eng.update(DT)
        if eng.phase_of[leg] == LegPhase.STANCE and not eng.step_active:
            return
    raise AssertionError(f"{leg} 落地未收口: {eng.status()} frozen={eng.frozen}")


def test_handover_weighted_allocation_by_rotation_distance():
    """--handover-weights：交接载荷按窗序轮转距离分配（w[0]=最晚才轮到抬的
    支撑腿=刚抬过的，w[4]=下一个要抬的）。窗序 R3→L1→R2→L3→R1→L2：抬 L1
    时晚→先 = R3,L2,R1,L3,R2——**首抬就正确**（第一版按落地新旧排，当时
    吸附序还是 L1..R3 把首轮分错；现吸附序也=窗序，但排序不依赖吸附历史
    这点不变）；再抬窗序继任者 R2 验证随抬腿换位（权重只在按轮转抬时套用
    ——轮转口径守卫见下一测试）。任意归一化权重下六腿位移矢量和 = 0
    （均值不变=身体指令不动）照旧成立。"""
    w = (0.6, 0.25, 0.1, 0.05, 0.0)
    cfg = replace(CFG, handover_slot_w=w, legs=tuple(
        replace(l, handover_mm=10.0) for l in CFG.legs))
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(cfg, ctl)
    start(eng)
    d = 10.0
    f0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    assert eng.request_lift("L1") is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.VENT:
            break
    assert eng.phase_of["L1"] == LegPhase.VENT
    assert math.isclose(eng.foot["L1"][0], f0["L1"][0] + d, abs_tol=1e-6)
    for n, share in (("R3", 0.6), ("L2", 0.25), ("R1", 0.1),
                     ("L3", 0.05), ("R2", 0.0)):
        assert math.isclose(eng.foot[n][0], f0[n][0] - d * share,
                            abs_tol=1e-6), n
        assert math.isclose(eng.foot[n][1], f0[n][1], abs_tol=1e-9)
    for _ in range(int(20 / DT)):                  # 先到悬停才能落地
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HOVER
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.STANCE and not eng.step_active:
            break
    assert eng.phase_of["L1"] == LegPhase.STANCE
    f1 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    assert eng.request_lift("R2") is None      # L1 的窗序继任者=R2
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["R2"] == LegPhase.VENT:
            break
    assert eng.phase_of["R2"] == LegPhase.VENT
    for n, share in (("L1", 0.6), ("R3", 0.25), ("L2", 0.1),
                     ("R1", 0.05), ("L3", 0.0)):
        assert math.isclose(eng.foot[n][0], f1[n][0] - d * share,
                            abs_tol=1e-6), n
    tot = sum(eng.foot[n][0] - f1[n][0] for n in LEG_NAMES)
    assert abs(tot) < 1e-6
    assert eng.handover_note is None           # 全程按轮转，守卫不应出手


def test_handover_weights_off_rotation_fall_back_to_equal():
    """轮转口径守卫（审核）：开权重时反复抬同一条腿（δ 标定工作流），第二次
    起偏离轮转——若照套权重，w[0]=0.6δ 每次砸向同一条支撑腿（抬 L1 时恒为
    R3）且落地复位只清被抬腿自己，实测 4~6 次就把 R3 推出工作空间。守卫：
    偏离轮转的交接退回均分 δ/5（帮助文案的 ~10 次预算照旧成立），
    handover_note 留痕；回到轮转（抬窗序继任者）时权重恢复。"""
    w = (0.6, 0.25, 0.1, 0.05, 0.0)
    cfg = replace(CFG, handover_slot_w=w, legs=tuple(
        replace(l, handover_mm=10.0) for l in CFG.legs))
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(cfg, ctl)
    start(eng)
    d = 10.0
    _lift_and_land(eng, "L1")                      # 首抬：权重（上一测试已验）
    assert eng.handover_note is None
    f1 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    assert eng.request_lift("L1") is None          # 再抬同一腿=偏离轮转
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.VENT:
            break
    assert eng.phase_of["L1"] == LegPhase.VENT
    for n in LEG_NAMES:                            # 本次均分 δ/5，不是 0.6δ
        if n == "L1":
            continue
        assert math.isclose(eng.foot[n][0], f1[n][0] - d / 5, abs_tol=1e-6), n
    assert eng.handover_note and "偏离轮转" in eng.handover_note
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HOVER:
            break
    assert eng.step_land() is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.STANCE and not eng.step_active:
            break
    f2 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    assert eng.request_lift("R2") is None          # 回到轮转：权重恢复
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["R2"] == LegPhase.VENT:
            break
    assert eng.phase_of["R2"] == LegPhase.VENT
    assert math.isclose(eng.foot["L1"][0], f2["L1"][0] - d * 0.6, abs_tol=1e-6)


def test_parse_handover_weights_formats_and_bounds():
    """--handover-weights 解析：5 元、自动归一化、负值/零和/错个数拒绝。"""
    from hexapod.climb import parse_handover_weights
    w = parse_handover_weights("0.6,0.25,0.1,0.05,0")
    assert all(math.isclose(a, b, abs_tol=1e-12)
               for a, b in zip(w, (0.6, 0.25, 0.1, 0.05, 0.0)))
    w2 = parse_handover_weights("2,1,1,0,0")           # 自动归一化
    assert math.isclose(sum(w2), 1.0) and math.isclose(w2[0], 0.5)
    # nan/inf 必须显式拒绝（审核）：NaN 参与 </<= 比较全 False 会溜过负值与
    # 零和检查，归一化产出 NaN 份额——下游 leg_ik 对 NaN 不抛 WorkspaceError、
    # joint_deg_to_us 把 NaN 钳成 min_us，15 路支撑舵机静默打到 500µs 端点
    for bad in ("0.6,0.25", "1,1,1,1,1,1", "1,1,1,1,-1", "0,0,0,0,0",
                "a,b,c,d,e", "", "nan,1,1,1,1", "1,1,1,1,inf",
                "1e999,1,1,1,1"):
        try:
            parse_handover_weights(bad)
        except ValueError:
            continue
        raise AssertionError(f"{bad!r} 应被拒绝")


def test_handover_slot_w_config_path_validated_and_normalized():
    """引擎口对直设 config 的权重再验+归一化（审核）：CLI 之外的路径原先
    既不验也不归一——(1,1,1,1,1) 会让每次抬腿六腿位移和 = -4δ（身体指令净
    下坡），静默破坏只有 parse_handover_weights 才保证的零和不变量。"""
    for bad in ((1.0, 1.0, 1.0, 1.0, -1.0), (0.0,) * 5,
                (float("nan"), 1.0, 1.0, 1.0, 1.0), (1.0, 1.0, 1.0)):
        try:
            ClimbEngine(replace(CFG, handover_slot_w=bad),
                        AdhesionController(MockVacuumIO(6)))
        except ValueError:
            continue
        raise AssertionError(f"{bad!r} 应被拒绝")
    cfg = replace(CFG, handover_slot_w=(2.0, 1.0, 1.0, 0.0, 0.0), legs=tuple(
        replace(l, handover_mm=10.0) for l in CFG.legs))
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = ClimbEngine(cfg, ctl)
    assert math.isclose(sum(eng._slot_w), 1.0)
    assert math.isclose(eng._slot_w[0], 0.5)
    start(eng)
    f0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    assert eng.request_lift("L1") is None
    for _ in range(int(20 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.VENT:
            break
    assert eng.phase_of["L1"] == LegPhase.VENT
    assert math.isclose(eng.foot["R3"][0], f0["R3"][0] - 10.0 * 0.5,
                        abs_tol=1e-6)
    tot = sum(eng.foot[n][0] - f0[n][0] for n in LEG_NAMES)
    assert abs(tot) < 1e-6                     # 归一化后零和照旧成立


def test_handover_leg_leak_pauses_ramp_and_resumes():
    """交接腿自身漏气纳入监护（审核）：原 _leak_watch 只认 STANCE，交接腿
    （密封承载且正被卸载）脱管最长 δ/速率 秒——唇口中途失效时 4.7 步继续
    从一只已不承载的脚"还力"，储能照样瞬释。修后：交接腿漏气同样触发
    leak_pause，_ho_left 冻结、目标保持、不放气；挽救成功续铺完成。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    assert eng.request_lift("L1") is None
    for _ in range(int(10 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HANDOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HANDOVER
    run(eng, 0.3)                                      # 先铺一小段
    li = LEG_NAMES.index("L1")
    io.sealed[li] = False                              # 交接腿自己漏
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if ctl.is_leaking(li):
            break
    assert ctl.is_leaking(li)
    left0 = eng._ho_left
    assert 0.0 < left0 < HO["L1"]
    feet0 = {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    run(eng, 0.5)                                      # 挽救窗（2s）内
    assert eng.frozen is None
    assert eng._ho_left == left0                       # 交接暂停不丢
    assert eng.phase_of["L1"] == LegPhase.HANDOVER     # 没放气
    assert all(tuple(eng.foot[n]) == feet0[n] for n in LEG_NAMES)
    io.sealed[li] = True                               # 挽救成功
    for _ in range(int(3 / DT)):
        eng.update(DT)
        if not ctl.is_leaking(li):
            break
    assert not ctl.is_leaking(li)
    for _ in range(int(30 / DT)):                      # 续铺完成、走到悬停
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HOVER and eng.frozen is None


def test_handover_leg_leak_rescue_timeout_freezes():
    """交接腿漏气挽救超时 -> 冻结报警点名交接腿（与支撑腿同款兜底）。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    assert eng.request_lift("L1") is None
    for _ in range(int(10 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HANDOVER:
            break
    run(eng, 0.3)
    io.sealed[LEG_NAMES.index("L1")] = False
    run(eng, 4.0)                                      # 超 leak_rescue_s=2
    assert eng.frozen and "L1" in eng.frozen and "漏气挽救" in eng.frozen


def test_handover_clips_at_envelope_instead_of_freezing():
    """越界截断（审核）：4.7 步原先是唯一没有包络钳制的支撑系推手——δ 偏大
    /反复抬同一腿累积漂移时把支撑腿推出 IK 包络 → WorkspaceError 冻结，且
    clear_freeze 保留在途交接，墙上 f 续铺同帧复冻=死循环。修后：任一腿的
    本 tick 新目标越出安全包络（同 _lean_room 口径）即截断剩余交接量提前
    放气——全程绝不冻结、逐 tick IK 可解，handover_note 留痕。用 δ=25 反复
    抬 L1 逼近边界（每次支撑各漂下坡 5mm，≈11 次触界）。"""
    cfg, io, ctl, eng = make_ho_engine({n: 25.0 for n in LEG_NAMES})
    start(eng)
    bot = Hexapod(MockDriver(), cfg)
    clipped_at = None
    for k in range(14):
        assert eng.request_lift("L1") is None
        for _ in range(int(30 / DT)):
            bot.pulses(eng.update(DT))                 # 全程可解、绝不冻结
            assert eng.frozen is None
            if eng.phase_of["L1"] == LegPhase.HOVER:
                break
        assert eng.phase_of["L1"] == LegPhase.HOVER, eng.status()
        assert eng.step_land() is None
        for _ in range(int(30 / DT)):
            bot.pulses(eng.update(DT))
            assert eng.frozen is None
            if eng.phase_of["L1"] == LegPhase.STANCE and not eng.step_active:
                break
        assert eng.phase_of["L1"] == LegPhase.STANCE
        if clipped_at is None and eng.handover_note \
                and "截断" in eng.handover_note:
            clipped_at = k + 1
            break
    assert clipped_at is not None, "14 次抬腿都没触发越界截断"
    assert clipped_at >= 4                             # 边界前的正常次数不受扰
    for n in LEG_NAMES:                                # 没有腿被推出安全包络
        assert eng._foot_xy_ok(n, eng.foot[n][0], eng.foot[n][1]), n


class _FloorIO(MockVacuumIO):
    """盘压钳在 floor 之上的仿真（可中途改 floor）：复现"交接期间支撑盘
    漏软到门槛(-50)与漏气绊线(-20)之间的监护盲区"。"""

    def __init__(self, n, floor=-100.0):
        super().__init__(n)
        self.floor = floor

    def step(self, dt):
        super().step(dt)
        self.tank_kpa = max(self.tank_kpa, self.floor)
        for i in range(self.n):
            self.foot_kpa[i] = max(self.foot_kpa[i], self.floor)


def test_handover_release_gate_rechecked_after_ramp():
    """放气前门槛复检（审核）：窗头门槛判定距 request_release 已过 δ/速率 秒
    ——期间支撑盘漏到 -50 门槛之上、-20 漏气绊线之下时，原实现照常对着软
    肩膀放气（重演 08-19 新盘塌事故类）。修后：铺完复检，不过则保持密封等
    泵（gate_wait 外显），恢复即放气；超时冻结报警。"""
    io = _FloorIO(6)
    ctl = AdhesionController(io)
    cfg = replace(CFG, legs=tuple(
        replace(l, handover_mm=HO.get(l.name, 0.0)) for l in CFG.legs))
    eng = ClimbEngine(cfg, ctl)
    start(eng)
    assert eng.request_lift("L1") is None              # δ=17，铺 1.7s
    for _ in range(int(10 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HANDOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HANDOVER
    io.floor = -41.0                                   # 盲区：过 -30/-20，欠 -50
    run(eng, 3.0)                                      # 铺完 + 等待
    li = LEG_NAMES.index("L1")
    assert eng._ho_left <= 1e-6                        # 铺是铺完了
    assert eng.phase_of["L1"] == LegPhase.HANDOVER     # 但拒放气
    assert ctl.state[li] == FootState.ATTACHED and io.valve[li]
    assert eng.gate_wait is not None and eng.gate_wait[0] == "L1"
    assert eng.gate_wait[1]                            # 点名软腿非空
    assert eng.frozen is None                          # 默认 15s 内不冻结
    io.floor = -100.0                                  # 泵拽回去了
    for _ in range(int(5 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.VENT:
            break
    assert eng.phase_of["L1"] == LegPhase.VENT         # 恢复即放气
    assert ctl.state[li] == FootState.VENTING


def test_handover_release_gate_timeout_freezes():
    """放气前门槛等待超时 -> 冻结报警（交接腿保持密封悬停，不静默停摆）。"""
    io = _FloorIO(6)
    ctl = AdhesionController(io)
    cfg = replace(CFG, lift_gate_timeout_s=2.0, legs=tuple(
        replace(l, handover_mm=HO.get(l.name, 0.0)) for l in CFG.legs))
    eng = ClimbEngine(cfg, ctl)
    start(eng)
    assert eng.request_lift("L1") is None
    for _ in range(int(10 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HANDOVER:
            break
    io.floor = -41.0
    run(eng, 5.0)
    assert eng.frozen and "放气门槛超时" in eng.frozen
    assert eng.phase_of["L1"] == LegPhase.HANDOVER     # 密封保持，没放气
    assert ctl.state[LEG_NAMES.index("L1")] == FootState.ATTACHED


def test_lean_room_reserves_inflight_handover():
    """_lean_room 预扣在途交接（审核）：交接 Z 段里授予的倾身要等 4.6 步
    恢复推进才执行，届时支撑腿可能已再吃进 份额×剩余交接量 的下坡位移——
    原实现只扣 _lean_left，Z 段授予的倾身执行时会超包络。"""
    cfg, io, ctl, eng = make_ho_engine()
    start(eng)
    room0 = eng._lean_room(1.0)
    assert eng.request_lift("L1") is None              # δ=17
    for _ in range(int(10 / DT)):
        eng.update(DT)
        if eng.phase_of["L1"] == LegPhase.HANDOVER:
            break
    assert eng.phase_of["L1"] == LegPhase.HANDOVER
    assert eng._ho_left > 10.0                         # 大头还没铺
    # 预扣 ≈ 剩余交接量 × 最大份额（均分 0.2）≈ 3.4mm
    assert eng._lean_room(1.0) <= room0 - 2.0
