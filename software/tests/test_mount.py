"""MountEngine（地-墙过渡引擎）测试：启动六足吸附 / 前腿上墙悬停-落下 /
俯仰铺设接触足钉死 / 不可行请求拒绝 / 收腿豁免互锁 / 吸附失败加深重试 /
零力交接与接管。
全部跑在 MockVacuumIO + MockDriver 上；每拍都过 Hexapod.move_feet 的真 IK，
足端目标出工作空间会当场抛 WorkspaceError。"""
import math
from dataclasses import replace

from hexapod.adhesion import AdhesionController, MockVacuumIO, FootState
from hexapod.mount import (MountEngine, MountPhase, FLOOR, b2w, w2b, _add,
                           TILT_BAND_DEG, HOLD_TILT_DEG, COXA_MAX_DEG,
                           PRESS_DEPTH_MAX)
from hexapod.config import DEFAULT_CONFIG as CFG, LEG_NAMES
from hexapod.driver import MockDriver
from hexapod.robot import Hexapod

DT = 0.02


def make(**kw):
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = MountEngine(CFG, ctl, **kw)
    bot = Hexapod(MockDriver(), CFG)
    return io, ctl, eng, bot


def run(eng, bot, seconds, until=None):
    for _ in range(int(seconds / DT)):
        bot.move_feet(eng.update(DT))
        if until is not None and until():
            return True
    return until is None


def start(eng, bot):
    assert run(eng, bot, 20.0, lambda: eng.started), \
        f"启动 20s 未完成 {eng.status()} frozen={eng.frozen}"


def idx(name):
    return LEG_NAMES.index(name)


def contact_world(eng, name):
    """接触腿指令目标换回世界系。"""
    return b2w(tuple(eng.foot[name]), eng.pose)


def test_startup_six_attached_on_floor_in_climb_stance():
    io, ctl, eng, bot = make()
    start(eng, bot)
    assert ctl.attached_count() == 6
    assert eng.pose == eng.pose0
    for n in LEG_NAMES:
        assert eng.surf[n] is FLOOR
        assert abs(eng.pw[n][2]) < 1e-9                      # 接触点在地面
        x0, y0, z0 = eng.default_feet[n]
        fx, fy, fz = eng.foot[n]
        assert math.isclose(fx, x0, abs_tol=1e-6) and math.isclose(fy, y0, abs_tol=1e-6)
        assert math.isclose(fz, z0 - CFG.leg(n).press_delta_mm, abs_tol=1e-6)   # 压入位
    assert eng.frozen is None


def test_front_leg_to_wall_hover_then_land():
    io, ctl, eng, bot = make()
    start(eng, bot)
    band = eng.wall_band("L1")
    assert band is not None and band[1] - band[0] >= 40     # 可落足带存在
    h = (band[0] + band[1]) / 2.0
    others = {n: tuple(eng.foot[n]) for n in LEG_NAMES if n != "L1"}
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", h)) is None
    assert eng.phase_of["L1"] == MountPhase.VENT            # 先通气再抬
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    fw = contact_world(eng, "L1")
    assert math.isclose(fw[0], -CFG.lift_clearance, abs_tol=1e-6)   # 墙前 15mm 悬停
    assert math.isclose(fw[2], h, abs_tol=1e-6)
    assert eng.land() is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    assert eng.surf["L1"] is eng.wall
    fw = contact_world(eng, "L1")
    assert math.isclose(fw[0], CFG.leg("L1").press_delta_mm, abs_tol=1e-6)  # 沿 +x 压入
    sol = eng.geom["L1"].solve(tuple(eng.foot["L1"]), (1.0, 0.0, 0.0))
    assert sol["tilt"] <= TILT_BAND_DEG
    assert abs(sol["gamma"]) <= COXA_MAX_DEG and sol["gamma"] < -40   # coxa 内摆指正前
    for n, p in others.items():                              # 其余腿纹丝不动
        assert tuple(eng.foot[n]) == p
    assert eng.frozen is None


def test_pitch_glide_keeps_contacts_fixed_in_world():
    io, ctl, eng, bot = make()
    start(eng, bot)
    before = {n: contact_world(eng, n) for n in LEG_NAMES}
    assert eng.request_pose(dpitch_deg=8.0, dz=5.0) is None
    assert run(eng, bot, 30.0, lambda: not eng.pose_pending)
    assert math.isclose(eng.pitch_deg, 8.0, abs_tol=1e-6)
    assert math.isclose(eng.pose[1], eng.pose0[1] + 5.0, abs_tol=1e-6)
    for n in LEG_NAMES:
        a, b = before[n], contact_world(eng, n)
        assert all(math.isclose(x, y, abs_tol=1e-6) for x, y in zip(a, b))
        sol = eng.geom[n].solve(tuple(eng.foot[n]), None)
        assert sol is not None
    assert eng.frozen is None


def test_pose_refused_when_infeasible_and_nothing_moves():
    io, ctl, eng, bot = make()
    start(eng, bot)
    pose, feet = eng.pose, {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    deny = eng.request_pose(dpitch_deg=60.0)
    assert isinstance(deny, str) and "不可行" in deny
    assert not eng.pose_pending
    run(eng, bot, 1.0)
    assert eng.pose == pose
    assert {n: tuple(eng.foot[n]) for n in LEG_NAMES} == feet


def test_tuck_exempt_from_interlock_but_fault_blocks():
    io, ctl, eng, bot = make()
    start(eng, bot)
    assert eng.request_tuck("L2") is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L2"] == MountPhase.AIR)
    assert eng.surf["L2"] is None
    fw = b2w(tuple(eng.foot["L2"]), eng.pose)
    assert fw[2] > 10.0                                      # 真在空中
    # 悬空腿不算接触腿：互锁只查其余接触腿，R1 可以动
    band = eng.wall_band("R1")
    tgt = eng.wall_target("R1", (band[0] + band[1]) / 2.0)
    assert eng.request_move("R1", eng.wall, tgt) is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["R1"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["R1"] == MountPhase.STANCE)
    # 接触腿掉了（FAULT）就拒
    ctl.state[idx("L3")] = FootState.FAULT
    deny = eng.request_move("L1", eng.wall, eng.wall_target("L1", tgt[2]))
    assert isinstance(deny, str) and "L3" in deny
    # 悬空腿在位姿铺设中随身体：俯仰后它的身体系目标不变、世界系抬高
    ctl.state[idx("L3")] = FootState.ATTACHED
    pb = tuple(eng.foot["L2"])
    assert eng.request_pose(dpitch_deg=5.0) is None
    assert run(eng, bot, 20.0, lambda: not eng.pose_pending)
    assert tuple(eng.foot["L2"]) == pb
    assert eng.frozen is None


def test_wall_target_out_of_band_refused():
    io, ctl, eng, bot = make()
    start(eng, bot)
    deny = eng.request_move("L1", eng.wall, eng.wall_target("L1", 600.0))
    assert isinstance(deny, str) and "L1" in deny
    assert eng.phase_of["L1"] == MountPhase.STANCE
    # 中腿在平身时对墙面外倾角 ~90°，落不了墙
    band = eng.wall_band("L2")
    assert band is None


def test_attach_fault_retries_deeper_then_succeeds():
    io, ctl, eng, bot = make()
    start(eng, bot)
    band = eng.wall_band("L1")
    h = (band[0] + band[1]) / 2.0
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", h)) is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    io.sealed[idx("L1")] = False
    assert eng.land() is None
    assert run(eng, bot, 15.0, lambda: eng.retries["L1"] >= 1)
    io.sealed[idx("L1")] = True
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    assert eng.depth["L1"] >= CFG.leg("L1").press_delta_mm + CFG.retry_deeper_mm
    assert eng.frozen is None


def test_commands_refused_while_busy():
    io, ctl, eng, bot = make()
    start(eng, bot)
    assert eng.request_pose(dpitch_deg=3.0) is None
    band = eng.wall_band("L1")
    deny = eng.request_move("L1", eng.wall, eng.wall_target("L1", band[0]))
    assert isinstance(deny, str) and "位姿" in deny
    eng.cancel_pose()
    assert not eng.pose_pending
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", band[0])) is None
    deny = eng.request_pose(dpitch_deg=3.0)
    assert isinstance(deny, str) and "L1" in deny
    deny = eng.request_tuck("R1")
    assert isinstance(deny, str) and "L1" in deny


def test_rear_leg_straight_back_on_floor():
    io, ctl, eng, bot = make()
    start(eng, bot)
    tgt = eng.floor_back("L3")
    assert abs((eng.hip_world("L3")[0] - tgt[0]) - eng.r0["L3"]) < 1e-9
    assert eng.request_move("L3", FLOOR, tgt) is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L3"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L3"] == MountPhase.STANCE
               and ctl.is_attached(idx("L3")))
    sol = eng.geom["L3"].solve(tuple(eng.foot["L3"]), (0.0, 0.0, -1.0))
    assert sol["tilt"] <= TILT_BAND_DEG and sol["gamma"] > 40   # coxa 后摆指正后
    assert eng.frozen is None


def test_set_wall_dist_before_start_shifts_world_frame():
    io, ctl, eng, bot = make(front_hip_to_wall=140.0)
    d0 = eng.front_hip_to_wall()
    band0 = eng.wall_band("L1")
    assert eng.set_wall_dist(160.0) is None
    assert math.isclose(eng.front_hip_to_wall(), 160.0, abs_tol=1e-9)
    assert math.isclose(d0, 140.0, abs_tol=1e-9)
    for n in LEG_NAMES:                                  # 接触点仍在地面、随身体平移
        assert abs(eng.pw[n][2]) < 1e-9
    assert eng.wall_band("L1") != band0
    start(eng, bot)
    assert ctl.attached_count() == 6
    assert isinstance(eng.set_wall_dist(150.0), str)     # 启动后不许改


def test_wall_trim_moves_hover_and_press_deeper_only_on_wall():
    io, ctl, eng, bot = make()
    start(eng, bot)
    band = eng.wall_band("L1")
    h = (band[0] + band[1]) / 2.0
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", h)) is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    floor_before = {n: tuple(eng.foot[n]) for n in LEG_NAMES if n != "L1"}
    assert eng.set_wall_trim(10.0) is None
    run(eng, bot, 0.1)
    fw = contact_world(eng, "L1")
    assert math.isclose(fw[0], -CFG.lift_clearance + 10.0, abs_tol=1e-6)   # 悬停点贴近墙 10
    for n, p in floor_before.items():                                        # 地面腿不动
        assert tuple(eng.foot[n]) == p
    assert eng.land() is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    fw = contact_world(eng, "L1")
    assert math.isclose(fw[0], CFG.leg("L1").press_delta_mm + 10.0, abs_tol=1e-6)  # 压入位也深 10
    # 够不到的修正量被拒、原值保留
    deny = eng.set_wall_trim(40.0)
    assert deny is None or isinstance(deny, str)
    if deny:
        assert math.isclose(eng.wall_trim["L1"], 10.0)
    assert eng.frozen is None


def test_wall_trim_allows_return_to_floor_and_no_jump_at_hover():
    """09-09 实机：修正 +16 吸在墙上后按 g 被拒'第 0 点足端撞墙面'——抬离点离模型墙面
    15−16<10；修正量应放宽墙面净空。且 --wall-trim 预置时 1 w 到位不能跳。"""
    io, ctl, eng, bot = make()
    start(eng, bot)
    assert eng.set_wall_trim(16.0) is None
    band = eng.wall_band("L1")
    h = (band[0] + band[1]) / 2.0
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", h)) is None
    prev = contact_world(eng, "L1")
    max_step = 0.0
    for _ in range(int(20 / DT)):
        bot.move_feet(eng.update(DT))
        cur = contact_world(eng, "L1")
        max_step = max(max_step, math.dist(prev, cur))
        prev = cur
        if eng.phase_of["L1"] == MountPhase.HOVER:
            break
    assert eng.phase_of["L1"] == MountPhase.HOVER
    assert max_step < 5.0                                     # 平移→悬停无跳变
    fw = contact_world(eng, "L1")
    assert math.isclose(fw[0], -CFG.lift_clearance + 16.0, abs_tol=1e-6)
    assert eng.land() is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    # 吸在墙上后回地：抬离点离模型墙面只有 −1，按修正放宽后必须受理
    assert eng.request_move("L1", FLOOR, eng.floor_home("L1")) is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and eng.surf["L1"] is FLOOR and ctl.is_attached(idx("L1")))
    assert eng.frozen is None


def test_attach_order_presses_legs_in_given_sequence():
    """诊断用：可疑腿排最后，其余五足吸牢当反力座再压它（09-09 L1 吸不上）。"""
    order = ("R3", "R2", "L3", "R1", "L2", "L1")
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = MountEngine(CFG, ctl, attach_order=order)
    bot = Hexapod(MockDriver(), CFG)
    assert eng.attach_order == order
    seen = []
    for _ in range(int(30 / DT)):
        bot.move_feet(eng.update(DT))
        for n in LEG_NAMES:
            if eng.phase_of[n] == MountPhase.PRESS and n not in seen:
                seen.append(n)
        if eng.started:
            break
    assert eng.started and ctl.attached_count() == 6
    assert tuple(seen) == order                      # 压入次序就是给的次序
    assert eng.frozen is None


def test_attach_order_rejects_bad_permutation():
    io = MockVacuumIO(6)
    for bad in (("L1",), ("L1", "L1", "L2", "L3", "R1", "R2"), ("L1", "X", "L2", "L3", "R1", "R2")):
        try:
            MountEngine(CFG, AdhesionController(io), attach_order=bad)
        except ValueError:
            continue
        raise AssertionError(f"{bad} 应被拒绝")


def test_wall_trim_is_per_leg():
    """09-09 实机：wall_dist 155 修正 16 时 L1 停在玻璃外 15mm、R1 已贴上——两只
    前腿的到达差 16mm，修正量必须逐腿，全局一个值会把 R1 按进玻璃。"""
    io, ctl, eng, bot = make()
    start(eng, bot)
    assert eng.set_wall_trim(16.0, ["L1"]) is None
    assert math.isclose(eng.wall_trim["L1"], 16.0)
    assert all(math.isclose(eng.wall_trim[n], 0.0) for n in LEG_NAMES if n != "L1")
    band = eng.wall_band("L1")
    h = (band[0] + band[1]) / 2.0
    for leg, want in (("L1", -CFG.lift_clearance + 16.0), ("R1", -CFG.lift_clearance)):
        assert eng.request_move(leg, eng.wall, eng.wall_target(leg, h)) is None
        assert run(eng, bot, 20.0, lambda: eng.phase_of[leg] == MountPhase.HOVER)
        fw = contact_world(eng, leg)
        assert math.isclose(fw[0], want, abs_tol=1e-6), f"{leg} 悬停 x={fw[0]}"
        assert eng.land() is None
        assert run(eng, bot, 20.0, lambda: eng.phase_of[leg] == MountPhase.STANCE
                   and ctl.is_attached(idx(leg)))
    # 压入位：L1 深 16、R1 不深
    assert math.isclose(contact_world(eng, "L1")[0],
                        CFG.leg("L1").press_delta_mm + 16.0, abs_tol=1e-6)
    assert math.isclose(contact_world(eng, "R1")[0],
                        CFG.leg("R1").press_delta_mm, abs_tol=1e-6)
    assert eng.set_wall_trim(1.0, ["ZZ"]) is not None      # 未知腿名被拒
    assert eng.frozen is None


def test_nudge_wall_target_moves_hover_height_only_while_hovering():
    """09-09 实机：L1 上墙吸住后机身前部被压沉，R1 悬停时比 L1 低且贴到玻璃——
    没有 IMU 只能悬停时按眼睛把落点挪齐平。"""
    io, ctl, eng, bot = make()
    start(eng, bot)
    band = eng.wall_band("L1")
    h = (band[0] + band[1]) / 2.0
    assert isinstance(eng.nudge_wall_target("L1", dz=5.0), str)     # 不在悬停：拒
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", h)) is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    z0 = contact_world(eng, "L1")[2]
    assert eng.nudge_wall_target("L1", dz=5.0) is None
    assert isinstance(eng.land(), str)                             # 铺设未完不许落下
    steps = []
    for _ in range(int(2.0 / DT)):
        bot.move_feet(eng.update(DT))
        steps.append(contact_world(eng, "L1"))
        if not eng.nudge_pending:
            break
    assert not eng.nudge_pending
    gaps = [math.dist(a, b) for a, b in zip(steps, steps[1:])]
    assert gaps and max(gaps) < 1.0                                # 匀速铺设，不跳变
    fw = contact_world(eng, "L1")
    assert math.isclose(fw[2], z0 + 5.0, abs_tol=1e-6)             # 只动高度
    assert math.isclose(fw[0], -CFG.lift_clearance, abs_tol=1e-6)
    assert math.isclose(eng.pw["L1"][2], h + 5.0, abs_tol=1e-6)
    # 挪出可落足带被拒、落点不动
    far = eng.nudge_wall_target("L1", dz=400.0)
    assert isinstance(far, str) and math.isclose(eng.pw["L1"][2], h + 5.0, abs_tol=1e-6)
    assert not eng.nudge_pending
    assert eng.land() is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    assert math.isclose(contact_world(eng, "L1")[2], h + 5.0, abs_tol=1e-6)
    assert isinstance(eng.nudge_wall_target("L1", dz=5.0), str)     # 已落地：拒
    assert eng.frozen is None


def test_floor_forward_moves_middle_leg_under_front_hip():
    """09-09 实机：默认站位中足在机身中心正下方、与重心几乎重合，抬第二只前足时
    前半机身成悬臂，前缘沉 26mm 且吸住后不回弹。把中腿先走到前髋底下缓解。"""
    io, ctl, eng, bot = make()
    start(eng, bot)
    hip = eng.hip_world("L2")
    home = eng.floor_home("L2")
    r0 = math.hypot(home[0] - hip[0], home[1] - hip[1])
    tgt = eng.floor_forward("L2", 85.0)
    assert math.isclose(tgt[0] - home[0], 85.0, abs_tol=1e-6)     # 前向分量就是 dist
    assert abs(tgt[2]) < 1e-9
    # 关键：绕髋摆动，髋足距离不变 → 吸盘轴仍⊥地面（平移会拉长半径把倾角带到 10.6°）
    assert math.isclose(math.hypot(tgt[0] - hip[0], tgt[1] - hip[1]), r0, abs_tol=1e-6)
    assert abs(tgt[1] - home[1]) > 10.0                            # 侧向确实收了
    pb0 = w2b((tgt[0], tgt[1], -CFG.leg("L2").press_delta_mm), eng.pose)
    assert eng.geom["L2"].solve(pb0, (0.0, 0.0, -1.0))["tilt"] < 0.5
    assert eng.floor_forward("L2", 500.0) is None                  # 摆不到：超髋足距离
    assert eng.request_move("L2", FLOOR, tgt) is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L2"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L2"] == MountPhase.STANCE
               and ctl.is_attached(idx("L2")))
    # 落到前髋（身体系 x=+83.5）正下方附近
    pb = w2b(eng.pw["L2"], eng.pose)
    assert 80.0 < pb[0] < 90.0
    sol = eng.geom["L2"].solve(tuple(eng.foot["L2"]), (0.0, 0.0, -1.0))
    assert sol["tilt"] < 0.5 and abs(sol["gamma"]) <= COXA_MAX_DEG
    assert eng.frozen is None


def test_floor_moves_lift_higher_than_wall_moves():
    """09-09 实机：L2/R2 绕髋摆动时吸盘蹭到玻璃地板——地面抬离量要盖过抬腿时的
    自重下垂，墙面（法向无重力分量）仍用 lift_clearance。"""
    io, ctl, eng, bot = make(floor_clear_mm=45.0)
    start(eng, bot)
    tgt = eng.floor_forward("L2", 85.0)
    assert eng.request_move("L2", FLOOR, tgt) is None
    lo = 1e9
    for _ in range(int(25 / DT)):
        bot.move_feet(eng.update(DT))
        if eng.phase_of["L2"] in (MountPhase.TRANSFER, MountPhase.HOVER):
            lo = min(lo, contact_world(eng, "L2")[2])      # 平移全程离地最低点
        if eng.phase_of["L2"] == MountPhase.HOVER:
            break
    assert eng.phase_of["L2"] == MountPhase.HOVER
    assert lo > 40.0, f"地面摆动最低离地 {lo:.1f}mm，应 >40"
    assert math.isclose(contact_world(eng, "L2")[2], 45.0, abs_tol=1e-6)   # 悬停高度
    assert eng.land() is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L2"] == MountPhase.STANCE
               and ctl.is_attached(idx("L2")))
    # 墙面不受影响：悬停仍在墙前 lift_clearance
    band = eng.wall_band("L1")
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", sum(band) / 2)) is None
    assert run(eng, bot, 25.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    assert math.isclose(contact_world(eng, "L1")[0], -CFG.lift_clearance, abs_tol=1e-6)
    assert eng.frozen is None


def test_support_only_legs_bear_load_without_attaching():
    """09-09 实机：上墙过程中中腿吸盘吸不住地面，但压着能靠摩擦当支撑。这些腿要
    压到位即回支撑、不抽气，且不参与互锁——否则互锁会拒绝一切动作。抬它们之前
    照样开阀放气（09-13），但不向状态机要放气、不看盘压。"""
    io = MockVacuumIO(6)
    io.sealed[LEG_NAMES.index("L2")] = False          # 中腿吸不住（真机情形）
    io.sealed[LEG_NAMES.index("R2")] = False
    ctl = AdhesionController(io)
    eng = MountEngine(CFG, ctl, support_only=("L2", "R2"))
    bot = Hexapod(MockDriver(), CFG)
    start(eng, bot)
    assert eng.started and eng.frozen is None
    assert ctl.attached_count() == 4                   # 四条吸住，中腿只压着
    for n in ("L2", "R2"):
        assert eng.phase_of[n] == MountPhase.STANCE and eng.surf[n] is FLOOR
        assert not ctl.is_attached(idx(n))
        assert math.isclose(eng.depth[n], CFG.leg(n).press_delta_mm, abs_tol=1e-6)
    # 互锁不因中腿没吸住而拒绝
    band = eng.wall_band("L1")
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", sum(band) / 2)) is None
    assert run(eng, bot, 25.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    # 只承重腿自己被挪走时：不向状态机要放气（它没吸附），但照样先在 VENT 停
    # lift_vent_s 开阀——气路接着时它会被单向阀憋出被动真空（09-13）。不看盘压：
    # 状态机从没采过它的足压，照盘压门槛等就会冻结
    assert eng.request_move("L2", FLOOR, eng.floor_forward("L2", 60.0)) is None
    assert eng.phase_of["L2"] == MountPhase.VENT
    assert ctl.last_kpa[idx("L2")] is None
    run(eng, bot, CFG.lift_vent_s - 0.1)
    assert eng.phase_of["L2"] == MountPhase.VENT
    assert run(eng, bot, 0.2, lambda: eng.phase_of["L2"] == MountPhase.LIFT)
    assert not ctl.is_attached(idx("L2")) and eng.frozen is None
    assert run(eng, bot, 25.0, lambda: eng.phase_of["L2"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L2"] == MountPhase.STANCE)
    assert not ctl.is_attached(idx("L2")) and eng.frozen is None


# ---------------------------------------------------------------- 零力交接
HO_DELTA = 12.0          # 测试用的统一 δ（实机起标表见 HANDOVER-DESIGN §5）


def make_ho(delta=HO_DELTA, cfg=CFG, **kw):
    """开了零力交接的引擎（δ 逐腿统一）。"""
    cfg = replace(cfg, legs=tuple(replace(l, handover_mm=delta) for l in cfg.legs))
    io = MockVacuumIO(6)
    ctl = AdhesionController(io)
    eng = MountEngine(cfg, ctl, **kw)
    bot = Hexapod(MockDriver(), cfg)
    return io, ctl, eng, bot


def to_wall(eng, bot, name, h=None):
    """把 name 送上墙并吸住（可落足带中点）。"""
    band = eng.wall_band(name)
    assert band is not None
    h = sum(band) / 2.0 if h is None else h
    assert eng.request_move(name, eng.wall, eng.wall_target(name, h)) is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of[name] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of[name] == MountPhase.STANCE
               and eng.ctl.is_attached(idx(name)))


def test_handover_unloads_lifted_leg_and_others_take_the_share():
    """08-20 量化：83% 的下滑发生在放气密封破裂一瞬（被抬腿攒的弹性势能一步
    释放）。抬腿前先把这条腿的指令竖直还 δ（力卸到 ~0）、其余接触腿各接 δ/n，
    六腿偏移代数和恒 0 = 身体指令不动；铺完才放气。"""
    io, ctl, eng, bot = make_ho()
    start(eng, bot)
    w0 = {n: contact_world(eng, n) for n in LEG_NAMES}
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", 220.0)) is None
    assert eng.phase_of["L1"] == MountPhase.HANDOVER      # 先交接，不放气
    t0 = eng.t
    worst = 0.0
    while True:
        bot.move_feet(eng.update(DT))
        assert eng.t - t0 < 10.0, "交接没在 10s 内铺完"
        if eng.phase_of["L1"] != MountPhase.HANDOVER:
            break
        # 铺设的每一拍都守恒：Σ偏移=0（各腿按同一进度成比例走）
        worst = max(worst, abs(sum(eng.ho_off.values())))
        assert ctl.state[idx("L1")] == FootState.ATTACHED   # 全程仍吸附密封
    assert worst < 1e-9, f"铺设期 Σ偏移 最大 {worst:.3g}mm，应恒 0"
    assert math.isclose(eng.t - t0, HO_DELTA / CFG.handover_rate_mms, abs_tol=3 * DT)
    assert eng.phase_of["L1"] == MountPhase.VENT          # 铺完才放气
    # 位移账：被抬腿竖直 +δ（卸载），其余五条各 −δ/5（接载）
    assert math.isclose(eng.ho_off["L1"], HO_DELTA, abs_tol=1e-9)
    for n in LEG_NAMES:
        if n == "L1":
            continue
        assert math.isclose(eng.ho_off[n], -HO_DELTA / 5.0, abs_tol=1e-9)
        a, b = w0[n], contact_world(eng, n)
        assert math.isclose(b[0], a[0], abs_tol=1e-9)     # 只沿竖直动
        assert math.isclose(b[1], a[1], abs_tol=1e-9)
        assert math.isclose(b[2] - a[2], -HO_DELTA / 5.0, abs_tol=1e-9)
    assert math.isclose(contact_world(eng, "L1")[2] - w0["L1"][2], HO_DELTA,
                        abs_tol=1e-9)
    assert eng.frozen is None


def test_handover_share_is_press_on_floor_and_shear_on_wall():
    """竖直口径的分面后果（WALL-MOUNT-OPEN §7 原写的"沿法向"对墙面足是错的）：
    地面足的竖直=法向，接载=多压 δ/n；墙面足的竖直=切向，接载=沿墙下滑、
    压深一毫米不变——往墙里压根本接不了体重。"""
    io, ctl, eng, bot = make_ho()
    start(eng, bot)
    to_wall(eng, bot, "L1")                      # L1 先上墙吸住
    pen0 = {n: eng._pen(n) for n in LEG_NAMES}
    wall_w0 = contact_world(eng, "L1")
    assert eng.request_move("R1", eng.wall, eng.wall_target("R1", 220.0)) is None
    assert run(eng, bot, 10.0, lambda: eng.phase_of["R1"] == MountPhase.VENT)
    share = HO_DELTA / 5.0                       # R1 之外还有 5 条接触腿
    for n in ("L2", "L3", "R2", "R3"):           # 地面足：份额全变成压深
        assert math.isclose(eng._pen(n), pen0[n] + share, abs_tol=1e-9)
    assert math.isclose(eng._pen("L1"), pen0["L1"], abs_tol=1e-9)   # 墙面足压深不动
    a, b = wall_w0, contact_world(eng, "L1")
    assert math.isclose(b[0], a[0], abs_tol=1e-9)                   # 离墙距离不动
    assert math.isclose(b[2] - a[2], -share, abs_tol=1e-9)          # 沿墙下滑=吃剪切
    assert eng.frozen is None


def test_takeover_after_landing_returns_the_share_and_zeroes_offsets():
    """落地吸住后自动做反向交接（接管）：把其余腿替它接的载荷还回去——不还的话
    份额会一次次往下累积，34 个单腿动作的序列几次就把压深余量吃穿。还完全机
    偏移归零（除了新腿自己），指令回到纯几何。"""
    io, ctl, eng, bot = make_ho()
    start(eng, bot)
    pen0 = {n: eng._pen(n) for n in LEG_NAMES}
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", 220.0)) is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    assert eng._ho_debt["L1"] == HO_DELTA                  # 欠账 = 其余腿接走的总量
    assert eng.ho_off["L1"] == 0.0                         # 离面即清自己的偏移
    assert eng.land() is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L1"] == MountPhase.TAKEOVER)
    assert ctl.is_attached(idx("L1"))                      # 先吸住，再接管
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE)
    assert "L1" not in eng._ho_debt
    for n in LEG_NAMES:                                    # 五条地面腿全部还清
        if n == "L1":
            continue
        assert abs(eng.ho_off[n]) < 1e-9
        assert math.isclose(eng._pen(n), pen0[n], abs_tol=1e-9)
    # 新腿自己吃下这份载荷：墙面腿的接管是沿墙下滑，压深仍是名义值
    assert math.isclose(eng.ho_off["L1"], -HO_DELTA, abs_tol=1e-9)
    assert math.isclose(eng._pen("L1"), CFG.leg("L1").press_delta_mm, abs_tol=1e-9)
    assert eng.frozen is None


def test_takeover_mm_moves_more_load_onto_the_fresh_wall_leg():
    """用户 09-11：L1 抬上墙后应尽量把势能转移给它。--takeover 让新腿落地后
    接管 max(欠账, 设定值)——其余腿（还压着弹性变形的那几条）被松回去，下一条
    腿再抬时储能就小了。零和仍然成立：身体指令不动。"""
    io, ctl, eng, bot = make_ho(takeover_mm={"L1": 18.0})
    start(eng, bot)
    pen0 = {n: eng._pen(n) for n in LEG_NAMES}
    to_wall(eng, bot, "L1")
    assert math.isclose(eng.ho_off["L1"], -18.0, abs_tol=1e-9)     # 接管 18 > 欠账 12
    # 抬 L1 时五条腿各接了 δ/5=2.4，接管又各还 18/5=3.6 ⇒ 净松 1.2mm（压深跟着松）
    for n in ("L2", "L3", "R1", "R2", "R3"):
        assert math.isclose(eng.ho_off[n], 3.6 - HO_DELTA / 5.0, abs_tol=1e-9)
        assert math.isclose(eng._pen(n), pen0[n] - 1.2, abs_tol=1e-9)
    # 每一笔铺设各自零和；累计和 = −δ，正是 L1 离面时作废掉的那份卸载量
    assert math.isclose(sum(eng.ho_off.values()), -HO_DELTA, abs_tol=1e-9)
    # 手动追加（脚本 z 键）：可连按累加，直到吸盘倾角/压深咬住为止
    assert eng.request_takeover("L1", 3.0) is None
    assert eng.phase_of["L1"] == MountPhase.TAKEOVER
    assert run(eng, bot, 10.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE)
    assert math.isclose(eng.ho_off["L1"], -21.0, abs_tol=1e-9)
    for n in ("L2", "L3", "R1", "R2", "R3"):
        assert math.isclose(eng.ho_off[n], 4.2 - HO_DELTA / 5.0, abs_tol=1e-9)
    # 墙面腿吃的是剪切：接管 21mm 一毫米也没进到压深里
    assert math.isclose(eng._pen("L1"), CFG.leg("L1").press_delta_mm, abs_tol=1e-9)
    # 加到吸盘倾角咬住为止（墙面腿的真正上界），拒绝时原地不动
    off0 = dict(eng.ho_off)
    for _ in range(20):
        deny = eng.request_takeover("L1", 3.0)
        if deny:
            break
        assert run(eng, bot, 10.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE)
        off0 = dict(eng.ho_off)
    assert isinstance(deny, str) and "倾角" in deny
    assert eng.ho_off == off0 and eng.frozen is None


def test_takeover_refused_when_it_would_stall_or_unload_support_legs():
    """接管的两条硬界：接载的腿压入 ≤ PRESS_DEPTH_MAX（再深=顶着刚性玻璃堵转）、
    还载的腿指令不许抬到面以上（吸附腿会变成往外拔，只承重腿直接失去摩擦支撑）。
    不过则拒绝、原地不动。"""
    io, ctl, eng, bot = make_ho(delta=0.0)
    start(eng, bot)
    off0, feet0 = dict(eng.ho_off), {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    room = PRESS_DEPTH_MAX - CFG.leg("L3").press_delta_mm          # 只剩 10mm
    deny = eng.request_takeover("L3", room + 4.0)                  # 自己压穿
    assert isinstance(deny, str) and "压入" in deny and "上限" in deny
    assert eng.phase_of["L3"] == MountPhase.STANCE
    assert eng.ho_off == off0
    assert all(tuple(eng.foot[n]) == feet0[n] for n in LEG_NAMES)
    # 反过来：份额把其余腿抬到地面以上（press_delta=18，每条最多松 18）
    deny = eng.request_takeover("L1", 5.0 * CFG.leg("L2").press_delta_mm + 10.0)
    assert isinstance(deny, str) and ("以上" in deny or "越界" in deny)
    assert eng.ho_off == off0
    assert eng.request_takeover("L3", 0.0) is not None              # 量非法
    assert eng.frozen is None


def test_handover_refused_whole_command_when_share_does_not_fit():
    """交接不可行就**拒绝整条挪腿命令**（半截交接比不交接更糟：载荷挪了一半就
    放气）。δ=45 分给 5 条腿还塞得下，收起一条中腿后只剩 4 条分母就塞不下了。"""
    io, ctl, eng, bot = make_ho(delta=45.0)
    start(eng, bot)
    assert eng.request_tuck("L2") is None                  # 交接后收起中腿
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L2"] == MountPhase.AIR)
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L2"] == MountPhase.AIR
               and eng.swing_leg is None)
    off0, feet0 = dict(eng.ho_off), {n: tuple(eng.foot[n]) for n in LEG_NAMES}
    deny = eng.request_move("L1", eng.wall, eng.wall_target("L1", 220.0))
    assert isinstance(deny, str) and "零力交接不可行" in deny and "--handover" in deny
    assert eng.phase_of["L1"] == MountPhase.STANCE         # 原地不动
    assert eng.ho_off == off0
    assert all(tuple(eng.foot[n]) == feet0[n] for n in LEG_NAMES)
    assert eng.frozen is None


def test_handover_pauses_on_leak_and_resumes_without_venting():
    """漏气挽救期暂停（漏着的盘摩擦余量低，不该被推）：铺设量不丢、不放气；
    挽救成功后续铺完成。交接腿自己漏气同样算（climb 审核 §10.1 同口径）。"""
    io, ctl, eng, bot = make_ho(delta=20.0)
    start(eng, bot)
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", 220.0)) is None
    run(eng, bot, 0.4)                                     # 先铺一小段
    ri = idx("R3")
    io.sealed[ri] = False
    assert run(eng, bot, 3.0, lambda: ctl.is_leaking(ri))
    off_paused = dict(eng.ho_off)
    assert 0.0 < off_paused["L1"] < 20.0
    run(eng, bot, CFG.leak_rescue_s * 0.5)                 # 挽救窗内
    assert eng.frozen is None
    assert eng.ho_off == off_paused                        # 冻住不推进
    assert eng.phase_of["L1"] == MountPhase.HANDOVER
    assert ctl.state[idx("L1")] == FootState.ATTACHED      # 没放气
    io.sealed[ri] = True
    assert run(eng, bot, 3.0, lambda: not ctl.is_leaking(ri))
    assert run(eng, bot, 10.0, lambda: eng.phase_of["L1"] == MountPhase.VENT)
    assert math.isclose(eng.ho_off["L1"], 20.0, abs_tol=1e-9)
    assert eng.frozen is None


def test_handover_rechecks_lift_gate_before_venting():
    """放气前复检门槛（climb 审核 §10.3 同款）：窗头那次判定距此已过 δ/速率 秒，
    期间支撑盘可能漏到"深于漏气绊线、浅于抬腿门槛"的监护盲区——带着软肩膀放气
    正是 08-19 事故类。复检不过就保持密封等泵，超时冻结点名。"""
    cfg = replace(CFG, lift_gate_timeout_s=1.0)
    io, ctl, eng, bot = make_ho(cfg=cfg)
    start(eng, bot)
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", 220.0)) is None
    real, ri = io.read_foot_kpa, idx("R3")
    io.read_foot_kpa = lambda i: -35.0 if i == ri else real(i)   # 吸着但浅于门槛
    assert run(eng, bot, 6.0, lambda: eng.frozen is not None)    # 铺 1.2s + 等 1s
    assert not ctl.is_leaking(ri)                          # 不是漏气，是盘压浅
    assert eng.phase_of["L1"] == MountPhase.HANDOVER       # 卡在交接尾，没放气
    assert ctl.state[idx("L1")] == FootState.ATTACHED
    assert "门槛" in eng.frozen and "R3" in eng.frozen
    io.read_foot_kpa = real                                # 泵把它拽深了
    eng.clear_freeze()                                     # 解冻不取消在途交接
    assert run(eng, bot, 10.0, lambda: eng.phase_of["L1"] == MountPhase.VENT)
    assert math.isclose(eng.ho_off["L1"], HO_DELTA, abs_tol=1e-9)


def test_support_only_leg_hands_over_then_vents_by_time_and_lifts():
    """只承重腿（中腿靠摩擦支撑）也承载，抬它之前一样要交接；铺完照样先开阀放气
    lift_vent_s（气路接着时会被单向阀憋出被动真空，09-13）再抬，但不看盘压——状态机
    不采它的足压。它也照样接别人的份额（多压一点=多一点正压力）。"""
    io = MockVacuumIO(6)
    for n in ("L2", "R2"):
        io.sealed[idx(n)] = False
    ctl = AdhesionController(io)
    cfg = replace(CFG, legs=tuple(replace(l, handover_mm=HO_DELTA) for l in CFG.legs))
    eng = MountEngine(cfg, ctl, support_only=("L2", "R2"))
    bot = Hexapod(MockDriver(), cfg)
    start(eng, bot)
    pen0 = eng._pen("R2")
    assert eng.request_move("L2", FLOOR, eng.floor_forward("L2", 60.0)) is None
    assert eng.phase_of["L2"] == MountPhase.HANDOVER
    assert run(eng, bot, 10.0, lambda: eng.phase_of["L2"] != MountPhase.HANDOVER)
    assert eng.phase_of["L2"] == MountPhase.VENT           # 铺完先开阀放气，不直接抬
    assert run(eng, bot, CFG.lift_vent_s + 0.1, lambda: eng.phase_of["L2"] == MountPhase.LIFT)
    assert math.isclose(eng.ho_off["L2"], HO_DELTA, abs_tol=1e-9)
    assert math.isclose(eng._pen("R2"), pen0 + HO_DELTA / 5.0, abs_tol=1e-9)
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L2"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L2"] == MountPhase.STANCE)
    # 压到位即接管还账。但地面腿的接管量直接变成自己的压深，余量只有
    # PRESS_DEPTH_MAX−press_delta=10mm < 欠账 12 ⇒ 截断，其余腿只松回大半
    room = PRESS_DEPTH_MAX - CFG.leg("L2").press_delta_mm
    assert -HO_DELTA / 5.0 < eng.ho_off["R2"] < -(HO_DELTA - room) / 5.0 + 0.1
    assert eng._pen("L2") <= PRESS_DEPTH_MAX + 1e-9
    assert "截到" in (eng.handover_note or "")
    assert not ctl.is_attached(idx("L2")) and eng.frozen is None


def test_tuck_keeps_the_debt_until_the_leg_comes_back_down():
    """收到空中的腿不还账：载荷确实还在其余腿身上（它们真压着变形）。等它落回
    面上吸住，才把这笔还回去——偏移随之归零。"""
    io, ctl, eng, bot = make_ho()
    start(eng, bot)
    pen0 = {n: eng._pen(n) for n in LEG_NAMES}
    assert eng.request_tuck("L2") is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L2"] == MountPhase.AIR)
    assert eng._ho_debt["L2"] == HO_DELTA                  # 欠账挂着
    for n in ("L1", "L3", "R1", "R2", "R3"):
        assert math.isclose(eng._pen(n), pen0[n] + HO_DELTA / 5.0, abs_tol=1e-9)
    assert eng.request_move("L2", FLOOR, eng.floor_home("L2")) is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L2"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of["L2"] == MountPhase.STANCE)
    assert "L2" not in eng._ho_debt
    room = PRESS_DEPTH_MAX - CFG.leg("L2").press_delta_mm       # 地面腿只有 10mm 余量
    assert math.isclose(eng._pen("L2"), pen0["L2"] + min(HO_DELTA, room),
                        abs_tol=0.1)
    for n in ("L1", "L3", "R1", "R2", "R3"):                    # 其余腿松回大半
        assert math.isclose(eng._pen(n), pen0[n] + max(0.0, HO_DELTA - room) / 5.0,
                            abs_tol=0.05)
    assert eng.frozen is None


def test_handover_phases_never_open_the_valve_and_block_other_commands():
    """两条硬约束：①干跑真阀按 SWING_PHASES 通电排气，交接/接管两段必须在它之外
    ——否则交接期间就把盘放了，整件事的前提没了；②交接在途时全机不受理别的命令
    （位姿铺设会横拖正在加减载的接触足）。"""
    from hexapod.mount import SWING_PHASES, HO_PHASES, BUSY_PHASES
    assert MountPhase.HANDOVER not in SWING_PHASES
    assert MountPhase.TAKEOVER not in SWING_PHASES
    assert set(BUSY_PHASES) == set(SWING_PHASES) | set(HO_PHASES)
    io, ctl, eng, bot = make_ho(delta=20.0)
    start(eng, bot)
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", 220.0)) is None
    run(eng, bot, 0.4)
    assert eng.phase_of["L1"] == MountPhase.HANDOVER
    assert eng.swing_leg == "L1" and eng.ho_leg == "L1"
    # 阀还在通罐位（valve=True=接通真空）、盘压还在深处：一点没放气
    assert io.valve[idx("L1")] is True
    assert ctl.state[idx("L1")] == FootState.ATTACHED
    assert ctl.last_kpa[idx("L1")] <= CFG.lift_gate_kpa
    pose0 = eng.pose
    assert isinstance(eng.request_pose(dpitch_deg=3.0), str)
    assert isinstance(eng.request_move("R1", FLOOR, eng.floor_home("R1")), str)
    assert isinstance(eng.request_takeover("L3", 3.0), str)
    assert eng.land() is not None                          # 没有悬停腿
    assert eng.pose == pose0
    assert run(eng, bot, 10.0, lambda: eng.phase_of["L1"] == MountPhase.VENT)
    assert eng.frozen is None


def test_over_cap_leg_can_still_share_load_but_never_goes_deeper():
    """压深上限要按**相对**判：只拦"把已经越界的量推得更糟"的动作。
    09-11 仿真复现的真 bug——L1 带 --wall-trim 16 上墙后命令压深 18+16=34 本就超
    28，而墙面腿的交接是沿墙切向（n_z=0）、压深一毫米都不动，绝对值判会拒掉一个
    根本不改这个量的动作：L1 一上墙，抬 R1 的交接就被"L1 压入 34 超 28"整条拒绝，
    B 组恰好死在要测的那一步。"""
    io, ctl, eng, bot = make_ho()
    assert eng.set_wall_trim(16.0, ["L1"]) is None
    start(eng, bot)
    to_wall(eng, bot, "L1", 208.0)
    pen_wall = eng._pen("L1")
    assert pen_wall > PRESS_DEPTH_MAX                      # 本就超上限（trim 叠进去）
    # 抬 R1：L1 作为接载腿必须能参与分摊（它的压深根本不会变）
    assert eng.request_move("R1", eng.wall, eng.wall_target("R1", 215.0)) is None
    assert run(eng, bot, 10.0, lambda: eng.phase_of["R1"] == MountPhase.VENT)
    assert math.isclose(eng._pen("L1"), pen_wall, abs_tol=1e-9)   # 切向，压深不动
    assert eng.ho_off["L1"] < 0.0                                 # 确实接了份额
    assert run(eng, bot, 40.0, lambda: eng.phase_of["R1"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 40.0, lambda: eng.phase_of["R1"] == MountPhase.STANCE
               and ctl.is_attached(idx("R1")))
    # 但"往更深里推"照样拦：地面腿从 18 推过 28 仍然拒
    deny = eng.request_takeover("L3", PRESS_DEPTH_MAX - CFG.leg("L3").press_delta_mm + 5.0)
    assert isinstance(deny, str) and "压入" in deny
    assert eng.frozen is None


def test_support_only_legs_use_a_looser_tilt_bound_than_sealing_legs():
    """只承重腿（support_only）只压不吸，盘面对不对正不影响它传正压力——`HOLD_TILT_DEG`
    是吸盘**密封**的容差，套在它头上是错的：按 15° 卡，中腿在俯仰 >30° 就被拒，而几何上
    它能撑到 90°（OPEN §1.1）。09-12 台架实测（LAB E4a）盘面斜 35° 时接触仍在盘面、
    仍压得住，所以默认放宽到 35°。默认站位下中腿 β=90° ⇒ 倾角恰好 = 俯仰角，
    抬到 16° 就能把两套口径分开。"""
    from hexapod.mount import SUPPORT_TILT_DEG
    io, ctl, eng, bot = make()                       # 没有 support_only：全按 15° 卡
    start(eng, bot)
    assert eng._tilt_lim("L2") == HOLD_TILT_DEG
    deny = eng.request_pose(dpitch_deg=16.0)
    assert isinstance(deny, str) and "L2" in deny and "倾角" in deny
    # 同样的位姿，中腿改成只承重腿就该放行
    io2 = MockVacuumIO(6)
    for n in ("L2", "R2"):
        io2.sealed[LEG_NAMES.index(n)] = False       # 中腿吸不住（真机情形）
    ctl2 = AdhesionController(io2)
    eng2 = MountEngine(CFG, ctl2, support_only=("L2", "R2"))
    bot2 = Hexapod(MockDriver(), CFG)
    start(eng2, bot2)
    assert eng2._tilt_lim("L2") == SUPPORT_TILT_DEG and eng2._tilt_lim("L1") == HOLD_TILT_DEG
    assert eng2.request_pose(dpitch_deg=16.0) is None
    assert run(eng2, bot2, 30.0, lambda: not eng2.pose_pending)
    assert math.isclose(eng2.pitch_deg, 16.0, abs_tol=1e-6)
    # 但放宽是有限的：把容差调回 15° 就又该拒
    eng3 = MountEngine(CFG, AdhesionController(MockVacuumIO(6)),
                       support_only=("L2", "R2"), support_tilt_deg=15.0)
    bot3 = Hexapod(MockDriver(), CFG)
    start(eng3, bot3)
    assert isinstance(eng3.request_pose(dpitch_deg=16.0), str)
    # 吸附腿不受影响：落点带仍按 12° 密封口径
    assert eng2._tilt_lim("L1", band=True) == TILT_BAND_DEG
    assert eng2.frozen is None and eng.frozen is None


def test_wall_landing_height_is_the_pitch_budget():
    """落点高度 = 抬头余量（09-12 LAB E5 实机复算）：⊥ 点、带上沿、接管量三者
    花的是吸盘倾角这同一笔 15° 预算；接管 1mm 等价于落点低 1mm。"""
    io, ctl, eng, bot = make(support_only=("L2", "R2"),
                             takeover_mm={"L1": 10.0, "R1": 10.0})
    start(eng, bot)
    band = eng.wall_band("L1")
    zp = eng.wall_perp_height("L1")
    assert band[0] < zp < band[1]                       # ⊥ 点落在带内
    mid, top = (band[0] + band[1]) / 2.0, band[1]
    r_mid, r_top = eng.wall_pitch_room("L1", mid), eng.wall_pitch_room("L1", top)
    assert r_top > r_mid + 5.0                          # 带上沿比带中点多一大截抬头
    # 接管吃掉的量 = 同样毫米数的落点高度（09-12：0.45°/mm 量级）
    r_mid_t0 = eng.wall_pitch_room("L1", mid, takeover_mm=0.0)
    r_low = eng.wall_pitch_room("L1", mid - 10.0, takeover_mm=0.0)
    assert r_mid_t0 > r_mid
    assert abs(r_low - r_mid) <= 1.0
    # 这条腿给出的上限与整套位姿预检一致（E5 实机：卡的一直是 L1 倾角）
    assert eng.request_move("L1", eng.wall, eng.wall_target("L1", mid)) is None
    assert run(eng, bot, 15.0, lambda: eng.phase_of["L1"] == MountPhase.HOVER)
    assert eng.land() is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    assert run(eng, bot, 10.0, lambda: eng.ho_leg is None)
    assert math.isclose(eng.ho_off["L1"], -10.0, abs_tol=0.2)   # 接管挂在墙面腿上
    ok = None
    for p in range(0, 40):
        if eng._check_pose((eng.pose[0], eng.pose[1], math.radians(p))) is not None:
            break
        ok = p
    assert abs(ok - r_mid) <= 1.0
