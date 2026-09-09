"""MountEngine（地-墙过渡引擎）测试：启动六足吸附 / 前腿上墙悬停-落下 /
俯仰铺设接触足钉死 / 不可行请求拒绝 / 收腿豁免互锁 / 吸附失败加深重试。
全部跑在 MockVacuumIO + MockDriver 上；每拍都过 Hexapod.move_feet 的真 IK，
足端目标出工作空间会当场抛 WorkspaceError。"""
import math

from hexapod.adhesion import AdhesionController, MockVacuumIO, FootState
from hexapod.mount import (MountEngine, MountPhase, FLOOR, b2w, w2b, _add,
                           TILT_BAND_DEG, HOLD_TILT_DEG, COXA_MAX_DEG)
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
    run(eng, bot, 0.1)
    fw = contact_world(eng, "L1")
    assert math.isclose(fw[2], z0 + 5.0, abs_tol=1e-6)             # 只动高度
    assert math.isclose(fw[0], -CFG.lift_clearance, abs_tol=1e-6)
    assert math.isclose(eng.pw["L1"][2], h + 5.0, abs_tol=1e-6)
    # 挪出可落足带被拒、落点不动
    far = eng.nudge_wall_target("L1", dz=400.0)
    assert isinstance(far, str) and math.isclose(eng.pw["L1"][2], h + 5.0, abs_tol=1e-6)
    assert eng.land() is None
    assert run(eng, bot, 20.0, lambda: eng.phase_of["L1"] == MountPhase.STANCE
               and ctl.is_attached(idx("L1")))
    assert math.isclose(contact_world(eng, "L1")[2], h + 5.0, abs_tol=1e-6)
    assert isinstance(eng.nudge_wall_target("L1", dz=5.0), str)     # 已落地：拒
    assert eng.frozen is None
