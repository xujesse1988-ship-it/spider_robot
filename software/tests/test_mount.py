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
