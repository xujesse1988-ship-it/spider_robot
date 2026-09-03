import math
from dataclasses import replace

from hexapod.config import DEFAULT_CONFIG as CFG, LEG_NAMES
from hexapod.gait import GaitEngine, MarchEngine, TRIPOD, WAVE, CLIMB
from hexapod.kinematics import leg_ik
from hexapod.robot import Hexapod
from hexapod.driver import MockDriver


def test_tripod_always_three_stance():
    eng = GaitEngine(CFG, TRIPOD)
    for i in range(200):
        t = i * CFG.cycle_time / 200
        assert len(eng.stance_legs(t)) == 3


def test_wave_and_climb_at_least_five_stance():
    for gait in (WAVE, CLIMB):
        eng = GaitEngine(CFG, gait)
        for i in range(300):
            t = i * CFG.cycle_time / 300
            assert len(eng.stance_legs(t)) >= 5


def test_all_targets_reachable_during_walk():
    """整个步态周期内所有足端目标都在工作空间内（不抛 WorkspaceError）。"""
    bot = Hexapod(MockDriver())
    for gait in (TRIPOD, WAVE):
        bot.engine = GaitEngine(CFG, gait)
        for i in range(120):
            t = i * CFG.cycle_time / 120
            targets = bot.engine.foot_targets(t, 60.0, 20.0, 0.3)
            bot.pulses(targets)  # 内部做 IK + 脉宽映射


def test_crouch_pose_reachable():
    """缓慢站起用的蹲姿（身体离地 20mm）全腿 IK 可解、蹲->站全程可达。"""
    bot = Hexapod(MockDriver())
    crouch = bot.crouch_feet()
    bot.pulses(crouch)
    for s in range(1, 11):                 # 蹲姿到站姿的插值路径逐点检查
        mix = {n: tuple(a + (b - a) * s / 10 for a, b in
                        zip(crouch[n], bot.engine.default_feet[n]))
               for n in crouch}
        bot.pulses(mix)


def test_stride_capped():
    eng = GaitEngine(CFG, TRIPOD)
    ux, uy = eng._stride("L1", 1000.0, 0, 0)  # 荒谬大的速度
    assert math.hypot(ux, uy) <= CFG.max_step + 1e-9


def test_static_command_returns_default_feet():
    eng = GaitEngine(CFG, TRIPOD)
    assert eng.foot_targets(1.23) == eng.default_feet


def test_trim_equals_explicit_speed_offset():
    """跑偏 trim = 按 vx 比例叠加的 wz/vy 修正，且符号与实测漂移相反。"""
    vx = 40.0
    trimmed = GaitEngine(replace(CFG, yaw_trim_deg_per_m=2.0,
                                 side_trim_mm_per_m=15.0), TRIPOD)
    plain = GaitEngine(CFG, TRIPOD)
    expect = plain.foot_targets(0.4, vx, -15.0 * vx / 1000.0,
                                -math.radians(2.0) * vx / 1000.0)
    assert trimmed.foot_targets(0.4, vx) == expect


def test_trim_inactive_without_forward_speed():
    """静止和原地转向不受 trim 影响。"""
    trimmed = GaitEngine(replace(CFG, yaw_trim_deg_per_m=2.0,
                                 side_trim_mm_per_m=15.0), TRIPOD)
    plain = GaitEngine(CFG, TRIPOD)
    assert trimmed.foot_targets(0.4) == plain.default_feet
    assert trimmed.foot_targets(0.4, 0.0, 0.0, 0.3) == \
        plain.foot_targets(0.4, 0.0, 0.0, 0.3)


def test_march_keymap_covers_six_legs_in_view_layout():
    """六个踏步键一一对应六条腿，键盘 2×3 块 = 俯视腿位（左列 t/g/b、右列 y/h/n）。"""
    eng = MarchEngine(CFG)
    assert eng.keymap == {"t": "L1", "g": "L2", "b": "L3",
                          "y": "R1", "h": "R2", "n": "R3"}
    assert sorted(eng.keymap.values()) == sorted(LEG_NAMES)
    assert eng.leg_of_key("T") == "L1"          # 大小写都认
    for k in ("w", "s", " ", "1", "v", "m", ""):
        assert eng.leg_of_key(k) is None        # 非踏步键一律不认


def test_march_toggle_lifts_only_that_leg_and_toggles_back():
    """按一下抬起、再按踩下；只有被点的那只脚动 z，其余六脚原样站着。"""
    eng = MarchEngine(CFG, lift_mm=70.0, move_s=0.5)
    assert eng.foot_targets() == eng.default_feet and eng.settled
    assert eng.toggle("L1") is True and eng.up_legs() == ("L1",)
    for _ in range(25):                          # 0.5s 抬到位
        eng.update(0.02)
    assert eng.settled and abs(eng.height_of("L1") - 70.0) < 1e-9
    for n, (x, y, z) in eng.foot_targets().items():
        x0, y0, z0 = eng.default_feet[n]
        assert (x, y) == (x0, y0)                # 身体不动，只动 z
        assert z == z0 + (70.0 if n == "L1" else 0.0)
    assert eng.toggle("L1") is False             # 再按踩下
    for _ in range(25):
        eng.update(0.02)
    assert eng.settled and eng.foot_targets() == eng.default_feet


def test_march_lift_is_smooth_and_reverses_midway():
    """抬落两端速度为零、单调；抬到一半再按平滑折返，不跳变。"""
    eng = MarchEngine(CFG, lift_mm=70.0, move_s=0.5)
    eng.toggle("R2")
    hs = []
    for _ in range(25):
        eng.update(0.02)
        hs.append(eng.height_of("R2"))
    assert hs == sorted(hs) and abs(hs[-1] - 70.0) < 1e-9
    assert hs[0] < 0.02 * 70.0 and (hs[-1] - hs[-2]) < (hs[13] - hs[12])  # 两端慢中间快
    eng2 = MarchEngine(CFG, lift_mm=70.0, move_s=0.5)
    eng2.toggle("R2")
    for _ in range(12):
        eng2.update(0.02)
    mid = eng2.height_of("R2")
    assert 0 < mid < 70.0 and not eng2.settled
    eng2.toggle("R2")                            # 半空中反悔
    assert eng2.height_of("R2") == mid           # 折返不跳变
    for _ in range(12):
        eng2.update(0.02)
    assert eng2.height_of("R2") == 0.0 and eng2.settled


def test_march_targets_reachable_for_every_leg():
    """每条腿单独抬到默认高度、以及六脚全抬，IK 都可解。"""
    bot = Hexapod(MockDriver())
    eng = MarchEngine(CFG)
    assert eng.lift_mm > CFG.step_height          # "抬高一些"：比步行抬脚高
    for name in LEG_NAMES:
        eng.toggle(name)
        for _ in range(30):
            eng.update(0.02)
            bot.pulses(eng.foot_targets())        # 内部做 IK + 脉宽映射
        assert eng.up_legs() == (name,)
        eng.toggle(name)
        for _ in range(30):
            eng.update(0.02)
            bot.pulses(eng.foot_targets())
    for name in LEG_NAMES:                        # 六脚同时抬（IK 上限校验）
        eng.toggle(name)
    for _ in range(30):
        eng.update(0.02)
        bot.pulses(eng.foot_targets())
    assert eng.up_legs() == LEG_NAMES
