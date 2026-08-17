import math
from dataclasses import replace

from hexapod.config import DEFAULT_CONFIG as CFG, LEG_NAMES
from hexapod.gait import GaitEngine, TRIPOD, WAVE, CLIMB
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
