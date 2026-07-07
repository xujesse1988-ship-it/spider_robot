import math

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


def test_stride_capped():
    eng = GaitEngine(CFG, TRIPOD)
    ux, uy = eng._stride("L1", 1000.0, 0, 0)  # 荒谬大的速度
    assert math.hypot(ux, uy) <= CFG.max_step + 1e-9


def test_static_command_returns_default_feet():
    eng = GaitEngine(CFG, TRIPOD)
    assert eng.foot_targets(1.23) == eng.default_feet
