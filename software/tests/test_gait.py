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


def test_march_groups_come_from_gait_offsets():
    """踏步分组=步态同偏移的腿，组序按偏移升序（= 步行抬腿先后），六腿不重不漏。"""
    tri = MarchEngine(CFG, TRIPOD)
    assert tri.groups == (("L1", "L3", "R2"), ("L2", "R1", "R3"))
    wave = MarchEngine(CFG, WAVE)
    assert wave.groups == (("R3",), ("R2",), ("R1",), ("L3",), ("L2",), ("L1",))
    for eng in (tri, wave):
        flat = [n for g in eng.groups for n in g]
        assert sorted(flat) == sorted(LEG_NAMES)


def test_march_holds_five_seconds_up_and_down():
    """抬到顶悬停 5s、落地站定 5s，两头都停满才走下一段。"""
    eng = MarchEngine(CFG, TRIPOD, hold_s=5.0)
    assert eng.period == 2 * eng.lift_s + 10.0
    eps = 1e-6                                        # 段端点归下一段，取严格段内
    # 悬空段：整整 5s 停在最高点不动，只有本组三只腿抬着
    for i in range(51):
        t = eng.lift_s + eps + i * (5.0 - 2 * eps) / 50
        assert eng.phase_at(t)[0] == "top"
        assert abs(eng.height_at(t) - CFG.step_height) < 1e-9
        up = [n for n, p in eng.foot_targets(t).items()
              if p[2] > eng.default_feet[n][2] + 1e-9]
        assert sorted(up) == sorted(eng.groups[0])
    # 落地段：整整 5s 六脚都在默认站位
    for i in range(51):
        t = 2 * eng.lift_s + 5.0 + eps + i * (5.0 - 2 * eps) / 50
        assert eng.phase_at(t)[0] == "ground" and not eng.airborne(t)
        assert eng.foot_targets(t) == eng.default_feet
    # 站定停满才轮到下一组抬起
    t = eng.period + eng.lift_s / 2
    assert eng.phase_at(t)[0] == "rise" and eng.group_at(t) == 1
    assert eng.group_at(len(eng.groups) * eng.period + 0.1) == 0


def test_march_moves_only_z_and_peaks_at_step_height():
    """原地踏步：x/y 恒在默认站位（身体不前进），z 峰高 = step_height，抬落单调。"""
    eng = MarchEngine(CFG, TRIPOD, hold_s=5.0)
    peak = 0.0
    for i in range(400):
        t = i * len(eng.groups) * eng.period / 400
        for n, (x, y, z) in eng.foot_targets(t).items():
            x0, y0, z0 = eng.default_feet[n]
            assert (x, y) == (x0, y0)
            assert z >= z0 - 1e-9
            peak = max(peak, z - z0)
    assert abs(peak - CFG.step_height) < 1e-9
    rise = [eng.height_at(i * eng.lift_s / 20) for i in range(21)]
    assert rise == sorted(rise)                       # 抬起单调升
    fall = [eng.height_at(eng.lift_s + 5.0 + i * eng.lift_s / 20) for i in range(21)]
    assert fall == sorted(fall, reverse=True)         # 落下单调降


def test_march_hold_zero_is_continuous_and_targets_reachable():
    """hold=0 退化成不停歇的连续踏步；两种步态整轮 IK 都可解。"""
    bot = Hexapod(MockDriver())
    for gait in (TRIPOD, WAVE):
        for hold in (0.0, 5.0):
            eng = MarchEngine(CFG, gait, hold_s=hold)
            for i in range(120):
                t = i * len(eng.groups) * eng.period / 120
                bot.pulses(eng.foot_targets(t))     # 内部做 IK + 脉宽映射
            if hold == 0.0:
                assert all(eng.airborne(i * eng.period / 9) for i in range(1, 9))
