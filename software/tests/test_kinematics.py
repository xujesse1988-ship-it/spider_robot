import math

import pytest

from hexapod.config import DEFAULT_CONFIG as CFG
from hexapod.kinematics import leg_ik, leg_fk, leg_joint_points, WorkspaceError


def test_ik_fk_roundtrip():
    """工作空间内网格点：IK 后 FK 应回到原点位。"""
    for x in range(60, 200, 20):
        for y in range(-80, 81, 40):
            for z in range(-130, 10, 20):
                d = math.hypot(math.hypot(x, y) - CFG.coxa_len, z)
                if not (abs(CFG.femur_len - CFG.tibia_len) + 5 < d
                        < CFG.femur_len + CFG.tibia_len - 5):
                    continue
                g, a, th = leg_ik(CFG, x, y, z)
                fx, fy, fz = leg_fk(CFG, g, a, th)
                assert math.hypot(fx - x, fy - y) + abs(fz - z) < 1e-6


def test_default_stance_reachable():
    """默认站姿必须可达且膝角合理。"""
    g, a, th = leg_ik(CFG, CFG.foot_reach, 0, -CFG.stand_height)
    assert abs(math.degrees(g)) < 1e-9
    assert 30 < math.degrees(th) < 175


def test_unreachable_raises():
    with pytest.raises(WorkspaceError):
        leg_ik(CFG, 400, 0, 0)      # 太远
    with pytest.raises(WorkspaceError):
        leg_ik(CFG, CFG.coxa_len, 0, -20)  # 太近（贴着髋轴）


def test_joint_points_consistent_with_fk():
    g, a, th = leg_ik(CFG, 130, 20, -90)
    pts = leg_joint_points(CFG, g, a, th)
    assert pts[0] == (0.0, 0.0, 0.0)
    fx, fy, fz = leg_fk(CFG, g, a, th)
    assert math.hypot(pts[3][0] - fx, pts[3][1] - fy) + abs(pts[3][2] - fz) < 1e-9
    # 各段长度正确
    def dist(p, q):
        return math.dist(p, q)
    assert abs(dist(pts[0], pts[1]) - CFG.coxa_len) < 1e-6
    assert abs(dist(pts[1], pts[2]) - CFG.femur_len) < 1e-6
    assert abs(dist(pts[2], pts[3]) - CFG.tibia_len) < 1e-6
