"""单腿 3 自由度运动学。

腿坐标系：原点 = coxa 转轴，+X = 腿中性朝向（水平向外），+Z = 上，Y 右手系。
关节约定：
  gamma  coxa 水平摆角，0 = 沿 +X，逆时针为正（rad）
  alpha  femur 相对水平面仰角，抬起为正（rad）
  theta  膝关节内角（femur 与 tibia 夹角），伸直 = pi（rad）
"""
import math

from .config import RobotConfig


class WorkspaceError(ValueError):
    """目标点不在腿的工作空间内。"""


def leg_ik(cfg: RobotConfig, x: float, y: float, z: float):
    """足端位置 (x,y,z) mm -> (gamma, alpha, theta) rad。"""
    l1, l2, l3 = cfg.coxa_len, cfg.femur_len, cfg.tibia_len
    gamma = math.atan2(y, x)
    r = math.hypot(x, y) - l1
    d = math.hypot(r, z)
    if d > l2 + l3 - 1e-9 or d < abs(l2 - l3) + 1e-9:
        raise WorkspaceError(
            f"目标 ({x:.1f},{y:.1f},{z:.1f}) 距 femur 轴 {d:.1f}mm，"
            f"可达范围 ({abs(l2-l3):.1f}, {l2+l3:.1f})")
    theta = math.acos((l2 * l2 + l3 * l3 - d * d) / (2 * l2 * l3))
    alpha = math.atan2(z, r) + math.acos((l2 * l2 + d * d - l3 * l3) / (2 * l2 * d))
    return gamma, alpha, theta


def leg_fk(cfg: RobotConfig, gamma: float, alpha: float, theta: float):
    """关节角 -> 足端位置 (x,y,z) mm。测试与仿真用。"""
    l1, l2, l3 = cfg.coxa_len, cfg.femur_len, cfg.tibia_len
    a_t = alpha + theta - math.pi          # tibia 相对水平面的绝对角
    r = l1 + l2 * math.cos(alpha) + l3 * math.cos(a_t)
    z = l2 * math.sin(alpha) + l3 * math.sin(a_t)
    return r * math.cos(gamma), r * math.sin(gamma), z


def leg_joint_points(cfg: RobotConfig, gamma: float, alpha: float, theta: float):
    """返回 [髋, 肘(coxa末端), 膝, 足] 四个点（腿坐标系），画图用。"""
    l1, l2, l3 = cfg.coxa_len, cfg.femur_len, cfg.tibia_len
    cg, sg = math.cos(gamma), math.sin(gamma)
    a_t = alpha + theta - math.pi
    elbow = (l1 * cg, l1 * sg, 0.0)
    knee_r = l1 + l2 * math.cos(alpha)
    knee = (knee_r * cg, knee_r * sg, l2 * math.sin(alpha))
    foot_r = knee_r + l3 * math.cos(a_t)
    foot = (foot_r * cg, foot_r * sg, knee[2] + l3 * math.sin(a_t))
    return [(0.0, 0.0, 0.0), elbow, knee, foot]
