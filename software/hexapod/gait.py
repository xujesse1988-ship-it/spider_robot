"""步态引擎：相位式步态生成。

每条腿一个相位偏移 offset，全局相位 p = (t / cycle_time + offset) % 1：
  p < duty      支撑相（stance）：足端贴地，相对身体匀速后移
  p >= duty     摆动相（swing）：足端抬起，前移到下一落点

输出为各腿足端在**身体坐标系**中的目标点（默认站位 + 步幅偏移）。
"""
import math
from dataclasses import dataclass

from .config import RobotConfig, LEG_NAMES


@dataclass(frozen=True)
class Gait:
    name: str
    duty: float                 # 支撑相占比
    offsets: dict               # 腿名 -> 相位偏移 [0,1)


# 三角步态：L1/R2/L3 与 R1/L2/R3 交替，地面行走用
TRIPOD = Gait("tripod", 0.5, {"L1": 0.0, "R2": 0.0, "L3": 0.0,
                              "R1": 0.5, "L2": 0.5, "R3": 0.5})
# 波浪步态：一次一腿（后->前），慢但稳
WAVE = Gait("wave", 5 / 6, {"R3": 0.0, "R2": 1 / 6, "R1": 2 / 6,
                            "L3": 3 / 6, "L2": 4 / 6, "L1": 5 / 6})
# 爬墙步态：等同波浪（任意时刻 5 腿支撑），配合吸附状态机使用
CLIMB = Gait("climb", 5 / 6, WAVE.offsets)


def _smoothstep(s: float) -> float:
    return s * s * (3 - 2 * s)


class GaitEngine:
    def __init__(self, cfg: RobotConfig, gait: Gait = TRIPOD):
        self.cfg = cfg
        self.gait = gait
        # 默认足端位置（身体系，z 相对髋轴平面）
        self.default_feet = {}
        for leg in cfg.legs:
            a = math.radians(leg.mount_angle_deg)
            self.default_feet[leg.name] = (
                leg.mount_x + cfg.foot_reach * math.cos(a),
                leg.mount_y + cfg.foot_reach * math.sin(a),
                -cfg.stand_height,
            )

    def _stride(self, leg_name: str, vx: float, vy: float, wz: float):
        """该腿在一个支撑相内的位移向量（身体系，mm）。
        vx/vy: 身体速度 mm/s；wz: 转向角速度 rad/s。"""
        T_st = self.cfg.cycle_time * self.gait.duty
        px, py, _ = self.default_feet[leg_name]
        # 刚体速度场：v + w x r
        ux = (vx - wz * py) * T_st
        uy = (vy + wz * px) * T_st
        # 限幅到最大步幅
        mag = math.hypot(ux, uy)
        if mag > self.cfg.max_step:
            ux, uy = ux * self.cfg.max_step / mag, uy * self.cfg.max_step / mag
        return ux, uy

    def phase(self, leg_name: str, t: float) -> float:
        return (t / self.cfg.cycle_time + self.gait.offsets[leg_name]) % 1.0

    def stance_legs(self, t: float):
        return [n for n in LEG_NAMES if self.phase(n, t) < self.gait.duty]

    def foot_targets(self, t: float, vx: float = 0.0, vy: float = 0.0,
                     wz: float = 0.0) -> dict:
        """t 时刻各腿足端目标（身体系）。静止指令时全部回默认站位。"""
        targets = {}
        moving = abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(wz) > 1e-6
        for name in LEG_NAMES:
            x0, y0, z0 = self.default_feet[name]
            if not moving:
                targets[name] = (x0, y0, z0)
                continue
            ux, uy = self._stride(name, vx, vy, wz)
            p = self.phase(name, t)
            if p < self.gait.duty:               # 支撑：+u/2 -> -u/2
                s = p / self.gait.duty
                targets[name] = (x0 + ux * (0.5 - s), y0 + uy * (0.5 - s), z0)
            else:                                # 摆动：-u/2 -> +u/2，抬高
                s = (p - self.gait.duty) / (1 - self.gait.duty)
                ss = _smoothstep(s)
                targets[name] = (
                    x0 + ux * (ss - 0.5),
                    y0 + uy * (ss - 0.5),
                    z0 + self.cfg.step_height * math.sin(math.pi * s),
                )
        return targets
