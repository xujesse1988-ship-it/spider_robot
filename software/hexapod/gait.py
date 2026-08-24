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
# 爬墙步态：对角交替波浪（每侧后->前，两侧错半周期），任意时刻 5 腿支撑。
# 窗序 R3→L1→R2→L3→R1→L2（窗头时刻 = (duty-offset) mod 1 升序）：相邻
# 抬腿全在对角/交叉位（最小间距 167mm），不再同侧三连（旧序 L 三连再 R 三连
# ——半周期扰动集中一侧，08-19 实测 R2/R3 抬腿下坠的结构性根源）也不同排
# 背靠背；同排搭档抬腿间隔恒半周期——前排（上墙剥离力矩集中处）抬其一时
# 另一只已坐实 3 个窗。地面 WAVE 保持原序不动
CLIMB = Gait("climb", 5 / 6, {"R3": 5 / 6, "L1": 4 / 6, "R2": 3 / 6,
                              "L3": 2 / 6, "R1": 1 / 6, "L2": 0.0})
# 双足爬墙步态（docs/DUAL-SWING-DESIGN.md）：偏移与 CLIMB 完全同集，只把
# 占空 5/6→4/6——摆动窗从 1 槽变 2 槽、窗口两两重叠，任意时刻恰 2 腿在窗、
# 稳态恒 4 足吸附。ClimbEngine 的事件驱动钟把窗头攒成"错峰双摆"（对内
# 窗头差 T/6，周期=3×单窗时长）；duty 变小使窗头序整体平移一槽（首腿
# R3→L1），环序不变，仍是对角交替波浪——同刻在空的两腿由"相邻抬腿全在
# 对角/交叉位"性质保证必异侧、必不同排（前排恒 ≥1 只吸附扛剥离力矩，
# 两只最软的中腿永不同时离墙）
CLIMB_DUAL = Gait("climb-dual", 4 / 6, dict(CLIMB.offsets))


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
        if vx:      # 跑偏补偿：与前进速度成正比，反号抵消实测漂移
            wz -= math.radians(self.cfg.yaw_trim_deg_per_m) * vx / 1000.0
            vy -= self.cfg.side_trim_mm_per_m * vx / 1000.0
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
