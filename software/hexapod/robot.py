"""Hexapod 主控类：身体系足端目标 -> 各腿 IK -> 18 路脉宽 -> 驱动板。"""
import math
import time

from .config import RobotConfig, DEFAULT_CONFIG, LEG_NAMES
from .driver import NUM_SERVOS
from .gait import GaitEngine, Gait, TRIPOD
from .kinematics import leg_ik


def _rot_z(x, y, rad):
    c, s = math.cos(rad), math.sin(rad)
    return c * x - s * y, s * x + c * y


class Hexapod:
    def __init__(self, driver, cfg: RobotConfig = DEFAULT_CONFIG,
                 gait: Gait = TRIPOD):
        self.cfg = cfg
        self.driver = driver
        self.engine = GaitEngine(cfg, gait)
        # 身体姿态微调（爬墙阶段用于贴墙姿态）：平移 mm / 偏航俯仰滚转 rad
        self.body_shift = [0.0, 0.0, 0.0]
        self.body_rpy = [0.0, 0.0, 0.0]
        self._last_feet = dict(self.engine.default_feet)

    # ---------- 坐标变换 ----------
    def _body_to_leg(self, leg, p):
        """身体系点 -> 该腿腿坐标系。含身体姿态逆变换。"""
        x, y, z = p
        # 身体平移/旋转的逆变换（roll/pitch 小角度场景，Z-Y-X 顺序）
        x, y, z = x - self.body_shift[0], y - self.body_shift[1], z - self.body_shift[2]
        yaw, pitch, roll = self.body_rpy[2], self.body_rpy[1], self.body_rpy[0]
        if yaw:
            x, y = _rot_z(x, y, -yaw)
        if pitch:
            c, s = math.cos(-pitch), math.sin(-pitch)
            x, z = c * x + s * z, -s * x + c * z
        if roll:
            c, s = math.cos(-roll), math.sin(-roll)
            y, z = c * y - s * z, s * y + c * z
        # 平移到髋轴、转到腿中性方向
        x, y = x - leg.mount_x, y - leg.mount_y
        x, y = _rot_z(x, y, -math.radians(leg.mount_angle_deg))
        return x, y, z

    # ---------- 输出 ----------
    def joint_angles(self, feet_body: dict) -> dict:
        """身体系足端目标 -> {腿名: (gamma, alpha, theta)}（rad）。"""
        out = {}
        for name, p in feet_body.items():
            leg = self.cfg.leg(name)
            out[name] = leg_ik(self.cfg, *self._body_to_leg(leg, p))
        return out

    def pulses(self, feet_body: dict):
        """身体系足端目标 -> 18 路脉宽数组（按通道号排列）。"""
        arr = [None] * NUM_SERVOS
        for name, (g, a, th) in self.joint_angles(feet_body).items():
            leg = self.cfg.leg(name)
            arr[leg.coxa.channel] = leg.coxa.joint_deg_to_us(math.degrees(g))
            arr[leg.femur.channel] = leg.femur.joint_deg_to_us(math.degrees(a))
            # 膝关节变量取弯曲角 k = 180 - theta（伸直=0）；tibia 的 attach_deg 必须存 k 基准值
            # （实测腿 = 180 - θ_center，见 docs/LEG-GEOMETRY-OPEN.md §2.11；官方 68 的基准存疑 §2.9）
            arr[leg.tibia.channel] = leg.tibia.joint_deg_to_us(180.0 - math.degrees(th))
        assert None not in arr, "6 条腿必须全部给目标"
        return arr

    def move_feet(self, feet_body: dict) -> None:
        self.driver.set_all_pulses_us(self.pulses(feet_body))
        self._last_feet = dict(feet_body)

    # ---------- 动作 ----------
    def stand(self, duration: float = 2.0) -> None:
        """从当前足端位置平滑过渡到默认站姿。"""
        self.glide_to(dict(self.engine.default_feet), duration)

    def glide_to(self, feet_target: dict, duration: float) -> None:
        steps = max(2, int(duration * self.cfg.update_hz))
        start = dict(self._last_feet)
        for i in range(1, steps + 1):
            s = i / steps
            s = s * s * (3 - 2 * s)
            mix = {n: tuple(a + (b - a) * s for a, b in
                            zip(start[n], feet_target[n]))
                   for n in feet_target}
            self.move_feet(mix)
            time.sleep(1.0 / self.cfg.update_hz)

    def walk(self, vx: float, vy: float, wz: float, duration: float,
             t0: float = 0.0) -> float:
        """按指定速度行走 duration 秒，返回结束时刻的步态时间（供连续调用）。"""
        dt = 1.0 / self.cfg.update_hz
        t = t0
        end = t0 + duration
        while t < end:
            self.move_feet(self.engine.foot_targets(t, vx, vy, wz))
            time.sleep(dt)
            t += dt
        return t

    # ---------- 传感 ----------
    def touch_states(self) -> dict:
        """{腿名: 是否触地}。"""
        raw = self.driver.read_touch_raw()
        return {leg.name: raw[leg.touch_idx - 18] > 512 for leg in self.cfg.legs}

    def check_power(self):
        """返回 (电压, 电流)，低于保护阈值时抛异常。"""
        v = self.driver.read_voltage_v()
        c = self.driver.read_current_a()
        if v < self.cfg.volt_cutoff:
            raise RuntimeError(f"电池电压 {v:.2f}V 低于保护值 {self.cfg.volt_cutoff}V")
        return v, c
