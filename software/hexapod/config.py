"""机器人配置。

默认值来自 MakeYourPet 官方配置 hardware/makeyourpet-hexapod/chica-config-2040.txt：
连杆长度、髋关节布局、舵机通道映射、±45° 标定格式、安装偏角(*_ATTACH_ANGLE)。
装配完成后需要做的事：
  1. 用 scripts/servo_center.py 让全部舵机回中，按官方视频装舵盘；
  2. 每个舵机实测 -45°/+45° 脉宽，填进 ServoCal.us_m45/us_p45；
  3. 方向反了改 sign，零位偏差改 attach_deg。
"""
from dataclasses import dataclass, field, replace
import math


@dataclass(frozen=True)
class ServoCal:
    """单舵机标定。官方标定格式：[-45° 脉宽] [+45° 脉宽]（默认 2000/1000，即方向反装）。"""
    channel: int            # Servo2040 通道 0..17
    us_m45: float = 2000.0  # 关节角 -45° 时脉宽
    us_p45: float = 1000.0  # 关节角 +45° 时脉宽
    attach_deg: float = 0.0 # 舵机中位时关节实际角度（官方 ATTACH_ANGLE）
    sign: float = 1.0       # 方向修正（装配后校准用，通常 ±1）
    min_us: float = 500.0
    max_us: float = 2500.0

    def joint_deg_to_us(self, joint_deg: float) -> float:
        servo_deg = self.sign * (joint_deg - self.attach_deg)
        center = (self.us_m45 + self.us_p45) / 2
        us = center + servo_deg * (self.us_p45 - self.us_m45) / 90.0
        return min(self.max_us, max(self.min_us, us))


@dataclass(frozen=True)
class LegConfig:
    name: str               # L1..L3 左前/中/后, R1..R3 右前/中/后
    mount_x: float          # coxa 轴在身体坐标系位置 mm（+X 前，+Y 左）
    mount_y: float
    mount_angle_deg: float  # 腿中性朝向（身体系，逆时针为正）
    coxa: ServoCal
    femur: ServoCal
    tibia: ServoCal
    touch_idx: int          # chica 协议 GET 索引（18..23）


# 官方安装偏角
COXA_ATTACH = -8.0
FEMUR_ATTACH = 35.0
TIBIA_ATTACH = 68.0


def _leg(name, mx, my, ang, ch, touch):
    c, f, t = ch
    return LegConfig(
        name=name, mount_x=mx, mount_y=my, mount_angle_deg=ang,
        coxa=ServoCal(channel=c, attach_deg=COXA_ATTACH),
        femur=ServoCal(channel=f, attach_deg=FEMUR_ATTACH),
        tibia=ServoCal(channel=t, attach_deg=TIBIA_ATTACH),
        touch_idx=touch,
    )


# 髋关节布局：L1_TO_R1=126, L1_TO_L3=167, L2_TO_R2=163；角部腿倾角 55°
# 舵机通道映射与足底开关索引来自官方配置（TS_L1=P23 ... TS_R3=P18）
DEFAULT_LEGS = (
    _leg("L1",  83.5,  63.0,  55.0, (15, 16, 17), 23),
    _leg("L2",   0.0,  81.5,  90.0, (9, 10, 11), 21),
    _leg("L3", -83.5,  63.0, 125.0, (3, 4, 5), 19),
    _leg("R1",  83.5, -63.0, -55.0, (12, 13, 14), 22),
    _leg("R2",   0.0, -81.5, -90.0, (6, 7, 8), 20),
    _leg("R3", -83.5, -63.0, -125.0, (0, 1, 2), 18),
)

LEG_NAMES = tuple(l.name for l in DEFAULT_LEGS)


@dataclass(frozen=True)
class RobotConfig:
    # 连杆长度 mm（官方 COXA_LEN/FEMUR_LEN/TIBIA_LEN）
    coxa_len: float = 43.0
    femur_len: float = 80.0
    tibia_len: float = 134.0    # 换吸盘足模块后重新量测（约 +45mm）
    # 站立姿态
    stand_height: float = 90.0  # 髋轴平面离地高度
    foot_reach: float = 130.0   # 足端到髋轴的水平距离（沿腿中性方向）
    # 步态
    step_height: float = 40.0   # 抬脚高度 mm（官方 MODE_STANDARD step lift=40）
    max_step: float = 60.0      # 单周期最大步幅 mm
    cycle_time: float = 1.5     # 步态周期 s
    update_hz: float = 50.0
    # 安全阈值（官方 WARN_*：2S 电池）
    volt_warn: float = 6.4
    volt_cutoff: float = 6.0
    curr_warn: float = 8.0
    legs: tuple = DEFAULT_LEGS

    def leg(self, name: str) -> LegConfig:
        for l in self.legs:
            if l.name == name:
                return l
        raise KeyError(name)


DEFAULT_CONFIG = RobotConfig()
