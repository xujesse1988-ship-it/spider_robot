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
    # 吸盘预压行程 mm（P4-GUIDE §4.2）：触面后沿法向再压入这么多才密封。
    # 几何口径 = L1 实测 (h_cup 自由 19 − 密封 7.5) + 预载 1.5 = 13；但 08-19
    # 实机发现指令行程被腿链让差（舵机带载位置误差+齿隙）吃掉大半，13 只压
    # 进波纹几 mm，"踩住"靠抽气后真空收尾。默认加深到 18 用指令超程换真实
    # 压缩（climb_walk --press-delta 可在线对比 13/18）。代价：唇口密封后多
    # 余指令量变成舵机对抗——盯"吸附后总电流回落"判据，不回落就是白烧。
    # 杯批次差 2mm，各腿 h_cup 逐腿实测回填仍欠账（P4 第 7 步）。
    press_delta_mm: float = 18.0


# 官方安装偏角
COXA_ATTACH = -8.0
FEMUR_ATTACH = 35.0
TIBIA_ATTACH = 68.0


def _leg(name, mx, my, ang, ch, touch, coxa_cal=None, femur_cal=None, tibia_cal=None):
    c, f, t = ch
    # 右腿是左腿的镜像装配：femur/tibia 舵机转向与关节约定相反（±45 脉宽对调），
    # coxa 转向不变（左右轴都竖直，镜像不翻转向）但零位镜像（attach 变号）。
    # 官方配置 18 路同为 2000/1000，方向差在官方 app 运动学里消化；本包左右腿
    # 用同一关节约定（α 抬起为正、k 弯曲为正、γ 逆时针为正），方向差收进标定表。
    right = name.startswith("R")
    m45, p45 = (1000.0, 2000.0) if right else (2000.0, 1000.0)
    return LegConfig(
        name=name, mount_x=mx, mount_y=my, mount_angle_deg=ang,
        coxa=coxa_cal or ServoCal(channel=c,
                                  attach_deg=-COXA_ATTACH if right else COXA_ATTACH),
        femur=femur_cal or ServoCal(channel=f, attach_deg=FEMUR_ATTACH,
                                    us_m45=m45, us_p45=p45),
        tibia=tibia_cal or ServoCal(channel=t, attach_deg=TIBIA_ATTACH,
                                    us_m45=m45, us_p45=p45),
        touch_idx=touch,
    )


# 髋关节布局：L1_TO_R1=126, L1_TO_L3=167, L2_TO_R2=163；角部腿倾角 55°
# 舵机通道映射与足底开关索引来自官方配置（TS_L1=P23 ... TS_R3=P18）
# ±45° 标定 2026-08-08 calib_fit.py 全 18 路拟合（样本与残差见 docs/data/calib_pm45.json）
DEFAULT_LEGS = (
    _leg("L1",  83.5,  63.0,  55.0, (15, 16, 17), 23,
         coxa_cal=ServoCal(channel=15, attach_deg=3.31, us_m45=1959.0, us_p45=1041.0),
         femur_cal=ServoCal(channel=16, attach_deg=45.71, us_m45=1973.4, us_p45=1026.6),
         tibia_cal=ServoCal(channel=17, attach_deg=104.06, us_m45=1947.3, us_p45=1052.7)),
    _leg("L2",   0.0,  81.5,  90.0, (9, 10, 11), 21,
         coxa_cal=ServoCal(channel=9, attach_deg=1.55, us_m45=1955.8, us_p45=1044.2),
         # femur 2026-08-09 重拟合：原 4 样本里 1200µs 那个 α=81°，高差法在接近竖直处
         # 正弦变平（1mm 读数误差 ≈ 4.5°），把斜率拉到 -0.1039（其余五路 0.095~0.101）。
         # 去掉后 attach 48.48->47.81、残差 1.34->0.01°。行走工作点 α≈24°(≈1735µs)，
         # 由 1500/1800 两样本夹住，比原拟合可信。
         femur_cal=ServoCal(channel=10, attach_deg=47.81, us_m45=1952.5, us_p45=1047.5),
         tibia_cal=ServoCal(channel=11, attach_deg=94.04, us_m45=1949.5, us_p45=1050.5)),
    _leg("L3", -83.5,  63.0, 125.0, (3, 4, 5), 19,
         coxa_cal=ServoCal(channel=3, attach_deg=18.37, us_m45=1956.9, us_p45=1043.1),
         femur_cal=ServoCal(channel=4, attach_deg=45.25, us_m45=1961.1, us_p45=1038.9),
         tibia_cal=ServoCal(channel=5, attach_deg=100.4, us_m45=1927.1, us_p45=1072.9)),
    _leg("R1",  83.5, -63.0, -55.0, (12, 13, 14), 22,
         coxa_cal=ServoCal(channel=12, attach_deg=3.38, us_m45=1954.9, us_p45=1045.1),
         femur_cal=ServoCal(channel=13, attach_deg=55.7, us_m45=1043.2, us_p45=1956.8),
         tibia_cal=ServoCal(channel=14, attach_deg=101.12, us_m45=1051.0, us_p45=1949.0)),
    _leg("R2",   0.0, -81.5, -90.0, (6, 7, 8), 20,
         coxa_cal=ServoCal(channel=6, attach_deg=-0.21, us_m45=1954.0, us_p45=1046.0),
         femur_cal=ServoCal(channel=7, attach_deg=53.78, us_m45=1054.0, us_p45=1946.0),
         tibia_cal=ServoCal(channel=8, attach_deg=105.08, us_m45=1046.8, us_p45=1953.2)),
    _leg("R3", -83.5, -63.0, -125.0, (0, 1, 2), 18,
         coxa_cal=ServoCal(channel=0, attach_deg=-20.41, us_m45=1959.6, us_p45=1040.4),
         femur_cal=ServoCal(channel=1, attach_deg=47.26, us_m45=1045.1, us_p45=1954.9),
         tibia_cal=ServoCal(channel=2, attach_deg=100.04, us_m45=1058.4, us_p45=1941.6)),
)

LEG_NAMES = tuple(l.name for l in DEFAULT_LEGS)


@dataclass(frozen=True)
class RobotConfig:
    # 连杆长度 mm（官方 COXA_LEN/FEMUR_LEN/TIBIA_LEN）
    coxa_len: float = 43.0
    femur_len: float = 81.0     # 实测舵盘螺丝心距（2026-08-06 同侧缘法复核）；官方设计=80
    tibia_len: float = 123.7    # 2026-08-07 定案：K=输出轴(54.0,-7.2)+h_cup=19，几何链
                                # 123.65 与 08-06 贴纸勾股 123.7 咬合，双腿复测一致
                                # （docs/L3-DISPUTE-OPEN.md 裁决节）；旧 120 锚错 K，官方=134
    # 站立姿态
    stand_height: float = 90.0  # 髋轴平面离地高度
    foot_reach: float = 130.0   # 足端到髋轴的水平距离（沿腿中性方向）
    # 步态
    step_height: float = 40.0   # 抬脚高度 mm（官方 MODE_STANDARD step lift=40）
    max_step: float = 60.0      # 单周期最大步幅 mm
    cycle_time: float = 1.5     # 步态周期 s
    update_hz: float = 50.0
    # 直线跑偏补偿（开环步行必然有残余漂移：标定残差 + 支撑相打滑）。
    # 测法：直行 2m，量末端航向变化和侧移，除以 2 填进来，符号"向左为正"。
    # 只在有前进/后退速度时按 vx 比例生效，静止和原地转向不受影响。
    yaw_trim_deg_per_m: float = 0.0   # 每走 1m 实测左转多少度
    side_trim_mm_per_m: float = 0.0   # 每走 1m 实测左移多少 mm
    # 吸盘轴几何（L3-DISPUTE 裁决节定案；勿再拿 a_t 当吸盘轴——§2.13 教训）：
    # 物理吸盘轴方向 = a_t + cup_delta_deg（a_t = alpha+theta-180 是 K→唇心方向，
    # 与方轴夹 25.2°，K=(54.0,-7.2) 口径，见 html/tibia-three-view.html）。
    # 爬墙站位与落点带由 ClimbEngine 据此解出：压入位吸盘轴 ⊥ 吸附面。
    # ⚠ 口径配套：-25.2 是**自由态**（l3=123.7 目标系）的值，与本包"全程自由态
    # 模型 + press_delta 记账"的做法配套（P4-GUIDE §4.2）；吸附态口径是
    # 27.6°/l3≈113.8（裁决节），只在直接用吸附态 IK 时才用，勿混用。
    cup_delta_deg: float = -25.2
    # 爬墙步态（ClimbEngine 用，设计依据 docs/P4-GUIDE.md 第 4 步）。
    # 摆动相分段速度都是绝对值（mm/s），超出相位窗时长时相位推进自动暂停，
    # 所以先取保守慢速，架空实测后再定值。
    climb_cycle_time: float = 3.6   # 爬墙周期 s（guide 3~4s，跑顺再提速）
    climb_max_step: float = 40.0    # 爬墙单步步幅上限 mm（CLIMBING-DESIGN §6：
                                    # 站高90+reach≈176 下步幅≤40 → 唇口偏角≤11.5°；
                                    # 也约束支撑相真实位移——吸住的脚不能滑）
    climb_sag_comp_mm: float = 0.0  # 下滑补偿：每次抬腿事件把全部支撑足沿身体系
                                    # "下坡"方向（头朝上贴墙=-X，随积分航向旋转）
                                    # 额外平移的距离 mm，顶回"抬腿弹性下沉+重吸附
                                    # 锁死"的棘轮（上墙实测每抬一腿被拽下一截，
                                    # 6 次/周期能吃掉整个步幅=原地踏步）。0=关
                                    # （地面/架空用不上）。上墙实测下沉量后标定，
                                    # 取实测值八成以内——超出真实下沉的部分会把
                                    # 吸附中的盘往下坡拖（climb.py update 4.5 步）
    lift_vent_s: float = 0.3        # 抬腿前先通气时长 s：原地保持、只开排气阀，
                                    # 到时才进 LIFT。08-19 实机：边放气边抬时排气
                                    # 建立慢于抬离，腿是被残余真空"拽起"的
    lift_clearance: float = 15.0    # LIFT 沿面法向退开距离 mm（≥15，吸盘回弹 11~13）
    lift_speed: float = 40.0        # LIFT 抬离速度 mm/s（放气已先行 lift_vent_s）
    transfer_time: float = 0.6      # TRANSFER 平移段时长 s（沿用 smoothstep+正弦形状）
    descend_speed: float = 30.0     # DESCEND 竖直下探速度 mm/s（消灭"拍地"）
    press_speed: float = 15.0       # PRESS 压入速度 mm/s
    retry_lift_mm: float = 5.0      # SUCKING 超时重试的回抬量
    retry_deeper_mm: float = 5.0    # 每次重试比上次再加深的压入量 mm：原深度重压
                                    # 几何缺口不变必然同败（08-19 墙上实测落脚压
                                    # 不到位吸不紧）。双封顶：max_attach_retry×
                                    # 本值 与 总压入 ≤climb.PRESS_DEPTH_MAX=28
                                    # （实测校验点 z=-118；press 18 时深度封顶
                                    # 先到=加深至多 10）；吸上后保持段停在加深
                                    # 后的实际深度
    max_attach_retry: int = 3       # 连续失败次数上限，超过全机冻结报警
    leak_rescue_s: float = 2.0      # ATTACHED 漏气挽救窗口 s，超时全机冻结
    interlock_timeout_s: float = 2.0  # 抬腿互锁不满足的等待上限 s，超时冻结报警
                                      # （与 leak_rescue_s 是两个独立的安全时延）
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
