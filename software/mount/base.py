"""地爬墙自己的一份底层常量与小函数——2026-09-13 从 hexapod/climb.py、hexapod/gait.py
复制，此后与那边脱钩：墙上爬行改这些数不影响地爬墙，反之亦然。注释保留原文（数字的由来）。"""
import math

from hexapod.config import LEG_NAMES
from hexapod.kinematics import leg_ik

# ---- 常量（原 climb.py）----
TANK_READY_KPA = -40.0      # 启动序列首次抽气前罐压须建立到此值（冷罐上电
                            # 直接抽必然重试穷尽误冻结；-40 给 -30 判据留余量）
TANKLESS_PRECHARGE_S = 3.0  # 无罐模式首次抽气前的盲抽时长：阀全关、歧管容积
                            # 小，抽几秒即空——省得首足 SUCK 独扛整段大气歧管
PRECHARGE_TIMEOUT_S = 30.0  # 罐压建立超时 -> 冻结报警（泵坏/大漏不能静默干等）
VENT_STALL_S = 2.0          # 放气等待的额外上限 s，两处共用：VENT 计时满后等本足
                            # 盘压过 lift_release_kpa（不抬）、LIFT 抬到位后等
                            # RELEASED（不平移）。超时冻结报警——排气堵/阀没
                            # 动作/传感器漂移不能静默停摆
D_SAFE_MARGIN = 3.0         # 支撑目标离 IK 包络的最小预留 mm。原硬编码
                            # COMP_TAIL_MAX=40 即 press 13 口径下按本预留反解的
                            # 平面尾预算（tail 40 时后腿最紧 d≈201.7=204.7−3）
PRESS_DEPTH_MAX = 28.0      # press_delta+重试加深的总压入上限 mm（z=-118）：
                            # 实测校验点——满速+满拖尾下最紧 d≈200.4、余 4.3mm；
                            # 更深实测贴死包络（press18+extra15=z-123 时 d=203.7
                            # 只余 1mm）。press 18 时深度封顶 10 先到。
                            # ⚠ 地爬墙的交接欠账落地后压到落地腿身上（18+δ），
                            # 这条上限就是后/中腿 δ 给不大的原因（OPEN §6）
TILT_BAND_DEG = 12.0        # 落点带：压入位物理吸盘轴偏面法线的许用角。
                            # P1 台架实测容差 ±15°，留余量；CLIMBING-DESIGN §6
                            # 接受的工作带 ≤11.5°
_R_BRACKET = (110.0, 210.0)  # 站位半径求解区间 mm（区间内倾角随半径单调增）

# ---- 启动逐足压入的次序（原 gait.CLIMB 的窗序：(duty−offset) mod 1 升序）----
# CLIMB = duty 5/6, offsets R3 5/6, L1 4/6, R2 3/6, L3 2/6, R1 1/6, L2 0 ⇒
ATTACH_ORDER = ("R3", "L1", "R2", "L3", "R1", "L2")


# ---- 站位半径（原 climb.py）----
def _press_tilt(cfg, r, z_press, extra_deg=0.0):
    """(径向 r, 压入深度 z) 姿态下，物理吸盘轴偏离面法线的带符号角（rad）。
    吸盘轴 = a_t + cup_delta（勿拿 a_t 当吸盘轴，LEG-GEOMETRY §2.13 教训）；
    倾角只依赖 (r, z)——coxa 偏摆整体旋转腿平面，不改轴线离垂直的角度。
    cup_tilt_trim_deg（整机实测垂直度修正）在此并入：加正修正后同一半径的
    "轴向角"变大，垂直解/落点带/步幅上限全部自动整体内收。"""
    _, a, th = leg_ik(cfg, r, 0.0, z_press)
    a_t = a + th - math.pi
    return a_t + math.radians(cfg.cup_delta_deg + cfg.cup_tilt_trim_deg + extra_deg) \
        + math.pi / 2


def _solve_reach(cfg, z_press, tilt_rad=0.0, extra_deg=0.0):
    """解站位半径：压入位吸盘轴偏法线 = tilt_rad（0 = 严格垂直）。extra_deg = 该腿的逐腿修正。"""
    lo, hi = _R_BRACKET
    if _press_tilt(cfg, lo, z_press, extra_deg) >= tilt_rad:
        return lo
    if _press_tilt(cfg, hi, z_press, extra_deg) <= tilt_rad:
        return hi
    for _ in range(48):
        mid = (lo + hi) / 2.0
        if _press_tilt(cfg, mid, z_press, extra_deg) < tilt_rad:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


# ---- 命令行参数解析（原 climb.py）----
def parse_handover(spec):
    """解析 --handover 参数为 {腿名: δmm}（未给的腿 0）。两种格式：
    "8" = 六腿统一；"L1:17,R3:9" = 逐腿设（可只给部分腿，腿名不分大小写）。
    范围 0~45mm（08-24 三组原地标定外推储能上界 δ*≈28~42mm/腿 + 余量）。
    非法格式/腿名/越界抛 ValueError（脚本层转 ap.error）。"""
    out = {n: 0.0 for n in LEG_NAMES}
    try:
        if ":" in spec:
            for tok in spec.split(","):
                name, _, val = tok.strip().partition(":")
                name = name.strip().upper()
                if name not in LEG_NAMES:
                    raise ValueError(f"未知腿 {name}")
                out[name] = float(val)
        else:
            out = {n: float(spec) for n in LEG_NAMES}
    except ValueError as e:
        raise ValueError(f"--handover 格式错（{spec!r}）：统一值如 8，或逐腿 "
                         f"L1:17,R3:9——{e}") from None
    for n, v in out.items():
        if not 0.0 <= v <= 45.0:
            raise ValueError(f"--handover {n}={v:g} 越界：范围 0~45mm"
                             "（08-24 外推储能上界 δ*≈42 加余量；"
                             "更大先查标定口径是不是把弹跳当了储能）")
    return out


def parse_leg_order(spec):
    """解析 --attach-order 为六腿元组。格式 L1_R1_L2_R2_L3_R3（下划线或
    逗号分隔，腿名不分大小写），必须恰是六腿的一个排列（抛 ValueError，脚本层转 ap.error）。"""
    names = tuple(t.strip().upper()
                  for t in spec.replace(",", "_").split("_") if t.strip())
    if sorted(names) != sorted(LEG_NAMES):
        raise ValueError(
            f"--attach-order 必须是六腿的一个排列（如 {'_'.join(LEG_NAMES)}，"
            f"不缺不重），给了 {spec!r}")
    return names
