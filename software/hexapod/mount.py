"""地-墙过渡引擎（P5 探索线，2026-09-06）：足端可以分别落在地面和墙面，
身体可以俯仰/平移而接触足在世界系钉死不动。

为什么不扩 ClimbEngine：它把"吸附面 = 身体系 z0 平面、压入方向恒为 -z、
落点 xy 在身体系"焊死在每个分段里（相位钟、支撑场、交接、补偿全基于此）。
过渡阶段每条腿各有各的面（前足在墙、后足在地、中腿悬空）而且身体要转，
硬塞进去要把 1400 行稳态爬墙逻辑翻一遍。本引擎只做过渡实验需要的三件事，
吸附状态机（AdhesionController）与黑匣子（runlog.ClimbWatch）原样复用：

  1. 每腿一个接触面 Surface（世界系平面 n·p=d，n 指向机器人一侧）。压入、
     抬离都沿该面法向；落点与压入位的吸盘轴倾角按三维算——含 coxa 偏摆造成
     的面外分量：地面足 asin(sinφ·|sinβ|)、墙面足 asin(cosφ·|sinβ|)
     （φ 俯仰，β 腿平面相对机身 x 轴；tools/mount_analysis.py 一节）。
     这条规律决定了过渡顺序只能是 前→后→中：前腿 coxa 内摆到指正前、后腿
     后摆到指正后时面外角恒 0；中腿在 φ∈(20°,70°) 谁都对不正，只能悬空。
  2. 单腿从一个面挪到另一个面（或收到空中）：VENT→LIFT→TRANSFER→HOVER→
     (land)→DESCEND→PRESS→WAIT，分段语义与 ClimbEngine 一致（先通气再抬、
     悬停等人确认落点、慢速下探、压入 press_delta、FAULT 加深重试、耗尽冻结、
     漏气挽救超时冻结）。一次只挪一条腿；抬腿前其余**接触**腿须全 ATTACHED
     且盘压过抬腿门槛（悬空腿豁免——它们本来就不承载）。
  3. 身体位姿 (xb, zb, pitch) 慢速改变：接触足世界系不动、悬空足随身体。
     body_lean 倾身的三维推广。请求按 1°/5mm 采样整段中间位姿逐个预检
     （IK 余量/行程/倾角/膝不撞面/腹面不触面）才受理，不受理就不动。
  4. 零力交接（HANDOVER）与接管（TAKEOVER）：抬腿前把该腿的力卸到零、其余
     接触腿按份额接住；落地吸住后反向做一次，把载荷（也就是其余腿攒着的
     弹性势能）转移到刚吸上的新腿。docs/HANDOVER-DESIGN.md 的三维推广，
     治 09-09 实测的"抬一只前足机身下沉 26mm 且不回弹"。

⚠ 交接方向是**世界竖直**，不是接触面法向（WALL-MOUNT-OPEN §7 原先写的）：
载荷是重力，地面足的竖直恰好=法向（等价于改压深），墙面足的竖直是**切向**
（沿墙下滑方向=剪切，正是 climb.py 4.7 步"沿下坡"的那个方向）。往墙里压
一点并不接体重，按法向卸载墙面足会一点力都卸不掉。竖直口径同时退化成两者。

坐标：世界系 地面 z=0、墙面 x=0（机器人在 x<0 侧，头朝墙 +x）、y 左；
身体系 x 前 y 左 z 上；身体位姿 (xb, zb, pitch)，抬头为正（绕 y）。
不做：步态/连续行走、下滑补偿、双足。

假设（待实机核，见 tools/mount_analysis.py）：coxa 相对中性最多摆 COXA_MAX_DEG；
髋平面以下舱体厚 BELLY_MM；机身外廓 ±BODY_HALF_MM；重心不参与（纯运动学）。
"""
import math
from enum import Enum

from .adhesion import FootState
from .climb import (D_SAFE_MARGIN, PRESS_DEPTH_MAX, TILT_BAND_DEG,
                    VENT_STALL_S, TANK_READY_KPA, TANKLESS_PRECHARGE_S,
                    PRECHARGE_TIMEOUT_S, _solve_reach)
from .config import RobotConfig, LEG_NAMES
from .gait import CLIMB
from .kinematics import leg_ik, WorkspaceError

_EPS = 1e-6
COXA_MAX_DEG = 60.0        # coxa 相对中性的最大偏摆（前腿指正前/后腿指正后需 55°）
JOINT_MARGIN_DEG = 2.0     # 舵机电气行程（attach±90°）内缩量
BELLY_MM = 40.0            # 髋平面（femur 轴平面）以下舱体厚度：09-08 实测约 35，取 40 留余量；
                           # 舱体前后不超出框架外廓（±BODY_HALF_MM 成立）
BODY_HALF_MM = 100.0       # 机身外廓半长/半宽（frame 198×205）
BODY_CLEAR_MM = 5.0        # 机身/腹面离任何面的最小净空
KNEE_CLEAR_MM = 8.0        # 膝离任何面的最小净空
FOOT_AIR_CLEAR_MM = 10.0   # 悬空足离任何面的最小净空（位姿铺设预检）
HOLD_TILT_DEG = 15.0       # 已吸附足在位姿改变中允许的指令倾角（吸盘容差）
SUPPORT_TILT_DEG = 35.0    # **只承重腿**（support_only，只压不吸）的倾角容差：盘面对不对正
                           # 不影响它传正压力，15° 那条是吸盘**密封**的容差，套在它头上是
                           # 错的——按此中腿在俯仰 >30° 就被拒，而几何上它能撑到 90°。
                           # 09-12 台架实测（LAB E4a）：盘面斜 35° 时接触仍在盘面、仍压得住；
                           # 35° 也正是中腿在俯仰 90° 时盘面最多斜到的角度（coxa ±55° 口径，
                           # OPEN §1.1），所以这个值等于"倾角这条永远不卡中腿"。
                           # 再大没有意义（几何到不了），要改用 --support-tilt
FLOOR_CLEAR_MM = 45.0      # 地面移动的抬离高度 mm（默认，--floor-clear 可调）：
                           # cfg.lift_clearance=15 是按吸盘回弹 11~13mm 定的、够墙面
                           # 用（墙面法向上没有重力下垂），但地面上腿一抬就因自重
                           # 下垂十几到二十几毫米（08-19 量化同源），15mm 的抬离量
                           # 让盘在摆动全程蹭着玻璃地板——09-09 实机 L2/R2 摆动时
                           # 吸盘碰到地板
NUDGE_SPEED_MMS = 20.0     # 悬停中挪落点的铺设速率 mm/s：盘离面只十几毫米时
                           # 不能一步跳过去（步进指令会让腿冲一下蹭到面）
PITCH_RATE_DPS = 2.0       # 位姿铺设：俯仰速率 °/s
LIN_RATE_MMS = 10.0        # 位姿铺设：平移速率 mm/s（= body_lean LEAN_SPEED）
TRANSFER_SPEED_MMS = 60.0  # 摆动平移速度 mm/s（过渡的挪腿动辄 150~250mm）
TUCK_RADIUS = 0.6          # 收腿点：髋外 0.6×站位半径
TUCK_RISE_MM = 45.0        # 收腿点：站位平面以上高度
POSE_SAMPLE_DEG = 1.0      # 位姿预检采样粒度
POSE_SAMPLE_MM = 5.0
PATH_SAMPLES = 12          # 摆动路径预检采样点数
HO_SAMPLE_MM = 2.0         # 交接/接管铺设的预检采样粒度 mm（逐点过 IK/倾角/膝）
TAKEOVER_STEP_MM = 3.0     # 手动接管（脚本 z 键）每按一次转移的量 mm
TAKEOVER_MAX_MM = 40.0     # 单次接管请求上限 mm（预检另有压深/包络/面上三重硬界）

d2r, r2d = math.radians, math.degrees


# ---------------------------------------------------------------- 几何
def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(a):
    n = math.sqrt(_dot(a, a))
    return (a[0] / n, a[1] / n, a[2] / n)


def _add(a, b, k=1.0):
    return (a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2])


class Surface:
    """世界系平面 n·p = d，n 为单位法向、指向机器人一侧。"""

    def __init__(self, name, n, d):
        self.name, self.n, self.d = name, _norm(n), float(d)

    def height(self, p):
        """点离面的法向距离（正 = 在机器人一侧）。"""
        return _dot(self.n, p) - self.d

    def project(self, p):
        return _add(p, self.n, -self.height(p))

    def __repr__(self):
        return f"Surface({self.name})"


FLOOR = Surface("floor", (0.0, 0.0, 1.0), 0.0)


def wall_at(x):
    """竖直墙面 x = x_wall，机器人在 x 更小的一侧。"""
    return Surface("wall", (-1.0, 0.0, 0.0), -float(x))


def rot_b2w(phi):
    c, s = math.cos(phi), math.sin(phi)
    return ((c, 0.0, -s), (0.0, 1.0, 0.0), (s, 0.0, c))   # 行=世界轴 列=身体轴


def b2w(v, pose):
    xb, zb, phi = pose
    R = rot_b2w(phi)
    return tuple(sum(R[i][j] * v[j] for j in range(3)) + (xb, 0.0, zb)[i]
                 for i in range(3))


def w2b(v, pose):
    xb, zb, phi = pose
    R = rot_b2w(phi)
    dv = (v[0] - xb, v[1], v[2] - zb)
    return tuple(sum(R[j][i] * dv[j] for j in range(3)) for i in range(3))


def w2b_dir(v, pose):
    R = rot_b2w(pose[2])
    return tuple(sum(R[j][i] * v[j] for j in range(3)) for i in range(3))


def b2w_dir(v, pose):
    R = rot_b2w(pose[2])
    return tuple(sum(R[i][j] * v[j] for j in range(3)) for i in range(3))


class Infeasible(ValueError):
    """目标不可行（原因文本给操作者看）。"""


class LegGeom:
    """单腿几何：身体系足端点 -> 关节角/吸盘轴/膝点，含行程、包络余量、
    coxa 偏摆判据。关节行程按该腿 ServoCal.attach_deg±(90−JOINT_MARGIN)
    取（舵机电气 ±90°；tibia 的 attach 存 k=180−θ 基准）。"""

    def __init__(self, cfg: RobotConfig, leg, coxa_max_deg=COXA_MAX_DEG):
        self.cfg, self.leg = cfg, leg
        self.psi = d2r(leg.mount_angle_deg)
        m = 90.0 - JOINT_MARGIN_DEG
        self.alpha_lim = (leg.femur.attach_deg - m, leg.femur.attach_deg + m)
        k0 = leg.tibia.attach_deg
        self.theta_lim = (max(0.0, 180.0 - k0 - m), min(180.0, 180.0 - k0 + m))
        self.coxa_max = coxa_max_deg
        self.delta = d2r(cfg.cup_delta_deg + cfg.cup_tilt_trim_deg)
        self.d_safe = cfg.femur_len + cfg.tibia_len - D_SAFE_MARGIN

    def to_leg(self, p_body):
        x, y = p_body[0] - self.leg.mount_x, p_body[1] - self.leg.mount_y
        c, s = math.cos(self.psi), math.sin(self.psi)
        return (x * c + y * s, -x * s + y * c, p_body[2])

    def solve(self, p_body, press_dir_body=None):
        """返回 dict(gamma, alpha, theta[deg], knee_b, tilt, oop)；不可达/超行程
        抛 Infeasible。tilt = 物理吸盘轴与压入方向夹角（°），oop = 其面外分量。"""
        xl, yl, z = self.to_leg(p_body)
        try:
            g, a, th = leg_ik(self.cfg, xl, yl, z)
        except WorkspaceError:
            raise Infeasible("IK 不可达")
        r = math.hypot(xl, yl) - self.cfg.coxa_len
        if math.hypot(r, z) > self.d_safe:
            raise Infeasible(f"距 IK 包络不足 {D_SAFE_MARGIN:g}mm")
        gd, ad, thd = r2d(g), r2d(a), r2d(th)
        if abs(gd) > self.coxa_max:
            raise Infeasible(f"coxa 偏摆 {gd:+.0f}° 超 ±{self.coxa_max:.0f}°")
        if not (self.alpha_lim[0] <= ad <= self.alpha_lim[1]):
            raise Infeasible(f"femur {ad:.0f}° 超行程")
        if not (self.theta_lim[0] <= thd <= self.theta_lim[1]):
            raise Infeasible(f"膝角 {thd:.0f}° 超行程")
        cg, sg = math.cos(g), math.sin(g)
        cp, sp = math.cos(self.psi), math.sin(self.psi)
        kx = self.cfg.coxa_len * cg + self.cfg.femur_len * math.cos(a) * cg
        ky = self.cfg.coxa_len * sg + self.cfg.femur_len * math.cos(a) * sg
        kz = self.cfg.femur_len * math.sin(a)
        knee_b = (kx * cp - ky * sp + self.leg.mount_x,
                  kx * sp + ky * cp + self.leg.mount_y, kz)
        out = dict(gamma=gd, alpha=ad, theta=thd, knee_b=knee_b,
                   tilt=None, oop=None)
        if press_dir_body is not None:
            a_c = a + th - math.pi + self.delta          # 腿平面内吸盘轴角
            axis_leg = (math.cos(a_c) * cg, math.cos(a_c) * sg, math.sin(a_c))
            axis_b = (axis_leg[0] * cp - axis_leg[1] * sp,
                      axis_leg[0] * sp + axis_leg[1] * cp, axis_leg[2])
            pd = _norm(press_dir_body)
            out["tilt"] = r2d(math.acos(max(-1.0, min(1.0, _dot(axis_b, pd)))))
            beta = self.psi + g
            n_plane = (-math.sin(beta), math.cos(beta), 0.0)
            out["oop"] = abs(r2d(math.asin(max(-1.0, min(1.0, _dot(n_plane, pd))))))
        return out


# ---------------------------------------------------------------- 引擎
class MountPhase(Enum):
    STANCE = "stance"     # 接触面上（吸附中或启动前等待压入）
    AIR = "air"           # 收在空中（不承载，位姿改变时随身体）
    HANDOVER = "handover"  # 抬腿前零力交接：本腿指令竖直还 δ、其余接触腿各接 δ/n
    TAKEOVER = "takeover"  # 接管（反向交接）：本腿指令竖直压 T、其余各还 T/n
    VENT = "vent"
    LIFT = "lift"
    TRANSFER = "transfer"
    HOVER = "hover"       # 落点前方 lift_clearance 处悬停，等 land()
    DESCEND = "descend"
    PRESS = "press"
    RETRY_LIFT = "retry"
    WAIT = "wait"


SWING_PHASES = (MountPhase.VENT, MountPhase.LIFT, MountPhase.TRANSFER,
                MountPhase.HOVER, MountPhase.DESCEND, MountPhase.PRESS,
                MountPhase.RETRY_LIFT, MountPhase.WAIT)
# 交接/接管：脚还吸在面上（密封、承载），只是指令在被慢慢加减载——阀绝不能动
# （干跑真阀按 SWING_PHASES 通电排气，把这两段算进去等于交接期间就把盘放了）
HO_PHASES = (MountPhase.HANDOVER, MountPhase.TAKEOVER)
# "这条腿正忙"：互锁与位姿铺设看这个，比 SWING_PHASES 多两段交接
BUSY_PHASES = SWING_PHASES + HO_PHASES


class MountEngine:
    """每控制周期调用 update(dt) 取身体系足端目标；内部替调用方跑
    AdhesionController.update(dt)。启动：六足在地面爬墙站位逐足压入吸附
    （与 ClimbEngine 同序），全部 ATTACHED 后 started=True 才受理命令。"""

    def __init__(self, cfg: RobotConfig, ctl, wall_x=0.0, front_hip_to_wall=140.0,
                 pitch_max_deg=90.0, ignore_tank_fault=False, attach_order=None,
                 floor_clear_mm=FLOOR_CLEAR_MM, support_only=(), takeover_mm=None,
                 support_tilt_deg=SUPPORT_TILT_DEG):
        self.cfg, self.ctl = cfg, ctl
        self.ignore_tank_fault = ignore_tank_fault
        self.wall = wall_at(wall_x)
        self.surfaces = (FLOOR, self.wall)
        self.pitch_max = d2r(pitch_max_deg)
        self.floor_clear = float(floor_clear_mm)
        # 只承重不吸附的腿（09-09 实机：上墙过程中中腿吸盘吸不住地面，但压着能靠
        # 摩擦当支撑）。这些腿：压到位即回支撑不抽气、不进 VENT（无真空可放）、
        # 不参与互锁与漏气监护。⚠ 只能承压不能承拉——身体俯仰后它们扛不住剥离
        # 力矩，靠它们的支撑随俯仰增大而失效
        self.support_only = set(support_only)
        bad = self.support_only - set(LEG_NAMES)
        if bad:
            raise ValueError(f"support_only 含未知腿 {sorted(bad)}")
        if not (0.0 < float(support_tilt_deg) <= 60.0):
            raise ValueError(f"support_tilt_deg {support_tilt_deg!r} 非法：0~60°")
        self.support_tilt = float(support_tilt_deg)
        self.geom = {leg.name: LegGeom(cfg, leg) for leg in cfg.legs}
        self.slot_order = tuple(sorted(
            LEG_NAMES, key=lambda n: (CLIMB.duty - CLIMB.offsets[n]) % 1.0))
        # 启动逐足压入的次序（默认=窗序）。诊断用：可疑的腿排最后，等其余五足
        # 都吸牢当反力座再压它——启动早段只有一两足吸住，压入的反力会把机身
        # 顶起/顶偏而不是把盘压进面里（08-19"六足同时压没有反力座"同源）
        self.attach_order = tuple(attach_order) if attach_order else self.slot_order
        if sorted(self.attach_order) != sorted(LEG_NAMES):
            raise ValueError(f"attach_order 必须是六腿的一个排列，给了 {attach_order!r}")
        # 爬墙站位（与 ClimbEngine 同解：压入位吸盘轴 ⊥ 面）
        self.z0 = -cfg.stand_height
        self.default_feet = {}
        self.r0 = {}
        for leg in cfg.legs:
            r0 = _solve_reach(cfg, self.z0 - leg.press_delta_mm)
            a = d2r(leg.mount_angle_deg)
            self.r0[leg.name] = r0
            self.default_feet[leg.name] = (leg.mount_x + r0 * math.cos(a),
                                           leg.mount_y + r0 * math.sin(a),
                                           self.z0)
        # 身体位姿：前髋距墙 front_hip_to_wall、平身、髋高 stand_height
        self.wall_x = float(wall_x)
        self._front_x = max(l.mount_x for l in cfg.legs)
        self.pose = (self.wall_x - front_hip_to_wall - self._front_x,
                     cfg.stand_height, 0.0)
        self.pose0 = self.pose
        # 逐腿状态
        self.surf = {}        # 腿 -> Surface | None（None=空中）
        self.pw = {}          # 腿 -> 接触点（世界系，在面上）
        self.depth = {}       # 腿 -> 当前沿 -n 的压入深度 mm（负=离面）
        self.air_pb = {}      # 腿 -> 空中点（身体系）
        self.phase_of = {n: MountPhase.STANCE for n in LEG_NAMES}
        self.retries = {n: 0 for n in LEG_NAMES}
        self._press_extra = {n: 0.0 for n in LEG_NAMES}
        self.landing = {}     # 腿 -> 落点 (x,y) 身体系（ClimbWatch 相位事件用）
        for n, p in self.default_feet.items():
            self.surf[n] = FLOOR
            self.pw[n] = FLOOR.project(b2w(p, self.pose))
            self.depth[n] = 0.0
            self.landing[n] = p[:2]
        self.foot = {n: list(p) for n, p in self.default_feet.items()}
        self._sw = {}         # 摆动簿：腿 -> dict
        self._seg_t = {n: 0.0 for n in LEG_NAMES}
        self._last_ph = dict(self.phase_of)
        self.frozen = None
        self.started = False
        self.t = 0.0
        self._attach_queue = list(self.attach_order)
        self._precharge_t = 0.0
        self._tankless_precharged = False
        # 悬停落点挪动目标（腿名 -> 面上的目标点）：update() 按 NUDGE_SPEED_MMS
        # 匀速把 pw 铺过去，离面这么近不许步进
        self._pw_goal = {}
        # 位姿铺设
        self._pose_from = self._pose_to = None
        self._pose_s = 0.0
        self._pose_T = 0.0
        # 零力交接（docs/HANDOVER-DESIGN.md，本引擎 §4）。ho_off = 该腿足端指令在
        # **世界竖直**方向上的附加偏移 mm（+ 上 = 卸载自己，− 下 = 多接载荷）；
        # 六腿偏移的代数和恒为 0 = 身体指令不动（吸住的脚不能滑 ⇒ 改指令=改力）。
        # 偏移只加在指令上，pw（吸盘真实钉在面上的点）始终是物理真值
        self.ho_off = {n: 0.0 for n in LEG_NAMES}
        self._ho_goal = {n: 0.0 for n in LEG_NAMES}
        self._ho_start = {n: 0.0 for n in LEG_NAMES}   # 本次铺设起点（记欠账用）
        self._ho_move = {}        # 本次铺设各腿的总位移（世界竖直 mm）
        self._ho_s = 1.0          # 铺设进度 0~1（全体腿共用，保住"和为 0"）
        self.ho_span = 0.0        # 本次铺设里位移最大的那条腿的量 mm（限速基准
                                  # 兼显示：卸载量 δ 或接管量 T）
        # 欠账：腿 -> 其余腿一共替它接走了多少 mm（按实际铺进去的量记，可能被
        # 截断过）。该腿落地吸住后由 TAKEOVER 还回去——不还的话份额会在支撑腿
        # 上只进不出，34 个单腿动作的序列几次就把压深余量吃穿。只记总量不记
        # 债主：还的时候按**当时**的接触腿均摊（中间可能有腿换过面），零和照旧
        self._ho_debt = {}
        self.handover_note = None      # 截断/退化留痕（脚本打印+落黑匣子）
        r = cfg.handover_rate_mms
        if not (0.0 < r <= 50.0) or r != r:
            raise ValueError(f"handover_rate_mms 非法（{r!r}）：须有限且 0<r≤50")
        self._ho_rate = float(r)
        # 落地后主动接管的量 mm（逐腿，0=只把欠账还回去）：L1 上墙吸住后其余腿
        # 还攒着弹性势能，往新腿身上转载荷=就地把势能卸掉，下一条腿再抬时储能
        # 已经小了。墙面腿接的是剪切（实测容量 15N），宁欠勿过、逐次加
        self.takeover = {n: 0.0 for n in LEG_NAMES}
        for n, v in (takeover_mm or {}).items():
            if n not in LEG_NAMES:
                raise ValueError(f"takeover_mm 含未知腿 {n!r}")
            if not 0.0 <= float(v) <= TAKEOVER_MAX_MM:
                raise ValueError(f"takeover_mm {n}={v} 越界：0~{TAKEOVER_MAX_MM:g}mm")
            self.takeover[n] = float(v)
        self._gate_t = 0.0             # 交接铺完后卡在放气门槛的等待计时
        # 墙面目标修正 mm，**逐腿**（正=再往墙里压）：腿在"前伸上举"姿态的实际
        # 到达比模型短，且逐腿不同——09-09 实机 wall_dist=155、修正 16 时 L1 停在
        # 玻璃外 15mm（短 16）而 R1 已贴上（几乎不差）；全局一个值按 L1 标好会把
        # R1 按进玻璃里（对刚性面=舵机堵转）。只作用于墙面接触/悬停/压入目标，
        # 地面不动；set_wall_trim 改，脚本键 . , 在线逐腿试
        self.wall_trim = {n: 0.0 for n in LEG_NAMES}

    # ---------- 对外：查询 ----------
    def targets(self):
        return {n: tuple(f) for n, f in self.foot.items()}

    def status(self):
        return {n: (self.phase_of[n].value,
                    self.ctl.state[LEG_NAMES.index(n)].value,
                    self.ctl.is_leaking(LEG_NAMES.index(n)))
                for n in LEG_NAMES}

    @property
    def pitch_deg(self):
        return r2d(self.pose[2])

    @property
    def swing_leg(self):
        """动作在途的腿名（含悬停、交接、接管），无则 None。一次只许一条。"""
        for n in LEG_NAMES:
            if self.phase_of[n] in BUSY_PHASES:
                return n
        return None

    @property
    def ho_leg(self):
        """正在交接/接管的腿名，无则 None。"""
        for n in LEG_NAMES:
            if self.phase_of[n] in HO_PHASES:
                return n
        return None

    @property
    def hover_leg(self):
        for n in LEG_NAMES:
            if self.phase_of[n] == MountPhase.HOVER:
                return n
        return None

    @property
    def pose_pending(self):
        return self._pose_to is not None

    def contact_legs(self):
        """踩在面上承载的腿（交接/接管中的腿仍然踩着，算在内）。"""
        return tuple(n for n in LEG_NAMES if self.surf[n] is not None
                     and self.phase_of[n] in (MountPhase.STANCE,) + HO_PHASES)

    def _bearing(self, name):
        """吸附口径的承载腿：在面上、STANCE/交接中、且不是只承重腿（那些没真空，
        互锁与漏气监护都不看它们）。"""
        return (self.surf[name] is not None and name not in self.support_only
                and self.phase_of[name] in (MountPhase.STANCE,) + HO_PHASES)

    def _share_legs(self, name):
        """接/还载荷的腿：除 name 外所有踩在面上且已收口（STANCE）的腿。只承重腿
        算在内——它们靠摩擦支撑，多压一点就是多一点正压力；悬空/悬停腿不算。"""
        return [n for n in LEG_NAMES if n != name and self.surf[n] is not None
                and self.phase_of[n] == MountPhase.STANCE]

    def hip_world(self, name, pose=None):
        leg = self.cfg.leg(name)
        return b2w((leg.mount_x, leg.mount_y, 0.0), pose or self.pose)

    def front_hip_to_wall(self, pose=None):
        front = max(LEG_NAMES, key=lambda n: self.cfg.leg(n).mount_x)
        return self.wall.height(self.hip_world(front, pose))

    def set_wall_dist(self, d):
        """启动前改前髋距墙（就位暂停时按卷尺实测值修正）：整机世界系 x 平移，
        六足地面接触点随之重投影。启动后（有足已吸附）不许改。返回 None=成功。"""
        if self.started or self._attach_queue != list(self.attach_order):
            return "启动序列已开始，不可改"
        self.pose = (self.wall_x - float(d) - self._front_x, self.pose[1], 0.0)
        self.pose0 = self.pose
        for n, p in self.default_feet.items():
            self.pw[n] = FLOOR.project(b2w(p, self.pose))
        return None

    # ---------- 对外：几何工具（脚本层选落点用）----------
    def wall_target(self, name, height, y_w=None):
        """墙面落点：离地 height，侧向 y_w（默认=髋的世界 y，即腿指正前 β=0）。"""
        if y_w is None:
            y_w = self.hip_world(name)[1]
        return self.wall.project((0.0, y_w, float(height)))

    def wall_band(self, name, y_w=None, step=5.0, zmax=600.0):
        """当前位姿下该腿能落到墙上的高度带 (lo, hi)（离地 mm，压入位倾角
        ≤TILT_BAND，含压深/加深深度）；无则 None。"""
        ok = [z for z in _frange(0.0, zmax, step)
              if self._check_landing(name, self.wall_target(name, z, y_w),
                                     self.wall, self.pose) is None]
        return (min(ok), max(ok)) if ok else None

    def wall_perp_height(self, name, y_w=None, step=1.0):
        """带内"吸盘轴⊥墙"的落点高度（离地 mm），无带返回 None。

        **抬头余量几乎全压在这一个数上**（09-12 LAB E5 实测+复算）：落点每高 1mm
        约多 0.45° 四接触抬头上限，接管量每 1mm 抵消同样多——两者花的是吸盘倾角
        那**同一笔** 15° 预算（接管沿墙竖直挪指令点，等价于落点低同样多）。
        脚本缺省取落足带**中点**，那是照顾落点密封的选法，比 ⊥ 点低不了几毫米，
        但比它**低**就是净亏；要抬头就往带上沿放（代价=落点倾角变大，密封余量变小）。
        """
        band = self.wall_band(name, y_w)
        if band is None:
            return None
        depth = self.cfg.leg(name).press_delta_mm + self._trim(name, self.wall)
        pd = w2b_dir(_scale(self.wall.n, -1.0), self.pose)
        best, best_t = None, None
        for z in _frange(band[0], band[1], step):
            q = _add(self.wall_target(name, z, y_w), self.wall.n, -depth)
            try:
                sol = self.geom[name].solve(w2b(q, self.pose), pd)
            except Infeasible:
                continue
            if best_t is None or sol["tilt"] < best_t:
                best, best_t = z, sol["tilt"]
        return best

    def wall_pitch_room(self, name, height, y_w=None, takeover_mm=None,
                        step=1.0, pitch_max_deg=45.0):
        """把 name 落到墙上 height、落地接管 takeover_mm 之后，**这条腿的吸盘倾角**
        还允许把身体抬到多少度（绝对俯仰°，只算这一条腿；其余腿各有各的界，
        真正的上限取最小）。落点当场就超容差返回 None。

        算法就是拿 HOLD_TILT_DEG 那条判据逐度试：墙面腿的接管是**世界竖直**的
        指令偏移（沿墙下滑），所以它和落点高度是同一个自由度——接管 1mm 等于
        落点低 1mm。
        """
        depth = self.cfg.leg(name).press_delta_mm + self._trim(name, self.wall)
        q = _add(self.wall_target(name, height, y_w), self.wall.n, -depth)
        off = -(self.takeover[name] if takeover_mm is None else float(takeover_mm))
        xb, zb, _ = self.pose
        tol = self._tilt_lim(name)
        last = None
        for p in _frange(0.0, pitch_max_deg, step):
            pose = (xb, zb, d2r(p))
            try:
                sol = self.geom[name].solve(
                    w2b((q[0], q[1], q[2] + off), pose),
                    w2b_dir(_scale(self.wall.n, -1.0), pose))
            except Infeasible:
                return last
            if sol["tilt"] > tol:
                return last
            last = p
        return last

    def floor_home(self, name):
        """该腿爬墙站位在当前位姿下投影到地面的点。"""
        return FLOOR.project(b2w(self.default_feet[name], self.pose))

    def floor_forward(self, name, dist):
        """该腿地面落点绕髋**摆**到站位前方 dist 处（+ 朝墙），**保持髋足距离不变**。
        用途：把中腿走到前髋底下再做前足上墙——默认站位下中足在机身中心正下方
        （身体系 x=0），与重心几乎重合，抬起第二只前足时前半机身成悬臂，
        09-09 实测机身前缘下沉 26mm 且吸住后不回弹。
        ⚠ 必须摆不能平移：吸盘轴对面的倾角只由髋足距离（与压深）决定，站位半径
        是"轴⊥面"的解；平移会把半径拉长（中腿前移 85 时 176.6→196.0，倾角 10.6°，
        卡在 12° 带内侧不被拒但目视明显斜，09-09 实机复现）。摆动下倾角恒 0。
        超出该腿半径（|前向分量| > r）时返回 None，调用方按不可行处理。"""
        hip = self.hip_world(name)
        home = self.floor_home(name)
        ox, oy = home[0] - hip[0], home[1] - hip[1]
        r = math.hypot(ox, oy)
        fx, fy, _ = b2w_dir((1.0, 0.0, 0.0), self.pose)
        k = math.hypot(fx, fy)
        if k < 1e-9 or r < 1e-9:
            return home
        ux, uy = fx / k, fy / k          # 机身前进方向在地面上的投影
        vx, vy = -uy, ux                 # 其左法向
        a = ox * ux + oy * uy            # 站位落点相对髋的前向/侧向分量
        b = ox * vx + oy * vy
        a2 = a + float(dist)
        if abs(a2) > r - 1e-9:
            return None                  # 摆不到（前向分量超过髋足距离）
        sgn = math.copysign(1.0, b if abs(b) > 1e-9 else self.cfg.leg(name).mount_y)
        b2 = sgn * math.sqrt(r * r - a2 * a2)
        return FLOOR.project((hip[0] + a2 * ux + b2 * vx,
                              hip[1] + a2 * uy + b2 * vy, 0.0))

    def floor_back(self, name, dist=None):
        """后腿指正后：髋正后方 dist 处的地面点（世界 y=髋 y）。dist 缺省取该腿
        爬墙站位半径（压入位吸盘轴 ⊥ 面的解，≈176）——更近会带面内倾角。"""
        hx, hy, _ = self.hip_world(name)
        if dist is None:
            dist = self.r0[name]
        return (hx - float(dist), hy, 0.0)

    def tuck_point(self, name):
        """收腿点（身体系）：髋外 TUCK_RADIUS×站位半径、站位平面上 TUCK_RISE。"""
        leg = self.cfg.leg(name)
        a = d2r(leg.mount_angle_deg)
        r = self.r0[name] * TUCK_RADIUS
        return (leg.mount_x + r * math.cos(a), leg.mount_y + r * math.sin(a),
                self.z0 + TUCK_RISE_MM)

    # ---------- 对外：命令 ----------
    def request_move(self, name, surf, p_w):
        """把 name 挪到面 surf 的世界点 p_w（先抬起悬停在落点前
        lift_clearance 处，land() 才压入吸附）。返回 None=受理；str=拒绝原因。"""
        deny = self._may_command(name)
        if deny:
            return deny
        p_w = surf.project(p_w)
        why = self._check_landing(name, p_w, surf, self.pose)
        if why:
            return f"{name} 落点不可行：{why}"
        plan, why = self._plan_handover(name)
        if why:
            return why
        # 抬离点含卸载量：交接把这条腿的指令先竖直抬 δ，摆动就是从那里起步
        start_w = self._liftoff_world(name, up=plan[0] if plan else 0.0)
        n_a = self.surf[name].n if self.surf[name] else surf.n
        # 悬停点带修正量：与 HOVER 相位的目标（pw − n·(depth+trim)，depth=−clearance）
        # 一致，否则平移到位切 HOVER 瞬间足端会跳 trim 毫米
        end_w = _add(p_w, surf.n, self._clear(surf) - self._trim(name, surf))
        arc, why = self._check_path(name, start_w, end_w, n_a, surf.n)
        if why:
            return f"{name} 路径不可行：{why}"
        self.landing[name] = w2b(p_w, self.pose)[:2]
        self._sw[name] = dict(dst_surf=surf, dst_pw=p_w, n_a=n_a,
                              end_w=end_w, air=False, arc=arc)
        self._begin_swing(name, plan)
        return None

    def request_tuck(self, name):
        """把 name 收到空中（tuck_point），留在那里随身体动。"""
        deny = self._may_command(name)
        if deny:
            return deny
        if self.phase_of[name] == MountPhase.AIR:
            return f"{name} 已在空中"
        pb = self.tuck_point(name)
        why = self._check_air(name, pb, self.pose)
        if why:
            return f"{name} 收腿点不可行：{why}"
        plan, why = self._plan_handover(name)
        if why:
            return why
        start_w = self._liftoff_world(name, up=plan[0] if plan else 0.0)
        n_a = self.surf[name].n if self.surf[name] else (0.0, 0.0, 1.0)
        end_w = b2w(pb, self.pose)
        # 收腿不落面，无需抬弧：LIFT 已离面 lift_clearance，直线过去即可
        # （从墙上收前腿时带弧会把足端往髋上方推，femur 超行程）
        arc, why = self._check_path(name, start_w, end_w, n_a, n_a, arcs=(0.0,))
        if why:
            return f"{name} 路径不可行：{why}"
        self.landing[name] = pb[:2]
        self._sw[name] = dict(dst_surf=None, dst_pb=pb, n_a=n_a,
                              end_w=end_w, air=True, arc=arc)
        self._begin_swing(name, plan)
        return None

    def request_takeover(self, name, mm):
        """把 mm 毫米的载荷**转移到** name 身上（接管 / 反向零力交接）：该腿足端
        指令沿世界竖直向下 mm、其余接触腿各向上 mm/n，六腿偏移和仍为 0 ⇒ 身体
        指令不动，但载荷（= 其余腿攒着的弹性势能）挪到这条腿身上。

        用途（09-09 实机 32mm 不回弹的系统性治法的另一半）：L1 抬上墙吸住时它
        是零预载的新腿，机身的重量还全压在被压弯的地面腿里；把势能就地转给
        L1，地面腿松回来，下一条腿（R1）再抬时储能已经小了、下沉也就小了。
        落地后引擎自动接管 max(欠账, self.takeover[name])，本方法是手动追加。

        ⚠ 墙面腿接的是**剪切**（吸盘实测剪切容量 15N，换前足时本来就顶着上界），
        地面腿松回去太多会从"压住"变成"被往外拔"（只承重腿直接失去摩擦支撑）
        ——所以逐次小量加，盯盘压/电流/机身高度。预检：其余腿指令不许抬到面
        以上、本腿压深不许超 PRESS_DEPTH_MAX、全程 IK/行程/倾角/膝净空。
        返回 None=受理；str=拒绝原因。"""
        deny = self._may_command(name)
        if deny:
            return deny
        if self.surf[name] is None or self.phase_of[name] != MountPhase.STANCE:
            return f"{name} 不在面上（{self.phase_of[name].value}），无处接管"
        mm = float(mm)
        if not 0.0 < mm <= TAKEOVER_MAX_MM:
            return f"接管量 {mm:g} 越界：0~{TAKEOVER_MAX_MM:g}mm"
        why = self._start_takeover(name, mm)
        if why:
            return f"{name} 接管 {mm:g}mm 不可行：{why}"
        return None

    def land(self):
        """悬停腿落下：DESCEND→PRESS→吸附确认。返回 None=受理；str=拒绝。"""
        if self.frozen:
            return "冻结中"
        h = self.hover_leg
        if h is None:
            return "没有悬停中的腿"
        if h in self._pw_goal:
            d = math.dist(self._pw_goal[h], self.pw[h])
            return f"{h} 落点还在挪动（剩 {d:.0f}mm，约 {d / NUDGE_SPEED_MMS:.1f}s）"
        self.phase_of[h] = MountPhase.DESCEND
        return None

    def request_pose(self, dpitch_deg=0.0, dx=0.0, dz=0.0):
        """身体位姿增量：俯仰 dpitch（°，抬头为正）、世界 x 平移 dx（+=贴墙）、
        高度 dz。整段中间位姿逐个预检，任一不可行整段拒绝（不动）。
        返回 None=受理；str=拒绝原因。"""
        if not self.started:
            return "启动序列未完成"
        if self.frozen:
            return "冻结中"
        if self.swing_leg:
            return f"{self.swing_leg} 摆动在途"
        if self.pose_pending:
            return "位姿铺设未完成（空格取消）"
        if self._leaking_contact():
            return "接触足漏气挽救中"
        xb, zb, phi = self.pose
        to = (xb + dx, zb + dz, phi + d2r(dpitch_deg))
        if not (0.0 <= to[2] <= self.pitch_max + _EPS):
            return f"俯仰 {r2d(to[2]):.1f}° 出 0~{r2d(self.pitch_max):.0f}° 范围"
        n = max(abs(dpitch_deg) / POSE_SAMPLE_DEG, abs(dx) / POSE_SAMPLE_MM,
                abs(dz) / POSE_SAMPLE_MM)
        n = max(1, int(math.ceil(n)))
        for k in range(1, n + 1):
            s = k / n
            pose = tuple(a + (b - a) * s for a, b in zip(self.pose, to))
            why = self._check_pose(pose)
            if why:
                return (f"位姿不可行（俯仰 {r2d(pose[2]):.1f}° 高 {pose[1]:.0f} "
                        f"前髋距墙 {self.front_hip_to_wall(pose):.0f}）：{why}")
        self._pose_from, self._pose_to = self.pose, to
        self._pose_s = 0.0
        self._pose_T = max(abs(dpitch_deg) / PITCH_RATE_DPS,
                           abs(dx) / LIN_RATE_MMS, abs(dz) / LIN_RATE_MMS, 0.05)
        return None

    def cancel_pose(self):
        """停在当前位姿（每个中间位姿都预检过，停哪里都安全）。"""
        self._pose_from = self._pose_to = None

    def clear_freeze(self):
        """人工处理后解冻：挂 FAULT 的腿自动重新压附（加深从上次深度续）。
        在途的交接/接管**不取消**（与 climb 同口径）：这条腿已经过了抬腿决策、
        载荷挪了一半，半途丢掉等于带着不对称的内应力放气；交接本身是慢速可逆
        动作，续铺无害。位姿铺设照旧取消（那是纯人工请求，可弃）。"""
        self.frozen = None
        self._precharge_t = 0.0
        self._pose_from = self._pose_to = None
        for n in LEG_NAMES:
            i = LEG_NAMES.index(n)
            if self.phase_of[n] == MountPhase.WAIT \
                    and self.ctl.state[i] == FootState.FAULT:
                self.retries[n] = 0
                self.ctl.clear_fault(i)
                self.phase_of[n] = MountPhase.RETRY_LIFT

    # ---------- 主循环 ----------
    def update(self, dt):
        self.t += dt
        self.ctl.update(dt)
        if (getattr(self.ctl, "tank_fault", False)
                and not self.ignore_tank_fault and not self.frozen):
            self.frozen = "罐压传感器读数出合理区间（未接/失效），泵已停"
        if self.frozen:
            return self.targets()
        self._leak_watch()
        if self.frozen:
            return self.targets()
        if not self.started:
            self._startup(dt)
            return self.targets()
        if self._pose_to is not None:
            self._pose_s = min(1.0, self._pose_s + dt / self._pose_T)
            s = self._pose_s
            s = s * s * (3.0 - 2.0 * s)
            self.pose = tuple(a + (b - a) * s
                              for a, b in zip(self._pose_from, self._pose_to))
            if self._pose_s >= 1.0:
                self.pose = self._pose_to
                self._pose_from = self._pose_to = None
        self._run_handover(dt)
        self._run_machines(dt)
        self._run_nudges(dt)
        self._refresh_targets()
        return self.targets()

    # ---------- 内部：启动 ----------
    def _tank_ready(self):
        if getattr(self.ctl, "tankless", False):
            if not self._tankless_precharged:
                if self._precharge_t < TANKLESS_PRECHARGE_S:
                    self.ctl.precharge = True
                    return False
                self._tankless_precharged = True
                self.ctl.precharge = False
            return True
        if self.ignore_tank_fault and getattr(self.ctl, "tank_fault", False):
            return True
        tank = getattr(self.ctl, "last_tank_kpa", None)
        return tank is not None and tank <= TANK_READY_KPA

    def _startup(self, dt):
        if not self._tank_ready():
            self._precharge_t += dt
            if self._precharge_t > PRECHARGE_TIMEOUT_S:
                self.frozen = (f"罐压连续 {PRECHARGE_TIMEOUT_S:.0f}s 未建立到 "
                               f"{TANK_READY_KPA:.0f}kPa（泵/气路异常）")
                return
        else:
            self._precharge_t = 0.0
        if self._attach_queue:
            head = self._attach_queue[0]
            if self.phase_of[head] == MountPhase.STANCE:
                self.phase_of[head] = MountPhase.PRESS
        self._run_machines(dt)
        self._refresh_targets()
        if not self._attach_queue and all(
                p == MountPhase.STANCE for p in self.phase_of.values()):
            self.started = True

    def _may_attach(self, name):
        if self.started:
            return True
        return bool(self._attach_queue) and self._attach_queue[0] == name \
            and self._tank_ready()

    # ---------- 内部：判据 ----------
    def _may_command(self, name):
        if not self.started:
            return "启动序列未完成"
        if self.frozen:
            return "冻结中"
        if name not in LEG_NAMES:
            return f"未知腿 {name}"
        if self.pose_pending:
            return "位姿铺设未完成（空格取消或等它铺完）"
        sw = self.swing_leg
        if sw and sw != name:
            return (f"{sw} 在{'交接' if self.phase_of[sw] in HO_PHASES else '摆动'}"
                    f"在途（{self.phase_of[sw].value}）")
        if self.phase_of[name] not in (MountPhase.STANCE, MountPhase.AIR,
                                       MountPhase.HOVER):
            return f"{name} 在 {self.phase_of[name].value}，不可改目标"
        return self._interlock_why(name)

    def _interlock_why(self, name):
        """互锁：其余接触腿全 ATTACHED 且不漏且盘压过抬腿门槛（悬空腿、只承重腿
        豁免）。None=过。放气前还要复检一次（交接铺设期间盘压可能漏软）。"""
        for n in LEG_NAMES:
            if n == name or not self._bearing(n) \
                    or self.phase_of[n] != MountPhase.STANCE:
                continue
            i = LEG_NAMES.index(n)
            if not self.ctl.is_attached(i):
                return f"{n} 未吸附（{self.ctl.state[i].value}）"
            if self.ctl.is_leaking(i):
                return f"{n} 漏气挽救中"
            k = self.ctl.last_kpa[i]
            if self.cfg.lift_gate_kpa < 0.0 and (k is None or k > self.cfg.lift_gate_kpa):
                ks = "无读数" if k is None else f"{k:.0f}kPa"
                return f"{n} 盘压 {ks} 未深于抬腿门槛 {self.cfg.lift_gate_kpa:g}"
        return None

    def _leaking_contact(self):
        return any(self._bearing(n) and self.ctl.is_leaking(LEG_NAMES.index(n))
                   for n in LEG_NAMES)

    def _leak_watch(self):
        for n in LEG_NAMES:
            i = LEG_NAMES.index(n)
            if self._bearing(n) and self.ctl.is_leaking(i) \
                    and self.ctl.leak_time(i) > self.cfg.leak_rescue_s:
                self.frozen = (f"{n} 漏气挽救超 {self.cfg.leak_rescue_s}s"
                               f"（查 {n} 吸盘唇口/支路密封）")

    def _clear_of_surfaces(self, p_w, clear):
        for s in self.surfaces:
            if s.height(p_w) < clear:
                return s
        return None

    def _tilt_lim(self, name, band=False):
        """该腿的倾角容差。吸附腿按吸盘**密封**容差（落点 TILT_BAND_DEG、保持
        HOLD_TILT_DEG）；**只承重腿按 support_tilt**——它只压不吸，盘面对不对正不影响
        传正压力，拿密封容差卡它是错的（SUPPORT_TILT_DEG 的注释里有账）。"""
        if name in self.support_only:
            return self.support_tilt
        return TILT_BAND_DEG if band else HOLD_TILT_DEG

    def _check_contact(self, name, p_w, surf, pose, depth, tol, off=0.0):
        """接触足在 pose 下的可行性；None=可行，否则原因。off=该腿的交接偏移
        （世界竖直 mm，+ 上）——指令点带着它算，吸盘物理位置不变。"""
        q = _add(p_w, surf.n, -depth)
        pb = w2b((q[0], q[1], q[2] + off), pose)
        try:
            sol = self.geom[name].solve(pb, w2b_dir(_scale(surf.n, -1.0), pose))
        except Infeasible as e:
            return str(e)
        kw = b2w(sol["knee_b"], pose)
        hit = self._clear_of_surfaces(kw, KNEE_CLEAR_MM)
        if hit:
            return f"膝撞{_cn(hit)}"
        if sol["tilt"] > tol:
            return f"吸盘倾角 {sol['tilt']:.0f}°（面外 {sol['oop']:.0f}°）超 {tol:g}°"
        return None

    def _check_landing(self, name, p_w, surf, pose):
        leg = self.cfg.leg(name)
        deep = min(leg.press_delta_mm + self.cfg.max_attach_retry * self.cfg.retry_deeper_mm,
                   PRESS_DEPTH_MAX)
        tr = self._trim(name, surf)
        for depth in (0.0, leg.press_delta_mm, deep):
            why = self._check_contact(name, p_w, surf, pose, depth + tr,
                                      self._tilt_lim(name, band=True))
            if why:
                return why
        return None

    def _check_air(self, name, pb, pose):
        try:
            sol = self.geom[name].solve(pb)
        except Infeasible as e:
            return str(e)
        # 足端离面净空：墙面按修正量放宽——指令点比真实到达深 trim（腿在这个姿态
        # 够不到），指令点离模型墙面 15−trim 时物理上离玻璃仍约 15
        p_w = b2w(pb, pose)
        for sf in self.surfaces:
            if sf.height(p_w) + self._trim(name, sf) < FOOT_AIR_CLEAR_MM:
                return f"足端撞{_cn(sf)}"
        hit = self._clear_of_surfaces(b2w(sol["knee_b"], pose), KNEE_CLEAR_MM)
        if hit:
            return f"膝撞{_cn(hit)}"
        return None

    def _check_path(self, name, a_w, b_w, n_a, n_b, arcs=None):
        """摆动路径（当前位姿，世界系弧线）逐点可达且不撞面。抬弧候选按序
        试（默认 满弧→半弧→直线），返回 (可行弧高, None) 或 (None, 原因)。"""
        if arcs is None:
            full = max(0.0, self.cfg.step_height - self.cfg.lift_clearance)
            arcs = (full, full / 2.0, 0.0)
        why = None
        for arc in arcs:
            why = None
            for k in range(PATH_SAMPLES + 1):
                p = self._path_point(a_w, b_w, n_a, n_b, k / PATH_SAMPLES, arc)
                why = self._check_air(name, w2b(p, self.pose), self.pose)
                if why:
                    why = f"第 {k}/{PATH_SAMPLES} 点 {why}（弧高 {arc:g}）"
                    break
            if why is None:
                return arc, None
        return None, why

    def _check_body(self, pose):
        for bx in (BODY_HALF_MM, -BODY_HALF_MM):
            for by in (BODY_HALF_MM, -BODY_HALF_MM):
                for bz in (0.0, -BELLY_MM):
                    hit = self._clear_of_surfaces(b2w((bx, by, bz), pose),
                                                  BODY_CLEAR_MM)
                    if hit:
                        return f"机身/腹面撞{_cn(hit)}"
        return None

    def _check_pose(self, pose):
        why = self._check_body(pose)
        if why:
            return why
        for n in LEG_NAMES:
            if self.surf[n] is not None:
                why = self._check_contact(n, self.pw[n], self.surf[n], pose,
                                          self.depth[n] + self._trim(n, self.surf[n]),
                                          self._tilt_lim(n), self.ho_off[n])
            else:
                why = self._check_air(n, self.air_pb[n], pose)
            if why:
                return f"{n} {why}"
        return None

    # ---------- 内部：零力交接 / 接管 ----------
    # 一维弹簧账（docs/HANDOVER-DESIGN.md §2）：吸住的脚不能滑 ⇒ 改指令=改力、
    # 不改位。身体的实际高度 X = mean(各腿指令隐含的身体位置) − mg/(nk)，所以
    # 只要六腿指令偏移的代数和为 0，身体指令就没动；单独把一条腿的偏移往上还
    # δ，它的力就降到 ~0。放气那一瞬没有储能可释放 = 不再有"下坠-重吸附"棘轮。
    # 本引擎的方向是**世界竖直**（见模块 docstring 的 ⚠）：地面足竖直=法向
    # （= 改压深），墙面足竖直=切向（= 沿墙剪切，climb 的"下坡"）。
    def _pen(self, name, off=None):
        """该腿沿接触面法向的实际指令压入量 mm（含墙面修正与交接偏移）。
        偏移是世界竖直的，投影到法向只取 n_z 分量——墙面足 n_z=0，所以接/还
        载荷完全不改它的压深，只沿墙上下滑。"""
        surf = self.surf[name]
        o = self.ho_off[name] if off is None else off
        return self.depth[name] + self._trim(name, surf) - o * surf.n[2]

    def _ho_check(self, moves, lift=None):
        """交接位移预检：moves = {腿名: 相对当前偏移的世界竖直位移 mm（+上）}，
        按 HO_SAMPLE_MM 逐点过 IK 余量/关节行程/coxa 偏摆/倾角/膝净空，另加
        两条硬界：
          · 接载的腿压入总量 ≤ PRESS_DEPTH_MAX——再深就是命令腿往刚性玻璃里
            顶（舵机堵转），与 wall_trim 叠进压深是同一笔账；
          · 还载的腿指令不许抬到面以上（压入量 ≥0）——吸附腿会从"压住"变成
            "往外拔"（法向剥离），只承重腿直接失去摩擦支撑。被抬腿（lift）
            例外：它正要离面，卸载方向就是抬离方向。
        None=可行，否则原因文本。"""
        for n, d in moves.items():
            if self.surf[n] is None or abs(d) <= _EPS:
                continue
            # 两条硬界都按**相对**判：只拦"把已经越界的量推得更糟"的动作。绝对值
            # 判会把本来就超标的腿永久排除在载荷分摊之外——L1 带 wall_trim 16 时
            # 命令压深 18+16=34 本就超 28，而墙面腿的交接是沿墙切向（n_z=0）、
            # 压深一毫米都不动，绝对值判会拒掉一个根本不改这个量的动作，B 组
            # 一上墙就寸步难行（09-11 仿真复现）
            pen0 = self._pen(n)
            k = max(1, int(math.ceil(abs(d) / HO_SAMPLE_MM)))
            for j in range(1, k + 1):
                off = self.ho_off[n] + d * j / k
                pen = self._pen(n, off)
                if pen > PRESS_DEPTH_MAX + _EPS and pen > pen0 + _EPS:
                    return (f"{n} 压入 {pen:.0f}mm 超上限 {PRESS_DEPTH_MAX:g}"
                            "（再深就是命令腿顶着刚性面堵转）")
                if n != lift and pen < -_EPS and pen < pen0 - _EPS:
                    return (f"{n} 指令已抬到{_cn(self.surf[n])}以上 "
                            f"{-pen:.0f}mm（再卸就成往外拔"
                            + ("/失去摩擦支撑" if n in self.support_only else "")
                            + "）")
                why = self._check_contact(
                    n, self.pw[n], self.surf[n], self.pose,
                    self.depth[n] + self._trim(n, self.surf[n]),
                    self._tilt_lim(n), off)
                if why:
                    return f"{n} {why}"
        return None

    def _plan_handover(self, name):
        """抬 name 之前的零力交接计划。返回 (plan, why)：plan=(δ, {支撑腿: 份额})
        或 None（关着/无处卸）；why 非 None 时调用方**拒绝整条命令**——半截交接
        比不交接更糟（载荷挪了一半就放气）。"""
        delta = self.cfg.leg(name).handover_mm
        if delta <= 0.0 or self.surf[name] is None \
                or self.phase_of[name] != MountPhase.STANCE:
            return None, None          # 关着，或这条腿本来就悬着（无力可卸）
        sup = self._share_legs(name)
        if not sup:
            self.handover_note = (f"{name} 交接跳过：没有别的接触腿接载荷"
                                  "（它是唯一支撑）")
            return None, None
        # 份额按条数均分：这是"各腿刚度相同"的近似。实测逐腿刚度差 4 倍
        # （08-24 三组 A/B 的 δ-弹跳斜率），严格的均分应按刚度加权——没有力
        # 传感器，先均分，A/B 里看机身高度残差再说（climb 的窗序权重在这里
        # 用不上：过渡不是轮转步态，没有"下一个轮到谁"）
        share = {n: delta / len(sup) for n in sup}
        moves = {name: +delta}
        moves.update({n: -v for n, v in share.items()})
        why = self._ho_check(moves, lift=name)
        if why:
            return None, (f"{name} 零力交接不可行：{why}；δ={delta:g}mm 分给 "
                          f"{len(sup)} 条腿——减小 --handover 或先调位姿/压深")
        return (delta, share), None

    def _takeover_moves(self, name, mm):
        sup = self._share_legs(name)
        if not sup:
            return None
        moves = {name: -float(mm)}
        moves.update({n: +float(mm) / len(sup) for n in sup})
        return moves

    def _start_takeover(self, name, mm):
        """接管：name 沿竖直向下 mm、其余接触腿各向上 mm/n。返回 None=已启动。"""
        moves = self._takeover_moves(name, mm)
        if moves is None:
            return "没有别的接触腿可以卸载"
        why = self._ho_check(moves)
        if why:
            return why
        self._begin_ramp(name, MountPhase.TAKEOVER, moves)
        return None

    def _fit_takeover(self, name, want):
        """能塞得下的最大接管量（二分，0.1mm 粒度）：还一部分远好过一点不还。
        地面腿尤其要截——它的接管量直接变成压深，余量只有
        PRESS_DEPTH_MAX − press_delta（默认 28−18 = 10mm，要更大就调小
        --press-delta）；墙面腿的接管是沿墙剪切，不吃压深，能到吸盘倾角为止。"""
        if self._takeover_moves(name, want) is None:
            return 0.0
        if self._ho_check(self._takeover_moves(name, want)) is None:
            return want
        lo, hi = 0.0, want
        while hi - lo > 0.1:
            mid = (lo + hi) / 2.0
            if self._ho_check(self._takeover_moves(name, mid)) is None:
                lo = mid
            else:
                hi = mid
        return lo

    def _begin_ramp(self, name, phase, moves):
        """按真实时间匀速铺（载荷重分配是准静态过程）。全体腿由**同一个进度
        变量**驱动、按各自的总位移成比例走——这样"偏移代数和 = 0"在铺设的每
        一拍都成立（各腿各自限速的话，份额小的腿会先到位，中途身体指令要漂
        一下）。限速看位移最大的那条腿：速率 = handover_rate_mms。
        铺设期间该腿相位=HANDOVER/TAKEOVER：吸盘照常密封吸附，互锁与漏气监护
        对它照常成立，阀绝不动。"""
        self._ho_start = dict(self.ho_off)
        self._ho_move = {n: d for n, d in moves.items() if abs(d) > _EPS}
        self._ho_s = 0.0
        self.ho_span = max((abs(d) for d in self._ho_move.values()), default=0.0)
        for n, d in moves.items():
            self._ho_goal[n] = self.ho_off[n] + d
        self._gate_t = 0.0
        self.phase_of[name] = phase

    def _run_handover(self, dt):
        """交接/接管的铺设步（update() 内，位姿铺设之后、分段状态机之前）。"""
        cur = self.ho_leg
        if cur is None:
            return
        if self._leaking_contact():
            return      # 漏气挽救期暂停：漏着的盘摩擦余量低，不该被推；量不丢
        if self._ho_s < 1.0 and self.ho_span > _EPS:
            s = min(1.0, self._ho_s + self._ho_rate * dt / self.ho_span)
            # 逐 tick 包络复检后才落位：铺设期间压深/位姿可能已变（重试加深、
            # wall_trim），任一腿越界就**整笔停在当前进度**并留痕（停在同一个
            # s 上才保得住"和为 0"），绝不走到冻结——半截卸载=弹跳打折但安全
            # （climb 审核 §10.2 同款口径）
            step = {n: d * (s - self._ho_s) for n, d in self._ho_move.items()}
            why = self._ho_check(step, lift=cur)
            if why:
                self.handover_note = (
                    f"{cur} {'交接' if self.phase_of[cur] == MountPhase.HANDOVER else '接管'}"
                    f"截断：{why}；铺到 {self._ho_s * self.ho_span:.1f}/"
                    f"{self.ho_span:.1f}mm 即收口（量偏大，或支撑腿压深已到底）")
                self._ho_s = 1.0
                self._ho_move = {}
            else:
                self._ho_s = s
                for n, d in self._ho_move.items():
                    self.ho_off[n] = self._ho_start[n] + d * s
        if self._ho_s < 1.0:
            return
        if self.phase_of[cur] == MountPhase.TAKEOVER:
            self.phase_of[cur] = MountPhase.STANCE
            return
        # 卸载铺完，放气前复检门槛/互锁：窗头那次判定距现在已过 δ/速率 秒
        # （漏气暂停/冻结更久），期间支撑盘可能漏到监护盲区——带着软肩膀放气
        # 正是 08-19 事故类。不过则保持密封等泵拽深，超时冻结点名
        why = self._interlock_why(cur)
        if why:
            self._gate_t += dt
            if self._gate_t > self.cfg.lift_gate_timeout_s:
                self.frozen = (f"交接后放气门槛超时：{why}，等 "
                               f"{self.cfg.lift_gate_timeout_s:.0f}s 未恢复，"
                               f"{cur} 拒放气（交接期间支撑盘漏软？）")
            return
        self._gate_t = 0.0
        # 欠账按**实际铺进去**的量记（可能被截断过）：其余腿一共替它接了多少，
        # 它落地吸住后就还多少
        self._ho_debt[cur] = sum(
            max(0.0, self._ho_start[n] - self.ho_off[n])
            for n in LEG_NAMES if n != cur)
        self._release_and_lift(cur)

    def _release_and_lift(self, name):
        """交接完成（或没开交接）后的放气抬离：只承重腿没真空可放，直接抬。"""
        if name in self.support_only:
            self.phase_of[name] = MountPhase.LIFT
        else:
            self.ctl.request_release(LEG_NAMES.index(name))
            self.phase_of[name] = MountPhase.VENT

    def _settle(self, name):
        """落地吸住（或只承重腿压到位）后的收口：先把载荷接到这条新腿身上
        （接管量 = max(交接欠账, 逐腿 takeover)），再回 STANCE。

        为什么落地后一定要还：份额不还，支撑腿的偏移会一次次往下累积——序列
        里 34 个单腿动作，几次就把 PRESS_DEPTH_MAX 的余量（press 18 时只剩
        10mm）吃穿。还回去之后全机偏移回到 0，指令=纯几何。
        为什么值得多还（takeover > 欠账）：刚吸上的腿是零预载的，机身重量还
        压在被压弯的其余腿里；主动往新腿身上转载荷 = 就地把那些腿攒的弹性
        势能卸掉，下一条腿再抬时储能小、下沉也小。L1 上墙后尤其值得——它是
        接下来整条扶梯路线的主力（用户 09-11 指出）。"""
        self._sw.pop(name, None)
        want = max(self._ho_debt.pop(name, 0.0), self.takeover[name])
        if not self.started:
            # 启动逐足压入：六腿还在一条条吸上，没有"新腿 vs 压弯的老腿"这回事
            self.phase_of[name] = MountPhase.STANCE
            return
        take = self._fit_takeover(name, want)
        if take <= _EPS:
            if want > _EPS:
                moves = self._takeover_moves(name, want)
                why = self._ho_check(moves) if moves else "没有别的接触腿可以卸载"
                self.handover_note = (
                    f"{name} 接管 {want:.1f}mm 一点也塞不下（{why}）"
                    "——载荷留在其余腿上，它们还压着弹性变形")
            self.phase_of[name] = MountPhase.STANCE
            return
        if take < want - 0.1:
            self.handover_note = (
                f"{name} 接管截到 {take:.1f}/{want:.1f}mm（余下的塞不下："
                f"{self._ho_check(self._takeover_moves(name, want))}）"
                "——其余腿只松回一部分，压深余量不够就调小 --press-delta")
        self._start_takeover(name, take)

    def ho_text(self):
        """逐腿交接偏移的紧凑文本（全 0 返回 '0'），+ 上（已卸载）/ − 下（接了载荷）。"""
        if not any(abs(v) > _EPS for v in self.ho_off.values()):
            return "0"
        return " ".join(f"{n}{self.ho_off[n]:+.1f}" for n in LEG_NAMES
                        if abs(self.ho_off[n]) > _EPS)

    # ---------- 内部：摆动 ----------
    def _clear(self, surf):
        """该面的抬离高度：墙面用 cfg.lift_clearance（吸盘回弹口径），地面用
        floor_clear（还要盖过抬腿时的自重下垂，否则盘在摆动中蹭地）。"""
        return self.floor_clear if surf is FLOOR else self.cfg.lift_clearance

    def _trim(self, name, surf):
        """该腿在该面的目标修正量（只有墙面有，逐腿）。"""
        return self.wall_trim[name] if surf is self.wall else 0.0

    def _foot_world(self, name):
        """该腿的足端**指令**点（世界系）：面上点 − 法向(压深+修正) + 交接偏移。
        ⚠ 与吸盘的物理位置差一个 ho_off——吸住的脚不能滑，改指令改的是力。"""
        if self.surf[name] is not None:
            p = _add(self.pw[name], self.surf[name].n,
                     -(self.depth[name] + self._trim(name, self.surf[name])))
            return (p[0], p[1], p[2] + self.ho_off[name])
        return b2w(self.air_pb[name], self.pose)

    def _liftoff_world(self, name, up=0.0):
        """摆动路径起点：接触腿 = 面上方 lift_clearance 的抬离点（LIFT 段终点），
        悬停/空中腿 = 当前点。up = 摆动前还要竖直卸载掉的交接量（含已有偏移）。"""
        if self.surf[name] is not None:
            p = _add(self.pw[name], self.surf[name].n,
                     self._clear(self.surf[name]) - self._trim(name, self.surf[name]))
            return (p[0], p[1], p[2] + self.ho_off[name] + up)
        return b2w(self.air_pb[name], self.pose)

    def set_wall_trim(self, mm, legs=None):
        """改墙面目标修正量（legs=None 改全部腿，否则只改给的那些）：改后现有
        墙面接触/悬停腿须仍可行（悬停腿还按落地压深复核，免得 i 落下时压出工作
        空间）。不过则整笔回滚。返回 None=成功；str=拒绝原因。"""
        old = dict(self.wall_trim)
        for n in (LEG_NAMES if legs is None else legs):
            if n not in self.wall_trim:
                return f"未知腿 {n}"
            self.wall_trim[n] = float(mm)
        why = self._check_pose(self.pose)
        if why is None:
            for n in LEG_NAMES:
                if self.surf[n] is self.wall and self.phase_of[n] == MountPhase.HOVER:
                    why = self._check_landing(n, self.pw[n], self.wall, self.pose)
                    if why:
                        why = f"{n} 落地压深下 {why}"
                        break
        if why:
            self.wall_trim = old
            return why
        return None

    def nudge_wall_target(self, name, dz=0.0, dy=0.0):
        """悬停中沿墙面平移该腿落点（dz 上正、dy 左正 mm）：机身在抬腿时被腿链
        弹性压沉（08-19 实测 13~27mm），模型以为还在指令位姿上，按同一世界高度
        放第二只前足就会低一截——没有 IMU 只能悬停时按眼睛纠。新落点须过落点带
        与压深复核，不过则拒绝原地不动；受理后由 update() 按 NUDGE_SPEED_MMS
        匀速铺过去（不步进：盘离面只十几毫米，跳变会让腿冲一下蹭到面）。
        连按累加在上一次的目标上。返回 None=成功；str=拒绝原因。"""
        if self.frozen:
            return "冻结中"
        if self.phase_of[name] != MountPhase.HOVER:
            return f"{name} 不在悬停（只有悬停中才能挪落点）"
        if self.surf[name] is not self.wall:
            return f"{name} 悬停在{_cn(self.surf[name])}，本键只挪墙面落点"
        base = self._pw_goal.get(name, self.pw[name])
        p = self.wall.project((base[0], base[1] + dy, base[2] + dz))
        why = self._check_landing(name, p, self.wall, self.pose)
        if why is None:                      # 悬停点自身也要可达/不撞面
            old = self.pw[name]
            self.pw[name] = p
            why = self._check_air(name, w2b(self._foot_world(name), self.pose),
                                  self.pose)
            self.pw[name] = old
            if why:
                why = f"悬停点 {why}"
        if why:
            return why
        self._pw_goal[name] = p
        return None

    @property
    def nudge_pending(self):
        """仍在铺设的落点挪动（腿名元组）。"""
        return tuple(self._pw_goal)

    def target_height(self, name):
        """该腿落点的离地高度（挪动中取目标值，显示用）。"""
        return self._pw_goal.get(name, self.pw[name])[2]

    def _run_nudges(self, dt):
        """把 pw 沿面匀速铺向目标；腿离开悬停即作废（落地/改去别处）。"""
        for n in list(self._pw_goal):
            if self.phase_of[n] != MountPhase.HOVER or self.surf[n] is not self.wall:
                del self._pw_goal[n]
                continue
            goal, cur = self._pw_goal[n], self.pw[n]
            d = math.dist(goal, cur)
            step = NUDGE_SPEED_MMS * dt
            if d <= step + _EPS:
                self.pw[n] = goal
                del self._pw_goal[n]
            else:
                k = step / d
                self.pw[n] = tuple(c + (g - c) * k for c, g in zip(cur, goal))
            self.landing[n] = w2b(self.pw[n], self.pose)[:2]

    def trim_text(self):
        """逐腿修正量的紧凑文本（全 0 时返回 '0'）。"""
        if not any(self.wall_trim.values()):
            return "0"
        return " ".join(f"{n}{self.wall_trim[n]:+g}" for n in LEG_NAMES
                        if self.wall_trim[n])

    def _path_point(self, a, b, n_a, n_b, s, arc):
        """世界系摆动弧线：直线 smoothstep + 沿两面法向均值的正弦抬弧（弧顶
        抬高 arc，满弧 = step_height − lift_clearance，沿用地面步态摆腿形状）。
        地→墙的弧顶朝斜上远离墙角。"""
        ss = s * s * (3.0 - 2.0 * s)
        p = tuple(x + (y - x) * ss for x, y in zip(a, b))
        if arc <= 0.0:
            return p
        u = _add(n_a, n_b)
        if _dot(u, u) < _EPS:
            u = n_b
        return _add(p, _norm(u), arc * math.sin(math.pi * s))

    def _begin_swing(self, name, plan=None):
        if self.surf[name] is not None and self.phase_of[name] == MountPhase.STANCE:
            if plan:
                # 先零力交接（仍吸附、不放气）：铺完由 _run_handover 接着走
                delta, share = plan
                moves = {name: +delta}
                moves.update({n: -v for n, v in share.items()})
                self._begin_ramp(name, MountPhase.HANDOVER, moves)
                return
            self._release_and_lift(name)
        else:
            # 已在空中/悬停：直接从当前点平移
            self._start_transfer(name)

    def _start_transfer(self, name):
        sw = self._sw[name]
        start = self._foot_world(name)          # 含卸载量：从抬高后的点起步
        sw["start_w"] = start
        d = math.sqrt(sum((a - b) ** 2 for a, b in zip(start, sw["end_w"])))
        sw["T"] = max(self.cfg.transfer_time, d / TRANSFER_SPEED_MMS)
        sw["s"] = 0.0
        # 平移期间足端按世界系轨迹走，与面脱钩
        self.surf[name] = None
        self.air_pb[name] = w2b(start, self.pose)
        # 离面即清掉自己的交接偏移（起点已经把它吃进去了，所以指令不跳）：脚在
        # 空中，"改指令=改力"的前提不成立，偏移没有意义。欠账留着——那是**其余
        # 腿**替它接的载荷，等它落地吸住再还
        self.ho_off[name] = self._ho_goal[name] = 0.0
        self.phase_of[name] = MountPhase.TRANSFER

    def _run_machines(self, dt):
        for n in LEG_NAMES:
            if self.phase_of[n] != self._last_ph[n]:
                self._last_ph[n] = self.phase_of[n]
                self._seg_t[n] = 0.0
            else:
                self._seg_t[n] += dt
            if self.phase_of[n] in SWING_PHASES:
                self._step_swing(n, dt)

    def _step_swing(self, name, dt):
        cfg, leg = self.cfg, self.cfg.leg(name)
        i = LEG_NAMES.index(name)
        ph = self.phase_of[name]
        press_depth = leg.press_delta_mm + self._press_extra[name]
        if ph == MountPhase.VENT:
            if self._seg_t[name] >= cfg.lift_vent_s:
                k = self.ctl.last_kpa[i]
                if k is not None and k >= cfg.lift_release_kpa:
                    self.phase_of[name] = MountPhase.LIFT
                elif self._seg_t[name] > cfg.lift_vent_s + VENT_STALL_S:
                    ks = "无读数" if k is None else f"{k:.0f}kPa"
                    self.frozen = (f"{name} 放气未建立：盘压 {ks} 未回升到 "
                                   f"{cfg.lift_release_kpa:g}kPa，不抬")
        elif ph == MountPhase.LIFT:
            top = -self._clear(self.surf[name])
            self.depth[name] = max(top, self.depth[name] - cfg.lift_speed * dt)
            if self.depth[name] <= top + _EPS:
                if self.ctl.state[i] == FootState.RELEASED:
                    self._start_transfer(name)
                else:
                    lift_t = (-top + press_depth) / cfg.lift_speed
                    if self._seg_t[name] > lift_t + VENT_STALL_S:
                        self.frozen = f"{name} 放气确认超时（排气堵/传感器漂移？）"
        elif ph == MountPhase.TRANSFER:
            sw = self._sw[name]
            sw["s"] = min(1.0, sw["s"] + dt / sw["T"])
            p = self._path_point(sw["start_w"], sw["end_w"], sw["n_a"],
                                 sw["dst_surf"].n if sw["dst_surf"] else sw["n_a"],
                                 sw["s"], sw["arc"])
            self.air_pb[name] = w2b(p, self.pose)
            if sw["s"] >= 1.0:
                self._press_extra[name] = 0.0
                if sw["air"]:
                    self.air_pb[name] = sw["dst_pb"]
                    self.phase_of[name] = MountPhase.AIR
                    self._sw.pop(name, None)
                else:
                    # 悬停：挂到目标面上、深度 = -lift_clearance
                    self.surf[name] = sw["dst_surf"]
                    self.pw[name] = sw["dst_pw"]
                    self.depth[name] = -self._clear(sw["dst_surf"])
                    self.phase_of[name] = MountPhase.HOVER
        elif ph == MountPhase.HOVER:
            pass
        elif ph == MountPhase.DESCEND:
            self.depth[name] = min(0.0, self.depth[name] + cfg.descend_speed * dt)
            if self.depth[name] >= -_EPS:
                self.phase_of[name] = MountPhase.PRESS
        elif ph == MountPhase.PRESS:
            self.depth[name] = min(press_depth, self.depth[name] + cfg.press_speed * dt)
            if self.depth[name] >= press_depth - _EPS:
                if name in self.support_only:             # 只承重：压到位即收口
                    if self._attach_queue and self._attach_queue[0] == name:
                        self._attach_queue.pop(0)
                    self._settle(name)
                elif self._may_attach(name):
                    self.ctl.request_attach(i)
                    self.phase_of[name] = MountPhase.WAIT
        elif ph == MountPhase.RETRY_LIFT:
            top = press_depth - cfg.retry_deeper_mm - cfg.retry_lift_mm
            self.depth[name] = max(top, self.depth[name] - cfg.press_speed * dt)
            if self.depth[name] <= top + _EPS:
                self.phase_of[name] = MountPhase.PRESS
        elif ph == MountPhase.WAIT:
            st = self.ctl.state[i]
            if st == FootState.ATTACHED:
                self.retries[name] = 0
                if self._attach_queue and self._attach_queue[0] == name:
                    self._attach_queue.pop(0)
                self._settle(name)
            elif st == FootState.FAULT:
                self.retries[name] += 1
                if self.retries[name] > cfg.max_attach_retry:
                    self.frozen = (f"{name} 连续 {cfg.max_attach_retry} 次"
                                   "吸附失败，全机冻结")
                else:
                    self.ctl.clear_fault(i)
                    self._press_extra[name] = min(
                        self._press_extra[name] + cfg.retry_deeper_mm,
                        cfg.max_attach_retry * cfg.retry_deeper_mm,
                        max(0.0, PRESS_DEPTH_MAX - leg.press_delta_mm))
                    self.phase_of[name] = MountPhase.RETRY_LIFT

    def _refresh_targets(self):
        for n in LEG_NAMES:
            pb = (w2b(self._foot_world(n), self.pose)      # 含压深/修正/交接偏移
                  if self.surf[n] is not None else self.air_pb[n])
            self.foot[n] = list(pb)


# ---------------------------------------------------------------- 小工具
def _scale(v, k):
    return (v[0] * k, v[1] * k, v[2] * k)


def _cn(surf):
    return {"floor": "地面", "wall": "墙面"}.get(surf.name, surf.name)


def _frange(a, b, step):
    x = a
    while x <= b + _EPS:
        yield x
        x += step

