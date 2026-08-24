"""P4 爬墙步态引擎：相位推进可被吸附事件暂停的事件驱动步态。

P3 的 GaitEngine 是纯开环相位表——假定触地精确发生在相位边界、触地瞬间
垂直速度最大（"拍地"）、一过边界立刻横向拖脚，对"落脚即密封"全是反的
（P4-GUIDE 第 0 步）。本引擎把摆动相拆成分段状态机（§4.1）：

  HANDOVER vent 前零力交接（handover_mm>0 时插在窗头决策与 VENT 之间，
           docs/HANDOVER-DESIGN.md）：08-20 原地踏步量化显示 83% 的下滑
           发生在放气密封破裂一瞬——被抬腿吸附以来攒的弹性势能一步释放，
           慢放气无效（力的释放不随泄压渐进）。吸住的脚不能滑 ⇒ 改指令=
           改力不改位：本腿指令沿"上坡"还 δ（卸载自己），其余支撑腿各沿
           "下坡"接 δ/5（均值不变=身体指令不动），HANDOVER_SPEED_MMS
           匀速铺完才放气进 VENT——密封破裂时无能量可释放。本足全程仍
           吸附贴面，须随支撑场平移（update() 4.7 步驱动交接位移）。
           三道守卫（审核补）：漏气监护覆盖交接腿本身；位移逐 tick 过包络
           预检，越界当场截断剩余量提前放气（绝不推到 WorkspaceError 冻结）；
           铺完放气前复检抬腿门槛/互锁（窗头判定已陈旧 δ/速率 秒）
  VENT     先通气再抬（08-19 实机：边放气边抬时排气建立慢于抬离，腿被残余
           真空"拽起"）：原地保持 lift_vent_s 只开排气；本足仍贴面，须随
           支撑场平移（不随场 = 在墙面系被拖着划）
  LIFT     沿面法向退开 lift_clearance（吸盘回弹 11~13mm 会把脚顶回面上，
           放气已先行 lift_vent_s）；放气未确认（RELEASED）不进 TRANSFER
  TRANSFER 平移到落点上方：沿用 smoothstep + 正弦抬腿形状
  HOVER    单步专用（连续行走不经过）：TRANSFER 到位后悬停在落点上方
           （z_lift）等 step_land() 确认才落地；故意无超时——节奏由人定
  DESCEND  落点正上方竖直减速下探：XY 冻结，descend_speed 慢速——消灭拍地
  PRESS    XY 继续冻结，压入该腿 press_delta_mm（§4.2 预压行程），
           到位即 request_attach
  WAIT     等 ATTACHED（-30kPa 确认窗）；FAULT -> 抬 retry_lift_mm 且比上次
           加深 retry_deeper_mm 重压（原深度重压几何缺口不变必然同败），
           连续 max_attach_retry 次失败 -> 全机冻结报警
  （支撑）  ATTACHED 后目标保持在压入位，随支撑速度场整体反向平移——
           吸住的脚不能滑，支撑目标必须连续积分，速度突变不产生跳变；
           climb_sag_comp_mm > 0 时每次抬腿事件再沿"下坡"方向额外匀速
           平移一份补偿量，顶回上墙的弹性下滑棘轮（update() 4.5 步）

坐标约定：一切在身体系，z0 = -stand_height 是吸附面，面法向 = ±z。
身体平行于面时对地面/斜面/竖墙通用（贴墙姿态由 Hexapod.body_rpy 另调）。

相位钟规则（"事件驱动"的实现）：全局钟 t 正常推进，但
  - 摆动腿在其相位窗结束时还没吸牢 -> 钟停在窗尾等它（支撑腿随之冻结）
  - 抬腿前互锁不满足（其余 5 足没全 ATTACHED，比 adhesion 注释的 >=4 严，
    首爬用严的）-> 钟停在窗头；相邻足不同时释放由一次一腿天然保证
  - 互锁通过但有支撑盘压未深于 lift_gate_kpa -> 钟停在窗头等泵把它拽深
    （ATTACHED 的 -30 只是密封判据，窗序首尾相接、每腿都在前一腿刚过线的
    瞬间开抬，刚吸上的邻腿是软肩膀——08-19 实测 R2 刚过线 R3 就抬，
    R2/R3 抬腿整机下坠），超 lift_gate_timeout_s 冻结报警点名软腿
  - 任一支撑足漏气 -> 钟暂停（挽救窗内），超 leak_rescue_s -> 冻结报警
  - 速度指令为零 -> 该窗不抬腿，钟空转过窗（吸住不动，省吸盘寿命）
  - 静止转起步（速度指令上升沿/单步受理）-> 相位钟快进对齐 step_leg 的
    窗头：首抬腿从轮次指针开始（新鲜启动=窗序首腿 R3，此后按窗序续接，单步
    与连续互通），不取决于按键时刻落在周期哪个位置；有摆动在途不对齐
    （收口后的下一窗天然就是轮次下一腿）
  - 单步模式（request_step）：引擎记录"当前轮到哪条腿"（step_leg，窗序
    slot_order＝R3→L1→R2→L3→R1→L2 轮转，连续行走的抬腿同样推进——
    两种模式共用一份轮次），受理
    即对齐该腿窗头当拍以单步速度抬腿、平移到落点上方 HOVER 悬停（半步）；
    step_land() 确认后才落地吸附，回支撑自动停（后半步）

冻结（frozen）是粘滞报警态：足端目标全部保持、吸附状态机照跑（常闭阀保
真空，吸附态冻结是安全态），人工处理后 clear_freeze() 继续。

双足爬行（gait=CLIMB_DUAL，docs/DUAL-SWING-DESIGN.md）：只把占空 5/6→4/6
（偏移同集），摆动窗变 2 槽、窗口两两重叠——任意时刻恰 2 腿在窗、稳态恒
4 足吸附。上面的相位钟规则逐腿泛化（窗实例 _win：待决停窗头、摆动未收口
停自己窗尾），事件驱动钟自然把抬腿攒成"错峰双摆"（对内窗头差 T/6≈0.6s，
周期=3×单窗时长，理论提速 ~2.5×）；互锁/抬腿门槛只查支撑腿（STANCE，
在途摆动=已按窗序放行的设计常态，豁免）；放气按窗头序排队、错峰
≥VENT_STAGGER_S（两腿同刻密封破裂=双倍储能砸进 4 只支撑，最坏弹跳
×2.5——错峰不能只靠窗头差，δ 逐腿不同时后窗腿会先铺完反超）。单足
（duty 5/6）窗口不重叠，全部泛化逐字退化为原行为。⚠ 双足禁 sag_comp
（/5 分摊按单摆+5 支撑推导，拍板 B）；权重 v1.1 起双足解禁（单足实测
权重有效后的拍板修订）：份额按轮转距离取值+就地归一（_share_now），
档位按口径分家——双足推荐 1,1,1,0,0，单足档勿照搬（DUAL-SWING §3.5/
附录 C 模型）；δ 表须按双足工况整表重标（弹跳读数 ×5/4、稳态储能变，
开权重再降一档），A/B 勿与单足跨口径比较。

启动序列：调用方先让机器人 stand() 到默认站姿，引擎按窗序
（默认 R3→L1→R2→L3→R1→L2；--leg-order 换序时吸附序同步跟随）
逐足"压入->抽气确认->下一腿"（08-19 实机结论：
六足同时压入没有反力座——机身被整体顶起/顶离墙，杯压不实；逐足压时其余
腿还站着当反力座，压紧力=体重/扶持力的分摊。逐足抽气原本也是防六阀同开
抽垮罐。吸附序取窗序＝落地新旧与抬腿轮转从启动起对齐、首抬腿 R3 是驻留
最久的盘），全部 ATTACHED 后相位钟才开始走（started=True）。
"""
import math
from dataclasses import replace
from enum import Enum

from .adhesion import FootState
from .config import RobotConfig, LEG_NAMES
from .gait import Gait, GaitEngine, CLIMB, _smoothstep
from .kinematics import leg_ik

_EPS = 1e-6
TANK_READY_KPA = -40.0      # 启动序列首次抽气前罐压须建立到此值（冷罐上电
                            # 直接抽必然重试穷尽误冻结；-40 给 -30 判据留余量）
TANKLESS_PRECHARGE_S = 3.0  # 无罐模式首次抽气前的盲抽时长：阀全关、歧管容积
                            # 小，抽几秒即空——省得首足 SUCK 独扛整段大气歧管
PRECHARGE_TIMEOUT_S = 30.0  # 罐压建立超时 -> 冻结报警（泵坏/大漏不能静默干等）
VENT_STALL_S = 2.0          # LIFT 抬到位后等放气确认（RELEASED）的额外上限，
                            # 超时冻结报警——排气堵/传感器漂移不能静默停摆
D_SAFE_MARGIN = 3.0         # 支撑目标离 IK 包络的最小预留 mm。原硬编码
                            # COMP_TAIL_MAX=40 即 press 13 口径下按本预留反解的
                            # 平面尾预算（tail 40 时后腿最紧 d≈201.7=204.7−3）；
                            # 现预算随压深由引擎计算（__init__ 的 comp_tail），
                            # press 18 时 ≈36.8——总账必须跟 z 走，不能锚死名义
                            # 深度（--press-delta 实验暴露，审核 #1 的延伸）
PRESS_DEPTH_MAX = 28.0      # press_delta+重试加深的总压入上限 mm（z=-118）：
                            # 实测校验点——满速+满拖尾下最紧 d≈200.4、余 4.3mm；
                            # 更深实测贴死包络（press18+extra15=z-123 时 d=203.7
                            # 只余 1mm）。press 13 时数量封顶（3×5=15）先生效，
                            # 行为与旧口径完全一致；press 18 时深度封顶 10 先到
TILT_BAND_DEG = 12.0        # 落点带：压入位物理吸盘轴偏面法线的许用角。
                            # P1 台架实测容差 ±15°，留余量；CLIMBING-DESIGN §6
                            # 接受的工作带 ≤11.5°。越界步长按半径裁剪（§4.3）
LEAN_SPEED_MMS = 10.0        # 倾身平移速率 mm/s（body_lean 实验）：吸住的
                             # 支撑足不能滑，身体平移=支撑目标连续重解，与
                             # 支撑场同量级的慢速率；按键请求排队后按此匀速
                             # 铺完（update() 4.6 步），不做阶跃
HANDOVER_SPEED_MMS = 10.0    # 零力交接铺设速率 mm/s（update() 4.7 步）：
                             # 吸住的脚改指令=改力，载荷重分配要留准静态
                             # 时间，与 LEAN_SPEED_MMS 同量级；δ=17 时
                             # 交接段 ~1.7s（docs/HANDOVER-DESIGN.md §3.1）
VENT_STAGGER_S = 0.4         # 双摆动窗对内放气最小间隔 s（DUAL-SWING §3.3）：
                             # 两腿同刻密封破裂=双倍储能砸进 4 只支撑盘（最坏
                             # 弹跳 ×2.5），且 ab_quant 逐腿 vent 跳变需要视频
                             # 可分辨（30fps 下 0.4s=12 帧）。后窗腿铺完须等
                             # 前窗腿已放气且距上次放气足此间隔——窗头差 0.6s
                             # 靠不住：δ 逐腿不同时后窗腿会先铺完反超。单足
                             # 队列恒 ≤1 元、放气间隔恒 ≥ 一整窗，行为不变
LAND_STAGGER_S = 0.5         # 对步（'i'）两腿落地错峰间隔 s：悬停把双腿节奏
                             # 对齐了，同刻下探会同刻 SUCK（罐瞬时跌深 ×2）；
                             # 0.5s 让抽气错开（SUCK 实测 ~0.3s）。连续行走的
                             # 落地由窗头差天然错峰，不走本口
_R_BRACKET = (110.0, 210.0)  # 站位半径求解区间 mm（区间内倾角随半径单调增，
                             # 至 210 数值验证过；z=-108 时 210 仍在 IK 可达
                             # 内）。原上限 190 会把 +12° 带的真实半径 ~198
                             # 截在书写边界上——直行步幅 >44 的角腿落点先被
                             # 这个假边界裁短、尾账随之失真（--max-step 实验
                             # 暴露）


TILT_WORK_DEG = 11.5         # 设计工作带（CLIMBING-DESIGN §6 接受 ≤11.5°）：
                             # 大步幅直行的落点唇口角验证线。TILT_BAND_DEG=12
                             # 是落点裁剪硬线，两者故意留 0.5° 台阶


def max_straight_step(cfg, gait=CLIMB):
    """直行（±X）方向的安全步幅上限 mm（--max-step 的验证口径；侧移/转向
    不适用本账，脚本层在 >40 时禁用那些键）。联合约束逐腿双向查：
      1. 落点唇口角 ≤ TILT_WORK_DEG（落点越远吸盘轴越斜，斜着压不密封）；
      2. 支撑尾端距 IK 包络 ≥ D_SAFE_MARGIN，两种不共存的最坏口径都要过：
         名义深度+满额补偿、加深封顶深度+无补偿（加深时补偿被门控），
         含 VENT 随场拖尾；后退方向把补偿也计在加重侧（保守）。
    站高越矮上限越大（90→66、62→79）：z 占用小则同样平面外摆下 d 与唇口
    角都更小。"""
    eng = ClimbEngine(cfg, None, gait)   # 只取几何，不跑 update（ctl 不触碰）
    d_safe = cfg.femur_len + cfg.tibia_len - D_SAFE_MARGIN
    band = math.radians(TILT_WORK_DEG)

    def ok(S):
        half = S / 2.0
        vent_drift = S / (cfg.climb_cycle_time * gait.duty) * cfg.lift_vent_s
        comp = min(cfg.climb_sag_comp_mm,
                   max(0.0, (eng.comp_tail - half) / 5.0))
        for n in LEG_NAMES:
            leg = cfg.leg(n)
            z_press = -cfg.stand_height - leg.press_delta_mm
            z_deep = -cfg.stand_height - min(
                leg.press_delta_mm
                + cfg.max_attach_retry * cfg.retry_deeper_mm, PRESS_DEPTH_MAX)
            x0, y0, _ = eng.default_feet[n]
            for sgn in (1.0, -1.0):
                lx = x0 + sgn * half - leg.mount_x
                ly = y0 - leg.mount_y
                if abs(_press_tilt(cfg, math.hypot(lx, ly), z_press)) \
                        > band + 1e-9:
                    return False
                for z, c in ((z_press, 5.0 * comp), (z_deep, 0.0)):
                    tx = lx - sgn * (S + c + vent_drift)
                    d = math.hypot(math.hypot(tx, ly) - cfg.coxa_len, z)
                    if d > d_safe:
                        return False
        return True

    lo, hi = 20.0, 120.0
    if not ok(lo):
        return lo
    if ok(hi):
        return hi
    for _ in range(40):
        mid = (lo + hi) / 2.0
        if ok(mid):
            lo = mid
        else:
            hi = mid
    return lo


def parse_handover(spec):
    """解析 --handover 参数为 {腿名: δmm}（未给的腿 0）。两种格式：
    "8" = 六腿统一；"L1:17,R3:9" = 逐腿设（可只给部分腿，腿名不分大小写）。
    范围 0~45mm。上限依据 08-24 三组原地标定（δ=0/12/24，逐腿弹跳对 δ 严格
    线性，外推零弹跳需 δ*≈28~42mm/腿）：旧上限 25 的理由"弹跳封顶 ~21 ⇒
    更大没意义"错把放气瞬间的弹跳当储能——弹跳是储能被其余五腿并联接住后
    的读数（≈储能/5~8，随逐腿刚度），δ 要还的是储能本身。45 = 外推上界 +
    余量；工作空间不归本口径管——climb_walk 启动按 δmax ≤ comp_tail−半步幅
    硬拒、引擎 4.7 步逐 tick 包络越界截断兜底（HANDOVER-DESIGN §5/§6）。
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


def parse_handover_weights(spec):
    """解析 --handover-weights 为 5 元权重元组（自动归一化）。份额按窗序
    轮转距离套用：w[0]=最晚才轮到抬的支撑腿（刚抬过的那条），w[4]=下一个
    要抬的。非法格式/负值/零和抛 ValueError（脚本层转 ap.error）。"""
    try:
        w = tuple(float(v) for v in spec.split(","))
    except ValueError:
        raise ValueError(f"--handover-weights 格式错（{spec!r}）：5 个逗号分隔"
                         "数（新→老），如 0.6,0.25,0.1,0.05,0") from None
    if len(w) != 5:
        raise ValueError(f"--handover-weights 需要恰 5 个值（新→老），"
                         f"给了 {len(w)}")
    # 有限性必须显式验（审核）：NaN 参与 < 与 <= 比较全为 False，
    # "nan,1,1,1,1" 会溜过下面两条检查；inf 则在归一化处产出 inf/inf=NaN。
    # NaN 份额下游没有任何守卫接得住——leg_ik 的可达性判断对 NaN 也全 False
    # 不抛 WorkspaceError，joint_deg_to_us 的 min/max 钳位把 NaN 解析成
    # min_us：五条支撑腿 15 路舵机被静默打到 500µs 端点
    if (any(not math.isfinite(v) for v in w) or any(v < 0 for v in w)
            or sum(w) <= 0):
        raise ValueError("--handover-weights 权重须为有限数、非负且和大于 0")
    total = sum(w)
    return tuple(v / total for v in w)


def parse_leg_order(spec):
    """解析 --leg-order 为六腿窗序元组。格式 L1_R1_L2_R2_L3_R3（下划线或
    逗号分隔，腿名不分大小写），必须恰是六腿的一个排列——缺腿/重复/未知名
    都会破坏"5/6 占空+等距相位恰好铺满一圈"的窗口拼接，一律拒绝
    （抛 ValueError，脚本层转 ap.error）。"""
    names = tuple(t.strip().upper()
                  for t in spec.replace(",", "_").split("_") if t.strip())
    if sorted(names) != sorted(LEG_NAMES):
        raise ValueError(
            f"--leg-order 必须是六腿的一个排列（如 {'_'.join(LEG_NAMES)}，"
            f"不缺不重），给了 {spec!r}")
    return names


def gait_with_slot_order(order, gait=CLIMB):
    """按指定窗序重排步态的相位偏移：窗序（抬腿轮转）= 窗头时刻
    (duty−offset) mod 1 升序，是从相位表**导出**的——所以改抬腿顺序的正确
    做法是重排偏移而不是碰 slot_order。取原步态的偏移值集合按窗头排序后
    按新序重新指派：浮点值与原步态完全同集，窗口拼接的边界行为不变；
    slot_order/轮次推进/权重轮转距离/启动吸附序（=窗序）全部自动跟随。
    ⚠ 默认 CLIMB 序（R3→L1→R2→L3→R1→L2 对角交替波浪）是有讲究的：相邻
    抬腿全在对角/交叉位、不同侧三连（08-19 实测同侧连抬扰动集中一侧=
    R2/R3 下坠的结构性根源）——自定义顺序属实验口径，注意同侧/同排连抬
    的代价；换序改变权重与逐腿 δ 的稳态格局，A/B 不得跨序比较。"""
    heads = sorted(LEG_NAMES,
                   key=lambda n: (gait.duty - gait.offsets[n]) % 1.0)
    vals = [gait.offsets[n] for n in heads]
    return replace(gait, name=f"{gait.name}[{'_'.join(order)}]",
                   offsets={n: vals[k] for k, n in enumerate(order)})


def _press_tilt(cfg, r, z_press):
    """(径向 r, 压入深度 z) 姿态下，物理吸盘轴偏离面法线的带符号角（rad）。
    吸盘轴 = a_t + cup_delta（勿拿 a_t 当吸盘轴，LEG-GEOMETRY §2.13 教训）；
    倾角只依赖 (r, z)——coxa 偏摆整体旋转腿平面，不改轴线离垂直的角度。
    cup_tilt_trim_deg（整机实测垂直度修正）在此并入：加正修正后同一半径的
    "轴向角"变大，垂直解/落点带/步幅上限全部自动整体内收。"""
    _, a, th = leg_ik(cfg, r, 0.0, z_press)
    a_t = a + th - math.pi
    return a_t + math.radians(cfg.cup_delta_deg + cfg.cup_tilt_trim_deg) \
        + math.pi / 2


def _solve_reach(cfg, z_press, tilt_rad=0.0):
    """解站位半径：压入位吸盘轴偏法线 = tilt_rad（0 = 严格垂直）。"""
    lo, hi = _R_BRACKET
    if _press_tilt(cfg, lo, z_press) >= tilt_rad:
        return lo
    if _press_tilt(cfg, hi, z_press) <= tilt_rad:
        return hi
    for _ in range(48):
        mid = (lo + hi) / 2.0
        if _press_tilt(cfg, mid, z_press) < tilt_rad:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


class LegPhase(Enum):
    STANCE = "stance"        # 支撑（含启动前的等待）：目标在压入位，随速度场平移
    HANDOVER = "handover"    # vent 前零力交接（handover_mm>0）：仍吸附贴面，
                             # 指令沿上坡还 δ 卸载、支撑腿各接 δ/5，铺完才放气
    VENT = "vent"            # 抬腿前先通气：贴面原地保持 lift_vent_s，随支撑场
    LIFT = "lift"
    TRANSFER = "transfer"
    HOVER = "hover"          # 单步分段：落点上方悬停等 step_land() 确认落地
    DESCEND = "descend"
    PRESS = "press"
    RETRY_LIFT = "retry"
    WAIT = "wait"


class ClimbEngine:
    """吸附联动爬墙步态。每控制周期调用 update(dt, vx, vy, wz) 取足端目标。

    update() 内部会替调用方跑 AdhesionController.update(dt)，外层循环
    只需要 move_feet(engine.update(...))。
    """

    def __init__(self, cfg: RobotConfig, ctl, gait: Gait = CLIMB,
                 ignore_tank_fault: bool = False, air_mode: bool = False):
        self.cfg = cfg
        self.ctl = ctl
        self.gait = gait
        # air_mode = 实机架空联调（P4-GUIDE 4.6.3）：吸盘悬空必然 FAULT，
        # 重试穷尽后不冻结而是当作已吸附继续走、互锁放行——阀/泵/重试路径
        # 全部真跑，只旁路"吸不住就停"。上墙严禁开。
        self.air_mode = air_mode
        self.air_giveups = 0         # 架空模式放弃吸附的次数（联调统计用）
        # 罐压传感器未接时的专用旁路，上墙严禁开
        self.ignore_tank_fault = ignore_tank_fault or air_mode
        self.z0 = -cfg.stand_height
        # 步幅/相位几何复用 GaitEngine（cycle_time/max_step 换成爬墙参数），
        # 避免公式漂移
        self._geom = GaitEngine(replace(cfg, cycle_time=cfg.climb_cycle_time,
                                        max_step=cfg.climb_max_step), gait)
        # 窗序（抬腿轮转顺序）由步态相位表导出：窗头时刻 = (duty-offset) mod 1
        # 升序，CLIMB 默认 R3→L1→R2→L3→R1→L2（对角交替波浪）。轮次推进
        # 与起步对轮次全按它走，不许再拿 LEG_NAMES 的排列当窗序硬编码；
        # 换序的唯一正道 = gait_with_slot_order 重排偏移（--leg-order 入口），
        # 吸附序/轮次/权重轮转距离全自动跟随
        self.slot_order = tuple(sorted(
            LEG_NAMES, key=lambda n: (gait.duty - gait.offsets[n]) % 1.0))
        # 同刻摆动窗数 = (1−duty)×6（窗长/窗头间距）：CLIMB 5/6 → 1（单足，
        # 现行为），CLIMB_DUAL 4/6 → 2（双足：窗口两两重叠，任意时刻恰 2 腿
        # 在窗、稳态恒 4 足吸附——docs/DUAL-SWING-DESIGN.md §2）
        self._n_swing = max(1, round((1.0 - gait.duty) * 6))
        if self._n_swing > 1 and cfg.climb_sag_comp_mm > 0.0:
            # 双足禁 sag_comp（拍板 B，DUAL-SWING-DESIGN §3.5）：/5 分摊
            # 算术按"单摆动+5 支撑"推导，双足下语义不成立。CLI 已 ap.error，
            # 这里再拒一道直设 config 的路径（与权重再验同一教训：只在 CLI
            # 口拦会漏）。权重 v1.1 起双足解禁（份额按轮转距离取值+就地
            # 归一，_share_now；单足实测权重有效后的拍板 B 修订，§3.5）
            raise ValueError("双摆动窗（duty<5/6）不支持 climb_sag_comp_mm"
                             "：/5 分摊账按单摆+5 支撑推导；治坠用零力交接"
                             "（handover_mm）")
        # 爬墙站位 ≠ 地面站位（foot_reach=130 时吸盘轴偏法线 ~19°，超 ±15°
        # 容差，唇口斜着接面吸不住）：逐腿解"压入位吸盘轴 ⊥ 吸附面"的半径
        # （默认参数约 176mm），落点带按 ±TILT_BAND_DEG 换算成半径区间裁剪。
        self.default_feet = {}
        self._r_band = {}
        for leg in cfg.legs:
            z_press = self.z0 - leg.press_delta_mm
            band = math.radians(TILT_BAND_DEG)
            r0 = _solve_reach(cfg, z_press)
            self._r_band[leg.name] = (_solve_reach(cfg, z_press, -band),
                                      _solve_reach(cfg, z_press, band))
            a = math.radians(leg.mount_angle_deg)
            self.default_feet[leg.name] = (leg.mount_x + r0 * math.cos(a),
                                           leg.mount_y + r0 * math.sin(a),
                                           self.z0)
        self._geom.default_feet = dict(self.default_feet)  # 速度场用同一站位
        # 支撑尾平面预算（原 COMP_TAIL_MAX=40 的按压深泛化）：支撑目标距 IK
        # 包络至少留 D_SAFE_MARGIN，安全平面半径 sqrt(d_safe²−z_press²) 减
        # 默认站位平面半径 = 允许的外摆总量（半步幅+5 次补偿共用）。
        # press 13 时 ≈40.2 与原标定 40 咬合
        d_safe = cfg.femur_len + cfg.tibia_len - D_SAFE_MARGIN
        self.comp_tail = min(
            math.sqrt(d_safe ** 2
                      - (self.z0 - cfg.leg(n).press_delta_mm) ** 2)
            - (math.hypot(self.default_feet[n][0] - cfg.leg(n).mount_x,
                          self.default_feet[n][1] - cfg.leg(n).mount_y)
               - cfg.coxa_len)
            for n in LEG_NAMES)
        # 当前指令足端目标（唯一事实源，机内所有分段都直接改它）
        self.foot = {n: list(p) for n, p in self.default_feet.items()}
        # 启动逐足压入：全部先留在 STANCE 等待，启动分支按队列一次放行一腿
        # （六腿同时压没有反力座，机身被整体顶起/顶离面，杯压不实——08-19）
        self.phase_of = {n: LegPhase.STANCE for n in LEG_NAMES}
        self.retries = {n: 0 for n in LEG_NAMES}
        # 重试加深的当前附加压入量 mm（吸附保持段也停在加深后的实际深度；
        # 新落点在 DESCEND 入口清零，从名义深度重新起算）
        self._press_extra = {n: 0.0 for n in LEG_NAMES}
        self.landing = {n: self.default_feet[n][:2] for n in LEG_NAMES}
        self._transfer = {}          # 腿名 -> (进度 s, 起点 xyz)
        self.frozen = None           # 冻结原因；None = 正常
        self.started = False         # 启动全吸附完成，相位钟开始走
        self.t = 0.0                 # 相位钟
        # 启动吸附序 = 窗序（原 LEG_NAMES 的 L1..R3）：①落地新旧从启动起就
        # 与抬腿轮转对齐，权重模型"晚轮到多接"的口径不再有启动首轮的特例；
        # ②首抬腿 R3 从"最后吸上的最新盘"变成"最先吸上、驻留最久的盘"，
        # 首抬肩膀更瓷实；③压入次序左右交替（对角波浪）而不是先左排后右排
        self._attach_queue = list(self.slot_order)
        # 摆动窗实例（"一次一腿"→集合泛化，DUAL-SWING-DESIGN §4）：键=当前
        # 在窗（相位 ≥ duty）的腿，值=本窗一次性状态 {active 摆动已启动,
        # skipped 静止跳过, block 互锁等待时长, gate 门槛等待时长}。单足任意
        # 时刻至多 1 键（原 _slot_* 标量的逐字等价），双足恒 2 键。窗头沿=
        # 键新建（决策在 update 第 2 步），出窗=键删除——摆动未收口的腿因
        # 钟停在其窗尾（第 3 步）不会被中途删除
        self._win = {}
        self._was_going = False      # 上一拍是否有抬腿需求（起步对轮次的沿检测）
        self.gate_wait = None        # (拒抬腿, (未达门槛腿,...))；窗头等待中
                                     # 置位，climb_walk 状态提示用
        self._precharge_t = 0.0      # 启动序列等罐压建立已耗时长
        self._tankless_precharged = False   # 无罐盲抽已完成（一次性）
        self._seg_t = {n: 0.0 for n in LEG_NAMES}   # 当前摆动分段停留时长
        self._last_ph = dict(self.phase_of)
        # 下滑补偿（update() 4.5 步）：_down = 墙面系"下坡"在身体系里的方向，
        # 开机按机器人头朝上贴墙取 -X，随积分航向一起旋转（与支撑足同一旋转，
        # 同一套航向推算——打滑造成的航向漂移两边一致）。每次抬腿事件把剩余
        # 补偿量重置为 climb_sag_comp_mm，在 LIFT+TRANSFER 名义时长内匀速铺完
        self._down = (-1.0, 0.0)
        self._comp_left = 0.0
        self._comp_rate = 0.0
        # 零力交接（update() 4.7 步）：{腿: 剩余铺设量 mm}。双摆动窗下两个
        # 交接可重叠（原"一次一腿保证至多一个在途"不再成立），按腿存；
        # 放行入册、放气出册，铺完未放气（排队/门槛等待）期间值为 0 仍在册
        self._ho_left = {}
        # 放气错峰（双足硬要求，§3.3）：已放行未放气的腿按窗头序排队，队首
        # 且距上次放气 ≥ VENT_STAGGER_S 才许 request_release。间隔按真实时间
        # 计（_rt，相位钟会停不能用）
        self._vent_queue = []
        self._rt = 0.0
        self._last_vent_t = float("-inf")
        # 权重在引擎口再验一次并归一化（审核）：CLI 之外直设 config 的路径
        # 原先既不验也不归一——(1,1,1,1,1) 会让每次抬腿身体指令净下坡 +4δ，
        # 静默破坏只有 parse_handover_weights 才保证的零和不变量；nan/inf
        # 同理只有 CLI 口在拦。归一化后的份额存 _slot_w，4.7 步只认它
        w = cfg.handover_slot_w
        if w:
            if (len(w) != 5 or any(not math.isfinite(v) for v in w)
                    or any(v < 0 for v in w) or sum(w) <= 0):
                raise ValueError(f"handover_slot_w 非法（{w!r}）：须 5 元、"
                                 "有限、非负、和大于 0（空元组=均分）")
            self._slot_w = tuple(v / sum(w) for v in w)
        else:
            self._slot_w = ()
        # {腿: True}：本次交接套权重（放行瞬间由 _weights_ok 定夺，含轮转
        # 口径守卫）；份额本身由 _share_now 每 tick 按当前 STANCE 集合现算
        # ——权重档按前向轮转距离取值+就地归一（单足铺设期支撑恒 5、权重和
        # 恒 1，与 v1.5 定格表逐位一致；双足支撑集随对内先后/前对落地动态
        # 变化，归一自动适配）。不在册=均分
        self._ho_weighted = {}
        self._prev_lift = None   # 最近一次实际抬过的腿（权重轮转口径守卫用）
        # 交接守卫最近一次动作的说明文本（越界截断/偏离轮转退均分），脚本
        # 层沿检测亮给操作者并落黑匣子——引擎自己没有输出口
        self.handover_note = None
        # 起步过渡记账：comp_tail 稳态账假设"支撑从 +半步幅落点起"，只在连续
        # 行走成立——起步/停走再起步的过渡周期里，窗序靠后的腿从默认位/停点
        # 起被拖满步幅+VENT 随场，账外多 ~20mm（实测 press18 满速满额补偿在
        # R3 首摆 VENT 段 d=204.8 出包络）。六腿都摆过一轮才允许装填补偿；
        # 停走清空重新计账。代价：每次起步头一个周期（~20s）无补偿
        self._swung_since_go = set()
        # 单步模式（climb_walk 'i' 键）：step_leg = 当前轮到的腿（按窗序
        # slot_order 轮转；任何模式的抬腿事件都推进它，单步与连续行走轮次
        # 互通）。_step_wait = 已受理未启动的腿集合（双足=对：轮次腿+窗序
        # 继任，一次受理 _n_swing 条），_step_air = 已启动未收口的；
        # step_pending/step_active 是它们的只读 property（脚本层沿用旧名）。
        # 轮次指针每次启动推进 1——对步一按共推进 2 条腿
        self.step_leg = self.slot_order[0]
        self._step_wait = set()
        self._step_air = set()
        self._step_v = (0.0, 0.0, 0.0)
        # 对步落地错峰：step_land 头一只当拍下探，其余排队每 LAND_STAGGER_S
        # 放一只；_hover_order 记进入悬停的先后（落地次序=悬停次序）
        self._hover_order = []
        self._land_queue = []
        self._land_t = 0.0
        # 倾身（body_lean 实验脚本）：请求量入队（_lean_left，带符号，+X=
        # 前进方向），update() 4.6 步按 LEAN_SPEED_MMS 匀速铺完；lean_mm =
        # 已执行的身体累计前移（显示用；重新落脚不清零——身体没有回去）
        self.lean_mm = 0.0
        self._lean_left = 0.0
        # 本周期实际下发的速度指令（步幅限幅缩放后）。状态行/黑匣子 TLM
        # 必须取这个而不是用户原始指令：SPEED=15 在默认参数下每帧被隐性
        # 缩到 ~13.3mm/s，记未缩放值会把 --sag-comp 标定系统性带偏
        # （按日志推算预期位移，把限幅缺口误当下滑去补）
        self.cmd = (0.0, 0.0, 0.0)

    # ---------- 对外 ----------
    def update(self, dt, vx=0.0, vy=0.0, wz=0.0):
        """推进一个控制周期，返回 {腿名: (x,y,z)} 身体系足端目标。"""
        self.cmd = (0.0, 0.0, 0.0)   # 冻结/启动期不下发速度；限幅后回填
        self._rt += dt               # 真实时间（放气错峰间隔按它计，相位钟会停）
        self.ctl.update(dt)
        if (getattr(self.ctl, "tank_fault", False)
                and not self.ignore_tank_fault and not self.frozen):
            self.frozen = "罐压传感器读数出合理区间（未接/失效），泵已停"
        if self.frozen:
            return self.targets()          # 冻结：目标保持，只有吸附机在跑
        leak_pause = self._leak_watch()    # 支撑足漏气：挽救窗内暂停，超时冻结
        if self.frozen:
            return self.targets()
        if not self.started:
            if not self._tank_ready():   # 冷罐上电：先建罐压再抽第一只脚
                # 计"连续"不就绪时长：抽气尝试会周期性放掉罐压再等泵恢复，
                # 累计口径会把正常的反复等待攒成假超时（--air 架空必踩）
                self._precharge_t += dt
                if self._precharge_t > PRECHARGE_TIMEOUT_S:
                    self.frozen = (f"罐压连续 {PRECHARGE_TIMEOUT_S:.0f}s 未建立到 "
                                   f"{TANK_READY_KPA:.0f}kPa（泵/气路异常）")
                    return self.targets()
            else:
                self._precharge_t = 0.0
            # 逐足压入：队首腿从等待（STANCE）放行进 PRESS，其余腿站着当
            # 反力座；上一腿 ATTACHED 出队后下一腿才开始压。压入是纯机械
            # 动作不等罐压（与预抽并行省时），抽气仍由 _may_attach 把关
            if self._attach_queue:
                head = self._attach_queue[0]
                if self.phase_of[head] == LegPhase.STANCE:
                    self.phase_of[head] = LegPhase.PRESS
            self._run_machines(dt)
            if not self._attach_queue and all(
                    p == LegPhase.STANCE for p in self.phase_of.values()):
                self.started = True
            return self.targets()

        vx, vy, wz = self._clamp_speed(vx, vy, wz)

        # 0.9 起步对轮次：整机静止转入"有抬腿需求"（速度指令非零或单步受理）
        # 的上升沿，把相位钟快进对齐 step_leg 的窗头——首抬腿从轮次指针开始
        # （新鲜启动=窗序首腿 R3，此后按窗序续接，单步与连续互通），不再取决
        # 于按键时刻落在周期哪个位置（修前实测静止 0.1~3.4s 后按 w 首抬
        # L2/L3/R1/R2/R3/L1 各不相同）；单步的等窗空转同时消灭（受理即当拍
        # 启动）。快进只改相位变量、瞬时完成、不动任何足端目标，仅在全腿
        # STANCE 且本窗无摆动在途时做；有摆动在途不对齐——收口后的下一窗
        # 天然就是轮次下一腿，顺序不破
        # 漏气挽救中不受理起步（也不消费上升沿：挽救结束时 w 还按着照常对
        # 齐）——挽救窗内对齐会让窗头决策的互锁计时空转误启
        if not leak_pause:
            want_go = (abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(wz) > 1e-6
                       or bool(self._step_wait))
            # "无摆动在途"由全腿 STANCE 蕴含，勿再另查窗实例 active——刚
            # 收口的摆动（腿已回 STANCE）其窗实例在窗尾前仍挂 active，拿它
            # 挡对齐会把 want_go 上升沿白白消费掉：对步收口同拍再受理时
            # 轮次腿的当前窗早被标了跳过，只能等下个周期，而搭档一旦先抬
            # 到悬停就把钟停死=死锁（双足对抬回归实测）
            if want_go and not self._was_going \
                    and all(p == LegPhase.STANCE
                            for p in self.phase_of.values()):
                head = ((self.gait.duty - self.gait.offsets[self.step_leg])
                        % 1.0) * self.cfg.climb_cycle_time
                self.t += (head - self.t) % self.cfg.climb_cycle_time
                if self._phase(self.step_leg, self.t) < self.gait.duty:
                    # 浮点边界把相位落在 duty 线下才补步：无脑 +_EPS 会让
                    # "钟推进量↔航向积分"的严格账差出 1e-8 级（sag 方向
                    # 回归测试当场抓获）
                    self.t += _EPS
                # 重建窗实例：只有轮次腿从头开窗；其余恰在窗内的腿（双足=
                # 上一窗的后半截、单足=浮点边界残留）本窗标跳过——首抬腿
                # 必须从轮次指针开始，半截窗放行等于给它截短的窗
                self._win = {
                    n: {"active": False, "skipped": n != self.step_leg,
                        "block": 0.0, "gate": 0.0}
                    for n in LEG_NAMES
                    if self._phase(n, self.t) >= self.gait.duty}
            self._was_going = want_go

        # 1. 窗口归属：在窗集合（相位 ≥ duty）。新进窗=窗头沿开实例（决策
        #    在第 2 步）；出窗=删除——摆动未收口的腿钟停在其窗尾（第 3 步），
        #    走不到删除。单足恒 ≤1 键=原"当前窗"标量的逐字等价
        for n in list(self._win):
            if self._phase(n, self.t) < self.gait.duty:
                del self._win[n]
        for n in LEG_NAMES:
            if n not in self._win and self._phase(n, self.t) >= self.gait.duty:
                self._win[n] = {"active": False, "skipped": False,
                                "block": 0.0, "gate": 0.0}

        # 1.5 单步速度口径：受理腿在窗待决或在途时场速度=单步速度（request
        #     已限幅），否则强制静止——其余腿的窗照常空转跳过。窗头决策另按
        #     逐腿速度（第 2 步）：只有受理腿吃单步速度，防止双足下窗口重叠
        #     时对外的第三腿蹭着步速被放行（单足窗口不重叠无此情形）。
        #     cmd 镜像取真实下发值
        step_mode = bool(self._step_air or self._step_wait)
        if step_mode:
            armed_waiting = any(
                n in self._win and not self._win[n]["active"]
                and not self._win[n]["skipped"] for n in self._step_wait)
            if self._step_air or armed_waiting:
                vx, vy, wz = self._step_v
            else:
                vx = vy = wz = 0.0
        self.cmd = (vx, vy, wz)
        moving = abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(wz) > 1e-6
        if not moving:
            self._swung_since_go.clear()   # 停走：补偿过渡期重新计账（见装填处）

        # 2. 窗头决策（逐腿；"待决停钟"规则保证同刻至多一腿待决）：跳过 /
        #    启动摆动 / 互锁等待 / 抬腿门槛等待。
        #    漏气挽救期间（leak_pause）绝不放行新的抬腿——漏着的脚不算可靠支撑
        self.gate_wait = None
        for cur, st in self._win.items():
            if st["active"] or st["skipped"]:
                continue
            # 决策速度按腿取：单步模式下受理腿=单步速度（含零速原地抬起）、
            # 其余腿=0（窗被跳过）；连续行走=场速度
            if step_mode:
                dvx, dvy, dwz = (self._step_v if cur in self._step_wait
                                 else (0.0, 0.0, 0.0))
            else:
                dvx, dvy, dwz = vx, vy, wz
            # 原地抬起（request_lift 的零速单步，body_lean 实验）：速度为零
            # 但本窗挂着抬起请求，照走互锁/门槛决策；零速下 _landing_xy
            # 落点=默认站位（倾身后的腿重新落回站位=复位几何）
            lift0 = cur in self._step_wait
            dec_moving = (abs(dvx) > 1e-6 or abs(dvy) > 1e-6
                          or abs(dwz) > 1e-6)
            if not dec_moving and not lift0:
                st["skipped"] = True
            elif leak_pause or not self._interlock_ok(cur):
                st["block"] += dt
                if st["block"] > self.cfg.interlock_timeout_s:
                    bad = [n for n in LEG_NAMES
                           if n != cur and self.phase_of[n] == LegPhase.STANCE
                           and (not self.ctl.is_attached(LEG_NAMES.index(n))
                                or self.ctl.is_leaking(LEG_NAMES.index(n)))]
                    self.frozen = (f"互锁失败：{'/'.join(bad) or '?'} 不可靠，"
                                   f"{cur} 拒抬")
            elif (shallow := self._lift_gate_shallow(cur)):
                # 互锁过了但有支撑盘还软（刚过 -30 密封线）：钟停在窗头等
                # 泵把它拽深再抬——立刻抬会把载荷压上软肩膀（08-19 实测
                # R2 刚过线 R3 就抬，R2/R3 抬腿整机下坠）。等待与互锁分开
                # 计时：这不是故障是常态等待，2s 上限太紧
                self.gate_wait = (cur, tuple(n for n, _ in shallow))
                st["gate"] += dt
                if st["gate"] > self.cfg.lift_gate_timeout_s:
                    txt = "/".join(f"{n}({k:.0f}kPa)" if k is not None
                                   else f"{n}(--)" for n, k in shallow)
                    self.frozen = (
                        f"抬腿门槛超时：{txt} 等 "
                        f"{self.cfg.lift_gate_timeout_s:.0f}s 未深于 "
                        f"{self.cfg.lift_gate_kpa:.0f}kPa，{cur} 拒抬"
                        "（唇口漏/泵弱？）")
            else:
                self.landing[cur] = self._landing_xy(cur, dvx, dvy, dwz)
                # 放行一律先进 HANDOVER（δ=0 在 4.7 步同拍直落 VENT：拍间
                # 不可观测，行为与旧"直接 VENT"一致）——放气错峰的排队等待
                # 需要一个"已放行未放气"的驻留态。交接期间吸盘保持密封吸附
                # （ATTACHED 控制环照跑，互锁照常；漏气监护对交接腿本身也
                # 覆盖——_leak_watch 收 STANCE+HANDOVER，审核修）。是否套
                # 权重在此定夺（_weights_ok，含轮转口径守卫），份额由
                # _share_now 每 tick 现算
                self._ho_left[cur] = self.cfg.leg(cur).handover_mm
                if self._weights_ok(cur):
                    self._ho_weighted[cur] = True
                self.phase_of[cur] = LegPhase.HANDOVER
                self._vent_queue.append(cur)
                # 轮转指针（权重口径守卫用）：δ=0 直放气的抬腿也要记——
                # 它同样推进真实的抬腿轮转
                self._prev_lift = cur
                st["active"] = True
                if cur in self._step_wait:
                    self._step_wait.discard(cur)
                    self._step_air.add(cur)     # 单步：本窗归它，回支撑即停
                # 轮次推进：任何模式的抬腿事件都算数（单步与连续行走互通），
                # 按窗序 slot_order 数而非 LEG_NAMES 排列
                self.step_leg = self.slot_order[
                    (self.slot_order.index(cur) + 1) % 6]
                self._swung_since_go.add(cur)   # 补偿过渡期计账
                if self.cfg.climb_sag_comp_mm > 0.0:
                    # 下滑补偿按事件装填，量按当前步幅动态限额（comp_tail
                    # 总账：支撑尾端 = 半步幅 + 5 次事件累计补偿，不得把后腿
                    # 推出 IK 包络；预算随压深在 __init__ 反解），在
                    # LIFT+TRANSFER 名义时长内匀速铺完（真实时间口径，4.5 步）。
                    # 有腿加深吸附（_press_extra>0）在支撑时本事件**不装填**
                    # （审核发现 #1）：加深腿整个支撑相停在更深 z，同样平面
                    # 外摆下 d=hypot(r,z) 更大，名义深度校验的总账不再成立
                    # ——且账外还有首周期从默认位起拖满步幅（40 而非半步幅
                    # 20）+VENT 段随场 ~4mm，几何缩表实测兜不住（extra=15 时
                    # 2mm/事件仍在 R3 的 VENT 段 d=204.8 出包络）。加深是吸
                    # 不紧的临时抢救态，至多持续到该腿下次摆动落回名义深度，
                    # 停几轮补偿换不出包络（无补偿实测 d≈200.0，余 4.7mm）
                    deep = max((self._press_extra[n] for n in LEG_NAMES
                                if self.phase_of[n] == LegPhase.STANCE),
                               default=0.0)
                    # 起步过渡周期（六腿未都摆过一轮）同样不装填：稳态账的
                    # "+半步幅起点"假设未建立（见 _swung_since_go 注释）
                    warm = len(self._swung_since_go) >= 6
                    allow = 0.0 if (deep > 0.0 or not warm) else \
                        (self.comp_tail - self._worst_stance_travel(
                            dvx, dvy, dwz) / 2.0) / 5.0
                    c_eff = min(self.cfg.climb_sag_comp_mm, max(0.0, allow))
                    self._comp_left = c_eff
                    if c_eff > 0.0:
                        lift_t = (self.cfg.leg(cur).press_delta_mm
                                  + self.cfg.lift_clearance) / self.cfg.lift_speed
                        self._comp_rate = c_eff \
                            / (lift_t + self.cfg.transfer_time)

        # 3. 相位钟推进量：每条在窗腿给一个上界——待决（互锁/门槛）=停在
        #    窗头，摆动未收口=至多推到该腿窗尾（等吸附事件），跳过/已收口=
        #    不设界。单足恒一腿在窗=原语义逐字不变；双足下钟最远推到最早的
        #    未收口窗尾——后续窗头被攒到前一腿吸牢之后，正是"错峰双摆"
        #    节奏的成因（DUAL-SWING-DESIGN 附录 B）
        if leak_pause:
            adv = 0.0
        else:
            adv = dt
            for n, st in self._win.items():
                if st["skipped"] or (st["active"] and
                                     self.phase_of[n] == LegPhase.STANCE):
                    continue                           # 空转过窗/摆动已完成
                if not st["active"]:
                    adv = 0.0                          # 互锁/门槛等待：停窗头
                    break
                p = self._phase(n, self.t)
                adv = min(adv, max(0.0, (1.0 - p)
                                   * self.cfg.climb_cycle_time - _EPS))
        self.t += adv

        # 4. 支撑足随速度场反向平移（只随钟走：钟停 = 全体支撑冻结）
        if adv > 0.0 and moving:
            dyaw = wz * adv
            c, s = math.cos(dyaw), math.sin(dyaw)
            dx, dy = self._down          # 下坡方向随航向转（与支撑足同一旋转）
            self._down = (c * dx + s * dy, -s * dx + c * dy)
            for n in LEG_NAMES:
                # VENT/HANDOVER 足仍贴面（交接中还密封吸着、通气中在排气），
                # 必须跟随支撑场：身体在动，不随场 = 该足在墙面系被拖着划、
                # 蹭移唇口；交接位移（4.7 步）叠加在场平移之上，与 sag_comp
                # 同口径
                if self.phase_of[n] in (LegPhase.STANCE, LegPhase.VENT,
                                        LegPhase.HANDOVER):
                    f = self.foot[n]
                    x, y = f[0] - vx * adv, f[1] - vy * adv
                    f[0], f[1] = c * x + s * y, -s * x + c * y
                    # 保持段深度含重试加深量：吸上多深就停多深，回名义深度
                    # 会把刚吸上的盘拔回去
                    f[2] = self.z0 - self.cfg.leg(n).press_delta_mm \
                        - self._press_extra[n]

        # 4.5 下滑补偿：每次抬腿事件把全部支撑足沿"下坡"方向额外匀速平移
        # climb_sag_comp_mm = 把身体往上坡顶回去，抵消"抬腿瞬间载荷重分配的
        # 弹性下沉被重新吸附锁死"的棘轮（上墙实测：每抬一腿整机被拽下一截，
        # 6 次/周期能吃掉整个步幅=原地踏步）。
        # - 只在摆动足离面段（LIFT/TRANSFER）注入：DESCEND 起摆动足 XY 已
        #   冻结贴面，此时再动支撑系会横拖正在压入的吸盘
        # - 按真实时间匀速走、不随相位钟停（下沉发生在真实时间里）；位置
        #   连续、速度小步阶跃，与速度场同一口径
        # - ⚠ 只补得回弹性让位，且必须 ≤ 实测下沉量：超出部分会真把吸附中
        #   的支撑盘沿面往下坡拖（等于加大支撑相真实位移、越出落点带）。
        #   界面滑移型损失（玻璃脏/盘压浅）补不回来，靠清洁与真空度
        # - 漏气挽救期间（leak_pause）暂停：漏着的盘摩擦余量低，不该被推
        # - 每事件量已按 COMP_TAIL_MAX 动态限额（装填处）：支撑尾端总外摆
        #   有硬上限，速度场后半程需要的行程不会被补偿提前花掉——满速时
        #   有效补偿 4mm/事件，想吃满更大设定值就降速走
        if (not leak_pause and self._comp_left > 0.0
                and any(self.phase_of[n] in (LegPhase.LIFT, LegPhase.TRANSFER)
                        for n in LEG_NAMES)):
            step = min(self._comp_left, self._comp_rate * dt)
            self._comp_left -= step
            for n in LEG_NAMES:
                if self.phase_of[n] == LegPhase.STANCE:
                    self.foot[n][0] += self._down[0] * step
                    self.foot[n][1] += self._down[1] * step

        # 4.6 倾身（request_lean，body_lean 实验）：排队量按真实时间匀速铺完。
        # 只在全腿 STANCE/HOVER 时推进——摆动/落压期间平移支撑系会横拖
        # 离面/压入中的脚（与下滑补偿同禁区），暂停不丢；漏气挽救期同停。
        # 身体系里支撑足 -X = 身体 +X（前进方向）；HOVER 腿不平移（在空中
        # 随身体走，落点已定为站位）
        if (not leak_pause and abs(self._lean_left) > _EPS
                and all(p in (LegPhase.STANCE, LegPhase.HOVER)
                        for p in self.phase_of.values())):
            step = math.copysign(min(abs(self._lean_left),
                                     LEAN_SPEED_MMS * dt), self._lean_left)
            self._lean_left -= step
            self.lean_mm += step
            for n in LEG_NAMES:
                if self.phase_of[n] == LegPhase.STANCE:
                    self.foot[n][0] -= step

        # 4.7 零力交接（docs/HANDOVER-DESIGN.md；08-20 量化：83% 下滑发生在
        # 放气密封破裂瞬间）：被抬腿指令沿"上坡"还 δ（卸载自己），其余支撑
        # 腿各沿"下坡"接 δ/n（接住载荷）——全体指令均值不变 = 身体指令不动，
        # f→0 后放气无能量可释放。按真实时间匀速铺（载荷重分配是准静态
        # 过程，不随相位钟停），漏气挽救期暂停（漏着的盘摩擦余量低，不该
        # 被推，与 4.5/4.6 同禁区）。多摆泛化（DUAL-SWING-DESIGN §4）：全部
        # HANDOVER 腿并行铺设；分摊每 tick 按当前 STANCE 集合现算
        # （_share_now：均分或按轮转距离取权重就地归一——先行腿若用定格表
        # 会把份额塞给已进 HANDOVER 的对内第二腿=账污染；单足下铺设期集合
        # 恒定，现算与 v1.5 定格表逐位一致）。铺完才 request_release 进 VENT，放气过
        # 三关：①窗头序队首（前窗腿先放）；②距上次放气 ≥ VENT_STAGGER_S
        # （两腿同刻密封破裂=双倍储能砸进 4 只支撑，§3.3）；③门槛/互锁复检。
        # ①②是静默短等——前窗腿的铺设速率固定、③有 lift_gate_timeout_s
        # 冻结兜底，等待链有界，无需独立超时
        if not leak_pause:
            for cur in list(self._vent_queue):
                if self.phase_of[cur] != LegPhase.HANDOVER:
                    continue                   # 防御：放气才出队，不应到达
                step = min(self._ho_left[cur], HANDOVER_SPEED_MMS * dt)
                dx, dy = self._down
                sup = [n for n in LEG_NAMES
                       if self.phase_of[n] == LegPhase.STANCE]
                share = self._share_now(cur, sup)
                if step > 0.0 and share:
                    # 越界预检截断（审核）：4.5 步有 comp_tail 限额、4.6 步有
                    # _lean_room 截短，原 4.7 步是唯一没有包络钳制的支撑系推
                    # 手——δ 偏大/支撑系累积漂移时把某腿推出 IK 包络，
                    # WorkspaceError 冻结；而 clear_freeze 保留在途交接（那是
                    # 对的，见其注释），墙上按 f 续铺 ~0.2mm 又在同一帧复冻=
                    # 死循环，交接腿非 STANCE 连 oo 取机都被拒，唯一出路
                    # ESC×2 全放气=坠落。修法：本 tick 位移先算后落，任一腿
                    # 的新目标越出安全包络（同 _lean_room 口径）就当场截断
                    # 剩余交接量、提前放气——部分卸载=弹跳打折但安全，绝不
                    # 走到冻结
                    moves = [(cur, -dx * step, -dy * step)] + [
                        (n, dx * step * w, dy * step * w)
                        for n, w in share.items()]   # cur 反下坡=上坡=卸载
                    clip = next((n for n, mx, my in moves
                                 if not self._foot_xy_ok(
                                     n, self.foot[n][0] + mx,
                                     self.foot[n][1] + my)), None)
                    if clip is not None:
                        self.handover_note = (
                            f"交接越界截断：{clip} 已到工作空间边界，{cur} "
                            f"剩余 {self._ho_left[cur]:.1f}mm 未卸载即提前"
                            "放气——δ 偏大或支撑系累积漂移（反复抬同一腿标 "
                            "δ？），检查 δ 表/步幅")
                        self._ho_left[cur] = 0.0
                    else:
                        self._ho_left[cur] -= step
                        for n, mx, my in moves:
                            self.foot[n][0] += mx
                            self.foot[n][1] += my
                if self._ho_left[cur] > _EPS:
                    continue
                # ①窗头序 + ②错峰间隔：不满足就保持密封驻留，下拍再试
                if self._vent_queue[0] != cur \
                        or self._rt - self._last_vent_t < VENT_STAGGER_S:
                    continue
                # ③放气前门槛/互锁复检（审核）：窗头那次判定距此已过 δ/速率
                # 秒（漏气暂停、冻结+f 更久），期间某支撑盘可能从过线漏到
                # "深于 -20 漏气绊线、浅于门槛"的监护盲区——带着软肩膀放气
                # 正是门槛要防的 08-19 事故类。复检不过：保持密封等泵拽深，
                # 与窗头等待同款计时/超时冻结（计时挂本腿的窗实例 gate；
                # 摆动未收口钟出不了本窗，实例必在）
                shallow = self._lift_gate_shallow(cur)
                bad = () if self.air_mode else tuple(
                    n for n in LEG_NAMES
                    if n != cur and self.phase_of[n] == LegPhase.STANCE
                    and not self.ctl.is_attached(LEG_NAMES.index(n)))
                st = self._win.get(cur)
                if shallow or bad:
                    self.gate_wait = (cur, bad + tuple(n for n, _ in shallow))
                    if st is not None:
                        st["gate"] += dt
                        if st["gate"] > self.cfg.lift_gate_timeout_s:
                            txt = "/".join(list(bad) + [
                                f"{n}({k:.0f}kPa)" if k is not None
                                else f"{n}(--)" for n, k in shallow])
                            self.frozen = (
                                f"交接后放气门槛超时：{txt} 等 "
                                f"{self.cfg.lift_gate_timeout_s:.0f}s 未恢复，"
                                f"{cur} 拒放气（交接期间支撑盘漏软？）")
                else:
                    if st is not None:
                        st["gate"] = 0.0
                    self.ctl.request_release(LEG_NAMES.index(cur))
                    self.phase_of[cur] = LegPhase.VENT
                    self._vent_queue.pop(0)
                    self._ho_left.pop(cur, None)
                    self._ho_weighted.pop(cur, None)
                    self._last_vent_t = self._rt

        # 4.8 对步落地错峰（step_land 排队的后续腿）：头一只已当拍下探，其余
        # 每 LAND_STAGGER_S 放一只——悬停把双腿节奏对齐了，同刻下探会同刻
        # SUCK（罐瞬时跌深 ×2）。冻结时 update 顶部已早退，队列自动挂起
        if self._land_queue:
            self._land_t += dt
            if self._land_t >= LAND_STAGGER_S:
                self._land_t = 0.0
                nxt = self._land_queue.pop(0)
                if self.phase_of[nxt] == LegPhase.HOVER:
                    self.phase_of[nxt] = LegPhase.DESCEND

        # 5. 摆动分段状态机按真实时间推进（钟停时重试/等待照常进行）
        self._run_machines(dt)
        return self.targets()

    def targets(self):
        return {n: tuple(f) for n, f in self.foot.items()}

    def status(self):
        """{腿名: (步态段, 吸附态, 是否漏气)}，联调脚本显示用。"""
        return {n: (self.phase_of[n].value,
                    self.ctl.state[LEG_NAMES.index(n)].value,
                    self.ctl.is_leaking(LEG_NAMES.index(n)))
                for n in LEG_NAMES}

    @property
    def step_pending(self):
        """已受理未启动的单步/原地抬起（对步=对内还有腿没启动）。"""
        return bool(self._step_wait)

    @property
    def step_active(self):
        """摆动在途的单步/原地抬起（对步=对内还有腿没收口）。"""
        return bool(self._step_air)

    def step_group(self):
        """下一次单步/抬起会动哪些腿：从轮次指针起按窗序连排 _n_swing 条
        （单足=1 条，双足=对：轮次腿+窗序继任）。脚本层"轮到 X"提示用。"""
        k0 = self.slot_order.index(self.step_leg)
        return tuple(self.slot_order[(k0 + i) % 6]
                     for i in range(self._n_swing))

    def _arm_step(self):
        """受理单步/原地抬起：把 step_group 整组挂为待启动（各腿在自己的
        窗头被逐一放行——双足下窗头差 T/6，先后启动即错峰双摆）。"""
        self._step_wait = set(self.step_group())

    def request_step(self, vx, vy=0.0, wz=0.0):
        """单步第一段（抬半步）：受理即把相位钟对齐 step_leg（当前轮到的腿）
        的窗头当拍启动，以给定速度抬腿并平移到落点上方悬停（HOVER；落点/
        支撑场平移与连续行走同口径），等 step_land()（第二次 i）确认才落地。
        双摆动窗下一次受理一对（轮次腿+窗序继任，先后悬停、一并落地）。
        返回 None=受理；str=拒绝原因。"""
        if not self.started:
            return "启动序列未完成"
        if self.frozen:
            return "冻结中"
        if self.step_pending or self.step_active:
            return "已有单步在途"
        v = self._clamp_speed(vx, vy, wz)
        if not any(abs(x) > 1e-6 for x in v):
            return "单步速度为零"
        self._step_v = v
        self._arm_step()
        return None

    def cancel_step(self):
        """取消尚未开始的单步；任一腿已进入摆动则整组不可撤（对步是一次
        受理的整体，先行腿在途时撤掉后行腿会留下单摆殿后的混合态；中途撤
        速度也会让落点与支撑场口径不一致），悬停中的也不撤——step_land
        落地收口是唯一出路。返回 True=已取消，False=在途不可撤。"""
        if self._step_air:
            return False
        self._step_wait.clear()
        return True

    def step_hover_legs(self):
        """悬停等落地的腿（按进入悬停先后；对步会有两只）。"""
        return tuple(n for n in self._hover_order
                     if self.phase_of[n] == LegPhase.HOVER)

    def step_hover_leg(self):
        """悬停等落地的首腿名（单步第一段已完成），无则 None。"""
        legs = self.step_hover_legs()
        return legs[0] if legs else None

    def step_land(self):
        """单步第二段（落半步）：悬停中的腿落地（DESCEND→PRESS→吸附确认，
        回支撑自动停并推进轮次提示）。对步：两只都到悬停才受理（半途落一只
        会把对拆成混合态），头一只当拍下探、其余按 LAND_STAGGER_S 错峰跟进
        （update 4.8 步）。返回 None=受理；str=拒绝原因。"""
        if self.frozen:
            return "冻结中"
        hov = self.step_hover_legs()
        if not hov:
            return "没有悬停中的腿"
        if self._step_wait:
            return "对内另一腿尚未抬起（互锁/漏气等待中，等它抬到悬停）"
        if any(self.phase_of[n] != LegPhase.HOVER for n in self._step_air):
            return "对内另一腿尚未悬停（等它到位再一并落地）"
        self.phase_of[hov[0]] = LegPhase.DESCEND
        self._land_queue = list(hov[1:])
        self._land_t = 0.0
        return None

    def select_step_leg(self, name):
        """指定下一个单步/原地抬起用哪条腿（body_lean 的 1~6 键）。轮次指针
        直接改写——本脚本无连续行走，不存在轮次账被打乱的问题。
        返回 None=成功；str=拒绝原因。"""
        if self.step_pending or self.step_active:
            return "已有单步在途"
        if name not in LEG_NAMES:
            return f"未知腿 {name}"
        if self.phase_of[name] != LegPhase.STANCE:
            return f"{name} 不在支撑相"
        self.step_leg = name
        return None

    def request_lift(self, name=None):
        """原地抬起（body_lean 实验）：step_leg（或指定腿）以零速单步抬到
        HOVER 悬停——互锁/抬腿门槛/VENT 先通气全套照走，落点=默认站位
        （倾身后的腿落回站位=该腿几何复位，身体保持已倾量）。双摆动窗下
        一次抬一对（指定腿=对首腿，搭档=窗序继任；双足 δ 三组标定的载体）。
        step_land() 落地。返回 None=受理；str=拒绝原因。"""
        if not self.started:
            return "启动序列未完成"
        if self.frozen:
            return "冻结中"
        if self.step_pending or self.step_active:
            return "已有单步在途"
        if name is not None:
            deny = self.select_step_leg(name)
            if deny:
                return deny
        self._step_v = (0.0, 0.0, 0.0)
        self._arm_step()
        return None

    @property
    def lean_pending(self):
        """尚未铺完的倾身量 mm（带符号，+ = 向前进方向）。"""
        return self._lean_left

    def request_lean(self, dmm):
        """倾身（body_lean 实验）：身体沿 +X（前进方向）平移 dmm（负=向后），
        请求入队后 update() 按 LEAN_SPEED_MMS 匀速铺完（摆动/落压期间暂停
        不丢）。授予量按当前支撑足几何截短：每足平移后距 IK 包络
        ≥ D_SAFE_MARGIN 且平面半径 ≥ 求解区间下限（与支撑尾预算同口径，
        按实际压深逐足实算）。返回 (授予 mm, 拒绝原因|None)。"""
        if not self.started:
            return 0.0, "启动序列未完成"
        if self.frozen:
            return 0.0, "冻结中"
        if any(self.phase_of[n] in (LegPhase.STANCE, LegPhase.HANDOVER)
               and self.ctl.is_leaking(LEG_NAMES.index(n)) for n in LEG_NAMES):
            return 0.0, "支撑足漏气挽救中"
        sign = 1.0 if dmm > 0 else -1.0
        granted = sign * max(0.0, min(abs(dmm), self._lean_room(sign)))
        if abs(granted) < 1e-9:
            return 0.0, "已到工作空间边界"
        self._lean_left += granted
        return granted, None

    def cancel_lean(self):
        """取消尚未铺完的倾身量（已执行部分不回退——身体已经移过去了）。"""
        self._lean_left = 0.0

    def _foot_xy_ok(self, name, x, y):
        """支撑目标 (x,y) 在该腿当前压深下是否仍在安全包络内：外界=IK 包络
        留 D_SAFE_MARGIN（同 comp_tail 口径，z 取该足当前指令深度），内界=
        站位半径求解区间下限（更近的半径没验证过唇口/IK 账）。倾身截短
        （_lean_room）与交接越界截断（4.7 步）共用，口径不许漂移。"""
        leg = self.cfg.leg(name)
        r = math.hypot(x - leg.mount_x, y - leg.mount_y)
        d_safe = self.cfg.femur_len + self.cfg.tibia_len - D_SAFE_MARGIN
        return r >= _R_BRACKET[0] and \
            math.hypot(r - self.cfg.coxa_len, self.foot[name][2]) <= d_safe

    def _lean_room(self, sign):
        """当前支撑足在 sign 方向（+1=身体向前）还能再倾多少 mm。从"当前
        足位＋已排队量"起算，1mm 步进扫描到某足越界为止（判据 _foot_xy_ok）。
        在途交接也要预扣（审核）：交接 Z 段里授予的倾身要等 4.6 步恢复推进
        才执行，届时支撑腿可能已再吃进最多 份额×剩余交接量 的下坡位移——
        原实现只扣 _lean_left 不扣 _ho_left，Z 段授予的倾身执行时会超包络。
        方向上保守：不区分交接下坡与倾身方向的夹角，按最大份额整量预扣。"""
        room = 80.0
        for n in LEG_NAMES:
            if self.phase_of[n] != LegPhase.STANCE:
                continue
            x = self.foot[n][0] - self._lean_left   # 排队量先记账
            y = self.foot[n][1]
            ok = 0.0
            while ok < room:
                if not self._foot_xy_ok(n, x - sign * (ok + 1.0), y):
                    break
                ok += 1.0
            room = min(room, ok)
        if self._ho_left:
            # 每个在途交接按"剩余量 × 当前最大份额"预扣（_share_now 现算：
            # 均分=1/支撑数——单足 0.2 与旧常数一致、双足 0.25；权重档取
            # 归一后的最大份额，单足恒 5 支撑时=max(w) 与旧口径一致）
            sup = [n for n in LEG_NAMES
                   if self.phase_of[n] == LegPhase.STANCE]
            for cur, left in self._ho_left.items():
                tbl = self._share_now(cur, sup)
                room -= left * (max(tbl.values()) if tbl else 0.0)
        return max(0.0, room)

    def clear_freeze(self):
        """人工处理后解除冻结；重试/等待计时全部清零，挂着 FAULT 的腿自动
        重新压附（罐压计时不清会导致解冻后一帧不就绪立刻复冻）。
        _press_extra 故意不清：冻结前试过的浅深度重来一遍没有意义，解冻
        重压从加深后的深度继续（增量处有封顶，不会越加越深出包络）。"""
        self.frozen = None
        for st in self._win.values():
            st["block"] = 0.0
            st["gate"] = 0.0         # 门槛超时冻结后重看一个完整等待窗
        self._precharge_t = 0.0
        # 挂起未开始的单步一并取消（审核发现 #2）：冻结处理时手在机器旁，
        # 不取消的话解冻后等到窗它会自行抬腿——与"带着冻结前旧速度恢复
        # 行走"同性质的残留。摆动中的 _step_air 保留，让该腿走完收口
        # （悬停中的腿原地保持不自行落地，解冻后按 i 落地收口——无残留
        # 动作）。对步的未启动搭档同属"未开始"一并取消：已启动的先行腿
        # 单独走完，解冻后单腿收口（操作者可控，不留自动动作）
        self._step_wait.clear()
        # 未铺完的倾身量同理取消（已执行部分不回退）：解冻后自行续倾与
        # "带旧速度恢复行走"同性质
        self._lean_left = 0.0
        for n in LEG_NAMES:
            self.retries[n] = 0

    # ---------- 内部 ----------
    def _phase(self, name, t):
        return self._geom.phase(name, t)   # _geom 的 cycle_time 已换成爬墙周期

    def _tank_ready(self):
        """罐压已建立（或传感器失效被旁路时视为就绪，air 模式用）。
        无罐模式改为定时盲抽：读不到歧管压力（足压传感器在阀的吸盘侧、
        阀关时看不见歧管），先请求泵抽 TANKLESS_PRECHARGE_S 再放行首足。"""
        if getattr(self.ctl, "tankless", False):
            if not self._tankless_precharged:
                if self._precharge_t < TANKLESS_PRECHARGE_S:
                    self.ctl.precharge = True
                    return False
                self._tankless_precharged = True
                self.ctl.precharge = False
            return True
        return self.ctl.tank_fault or \
            self.ctl.io.read_tank_kpa() <= TANK_READY_KPA

    def _worst_stance_travel(self, vx, vy, wz):
        """满支撑相时长内单腿最大位移 mm：刚体速度场 v+w×r 在六个默认站位点
        的最大模 × 支撑时长。限速与下滑补偿限额共用，公式不许漂移。"""
        T_st = self.cfg.climb_cycle_time * self.gait.duty
        worst = 0.0
        for n in LEG_NAMES:
            x0, y0, _ = self.default_feet[n]
            worst = max(worst, math.hypot(vx - wz * y0, vy + wz * x0))
        return worst * T_st

    def _clamp_speed(self, vx, vy, wz):
        """把速度指令整体缩放到爬墙步幅上限内。地面步态只截落点、超速部分靠
        支撑足打滑消化；爬墙吸住的脚不能滑，**支撑相真实位移 = 速度×支撑
        时长**必须 ≤ climb_max_step，否则支撑足会被拖出工作空间/落点带。"""
        worst = self._worst_stance_travel(vx, vy, wz)
        if worst <= self.cfg.climb_max_step:
            return vx, vy, wz
        s = self.cfg.climb_max_step / worst
        return vx * s, vy * s, wz * s

    def _interlock_ok(self, name):
        """抬 name 前，其余**支撑腿**（STANCE）必须全部 ATTACHED 且没在漏气。
        非 STANCE 的腿=已按窗序放行的在途摆动（双摆动窗下窗口重叠，对内先行
        腿正在空中是设计常态），豁免——单足下窗口不重叠，决策时其余腿必然
        全 STANCE，与原"其余 5 足全查"逐字等价。"""
        if self.air_mode:
            return True
        for n in LEG_NAMES:
            if n == name or self.phase_of[n] != LegPhase.STANCE:
                continue
            i = LEG_NAMES.index(n)
            if not self.ctl.is_attached(i) or self.ctl.is_leaking(i):
                return False
        return True

    def _lift_gate_shallow(self, name):
        """抬腿门槛未达标的支撑腿 [(腿, kPa)]：其余**支撑腿**（STANCE）盘压
        须全部深于 lift_gate_kpa 才许抬 name——非 STANCE 的在途摆动腿不在
        门槛口径内（与 _interlock_ok 同一豁免；单足下其余腿必然全 STANCE，
        行为不变）。读数取 last_kpa 镜像（ATTACHED 控制环每周期都在读，
        不陈旧、零额外 IO）；镜像缺失按未达标计（保守）。
        air 模式吸不上属预期，旁路；lift_gate_kpa=0 关闭本门槛。"""
        if self.air_mode or self.cfg.lift_gate_kpa >= 0.0:
            return []
        out = []
        for n in LEG_NAMES:
            if n == name or self.phase_of[n] != LegPhase.STANCE:
                continue
            k = self.ctl.last_kpa[LEG_NAMES.index(n)]
            if k is None or k > self.cfg.lift_gate_kpa:
                out.append((n, k))
        return out

    def _weights_ok(self, cur):
        """本次交接是否套权重（放行瞬间定夺，_ho_weighted 入册；份额由
        _share_now 每 tick 现算）。
        权重口径=按窗序轮转距离：w[0]=最晚才轮到抬的支撑腿（轮转上刚抬过的
        那条），w[4]=下一个要抬的。机理（附录 A 模型数值，模型排序即 (j-a)%6
        的轮转距离）：每次落地的零预载锁定把受力格局重摊——早收到的载荷被
        后续落地稀释回集体，晚收到的原封不动攒到该腿自己抬腿，所以"稀释机会
        最多的腿多接"。单足激进档 0.6/0.25/0.1/0.05/0 稳态循环内应力
        29.7→20.2（-32%）；非单调（全给一条收益归零），反向更糟。⚠ 不按
        "落地新旧"排而按窗序距离：从第一抬就正确、不依赖吸附历史、不需要
        落地时间戳（历史教训：v1.5 第一版按落地新旧排，当时启动吸附序还是
        L1..R3 ≠ 窗序，首轮分错腿被用户抓获；启动吸附序后来也改成了窗序，
        但排序独立于吸附序这一点不变）。⚠ 开权重改变稳态运行点，δ 表需
        下调重标。任意归一化权重下位移和=0（均值不变=身体指令不动）照旧
        成立。单足下其余腿有在途摆动=异常工况，退均分（双足下在途是设计
        常态，不查——份额归一由 _share_now 自动适配）。
        轮转口径守卫（审核）：分配数学只在实际按轮转抬腿时成立——偏离轮转
        （body_lean 反复抬同一腿标 δ 等）时套权重会把 w[0] 份额每次砸向同一
        条支撑腿且永不归还（落地复位只清被抬腿自己），激进档 0.6δ/次实测
        4~6 次就把该腿推出工作空间。cur 不是上一抬腿的窗序继任者就本次退回
        均分（帮助文案的 ~10 次预算随之照旧成立），handover_note 留痕。"""
        if not self._slot_w:
            return False
        if self._n_swing == 1 and any(
                self.phase_of[n] != LegPhase.STANCE
                for n in LEG_NAMES if n != cur):
            self.handover_note = (f"{cur} 交接时支撑非 5（有腿在途），"
                                  "权重退回均分——单足权重模型按 5 支撑推导")
            return False
        if (self._prev_lift is None or cur == self.slot_order[
                (self.slot_order.index(self._prev_lift) + 1) % 6]):
            return True
        self.handover_note = (
            f"{cur} 偏离轮转抬腿（上一抬 {self._prev_lift}），本次交接"
            "退回均分——权重分配只在按窗序轮抬时成立")
        return False

    def _share_now(self, cur, sup):
        """当前 tick 的交接分摊表 {腿: 份额}（sup=当前 STANCE 集合）。
        权重档（_ho_weighted 在册）：按前向轮转距离 d=(槽差 mod 6) 取
        w[5−d]、就地归一——单足铺设期支撑恒 5、权重和恒 1，与 v1.5 定格表
        逐位一致；双足支撑集随对内先后/前对落地动态变化，归一自动适配
        （DUAL-SWING-DESIGN §3.5 v1.1）。⚠ 档位按口径分家（附录 C 模型）：
        双足推荐 1,1,1,0,0（=刚落对+次对后腿三等分，最差单腿 −12%/均值
        −21%）；单足激进档照搬双足会把对内后腿推高过均分（32.9 vs 30.8），
        "均值最优"的 0.5,0.5,0,0,0 则把循环漏斗进后腿（单腿 57）——勿跨
        口径搬表。无权重/守卫退回=均分。"""
        if not sup:
            return {}
        if self._ho_weighted.get(cur):
            k0 = self.slot_order.index(cur)
            raw = {n: self._slot_w[5 - ((self.slot_order.index(n) - k0) % 6)]
                   for n in sup}
            tot = sum(raw.values())
            if tot > _EPS:
                return {n: v / tot for n, v in raw.items()}
        return {n: 1.0 / len(sup) for n in sup}

    def _leak_watch(self):
        """支撑足漏气监护（启动序列与行走共用）：挽救窗内返回 True
        （相位钟应暂停），超 leak_rescue_s 置 frozen。交接腿（HANDOVER）
        也在监护内（审核）：它密封承载且正被系统性卸载，唇口中途失效时
        4.7 步继续从一只已不承载的脚"还力"=储能照样瞬释，随后还会对已泄
        的盘放气——原实现只认 STANCE，交接腿恰好脱管最长 δ/速率 秒。"""
        pause = False
        for n in LEG_NAMES:
            i = LEG_NAMES.index(n)
            if self.phase_of[n] in (LegPhase.STANCE, LegPhase.HANDOVER) \
                    and self.ctl.is_leaking(i):
                pause = True
                if self.ctl.leak_time(i) > self.cfg.leak_rescue_s:
                    # 每足支路已装单向阀（歧管—阀罐口间，只许吸盘→歧管），
                    # 盘压互相独立：报谁就是谁，不再有落脚连坐顶包
                    self.frozen = (f"{n} 漏气挽救超 "
                                   f"{self.cfg.leak_rescue_s}s"
                                   f"（查 {n} 吸盘唇口/支路密封）")
        return pause

    def _landing_xy(self, name, vx, vy, wz):
        """本步落点（身体系）：默认位 + 半步幅（速度场与限幅复用 _geom），
        再按落点带做径向裁剪——半径出带 = 压入位唇口倾角超容差，宁可缩步。"""
        ux, uy = self._geom._stride(name, vx, vy, wz)
        x0, y0, _ = self.default_feet[name]
        lx, ly = x0 + ux / 2.0, y0 + uy / 2.0
        leg = self.cfg.leg(name)
        dx, dy = lx - leg.mount_x, ly - leg.mount_y
        r = math.hypot(dx, dy)
        r_lo, r_hi = self._r_band[name]
        r_c = min(max(r, r_lo), r_hi)
        if abs(r_c - r) > _EPS:
            dx, dy = dx * r_c / r, dy * r_c / r
        return (leg.mount_x + dx, leg.mount_y + dy)

    def _may_attach(self, name):
        """启动序列里只放行队首（逐足抽气，防六阀同开抽垮罐），
        且罐压必须已建立（冷罐直接抽会重试穷尽误冻结）。"""
        if self.started:
            return True
        return bool(self._attach_queue) and self._attach_queue[0] == name \
            and self._tank_ready()

    def _run_machines(self, dt):
        for n in LEG_NAMES:
            # 分段停留计时（放气确认超时等 stall 监护用）
            if self.phase_of[n] != self._last_ph[n]:
                self._last_ph[n] = self.phase_of[n]
                self._seg_t[n] = 0.0
            else:
                self._seg_t[n] += dt
            if self.phase_of[n] != LegPhase.STANCE:
                self._step_swing(n, dt)

    def _step_swing(self, name, dt):
        cfg = self.cfg
        i = LEG_NAMES.index(name)
        f = self.foot[name]
        ph = self.phase_of[name]
        # 压入深度含重试加深量（每次 FAULT 重试 +retry_deeper_mm，新落点清零）
        z_press = self.z0 - cfg.leg(name).press_delta_mm - self._press_extra[name]
        z_lift = self.z0 + cfg.lift_clearance

        if ph == LegPhase.HANDOVER:
            # 零力交接：运动由 update() 4.7 步驱动（XY = 卸载铺设 + 随场，
            # z 由第 4 步保持压入深度），铺完在那里放气切 VENT；
            # 本分支只留 _seg_t 照常计时备诊断
            pass
        elif ph == LegPhase.VENT:
            # 先通气：原地保持等排气建立（阀通电在下周期 ctl.update 生效，
            # 时长里已含），到时才抬——边放气边抬会被残余真空拽起（08-19）。
            # XY 由支撑场分支代管（本足贴面随场），这里只管计时切段
            if self._seg_t[name] >= cfg.lift_vent_s:
                self.phase_of[name] = LegPhase.LIFT
        elif ph == LegPhase.LIFT:
            f[2] = min(z_lift, f[2] + cfg.lift_speed * dt)
            if f[2] >= z_lift - _EPS:
                if self.ctl.state[i] == FootState.RELEASED:
                    self._transfer[name] = (0.0, tuple(f))
                    self.phase_of[name] = LegPhase.TRANSFER
                else:
                    # 抬到位还没等到放气确认：排气堵/传感器漂移不能静默停摆
                    # （行程含上一步的加深量——从多深抬起就多算多少预算）
                    lift_t = (cfg.lift_clearance + cfg.leg(name).press_delta_mm
                              + self._press_extra[name]) / cfg.lift_speed
                    if self._seg_t[name] > lift_t + VENT_STALL_S:
                        self.frozen = f"{name} 放气确认超时（排气堵/传感器漂移？）"
        elif ph == LegPhase.TRANSFER:
            s, start = self._transfer[name]
            s = min(1.0, s + dt / cfg.transfer_time)
            self._transfer[name] = (s, start)
            ss = _smoothstep(s)
            lx, ly = self.landing[name]
            f[0] = start[0] + (lx - start[0]) * ss
            f[1] = start[1] + (ly - start[1]) * ss
            arc = max(0.0, cfg.step_height - cfg.lift_clearance)
            f[2] = z_lift + arc * math.sin(math.pi * s)
            if s >= 1.0:
                f[0], f[1], f[2] = lx, ly, z_lift
                self._press_extra[name] = 0.0   # 新落点从名义深度起，重试再加深
                # 单步分段：抬腿半步到此为止，悬停等 step_land()（第二次 i）
                # 才落地；连续行走不停顿直接下探。悬停先后入册（对步落地
                # 次序=悬停次序）
                if name in self._step_air:
                    self._hover_order = [x for x in self._hover_order
                                         if self.phase_of[x] == LegPhase.HOVER]
                    self._hover_order.append(name)
                    self.phase_of[name] = LegPhase.HOVER
                else:
                    self.phase_of[name] = LegPhase.DESCEND
        elif ph == LegPhase.HOVER:
            pass   # 原地悬停等 step_land() 放行；故意无超时——落地节奏由人定
        elif ph == LegPhase.DESCEND:
            f[2] = max(self.z0, f[2] - cfg.descend_speed * dt)
            if f[2] <= self.z0 + _EPS:
                self.phase_of[name] = LegPhase.PRESS
        elif ph == LegPhase.PRESS:
            f[2] = max(z_press, f[2] - cfg.press_speed * dt)
            if f[2] <= z_press + _EPS and self._may_attach(name):
                self.ctl.request_attach(i)
                self.phase_of[name] = LegPhase.WAIT
        elif ph == LegPhase.RETRY_LIFT:
            # z_press 已含本次加深量，+retry_deeper_mm 抵回 = 抬到**上次**
            # 深度上方 retry_lift_mm 处，再压向加深后的新深度。封顶生效时
            # （增量为 0，仅 clear_freeze 后复试可达）会多抬 retry_deeper_mm
            # ——机械无害（top 恒低于 z0），多花 ~0.7s，不为此加分支（审核 #3）
            top = z_press + cfg.retry_lift_mm + cfg.retry_deeper_mm
            f[2] = min(top, f[2] + cfg.press_speed * dt)
            if f[2] >= top - _EPS:
                self.phase_of[name] = LegPhase.PRESS
        elif ph == LegPhase.WAIT:
            st = self.ctl.state[i]
            if st == FootState.ATTACHED:
                self.retries[name] = 0
                if self._attach_queue and self._attach_queue[0] == name:
                    self._attach_queue.pop(0)
                self.phase_of[name] = LegPhase.STANCE
                # 单步收口按腿销账：对步=两只都收口 step_active 才灭，速度
                # 顶替随之停（自动停）
                self._step_air.discard(name)
            elif st == FootState.FAULT:
                self.retries[name] += 1
                if self.retries[name] > cfg.max_attach_retry:
                    if self.air_mode:      # 架空空走：放弃本足，当作吸附成功
                        self.air_giveups += 1
                        self.retries[name] = 0
                        # clear_fault 把阀留在排气位（线圈通电）是**故意的**：
                        # 断电翻回通罐位后，敞口吸盘会经单向阀（吸盘→歧管
                        # 恰是导通方向）持续泄罐压，泵整场连转比线圈发热更糟。
                        # 台架久置的线圈负载由 ESC 退出统一收口，勿"优化"成断电
                        self.ctl.clear_fault(i)
                        if self._attach_queue and self._attach_queue[0] == name:
                            self._attach_queue.pop(0)
                        self.phase_of[name] = LegPhase.STANCE
                        self._step_air.discard(name)   # 架空放弃也算单步完成
                    else:
                        self.frozen = (f"{name} 连续 {cfg.max_attach_retry} 次"
                                       "吸附失败，全机冻结")
                else:
                    self.ctl.clear_fault(i)
                    # 重试加深：原深度重压几何缺口不变必然同败（08-19 墙上
                    # 实测）。双封顶：数量（max_attach_retry×retry_deeper_mm）
                    # 与总深（PRESS_DEPTH_MAX，含 press_delta——压深加大时
                    # 深度封顶先到），冻结-解冻反复重压也不会加深出工作空间
                    self._press_extra[name] = min(
                        self._press_extra[name] + cfg.retry_deeper_mm,
                        cfg.max_attach_retry * cfg.retry_deeper_mm,
                        max(0.0, PRESS_DEPTH_MAX
                            - cfg.leg(name).press_delta_mm))
                    self.phase_of[name] = LegPhase.RETRY_LIFT
