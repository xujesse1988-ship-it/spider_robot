#!/usr/bin/env python3
"""倾身/单腿抬起实验（独立于 climb_walk 的静态姿态脚本）。

用途：验证"抬腿前身体前倾"对吸附/让位/拽动的影响——六足全吸住时把身体沿
前进方向（+X，头指向）平移或后移；可单独抬起任一条腿悬停，悬停中继续倾身，
再落回站位。没有行走能力，行走去 climb_walk。

流程与 climb_walk 同口径：缓慢站起（直达爬墙站位）-> 就位暂停（按 p）->
全吸附启动序列（逐足压入->抽气->下一腿）-> 键盘实验。
  p    就位暂停后开始全吸附启动序列
  ↑/↓  向前/向后倾身一档（--lean-step，默认 5mm）：请求排队后以 10mm/s
       匀速铺完，摆动/落压期间自动暂停不丢；授予量按支撑足对 IK 包络
       实算截短（到边会拒绝并提示）
  t    一键实验序列（HANDOVER-AB-PROTOCOL §4）：①对时标记——+10mm 倾身
       → 铺完停 2s → 回位 → 静置 20s（ab_quant 靠这段大位移锁视频-日志
       对时：身体几乎不动的组活动对比度法会失锁，08-24 三组对时全错即此
       教训）；②自动轮换 --auto-rounds 轮（默认 3）×6 腿——按轮次序逐腿
       （--dual 逐对）抬起-悬停 1s-落下，腿间等泵歇 ≥2s 且距收口 ≥10s
       （协议"等盘压回 -75 泵停"的自动化，上限 30s 超时继续并留痕），
       轮间静置 15s——节奏与协议手动口径一致且逐组可复现，轮换序由引擎
       轮次保证（权重守卫要求的环序轮转不再依赖人手）；③结束静置 30s
       （电流回落观测窗）后打"✓ 实验序列完成"。空格随时取消（对时标记
       回位完成后取消仍有效，剩余轮换可 1~6+i 手动续）；冻结自动中止
       （该组按协议 §6 判定）；--auto-rounds 0 = 只做标记不轮换（反复抬
       同一腿标 δ 的场合用）。手动等价键序见协议 §4
  1~6  选腿（1=L1 2=L2 3=L3 4=R1 5=R2 6=R3），选完按 i 抬它
  i    原地抬起所选腿到悬停（互锁/抬腿门槛/先通气全套照走；--handover
       开启时抬起前自动先做零力交接：被抬腿沿上坡还 δ 卸载、其余支撑腿
       各沿下坡接 δ/5，铺完才放气——治 08-20 实测"83% 下滑在密封破裂
       瞬间"的弹跳棘轮，docs/HANDOVER-DESIGN.md）；悬停中
       再按 i 落地压入+吸附确认。落点=默认站位——倾身后落回站位等于
       该腿几何复位（身体保持已倾量），逐腿轮流可以"蠕动"前移
  空格 取消未铺完的倾身 + 未开始的抬起（悬停中的腿按 i 落地收口）
  f    解除冻结（人工处理完报警后按；未铺完的倾身一并取消）
  o×2  放开全部吸盘但六足保持站立（取机；墙上必须先扶稳——放开即坠；
       I2C 失联冻结下照常可用：盲态纯计时排气、阀由取机序列直驱）
  ESC×2 安全退出（停泵->逐足串行放气->断电，停在爬墙站位）

方向键说明：↑/↓ 是本脚本的倾身键（小幅可逆动作，误触无害），其余转义
序列照旧整包丢弃、裸 ESC 语义与 climb_walk 相同（双击才退出）。

用法:
  python body_lean.py --mock                 # 无硬件干跑
  python body_lean.py --handover L1:17,R1:15,L3:11,R3:9,R2:5,L2:5
                                             # 零力交接 A/B 标定（重跑 08-20
                                             # 原地踏步实验对照下滑量）
  python body_lean.py --dual --handover ...  # 双足对抬（DUAL-SWING-DESIGN §6）：
                                             # i 一次抬一对（所选腿+窗序继任，
                                             # 先后 0.6s 悬停，再按 i 一并落地）
                                             # ——双足 δ 三组标定载体（三组=单足
                                             # 现表×0.5/0.75/1.0，逐腿 vent 跳变
                                             # 线性外推 δ*_pair×0.8；放气错峰
                                             # ≥0.4s 保逐腿视频可分辨）。权重可
                                             # 同开（双足档 1,1,1,0,0，对序按
                                             # 环序轮转否则守卫退均分）
  python body_lean.py --dry                  # 真舵机 + 仿真气路（不碰阀泵）
  python body_lean.py --no-tank              # 无罐：泵直抽歧管（地面/上墙均可）
  python body_lean.py                        # 全链路
  善后（放气+回地面站姿）：python climb_walk.py --release

黑匣子：software/logs/lean_YYYYmmdd_HHMMSS.log（与 climb_walk 同机制）。
安全口径与 climb_walk 完全一致（ESC×2 序列屏蔽 Ctrl-C、IO/工作空间双降落
伞、欠压立即停机）；退出/取机序列是从 climb_walk 复制的同款实现——改那边
记得同步改这边。
"""
import argparse
import os
import select
import signal
import sys
import termios
import time
import tty
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver
from hexapod.adhesion import (AdhesionController, MockVacuumIO, FootState,
                              ATTACH_KPA, PUMP_ON_KPA, PUMP_OFF_KPA)
from hexapod.climb import (ClimbEngine, LegPhase, LEAN_SPEED_MMS,
                           parse_handover, parse_handover_weights,
                           parse_leg_order, gait_with_slot_order)
from hexapod.gait import CLIMB, CLIMB_DUAL
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES
from hexapod.kinematics import WorkspaceError
from hexapod.runlog import RunLog, ClimbWatch

from climb_walk import status_line, coils_off   # 显示/收尾与 climb_walk 同源

STATUS_S = 0.5
LEG_KEYS = {"1": "L1", "2": "L2", "3": "L3", "4": "R1", "5": "R2", "6": "R3"}
# 对时标记（t 键）三个常数 = 协议 §4.3 的手动键序参数：幅值 +10mm（↑×2 档）、
# 铺完停 2s、回位后静置 20s。改这里记得同步协议文档与 ab_quant 的窗口推导
# （lean_fit 的 t0/t1 从日志倾身事件自算，幅值/停留变了拟合仍自洽，但
# <6mm 幅值在 0.27mm/px 口径下逼近 1.2px 方差门槛，拟合会失败）
MARK_LEAN_MM = 10.0
MARK_HOLD_S = 2.0
MARK_REST_S = 20.0
# 自动轮换（t 序列第二段）参数：悬停 1s 即落（视频里分辨摆动/下探段）；
# 腿间间歇 = 泵歇 ≥2s 且距收口 ≥10s（协议 §4"等盘压回 -75 泵停再抬下一条"
# 的自动化，上限 30s 超时继续并留痕——泵一直不歇=漏气，盯状态行）；
# 轮间 15s、结束静置 30s = 协议 §4 手动口径原值
AUTO_HOVER_S = 1.0
AUTO_GAP_MIN_S = 10.0
AUTO_GAP_PUMP_S = 2.0
AUTO_GAP_MAX_S = 30.0
AUTO_ROUND_GAP_S = 15.0
AUTO_TAIL_S = 30.0
LEGS_PER_ROUND = 6


def read_key(timeout):
    """climb_walk.read_key 的方向键变体：↑/↓ 识别为 "UP"/"DOWN"（本脚本的
    倾身键），其余转义序列照旧整包丢弃（防误触），裸 ESC 语义不变（退出
    确认键）。同样必须 os.read 直读 fd（TextIOWrapper 缓冲会吞转义尾巴）。"""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    data = os.read(sys.stdin.fileno(), 8)
    if not data:
        return None
    if data[0:1] == b"\x1b":
        if len(data) > 1:
            if data[:3] == b"\x1b[A":
                return "UP"
            if data[:3] == b"\x1b[B":
                return "DOWN"
            return None
        # 裸 ESC：稍等一眼 fd，慢终端分包送到的转义尾巴按同规则处理
        r2, _, _ = select.select([sys.stdin], [], [], 0.02)
        if r2:
            tail = os.read(sys.stdin.fileno(), 8)
            if tail[:2] == b"[A":
                return "UP"
            if tail[:2] == b"[B":
                return "DOWN"
            return None
    return data[:1].decode("latin-1")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    ap.add_argument("--dry", action="store_true",
                    help="真舵机 + 仿真气路：不碰气路 GPIO/I2C，吸附确认是假的"
                         "——纯排练动作，上墙严禁")
    ap.add_argument("--no-tank", action="store_true",
                    help="无罐：泵直抽歧管；没有储备真空（挽救弱/断电不保"
                         "真空），地面/上墙均已多次实测可用")
    ap.add_argument("--lean-step", type=float, default=5.0,
                    help="每按一次 ↑/↓ 的倾身量 mm（默认 %(default)g，范围 "
                         "1~15）。授予量另按支撑足几何截短")
    ap.add_argument("--auto-rounds", type=int, default=3,
                    help="t 键序列的自动轮换轮数（默认 %(default)d，范围 0~6；"
                         "0=只做对时标记不轮换——反复抬同一腿标 δ 的场合用）。"
                         "每轮 6 腿按轮次序逐腿（--dual 下逐对）抬起-悬停-"
                         "落下，腿间等泵歇、轮间 15s、末尾结束静置 30s")
    ap.add_argument("--press-delta", type=float, default=None,
                    help="预压行程 mm，覆盖全部腿（默认用 config 值 "
                         f"{DEFAULT_CONFIG.legs[0].press_delta_mm:g}）")
    ap.add_argument("--handover", default=None,
                    help="vent 前零力交接 δ mm（默认关）：统一值如 8，或逐腿 "
                         "L1:17,R1:15,L3:11,R3:9,R2:5,L2:5（未给的腿 0）。"
                         "i 抬起前自动先交接（被抬腿沿上坡还 δ 卸载、支撑腿"
                         "各接 δ/5，铺完才放气）。本脚本是 δ 的 A/B 标定"
                         "入口：跑 δ=0/半/全三组，逐腿弹跳对 δ 线性外推零"
                         "弹跳点 δ*，取 δ*×0.8 宁欠勿过（08-24 实测 δ*≈"
                         "28~42/腿；弹跳只是储能被五腿并联接住后的 1/5~1/8，"
                         "别拿弹跳直接当 δ）。范围 0~45。"
                         "注意：反复抬同一条腿时支撑指令每次多漂下坡 δ/5"
                         "（开 --handover-weights 也一样：偏离轮转的抬腿被"
                         "引擎守卫退回均分）、轮抬一圈才互相抵回——单腿连标 "
                         "~10 次后换腿或重启，越界由引擎交接截断兜底（当场"
                         "提前放气并提示，不再推到冻结）")
    ap.add_argument("--handover-weights", nargs="?",
                    const="auto", default=None,
                    help="交接载荷分配按窗序轮转距离加权（5 权重：w1=最晚才"
                         "轮到抬的支撑腿=刚抬过的，w5=下一个要抬的；自动归一化，"
                         "需同时开 --handover）。不带值=按口径取推荐档：单足="
                         "激进档 0.6,0.25,0.1,0.05,0（晚轮到多接——早收到的"
                         "载荷被后续落地的零预载锁定稀释回集体，模型稳态循环内"
                         "应力较均分 -32%%，HANDOVER-DESIGN 附录 A；单足实测"
                         "对下滑改善明显）；--dual 下=1,1,1,0,0（刚落的对+次对"
                         "后腿三等分，份额按距离取值就地归一，模型最差单腿 "
                         "-12%%/均值 -21%%，DUAL-SWING-DESIGN 附录 C）。⚠ 档位"
                         "按口径分家，单足档勿照搬双足。口径假设按'轮到 X'提示"
                         "轮转抬腿；偏离轮转（本脚本反复抬同一腿标 δ 是常态）"
                         "的抬腿由引擎守卫自动退回均分并提示——权重工况标 δ "
                         "要按环序轮抬（对序 L1+R2→L3+R1→L2+R3，协议 §8.1）。"
                         "⚠ 非单调，且改变稳态运行点，δ 表按开权重工况标")
    ap.add_argument("--handover-rate", type=float, default=None,
                    help="交接铺设速率 mm/s（默认 10=n=3 数据的准静态基线；"
                         "须同开 --handover）。D′ 判别实验=20（一次一变量）："
                         "T1 拟合发现交接段 ~九成是'共模下沉≈蠕变率×窗时长'，"
                         "而窗时长≈0.5+δ/rate 与 δ 完全共线——提速后下沉按"
                         "比例降=时间蠕变（预测 C 交接段 39.7→~25mm），不降"
                         "=逐 mm 损耗（模型改写），两个结果都有效（html/"
                         "stiffness-fit-20260826.html §5、协议 §9）。范围 "
                         "0<r≤50（引擎再验）；提速=不那么准静态，判据 4"
                         "（交接段平滑）/漏气挽救/越界截断照常兜底")
    ap.add_argument("--leg-order", default=None,
                    help="抬腿窗序（含启动吸附序）：六腿排列如 "
                         "L1_R1_L2_R2_L3_R3（下划线或逗号分隔，不分大小写，"
                         "不缺不重）。默认=CLIMB 对角波浪 R3_L1_R2_L3_R1_L2。"
                         "'轮到 X'提示与 --handover-weights 轮转距离自动跟随"
                         "新序（守卫仍按新序判偏离轮转）。⚠ 换序改变权重与"
                         "逐腿 δ 的稳态格局，δ 标定按序分组、勿跨序比较")
    ap.add_argument("--dual", action="store_true",
                    help="双足对抬（双摆动窗，docs/DUAL-SWING-DESIGN.md）：i 一次"
                         "抬一对（数字键选对首腿，搭档=窗序继任，窗头差 0.6s "
                         "先后抬到悬停，两腿都悬停后再按 i 一并落地错峰下探）"
                         "——双足 δ 三组标定的载体（三组=单足现表 ×0.5/0.75/"
                         "1.0，ab_quant 逐腿 vent 跳变线性外推 δ*_pair，取 "
                         "×0.8 起标；引擎放气错峰 ≥0.4s 保逐腿可分辨）。"
                         "⚠ 对抬期间恒 4 足吸附；δ 勿与单足口径混标；"
                         "--handover-weights 可同开（双足档 1,1,1,0,0；标定"
                         "对序须按环序轮转，权重才不被守卫退回均分）")
    ap.add_argument("--stand-height", type=float,
                    default=DEFAULT_CONFIG.stand_height,
                    help="站高 mm（默认 %(default)g，范围 55~95）")
    ap.add_argument("--tilt-trim", type=float,
                    default=DEFAULT_CONFIG.cup_tilt_trim_deg,
                    help="吸盘轴垂直度实测修正角°（默认 %(default)g，±8）")
    args = ap.parse_args()
    if not 1.0 <= args.lean_step <= 15.0:
        ap.error(f"--lean-step {args.lean_step} 非法：范围 1~15mm。单档太大"
                 "＝一次排队太多，边界/漏气时不好收")
    if not 55.0 <= args.stand_height <= 95.0:
        ap.error(f"--stand-height {args.stand_height} 非法：范围 55~95mm")
    if not -8.0 <= args.tilt_trim <= 8.0:
        ap.error(f"--tilt-trim {args.tilt_trim} 非法：范围 -8~8°")
    if not 0 <= args.auto_rounds <= 6:
        ap.error(f"--auto-rounds {args.auto_rounds} 非法：范围 0~6（0=只做"
                 "对时标记）")
    handover = None
    if args.handover is not None:
        try:
            handover = parse_handover(args.handover)
        except ValueError as e:
            ap.error(str(e))
    ho_w = None
    if args.handover_weights is not None:
        if args.handover_weights == "auto":
            # 不带值的推荐档按口径分家（DUAL-SWING-DESIGN §3.5/附录 C）：
            # 单足档照搬双足会把对内后腿推高过均分
            args.handover_weights = ("1,1,1,0,0" if args.dual
                                     else "0.6,0.25,0.1,0.05,0")
        if handover is None:
            ap.error("--handover-weights 需要同时开 --handover（权重只作用于"
                     "交接载荷的分配）")
        try:
            ho_w = parse_handover_weights(args.handover_weights)
        except ValueError as e:
            ap.error(str(e))
    if args.handover_rate is not None:
        if handover is None:
            ap.error("--handover-rate 需要同时开 --handover（速率只作用于"
                     "交接铺设）")
        if not 0.0 < args.handover_rate <= 50.0:  # nan/inf 比较为假一并拒
            ap.error(f"--handover-rate {args.handover_rate:g} 非法：范围 "
                     "0<r≤50mm/s（默认 10；D′ 判别实验 20）")
    leg_order = None
    if args.leg_order is not None:
        try:
            leg_order = parse_leg_order(args.leg_order)
        except ValueError as e:
            ap.error(str(e))
    if not sys.stdin.isatty():
        sys.exit("需要交互终端（ssh 加 -t；勿用 nohup/管道跑本脚本）")

    cfg = replace(DEFAULT_CONFIG, stand_height=args.stand_height,
                  cup_tilt_trim_deg=args.tilt_trim)
    if args.press_delta is not None:
        if not 8.0 <= args.press_delta <= 20.0:
            ap.error(f"--press-delta {args.press_delta} 非法：范围 8~20mm")
        cfg = replace(cfg, legs=tuple(
            replace(l, press_delta_mm=args.press_delta) for l in cfg.legs))
    if handover is not None:
        cfg = replace(cfg, legs=tuple(
            replace(l, handover_mm=handover[l.name]) for l in cfg.legs))
    if ho_w is not None:
        cfg = replace(cfg, handover_slot_w=ho_w)
    if args.handover_rate is not None:
        cfg = replace(cfg, handover_rate_mms=args.handover_rate)

    log = RunLog(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs"),
        tag="lean")
    print(f"黑匣子日志: {log.path}")
    mode = "+".join(s for s, on in (("mock", args.mock), ("dry", args.dry),
                                    ("no-tank", args.no_tank)) if on) \
        or "实机全链路"
    log.note(f"模式={mode} port={args.port}")
    ho_txt = ("关" if all(l.handover_mm <= 0.0 for l in cfg.legs) else
              ",".join(f"{l.name}:{l.handover_mm:g}" for l in cfg.legs
                       if l.handover_mm > 0.0))
    log.note(f"参数: lean_step={args.lean_step:g}mm lean_speed={LEAN_SPEED_MMS:g}mm/s"
             f" press_delta={cfg.legs[0].press_delta_mm:g}mm"
             f" stand={cfg.stand_height:g} tilt_trim={cfg.cup_tilt_trim_deg:g}°"
             f" handover={ho_txt} handover_rate={cfg.handover_rate_mms:g}"
             f" dual={int(args.dual)}"
             + (f" leg_order={'_'.join(leg_order)}" if leg_order else ""))
    _prev_hook = sys.excepthook

    def _crash_hook(tp, val, tb):
        log.exc(val)
        log.close("uncaught")
        _prev_hook(tp, val, tb)
    sys.excepthook = _crash_hook

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    if args.mock or args.dry:
        io = MockVacuumIO(6)
    else:
        from hexapod.adhesion import Pi5VacuumIO
        io = Pi5VacuumIO(6)
    ctl_kw = dict(tankless=args.no_tank)
    if args.no_tank:
        ctl_kw["suck_timeout_s"] = 2.5
    ctl = AdhesionController(io, **ctl_kw)
    bot = Hexapod(drv, cfg)
    base = CLIMB_DUAL if args.dual else CLIMB
    gait = gait_with_slot_order(leg_order, base) if leg_order else base
    eng = ClimbEngine(cfg, ctl, gait)
    watch = ClimbWatch(log, eng, ctl, io, cfg)
    log.note(f"阈值: ATTACH={ATTACH_KPA} PUMP_ON={PUMP_ON_KPA}"
             f" PUMP_OFF={PUMP_OFF_KPA} comp_tail={eng.comp_tail:.1f}mm"
             f" suck_timeout={ctl.suck_timeout_s}s")

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    dt = 1.0 / cfg.update_hz
    peak_a = 0.0
    last_frozen = None
    clean_exit = False
    last_status = float("-inf")
    last_esc = float("-inf")
    last_o = float("-inf")
    released_hold = False
    mark_state = None            # t 实验序列状态机：None/对时标记四态（fwd_lay/
                                 # hold/back_lay/rest）/自动轮换六态（lift_req/
                                 # lift_wait/hover_dwell/land_wait/leg_gap/
                                 # round_gap）/tail（结束静置）
    mark_t = 0.0
    mark_base = 0.0              # 标记起点的已倾量——回程按"回到起点"实算，
                                 # 去程被几何截短/中途取消也能回准
    auto_round = 1               # 自动轮换：当前轮（1 起）
    auto_legs_done = 0           # 本轮已收口腿数（--dual 每次 +2）
    auto_group = ()              # 在途抬落组（受理时定格；轮次收口后会推进）
    pump_on_last = 0.0           # 最近一次见泵开的时刻（腿间"泵歇 ≥2s"判据）
    hover_was = None
    step_was = False
    gate_was = None
    ho_note_was = None
    was_started = False
    at_pause = True
    aborted = False

    def hold_release_deny():
        if not eng.started:
            return "启动序列未完成"
        if eng.step_pending or eng.step_active:
            return "抬腿在途（悬停中按 i 落地收口）"
        if abs(eng.lean_pending) > 1e-6:
            return "倾身未铺完（按空格取消或等它完成）"
        if any(p != LegPhase.STANCE for p in eng.phase_of.values()):
            return "有腿未回支撑相"
        return None

    def io_freeze(e):
        """IO 持续失败降落伞：冻结悬停而不是炸退进程（同 climb_walk）。"""
        if not eng.frozen:
            eng.frozen = (f"IO 持续失败（{e}）——恢复后按 f；oo 取机 / "
                          "ESC×2 退出均可盲态运行（纯计时放气）")
            log.event(f"⚠ IO 降落伞：{e!r}")
            try:
                io.set_pump(False)   # 一次性停泵（泵引脚不依赖 I2C）
            except Exception:
                pass
    try:
        bot.move_feet(bot.crouch_feet(feet=eng.default_feet))
        drv.enable(True)
        time.sleep(0 if args.mock else 1.0)
        print("缓慢站起（竖直升至爬墙站位，吸盘轴⊥面）……")
        log.event("缓慢站起（竖直升至爬墙站位）")
        bot.glide_to(dict(eng.default_feet), 4.0)
        print("爬墙站位就位。")
        log.event("爬墙站位就位，进入就位暂停")
        if args.dry:
            print("⚠ 干跑模式：气路是仿真的，阀泵不会动——上墙严禁")
        if args.no_tank:
            print("⚠ 无罐模式：泵直抽歧管，没有储备真空——断电不保真空")
        fwd_room = eng._lean_room(1.0)
        print(f"倾身几何余量（站位起算）：前 ~{fwd_room:.0f}mm / "
              f"后 ~{eng._lean_room(-1.0):.0f}mm，每档 {args.lean_step:g}mm，"
              f"铺设速率 {LEAN_SPEED_MMS:g}mm/s")
        if leg_order:
            # 打 eng.slot_order 而不是回显参数：证明相位表真按新序生效了
            print("抬腿窗序（含启动吸附序）：" + "→".join(eng.slot_order)
                  + "——'轮到 X'提示与权重轮转按新序走；δ 标定按序分组")
        if args.dual:
            print("双足对抬：环序 " + "→".join(eng.slot_order)
                  + "——i 抬一对（对=所选腿+窗序继任，先后 0.6s 错峰），两腿"
                  "都悬停后再按 i 一并落地（错峰下探）；对抬期间恒 4 足吸附。"
                  "δ 按双足工况标定（三组=单足表×0.5/0.75/1.0），勿与单足"
                  "混口径")
            log.note("dual=1 slot_order=" + "_".join(eng.slot_order))
        ho_max = max(cfg.leg(n).handover_mm for n in LEG_NAMES)
        if ho_max > 0.0:
            print("零力交接开启：δ "
                  + " ".join(f"{n}={cfg.leg(n).handover_mm:g}"
                             for n in LEG_NAMES if cfg.leg(n).handover_mm > 0)
                  + f"mm，铺设 {cfg.handover_rate_mms:g}mm/s"
                  f"（最长 {ho_max / cfg.handover_rate_mms:.1f}s/次抬起）。"
                  "A/B 判据：vent 无可见弹跳、每轮下滑 <10mm、电流不再爬升")
            if cfg.handover_slot_w:
                print("交接载荷分配：按窗序轮转距离加权（晚轮到→先轮到）"
                      + "/".join(f"{v:.2f}" for v in cfg.handover_slot_w)
                      + ("——双足：份额按距离取值就地归一（模型最差单腿 -12%/"
                         "均值 -21%），δ 按开权重工况标、对序须按环序轮转"
                         if args.dual else
                         "——模型稳态内应力较均分低 ~30%，δ 表偏大时下调重标")
                      + "；口径假设按'轮到 X'提示轮转抬腿")
                log.note("handover_slot_w="
                         + ",".join(f"{v:.3f}" for v in cfg.handover_slot_w))

        print("就位暂停：确认无异常后按 p 开始全吸附启动序列（ESC×2 断电退出）")
        while True:
            k = read_key(0.1)
            if k == "p":
                at_pause = False
                last_esc = float("-inf")
                print("开始全吸附启动序列……")
                log.event("按 p：开始全吸附启动序列")
                break
            if k == "\x1b":
                if time.monotonic() - last_esc < 2.0:
                    aborted = True
                    print("\n未开始吸附，断电退出。")
                    coils_off(io)
                    drv.close()
                    return
                last_esc = time.monotonic()
                print("再按一次 ESC 确认退出（尚未吸附；断电后请扶稳机身）")
            now = time.monotonic()
            if now - last_status > STATUS_S:
                last_status = now
                v, c = (drv.read_voltage_v(), drv.read_current_a()) \
                    if args.mock else bot.check_power()
                peak_a = max(peak_a, c)
                watch.telemetry(v, c, peak_a, (0.0, 0.0, 0.0), note=" 就位暂停")

        t_wall = time.monotonic()
        while True:
            k = read_key(0)
            if k == "\x1b":
                if time.monotonic() - last_esc < 2.0:
                    clean_exit = True
                    break
                last_esc = time.monotonic()
                print("\n再按一次 ESC 确认退出（会放气——墙上禁用！）")
            elif k in ("UP", "DOWN") and released_hold:
                print("\n吸盘已放开（取机窗口），不可再动——取下后 ESC×2 退出")
            elif k in ("UP", "DOWN") and mark_state:
                print("\n实验序列进行中，倾身键已屏蔽（空格取消后再手动倾身）")
            elif k in ("UP", "DOWN"):
                d = args.lean_step if k == "UP" else -args.lean_step
                granted, deny = eng.request_lean(d)
                if deny:
                    print(f"\n倾身拒绝：{deny}")
                    log.event(f"倾身拒绝（{d:+g}）：{deny}")
                else:
                    total = eng.lean_mm + eng.lean_pending
                    print(f"\n倾身 {granted:+g}mm 已排队（目标累计 "
                          f"{total:+.0f}mm，铺完约 "
                          f"{abs(eng.lean_pending) / LEAN_SPEED_MMS:.1f}s）")
                    log.event(f"倾身 {granted:+g}mm（目标累计 {total:+.0f}）")
            elif k in LEG_KEYS and released_hold:
                print("\n吸盘已放开（取机窗口），不可再动")
            elif k in LEG_KEYS and mark_state:
                print("\n实验序列进行中，选腿会打乱轮换序——空格取消后再手动")
            elif k in LEG_KEYS:
                deny = eng.select_step_leg(LEG_KEYS[k])
                if deny:
                    print(f"\n选腿拒绝：{deny}")
                else:
                    print(f"\n已选 {'+'.join(eng.step_group())}，按 i 原地抬起")
            elif k == "i" and released_hold:
                print("\n吸盘已放开（取机窗口），不可再动")
            elif k == "i" and mark_state:
                print("\n实验序列进行中（标记完自动轮换抬落）——空格取消后"
                      "可手动抬落")
            elif k == "i":
                hover = eng.step_hover_leg()
                if hover:
                    hs = "+".join(eng.step_hover_legs())  # land 前取（首腿受理即下探）
                    deny = eng.step_land()
                    if deny:
                        print(f"\n落地不可用：{deny}")
                    else:
                        print(f"\n{hs} 落回站位，压入+吸附确认后收口")
                        log.event(f"落地：{hs}")
                else:
                    grp = "+".join(eng.step_group())
                    deny = eng.request_lift()
                    if deny:
                        print(f"\n抬起不可用：{deny}")
                    else:
                        print(f"\n原地抬起 {grp}（悬停后可继续 ↑/↓ 倾身，"
                              + ("两腿都悬停后按 i 一并落地）" if args.dual
                                 else "再按 i 落地）"))
                        log.event(f"原地抬起受理：{grp}")
            elif k == "t" and released_hold:
                print("\n吸盘已放开（取机窗口），不可再动")
            elif k == "t":
                if mark_state:
                    print("\n实验序列进行中……（空格取消）")
                else:
                    deny = None
                    if eng.step_pending or eng.step_active or eng.step_hover_leg():
                        deny = "抬腿在途（先收口再做标记）"
                    elif abs(eng.lean_pending) > 1e-6:
                        deny = "倾身未铺完（等它完成或空格取消）"
                    if deny is None:
                        mark_base = eng.lean_mm
                        granted, deny = eng.request_lean(MARK_LEAN_MM)
                    if deny:
                        print(f"\n对时标记不可用：{deny}")
                        log.event(f"对时标记拒绝：{deny}")
                    else:
                        mark_state = "fwd_lay"
                        auto_round, auto_legs_done = 1, 0
                        clip = ("" if granted >= MARK_LEAN_MM - 1e-6 else
                                f"（几何截短自 {MARK_LEAN_MM:g}，拟合幅值以"
                                "日志为准）")
                        plan = ("" if args.auto_rounds == 0 else
                                f" → 自动轮换 {args.auto_rounds} 轮×"
                                f"{LEGS_PER_ROUND} 腿 → 尾静置 "
                                f"{AUTO_TAIL_S:g}s")
                        print(f"\n实验序列开跑：对时标记（倾身 {granted:+g}mm"
                              f"{clip} → 停 {MARK_HOLD_S:g}s → 回位 → 静置 "
                              f"{MARK_REST_S:g}s）{plan}。勿碰机身/按键，"
                              "空格随时取消")
                        log.event(f"倾身 {granted:+g}mm（对时标记 去程）")
                        if args.auto_rounds:
                            log.event(f"实验序列开跑：标记+自动轮换 "
                                      f"{args.auto_rounds} 轮")
            elif k == " ":
                msgs = []
                if abs(eng.lean_pending) > 1e-6:
                    eng.cancel_lean()
                    msgs.append("未铺完的倾身已取消")
                if mark_state in ("fwd_lay", "hold", "back_lay"):
                    msgs.append("实验序列已取消：对时标记未完成作废（当前倾 "
                                f"{eng.lean_mm:+.1f}mm，↑/↓ 回位后重按 t）")
                    mark_state = None
                elif mark_state:
                    msgs.append("实验序列已取消：对时标记已入日志仍有效；自动"
                                f"轮换止于第 {auto_round} 轮 {auto_legs_done}"
                                f"/{LEGS_PER_ROUND} 腿（可 1~6+i 手动续跑）")
                    mark_state = None
                if eng.step_pending:
                    if eng.cancel_step():
                        msgs.append("未开始的抬起已取消")
                    else:
                        msgs.append("对抬已有腿在途，整组不可撤"
                                    "（悬停后按 i 一并落地）")
                if eng.step_hover_leg():
                    msgs.append("悬停中的腿不可取消——按 i 落地收口")
                if msgs:
                    print("\n" + "；".join(msgs))
                    log.event("空格：" + "；".join(msgs))
            elif k == "f" and eng.frozen:
                # 实验序列不必在此善后：冻结出现的当拍序列已在主循环自动中止
                print(f"\n解除冻结: {eng.frozen}（未铺完倾身已取消）")
                eng.clear_freeze()
                last_frozen = None
            elif k == "o" and mark_state:
                print("\n实验序列进行中——空格取消后再取机")
            elif k == "o":
                # 取机窗口（与 climb_walk 同款实现：逐足串行排气防六线圈
                # 同刻阶跃；改那边同步改这边）
                if released_hold:
                    print("\n已是放开状态——取下后 ESC×2 退出")
                else:
                    deny = hold_release_deny()
                    if deny:
                        last_o = float("-inf")
                        print(f"\n不允许放开：{deny}")
                    elif time.monotonic() - last_o < 2.0:
                        released_hold = True
                        ctl.pump_inhibit = True
                        io.set_pump(False)
                        log.event("取机窗口：停泵 → 逐足串行排气（0.2s 间隔）"
                                  "→ 六足舵机撑住（泵禁开）")
                        print("\n放开吸盘：停泵 → 逐足串行排气（约 1.5s）……")

                        adh_dead = pwr_dead = False

                        def _pickup_tick():
                            nonlocal last_status, peak_a, adh_dead, pwr_dead
                            try:
                                ctl.update(dt)   # 排气驱动：读失败不许打断取机
                            except Exception as e:
                                # I2C 失联（如 Errno 121 冻结）下也必须能放开
                                # 吸盘从墙上取机：VENTING 分支先 set_valve 再
                                # 读压确认，异常只丢确认不丢排气，退化纯计时
                                # 排气（与 ESC 退出 _exit_tick 同款加固，
                                # 08-23 实测该口径盲态可用）。原实现不接异常，
                                # 盲态按 oo 会炸穿主环→进程死→舵机断电，
                                # 半放气挂墙比冻结糟得多
                                if not adh_dead:
                                    adh_dead = True
                                    log.event("⚠ 取机期吸附状态机异常"
                                              f"（I2C 降级？），退化纯计时排气: {e}")
                            watch.poll()   # 零传感器 IO：盲态也照记（阀直驱留痕）
                            t_now = time.monotonic()
                            if t_now - last_status > STATUS_S:
                                last_status = t_now
                                try:
                                    pv, pc = (drv.read_voltage_v(),
                                              drv.read_current_a()) \
                                        if args.mock else bot.check_power()
                                    peak_a = max(peak_a, pc)
                                    watch.telemetry(pv, pc, peak_a, (0, 0, 0),
                                                    note=" 取机放气")
                                except Exception as e:
                                    # 欠压 RuntimeError 同吞：善后期只记录，
                                    # 绝不打断取机（与退出序列同口径）
                                    if not pwr_dead:
                                        pwr_dead = True
                                        log.event(f"⚠ 取机期电压读取失败: {e}")
                            if not args.mock:
                                time.sleep(dt)

                        for _ in range(int(0.3 / dt)):
                            _pickup_tick()
                        for i in range(6):
                            ctl.request_release(i)
                            # 直驱阀到排气位（幂等，理由见 climb_walk 同段）：
                            # 盲态下不直驱 = 只放得开一只脚
                            io.set_valve(i, False)
                            for _ in range(int(0.2 / dt)):
                                _pickup_tick()
                        print("已放开：全阀排气、泵停，六足仅舵机撑住原地。"
                              "取下后 ESC×2 退出（阀线圈通电中，勿久放）")
                    else:
                        last_o = time.monotonic()
                        print("\n再按一次 o 确认放开全部吸盘（六足保持站立）"
                              "——墙上=放开即坠，先扶稳机身（安全绳兜底）"
                              "再确认")

            if k is not None:
                log.event(f"键 {k!r} 倾={eng.lean_mm:+.1f}"
                          f"待={eng.lean_pending:+.1f}"
                          + ("（已放开）" if released_hold else ""))

            try:
                bot.move_feet(eng.update(dt))
            except WorkspaceError as e:
                if not eng.frozen:
                    eng.frozen = f"足端目标出工作空间（{e}）——ESC×2 退出"
            except OSError as e:
                io_freeze(e)
            watch.poll()

            gate_now = eng.gate_wait
            if gate_now and (gate_was is None or gate_now[0] != gate_was[0]):
                leg, soft = gate_now
                print(f"\n抬腿门槛：{'/'.join(soft)} 未深于 "
                      f"{cfg.lift_gate_kpa:g}kPa，{leg} 等泵压实再抬")
                log.event(f"抬腿门槛等待：{leg} 等 {'/'.join(soft)}")
            gate_was = gate_now
            note_now = eng.handover_note
            if note_now and note_now != ho_note_was:
                # 交接守卫（越界截断/偏离轮转退均分，与 climb_walk 同款）：
                # 截断=δ 没铺满、本次弹跳会比预期大，标 δ 时勿把这种次的
                # 残余当 δ 不够继续加——先换腿或重启复位几何
                print(f"\n⚠ {note_now}")
                log.event(f"交接守卫：{note_now}")
            ho_note_was = note_now
            hover_now = "+".join(eng.step_hover_legs()) or None
            if hover_now and hover_now != hover_was and not mark_state:
                # 自动序列中不打手动指引（悬停 1s 即自动落，i 提示会误导）
                tail = ("i 一并落地" if "+" in hover_now else
                        ("等搭档悬停后 i 一并落地" if args.dual else "i 落地"))
                print(f"\n{hover_now} 已悬停（离面 {cfg.lift_clearance:g}mm）："
                      f"↑/↓ 继续倾身，{tail}")
            hover_was = hover_now
            step_now = eng.step_pending or eng.step_active
            if step_was and not step_now:
                grp = "+".join(eng.step_group())
                if not mark_state:
                    print(f"\n抬起-落地完成（1~6 重新选腿；当前轮到 {grp}）")
                log.event(f"抬落完成，轮到 {grp}")
            step_was = step_now

            if mark_state:
                now_m = time.monotonic()
                if io.pump:
                    pump_on_last = now_m
                if eng.frozen:
                    # 冻结=组内异常，序列当拍中止（该组是否作废按协议 §6）；
                    # 悬停中的腿解冻后手动 i 收口，不留自动动作
                    print(f"\n⚠ 实验序列中止：全机冻结（第 {auto_round} 轮 "
                          f"{auto_legs_done}/{LEGS_PER_ROUND} 腿）——处理后按"
                          " f；悬停腿按 i 收口；该组数据按协议 §6 判定")
                    log.event(f"实验序列中止：冻结（第 {auto_round} 轮 "
                              f"{auto_legs_done}/{LEGS_PER_ROUND}）")
                    mark_state = None
                elif mark_state == "fwd_lay" and abs(eng.lean_pending) < 1e-6:
                    mark_state, mark_t = "hold", now_m
                elif mark_state == "hold" and now_m - mark_t >= MARK_HOLD_S:
                    back = mark_base - eng.lean_mm
                    if abs(back) < 1e-6:
                        mark_state, mark_t = "rest", now_m
                    else:
                        granted, deny = eng.request_lean(back)
                        if deny:
                            mark_state = None
                            print(f"\n⚠ 对时标记回程被拒：{deny}——↑/↓ 手动"
                                  "回位后重按 t 重做")
                            log.event(f"对时标记回程被拒：{deny}")
                        else:
                            mark_state = "back_lay"
                            log.event(f"倾身 {granted:+g}mm（对时标记 回程）")
                elif mark_state == "back_lay" and abs(eng.lean_pending) < 1e-6:
                    mark_state, mark_t = "rest", now_m
                    print(f"\n对时标记回位完成，静置 {MARK_REST_S:g}s"
                          "（勿碰机身/按键）……")
                elif mark_state == "rest" and now_m - mark_t >= MARK_REST_S:
                    log.event("对时标记完成")
                    if args.auto_rounds:
                        mark_state = "lift_req"
                        print(f"\n✓ 对时标记完成 → 自动轮换第 1/"
                              f"{args.auto_rounds} 轮开始（轮换序 "
                              + "→".join(eng.slot_order) + "）")
                        log.event("自动轮换：第 1 轮开始")
                    else:
                        mark_state = None
                        print("\n✓ 对时标记完成（--auto-rounds 0），"
                              "手动开跑第 1 轮抬落")
                elif mark_state == "lift_req":
                    grp = "+".join(eng.step_group())
                    auto_group = eng.step_group()
                    deny = eng.request_lift()
                    if deny:
                        # 冻结在上面截获，走到这的拒绝=非预期状态，中止留人
                        mark_state = None
                        print(f"\n⚠ 实验序列中止：抬起被拒（{deny}）——可 "
                              "1~6+i 手动续跑或重按 t 整组重来")
                        log.event(f"实验序列中止：抬起被拒 {deny}")
                    else:
                        mark_state = "lift_wait"
                        print(f"\n[自动 {auto_round}/{args.auto_rounds} 轮] "
                              f"原地抬起 {grp}"
                              f"（{auto_legs_done + len(auto_group)}"
                              f"/{LEGS_PER_ROUND}）")
                        log.event(f"原地抬起受理：{grp}")
                elif mark_state == "lift_wait" and not eng.step_pending \
                        and len(eng.step_hover_legs()) == len(auto_group):
                    mark_state, mark_t = "hover_dwell", now_m
                elif mark_state == "hover_dwell" \
                        and now_m - mark_t >= AUTO_HOVER_S:
                    hs = "+".join(eng.step_hover_legs())
                    deny = eng.step_land()
                    if deny:
                        mark_state = None
                        print(f"\n⚠ 实验序列中止：落地被拒（{deny}）——悬停"
                              "腿按 i 手动收口")
                        log.event(f"实验序列中止：落地被拒 {deny}")
                    else:
                        mark_state = "land_wait"
                        log.event(f"落地：{hs}")
                elif mark_state == "land_wait" \
                        and not (eng.step_pending or eng.step_active):
                    auto_legs_done += len(auto_group)
                    print(f"\n[自动 {auto_round}/{args.auto_rounds} 轮] "
                          f"{'+'.join(auto_group)} 收口"
                          f"（{auto_legs_done}/{LEGS_PER_ROUND}）")
                    if auto_legs_done < LEGS_PER_ROUND:
                        mark_state, mark_t = "leg_gap", now_m
                    elif auto_round < args.auto_rounds:
                        log.event(f"自动轮换：第 {auto_round} 轮完成")
                        print(f"[自动] 第 {auto_round} 轮完成，轮间静置 "
                              f"{AUTO_ROUND_GAP_S:g}s……")
                        auto_round += 1
                        auto_legs_done = 0
                        mark_state, mark_t = "round_gap", now_m
                    else:
                        log.event(f"自动轮换：第 {auto_round} 轮完成")
                        print(f"[自动] 第 {auto_round} 轮完成——结束静置 "
                              f"{AUTO_TAIL_S:g}s（电流回落观测窗）……")
                        mark_state, mark_t = "tail", now_m
                elif mark_state == "leg_gap":
                    if now_m - mark_t >= AUTO_GAP_MIN_S \
                            and now_m - pump_on_last >= AUTO_GAP_PUMP_S:
                        mark_state = "lift_req"
                    elif now_m - mark_t >= AUTO_GAP_MAX_S:
                        print(f"\n⚠ 腿间间歇 {AUTO_GAP_MAX_S:g}s 泵仍未歇——"
                              "超时继续（漏气？盯状态行盘压）")
                        log.event("自动轮换：间歇超时（泵未歇）继续")
                        mark_state = "lift_req"
                elif mark_state == "round_gap" \
                        and now_m - mark_t >= AUTO_ROUND_GAP_S:
                    mark_state = "lift_req"
                    print(f"\n[自动] 第 {auto_round}/{args.auto_rounds} "
                          "轮开始")
                    log.event(f"自动轮换：第 {auto_round} 轮开始")
                elif mark_state == "tail" and now_m - mark_t >= AUTO_TAIL_S:
                    mark_state = None
                    print(f"\n✓ 实验序列完成：对时标记 + {args.auto_rounds} "
                          f"轮×{LEGS_PER_ROUND} 腿 + 结束静置——可 ESC×2 "
                          "退出取数")
                    log.event("实验序列完成")

            if eng.started and not was_started:
                was_started = True
                t_help = ("t 对时标记" if args.auto_rounds == 0 else
                          f"t 一键实验（标记+{args.auto_rounds} 轮轮换）")
                print(f"\n✓ 六足吸附完成：↑/↓ 前/后倾身  {t_help}  1~6 选腿"
                      "  i 抬/落  空格取消  f 解冻  o×2 取机  ESC×2 退出")
            if eng.frozen != last_frozen:
                last_frozen = eng.frozen
                if eng.frozen:
                    print(f"\a\n⚠⚠⚠ 全机冻结: {eng.frozen} —— 处理后按 f 继续\n")

            now = time.monotonic()
            if now - last_status > STATUS_S:
                last_status = now
                try:
                    v, c = (drv.read_voltage_v(), drv.read_current_a()) \
                        if args.mock else bot.check_power()
                except OSError as e:
                    io_freeze(e)
                else:
                    peak_a = max(peak_a, c)
                    tag = (f" 倾{eng.lean_mm:+.1f}"
                           + (f"→{eng.lean_mm + eng.lean_pending:+.0f}"
                              if abs(eng.lean_pending) > 1e-6 else "")
                           + (" 已放开" if released_hold else "")
                           + ((f" 自动{auto_round}/{args.auto_rounds}"
                               if mark_state not in ("fwd_lay", "hold",
                                                     "back_lay", "rest")
                               else " 对时标记") if mark_state else ""))
                    print("\r" + status_line(eng, ctl, v, c, peak_a,
                                             eng.cmd, tag) + "  ",
                          end="", flush=True)
                    watch.telemetry(v, c, peak_a, eng.cmd, note=tag)
            lag = time.monotonic() - t_wall
            if lag > 5 * dt:
                log.event(f"⚠ 主环卡顿 {lag * 1000:.0f}ms（目标 {dt * 1000:.0f}ms）")
            time.sleep(max(0.0, dt - lag))
            t_wall = time.monotonic()
    except KeyboardInterrupt:
        log.event("Ctrl-C 中断")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if aborted:
            log.event("退出：就位暂停处确认退出（未吸附，已断电）")
        elif at_pause:
            coils_off(io)
            log.event("中断：就位暂停处（未吸附，阀线圈已断电，舵机保持使能）")
            print("\n在就位暂停处中断：未吸附，阀线圈已断电。"
                  "舵机断电请关电源或跑 climb_walk --release。")
        elif clean_exit:
            # 退出序列与 climb_walk 同款（逐足串行放气防电流阶跃、盲态计时
            # 兜底、coils_off 必达；改那边同步改这边）
            log.event("退出序列：停泵 → 逐足串行放气（通→断，0.2s 间隔）"
                      " → 舵机断电（不回站姿）")
            print("\n退出：停泵 -> 逐足串行放气 -> 舵机断电"
                  "（停在爬墙站位，约 5s；Ctrl-C 已屏蔽，等它跑完）")
            prev_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                ctl.pump_inhibit = True
                io.set_pump(False)
                if not args.mock:
                    time.sleep(0.3)
                t_tlm = float("-inf")
                adh_dead = False

                def _exit_tick():
                    nonlocal t_tlm, peak_a, adh_dead
                    try:
                        ctl.update(dt)
                        watch.poll()
                    except Exception as e:
                        if not adh_dead:
                            adh_dead = True
                            log.event("⚠ 退出期吸附状态机异常（I2C 降级？），"
                                      f"退化纯计时放气: {e}")
                    if time.monotonic() - t_tlm > STATUS_S:
                        t_tlm = time.monotonic()
                        try:
                            v, c = drv.read_voltage_v(), drv.read_current_a()
                            peak_a = max(peak_a, c)
                            watch.telemetry(v, c, peak_a, (0, 0, 0),
                                            note=" 退出放气")
                        except Exception as e:
                            log.event(f"⚠ 退出期电压读取失败，停采: {e}")
                            t_tlm = float("inf")

                for i in range(6):
                    ctl.force_release(i)
                    for _ in range(int(1.5 / dt)):
                        _exit_tick()
                        if not args.mock:
                            time.sleep(dt)
                        if ctl.state[i] == FootState.RELEASED:
                            break
                    else:
                        ctl.abandon_release(i)
                        log.event(f"⚠ {LEG_NAMES[i]} 排气 1.5s 未确认"
                                  "（堵/传感器漂移？），线圈照断继续")
                    io.set_valve(i, True)
                    for _ in range(int(0.2 / dt)):
                        _exit_tick()
                        if not args.mock:
                            time.sleep(dt)
            finally:
                coils_off(io)
                log.event("退出：阀线圈已断电（coils_off）")
                drv.close()
                signal.signal(signal.SIGINT, prev_int)
            print("完成（阀线圈已断电）。")
            log.event("退出序列完成（舵机断电）")
        else:
            log.event(f"中断：不放气退出，冻结={eng.frozen or '无'}")
            print(f"\n中断：不放气退出。冻结: {eng.frozen or '无'}；"
                  "善后请跑 climb_walk --release。")
        log.close("正常退出" if (clean_exit or aborted) else "中断退出")


if __name__ == "__main__":
    main()
