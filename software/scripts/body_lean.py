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
                           HANDOVER_SPEED_MMS)
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES
from hexapod.kinematics import WorkspaceError
from hexapod.runlog import RunLog, ClimbWatch

from climb_walk import status_line, coils_off   # 显示/收尾与 climb_walk 同源

STATUS_S = 0.5
LEG_KEYS = {"1": "L1", "2": "L2", "3": "L3", "4": "R1", "5": "R2", "6": "R3"}


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
    ap.add_argument("--press-delta", type=float, default=None,
                    help="预压行程 mm，覆盖全部腿（默认用 config 值 "
                         f"{DEFAULT_CONFIG.legs[0].press_delta_mm:g}）")
    ap.add_argument("--handover", default=None,
                    help="vent 前零力交接 δ mm（默认关）：统一值如 8，或逐腿 "
                         "L1:17,R1:15,L3:11,R3:9,R2:5,L2:5（未给的腿 0）。"
                         "i 抬起前自动先交接（被抬腿沿上坡还 δ 卸载、支撑腿"
                         "各接 δ/5，铺完才放气）。本脚本是 δ 的 A/B 标定"
                         "入口：起标=实测单次弹跳×0.8，宁欠勿过。范围 0~25。"
                         "注意：反复抬同一条腿时支撑指令每次多漂下坡 δ/5"
                         "（开 --handover-weights 也一样：偏离轮转的抬腿被"
                         "引擎守卫退回均分）、轮抬一圈才互相抵回——单腿连标 "
                         "~10 次后换腿或重启，越界由引擎交接截断兜底（当场"
                         "提前放气并提示，不再推到冻结）")
    ap.add_argument("--handover-weights", nargs="?",
                    const="0.6,0.25,0.1,0.05,0", default=None,
                    help="交接载荷分配按窗序轮转距离加权（5 权重：w1=最晚才"
                         "轮到抬的支撑腿=刚抬过的，w5=下一个要抬的；自动归一化，"
                         "需同时开 --handover）。不带值=激进档 0.6,0.25,0.1,"
                         "0.05,0：晚轮到多接——早收到的载荷被后续落地的零预载"
                         "锁定稀释回集体，模型稳态循环内应力较均分 -32%%"
                         "（HANDOVER-DESIGN 附录 A）。口径假设按'轮到 X'提示"
                         "轮转抬腿；偏离轮转（本脚本反复抬同一腿标 δ 是常态）"
                         "的抬腿由引擎守卫自动退回均分 δ/5 并提示——原先照套"
                         "权重会把 0.6δ 每次砸向同一条支撑腿，实测 4~6 次抬腿"
                         "吹爆工作空间（审核）。"
                         "⚠ 非单调，且改变稳态运行点，δ 表需下调重标")
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
    handover = None
    if args.handover is not None:
        try:
            handover = parse_handover(args.handover)
        except ValueError as e:
            ap.error(str(e))
    ho_w = None
    if args.handover_weights is not None:
        if handover is None:
            ap.error("--handover-weights 需要同时开 --handover（权重只作用于"
                     "交接载荷的分配）")
        try:
            ho_w = parse_handover_weights(args.handover_weights)
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
             f" handover={ho_txt}")
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
    eng = ClimbEngine(cfg, ctl)
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
        ho_max = max(cfg.leg(n).handover_mm for n in LEG_NAMES)
        if ho_max > 0.0:
            print("零力交接开启：δ "
                  + " ".join(f"{n}={cfg.leg(n).handover_mm:g}"
                             for n in LEG_NAMES if cfg.leg(n).handover_mm > 0)
                  + f"mm，铺设 {HANDOVER_SPEED_MMS:g}mm/s"
                  f"（最长 {ho_max / HANDOVER_SPEED_MMS:.1f}s/次抬起）。"
                  "A/B 判据：vent 无可见弹跳、每轮下滑 <10mm、电流不再爬升")
            if cfg.handover_slot_w:
                print("交接载荷分配：按窗序轮转距离加权（晚轮到→先轮到）"
                      + "/".join(f"{v:.2f}" for v in cfg.handover_slot_w)
                      + "——模型稳态内应力较均分低 ~30%，δ 表偏大时下调重标；"
                      "口径假设按'轮到 X'提示轮转抬腿")
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
            elif k in LEG_KEYS:
                deny = eng.select_step_leg(LEG_KEYS[k])
                if deny:
                    print(f"\n选腿拒绝：{deny}")
                else:
                    print(f"\n已选 {eng.step_leg}，按 i 原地抬起")
            elif k == "i" and released_hold:
                print("\n吸盘已放开（取机窗口），不可再动")
            elif k == "i":
                hover = eng.step_hover_leg()
                if hover:
                    deny = eng.step_land()
                    if deny:
                        print(f"\n落地不可用：{deny}")
                    else:
                        print(f"\n{hover} 落回站位，压入+吸附确认后收口")
                        log.event(f"落地：{hover}")
                else:
                    deny = eng.request_lift()
                    if deny:
                        print(f"\n抬起不可用：{deny}")
                    else:
                        print(f"\n原地抬起 {eng.step_leg}（悬停后可继续 ↑/↓ "
                              "倾身，再按 i 落地）")
                        log.event(f"原地抬起受理：{eng.step_leg}")
            elif k == " ":
                msgs = []
                if abs(eng.lean_pending) > 1e-6:
                    eng.cancel_lean()
                    msgs.append("未铺完的倾身已取消")
                if eng.step_pending:
                    eng.cancel_step()
                    msgs.append("未开始的抬起已取消")
                if eng.step_hover_leg():
                    msgs.append("悬停中的腿不可取消——按 i 落地收口")
                if msgs:
                    print("\n" + "；".join(msgs))
                    log.event("空格：" + "；".join(msgs))
            elif k == "f" and eng.frozen:
                print(f"\n解除冻结: {eng.frozen}（未铺完倾身已取消）")
                eng.clear_freeze()
                last_frozen = None
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
            hover_now = eng.step_hover_leg()
            if hover_now and hover_now != hover_was:
                print(f"\n{hover_now} 已悬停（离面 {cfg.lift_clearance:g}mm）："
                      "↑/↓ 继续倾身，i 落地")
            hover_was = hover_now
            step_now = eng.step_pending or eng.step_active
            if step_was and not step_now:
                print(f"\n抬起-落地完成（1~6 重新选腿；当前轮到 {eng.step_leg}）")
                log.event(f"抬落完成，轮到 {eng.step_leg}")
            step_was = step_now

            if eng.started and not was_started:
                was_started = True
                print("\n✓ 六足吸附完成：↑/↓ 前/后倾身  1~6 选腿  i 抬/落  "
                      "空格取消  f 解冻  o×2 取机  ESC×2 退出")
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
                           + (" 已放开" if released_hold else ""))
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
