#!/usr/bin/env python3
"""P4 步态-吸附联调：Mock 干跑 / 实机架空 / 地面玻璃板 / 上墙 共用入口。

流程：缓慢站起（直达爬墙站位）-> 就位暂停（按 p 继续）-> 全吸附启动序列
（逐足"压入->抽气确认->下一腿"；六腿同时压没有反力座压不实）-> 键盘遥控
CLIMB 步态。
  p   就位暂停后开始全吸附启动序列
  w/s 前进/后退   a/d 左移/右移   q/e 左转/右转   空格 停
  i   单步：只走"当前轮到"的一条腿（轮次记录在引擎里，窗序 L1→L2→L3→
      R1→R2→R3，连续行走的抬腿同样推进轮次），等到该腿相位窗以 SPEED
      向前走完整摆动+吸附确认后自动停；未开始的单步可按空格取消
  f   解除冻结（人工处理完报警后按）        ESC 安全退出（停走->停泵->逐足
      串行放气->断电，终态停在爬墙站位；要地面站姿另跑 --release）
  o×2 放开全部吸盘但六足保持站立（地面取机用：全阀排气+泵停，舵机撑住原地，
      整机可直接从玻璃上拿起）。仅站立非走动时允许；墙上严禁（等于坠落）

用法:
  python climb_walk.py --mock                # 无硬件干跑（Mock 吸盘必吸上）
  python climb_walk.py --mock --air          # 干跑架空路径（Mock 吸盘全漏）
  python climb_walk.py --dry                 # 真舵机 + 仿真气路：阀泵/气路
                                             # GPIO/I2C 不碰（舵机继电器 GPIO17
                                             # 仍占用），纯排练步态动作（气动舱
                                             # 未接全时用；吸附确认是假的）
  python climb_walk.py --air                 # 实机架空：阀/泵真动作，吸不上不停走
  python climb_walk.py --no-tank             # 无罐短测（储气罐未装）：泵直抽
                                             # 歧管，罐压传感器不参与；没有储备
                                             # 真空——挽救弱、断电不保真空，
                                             # 只在地面做短暂测试，严禁上墙
  python climb_walk.py                       # 地面玻璃板 / 上墙（全链路）
  python climb_walk.py --sag-comp 3          # 上墙下滑补偿：每次抬腿把支撑足
                                             # 沿下坡向额外平移 3mm，顶回"每抬
                                             # 一腿被拽下一截"的棘轮下滑（头朝
                                             # 上贴墙口径；从小标定，宁欠勿过）
  python climb_walk.py --release             # 善后：逐足串行放气+回站姿

黑匣子：每次运行自动写 software/logs/climb_YYYYmmdd_HHMMSS.log（事件 +
0.5s 遥测混排，含吸附跳变现场盘压/泵阀动作/电压电流/按键/异常栈；行缓冲
+ 每秒 fsync，死机断电最多丢 1s 尾巴——2026-08-18 掉电死机无日志可验尸的
教训）。验尸先看文件尾：最后一行 TLM 是死亡快照，末尾 EVT 是最后动作。

安全（P4-GUIDE）：
  - ESC×2 退出与 --release 的放气序列**屏蔽 Ctrl-C**（打断会把正在排气的阀
    留在通电电平，08-17 线圈发热事故同款）——序列约 5~10s，等它跑完
  - Ctrl-C 中断＝不放气退出，善后跑 --release。⚠ 进程退出后六阀翻回"通罐"位：
    六足全吸附时≈保持吸附；**有腿悬空时罐压会经敞口吸盘泄掉**——墙上异常
    优先"保持进程 + 冻结态"悬停，不到万不得已不杀进程
  - 上墙全程安全绳；--air 旁路只用于架空，上墙严禁
  - 电压/电流状态行常显：墙上五足持续剪切载荷 ~3A 级是常态，盯"吸附后总电流
    回落基线"判据（不回落 = 该腿 press_delta 没吃干净，舵机在和真空对抗）
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
from hexapod.climb import ClimbEngine, LegPhase, COMP_TAIL_MAX
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES
from hexapod.kinematics import WorkspaceError
from hexapod.runlog import RunLog, ClimbWatch, PHASE_CH, ADH_CH

SPEED = 15.0    # mm/s（爬墙宁慢勿快；跑顺再提）
TURN = 0.1      # rad/s
STATUS_S = 0.5  # 状态行刷新间隔


def read_key(timeout):
    """读一个按键。必须用 os.read 直读 fd——sys.stdin.read(1) 是带缓冲的
    TextIOWrapper，会把方向键 3 字节整批吞进 Python 缓冲、只吐出 ESC，
    随后按 fd 判"有没有尾巴"必然判空，防误触整体失效（审核发现 #1/#3）。"""
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    data = os.read(sys.stdin.fileno(), 8)
    if not data:
        return None
    if data[0:1] == b"\x1b":
        # 同批到达多字节 = 方向键/Alt 组合等转义序列：整包丢弃，不当 ESC 用。
        # 误触方向键绝不能触发"放气退出"（墙上等于坠落）。
        if len(data) > 1:
            return None
        # 裸 ESC：稍等一眼 fd，慢终端分包送到的转义尾巴同样丢弃
        r2, _, _ = select.select([sys.stdin], [], [], 0.02)
        if r2:
            os.read(sys.stdin.fileno(), 8)
            return None
    return data[:1].decode("latin-1")


def status_line(eng, ctl, v, c, peak, cmd, tag=""):
    legs = " ".join(
        f"{n}{PHASE_CH[ph]}{'!' if leak else ADH_CH[adh]}"
        for n, (ph, adh, leak) in eng.status().items())
    # 盘差 = 最差（最接近大气）的已吸附盘压 kPa：墙上真正承载的是它，无罐
    # 模式尤其要盯——泵一直不停 = 盘压压不进 -75 停泵线，法向力和横向刚度
    # 都打折。读数一律取状态机镜像（ctl.last_*，与黑匣子遥测同源）：显示
    # 路径零传感器 IO——不抢 I2C 突发配额，也绝不因转换超时抛 IOError 炸
    # 主环。ATTACHED 足控制环每周期都在读，镜像不陈旧
    att = [(ctl.last_kpa[i], LEG_NAMES[i])
           for i in range(6) if ctl.is_attached(i)
           and ctl.last_kpa[i] is not None]
    cup_txt = "盘差 {1}{0:6.1f}".format(*max(att)) if att else "盘差 --"
    if ctl.tankless:
        tank_txt = "罐 无罐"          # 无罐模式不读罐压传感器（读了也是悬空假数）
    else:
        tf = " 罐压失效!" if ctl.tank_fault else ""
        tank_txt = (f"罐 {ctl.last_tank_kpa:6.1f}kPa{tf}"
                    if ctl.last_tank_kpa is not None else f"罐 --{tf}")
    head = ("启动" if not eng.started else f"t={eng.t:5.1f}") + tag
    vx, vy, wz = cmd
    sp = "停" if not (vx or vy or wz) else f"{vx:+.0f}/{vy:+.0f}/{wz:+.2f}"
    return (f"[{head}] {legs}  速 {sp}  {cup_txt}  {tank_txt}  "
            f"{v:.2f}V {c:5.2f}A 峰 {peak:5.2f}A")


def coils_off(io):
    """退出前必做：六路阀线圈全部断电（GPIO 拉低）+ 泵停。
    本机阀是"通电=排气、断电=通罐"，排气是**维持态**——退出序列把 GPIO
    拉高放气后就退进程，引脚会停在高电平，六个线圈一直通电发热（实机
    2026-08-17 复现）。收尾必须像 p4_mosfet_check 一样全部拉低再走。
    注意 set_valve(True) 恰是线圈断电电平（True=通罐位=低电平）；罐里若有
    残余真空，地面吸盘可能被轻微重新吸住，关 12V 或重跑 --release 可放开。"""
    for i in range(6):
        io.set_valve(i, True)
    io.set_pump(False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    ap.add_argument("--air", action="store_true",
                    help="架空模式：吸不上也继续走（旁路互锁与罐压冻结），上墙严禁")
    ap.add_argument("--dry", action="store_true",
                    help="真舵机 + 仿真气路：不碰气路 GPIO(阀/泵)与 I2C，阀泵"
                         "不动（舵机继电器 GPIO17 仍由 driver 占用），吸附确认"
                         "由 Mock 假装成功；纯验证步态动作，上墙严禁")
    ap.add_argument("--no-tank", action="store_true",
                    help="无罐短测：泵按'抽气需求+吸附足压滞环'直抽歧管，"
                         "罐压传感器不参与，SUCK 超时放宽到 2.5s；没有储备真空"
                         "（挽救弱/断电不保真空），只在地面短测，严禁上墙")
    ap.add_argument("--cycle", type=float, default=DEFAULT_CONFIG.climb_cycle_time,
                    help="步态周期 s（先慢后提速）")
    ap.add_argument("--sag-comp", type=float,
                    default=DEFAULT_CONFIG.climb_sag_comp_mm,
                    help="下滑补偿 mm/抬腿事件（默认 0=关）：每次抬腿把全部支撑"
                         "足沿下坡方向额外平移这么多，顶回墙上'每抬一腿被拽下"
                         "一截'的棘轮下滑。方向按头朝上贴墙（下坡=-X）随转向"
                         "积分旋转。上墙标定：从 2~3 起步逐次加，盯每周期净位"
                         "移；只补得回弹性让位，超过实测下沉量会把吸着的盘往"
                         "下坡拖，宁欠勿过（上限 8）。每事件实际量=min(设定,"
                         "(40-半步幅)/5)：满速时 4mm 封顶（后腿工作空间总账），"
                         "设定 >4 想吃满就把 SPEED 降下来走")
    ap.add_argument("--release", action="store_true",
                    help="只做逐足串行放气+回站姿")
    args = ap.parse_args()
    if args.release and args.dry:
        ap.error("--release 不能配 --dry：真舵机配仿真气路，'放气'不碰任何"
                 "实阀也不做压力确认，随后回站姿会用舵机硬拽仍吸在玻璃上的"
                 "吸盘——恰是本路径要防的事。真机善后去掉 --dry 重跑")
    if args.cycle < 1.0:
        ap.error(f"--cycle {args.cycle} 非法：步态周期至少 1s（0/负值会在六足"
                 "吸附完成后才除零崩溃，审核发现 #8）")
    if not 0.0 <= args.sag_comp <= 8.0:
        ap.error(f"--sag-comp {args.sag_comp} 非法：范围 0~8mm。补偿超过真实"
                 "下沉量的部分会把吸附中的支撑盘沿面往下坡拖，宁欠勿过；"
                 "工作空间限额下更大的设定值也吃不满，没有意义")
    if not args.release and not sys.stdin.isatty():
        # TTY 检查必须在碰任何硬件之前：否则真机先上电站立、再在 termios
        # 处崩溃断电瘫倒（审核发现 #5）。--release 无需键盘，放行。
        sys.exit("需要交互终端（ssh 加 -t；勿用 nohup/管道跑本脚本）")

    cfg = replace(DEFAULT_CONFIG, climb_cycle_time=args.cycle,
                  climb_sag_comp_mm=args.sag_comp)
    if args.air:
        cfg = replace(cfg, max_attach_retry=1)   # 架空必 FAULT，少陪跑几轮重试

    # 黑匣子：建在碰任何硬件之前，硬件初始化失败也要留痕（excepthook 兜底）
    log = RunLog(os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs"))
    print(f"黑匣子日志: {log.path}")
    # 标签按标志组合拼接：先到先得的阶梯会把 --dry --air 记成"dry"，而该跑
    # 实为 air 旁路互锁语义——验尸首行定性字段不许失真
    mode = "+".join(s for s, on in (("mock", args.mock), ("dry", args.dry),
                                    ("air", args.air), ("no-tank", args.no_tank))
                    if on) or "实机全链路"
    log.note(f"模式={mode} port={args.port} release={args.release}")
    log.note(f"参数: cycle={cfg.climb_cycle_time}s sag_comp={cfg.climb_sag_comp_mm}mm"
             f" SPEED={SPEED} TURN={TURN} update_hz={cfg.update_hz:g}"
             f" max_step={cfg.climb_max_step} comp_tail_max={COMP_TAIL_MAX}")
    _prev_hook = sys.excepthook

    def _crash_hook(tp, val, tb):
        # 任何未捕获异常（含硬件初始化失败/欠压停机）落盘后再走默认打印
        log.exc(val)
        log.close("uncaught")
        _prev_hook(tp, val, tb)
    sys.excepthook = _crash_hook

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    if args.mock or args.dry:
        # --dry：舵机真走，气路用 Mock 顶替（不碰 GPIO/I2C，阀泵不会动）。
        # --air 叠加时 Mock 全漏，可在真腿上排练重试动作。
        io = MockVacuumIO(6)
        if args.air:
            io.sealed = [False] * 6
    else:
        from hexapod.adhesion import Pi5VacuumIO
        io = Pi5VacuumIO(6)
    # --air 时罐压传感器可能未接：泵降级为"有脚在抽就开"，不被 tank_fault 停死
    ctl_kw = dict(pump_without_tank=args.air, tankless=args.no_tank)
    if args.no_tank:
        ctl_kw["suck_timeout_s"] = 2.5   # 泵实时抽比罐存量慢，放宽超时
    ctl = AdhesionController(io, **ctl_kw)
    bot = Hexapod(drv, cfg)
    eng = ClimbEngine(cfg, ctl, air_mode=args.air)
    watch = ClimbWatch(log, eng, ctl, io, cfg)   # 挂上吸附钩子 + 跳变轮询
    log.note(f"阈值: ATTACH={ATTACH_KPA} PUMP_ON={PUMP_ON_KPA}"
             f" PUMP_OFF={PUMP_OFF_KPA} suck_timeout={ctl.suck_timeout_s}s"
             f" leak_rescue={cfg.leak_rescue_s}s retry={cfg.max_attach_retry}"
             f" volt_warn/cutoff={cfg.volt_warn}/{cfg.volt_cutoff}V")

    if args.release:
        # 顺序是安全关键：先放气（吸着的脚不能被舵机硬拉），再上舵机回站姿。
        # 放气逐足串行（与 ESC 退出同口径）：一次只通一个阀（通电=排气），
        # 确认回大气即断该阀电——本路径原先六阀同刻通电，正是事故善后指定
        # 工具自己在复现"六线圈同刻阶跃"嫌疑触发条件。断电翻回通罐位后，
        # 每足单向阀（只许吸盘→歧管）挡住罐侧残余真空，不会被明显重吸。
        # 全程屏蔽 Ctrl-C：打断会把阀线圈留在通电电平（08-17 事故同款终态），
        # 收尾必达由 finally 兜底；真卡死还有 SIGTERM/关电源
        prev_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
        log.event("release 善后：停泵 → 逐足串行放气（通→断，0.2s 间隔）"
                  "→ 回站姿")
        print("善后：停泵 -> 逐足串行放气 -> 回站姿（约 10s；Ctrl-C 已屏蔽）")
        real_pneu = not args.mock          # --dry 已被参数检查拦掉
        try:
            io.set_pump(False)
            if real_pneu:
                time.sleep(0.3)            # 母线从泵负载缓过来再上阀
            from hexapod.adhesion import RELEASE_KPA
            for i in range(6):
                io.set_valve(i, False)     # 通电=排气
                if real_pneu:              # 放气用读数确认，不靠开环 sleep
                    t_end = time.monotonic() + 1.5
                    while time.monotonic() < t_end:
                        if io.read_foot_kpa(i) >= RELEASE_KPA:
                            break
                        time.sleep(0.05)
                    else:
                        print(f"⚠ {LEG_NAMES[i]} 1.5s 未回大气（排气不畅？），"
                              "线圈照断继续，回站姿需谨慎")
                        log.event(f"⚠ release：{LEG_NAMES[i]} 1.5s 未回大气")
                io.set_valve(i, True)      # 确认排空即断该阀电（通→断）
                if real_pneu:
                    time.sleep(0.2)        # 足间间隔
            # 使能瞬间是硬跳：预置到爬墙站位（离中断时的姿态最近），再缓动回地面站姿
            bot.move_feet(eng.default_feet)
            drv.enable(True)
            time.sleep(0 if args.mock else 1.0)
            bot.stand(3.0)
        finally:
            coils_off(io)      # 收尾必达：无论上面哪步异常都把线圈断掉
            drv.close()
            signal.signal(signal.SIGINT, prev_int)
        print("已全放气并回站姿（阀线圈已断电）。")
        log.close("release 完成")
        return

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = vy = wz = 0.0
    dt = 1.0 / cfg.update_hz
    peak_a = 0.0
    last_frozen = None
    clean_exit = False
    # 计时一律 time.monotonic()：树莓派无 RTC，chrony 联网后阶跃校时，墙钟
    # 倒跳会让单次 ESC 凑满 2s 双击窗（墙上等于坠落）、正跳向黑匣子灌假
    # "主环卡顿"。单调钟起点可能贴近 0（刚开机），比对基线必须用 -inf
    last_status = float("-inf")
    last_esc = float("-inf")
    last_o = float("-inf")
    released_hold = False   # 'oo' 取机窗口：吸盘已全放开，六足仅舵机撑住
    step_was = False        # 单步在途标志（沿检测"在途→结束"打完成提示）
    was_started = False
    at_pause = True      # 就位暂停中（尚未开始任何吸附动作）
    aborted = False      # 在暂停处确认退出（finally 不再做善后）

    def hold_release_deny():
        """'oo' 放开吸盘的前置检查：返回拒绝原因，None = 允许。
        站立非走动 = 启动序列已完成 + 速度指令为零 + 六足全在支撑相。"""
        if not eng.started:
            return "启动序列未完成"
        if vx or vy or wz:
            return "尚在走动（先按空格停）"
        if eng.step_pending or eng.step_active:
            return "单步在途（等它走完，或未开始的按空格取消）"
        if any(p != LegPhase.STANCE for p in eng.phase_of.values()):
            return "有腿未回支撑相（等本步收尾再按）"
        return None
    try:
        # 缓慢站起，一步到位进爬墙站位：蹲姿直接用爬墙站位（吸盘轴⊥面的解，
        # 约 reach 176）的 XY，从蹲到站是纯竖直上升——吸盘落地后不横拖。
        # （旧流程"站姿130→爬墙站位176"的 glide 会拖着承重吸盘划 46mm）。
        # 站起放在 try 内：4s glide 里 Ctrl-C 同样要走 at_pause 收尾断线圈
        bot.move_feet(bot.crouch_feet(feet=eng.default_feet))
        drv.enable(True)
        time.sleep(0 if args.mock else 1.0)
        print("缓慢站起（竖直升至爬墙站位，吸盘轴⊥面）……")
        log.event("缓慢站起（竖直升至爬墙站位）")
        bot.glide_to(dict(eng.default_feet), 4.0)
        print("爬墙站位就位。")
        log.event("爬墙站位就位，进入就位暂停")
        if args.air:
            print("⚠ 架空模式：吸附失败不冻结，互锁旁路——上墙严禁本模式")
        if args.dry:
            print("⚠ 干跑模式：气路是仿真的，阀泵不会动，状态行气压是假数"
                  "——上墙严禁")
        if args.no_tank:
            print("⚠ 无罐模式：泵直抽歧管，没有储备真空——挽救能力弱、断电不保"
                  "真空。只做地面短测，时长自己控制，严禁上墙")
        worst = eng._worst_stance_travel(SPEED, 0.0, 0.0)
        if worst > cfg.climb_max_step:
            # 隐性限幅必须亮明：操作者按未缩放 SPEED 推算预期位移，会把固有
            # 缺口误当下滑/打滑去补，--sag-comp 标定被系统性带偏
            eff_v = SPEED * cfg.climb_max_step / worst
            print(f"速度口径：SPEED={SPEED:g}mm/s 直行超步幅上限（支撑相位移 "
                  f"{worst:.0f}>{cfg.climb_max_step:g}mm），引擎实际按 "
                  f"≈{eff_v:.1f}mm/s 下发；状态行与黑匣子记的都是限幅后实际值")
            log.note(f"SPEED={SPEED:g} 被步幅上限缩放：直行实际 ≈{eff_v:.2f}mm/s")
        if cfg.climb_sag_comp_mm > 0:
            # 本速度直行的每事件实际量（工作空间总账限额，见 climb.COMP_TAIL_MAX）
            half = min(worst, cfg.climb_max_step) / 2.0
            eff = min(cfg.climb_sag_comp_mm,
                      max(0.0, (COMP_TAIL_MAX - half) / 5.0))
            log.event(f"下滑补偿开启：设定 {cfg.climb_sag_comp_mm:g}mm/抬腿事件，"
                      f"本速度直行实际 {eff:g}mm")
            print(f"下滑补偿开启：设定 {cfg.climb_sag_comp_mm:g}mm/抬腿事件，"
                  f"本速度直行实际 {eff:g}mm（工作空间限额，降速可放宽）。方向口径"
                  "＝机器人头朝上贴墙（下坡=-X，随转向积分旋转），放置朝向不符时勿"
                  "用；标定从小往大，盯每周期净位移与支撑盘有无被拖歪")

        # —— 就位暂停：给操作者检查站位/气路/场地的窗口，按 p 才碰气路 ——
        print("就位暂停：确认无异常后按 p 开始全吸附启动序列（ESC×2 断电退出）")
        while True:
            k = read_key(0.1)
            if k == "p":
                at_pause = False
                # 暂停处的 ESC 单击残留必须清掉：不清的话按 p 后 2s 内主
                # 循环任意一次 ESC（含误触）就凑满双击、直接进放气退出
                last_esc = float("-inf")
                print("开始全吸附启动序列……")
                log.event("按 p：开始全吸附启动序列")
                break
            if k == "\x1b":
                if time.monotonic() - last_esc < 2.0:
                    aborted = True
                    print("\n未开始吸附，断电退出。")
                    coils_off(io)   # 上电即通电（排气位）的线圈也要收掉
                    drv.close()
                    return
                last_esc = time.monotonic()
                print("再按一次 ESC 确认退出（尚未吸附；断电后请扶稳机身）")
            # 暂停窗 = 18 舵机撑站姿 + 六线圈通电（上电即排气位）的无上限
            # 时长负载窗口：欠压切断与黑匣子不能失明（与退出窗补遥测同口径）
            now = time.monotonic()
            if now - last_status > STATUS_S:
                last_status = now
                v, c = (drv.read_voltage_v(), drv.read_current_a()) \
                    if args.mock else bot.check_power()   # 欠压直接抛异常停机
                peak_a = max(peak_a, c)
                watch.telemetry(v, c, peak_a, (0.0, 0.0, 0.0), note=" 就位暂停")

        t_wall = time.monotonic()
        while True:
            k = read_key(0)
            if k == "\x1b":
                # 双击确认：退出流程会放气，墙上等于坠落，不能被单次误触发
                if time.monotonic() - last_esc < 2.0:
                    clean_exit = True
                    break
                last_esc = time.monotonic()
                print("\n再按一次 ESC 确认退出（会放气、停在爬墙站位——墙上禁用！"
                      "墙上悬停 = 保持进程运行什么都不按）")
            elif k in ("w", "s", "a", "d", "q", "e", "i") and released_hold:
                print("\n吸盘已放开（取机窗口），不可再走动——取下后 ESC×2 退出")
            elif k in ("w", "s", "a", "d", "q", "e") \
                    and (eng.step_pending or eng.step_active):
                print("\n单步在途，走完自动停后再操作"
                      "（未开始的单步可按空格取消）")
            elif k == "w":
                vx, vy = SPEED, 0.0
            elif k == "s":
                vx, vy = -SPEED, 0.0
            elif k == "a":
                vx, vy = 0.0, SPEED
            elif k == "d":
                vx, vy = 0.0, -SPEED
            elif k == "q":
                wz = TURN
            elif k == "e":
                wz = -TURN
            elif k == " ":
                vx = vy = wz = 0.0
                if eng.step_pending:
                    eng.cancel_step()
                    step_was = False       # 别把取消误报成"单步完成"
                    print("\n已取消未开始的单步（摆动中的单步不可打断）")
                    log.event("单步取消（未开始）")
            elif k == "i":
                if vx or vy or wz:
                    print("\n单步：先按空格停（连续行走中）")
                else:
                    deny = eng.request_step(SPEED, 0.0, 0.0)
                    if deny:
                        print(f"\n单步不可用：{deny}")
                    else:
                        print(f"\n单步：轮到 {eng.step_leg}，等它的相位窗启动"
                              "（等窗最多一个周期，走完自动停）")
                        log.event(f"单步受理：{eng.step_leg} v={SPEED:g}")
            elif k == "f" and eng.frozen:
                # 解冻同时清零速度：人工处理时手还在机器旁，绝不能带着
                # 冻结前的旧速度立刻恢复行走（审核发现 #4）
                vx = vy = wz = 0.0
                print(f"\n解除冻结: {eng.frozen}（速度已清零，按 w 重新开始）")
                eng.clear_freeze()
                # 横幅比对基线同步清掉：目标仍越界时下一帧会以同一字符串
                # 复冻，不清的话横幅不再打印——用户按 f "看似成功实则无效"
                # 却毫无提示（工作空间冻结的目标不动，解冻即复冻是常态）
                last_frozen = None
            elif k == "o":
                # 取机窗口：放开全部吸盘但六足保持站立，便于整机从玻璃上拿起。
                # 双击确认口径同 ESC；每次按键都重查前置（两击间状态可能变）
                if released_hold:
                    print("\n已是放开状态——取下后 ESC×2 退出")
                else:
                    deny = hold_release_deny()
                    if deny:
                        last_o = float("-inf")
                        print(f"\n不允许放开：{deny}")
                    elif time.monotonic() - last_o < 2.0:
                        released_hold = True
                        ctl.pump_inhibit = True   # 罐模式滞环也不许再开泵
                        log.event("取机窗口：放开全部吸盘（泵禁开），六足舵机撑住")
                        for i in range(6):
                            ctl.request_release(i)
                        print("\n放开吸盘：全阀排气、泵停，六足仅舵机撑住原地。"
                              "可整机拿起；取下后 ESC×2 退出"
                              "（排气是维持态，阀线圈通电中，勿久放）")
                    else:
                        last_o = time.monotonic()
                        print("\n再按一次 o 确认放开全部吸盘（六足保持站立，"
                              "仅舵机撑住）——墙上严禁（等于坠落）")

            if k is not None:
                log.event(f"键 {k!r} → 速 {vx:g},{vy:g},{wz:g}"
                          + ("（已放开）" if released_hold else ""))

            try:
                bot.move_feet(eng.update(dt, vx, vy, wz))
            except WorkspaceError as e:
                # 降落伞：任何原因（极限速度组合、补偿边角、未来改动）导致
                # 足端目标出工作空间时，冻结悬停而不是让异常炸掉控制环——
                # 墙上进程崩溃 = 失控。本帧不下发，舵机保持上一帧姿态；
                # 冻结后引擎目标全保持，本异常会逐帧重现并被逐帧吞掉
                if not eng.frozen:
                    eng.frozen = f"足端目标出工作空间（{e}）——停走后 ESC×2 退出"
            watch.poll()   # 状态跳变（相位/吸附/泵阀/冻结/漏气）落黑匣子

            step_now = eng.step_pending or eng.step_active
            if step_was and not step_now:
                print(f"\n单步完成，下一条：{eng.step_leg}（再按 i）")
                log.event(f"单步完成，下一条 {eng.step_leg}")
            step_was = step_now

            if eng.started and not was_started:
                was_started = True
                # 清掉启动期间误触存下的速度：吸附完成瞬间不许自己开走
                vx = vy = wz = 0.0
                print("\n✓ 六足吸附完成，遥控就绪：w/s 前后  a/d 左右  "
                      "q/e 转向  i 单步  空格停  f 解冻  o×2 放吸盘取机  "
                      "ESC×2 退出\n"
                      "  （速度指令是 0 时不抬腿——按 w 才开始走；爬墙步态"
                      "宁慢勿快，每步约 4s 属正常）")

            if eng.frozen != last_frozen:
                last_frozen = eng.frozen
                if eng.frozen:
                    print(f"\a\n⚠⚠⚠ 全机冻结: {eng.frozen} —— 处理后按 f 继续\n")

            now = time.monotonic()
            if now - last_status > STATUS_S:
                last_status = now
                v, c = (drv.read_voltage_v(), drv.read_current_a()) \
                    if args.mock else bot.check_power()   # 欠压直接抛异常停机
                peak_a = max(peak_a, c)
                tag = (" 干跑" if args.dry else "") + \
                      (" 已放开" if released_hold else "")
                # 显示/落盘用 eng.cmd = 步幅限幅后的实际下发速度，不是键盘
                # 原始指令——日志速度必须等于下发速度（--sag-comp 标定口径）
                print("\r" + status_line(eng, ctl, v, c, peak_a,
                                         eng.cmd, tag) + "  ",
                      end="", flush=True)
                watch.telemetry(v, c, peak_a, eng.cmd, note=tag)
            lag = time.monotonic() - t_wall
            if lag > 5 * dt:
                # 主环卡顿（I2C 拖延/串口超时/系统卡）是死机前兆，必须留痕
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
            # 暂停/站起中 Ctrl-C 或异常：未吸附，舵机保持使能站立；但六阀
            # 线圈从上电 claim 起就在排气位（通电），进程退出后引脚保持
            # 电平会一直发热（08-17 同款）——此刻无任何足吸附，断线圈安全。
            # 同阶段的 ESC×2 中止路径早就这么收，这里补齐同款收尾
            coils_off(io)
            log.event("中断：就位暂停处（未吸附，阀线圈已断电，舵机保持使能）")
            print("\n在就位暂停处中断：未吸附，阀线圈已断电。"
                  "舵机断电请直接关电源或跑 --release。")
        elif clean_exit:
            log.event("退出序列：停走 → 停泵 → 逐足串行放气（通→断，0.2s 间隔）"
                      " → 舵机断电（不回站姿）")
            print("\n退出：停走 -> 停泵 -> 逐足串行放气 -> 舵机断电"
                  "（停在爬墙站位，约 5s；Ctrl-C 已屏蔽，等它跑完）")
            # 序列全程屏蔽 Ctrl-C：这里已在 finally 里，KeyboardInterrupt
            # 打断会跳过 coils_off/drv.close，正在排气的阀停在通电电平
            # （08-17 线圈发热事故同款终态）。异常收尾由内层 finally 必达；
            # 真卡死还有 SIGTERM/关电源
            prev_int = signal.signal(signal.SIGINT, signal.SIG_IGN)
            try:
                # 先停泵再放气：无罐模式泵虽会在首个 update 自停，但只先于阀
                # 通电几微秒（电气上等于同时换手）；罐模式滞环更会贯穿退出窗口
                # 给将废弃的罐补压。统一用 pump_inhibit 收口（与 'oo' 取机同
                # 机制，滞环不许再开），垫 0.3s 让母线从泵负载缓过来
                ctl.pump_inhibit = True
                io.set_pump(False)
                if not args.mock:
                    time.sleep(0.3)
                # 逐足串行放气（08-18 掉电死机对策，事故正发生在 ESC×2 之后）：
                # 一次只给一个阀通电（通电=排气），压力确认 RELEASED 即断该阀
                # 电，隔 0.2s 再下一足——阀侧峰值从六线圈同刻阶跃 ~25W 降为单
                # 线圈 ~4W，12V 轨不再吃大阶跃。断电后该足翻回通罐位，残余真空
                # 可能把盘轻微重新吸住（与原 coils_off 终态一致，关 12V 或
                # --release 放开）。终态停在爬墙站位（回站姿 glide 已删：拖盘
                # 46mm + 第二负载高原，ea46bab）。⚠ 'oo' 取机不适用本法：取机
                # 需要排气维持态撑到人把整机拿走。退出期继续记电压（TLM）
                t_tlm = float("-inf")
                adh_dead = False

                def _exit_tick():
                    nonlocal t_tlm, peak_a, adh_dead
                    try:
                        ctl.update(dt)     # 放气驱动 + 压力确认 + 泵禁开
                        watch.poll()
                    except Exception as e:
                        # "读失败绝不打断收尾"必须包住状态机本身，不只电压
                        # 读取——I2C 降级正是黑匣子要覆盖的场景。VENTING 分
                        # 支先 set_valve 再读压确认，异常只丢确认不丢排气，
                        # 退化成纯计时放气（单足 1.5s 预算兜底）
                        if not adh_dead:
                            adh_dead = True
                            log.event("⚠ 退出期吸附状态机异常（I2C 降级？），"
                                      f"退化纯计时放气: {e}")
                    if time.monotonic() - t_tlm > STATUS_S:
                        t_tlm = time.monotonic()
                        try:
                            # 直读不走 check_power：善后期欠压只记录，
                            # 绝不抛异常打断收尾
                            v, c = drv.read_voltage_v(), drv.read_current_a()
                            peak_a = max(peak_a, c)
                            watch.telemetry(v, c, peak_a, (0, 0, 0),
                                            note=" 退出放气")
                        except Exception as e:
                            log.event(f"⚠ 退出期电压读取失败，停采: {e}")
                            t_tlm = float("inf")   # 读不到就别再试，别拖慢善后

                for i in range(6):
                    # 任意状态一律转入放气：request_release 只认 ATTACHED——
                    # 从冻结（FAULT）或抽气中途（SUCKING）退出的足也必须真
                    # 排气（旧口径白烧预算、落"排气未确认"假日志，无罐时
                    # SUCKING 足还会被状态机跨时隙反手通电且从未放气）
                    ctl.force_release(i)
                    for _ in range(int(1.5 / dt)):   # 单足预算 1.5s（正常 ~0.5s）
                        _exit_tick()
                        if not args.mock:
                            time.sleep(dt)
                        if ctl.state[i] == FootState.RELEASED:
                            break
                    else:
                        # 弃疗必须先把状态迁出 VENTING 再断阀电：否则后续
                        # _exit_tick 的 VENTING 分支每周期会把刚断电的阀反手
                        # 重新通电，日志"线圈照断"与实际电平相反
                        ctl.abandon_release(i)
                        log.event(f"⚠ {LEG_NAMES[i]} 排气 1.5s 未确认"
                                  "（堵/传感器漂移？），线圈照断继续")
                    io.set_valve(i, True)            # 确认排空即断该阀电（通→断）
                    for _ in range(int(0.2 / dt)):   # 足间 0.2s 间隔
                        _exit_tick()
                        if not args.mock:
                            time.sleep(dt)
            finally:
                coils_off(io)      # 幂等收尾：全线圈确认断电 + 泵停——异常也必达
                log.event("退出：阀线圈已断电（coils_off）")
                drv.close()
                signal.signal(signal.SIGINT, prev_int)
            print("完成（阀线圈已断电）。")
            log.event("退出序列完成（舵机断电）")
        else:
            # Ctrl-C/异常：不主动放气（善后跑 --release）。
            # 进程退出后 GPIO 终态未定论：2026-08-17 实测偏向"保持最后驱动
            # 电平"（ESC 退出后拉高的线圈仍通电，故加了 coils_off 收尾），
            # 但"释放回默认上下拉"在别的内核/重启后也可能出现。墙上中断
            # 一律按最坏情形对待（可能泄罐压/可能线圈挂着耗电）：优先保持
            # 进程冻结悬停，不到万不得已不杀进程（审核发现 #2，系统级）。
            log.event(f"中断：不放气退出，冻结={eng.frozen or '无'}")
            print(f"\n中断：不放气退出。冻结: {eng.frozen or '无'}；"
                  f"善后请跑 --release。")
            if not args.no_tank:   # 无罐模式没有罐压可泄、也禁止上墙
                print("⚠ 若有腿悬空：进程退出后阀终态不可控、罐压可能泄漏"
                      "——墙上请确认安全绳受力。")
        log.close("正常退出" if (clean_exit or aborted) else "中断退出")


if __name__ == "__main__":
    main()
