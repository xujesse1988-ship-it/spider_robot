#!/usr/bin/env python3
"""P4 步态-吸附联调：Mock 干跑 / 实机架空 / 地面玻璃板 / 上墙 共用入口。

流程：缓慢站起（直达爬墙站位）-> 就位暂停（按 p 继续）-> 全吸附启动序列
（六足压入、逐足抽气确认）-> 键盘遥控 CLIMB 步态。
  p   就位暂停后开始全吸附启动序列
  w/s 前进/后退   a/d 左移/右移   q/e 左转/右转   空格 停
  f   解除冻结（人工处理完报警后按）        ESC 安全退出（停走->放气->站姿->断电）
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
  python climb_walk.py --release             # 善后：全阀放气+回站姿

安全（P4-GUIDE）：
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
import sys
import termios
import time
import tty
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver
from hexapod.adhesion import AdhesionController, MockVacuumIO
from hexapod.climb import ClimbEngine, LegPhase
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES

SPEED = 15.0    # mm/s（爬墙宁慢勿快；跑顺再提）
TURN = 0.1      # rad/s
STATUS_S = 0.5  # 状态行刷新间隔

PHASE_CH = {"stance": "·", "lift": "L", "transfer": "T", "descend": "D",
            "press": "P", "retry": "R", "wait": "W"}
ADH_CH = {"released": "r", "pressing": "p", "sucking": "s",
          "attached": "A", "venting": "v", "fault": "F"}


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


def status_line(eng, ctl, io, v, c, peak, cmd, tag=""):
    legs = " ".join(
        f"{n}{PHASE_CH[ph]}{'!' if leak else ADH_CH[adh]}"
        for n, (ph, adh, leak) in eng.status().items())
    if ctl.tankless:
        tank_txt = "罐 无罐"          # 无罐模式不读罐压传感器（读了也是悬空假数）
    else:
        tf = " 罐压失效!" if ctl.tank_fault else ""
        tank_txt = f"罐 {io.read_tank_kpa():6.1f}kPa{tf}"
    head = ("启动" if not eng.started else f"t={eng.t:5.1f}") + tag
    vx, vy, wz = cmd
    sp = "停" if not (vx or vy or wz) else f"{vx:+.0f}/{vy:+.0f}/{wz:+.2f}"
    return (f"[{head}] {legs}  速 {sp}  {tank_txt}  "
            f"{v:.2f}V {c:5.2f}A 峰 {peak:5.2f}A")


def release_all(io, ctl, bot):
    """善后：全阀排气（通电位），回站姿。地面用；墙上严禁（会坠落）。"""
    for i in range(6):
        io.set_valve(i, False)
    io.set_pump(False)
    time.sleep(1.0)
    bot.stand(2.0)


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
    ap.add_argument("--release", action="store_true", help="只做全放气+回站姿")
    args = ap.parse_args()
    if args.cycle < 1.0:
        ap.error(f"--cycle {args.cycle} 非法：步态周期至少 1s（0/负值会在六足"
                 "吸附完成后才除零崩溃，审核发现 #8）")
    if not args.release and not sys.stdin.isatty():
        # TTY 检查必须在碰任何硬件之前：否则真机先上电站立、再在 termios
        # 处崩溃断电瘫倒（审核发现 #5）。--release 无需键盘，放行。
        sys.exit("需要交互终端（ssh 加 -t；勿用 nohup/管道跑本脚本）")

    cfg = replace(DEFAULT_CONFIG, climb_cycle_time=args.cycle)
    if args.air:
        cfg = replace(cfg, max_attach_retry=1)   # 架空必 FAULT，少陪跑几轮重试

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

    if args.release:
        # 顺序是安全关键：先放气（吸着的脚不能被舵机硬拉），再上舵机回站姿
        for i in range(6):
            io.set_valve(i, False)
        io.set_pump(False)
        real_pneu = not (args.mock or args.dry)
        if real_pneu:
            from hexapod.adhesion import RELEASE_KPA
            t_end = time.time() + 5.0        # 放气用读数确认，不靠开环 sleep
            while time.time() < t_end:
                if all(io.read_foot_kpa(i) >= RELEASE_KPA for i in range(6)):
                    break
                time.sleep(0.2)
            else:
                print("⚠ 5s 后仍有足压未回大气（排气不畅？），继续回站姿需谨慎")
        # 使能瞬间是硬跳：预置到爬墙站位（离中断时的姿态最近），再缓动回地面站姿
        bot.move_feet(eng.default_feet)
        drv.enable(True)
        time.sleep(0 if args.mock else 1.0)
        bot.stand(3.0)
        coils_off(io)          # 排气是维持态：退出前必须把线圈全部断电
        drv.close()
        print("已全放气并回站姿（阀线圈已断电）。")
        return

    # 缓慢站起，一步到位进爬墙站位：蹲姿直接用爬墙站位（吸盘轴⊥面的解，
    # 约 reach 176）的 XY，从蹲到站是纯竖直上升——吸盘落地后不横拖。
    # （旧流程"站姿130→爬墙站位176"的 glide 会拖着承重吸盘划 46mm）
    bot.move_feet(bot.crouch_feet(feet=eng.default_feet))
    drv.enable(True)
    time.sleep(0 if args.mock else 1.0)
    print("缓慢站起（竖直升至爬墙站位，吸盘轴⊥面）……")
    bot.glide_to(dict(eng.default_feet), 4.0)
    print("爬墙站位就位。")
    if args.air:
        print("⚠ 架空模式：吸附失败不冻结，互锁旁路——上墙严禁本模式")
    if args.dry:
        print("⚠ 干跑模式：气路是仿真的，阀泵不会动，状态行气压是假数——上墙严禁")
    if args.no_tank:
        print("⚠ 无罐模式：泵直抽歧管，没有储备真空——挽救能力弱、断电不保真空。"
              "只做地面短测，时长自己控制，严禁上墙")

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = vy = wz = 0.0
    dt = 1.0 / cfg.update_hz
    last_status = 0.0
    peak_a = 0.0
    last_frozen = None
    clean_exit = False
    last_esc = 0.0
    last_o = 0.0
    released_hold = False   # 'oo' 取机窗口：吸盘已全放开，六足仅舵机撑住
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
        if any(p != LegPhase.STANCE for p in eng.phase_of.values()):
            return "有腿未回支撑相（等本步收尾再按）"
        return None
    try:
        # —— 就位暂停：给操作者检查站位/气路/场地的窗口，按 p 才碰气路 ——
        print("就位暂停：确认无异常后按 p 开始全吸附启动序列（ESC×2 断电退出）")
        while True:
            k = read_key(0.1)
            if k == "p":
                at_pause = False
                print("开始全吸附启动序列……")
                break
            if k == "\x1b":
                if time.time() - last_esc < 2.0:
                    aborted = True
                    print("\n未开始吸附，断电退出。")
                    coils_off(io)   # 上电即通电（排气位）的线圈也要收掉
                    drv.close()
                    return
                last_esc = time.time()
                print("再按一次 ESC 确认退出（尚未吸附；断电后请扶稳机身）")

        t_wall = time.time()
        while True:
            k = read_key(0)
            if k == "\x1b":
                # 双击确认：退出流程会放气，墙上等于坠落，不能被单次误触发
                if time.time() - last_esc < 2.0:
                    clean_exit = True
                    break
                last_esc = time.time()
                print("\n再按一次 ESC 确认退出（会放气回站姿——墙上禁用！"
                      "墙上悬停 = 保持进程运行什么都不按）")
            elif k in ("w", "s", "a", "d", "q", "e") and released_hold:
                print("\n吸盘已放开（取机窗口），不可再走动——取下后 ESC×2 退出")
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
            elif k == "f" and eng.frozen:
                # 解冻同时清零速度：人工处理时手还在机器旁，绝不能带着
                # 冻结前的旧速度立刻恢复行走（审核发现 #4）
                vx = vy = wz = 0.0
                print(f"\n解除冻结: {eng.frozen}（速度已清零，按 w 重新开始）")
                eng.clear_freeze()
            elif k == "o":
                # 取机窗口：放开全部吸盘但六足保持站立，便于整机从玻璃上拿起。
                # 双击确认口径同 ESC；每次按键都重查前置（两击间状态可能变）
                if released_hold:
                    print("\n已是放开状态——取下后 ESC×2 退出")
                else:
                    deny = hold_release_deny()
                    if deny:
                        last_o = 0.0
                        print(f"\n不允许放开：{deny}")
                    elif time.time() - last_o < 2.0:
                        released_hold = True
                        ctl.pump_inhibit = True   # 罐模式滞环也不许再开泵
                        for i in range(6):
                            ctl.request_release(i)
                        print("\n放开吸盘：全阀排气、泵停，六足仅舵机撑住原地。"
                              "可整机拿起；取下后 ESC×2 退出"
                              "（排气是维持态，阀线圈通电中，勿久放）")
                    else:
                        last_o = time.time()
                        print("\n再按一次 o 确认放开全部吸盘（六足保持站立，"
                              "仅舵机撑住）——墙上严禁（等于坠落）")

            bot.move_feet(eng.update(dt, vx, vy, wz))

            if eng.started and not was_started:
                was_started = True
                # 清掉启动期间误触存下的速度：吸附完成瞬间不许自己开走
                vx = vy = wz = 0.0
                print("\n✓ 六足吸附完成，遥控就绪：w/s 前后  a/d 左右  "
                      "q/e 转向  空格停  f 解冻  o×2 放吸盘取机  ESC×2 退出\n"
                      "  （速度指令是 0 时不抬腿——按 w 才开始走；爬墙步态"
                      "宁慢勿快，每步约 4s 属正常）")

            if eng.frozen != last_frozen:
                last_frozen = eng.frozen
                if eng.frozen:
                    print(f"\a\n⚠⚠⚠ 全机冻结: {eng.frozen} —— 处理后按 f 继续\n")

            now = time.time()
            if now - last_status > STATUS_S:
                last_status = now
                v, c = (drv.read_voltage_v(), drv.read_current_a()) \
                    if args.mock else bot.check_power()   # 欠压直接抛异常停机
                peak_a = max(peak_a, c)
                tag = (" 干跑" if args.dry else "") + \
                      (" 已放开" if released_hold else "")
                print("\r" + status_line(eng, ctl, io, v, c, peak_a,
                                         (vx, vy, wz), tag) + "  ",
                      end="", flush=True)
            time.sleep(max(0.0, dt - (time.time() - t_wall)))
            t_wall = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if aborted:
            pass          # 暂停处确认退出：已断电、已提示，无需善后
        elif at_pause:
            # 暂停处 Ctrl-C：未吸附，舵机保持使能站立
            print("\n在就位暂停处中断：未吸附。断电请直接关电源或跑 --release。")
        elif clean_exit:
            print("\n退出：停走 -> 放气 -> 站姿 -> 线圈断电 -> 舵机断电")
            # 逐足正常放气（VENTING 流程），吸附中的先释放
            for i in range(6):
                ctl.request_release(i)
            for _ in range(int(3.0 / dt)):
                ctl.update(dt)
                if not args.mock:
                    time.sleep(dt)
            release_all(io, ctl, bot)
            coils_off(io)      # 排气是维持态：退出前必须把线圈全部断电
            drv.close()
            print("完成（阀线圈已断电）。")
        else:
            # Ctrl-C/异常：不主动放气（善后跑 --release）。
            # 进程退出后 GPIO 终态未定论：2026-08-17 实测偏向"保持最后驱动
            # 电平"（ESC 退出后拉高的线圈仍通电，故加了 coils_off 收尾），
            # 但"释放回默认上下拉"在别的内核/重启后也可能出现。墙上中断
            # 一律按最坏情形对待（可能泄罐压/可能线圈挂着耗电）：优先保持
            # 进程冻结悬停，不到万不得已不杀进程（审核发现 #2，系统级）。
            print(f"\n中断：不放气退出。冻结: {eng.frozen or '无'}；"
                  f"善后请跑 --release。")
            if not args.no_tank:   # 无罐模式没有罐压可泄、也禁止上墙
                print("⚠ 若有腿悬空：进程退出后阀终态不可控、罐压可能泄漏"
                      "——墙上请确认安全绳受力。")


if __name__ == "__main__":
    main()
