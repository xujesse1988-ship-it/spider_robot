#!/usr/bin/env python3
"""P4 步态-吸附联调：Mock 干跑 / 实机架空 / 地面玻璃板 / 上墙 共用入口。

流程：站姿 -> 启动全吸附（六足压入、逐足抽气确认）-> 键盘遥控 CLIMB 步态。
  w/s 前进/后退   a/d 左移/右移   q/e 左转/右转   空格 停
  f   解除冻结（人工处理完报警后按）        ESC 安全退出（停走->放气->站姿->断电）

用法:
  python climb_walk.py --mock                # 无硬件干跑（Mock 吸盘必吸上）
  python climb_walk.py --mock --air          # 干跑架空路径（Mock 吸盘全漏）
  python climb_walk.py --air                 # 实机架空：阀/泵真动作，吸不上不停走
  python climb_walk.py                       # 地面玻璃板 / 上墙（全链路）
  python climb_walk.py --release             # 善后：全阀放气+回站姿

安全（P4-GUIDE）：
  - Ctrl-C 中断＝冻结不放气不断使能（吸附中的机器人不能掉），善后跑 --release
  - 上墙全程安全绳；--air 旁路只用于架空，上墙严禁
  - 电压/电流状态行常显：墙上五足持续剪切载荷 ~3A 级是常态，盯"吸附后总电流
    回落基线"判据（不回落 = 该腿 press_delta 没吃干净，舵机在和真空对抗）
"""
import argparse
import select
import sys
import termios
import time
import tty
from dataclasses import replace

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver
from hexapod.adhesion import AdhesionController, MockVacuumIO
from hexapod.climb import ClimbEngine
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES

SPEED = 15.0    # mm/s（爬墙宁慢勿快；跑顺再提）
TURN = 0.1      # rad/s
STATUS_S = 0.5  # 状态行刷新间隔

PHASE_CH = {"stance": "·", "lift": "L", "transfer": "T", "descend": "D",
            "press": "P", "retry": "R", "wait": "W"}
ADH_CH = {"released": "r", "pressing": "p", "sucking": "s",
          "attached": "A", "venting": "v", "fault": "F"}


def read_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return None
    ch = sys.stdin.read(1)
    if ch == "\x1b":
        # 方向键等发的是转义序列（ESC [ X）：把尾巴吞掉、不当 ESC 用。
        # 误触方向键绝不能触发"放气退出"（墙上等于坠落）。
        r2, _, _ = select.select([sys.stdin], [], [], 0.01)
        if r2:
            sys.stdin.read(2)
            return None
    return ch


def status_line(eng, ctl, io, v, c, peak):
    legs = " ".join(
        f"{n}{PHASE_CH[ph]}{'!' if leak else ADH_CH[adh]}"
        for n, (ph, adh, leak) in eng.status().items())
    tank = io.read_tank_kpa()
    head = "启动" if not eng.started else f"t={eng.t:5.1f}"
    tf = " 罐压失效!" if ctl.tank_fault else ""
    return (f"[{head}] {legs}  罐 {tank:6.1f}kPa{tf}  "
            f"{v:.2f}V {c:5.2f}A 峰 {peak:5.2f}A")


def release_all(io, ctl, bot):
    """善后：全阀断电放气，回站姿。地面用；墙上严禁（会坠落）。"""
    for i in range(6):
        io.set_valve(i, False)
    io.set_pump(False)
    time.sleep(1.0)
    bot.stand(2.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    ap.add_argument("--air", action="store_true",
                    help="架空模式：吸不上也继续走（旁路互锁与罐压冻结），上墙严禁")
    ap.add_argument("--cycle", type=float, default=DEFAULT_CONFIG.climb_cycle_time,
                    help="步态周期 s（先慢后提速）")
    ap.add_argument("--release", action="store_true", help="只做全放气+回站姿")
    args = ap.parse_args()

    cfg = replace(DEFAULT_CONFIG, climb_cycle_time=args.cycle)
    if args.air:
        cfg = replace(cfg, max_attach_retry=1)   # 架空必 FAULT，少陪跑几轮重试

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    if args.mock:
        io = MockVacuumIO(6)
        if args.air:
            io.sealed = [False] * 6              # 干跑架空路径
    else:
        from hexapod.adhesion import Pi5VacuumIO
        io = Pi5VacuumIO(6)
    # --air 时罐压传感器可能未接：泵降级为"有脚在抽就开"，不被 tank_fault 停死
    ctl = AdhesionController(io, pump_without_tank=args.air)
    bot = Hexapod(drv, cfg)
    eng = ClimbEngine(cfg, ctl, air_mode=args.air)

    if args.release:
        # 顺序是安全关键：先放气（吸着的脚不能被舵机硬拉），再上舵机回站姿
        for i in range(6):
            io.set_valve(i, False)
        io.set_pump(False)
        time.sleep(0 if args.mock else 1.5)
        bot.move_feet(bot.engine.default_feet)
        drv.enable(True)
        time.sleep(0 if args.mock else 1.0)
        bot.stand(2.0)
        drv.close()
        print("已全放气并回站姿。")
        return

    bot.move_feet(bot.engine.default_feet)       # 先发好姿态再使能，防上电乱跳
    drv.enable(True)
    time.sleep(0 if args.mock else 1.0)

    bot.stand()
    # 过渡到爬墙站位：ClimbEngine 的站位是"压入位吸盘轴⊥面"的解（约 reach 176），
    # 不是地面步态的 130——不先滑过去，引擎第一拍足端目标会跳变几十毫米
    bot.glide_to(dict(eng.default_feet), 2.0)
    print("爬墙站位就位（吸盘轴⊥面），开始全吸附启动序列……")
    if args.air:
        print("⚠ 架空模式：吸附失败不冻结，互锁旁路——上墙严禁本模式")

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = vy = wz = 0.0
    dt = 1.0 / cfg.update_hz
    last_status = 0.0
    peak_a = 0.0
    last_frozen = None
    clean_exit = False
    last_esc = 0.0
    try:
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
                      "悬停冻结请用 Ctrl-C）")
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
                print(f"\n解除冻结: {eng.frozen}")
                eng.clear_freeze()

            bot.move_feet(eng.update(dt, vx, vy, wz))

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
                print("\r" + status_line(eng, ctl, io, v, c, peak_a) + "  ",
                      end="", flush=True)
            time.sleep(max(0.0, dt - (time.time() - t_wall)))
            t_wall = time.time()
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        if clean_exit:
            print("\n退出：停走 -> 放气 -> 站姿 -> 断电")
            # 逐足正常放气（VENTING 流程），吸附中的先释放
            for i in range(6):
                ctl.request_release(i)
            for _ in range(int(3.0 / dt)):
                ctl.update(dt)
                if not args.mock:
                    time.sleep(dt)
            release_all(io, ctl, bot)
            drv.close()
            print("完成。")
        else:
            # Ctrl-C/异常：不放气不断使能（防坠落），善后跑 --release
            print(f"\n中断：阀/舵机保持当前状态（防坠落）。"
                  f"冻结: {eng.frozen or '无'}；善后请跑 --release。")


if __name__ == "__main__":
    main()
