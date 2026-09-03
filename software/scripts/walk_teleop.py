#!/usr/bin/env python3
"""键盘遥控行走（SSH 到树莓派运行）。

  w/s  前进/后退      a/d  左移/右移
  q/e  左转/右转      空格 停（踏步中=冻结/继续）
  1/2  三角/波浪步态   v    切阀策略 auto→on→off（对照用）
  m    原地踏步开关    ESC  退出

用法: python walk_teleop.py [--port /dev/ttyACM0] [--mock] [--vent auto|on|off]
                            [--march-hold 5]

原地踏步 m（身体不动、只抬落脚）：按当前步态分组轮流抬——三角=对角三只一组
（两组），波浪=一次一腿（六组）。一组的节拍两头都停满 --march-hold 秒（默认 5）：
    抬起 → 悬空停 5s → 落下 → 落地站定 5s → 轮到下一组
慢节拍是为了看清楚：抬腿幅度够不够、悬空 5s 里腿会不会往下掉、脚有没有被被动
真空吸住、落地后身体沉多少、站定 5s 里吸盘会不会自己攒出真空。
  空格   冻结/继续：时钟停住=当前姿势原样定格，**抬起过程中按也停在半空不动**
  1/2    换步态=换分组，踏步节拍重开
  m/方向键  退出踏步（抬着的脚先按落下同速放回站位，不砸地）
踏步全程六阀通电排气（吸盘通大气）：踏步按"走动"处理，悬空停/站定停/冻结都不
断电——身体没走不等于脚不抬，断了电脚就被被动真空吸住。按 m 时若阀策略是 off，
自动切 on 并提示。代价是六线圈≈25W 持续发热，长时间踏步或长时间冻结留意阀温和
舵机温度；想看脚被吸住的对照，踏步中按 v 切回 off。

阀策略 --vent（默认 auto）：地面行走不吸附，但本机阀断电=通罐位，吸盘经每足
单向阀接歧管——身体一压把盘里空气挤进歧管，抬腿时单向阀不让空气回流，盘内
形成被动真空把脚吸在地上（09-02 实机：装气路后抬腿幅度极低、吸盘不离地）。
  auto  站起时和走动时六阀通电（排气位，吸盘通大气），停走落稳 0.5s 后断电，
        站着不动不耗六线圈≈25W。起步时线圈按足串行上电约 1s，**通电完成前
        脚不抬**（状态行显示"排气中"），之后才开始走
  on    整段常通（对照/兜底）
  off   不碰阀 GPIO/I2C（装气路前旧行为，对照：脚会被吸住）
运行中按 v 轮换三种策略。退出时先断舵机电再断六线圈；线圈断电后吸盘回到
通罐位，搬机时若脚被轻微吸住属正常，关 12V 即放开。
"""
import argparse
import select
import sys
import termios
import time
import tty

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver, TRIPOD, WAVE
from hexapod.adhesion import GroundVent, MockVacuumIO
from hexapod.gait import GaitEngine, MarchEngine

SPEED = 40.0    # mm/s
TURN = 0.3      # rad/s
POWER_PRINT_S = 0.5   # 电压/电流状态行刷新间隔（原地覆写，不刷屏）


def read_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--vent", choices=GroundVent.MODES, default="auto",
                    help="六阀策略：auto=站起/走动通电排气、静止断电（默认）；"
                         "on=常通；off=不碰阀（脚会被被动真空吸住，对照用）。运行中 v 轮换")
    ap.add_argument("--march-hold", type=float, default=5.0,
                    help="原地踏步（m 键）抬到顶悬空停、落地站定各停多少秒，默认 5")
    args = ap.parse_args()
    mode = args.vent

    # 阀先于舵机：站起时吸盘就已通大气，压下去不攒被动真空
    vent = GroundVent(io_factory=(lambda: MockVacuumIO(6)) if args.mock else None,
                      stagger_s=0.0 if args.mock else 0.2)
    if mode != "off":
        vent.set(True)
        print(f"阀策略 {mode}：站起前六阀已拉到排气位（吸盘通大气）")
    else:
        print("阀策略 off：不碰阀（通罐位），吸盘可能被被动真空吸在地上，按 v 切策略对照")

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    bot = Hexapod(drv)
    # 缓慢站起：使能前先发蹲姿（离断电趴姿最近，使能跳变小），再慢滑到站姿
    bot.move_feet(bot.crouch_feet())
    drv.enable(True)
    time.sleep(0 if args.mock else 1.0)
    bot.stand(duration=4.0)

    old = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    vx = vy = wz = 0.0
    t = 0.0
    dt = 1.0 / bot.cfg.update_hz
    last_power_check = 0.0
    peak_a = 0.0
    march = None          # 原地踏步引擎（None=不踏步）
    march_t = 0.0         # 踏步已走时；阀没通电完时不推进，免得空跳过一组
    march_pause = False   # 踏步冻结：时钟不走=当前姿势原样保持（抬着的脚就停在半空）
    print("遥控就绪 (w/s/a/d/q/e, 空格停/踏步中冻结, 1/2 步态, v 阀策略, "
          "m 原地踏步, ESC 退出)")
    try:
        while True:
            k = read_key(0)
            if k == "\x1b":
                break
            elif k == "w":
                vx, vy = SPEED, 0
            elif k == "s":
                vx, vy = -SPEED, 0
            elif k == "a":
                vx, vy = 0, SPEED
            elif k == "d":
                vx, vy = 0, -SPEED
            elif k == "q":
                wz = TURN
            elif k == "e":
                wz = -TURN
            elif k == " ":
                vx = vy = wz = 0.0      # 踏步中本就是 0；空格在下面分流成冻结/继续
            elif k == "1":
                bot.engine = GaitEngine(bot.cfg, TRIPOD)
            elif k == "2":
                bot.engine = GaitEngine(bot.cfg, WAVE)
            elif k == "v":
                mode = GroundVent.MODES[(GroundVent.MODES.index(mode) + 1) % len(GroundVent.MODES)]
                print(f"\n阀策略 → {mode}")

            # 原地踏步：m 开关；踏步中空格=冻结/继续（不是"停"），方向键=交还遥控
            if k == "m" or (march is not None and k in ("w", "s", "a", "d", "q", "e")):
                if march is None:
                    vx = vy = wz = 0.0
                    march = MarchEngine(bot.cfg, bot.engine.gait, hold_s=args.march_hold)
                    march_t = 0.0
                    march_pause = False
                    if mode == "off":   # 踏步全程要六阀通电排气，off 不通电=脚被吸住
                        mode = "on"
                        print("\n阀策略 off → on：踏步要六阀全程通电排气"
                              "（想看被吸住的对照，踏步中按 v 切回 off）")
                    print(f"\n原地踏步：{bot.engine.gait.name} 分 {len(march.groups)} 组，"
                          f"抬起→悬空停 {march.hold_s:.1f}s→落下→站定 {march.hold_s:.1f}s→下一组"
                          f"（空格冻结当前姿势，m 或方向键退出）")
                else:
                    down_s = march.lift_s       # 抬着的脚按落下同速放回，别一拍砸下去
                    march = None
                    march_pause = False
                    bot.glide_to(dict(bot.engine.default_feet), down_s)
                    print("\n退出原地踏步" if k == "m" else "\n退出原地踏步（收到方向指令）")
            elif march is not None and k == " ":
                march_pause = not march_pause   # 时钟停住=姿势原样定格，半空中也定得住
                print("\n踏步冻结：保持当前姿势不动（空格继续）" if march_pause
                      else "\n踏步继续")
            elif march is not None and k in ("1", "2"):   # 换步态=换分组，节拍重开
                march = MarchEngine(bot.cfg, bot.engine.gait, hold_s=args.march_hold)
                march_t = 0.0
                march_pause = False
                print(f"\n踏步换 {bot.engine.gait.name}：{len(march.groups)} 组，节拍重开")

            moving = bool(vx or vy or wz)
            # auto：走动/踏步（含悬空停、站定停、冻结）全程通电、只有纯站着不动才断电；
            # 线圈没全部通电前脚不抬（起步原地等 ≈1s，踏步时钟也不推进，不空跳一组）
            if vent.drive(mode, moving or march is not None):
                if march is not None:
                    bot.move_feet(march.foot_targets(march_t))
                    if not march_pause:
                        march_t += dt
                else:
                    bot.move_feet(bot.engine.foot_targets(t, vx, vy, wz))
            else:
                bot.move_feet(bot.engine.foot_targets(t, 0.0, 0.0, 0.0))
            if t - last_power_check > POWER_PRINT_S:
                v, c = bot.check_power()  # 欠压直接抛异常停机
                peak_a = max(peak_a, c)
                line = (f"电压 {v:.2f}V  电流 {c:5.2f}A  峰值 {peak_a:5.2f}A  "
                        f"阀[{mode}] {vent.state_text()}")
                if march is not None:
                    line += (f"  踏步[{'冻结 ' if march_pause else ''}"
                             f"{march.state_text(march_t)}]")
                print("\r" + line.ljust(78), end="", flush=True)  # 补空格抹掉上一行残留
                last_power_check = t
            time.sleep(dt)
            t += dt
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        try:
            drv.close()
        finally:
            vent.close()    # 六线圈断电再释放，绝不拉高着退（阀会一直通电发热）
        print("\n已断舵机电、阀线圈已断电，退出。")


if __name__ == "__main__":
    main()
