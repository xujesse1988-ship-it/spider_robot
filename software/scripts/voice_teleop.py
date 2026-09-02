#!/usr/bin/env python3
"""语音遥控行走（树莓派 + ReSpeaker Lite USB 麦克风/喇叭板；键盘照旧可用）。

  说"小蜘蛛"唤醒 → 听令窗口内说指令，可连说：
    前进/后退/左移/右移/左转/右转 [N 秒 | N 步 | 一直] [快点 | 慢点]
    站起来 / 趴下 / 三角步态 / 波浪步态 / 电压多少 / 你好 / 退出（要再说"确定"）
  "停下 / 停止 / 别动"不用唤醒，随时生效（流式关键词急停，~0.3s）
  键盘（有终端时）：w/s/a/d/q/e 动，空格停，1/2 步态，ESC 退出，同 walk_teleop

安全边界：
  · 每条移动指令都有时长：默认 --default-secs，上限 --max-secs（"一直"=上限），
    到点自停；没听懂的句子不动；退出要二次确认；欠压照旧抛异常断舵机电。
  · 语音引擎线程只产事件；舵机/步态全部在主线程，和 walk_teleop 是同一个环。
  · 机器人自己说话期间默认不听麦克风（防听见自己说的"停"），TTS 句子都很短；
    Lite 板载回声消除实测有效后可加 --trust-aec，说话期间急停词照样生效。

用法:
  python voice_teleop.py                             # 真机 + ReSpeaker Lite
  python voice_teleop.py --mock                      # 无舵机干跑（要有麦克风）
  python voice_teleop.py --mock --wav test.wav --no-tts   # 无硬件：wav 顶替麦克风
  python voice_teleop.py --no-wake                   # 不要唤醒词（安静环境）
  python voice_teleop.py --models ~/models/voice --card ReSpeakerLite
"""
import argparse
import math
import queue
import select
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver, TRIPOD, WAVE
from hexapod.gait import GaitEngine
from hexapod.voice.audio import (ArecordSource, WavSource, AplayPlayer, NullPlayer,
                                 find_card, alsa_device, list_cards)
from hexapod.voice.engine import VoiceEngine, ModelPaths
from hexapod.voice.tts import Speaker

POWER_PRINT_S = 0.5
EXIT_CONFIRM_S = 10.0
PREWARM = ("在", "停", "起立", "趴下", "在呢", "没听懂", "好", "再见",
           "确定退出吗？请说 确定", "前进3秒", "后退3秒", "左转3秒", "右转3秒",
           "左移3秒", "右移3秒", "换三角步态", "换波浪步态")


def read_key(timeout):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="MockDriver，不碰舵机")
    ap.add_argument("--models", help="模型根目录（默认 $HEXAPOD_VOICE_MODELS 或 ~/models/voice）")
    ap.add_argument("--card", help="ALSA 声卡名（默认自动找 ReSpeaker Lite）")
    ap.add_argument("--wav", help="用 wav 文件顶替麦克风（按真实时间节奏喂）")
    ap.add_argument("--no-wake", action="store_true", help="不要唤醒词，句句都当指令")
    ap.add_argument("--no-tts", action="store_true", help="不说话")
    ap.add_argument("--trust-aec", action="store_true",
                    help="信任 Lite 板载回声消除：机器人说话期间也听麦克风（先实测再开）")
    ap.add_argument("--follow-up", type=float, default=8.0,
                    help="唤醒后/每条指令后继续听令的秒数")
    ap.add_argument("--speed", type=float, default=40.0, help="平移速度 mm/s")
    ap.add_argument("--turn", type=float, default=0.3, help="转向速度 rad/s")
    ap.add_argument("--default-secs", type=float, default=3.0, help="没说时长时走多久")
    ap.add_argument("--max-secs", type=float, default=10.0, help="单条指令时长上限")
    ap.add_argument("--sid", type=int, default=0, help="TTS 说话人编号（多人模型才有用）")
    ap.add_argument("--stand-secs", type=float, default=4.0)
    args = ap.parse_args()

    def log(s):
        print("\r" + s + " " * 8)

    # ---- 语音侧 ----
    paths = ModelPaths.discover(args.models)
    card = args.card or find_card()
    if args.wav:
        source = WavSource(args.wav, realtime=True)
    else:
        if card is None:
            sys.exit("没找到 ReSpeaker Lite（/proc/asound/cards 里没有 respeaker/lite）。"
                     f"现有声卡: {list_cards()}\n  USB 线插好了吗（要数据线不是纯充电线）；"
                     "或 --card 指定，或 --wav 顶替。")
        source = ArecordSource(alsa_device(card))
    speaker = None
    if not args.no_tts:
        if paths.tts_dir is None:
            log("⚠ 没找到 TTS 模型，改为不说话")
        else:
            player = AplayPlayer(alsa_device(card)) if card else NullPlayer()
            speaker = Speaker(paths.tts_dir, player, sid=args.sid, log=log)
            speaker.start()
            speaker.prewarm(PREWARM)
    engine = VoiceEngine(paths, source, speaker, wake_required=not args.no_wake,
                         follow_up_s=args.follow_up,
                         mute_during_tts=not args.trust_aec, log=log)
    engine.start()

    def say(text):
        if speaker and text:
            speaker.say(text)

    # ---- 机器人侧（与 walk_teleop 一致）----
    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    bot = Hexapod(drv)
    bot.move_feet(bot.crouch_feet())
    drv.enable(True)
    time.sleep(0 if args.mock else 1.0)
    bot.stand(duration=args.stand_secs)

    tty_mode = sys.stdin.isatty()
    if tty_mode:
        import termios
        import tty
        old = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    vx = vy = wz = 0.0
    deadline = None          # 移动指令到期时刻（monotonic）
    crouched = False
    pending_exit = None
    t = 0.0
    dt = 1.0 / bot.cfg.update_hz
    last_power_check = 0.0
    peak_a = 0.0
    engine_alive = True

    def halt():
        nonlocal vx, vy, wz, deadline
        vx = vy = wz = 0.0
        deadline = None

    def dispatch(it) -> bool:
        """执行一条意图；返回 True 表示要退出程序。"""
        nonlocal vx, vy, wz, deadline, crouched, pending_exit
        k = it.kind
        if k == "stop":
            halt()
            say(it.reply)
        elif k == "walk":
            if crouched:
                bot.stand(duration=2.0)
                crouched = False
            secs = args.default_secs if it.seconds is None else min(it.seconds, args.max_secs)
            if math.isinf(secs):
                secs = args.max_secs
            vx = it.vx * args.speed * it.speed
            vy = it.vy * args.speed * it.speed
            wz = it.wz * args.turn * it.speed
            deadline = time.monotonic() + secs
            say(it.reply)
            log(f"→ 移动 vx={vx:.0f} vy={vy:.0f} wz={wz:.2f} 持续 {secs:.1f}s")
        elif k == "stand":
            halt()
            say(it.reply)
            bot.stand(duration=2.0)
            crouched = False
        elif k == "crouch":
            halt()
            say(it.reply)
            bot.glide_to(bot.crouch_feet(), 2.0)
            crouched = True
        elif k == "gait":
            bot.engine = GaitEngine(bot.cfg, TRIPOD if it.gait == "tripod" else WAVE)
            say(it.reply)
        elif k == "status":
            v, c = bot.check_power()
            say(f"电压{v:.1f}伏，电流{c:.1f}安")
        elif k == "exit":
            pending_exit = time.monotonic() + EXIT_CONFIRM_S
            say(it.reply)
        elif k == "confirm":
            if pending_exit and time.monotonic() < pending_exit:
                halt()
                say("再见")
                return True
        elif k == "cancel":
            pending_exit = None
            say(it.reply)
        elif k in ("greet", "unsupported"):
            say(it.reply)
        elif k == "unknown":
            if len(it.text) >= 2:            # 单字/空串多半是噪声，不回话
                say(it.reply)
        return False

    print("语音遥控就绪：说“小蜘蛛”唤醒；“停下/别动”随时急停"
          + ("；键盘同 walk_teleop" if tty_mode else "") + "。")
    try:
        while True:
            now = time.monotonic()
            if tty_mode:
                k = read_key(0)
                if k == "\x1b":
                    break
                elif k == "w":
                    vx, vy, deadline = args.speed, 0, None
                elif k == "s":
                    vx, vy, deadline = -args.speed, 0, None
                elif k == "a":
                    vx, vy, deadline = 0, args.speed, None
                elif k == "d":
                    vx, vy, deadline = 0, -args.speed, None
                elif k == "q":
                    wz, deadline = args.turn, None
                elif k == "e":
                    wz, deadline = -args.turn, None
                elif k == " ":
                    halt()
                elif k == "1":
                    bot.engine = GaitEngine(bot.cfg, TRIPOD)
                elif k == "2":
                    bot.engine = GaitEngine(bot.cfg, WAVE)

            quit_now = False
            while True:
                try:
                    ev = engine.events.get_nowait()
                except queue.Empty:
                    break
                if ev.kind == "ready":
                    log(f"[voice] 模型就绪（{engine.load_s:.1f}s）")
                    say("语音控制就绪，叫我小蜘蛛")
                elif ev.kind == "stop":
                    halt()
                    log(f"[voice] 急停：{ev.text}")
                    say("停")
                elif ev.kind == "command":
                    log(f"[voice] “{ev.text}” → {ev.intent.kind}")
                    quit_now = dispatch(ev.intent) or quit_now
                elif ev.kind == "error":
                    halt()
                    engine_alive = False
                    log(f"[voice] ⚠ {ev.text}")
                elif ev.kind == "eof":
                    engine_alive = False
                    log("[voice] 音源结束")
            if quit_now:
                break
            if not engine_alive and (args.wav or not tty_mode):
                break                                   # wav 跑完 / 无键盘可用 → 退出

            if deadline is not None and now >= deadline:
                halt()
                log("[voice] 到时，停")

            bot.move_feet(bot.engine.foot_targets(t, vx, vy, wz))
            if t - last_power_check > POWER_PRINT_S:
                v, c = bot.check_power()
                peak_a = max(peak_a, c)
                state = "听令中" if engine.awake else "待唤醒"
                print(f"\r电压 {v:.2f}V  电流 {c:5.2f}A  峰值 {peak_a:5.2f}A  [{state}]  ",
                      end="", flush=True)
                last_power_check = t
            time.sleep(dt)
            t += dt
    finally:
        engine.stop()
        if speaker:
            speaker.wait(3.0)
            speaker.stop()
        if tty_mode:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        drv.close()
        print("\n已断舵机电，退出。")


if __name__ == "__main__":
    main()
