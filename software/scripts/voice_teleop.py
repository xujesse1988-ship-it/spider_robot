#!/usr/bin/env python3
"""语音遥控行走（树莓派 + ReSpeaker Lite USB 麦克风/喇叭板；键盘照旧可用）。

  说"小蜘蛛"唤醒 → 听令窗口内说指令，可连说：
    前进/后退/左移/右移/左转/右转 [N 秒 | N 步] [快点 | 慢点]
      ——不带时长=连续动作，一直走到喊停/新指令/键盘干预（回复念"一直前进，
      说停就停"）；带时长到点自停
    快点 / 慢点（单说）＝调速：持续倍率 ×1.5/×0.6 夹在 0.4~2.0，走动中立即
      生效、后续指令继承；键盘键不跟倍率走（始终基准速度）
    站起来 / 趴下 / 三角步态 / 波浪步态 / 电压多少 / 你好 / 自我介绍（长回话）
    退出（要再说"确定"）
  "停下 / 停止 / 别动"不用唤醒，随时生效（流式关键词急停，~0.3s）——
  机器人自己说话时也听得见，且急停会立即打断说话
  键盘（有终端时）：w/s/a/d/q/e 动，空格停，1/2 步态，v 轮换阀策略，ESC 退出，
  同 walk_teleop

安全边界：
  · 移动指令默认连续（不带时长=一直走），靠急停词/新指令/键盘停——回复会念
    "说停就停"提醒；带时长的到点自停，上限 --max-secs 防把时长听歪；
    没听懂的句子不动；退出要二次确认；欠压照旧抛异常断舵机电。
  · 语音引擎线程只产事件；舵机/步态全部在主线程，和 walk_teleop 是同一个环。
  · 机器人自己说话期间默认不听麦克风（防听见自己说的"停"），TTS 句子都很短；
    Lite 板载回声消除实测有效后可加 --trust-aec，说话期间急停词照样生效。

用法:
  python voice_teleop.py                             # 真机 + ReSpeaker Lite
  python voice_teleop.py --mock                      # 无舵机干跑（要有麦克风）
  python voice_teleop.py --mock --wav test.wav --no-tts   # 无硬件：wav 顶替麦克风
  python voice_teleop.py --no-wake                   # 不要唤醒词（安静环境）
  python voice_teleop.py --models ~/models/voice --card ReSpeakerLite
  python voice_teleop.py --vent off                  # 阀策略（默认 auto：站起/走动时六阀
                                                     # 通电排气让吸盘通大气、站着不动断电；
                                                     # on 常通；off 不碰阀=被动真空锁脚抬不
                                                     # 起，对照用。详见 walk_teleop 模块头）
"""
import argparse
import math
import queue
import select
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod import Hexapod, Servo2040Driver, MockDriver, TRIPOD, WAVE
from hexapod.adhesion import GroundVent, MockVacuumIO
from hexapod.gait import GaitEngine
from hexapod.voice.audio import (ArecordSource, WavSource, AplayPlayer, NullPlayer,
                                 find_card, alsa_device, list_cards)
from hexapod.voice.engine import VoiceEngine, ModelPaths
from hexapod.voice.intents import INTRO_REPLY
from hexapod.voice.tts import Speaker
from hexapod.voice.voiceprint import VoiceGate, default_profile

POWER_PRINT_S = 0.5
EXIT_CONFIRM_S = 10.0
PREWARM = ("在", "停", "起立", "趴下", "在呢", "没听懂", "好", "再见",
           "提速", "减速", "已经最快了", "已经最慢了",
           "确定退出吗？请说 确定",
           "一直前进，说停就停", "一直后退，说停就停", "一直左转，说停就停",
           "一直右转，说停就停", "一直左移，说停就停", "一直右移，说停就停",
           "换三角步态", "换波浪步态", INTRO_REPLY)


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
    ap.add_argument("--max-secs", type=float, default=10.0,
                    help="带时长指令的上限（防把'三秒'听歪成'三十秒'；不带时长=一直走不受限）")
    ap.add_argument("--sid", type=int, default=0, help="TTS 说话人编号（多人模型才有用）")
    ap.add_argument("--tts-gain", type=float, default=1.0,
                    help="TTS 音量倍率；喇叭离麦克风近时降到 0.6，提高说话期间急停命中")
    ap.add_argument("--no-voiceprint", action="store_true",
                    help="不开声纹锁（有注册档案时默认开：指令只听主人，急停不拦）")
    ap.add_argument("--voiceprint", help="声纹档案路径（默认 <模型根>/voiceprint_owner.npz）")
    ap.add_argument("--spk-threshold", type=float, help="声纹阈值，覆盖档案里的建议值")
    ap.add_argument("--stand-secs", type=float, default=4.0)
    ap.add_argument("--vent", choices=GroundVent.MODES, default="auto",
                    help="六阀策略：auto=站起/走动通电排气、静止断电（默认）；"
                         "on=常通；off=不碰阀（脚会被被动真空吸住，对照用）。运行中 v 轮换")
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
            speaker = Speaker(paths.tts_dir, player, sid=args.sid,
                              gain=args.tts_gain, log=log)
            speaker.start()
            speaker.prewarm(PREWARM)
    gate = None
    if not args.no_voiceprint:
        from pathlib import Path
        prof = Path(args.voiceprint) if args.voiceprint else default_profile(args.models)
        if prof.exists() and paths.spk_model is not None:
            gate = VoiceGate(paths.spk_model, prof, threshold=args.spk_threshold, log=log)
            log(f"[voice] 声纹锁开：{prof.name} 阈值 {gate.threshold:.2f}（急停谁喊都停）")
        elif args.voiceprint:
            sys.exit(f"声纹档案 {prof} 不存在或缺声纹模型——先跑 scripts/voice_enroll.py")
    engine = VoiceEngine(paths, source, speaker, wake_required=not args.no_wake,
                         follow_up_s=args.follow_up,
                         mute_during_tts=not args.trust_aec, voice_gate=gate, log=log)
    engine.start()

    def say(text):
        if speaker and text:
            speaker.say(text)

    # ---- 机器人侧（与 walk_teleop 一致）----
    # 阀先于舵机：站起时吸盘就已通大气，压下去不攒被动真空
    vent = GroundVent(io_factory=(lambda: MockVacuumIO(6)) if args.mock else None,
                      stagger_s=0.0 if args.mock else 0.2)
    vent_mode = args.vent
    if vent_mode != "off":
        vent.set(True)
        log(f"阀策略 {vent_mode}：站起前六阀已拉到排气位（吸盘通大气）")
    else:
        log("阀策略 off：不碰阀（通罐位），吸盘可能被被动真空吸在地上，按 v 切策略对照")
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
    spd_mult = 1.0     # 光杆"快点/慢点"的持续倍率（0.4~2.0）；只作用于语音指令，
                       # 键盘键始终基准速度（底层直控，不跟语音倍率走）

    def halt():
        nonlocal vx, vy, wz, deadline
        vx = vy = wz = 0.0
        deadline = None

    def stand_up():
        """蹲→站：先通电排气再起身（蹲姿压扁的吸盘随身体升起会攒被动真空）。
        阻塞约 1s+2s，站定后由 auto 策略在主环里落稳断电。"""
        nonlocal crouched
        if vent_mode != "off":
            vent.set(True)
        bot.stand(duration=2.0)
        crouched = False

    def dispatch(it) -> bool:
        """执行一条意图；返回 True 表示要退出程序。"""
        nonlocal vx, vy, wz, deadline, crouched, pending_exit, spd_mult
        k = it.kind
        if k == "stop":
            halt()
            if speaker:
                speaker.cancel()          # 急停连嘴一起停
            say(it.reply)
        elif k == "speed":
            # 光杆"快点/慢点"：调持续倍率，走动中立即生效（新指令也继承）
            old = spd_mult
            spd_mult = max(0.4, min(2.0, spd_mult * it.speed))
            if spd_mult == old:
                say("已经最快了" if it.speed > 1.0 else "已经最慢了")
            else:
                if vx or vy or wz:
                    vx *= spd_mult / old
                    vy *= spd_mult / old
                    wz *= spd_mult / old
                say(it.reply)
            log(f"[voice] 速度倍率 ×{spd_mult:g}")
        elif k == "walk":
            if crouched:
                stand_up()
            vx = it.vx * args.speed * it.speed * spd_mult
            vy = it.vy * args.speed * it.speed * spd_mult
            wz = it.wz * args.turn * it.speed * spd_mult
            say(it.reply)
            if it.seconds is None or math.isinf(it.seconds):
                deadline = None      # 连续动作：说停/新指令/键盘为止（同键盘 wasd）
                log(f"→ 移动 vx={vx:.0f} vy={vy:.0f} wz={wz:.2f} 一直（说停为止）")
            else:
                secs = min(it.seconds, args.max_secs)
                deadline = time.monotonic() + secs
                log(f"→ 移动 vx={vx:.0f} vy={vy:.0f} wz={wz:.2f} 持续 {secs:.1f}s")
        elif k == "stand":
            halt()
            say(it.reply)
            stand_up()
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
        elif k in ("greet", "unsupported", "intro"):
            say(it.reply)
        elif k == "unknown":
            if len(it.text) >= 2:            # 单字/空串多半是噪声，不回话
                say(it.reply)
        return False

    print("─" * 30 + " 指令一览 " + "─" * 30)
    print("唤醒：" + ("不用（--no-wake），直接说指令"
                     if args.no_wake else "小蜘蛛 / 蜘蛛同学 —— 唤醒后说指令，可连说"))
    print("急停：停下 / 停止 / 别动 / 站住 —— 不用唤醒随时喊，它自己说话时也管用")
    print("移动：前进 后退 左移 右移 左转 右转 [快点/慢点] —— 一直走，喊停为止；")
    print("      带时长（“前进三秒”/“走两步”）则到点自停；走动中单说 快点/慢点=调速")
    print("姿势：站起来 / 趴下        步态：三角步态 / 波浪步态")
    print("问答：电压多少 / 你好 / 自我介绍（15 秒长回话，可趁机试喊停）")
    print("退出：退出 → 10 秒内再说“确定”；说“取消”反悔")
    if gate is not None:
        print("声纹：行走等指令只听主人，急停谁喊都停（--no-voiceprint 关）")
    if tty_mode:
        print("键盘：w/s/a/d/q/e 动，空格停，1/2 步态，v 阀策略，ESC 退出")
    print("─" * 70)
    print("语音遥控就绪。")
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
                elif k == "v":
                    vent_mode = GroundVent.MODES[(GroundVent.MODES.index(vent_mode) + 1)
                                                 % len(GroundVent.MODES)]
                    log(f"阀策略 → {vent_mode}")

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
                    if speaker:
                        speaker.cancel()  # 正在说话也立即闭嘴
                    log(f"[voice] 急停：{ev.text}")
                    say("停")
                elif ev.kind == "command":
                    log(f"[voice] “{ev.text}” → {ev.intent.kind}")
                    quit_now = dispatch(ev.intent) or quit_now
                elif ev.kind == "denied":
                    log(f"[voice] 声纹不符，忽略：“{ev.text}”")
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

            # auto：走动通电/静止断电；线圈没全部通电前脚不抬（原地等 ≈1s，急停照常处理）
            if vent.drive(vent_mode, bool(vx or vy or wz)):
                bot.move_feet(bot.engine.foot_targets(t, vx, vy, wz))
            else:
                bot.move_feet(bot.engine.foot_targets(t, 0.0, 0.0, 0.0))
            if t - last_power_check > POWER_PRINT_S:
                v, c = bot.check_power()
                peak_a = max(peak_a, c)
                state = "听令中" if engine.awake else "待唤醒"
                print(f"\r电压 {v:.2f}V  电流 {c:5.2f}A  峰值 {peak_a:5.2f}A  [{state}]  "
                      f"阀[{vent_mode}] {vent.state_text()}  ", end="", flush=True)
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
        try:
            drv.close()
        finally:
            vent.close()    # 六线圈断电再释放，绝不拉高着退（阀会一直通电发热）
        print("\n已断舵机电、阀线圈已断电，退出。")


if __name__ == "__main__":
    main()
