#!/usr/bin/env python3
"""爬墙语音壳：climb_walk.py 原样跑在 pty 里，语音只当"另一只手"敲键盘。

架构（为什么不另写一遍爬墙环）：climb_walk 的主循环上千行全是审过的安全
代码（互锁/冻结/退出与取机序列/黑匣子），复制一遍等于养两个会分叉的爬墙
脚本。本脚本一行不改它：pty 里原样执行 climb_walk，语音识别出的指令映射
成按键写进 pty——与手敲完全等价，climb_walk 的全部拒绝条件（单步在途/
已放开/大步幅禁侧移/冻结…）照常生效，按键也照常进它的黑匣子；你的键盘
raw 透传（p/i/f/o×2/ESC×2/Ctrl-C 语义原封不动）；它的关键输出（就位/
吸附完成/冻结/悬停/单步完成/已放开）由本脚本念出来——眼睛盯机器人，
耳朵拿进度。语音引擎挂掉不影响爬墙（键盘照常）。

语音白名单（说 → 键）：
  停下/停止/别动     急停（不用唤醒随时喊、谁喊都停，机器人说话时也听）→ 空格
  前进/后退/左移/右移/左转/右转 [N 秒|N 步]   → w/s/a/d/q/e
      不带时长=一直走说停为止；带时长到点由本脚本补一个空格自停
      （上限 --max-secs；吸附完成前说了不注入，防启动期攒误触速度）
  单步/抬腿 → i      落地/踩下 → i      解冻/解除冻结 → f      开始吸附 → p
  电压 → 念最近状态行（电压/电流/最差盘压）      你好 → 在呢
语音黑名单（永远只认键盘——听歪一个词就放气的险不能冒）：
  退出（ESC×2，会放气，墙上=坠落）、放吸盘取机（o×2）：语音说了只回键盘
  指引。快点/慢点在爬墙无效（速度 --speed 定死，宁慢勿快）；站起/趴下/
  换步态爬墙不支持。播报措辞纪律：不得含 KWS 急停词（停下/停止/停下来/
  别动），否则机器人念到那儿自己触发急停。

用法（语音参数 + climb_walk 参数原样透传，两边参数名无冲突）：
  python voice_climb.py --mock                    # 干跑
  python voice_climb.py --tts-gain 0.6 --sag-comp 3
  python voice_climb.py --release                 # 不起语音，直接转交 climb_walk
"""
import argparse
import math
import os
import pty
import queue
import re
import select
import signal
import sys
import termios
import time
import tty

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.voice.audio import (ArecordSource, WavSource, AplayPlayer, NullPlayer,
                                 find_card, alsa_device, list_cards)
from hexapod.voice.engine import VoiceEngine, ModelPaths
from hexapod.voice.intents import parse_climb
from hexapod.voice.tts import Speaker
from hexapod.voice.voiceprint import VoiceGate, default_profile

CLIMB_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "climb_walk.py")
PREWARM = ("在", "停", "在呢", "没听懂", "单步", "落地", "解冻", "开始吸附序列",
           "语音就绪，叫我小蜘蛛", "还没吸附完成，先说 开始吸附",
           "一直前进，说停就停", "一直后退，说停就停", "一直左转，说停就停",
           "一直右转，说停就停", "一直左移，说停就停", "一直右移，说停就停")
# climb_walk 输出片段 → 播报（措辞与 climb_walk.py 的打印保持同步；改那边
# 的提示语时记得对一眼这张表——匹配不上只是不播报，不影响功能）
MARKERS = (
    ("就位暂停：确认无异常", "就位。检查没问题后说 开始吸附"),
    ("✓ 六足吸附完成", "六足吸附完成，可以走了"),
    ("⚠⚠⚠ 全机冻结", "冻结了，看屏幕处理"),
    ("已悬停在落点上方", "悬停中，说落地收口"),
    ("单步完成", "单步完成"),
    ("已放开：全阀排气", "吸盘已放开，可以取机"),
)
_STATUS_RE = re.compile(r"(\d+\.\d{2})V\s+(-?\d+\.\d{2})A")
_CUP_RE = re.compile(r"盘差 [LR]\d\s*(-?\d+\.\d)")


def walk_key(it):
    """走行意图 → climb_walk 按键（方向映射与它的键位表一致）。"""
    if it.wz > 0:
        return b"q"
    if it.wz < 0:
        return b"e"
    if it.vy > 0:
        return b"a"
    if it.vy < 0:
        return b"d"
    if it.vx > 0:
        return b"w"
    if it.vx < 0:
        return b"s"
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--models", help="模型根目录（默认 $HEXAPOD_VOICE_MODELS 或 ~/models/voice）")
    ap.add_argument("--card", help="ALSA 声卡名（默认自动找 ReSpeaker Lite）")
    ap.add_argument("--wav", help="用 wav 顶替麦克风（开发机联调）")
    ap.add_argument("--no-wake", action="store_true", help="不要唤醒词，句句都当指令")
    ap.add_argument("--no-tts", action="store_true", help="不说话（播报只落终端）")
    ap.add_argument("--trust-aec", action="store_true",
                    help="信任板载回声消除：机器人说话期间也听麦克风")
    ap.add_argument("--follow-up", type=float, default=8.0,
                    help="唤醒后/每条指令后继续听令的秒数")
    ap.add_argument("--max-secs", type=float, default=30.0,
                    help="带时长指令的上限（爬墙慢，默认 30s；不带时长=一直走不受限）")
    ap.add_argument("--sid", type=int, default=0, help="TTS 说话人编号")
    ap.add_argument("--tts-gain", type=float, default=1.0,
                    help="TTS 音量倍率；喇叭近麦克风降 0.6 提高说话期间急停命中")
    ap.add_argument("--no-voiceprint", action="store_true",
                    help="不开声纹锁（有档案默认开：指令只听主人，急停不拦）")
    ap.add_argument("--voiceprint", help="声纹档案路径（默认 <模型根>/voiceprint_owner.npz）")
    ap.add_argument("--spk-threshold", type=float, help="声纹阈值，覆盖档案建议值")
    args, climb_args = ap.parse_known_args()

    if "--release" in climb_args:
        # 善后不需要语音（也没有键盘交互），直接把进程交给 climb_walk
        os.execv(sys.executable, [sys.executable, CLIMB_SCRIPT] + climb_args)

    # ---- 语音前置检查全部放在 fork 之前：缺模型/缺声卡要死在碰机器人之前 ----
    paths = ModelPaths.discover(args.models)
    card = args.card or find_card()
    if not args.wav and card is None:
        sys.exit("没找到 ReSpeaker Lite（/proc/asound/cards 里没有 respeaker/lite）。"
                 f"现有声卡: {list_cards()}\n  --card 指定，或 --wav 顶替，"
                 "或直接跑 climb_walk.py（纯键盘）。")
    gate = None
    if not args.no_voiceprint:
        from pathlib import Path
        prof = Path(args.voiceprint) if args.voiceprint else default_profile(args.models)
        if prof.exists() and paths.spk_model is not None:
            gate = VoiceGate(paths.spk_model, prof, threshold=args.spk_threshold)
            print(f"[voice] 声纹锁开：{prof.name} 阈值 {gate.threshold:.2f}（急停谁喊都停）")
        elif args.voiceprint:
            sys.exit(f"声纹档案 {prof} 不存在或缺声纹模型——先跑 scripts/voice_enroll.py")

    print("─" * 24 + " 爬墙语音壳 " + "─" * 24)
    print("急停：停下 / 停止 / 别动 —— 不用唤醒随时喊，谁喊都停")
    print("移动：前进 后退 左移 右移 左转 右转 [N 秒]（不带时长=一直走）")
    print("单步：单步/抬腿=抬    落地/踩下=落    解冻：解除冻结    启动：开始吸附/启动")
    print("问答：电压    唤醒：小蜘蛛（急停之外先唤醒再说指令）")
    print("键盘全部照旧；退出（ESC×2）与取机（o×2）只认键盘，语音做不了")
    print("─" * 60)

    pid, master = pty.fork()
    if pid == 0:
        os.execv(sys.executable, [sys.executable, CLIMB_SCRIPT] + climb_args)

    # ---- 父进程：终端 raw 透传（Ctrl-C 变字节 \x03 送进 pty，由 climb_walk
    # 自己的 KeyboardInterrupt 路径处理；ESC×2/o×2 双击语义原样过去）----
    tty_mode = sys.stdin.isatty()
    old_tio = None
    if tty_mode:
        import fcntl
        try:
            fcntl.ioctl(master, termios.TIOCSWINSZ,
                        fcntl.ioctl(0, termios.TIOCGWINSZ, b"\0" * 8))
        except OSError:
            pass
        old_tio = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    # 非 tty（冒烟/管道）下 Ctrl-C 会打到本进程：转成 \x03 交给子进程善后
    for sg in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sg, lambda *_: os.write(master, b"\x03"))

    def log(s):
        # 子进程状态行 \r 常驻，父进程消息换行独占一行（raw 模式下要 \r\n）
        sys.stdout.write("\r\n" + s + "\r\n")
        sys.stdout.flush()

    # ---- 语音栈（线程必须在 fork 之后起）----
    source = WavSource(args.wav, realtime=True) if args.wav \
        else ArecordSource(alsa_device(card))
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
    if gate is not None:
        gate.log = log
    engine = VoiceEngine(paths, source, speaker, wake_required=not args.no_wake,
                         follow_up_s=args.follow_up,
                         mute_during_tts=not args.trust_aec,
                         voice_gate=gate, parser=parse_climb, log=log)
    engine.start()

    def say(text):
        if speaker and text:
            speaker.say(text)

    # ---- 从子进程输出提取的状态（只影响播报与提示，不影响任何控制）----
    pending = b""
    last = {"v": None, "c": None, "cup": None}
    started = False        # 见过"✓ 六足吸附完成"
    released = False       # 见过"已放开：全阀排气"（取机窗口，行走永久拒绝）
    deadline = None        # 语音带时长指令的自停时刻

    def scan(data):
        nonlocal pending, started, released
        pending += data
        parts = re.split(rb"[\r\n]+", pending)
        pending = parts.pop()
        if len(pending) > 4096:            # 长期无换行的兜底，只留尾巴
            pending = pending[-1024:]
        for raw in parts + [pending]:      # 状态行常驻在未换行的尾巴里
            line = raw.decode("utf-8", "ignore")
            m = _STATUS_RE.search(line)
            if m:
                last["v"], last["c"] = float(m.group(1)), float(m.group(2))
                mc = _CUP_RE.search(line)
                last["cup"] = float(mc.group(1)) if mc else None
        for raw in parts:
            line = raw.decode("utf-8", "ignore")
            for pat, speech in MARKERS:
                if pat in line:
                    if pat.startswith("✓ 六足吸附完成"):
                        started = True
                    if pat.startswith("已放开"):
                        released = True
                    say(speech)
                    break

    def status_speech():
        if last["v"] is None:
            return "还没读到状态"
        s = f"电压 {last['v']:.1f} 伏，电流 {last['c']:.1f} 安"
        if last["cup"] is not None:
            s += f"，最差盘压负 {abs(last['cup']):.0f} 千帕"
        return s

    def inject(key, why):
        os.write(master, key)
        log(f"[voice] {why} → 键 {key.decode()}")

    def do_stop(text):
        nonlocal deadline
        os.write(master, b" ")             # 先把速度归零，再管嘴
        deadline = None
        if speaker:
            speaker.cancel()
        log(f"[voice] 急停：{text} → 键 空格")
        say("停")

    def echoish(text):
        """动作词回声守卫（第二道，专为爬墙加）：引擎层回声过滤对 ≤2 字文本
        只认全等（保真急停），于是 --trust-aec 下自己播报"悬停中，说落地收口"
        被听成"落地"能穿过过滤——在墙上等于自己给自己下抬落腿指令。凡动作类
        意图，文本若包含在 2.5s 内自己说过的话里就丢弃；用户真想执行，等播报
        完再说一遍就是。急停永不经此守卫。"""
        return (speaker is not None and text
                and any(text in s for s in speaker.recent_texts()))

    def dispatch(it):
        nonlocal deadline
        k = it.kind
        if k in ("walk", "step", "land", "unfreeze", "begin") and echoish(it.text):
            log(f"[voice] “{it.text}” ≈ 自己刚说的短词，不执行（要执行等我说完再说）")
            return
        if k == "stop":                    # ASR 整句层的停（KWS 停另有事件）
            do_stop(it.text)
        elif k == "walk":
            if released:
                log("[voice] 已放开吸盘（取机窗口），行走指令不注入")
                say("吸盘已放开，动不了了")
                return
            if not started:
                log("[voice] 还没吸附完成，行走指令不注入（先说 开始吸附/启动）")
                say("还没吸附完成，先说 开始吸附")
                return
            key = walk_key(it)
            if key is None:
                return
            if it.speed != 1.0:
                log("[voice] 爬墙不支持快点/慢点（速度由 --speed 定死），按常速走")
            inject(key, f"“{it.text}”")
            deadline = None
            if it.seconds is not None and not math.isinf(it.seconds):
                deadline = time.monotonic() + min(it.seconds, args.max_secs)
            say(it.reply)
        elif k in ("step", "land"):
            if released:
                log("[voice] 已放开吸盘（取机窗口），单步不注入")
                say("吸盘已放开，动不了了")
            elif not started:
                log("[voice] 还没吸附完成，单步不注入（先说 开始吸附/启动）")
                say("还没吸附完成，先说 开始吸附")
            else:
                inject(b"i", f"“{it.text}”")   # 抬/落都是 i，phase 由 climb_walk 判
                say(it.reply)
        elif k == "unfreeze":
            inject(b"f", f"“{it.text}”")
            say(it.reply)
        elif k == "begin":
            inject(b"p", f"“{it.text}”")
            say(it.reply)
        elif k == "pickup":
            log("[voice] 取机不走语音：键盘按两次 o（先扶稳机身；墙上=放开即坠）")
            say("取机只认键盘")
        elif k == "exit":
            log("[voice] 退出不走语音：键盘 ESC 按两次（会放气；墙上禁用）")
            say("退出只认键盘")
        elif k == "status":
            say(status_speech())
        elif k in ("stand", "crouch", "gait"):
            say("爬墙状态不支持")
        elif k == "speed":
            say("爬墙速度是定死的，不支持调速")   # --speed 启动时定，宁慢勿快
        elif k == "intro":
            say("爬墙中，回头再介绍")       # 15s 长回话在墙上纯占耳朵
        elif k in ("greet", "unsupported"):
            say(it.reply)
        elif k == "unknown":
            if len(it.text) >= 2:          # 单字/噪声不回话
                say(it.reply)
        # confirm/cancel/ignore：静默（语音没有退出流程，确认词无处可用）

    # ---- 泵环：子进程输出透传+扫描、键盘透传、语音事件、定时自停 ----
    forward_stdin = True
    try:
        while True:
            fds = [master] + ([0] if forward_stdin else [])
            r, _, _ = select.select(fds, [], [], 0.05)
            if master in r:
                try:
                    data = os.read(master, 4096)
                except OSError:            # EIO = 子进程退出、pty 从端关闭
                    data = b""
                if not data:
                    break
                os.write(1, data)
                scan(data)
            if forward_stdin and 0 in r:
                data = os.read(0, 64)
                if not data:
                    forward_stdin = False  # stdin 到头（管道跑法），别空转
                else:
                    os.write(master, data)
                    if any(b in b" wsadqei" for b in data):
                        deadline = None    # 键盘接管，语音的定时自停作废
            while True:
                try:
                    ev = engine.events.get_nowait()
                except queue.Empty:
                    break
                if ev.kind == "ready":
                    log(f"[voice] 引擎就绪（模型加载 {engine.load_s:.1f}s）")
                    say("语音就绪，叫我小蜘蛛")
                elif ev.kind == "stop":
                    do_stop(ev.text)
                elif ev.kind == "command":
                    log(f"[voice] “{ev.text}” → {ev.intent.kind}")
                    dispatch(ev.intent)
                elif ev.kind == "denied":
                    log(f"[voice] 声纹不符，忽略：“{ev.text}”")
                elif ev.kind == "error":
                    log(f"[voice] ⚠ {ev.text} ——语音失效，键盘照常")
                elif ev.kind == "eof":
                    log("[voice] 音源结束（键盘照常）")
            if deadline is not None and time.monotonic() >= deadline:
                deadline = None
                os.write(master, b" ")
                log("[voice] 到时，停走")
    finally:
        engine.stop()
        if speaker:
            speaker.cancel()
            speaker.stop()
        if old_tio is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_tio)
        try:
            os.close(master)
        except OSError:
            pass
    _, st = os.waitpid(pid, 0)
    sys.exit(os.waitstatus_to_exitcode(st))


if __name__ == "__main__":
    main()
