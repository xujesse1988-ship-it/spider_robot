#!/usr/bin/env python3
"""语音链路自检（插上 ReSpeaker Lite、跑完 voice_setup.sh 之后第一个跑的脚本）。

  python voice_check.py --list            # 列声卡，确认 ReSpeaker Lite 在
  python voice_check.py --record 4        # 录 4 秒 → 回放（默认存 /tmp/voice_check.wav）
  python voice_check.py --asr FILE.wav    # 对 wav 跑 KWS 唤醒 + VAD 切句 + SenseVoice + 意图
  python voice_check.py --tts "语音系统就绪"   # 合成并从喇叭播放
  python voice_check.py --echo-test       # 自听测试：喇叭念数字同时双通道录音、分通道识别，
                                          #   判断 AEC 是否有效 / 处理后音频在哪个通道
  python voice_check.py                   # 全套：列声卡 → 录 5 秒（请说“小蜘蛛，前进三秒”）
                                          #        → 回放 → 识别 → TTS 报结果

不碰舵机/气路/GPIO，只用声卡。--asr 在开发机上也能跑（wav 顶替麦克风）。
"""
import argparse
import os
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.voice.audio import (ArecordSource, WavSource, AplayPlayer, find_card,
                                 alsa_device, list_cards, write_wav)
from hexapod.voice.engine import VoiceEngine, ModelPaths
from hexapod.voice.tts import Speaker

RATE = 16000


def do_list():
    cards = list_cards()
    print("声卡：")
    for idx, name, desc in cards:
        print(f"  card {idx}: {name:<18} {desc}")
    card = find_card()
    print(f"→ 选用: {card or '（没找到 ReSpeaker Lite，USB 线插好了吗）'}")
    return card


def do_record(card, secs, out):
    dev = alsa_device(card)
    print(f"录音 {secs}s（{dev}）… 请说：“小蜘蛛，前进三秒”")
    src = ArecordSource(dev)
    import numpy as np
    chunks, n = [], int(0.1 * RATE)
    t_end = time.time() + secs
    while time.time() < t_end:
        x = src.read(n)
        if x is None:
            break
        chunks.append(x)
    src.close()
    audio = np.concatenate(chunks) if chunks else np.zeros(0, np.float32)
    write_wav(out, audio, RATE)
    peak = float(np.abs(audio).max()) if len(audio) else 0.0
    rms = float(np.sqrt((audio ** 2).mean())) if len(audio) else 0.0
    print(f"  存到 {out}：{len(audio)/RATE:.1f}s 峰值 {peak:.3f} 有效值 {rms:.4f}"
          + ("  ⚠ 太小，跑 voice_mixer.sh / 查板上 Mute 键是不是按下了（红灯）"
             if peak < 0.02 else ""))
    print("  回放…")
    AplayPlayer(dev).play_file(out)
    return out


def do_asr(paths, wav, wake_required=True):
    print(f"识别 {wav} …")
    eng = VoiceEngine(paths, WavSource(wav), None, wake_required=wake_required, log=print)
    eng.start()
    events = []
    while True:
        ev = eng.events.get()
        events.append(ev)
        if ev.kind in ("eof", "error"):
            break
    eng.join(5.0)
    print("事件：")
    for ev in events:
        extra = f" → {ev.intent.kind} {ev.intent.reply}" if ev.intent else ""
        print(f"  {ev.kind:8s} {ev.text}{extra}")
    return events


def do_echo_test(paths, card, out="/tmp/voice_echo"):
    """机器人自己说话时双通道录音，分通道看回声大小和识别结果。

    判读：某一通道识别不出（或有效值明显小）→ 那路是 AEC 处理后的音频，
    HEXAPOD_AUDIO_PICK 换过去；两路都能整句认出数字 → 板载 AEC 对本场景
    无效，保持默认闭麦模式（引擎的自听过滤仍然兜底）。
    """
    import subprocess
    import wave as wave_mod
    import numpy as np
    dev = alsa_device(card)
    print("— 声卡原生参数（CHANNELS 行 = USB 固件真实通道数）—")
    r = subprocess.run(["arecord", "-D", dev.replace("plughw", "hw"), "-f", "S16_LE",
                        "--dump-hw-params", "-d", "1", "/dev/null"],
                       capture_output=True, text=True)
    for ln in (r.stderr or "").splitlines():
        if any(k in ln for k in ("CHANNELS", "RATE:", "FORMAT")):
            print("  " + ln.strip())

    text = "一二三四五六七八九十，一二三四五六七八九十，一二三四五六七八九十"
    rec = subprocess.Popen(["arecord", "-q", "-D", dev, "-f", "S16_LE", "-r", str(RATE),
                            "-c", "2", "-d", "10", "-t", "wav", f"{out}.wav"])
    time.sleep(0.5)
    print("录音 10s，喇叭念数字中…（期间别说话，只听回声）")
    spk = Speaker(paths.tts_dir, AplayPlayer(dev), log=print)
    spk.start()
    spk.say(text, block=True)
    spk.stop()
    rec.wait()

    with wave_mod.open(f"{out}.wav", "rb") as w:
        raw = w.readframes(w.getnframes())
    x = np.frombuffer(raw, dtype=np.int16).reshape(-1, 2).astype(np.float32) / 32768.0
    for ch in (0, 1):
        rms = float(np.sqrt((x[:, ch] ** 2).mean()))
        write_wav(f"{out}_ch{ch}.wav", x[:, ch], RATE)
        print(f"通道 {ch}: 有效值 {rms:.4f} → {out}_ch{ch}.wav")
    for ch in (0, 1):
        print(f"\n—— 识别通道 {ch}（当前代码用通道 {os.environ.get('HEXAPOD_AUDIO_PICK', '0')}）——")
        do_asr(paths, f"{out}_ch{ch}.wav", wake_required=False)


def do_tts(paths, card, text):
    if paths.tts_dir is None:
        print("没找到 TTS 模型目录")
        return
    spk = Speaker(paths.tts_dir, AplayPlayer(alsa_device(card)), log=print)
    spk.start()
    t0 = time.time()
    spk.say(text, block=True)
    print(f"  播完（含首次加载）{time.time()-t0:.1f}s")
    spk.stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--record", type=float, metavar="SECS")
    ap.add_argument("--asr", metavar="WAV")
    ap.add_argument("--no-wake", action="store_true", help="--asr 时不要求唤醒词")
    ap.add_argument("--tts", metavar="TEXT")
    ap.add_argument("--echo-test", action="store_true",
                    help="双通道自听测试：判断 AEC / 处理后音频在哪个通道")
    ap.add_argument("--out", default="/tmp/voice_check.wav")
    ap.add_argument("--models")
    ap.add_argument("--card")
    args = ap.parse_args()
    specific = any([args.list, args.record, args.asr, args.tts, args.echo_test])

    card = args.card
    if args.list or not specific:
        card = card or do_list()
    else:
        card = card or find_card()

    paths = None
    if args.asr or args.tts or args.echo_test or not specific:
        paths = ModelPaths.discover(args.models)
        print(f"模型：KWS={paths.kws_dir.name}  ASR={paths.asr_dir.name}  "
              f"TTS={paths.tts_dir.name if paths.tts_dir else '无'}  关键词={paths.keywords_file.name}")

    if args.record:
        do_record(card, args.record, args.out)
    if args.asr:
        do_asr(paths, args.asr, wake_required=not args.no_wake)
    if args.tts:
        do_tts(paths, card, args.tts)
    if args.echo_test:
        if card is None:
            sys.exit("没有 ReSpeaker Lite 声卡，--echo-test 要实机跑。")
        do_echo_test(paths, card)

    if not specific:
        if card is None:
            sys.exit("没有 ReSpeaker Lite 声卡，先解决 USB 连接。")
        wav = do_record(card, 5.0, args.out)
        events = do_asr(paths, wav)
        cmds = [e for e in events if e.kind == "command" and e.intent.kind != "unknown"]
        woke = any(e.kind == "wake" for e in events)
        summary = ("语音链路正常" if woke and cmds else
                   "没听到唤醒词" if not woke else "唤醒了但没听懂指令")
        print(f"\n结论：{summary}")
        do_tts(paths, card, summary)


if __name__ == "__main__":
    main()
