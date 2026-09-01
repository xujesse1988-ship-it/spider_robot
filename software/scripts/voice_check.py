#!/usr/bin/env python3
"""语音链路自检（装好 HAT 驱动、跑完 voice_setup.sh 之后第一个跑的脚本）。

  python voice_check.py --list            # 列声卡，确认 seeed2micvoicec 在
  python voice_check.py --record 4        # 录 4 秒 → 回放（默认存 /tmp/voice_check.wav）
  python voice_check.py --asr FILE.wav    # 对 wav 跑 KWS 唤醒 + VAD 切句 + SenseVoice + 意图
  python voice_check.py --tts "语音系统就绪"   # 合成并从喇叭播放
  python voice_check.py                   # 全套：列声卡 → 录 5 秒（请说“小蜘蛛，前进三秒”）
                                          #        → 回放 → 识别 → TTS 报结果

不碰舵机/气路/GPIO，只用声卡。--asr 在开发机上也能跑（wav 顶替麦克风）。
"""
import argparse
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
    print(f"→ 选用: {card or '（没找到 HAT，检查 dtoverlay / 接线）'}")
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
          + ("  ⚠ 太小，查 amixer 'Capture' 音量 / Boost 开关" if peak < 0.02 else ""))
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
    ap.add_argument("--out", default="/tmp/voice_check.wav")
    ap.add_argument("--models")
    ap.add_argument("--card")
    args = ap.parse_args()
    specific = any([args.list, args.record, args.asr, args.tts])

    card = args.card
    if args.list or not specific:
        card = card or do_list()
    else:
        card = card or find_card()

    paths = None
    if args.asr or args.tts or not specific:
        paths = ModelPaths.discover(args.models)
        print(f"模型：KWS={paths.kws_dir.name}  ASR={paths.asr_dir.name}  "
              f"TTS={paths.tts_dir.name if paths.tts_dir else '无'}  关键词={paths.keywords_file.name}")

    if args.record:
        do_record(card, args.record, args.out)
    if args.asr:
        do_asr(paths, args.asr, wake_required=not args.no_wake)
    if args.tts:
        do_tts(paths, card, args.tts)

    if not specific:
        if card is None:
            sys.exit("没有 HAT 声卡，先解决驱动/接线。")
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
