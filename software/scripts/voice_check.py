#!/usr/bin/env python3
"""语音链路自检（插上 ReSpeaker Lite、跑完 voice_setup.sh 之后第一个跑的脚本）。

  python voice_check.py --list            # 列声卡，确认 ReSpeaker Lite 在
  python voice_check.py --record 4        # 录 4 秒 → 回放（默认存 /tmp/voice_check.wav）
  python voice_check.py --asr FILE.wav    # 对 wav 跑 KWS 唤醒 + VAD 切句 + SenseVoice + 意图
  python voice_check.py --tts "语音系统就绪"   # 合成并从喇叭播放
  python voice_check.py --echo-test       # 自听测试：喇叭念数字同时双通道录音、分通道识别，
                                          #   判断 AEC 是否有效 / 处理后音频在哪个通道
  python voice_check.py --noise-test      # 走路噪声测试（另开终端让机器人走着）：三段引导
                                          #   录音 → 量化噪声底/带噪说话/静止说话的电平与
                                          #   信噪比 → 判"纯掩蔽"还是"AGC 被噪声压了增益"
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
                                 alsa_device, list_cards, read_wav, write_wav)
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


def _frame_dbfs(audio, pct):
    """30ms 帧 RMS 的百分位电平（dBFS）。噪声底取中位数（50），说话段取 90
    ——整段 RMS 会被停顿稀释，帧百分位才是"说话时到底多响"。"""
    import math
    import numpy as np
    n = int(0.03 * RATE)
    k = len(audio) // n
    if k == 0:
        return -120.0
    rms = np.sqrt((audio[:k * n].reshape(k, n) ** 2).mean(axis=1))
    v = float(np.percentile(rms, pct))
    return 20.0 * math.log10(max(v, 1e-6))


_BANDS = ((0, 150), (150, 300), (300, 1000), (1000, 2000), (2000, 4000), (4000, 8000))


def _band_ms(audio, lo, hi):
    """频段内的均方功率（线性；Hann 窗修正，全带求和≈整段均方）。"""
    import numpy as np
    if len(audio) < RATE // 4:
        return 1e-12
    w = np.hanning(len(audio))
    spec = np.abs(np.fft.rfft(audio * w)) ** 2
    freqs = np.fft.rfftfreq(len(audio), 1.0 / RATE)
    m = (freqs >= lo) & (freqs < hi)
    return max(float(2.0 * spec[m].sum() / (len(audio) * (w ** 2).sum())), 1e-12)


def do_noise_analyze(wavs):
    """三段录音分频段：看走路噪声堆在哪个频段，判"高通能不能赚"。
    低频(<300Hz)占大头=结构传导，HEXAPOD_AUDIO_HPF=200 能白赚几个 dB；
    噪声在语音带(300~4k)内=滤波无解，回物理（减振/挪位/朝向）。"""
    import math
    import numpy as np
    names = ("A 走动噪声", "B 走动+说话", "C 静止说话")
    datas = []
    for w in wavs:
        audio, _ = read_wav(w)
        datas.append(audio)
    db = lambda p: 10.0 * math.log10(p)
    print("\n分频段功率（dB，越大越响；关键看 A 列噪声堆在哪）：")
    print("  频段        " + "".join(f"{n:>12}" for n in names[:len(datas)]))
    for lo, hi in _BANDS:
        row = "".join(f"{db(_band_ms(d, lo, hi)):12.1f}" for d in datas)
        print(f"  {lo:>4}~{hi:<5}Hz{row}")
    a = datas[0]
    lf = sum(_band_ms(a, lo, hi) for lo, hi in _BANDS[:2])          # <300Hz
    hf = sum(_band_ms(a, lo, hi) for lo, hi in _BANDS[4:])          # >2kHz
    full = sum(_band_ms(a, lo, hi) for lo, hi in _BANDS)
    hpf_gain = db(full) - db(full - lf * 0.9)     # 高通 200 大致砍掉 <300 的九成
    print(f"判读：走动噪声 <300Hz 占 {lf / full * 100:.0f}%、>2kHz 占 {hf / full * 100:.0f}%"
          f"；高通 200Hz 约可压低噪声底 {hpf_gain:.1f} dB")
    if len(datas) >= 2:
        sa = _band_ms(a, 300, 4000)
        sb = _band_ms(datas[1], 300, 4000)
        print(f"  语音带（300~4k）内信噪比 ≈ {db(sb) - db(sa):.1f} dB")
    if hpf_gain >= 3.0:
        print("  低频占大头（结构传导/泵一类）：值得开高通实测 A/B"
              "（同一段录音，确定性对比）：\n"
              "    HEXAPOD_AUDIO_HPF=200 python scripts/voice_check.py"
              " --asr /tmp/voice_noise_b.wav --no-wake\n"
              "  有效的话 voice_teleop/voice_climb 前也带上这个环境变量")
    elif hf / full > 0.6:
        print("  高频（>2kHz 齿轮啸叫）占大头——滤波无解：低通/带限会连清辅音"
              "（小蜘蛛的x、停的t）一起砍，09-03 实测 ASR 反而更糊；高频方向性强"
              "衰减快，物理杠杆=挪远+麦克风朝人+与舵机之间加泡棉挡板（减振没用，"
              "低频占比才这点）。软件出路=走动会话用 --no-wake+声纹锁：09-03 实测"
              "纯舵机噪声 VAD 零切句不会乱出指令，且 ASR 常常听得懂（KWS 小模型"
              "先倒下），唤醒这一步跳过就好")
    else:
        print("  噪声分布平：先做物理（挪位/朝向），再回来复测")


def _rec(card, secs, out):
    import numpy as np
    src = ArecordSource(alsa_device(card))
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
    return audio


def do_noise_test(paths, card):
    """走路噪声下"要喊很大声"的归因（09-03 实机反馈）：
    A 纯噪声底 → B 带噪正常音量说话 → C 静止同距离同音量说话。
    判据：B 说话段与 A 噪声底的差 = 实际信噪比（<10dB 基本靠喊）；
    B 与 C 的说话段电平差 = AGC 嫌疑（被持续噪声压了增益会差很多）。"""
    print("走路噪声测试：三段录音，全程别动麦克风、人站平时指挥的位置。\n"
          "⚠ 噪声源用 walk_teleop（纯键盘）——voice_teleop/voice_climb 会占着"
          "麦克风，测试期间不能开。")
    input("A) 让机器人走起来（另开终端 walk_teleop，或晃动通电的腿）——"
          "人别说话，回车录 4s 纯噪声…")
    na = _rec(card, 4.0, "/tmp/voice_noise_a.wav")
    input("B) 保持机器人走动，回车后用**平时音量**说“小蜘蛛，前进三秒”（录 5s）…")
    nb = _rec(card, 5.0, "/tmp/voice_noise_b.wav")
    input("C) 让机器人停下站住，同距离同音量再说一遍（录 5s）…")
    nc = _rec(card, 5.0, "/tmp/voice_noise_c.wav")

    noise = _frame_dbfs(na, 50)
    sp_n = _frame_dbfs(nb, 90)
    sp_q = _frame_dbfs(nc, 90)
    snr = sp_n - noise
    agc_drop = sp_q - sp_n
    print(f"\n噪声底（走动，中位帧）      {noise:6.1f} dBFS")
    print(f"带噪说话段（90 分位帧）      {sp_n:6.1f} dBFS   → 信噪比 ≈ {snr:.1f} dB")
    print(f"静止说话段（90 分位帧）      {sp_q:6.1f} dBFS")
    print("判读：")
    if snr < 10.0:
        print("  · 信噪比 <10dB：以掩蔽为主——杠杆只有物理：麦克风板远离舵机、"
              "垫泡棉减振（结构传导比空气声更毒）、麦克风面朝人"
              "（USB 固件不给主机侧录音增益，amixer 无旋钮，09-03 实机证实）")
        print("  · 判别主攻方向：把麦克风板拆下拎在手里、机器人照走，重录 A 段"
              "——噪声底大降（如 -18→-30）=结构传导为主攻减振悬浮贴装；"
              "降得少=空气声为主攻距离和朝向。目标：走动噪声底 < -30dBFS")
    else:
        print("  · 信噪比尚可：若唤醒仍难，多半是阈值——确认已用新词表"
              "（唤醒 0.20:1.5，改完要重新生成 keywords_hexapod.txt）")
    if agc_drop > 6.0:
        print(f"  · 带噪说话比静止低 {agc_drop:.1f}dB：板载 AGC 疑似被持续噪声压了"
              "增益——把人和麦克风的距离缩短一半试；AGC 参数在固件里，暂无旋钮")
    elif agc_drop < -3.0:
        print(f"  · 带噪说话反而比静止高 {-agc_drop:.1f}dB：噪声能量叠加 + 人在噪声"
              "里自然提嗓（Lombard），无 AGC 压制迹象——固件没在害你")
    do_noise_analyze(["/tmp/voice_noise_a.wav", "/tmp/voice_noise_b.wav",
                      "/tmp/voice_noise_c.wav"])
    print("\n对 B 段跑识别（不要求唤醒，KWS 命中会照常显示 wake 事件；"
          "看 ASR 在噪声里到底听到什么）：")
    do_asr(paths, "/tmp/voice_noise_b.wav", wake_required=False)
    print("三段录音存 /tmp/voice_noise_{a,b,c}.wav，可拷回开发机细看频谱")


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
    ap.add_argument("--noise-test", action="store_true",
                    help="走路噪声测试：量化噪声底/信噪比/AGC 嫌疑（另开终端让机器人走）")
    ap.add_argument("--noise-analyze", nargs="*", metavar="WAV",
                    help="对已录的噪声测试 wav 分频段分析，不重录"
                         "（默认 /tmp/voice_noise_{a,b,c}.wav）")
    ap.add_argument("--out", default="/tmp/voice_check.wav")
    ap.add_argument("--models")
    ap.add_argument("--card")
    args = ap.parse_args()
    specific = any([args.list, args.record, args.asr, args.tts, args.echo_test,
                    args.noise_test, args.noise_analyze is not None])

    card = args.card
    if args.list or not specific:
        card = card or do_list()
    else:
        card = card or find_card()

    paths = None
    if args.asr or args.tts or args.echo_test or args.noise_test or not specific:
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
    if args.noise_test:
        if card is None:
            sys.exit("没有 ReSpeaker Lite 声卡，--noise-test 要实机跑。")
        do_noise_test(paths, card)
    if args.noise_analyze is not None:
        do_noise_analyze(args.noise_analyze or
                         ["/tmp/voice_noise_a.wav", "/tmp/voice_noise_b.wav",
                          "/tmp/voice_noise_c.wav"])

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
