#!/usr/bin/env python3
"""声纹注册 / 验证（声纹锁的档案由这里生成，用法见 docs/VOICE-GUIDE.md §3.8）。

  python voice_enroll.py                  # 对着麦克风读 5 段提示语 → 注册档案
  python voice_enroll.py --segs 6 --secs 5
  python voice_enroll.py --wav a.wav b.wav …   # 用现成录音注册（每个至少 2s 人声）
  python voice_enroll.py --test           # 录 4s 和档案比对（换别人说话试试拒不拒）
  python voice_enroll.py --test --wav x.wav    # 对 wav 打分

档案默认存 <模型根>/voiceprint_owner.npz（$HEXAPOD_VOICEPRINT 可改），
里面带按注册段自相似度算出的建议阈值；voice_teleop 有档案就自动开声纹锁。
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.voice.audio import ArecordSource, alsa_device, find_card, read_wav, write_wav
from hexapod.voice.engine import ModelPaths
from hexapod.voice.voiceprint import (VoiceGate, best_score, default_profile, embed,
                                      make_extractor, save_profile)

RATE = 16000
PROMPTS = ("小蜘蛛，前进三秒",
           "向左转两秒，再往右挪一点",
           "停下，别动，站起来",
           "今天天气不错，我们去爬墙吧",
           "电压多少，换成三角步态")


def record(card, secs: float, save_to=None):
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
    if save_to:
        write_wav(save_to, audio, RATE)
    return audio


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--segs", type=int, default=5, help="注册段数（默认 5）")
    ap.add_argument("--secs", type=float, default=4.0, help="每段秒数（默认 4）")
    ap.add_argument("--wav", nargs="+", help="用 wav 注册/测试，不录音")
    ap.add_argument("--test", action="store_true", help="和已有档案比对，不注册")
    ap.add_argument("--out", help="档案路径（默认 <模型根>/voiceprint_owner.npz）")
    ap.add_argument("--name", default="owner")
    ap.add_argument("--models")
    ap.add_argument("--card")
    args = ap.parse_args()

    import numpy as np
    paths = ModelPaths.discover(args.models)
    if paths.spk_model is None:
        sys.exit("模型根目录下没有声纹模型（3dspeaker_*campplus*.onnx）——"
                 "重跑 scripts/voice_setup.sh models 补下载")
    profile = args.out or default_profile(args.models)
    card = args.card or find_card()

    if args.test:
        gate = VoiceGate(paths.spk_model, profile, log=print)
        if args.wav:
            for w in args.wav:
                ok, sc = gate.accept(*read_wav(w))
                print(f"{w}: 声纹 {sc:.3f} 阈值 {gate.threshold:.2f} → "
                      f"{'✓ 是 ' + gate.name if ok else '✗ 不是主人'}")
        else:
            print(f"录 4s，请说话（档案 {profile}，阈值 {gate.threshold:.2f}）…")
            audio = record(card, 4.0, save_to="/tmp/voice_enroll_test.wav")
            ok, sc = gate.accept(audio)
            print(f"声纹 {sc:.3f} → {'✓ 是 ' + gate.name if ok else '✗ 不是主人'}"
                  "   （换个人再跑一次，看会不会被拒）")
        return

    # ---- 注册 ----
    ext = make_extractor(paths.spk_model)
    embs = []
    if args.wav:
        for w in args.wav:
            audio, _ = read_wav(w)
            embs.append(embed(ext, audio))
            print(f"  {w}: {len(audio)/RATE:.1f}s ✓")
    else:
        print(f"声纹注册：读 {args.segs} 段提示语，每段录 {args.secs:.0f} 秒，"
              "从头说到尾别停顿太久。\n")
        i = 0
        while i < args.segs:
            prompt = PROMPTS[i % len(PROMPTS)]
            input(f"[{i+1}/{args.segs}] 请读：“{prompt}”  —— 按回车开始录音")
            audio = record(card, args.secs)
            peak = float(np.abs(audio).max()) if len(audio) else 0.0
            if peak < 0.02:
                print(f"  ⚠ 峰值 {peak:.3f} 太小（没录到？），这段重来")
                continue
            embs.append(embed(ext, audio))
            print(f"  ✓ 峰值 {peak:.3f}")
            i += 1

    if len(embs) < 2:
        sys.exit("至少要 2 段才能注册（自相似度没法算）")
    E = np.stack(embs)
    sims = [best_score(np.delete(E, i, axis=0), E[i]) for i in range(len(E))]
    self_min, self_mean = min(sims), sum(sims) / len(sims)
    threshold = round(min(0.60, max(0.35, self_min - 0.15)), 2)
    save_profile(profile, E, threshold, args.name)
    print(f"\n注册段自相似度：最低 {self_min:.3f} 平均 {self_mean:.3f}"
          f"（低于 0.55 说明录音质量差/环境吵，建议重录）")
    print(f"档案 → {profile}（{len(E)} 段，建议阈值 {threshold}）")
    print("下一步：voice_enroll.py --test 自己和别人各验一次；"
          "voice_teleop.py 检测到档案会自动开声纹锁（急停不拦）")


if __name__ == "__main__":
    main()
