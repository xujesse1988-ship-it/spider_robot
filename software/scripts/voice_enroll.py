#!/usr/bin/env python3
"""声纹注册 / 验证（声纹锁的档案由这里生成，用法见 docs/VOICE-GUIDE.md §3.8）。

  python voice_enroll.py                  # 读 5 段提示语 + 逐个读短词 → 注册档案
  python voice_enroll.py --segs 6 --secs 5
  python voice_enroll.py --wav a.wav b.wav …   # 用现成录音注册（每个至少 2s 人声）
  python voice_enroll.py --append         # 只补录短词，追加进已有档案（阈值不动）
  python voice_enroll.py --append --wav 退出.wav   # 用现成短词录音追加
  python voice_enroll.py --test           # 录 4s 和档案比对（换别人说话试试拒不拒）
  python voice_enroll.py --test --wav x.wav    # 对 wav 打分

档案默认存 <模型根>/voiceprint_owner.npz（$HEXAPOD_VOICEPRINT 可改），
里面带按注册段自相似度算出的建议阈值；voice_teleop 有档案就自动开声纹锁。

短词补录是为"确认/退出"这类 0.5s 指令：声纹模型对 <1s 的音频受说话内容
影响明显（内容不同压低分数），注册里放上同词的短声纹，实战说这个词就有
逐词对上的锚点。急停词（停下/别动）不补——急停永不做声纹，补了没用。
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.voice.audio import ArecordSource, alsa_device, find_card, read_wav, write_wav
from hexapod.voice.engine import ModelPaths
from hexapod.voice.voiceprint import (VoiceGate, append_profile, best_score,
                                      default_profile, embed, make_extractor,
                                      save_profile, trim_voiced)

RATE = 16000
WIN_S = 1.5      # 除整段外，再按 1.5s 子窗提声纹：短指令（"退出"0.5s）对短窗
                 # 的相似度远高于对 4s 整段，缓解长短失配的误拒
PROMPTS = ("小蜘蛛，前进三秒",
           "向左转两秒，再往右挪一点",
           "停下，别动，站起来",
           "今天天气不错，我们去爬墙吧",
           "电压多少，换成三角步态")
# 声纹锁会拦的高频短词（急停词不在此列：永不拦，录了白录）
SHORT_PROMPTS = ("确定", "确认", "退出", "好的")
SHORT_SECS = 2.0          # 短词录 2s，取人声区间提声纹
SHORT_MIN_S = 0.25        # 裁完短于这个就是没录到词


def seg_embeddings(ext, audio):
    """一段录音 → [整段声纹] + [有人声的 1.5s 子窗声纹]。"""
    import numpy as np
    embs = [embed(ext, audio)]
    win = int(WIN_S * RATE)
    for k in range(len(audio) // win):
        w = audio[k * win:(k + 1) * win]
        if float(np.sqrt((w ** 2).mean())) > 0.01:      # 纯静音窗不要
            embs.append(embed(ext, w))
    return embs


def record_short_words(card, ext, words=SHORT_PROMPTS):
    """逐个读短词 → 声纹列表。每个录 SHORT_SECS 秒，裁出人声区间再提声纹
    （不裁的话半秒的词淹在 1.5s 静音里，锚点本身就是稀的）。"""
    import numpy as np
    out = []
    for w in words:
        while True:
            input(f"  请读短词：“{w}”  —— 按回车开始录音（{SHORT_SECS:.0f} 秒）")
            seg = trim_voiced(record(card, SHORT_SECS))
            dur = len(seg) / RATE
            peak = float(np.abs(seg).max()) if len(seg) else 0.0
            if peak < 0.02 or dur < SHORT_MIN_S:
                print(f"    ⚠ 没录到（人声段 {dur:.2f}s 峰值 {peak:.3f}），这个词重来")
                continue
            out.append(embed(ext, seg))
            print(f"    ✓ 人声段 {dur:.2f}s")
            break
    return out


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
    ap.add_argument("--wav", nargs="+", help="用 wav 注册/测试/追加，不录音")
    ap.add_argument("--test", action="store_true", help="和已有档案比对，不注册")
    ap.add_argument("--append", action="store_true",
                    help="只补录短词追加进已有档案（阈值不动），不整套重录")
    ap.add_argument("--no-short", action="store_true", help="注册时跳过短词补录环节")
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
                audio, r = read_wav(w)
                dur = len(audio) / r
                ok, sc = gate.accept(audio, r)
                print(f"{w}: {dur:.1f}s 声纹 {sc:.3f} 阈值 {gate.effective_threshold(dur):.2f}"
                      f" → {'✓ 是 ' + gate.name if ok else '✗ 不是主人'}")
        else:
            print(f"录 4s，请说话（档案 {profile}，阈值 {gate.threshold:.2f}）…")
            audio = record(card, 4.0, save_to="/tmp/voice_enroll_test.wav")
            ok, sc = gate.accept(audio)
            print(f"声纹 {sc:.3f} → {'✓ 是 ' + gate.name if ok else '✗ 不是主人'}"
                  "   （换个人再跑一次，看会不会被拒）")
        return

    if args.append:
        import os
        if not os.path.exists(profile):
            sys.exit(f"{profile} 不存在——先跑一次完整注册再 --append")
        ext = make_extractor(paths.spk_model)
        if args.wav:
            new = []
            for w in args.wav:
                audio, _ = read_wav(w)
                seg = trim_voiced(audio)
                new.append(embed(ext, seg))
                print(f"  {w}: 人声段 {len(seg)/RATE:.2f}s ✓")
        else:
            print("短词补录：逐个读下面的词，追加进现有档案（阈值不变）。\n")
            new = record_short_words(card, ext)
        total = append_profile(profile, np.stack(new))
        print(f"\n追加 {len(new)} 条 → {profile}（共 {total} 条声纹）")
        print("验证：voice_enroll.py --test，说一个短词（比如“退出”）看分数")
        return

    # ---- 注册 ----
    ext = make_extractor(paths.spk_model)
    full_embs, all_embs = [], []            # 整段（算自相似度用）/ 整段+子窗（存档案）
    if args.wav:
        for w in args.wav:
            audio, _ = read_wav(w)
            es = seg_embeddings(ext, audio)
            full_embs.append(es[0])
            all_embs.extend(es)
            print(f"  {w}: {len(audio)/RATE:.1f}s ✓（{len(es)} 个声纹）")
    else:
        print(f"声纹注册：读 {args.segs} 段提示语，每段录 {args.secs:.0f} 秒，"
              "从头说到尾别停顿太久；用什么距离/环境指挥机器人，就在什么条件下录。\n")
        i = 0
        while i < args.segs:
            prompt = PROMPTS[i % len(PROMPTS)]
            input(f"[{i+1}/{args.segs}] 请读：“{prompt}”  —— 按回车开始录音")
            audio = record(card, args.secs)
            peak = float(np.abs(audio).max()) if len(audio) else 0.0
            if peak < 0.02:
                print(f"  ⚠ 峰值 {peak:.3f} 太小（没录到？），这段重来")
                continue
            es = seg_embeddings(ext, audio)
            full_embs.append(es[0])
            all_embs.extend(es)
            print(f"  ✓ 峰值 {peak:.3f}（{len(es)} 个声纹）")
            i += 1
        if not args.no_short:
            print("\n再补录几个高频短词——“确认/退出”这类半秒指令实战分数最低，"
                  "给它们各留一个同词锚点：")
            all_embs.extend(record_short_words(card, ext))

    if len(full_embs) < 2:
        sys.exit("至少要 2 段才能注册（自相似度没法算）")
    F = np.stack(full_embs)
    sims = [best_score(np.delete(F, i, axis=0), F[i]) for i in range(len(F))]
    self_min, self_mean = min(sims), sum(sims) / len(sims)
    # 上限 0.50：注册是同场同长度的录音，自相似度偏乐观；实战短指令分数会低一截
    threshold = round(min(0.50, max(0.35, self_min - 0.15)), 2)
    save_profile(profile, np.stack(all_embs), threshold, args.name)
    print(f"\n注册段自相似度：最低 {self_min:.3f} 平均 {self_mean:.3f}"
          f"（低于 0.55 说明录音质量差/环境吵，建议重录）")
    print(f"档案 → {profile}（整段 {len(F)} + 子窗/短词共 {len(all_embs)} 个声纹，"
          f"阈值 {threshold}；短于 0.7s 的极短指令自动再降 0.05）")
    print("下一步：voice_enroll.py --test 自己和别人各验一次；"
          "voice_teleop.py 检测到档案会自动开声纹锁（急停不拦）")


if __name__ == "__main__":
    main()
