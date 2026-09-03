"""唤醒词 / 急停词 → sherpa-onnx KWS 关键词文件（部分拼音 ppinyin 格式）。

分词逻辑照抄 sherpa_onnx/utils.py 的 text2token(tokens_type="ppinyin")：每个字
→ pypinyin 带调拼音 → 拆成声母 + 带调韵母（"小"→"x iǎo"，"安"→"ān"），
但这里只依赖 pypinyin，不用装 sentencepiece / click。

关键词文件每行：`token token ... [#阈值] [:提升分] @类别_原词`，例如
    x iǎo zh ī zh ū #0.25 @WAKE_小蜘蛛
KWS 命中时 get_result() 返回 '@' 后面的名字，parse_result() 拆回 (类别, 原词)。

类别约定：WAKE = 唤醒词（进入听令状态）；STOP = 急停词（不需唤醒，随时生效）。
急停词故意不收单字"停"——单音节误触太多，单字"停"走唤醒后的整句识别。
急停词阈值比唤醒低、并带提升分：安全方向是"多停"，且要能从机器人自己
说话的回声里把用户的喊声捞出来（09-02 实机：0.35 时说话期间喊"停下"不命中）。

原始词表文件（scripts/voice_setup.sh 用 keywords_raw.txt 生成）：
    # 井号开头整行是注释
    小蜘蛛 @WAKE #0.25
    停下 @STOP #0.20 :2.0
命令行：python -m hexapod.voice.keywords --tokens <kws 模型>/tokens.txt --out keywords.txt
"""
import argparse
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

Entry = Tuple[str, str, Optional[float], Optional[float]]   # (词, 类别, 阈值, 提升分)

DEFAULT_KEYWORDS: List[Entry] = [
    ("小蜘蛛", "WAKE", 0.20, 1.5),      # 走路舵机噪声下 0.25 要喊很大声（09-03 实机）
    ("蜘蛛同学", "WAKE", 0.20, 1.5),
    ("停下", "STOP", 0.20, 2.0),
    ("停止", "STOP", 0.20, 2.0),
    ("停下来", "STOP", 0.20, 2.0),
    ("别动", "STOP", 0.20, 2.0),
]


def ppinyin(text: str) -> List[str]:
    from pypinyin import pinyin
    from pypinyin.contrib.tone_convert import to_initials, to_finals_tone

    out: List[str] = []
    for x in (p[0] for p in pinyin(text)):
        initial = to_initials(x, strict=False)
        final = to_finals_tone(x, strict=False)
        if not initial and not final:
            out.append(x)
            continue
        if initial:
            out.append(initial)
        if final:
            out.append(final)
    return out


def load_tokens(tokens_txt) -> set:
    toks = set()
    for line in Path(tokens_txt).read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if parts:
            toks.add(parts[0])
    return toks


def keyword_line(word: str, tag: str, threshold: Optional[float] = None,
                 boost: Optional[float] = None, tokens: Optional[set] = None) -> str:
    toks = ppinyin(word)
    if tokens is not None:
        bad = [t for t in toks if t not in tokens]
        if bad:
            raise ValueError(f"关键词 {word!r} 含模型词表没有的 token {bad}，换个词")
    parts = list(toks)
    if boost is not None:
        parts.append(f":{boost}")
    if threshold is not None:
        parts.append(f"#{threshold}")
    parts.append(f"@{tag}_{word}")
    return " ".join(parts)


def build(entries: Iterable[Entry], tokens_txt) -> str:
    tokens = load_tokens(tokens_txt)
    lines = [keyword_line(w, tag, th, boost=bo, tokens=tokens)
             for w, tag, th, bo in entries]
    return "\n".join(lines) + "\n"


def load_raw(path) -> List[Entry]:
    """原始词表：`词 @类别 [#阈值] [:提升分]`，'#' 开头整行注释，空行忽略。"""
    entries: List[Entry] = []
    for raw in Path(path).read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        word, tag, th, bo = parts[0], "WAKE", None, None
        for p in parts[1:]:
            if p.startswith("@"):
                tag = p[1:]
            elif p.startswith("#"):
                th = float(p[1:])
            elif p.startswith(":"):
                bo = float(p[1:])
        entries.append((word, tag, th, bo))
    return entries


def parse_result(name: str) -> Tuple[str, str]:
    """'WAKE_小蜘蛛' → ('WAKE', '小蜘蛛')；没有类别前缀时类别为 ''。"""
    name = (name or "").strip()
    if "_" in name:
        tag, word = name.split("_", 1)
        return tag, word
    return "", name


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="生成 sherpa-onnx KWS 关键词文件")
    ap.add_argument("--tokens", required=True, help="KWS 模型目录里的 tokens.txt")
    ap.add_argument("--out", required=True, help="输出 keywords.txt")
    ap.add_argument("--raw", help="原始词表（默认用内置 DEFAULT_KEYWORDS）")
    args = ap.parse_args(argv)
    entries = load_raw(args.raw) if args.raw else DEFAULT_KEYWORDS
    text = build(entries, args.tokens)
    Path(args.out).write_text(text, encoding="utf-8")
    print(text, end="")
    print(f"→ {args.out}（{len(entries)} 条）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
