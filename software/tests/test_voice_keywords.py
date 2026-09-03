import pytest

from hexapod.voice.keywords import parse_result, load_raw, DEFAULT_KEYWORDS


def test_parse_result_roundtrip():
    assert parse_result("WAKE_小蜘蛛") == ("WAKE", "小蜘蛛")
    assert parse_result("STOP_停下来") == ("STOP", "停下来")
    assert parse_result("小爱同学") == ("", "小爱同学")     # 模型自带样例表没有类别前缀


def test_load_raw_parses_tags_thresholds_boosts(tmp_path):
    f = tmp_path / "kw.txt"
    f.write_text("# 注释\n\n小蜘蛛 @WAKE #0.25\n停下 @STOP #0.20 :2.0\n你好机器人\n",
                 encoding="utf-8")
    assert load_raw(f) == [("小蜘蛛", "WAKE", 0.25, None), ("停下", "STOP", 0.20, 2.0),
                           ("你好机器人", "WAKE", None, None)]


def test_default_keywords_have_wake_and_stop():
    tags = {t for _, t, _, _ in DEFAULT_KEYWORDS}
    assert tags == {"WAKE", "STOP"}
    assert all(len(w) >= 2 for w, _, _, _ in DEFAULT_KEYWORDS)   # 单字词误触多，禁止
    for _, tag, th, bo in DEFAULT_KEYWORDS:
        if tag == "STOP":
            assert th <= 0.25 and bo
        if tag == "WAKE":                # 09-03 走路噪声：唤醒也放灵敏、带提升分
            assert th <= 0.20 and bo
    # 纪律：唤醒提升分必须压着急停一档——急停在任何混叠里都要先活下来
    wake_bo = max(bo for _, t, _, bo in DEFAULT_KEYWORDS if t == "WAKE")
    stop_bo = min(bo for _, t, _, bo in DEFAULT_KEYWORDS if t == "STOP")
    assert wake_bo < stop_bo


def test_ppinyin_matches_sherpa_text2token_format():
    pytest.importorskip("pypinyin")
    from hexapod.voice.keywords import ppinyin, keyword_line
    # 与 sherpa-onnx 文档示例一致："文森特卡索" → "w én s ēn t è k ǎ s uǒ"
    assert ppinyin("文森特卡索") == ["w", "én", "s", "ēn", "t", "è", "k", "ǎ", "s", "uǒ"]
    assert ppinyin("小蜘蛛") == ["x", "iǎo", "zh", "ī", "zh", "ū"]
    assert ppinyin("安") == ["ān"]                        # 零声母
    line = keyword_line("小蜘蛛", "WAKE", threshold=0.25)
    assert line == "x iǎo zh ī zh ū #0.25 @WAKE_小蜘蛛"
    with pytest.raises(ValueError):
        keyword_line("小蜘蛛", "WAKE", tokens={"x", "iǎo"})   # 词表缺 token 要报错
