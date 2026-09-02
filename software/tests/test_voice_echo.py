"""自听回声过滤（engine.looks_like_echo）与唤醒词整句识别的忽略。

背景（09-02 实机）：板载 AEC 没压住喇叭→麦克风回声，"电压7.4伏电流1.0安"的
回答被再次识别成 status 指令，机器人自问自答死循环。
"""
from hexapod.voice.engine import looks_like_echo
from hexapod.voice.intents import INTRO_REPLY, parse


def test_wake_word_text_ignored():
    assert parse("小蜘蛛。").kind == "ignore"
    assert parse("蜘蛛同学").kind == "ignore"
    assert parse("小蜘蛛。").reply == ""


def test_echo_exact_and_punctuation():
    assert looks_like_echo("没听懂。", ["没听懂"])
    assert looks_like_echo("电压7.4伏电流1.0安。", ["电压7.4伏，电流1.0安"])
    assert looks_like_echo("在", ["在"])


def test_echo_misrecognized_variants():
    # 回声被听歪也要能挡住（实机日志里的真实例子）
    assert looks_like_echo("电压7.4伏电容1.0N。", ["电压7.4伏，电流1.0安"])
    assert looks_like_echo("前进三秒", ["前进3秒"])


def test_real_speech_not_dropped():
    assert not looks_like_echo("电压多少", ["电压7.4伏，电流1.0安"])
    assert not looks_like_echo("停下", ["停"])       # 短回话只认全等：真急停不能被吃
    assert not looks_like_echo("前进三秒", ["在"])
    assert not looks_like_echo("后退两秒", ["前进3秒"])
    assert not looks_like_echo("小蜘蛛", [])


def test_intro_intent():
    assert parse("自我介绍").kind == "intro"
    assert parse("你是谁。").kind == "intro"
    assert parse("自我介绍").reply == INTRO_REPLY
    assert parse("介绍一下三角步态").kind == "gait"      # 步态优先于介绍
    for w in ("停下", "停止", "停下来", "别动"):          # 长回话不能含 KWS 急停词
        assert w not in INTRO_REPLY


def test_long_reply_echo_segments():
    # 长回话被 VAD 切成片段回来（自我介绍场景）
    assert looks_like_echo("六个真空吸盘管吸墙", [INTRO_REPLY])      # 原文片段
    assert looks_like_echo("六个真空锡盘管西墙。", [INTRO_REPLY])    # 听歪的片段
    assert not looks_like_echo("停下", [INTRO_REPLY])               # 真急停不吃
    assert not looks_like_echo("向左转两秒", [INTRO_REPLY])         # 真指令不吃
