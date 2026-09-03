import math

import pytest

from hexapod.voice.intents import (parse, parse_climb, cn_to_number,
                                   parse_duration, normalize)


def test_normalize_strips_punct_and_fullwidth():
    assert normalize("前进 ３ 秒。") == "前进3秒"
    assert normalize("  停！  ") == "停"


@pytest.mark.parametrize("s,n", [
    ("三", 3), ("3", 3), ("3.5", 3.5), ("十", 10), ("十五", 15), ("二十", 20),
    ("二十五", 25), ("两", 2), ("半", 0.5), ("零", 0),
])
def test_cn_to_number(s, n):
    assert cn_to_number(s) == n


def test_cn_to_number_rejects_garbage():
    assert cn_to_number("十十") is None
    assert cn_to_number("abc") is None
    assert cn_to_number("") is None


def test_parse_duration_variants():
    assert parse_duration("前进三秒") == 3
    assert parse_duration("前进3秒") == 3
    assert parse_duration("前进5秒钟") == 5
    assert parse_duration("走两步") == 2.0
    assert parse_duration("走几步") == 3.0
    assert parse_duration("向前走半分钟") == 30
    assert math.isinf(parse_duration("一直前进"))
    assert parse_duration("前进") is None


# ---- 安全优先 ----
@pytest.mark.parametrize("txt", ["停", "停下", "停下来。", "快停下", "别动",
                                 "站住", "不要动", "一直走别停"])
def test_stop_wins_over_everything(txt):
    assert parse(txt).kind == "stop"


def test_confirm_cancel_exact_only():
    assert parse("确定").kind == "confirm"
    assert parse("好的。").kind == "confirm"
    assert parse("取消").kind == "cancel"
    assert parse("确定前进").kind == "walk"      # 不是纯确认词就不算确认


def test_exit_needs_confirm_reply():
    it = parse("退出程序")
    assert it.kind == "exit" and "确定" in it.reply


# ---- 动作 ----
def test_forward_with_seconds_itn_digits():
    it = parse("前进3秒。")                     # SenseVoice use_itn 会把三变 3
    assert (it.kind, it.vx, it.vy, it.wz, it.seconds) == ("walk", 1, 0, 0, 3)
    assert it.reply == "前进3秒"


def test_forward_without_duration_is_continuous():
    it = parse("往前走")
    assert it.kind == "walk" and it.vx == 1 and it.seconds is None
    assert it.reply == "一直前进，说停就停"       # 没说时长=连续动作，回复要念明白
    for w in ("停下", "停止", "停下来", "别动"):  # 回复不能含 KWS 急停词（念了自触发）
        assert w not in it.reply


def test_backward_and_speed_modifier():
    it = parse("快点后退")
    assert it.vx == -1 and it.speed == 1.5
    assert parse("慢慢往后退").speed == 0.6


def test_bare_speed_words_adjust_pace():
    it = parse("快点。")                        # 光杆调速：连续走动中提速/减速
    assert (it.kind, it.speed, it.reply) == ("speed", 1.5, "提速")
    assert parse("慢一点").speed == 0.6
    assert parse("太快了").kind == "speed" and parse("太快了").speed == 0.6
    assert parse("太慢了").speed == 1.5
    assert parse("快点前进").kind == "walk"     # 带方向的仍归 walk，别被调速抢


def test_turn_beats_strafe():
    assert parse("左转").wz == 1 and parse("左转").vy == 0
    assert parse("向左转").wz == 1
    assert parse("往右拐").wz == -1
    assert parse("顺时针转").wz == -1


def test_strafe():
    it = parse("向左走两步")
    assert it.vy == 1 and it.vx == 0 and it.seconds == 2.0
    assert parse("右移").vy == -1
    assert parse("左").vy == 1


def test_forever_walk_reply_mentions_stop():
    it = parse("一直前进")
    assert math.isinf(it.seconds) and "停" in it.reply


# ---- 姿态 / 步态 / 状态 / 问候 ----
def test_posture_and_gait():
    assert parse("站起来").kind == "stand"
    assert parse("趴下").kind == "crouch"
    assert parse("换三角步态").gait == "tripod"
    assert parse("波浪步态").gait == "wave"


def test_status_and_greet_and_unknown():
    assert parse("电压多少").kind == "status"
    assert parse("低压多少").kind == "status"       # "电压"的高频听歪（实测收编）
    assert parse("你好").kind == "greet"
    assert parse("跳个舞").kind == "unsupported"
    assert parse("今天天气不错").kind == "unknown"
    assert parse("").kind == "unknown"


# ---- 爬墙口径 parse_climb（voice_climb.py 用）----
def test_climb_words():
    assert parse_climb("单步").kind == "step"
    assert parse_climb("抬腿。").kind == "step"
    assert parse_climb("落地").kind == "land"
    assert parse_climb("踩下").kind == "land"
    assert parse_climb("解除冻结").kind == "unfreeze"
    assert parse_climb("接出冻结").kind == "unfreeze"  # "解除"高频听歪，认"冻结"兜住
    assert parse_climb("开始吸附").kind == "begin"
    assert parse_climb("启动").kind == "begin"      # "吸附"易听歪，"启动"是硬朗备份
    assert parse_climb("取机").kind == "pickup"
    assert parse_climb("放开吸盘").kind == "pickup"


def test_climb_stop_beats_climb_words():
    # 急停永远第一优先：句里混进爬墙词也得停
    assert parse_climb("别动，先别抬腿").kind == "stop"
    assert parse_climb("停下单步").kind == "stop"


def test_climb_falls_back_to_parse():
    it = parse_climb("前进三秒")
    assert it.kind == "walk" and it.vx == 1 and it.seconds == 3
    assert parse_climb("电压多少").kind == "status"
    # "开始前进"必须是 walk 不是 begin（begin 词表故意不收裸"开始"）
    assert parse_climb("开始前进").kind == "walk"
    # 已知边界："走一步"走通用语法=1 秒定时行走，不是单步（模块注释有记）
    it = parse_climb("走一步")
    assert it.kind == "walk" and it.seconds == 1.0


def test_climb_action_replies_avoid_kws_stop_words():
    # 爬墙回复/播报纪律：机器人念的话不得含 KWS 急停词
    for text in ("单步", "落地", "解冻", "开始吸附"):
        reply = parse_climb(text).reply
        for w in ("停下", "停止", "停下来", "别动"):
            assert w not in reply
