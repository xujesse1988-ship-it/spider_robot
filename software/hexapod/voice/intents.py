"""语音文本 → 机器人意图（纯规则，零三方依赖，开发机 pytest 可测）。

输入是 SenseVoice 的识别文本（use_itn=True 时中文数字会变阿拉伯数字，句尾
常带"。"），先归一化，再按"先安全后动作"的顺序匹配：

  1. 停——句子里任何位置出现 停/别动/站住 → 立即停，其余一概不看
  2. 确认/取消——只给"退出"做二次确认用
  3. 退出 / 状态 / 站立 / 趴下 / 步态 / 问候 / 不支持的（跳舞）
  4. 移动：转向 > 平移 > 前后（"左转"含"左"，必须先判转向）
     可带时长（"三秒"/"5 秒"/"两步"/"一直"）和快慢（"快点"/"慢点"）

vx/vy/wz 只给 ±1 的方向，seconds=None 表示"没说时长"，inf 表示"一直"。
真实速度、时长语义由调用方（scripts/voice_teleop.py）决定——那边把
"没说时长"当"一直走"（连续动作，说停为止），reply 也按这个念。
"""
import math
import re
from dataclasses import dataclass
from typing import Optional

STEP_SECONDS = 1.0        # "走两步"：一步折算的秒数

_PUNCT = re.compile(r"[\s，。、！？!?,.:：;；\"'“”‘’()（）\[\]【】…~～\-—]+")
_FULLWIDTH = str.maketrans("０１２３４５６７８９", "0123456789")
_CN_NUM = {"零": 0, "〇": 0, "一": 1, "幺": 1, "二": 2, "两": 2, "俩": 2,
           "三": 3, "四": 4, "五": 5, "六": 6, "七": 7, "八": 8, "九": 9}
_NUM_RE = r"(\d+(?:\.\d+)?|[零〇一幺二两俩三四五六七八九十]+|半)"
_DUR_RE = re.compile(_NUM_RE + r"(秒钟|秒|s|分钟|分)")
_STEP_RE = re.compile(r"(\d+|[一二两俩三四五六七八九十]+|几)步")

WAKE_WORDS = ("小蜘蛛", "蜘蛛同学")   # 与 keywords_raw.txt 保持一致
STOP_WORDS = ("停", "别动", "站住", "不要动", "别走", "别跑", "定住", "stop")
INTRO_WORDS = ("自我介绍", "介绍一下", "介绍自己", "你是谁", "你叫什么")
# 长回话（≈15s）。措辞注意：不能含 KWS 急停词（停下/停止/停下来/别动），
# 否则机器人念到那儿自己触发急停；含"小蜘蛛"没事——引擎在说话期间忽略唤醒。
INTRO_REPLY = ("我是小蜘蛛，一台会爬墙的六足机器人。十八个舵机管走路，"
               "六个真空吸盘管吸墙。你可以让我前进、后退、平移、转向，"
               "也可以问我电压。想让我停，喊一声就行。")
CONFIRM_WORDS = {"确定", "确认", "是的", "是", "对", "对的", "好的", "好", "可以",
                 "嗯", "没错", "yes", "ok"}
CANCEL_WORDS = {"取消", "不用", "不用了", "算了", "不要", "不", "no"}
EXIT_WORDS = ("退出", "结束程序", "关闭程序", "关机", "下线")
STATUS_WORDS = ("电压", "电量", "电池", "几伏", "状态", "电流", "多少电")
STAND_WORDS = ("站起来", "站起", "站立", "起立", "站好", "起来", "立正")
CROUCH_WORDS = ("趴下", "蹲下", "坐下", "休息", "趴着", "卧倒", "趴")
GREET_WORDS = ("你好", "您好", "嗨", "哈喽", "哈罗", "在吗", "在不在", "hello", "hi",
               "早上好", "晚上好")
DANCE_WORDS = ("跳舞", "跳个舞", "跳支舞")

_TURN_L = re.compile(r"(向|往|朝)?左(转|拐)|转左|逆时针|左旋|转身|掉头|转个圈|转一圈|转弯")
_TURN_R = re.compile(r"(向|往|朝)?右(转|拐)|转右|顺时针|右旋")
_STRAFE_L = re.compile(r"(向|往|朝|靠)左|左移|左平移|左边|左挪|^左$")
_STRAFE_R = re.compile(r"(向|往|朝|靠)右|右移|右平移|右边|右挪|^右$")
_FWD = re.compile(r"前进|向前|往前|朝前|直走|前走|出发|前面|forward|go|走")
_BACK = re.compile(r"后退|倒退|向后|往后|朝后|退后|倒车|back|退")


@dataclass
class Intent:
    kind: str                     # stop/walk/stand/crouch/gait/status/greet/intro/
                                  # exit/confirm/cancel/unsupported/ignore/unknown
    text: str = ""                # 归一化后的原文
    vx: float = 0.0               # 方向 ±1（前+）
    vy: float = 0.0               # 方向 ±1（左+）
    wz: float = 0.0               # 方向 ±1（左转+）
    seconds: Optional[float] = None   # None=默认时长；inf="一直"
    speed: float = 1.0            # 快/慢倍率
    gait: str = ""                # tripod / wave
    reply: str = ""               # 建议的语音回复（status 由调用方填）

    @property
    def moving(self) -> bool:
        return self.kind == "walk" and (self.vx or self.vy or self.wz)


def normalize(text: str) -> str:
    return _PUNCT.sub("", (text or "").translate(_FULLWIDTH)).lower()


def cn_to_number(s: str) -> Optional[float]:
    """'三'→3、'十'→10、'十五'→15、'二十五'→25、'半'→0.5、'3'/'3.5' 原样。"""
    if not s:
        return None
    if re.fullmatch(r"\d+(\.\d+)?", s):
        return float(s)
    if s == "半":
        return 0.5
    if "十" in s:
        a, _, b = s.partition("十")
        if (a and a not in _CN_NUM) or (b and b not in _CN_NUM) or "十" in b:
            return None
        return float((_CN_NUM[a] if a else 1) * 10 + (_CN_NUM[b] if b else 0))
    total = 0
    for ch in s:
        if ch not in _CN_NUM:
            return None
        total = total * 10 + _CN_NUM[ch]
    return float(total)


def parse_duration(text: str) -> Optional[float]:
    """从句子里抠时长（秒）。没有 → None；"一直/持续" → inf。"""
    m = _DUR_RE.search(text)
    if m:
        n = cn_to_number(m.group(1))
        if n is not None:
            return n * (60.0 if m.group(2) in ("分钟", "分") else 1.0)
    m = _STEP_RE.search(text)
    if m:
        n = 3.0 if m.group(1) == "几" else cn_to_number(m.group(1))
        if n is not None:
            return n * STEP_SECONDS
    if "一直" in text or "持续" in text:
        return math.inf
    return None


def _fmt_secs(s: float) -> str:
    return str(int(s)) if float(s).is_integer() else f"{s:.1f}"


def _walk(text: str, kind_name: str, vx=0.0, vy=0.0, wz=0.0) -> Intent:
    secs = parse_duration(text)
    speed = 1.5 if "快" in text else (0.6 if "慢" in text else 1.0)
    if secs is None or math.isinf(secs):
        # 没说时长=一直走（连续动作）。"说停就停"不含 KWS 急停词
        # （停下/停止/停下来/别动），机器人念它不会触发自己的急停
        reply = f"一直{kind_name}，说停就停"
    else:
        reply = f"{kind_name}{_fmt_secs(secs)}秒"
    return Intent("walk", text, vx=vx, vy=vy, wz=wz, seconds=secs, speed=speed,
                  reply=reply)


def parse(text: str) -> Intent:
    t = normalize(text)
    if not t:
        return Intent("unknown", t, reply="没听清")

    if any(w in t for w in STOP_WORDS):
        return Intent("stop", t, reply="停")
    if t in WAKE_WORDS:
        # 唤醒词本身被 VAD 切成整句识别出来（KWS 已单独发过 wake 事件）：
        # 不动、不回话——否则会回"没听懂"，很吵
        return Intent("ignore", t)
    if t in CONFIRM_WORDS:
        return Intent("confirm", t)
    if t in CANCEL_WORDS:
        return Intent("cancel", t, reply="好")
    if any(w in t for w in EXIT_WORDS):
        return Intent("exit", t, reply="确定退出吗？请说 确定")
    if any(w in t for w in STATUS_WORDS):
        return Intent("status", t)
    if any(w in t for w in DANCE_WORDS):
        return Intent("unsupported", t, reply="跳舞要先把我架空，用 dance 脚本跑")
    if any(w in t for w in STAND_WORDS):
        return Intent("stand", t, reply="起立")
    if any(w in t for w in CROUCH_WORDS):
        return Intent("crouch", t, reply="趴下")
    if "三角" in t or "三足" in t or "tripod" in t:
        return Intent("gait", t, gait="tripod", reply="换三角步态")
    if "波浪" in t or "wave" in t:
        return Intent("gait", t, gait="wave", reply="换波浪步态")
    if any(w in t for w in INTRO_WORDS):
        # 长回话，顺带是"说话期间急停"的试金石（放在步态之后，
        # "介绍一下三角步态"仍归步态）
        return Intent("intro", t, reply=INTRO_REPLY)

    # 移动：转向 > 平移 > 前后
    if _TURN_L.search(t):
        return _walk(t, "左转", wz=1.0)
    if _TURN_R.search(t):
        return _walk(t, "右转", wz=-1.0)
    if _STRAFE_L.search(t):
        return _walk(t, "左移", vy=1.0)
    if _STRAFE_R.search(t):
        return _walk(t, "右移", vy=-1.0)
    if _BACK.search(t):
        return _walk(t, "后退", vx=-1.0)
    if _FWD.search(t):
        return _walk(t, "前进", vx=1.0)

    if any(w in t for w in GREET_WORDS):
        return Intent("greet", t, reply="在呢")
    return Intent("unknown", t, reply="没听懂")
