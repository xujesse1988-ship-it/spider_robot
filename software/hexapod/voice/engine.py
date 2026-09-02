"""语音引擎线程：KWS 常驻（唤醒词 + 急停词）→ VAD 切句 → SenseVoice → 意图事件。

两层设计的原因：
- 急停要快、要随时有效：KWS 是流式的，"停下/别动"一出口 ~0.3s 内就有事件，
  不用等 VAD 判断句尾，也不需要先喊唤醒词。误触的代价只是多停一次，安全方向。
- 其它指令走"唤醒词 → 整句识别"：SenseVoice 对短句准确率远高于关键词表，
  且指令可以自由组合（"快点往前走三秒"）。唤醒后 follow_up_s 内可连说多条。

事件（VoiceEvent.kind）：
  ready   模型加载完成，开始听
  wake    听到唤醒词（text=哪个词）
  asleep  听令窗口超时，回到只听唤醒词
  stop    KWS 急停词命中（text=哪个词）——调用方必须立即停
  command 一句识别完成（text=原文，intent=解析结果；unknown 也会给，供打印）
  denied  声纹锁拒绝（text=原文，intent 附带）——不是注册的主人在说话；急停不经此闸
  eof     音源结束（wav 顶替麦克风时）
  error   模型加载/运行异常（text=原因），引擎线程随之退出

线程模型：本线程只做音频→事件；机器人动作在调用方主线程里做（舵机串口
不跨线程）。TTS 通过 Speaker 线程播。机器人自己说话期间：KWS 照听——急停词
随时有效（长回话时不能聋），说话中冒出的唤醒词忽略（多半是自己念的
"我是小蜘蛛"）；整句识别默认不做（板载 AEC 实测无效，回声会被完整听见），
mute_during_tts=False（--trust-aec）时照做，识别结果先过自听过滤
looks_like_echo 再出事件。
"""
import difflib
import glob
import os
import queue
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

from .intents import Intent, normalize, parse
from .keywords import parse_result

RATE = 16000


def looks_like_echo(text: str, said_texts) -> bool:
    """识别结果是否只是机器人自己刚说过的话（喇叭→麦克风回声）。

    09-02 实机：板载 AEC 没压住自听，"电压7.4伏电流1.0安"的回答被再次识别成
    status 指令，机器人自问自答死循环——所以不管 AEC 好坏，这层过滤都要有。
    容错到回声被听歪的程度（"电流1.0安"→"电容1.0N"），长回话（自我介绍）被
    VAD 切成几段、每段再错几个字也能挡住（按短串覆盖率判）；短回话（"停""在"
    ≤2 字）只认全等，免得把用户真喊的"停下"当回声丢掉。
    """
    a = normalize(text)
    if not a:
        return False
    for said in said_texts:
        b = normalize(said)
        if not b:
            continue
        if len(a) <= 2 or len(b) <= 2:
            if a == b:
                return True
            continue
        if a in b or b in a:
            return True
        m = difflib.SequenceMatcher(None, a, b)
        matched = sum(bl.size for bl in m.get_matching_blocks())
        if matched / min(len(a), len(b)) >= 0.7:
            return True
    return False


@dataclass
class ModelPaths:
    kws_dir: Path
    asr_dir: Path
    vad_model: Path
    keywords_file: Path
    tts_dir: Optional[Path] = None
    spk_model: Optional[Path] = None      # 声纹模型（可选，声纹锁用）

    @staticmethod
    def default_root() -> Path:
        return Path(os.environ.get("HEXAPOD_VOICE_MODELS") or Path.home() / "models" / "voice")

    @classmethod
    def discover(cls, root=None, tts_name: Optional[str] = None) -> "ModelPaths":
        root = Path(root) if root else cls.default_root()

        def one(pattern, what):
            hits = sorted(glob.glob(str(root / pattern)))
            if not hits:
                raise FileNotFoundError(f"{root} 下找不到{what}（{pattern}）——先跑 "
                                        f"scripts/voice_setup.sh，或设 HEXAPOD_VOICE_MODELS")
            return Path(hits[-1])

        kws = one("sherpa-onnx-kws-zipformer-wenetspeech-*", "KWS 模型")
        asr = one("sherpa-onnx-sense-voice-*", "SenseVoice 模型")
        vad = one("silero_vad.onnx", "VAD 模型")
        kw = kws / "keywords_hexapod.txt"
        if not kw.exists():
            kw = kws / "keywords.txt"          # 模型自带样例词表（小爱同学…），只能应急
        tts = None
        if tts_name:
            tts = root / tts_name
        else:
            for pat in ("matcha-icefall-zh-*", "vits-melo-tts-zh_en", "sherpa-onnx-vits-zh-*",
                        "vits-zh-*"):
                hits = sorted(glob.glob(str(root / pat)))
                if hits:
                    tts = Path(hits[0])
                    break
        spk = None
        for pat in ("3dspeaker_*campplus*.onnx", "3dspeaker_*.onnx", "wespeaker*.onnx"):
            hits = sorted(glob.glob(str(root / pat)))
            if hits:
                spk = Path(hits[0])
                break
        return cls(kws_dir=kws, asr_dir=asr, vad_model=vad, keywords_file=kw,
                   tts_dir=tts, spk_model=spk)

    def kws_files(self):
        d = self.kws_dir

        def pick(prefix):
            c = sorted(d.glob(f"{prefix}-epoch-12-*.int8.onnx")) or sorted(d.glob(f"{prefix}-*.int8.onnx")) \
                or sorted(d.glob(f"{prefix}-*.onnx"))
            if not c:
                raise FileNotFoundError(f"{d} 缺 {prefix} onnx")
            return str(c[0])
        return str(d / "tokens.txt"), pick("encoder"), pick("decoder"), pick("joiner")

    def asr_files(self):
        d = self.asr_dir
        m = d / "model.int8.onnx"
        if not m.exists():
            m = d / "model.onnx"
        return str(m), str(d / "tokens.txt")


@dataclass
class VoiceEvent:
    kind: str
    text: str = ""
    intent: Optional[Intent] = None
    t: float = field(default_factory=time.time)


class VoiceEngine(threading.Thread):
    def __init__(self, paths: ModelPaths, source, speaker=None, *,
                 wake_required: bool = True, follow_up_s: float = 8.0,
                 listen_timeout_s: float = 6.0, num_threads: int = 2,
                 kws_threshold: float = 0.25, wake_ack: str = "在",
                 chunk_s: float = 0.1, mute_during_tts: bool = True,
                 voice_gate=None, log: Optional[Callable[[str], None]] = None):
        super().__init__(name="voice-engine", daemon=True)
        self.paths, self.source, self.speaker = paths, source, speaker
        self.voice_gate = voice_gate       # voiceprint.VoiceGate；None=不开声纹锁
        self.wake_required, self.follow_up_s = wake_required, follow_up_s
        self.listen_timeout_s, self.num_threads = listen_timeout_s, num_threads
        self.kws_threshold, self.wake_ack, self.chunk_s = kws_threshold, wake_ack, chunk_s
        self.mute_during_tts = mute_during_tts
        self.log = log or (lambda s: None)
        self.events: "queue.Queue[VoiceEvent]" = queue.Queue()
        self._stop = threading.Event()
        self._awake_until = 0.0
        self.load_s = 0.0

    # ---- 对外 ----
    @property
    def awake(self) -> bool:
        return (not self.wake_required) or time.time() < self._awake_until

    def say(self, text: str, block: bool = False) -> None:
        if self.speaker:
            self.speaker.say(text, block=block)

    def stop(self) -> None:
        self._stop.set()

    # ---- 主循环 ----
    def run(self) -> None:
        try:
            kws, vad, asr = self._load()
        except Exception as e:
            self.events.put(VoiceEvent("error", f"模型加载失败: {e}"))
            return
        self.events.put(VoiceEvent("ready"))
        n = int(self.chunk_s * RATE)
        ks = kws.create_stream()
        was_awake = False
        try:
            while not self._stop.is_set():
                chunk = self.source.read(n)
                if chunk is None:
                    vad.flush()
                    self._drain_vad(vad, asr)
                    self.events.put(VoiceEvent("eof"))
                    break
                # 1) KWS：唤醒词 + 急停词常驻——机器人自己说话期间也听，
                #    急停不能聋（长回话时尤其要紧）；说话期间的唤醒词多半是
                #    自己念的"我是小蜘蛛"，忽略
                ks.accept_waveform(RATE, chunk)
                while kws.is_ready(ks):
                    kws.decode_stream(ks)
                    r = kws.get_result(ks)
                    if r:
                        kws.reset_stream(ks)
                        tag, word = parse_result(r)
                        # 尾巴放宽到 1s：句尾的唤醒词（"…叫我小蜘蛛"）经声学+
                        # 缓冲延迟到达 KWS 时往往已过 0.3s 默认窗（09-02 实机：
                        # 就绪播报刚完它自己应了声"在"）
                        speaking = (self.speaker is not None
                                    and self.speaker.is_muting(tail_s=1.0))
                        if tag == "STOP":
                            self.log(f"[kws] 急停词 {word}")
                            self.events.put(VoiceEvent("stop", word))
                            self._awake_until = 0.0
                            vad.reset()
                        elif speaking:
                            self.log(f"[kws] 说话期间忽略唤醒 {word}")
                        elif tag == "WAKE" or not tag:
                            self.log(f"[kws] 唤醒 {word}")
                            self._awake_until = time.time() + self.listen_timeout_s
                            vad.reset()
                            self.events.put(VoiceEvent("wake", word))
                            if self.speaker is not None and self.wake_ack:
                                # 应答不屏蔽麦克风：用户常一口气说"小蜘蛛，前进三秒"
                                self.speaker.say(self.wake_ack, mute=False)

                # 2) 整句识别：机器人自己说话期间不做（回声会被 ASR 完整听见）
                if (self.mute_during_tts and self.speaker is not None
                        and self.speaker.is_muting()):
                    if getattr(self.source, "live", True):
                        continue                         # 麦克风数据只喂 KWS，不进 VAD
                    self.speaker.wait(10.0)              # 文件音源：等说完再处理这段

                # 3) 听令状态：VAD 切句 → ASR → 意图
                if self.awake:
                    was_awake = True
                    vad.accept_waveform(chunk)
                    if self._drain_vad(vad, asr):
                        self._awake_until = max(self._awake_until,
                                                time.time() + self.follow_up_s)
                elif was_awake:
                    was_awake = False
                    vad.reset()
                    self.log("[engine] 听令窗口结束")
                    self.events.put(VoiceEvent("asleep"))
        except Exception as e:
            self.events.put(VoiceEvent("error", f"引擎异常: {e!r}"))
        finally:
            try:
                self.source.close()
            except Exception:
                pass

    def _drain_vad(self, vad, asr) -> bool:
        """把 VAD 攒好的整句都识别掉；返回是否识别出至少一条有效指令。"""
        import numpy as np
        got = False
        while not vad.empty():
            seg = vad.front
            samples = np.asarray(seg.samples, dtype=np.float32)
            vad.pop()
            t0 = time.time()
            s = asr.create_stream()
            s.accept_waveform(RATE, samples)
            asr.decode_stream(s)
            text = s.result.text.strip()
            if (text and self.speaker is not None
                    and looks_like_echo(text, self.speaker.recent_texts())):
                self.log(f"[asr] {len(samples)/RATE:.1f}s → {text!r} ≈ 自己刚说的，丢弃")
                continue
            intent = parse(text)
            if self.voice_gate is not None and intent.kind != "stop":
                # 声纹锁：急停不经此闸（谁喊都停），其余指令只听主人
                ok, sc = self.voice_gate.accept(samples)
                if not ok:
                    self.log(f"[spk] 声纹 {sc:.2f} < {self.voice_gate.threshold:.2f}"
                             f"，拒绝 {text!r}")
                    self.events.put(VoiceEvent("denied", text, intent))
                    continue                 # 也不延长听令窗
            self.log(f"[asr] {len(samples)/RATE:.1f}s → {text!r} → {intent.kind}"
                     f" ({time.time()-t0:.2f}s)")
            self.events.put(VoiceEvent("command", text, intent))
            if intent.kind != "unknown":
                got = True
        return got

    def _load(self):
        import sherpa_onnx as so
        t0 = time.time()
        tokens, enc, dec, joi = self.paths.kws_files()
        kws = so.KeywordSpotter(tokens=tokens, encoder=enc, decoder=dec, joiner=joi,
                                keywords_file=str(self.paths.keywords_file),
                                num_threads=1, keywords_threshold=self.kws_threshold,
                                num_trailing_blanks=1, provider="cpu")
        vad = so.VoiceActivityDetector(
            so.VadModelConfig(
                silero_vad=so.SileroVadModelConfig(
                    model=str(self.paths.vad_model), threshold=0.5,
                    min_silence_duration=0.4, min_speech_duration=0.2,
                    max_speech_duration=6.0),
                sample_rate=RATE),
            buffer_size_in_seconds=30)
        model, atok = self.paths.asr_files()
        asr = so.OfflineRecognizer.from_sense_voice(
            model=model, tokens=atok, num_threads=self.num_threads,
            language="zh", use_itn=True)
        self.load_s = time.time() - t0
        self.log(f"[engine] 模型加载 {self.load_s:.1f}s，关键词表 {self.paths.keywords_file}")
        return kws, vad, asr
