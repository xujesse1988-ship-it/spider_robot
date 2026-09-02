"""离线中文语音合成（sherpa-onnx VITS / Matcha）+ 固定短语缓存 + 后台播放线程。

机器人的回话大多是固定短句（"停""前进3秒""在呢"），第一次合成后存成 wav
缓存（~/.cache/hexapod-voice/），以后直接播，零延迟；动态句子（电压读数）
现合成，Pi 5 上 vits 一句 ≈0.2~0.4s。

播放在独立线程里做，不阻塞语音引擎读麦克风；引擎用 is_muting() 在机器人
自己说话期间（含 0.3s 尾巴）丢弃麦克风数据，避免听见自己说的"停"。
"""
import hashlib
import os
import queue
import threading
import time
from collections import deque
from pathlib import Path
from typing import Callable, Iterable, Optional, Tuple

from .audio import read_wav, write_wav

VITS_MODEL = "model.onnx"
MATCHA_MODEL = "model-steps-3.onnx"
RULE_FSTS = ("phone.fst", "date.fst", "number.fst")


def describe_model(model_dir) -> str:
    d = Path(model_dir)
    if (d / MATCHA_MODEL).exists():
        return f"matcha:{d.name}"
    if (d / VITS_MODEL).exists():
        return f"vits:{d.name}"
    return f"?:{d.name}"


def make_tts(model_dir, num_threads: int = 2, vocoder: Optional[str] = None):
    """按目录内容自动选 VITS（model.onnx）或 Matcha（model-steps-3.onnx + 声码器）。"""
    import sherpa_onnx as so
    d = Path(model_dir)
    lexicon = str(d / "lexicon.txt") if (d / "lexicon.txt").exists() else ""
    tokens = str(d / "tokens.txt")
    dict_dir = str(d / "dict") if (d / "dict").is_dir() else ""
    data_dir = str(d / "espeak-ng-data") if (d / "espeak-ng-data").is_dir() else ""
    fsts = ",".join(str(d / f) for f in RULE_FSTS if (d / f).exists())
    if (d / MATCHA_MODEL).exists():
        voc = vocoder or os.environ.get("HEXAPOD_TTS_VOCODER") or ""
        if not voc:
            cands = sorted(d.parent.glob("vocos*.onnx")) + sorted(d.glob("vocos*.onnx"))
            if not cands:
                raise FileNotFoundError(f"{d.name} 是 Matcha 模型，需要声码器 vocos-22khz-univ.onnx"
                                        f"（放 {d.parent}/ 下或设 HEXAPOD_TTS_VOCODER）")
            voc = str(cands[0])
        model_cfg = so.OfflineTtsModelConfig(
            matcha=so.OfflineTtsMatchaModelConfig(
                acoustic_model=str(d / MATCHA_MODEL), vocoder=voc, lexicon=lexicon,
                tokens=tokens, data_dir=data_dir, dict_dir=dict_dir),
            num_threads=num_threads)
    elif (d / VITS_MODEL).exists():
        model_cfg = so.OfflineTtsModelConfig(
            vits=so.OfflineTtsVitsModelConfig(
                model=str(d / VITS_MODEL), lexicon=lexicon, tokens=tokens,
                data_dir=data_dir, dict_dir=dict_dir),
            num_threads=num_threads)
    else:
        raise FileNotFoundError(f"{d} 里既没有 {VITS_MODEL} 也没有 {MATCHA_MODEL}")
    return so.OfflineTts(so.OfflineTtsConfig(model=model_cfg, rule_fsts=fsts,
                                             max_num_sentences=1))


class Speaker(threading.Thread):
    """后台合成+播放线程。player 需有 play(samples, rate) / play_file(path)。"""

    def __init__(self, model_dir, player, *, sid: int = 0, speed: float = 1.0,
                 num_threads: int = 2, cache_dir: Optional[str] = None,
                 vocoder: Optional[str] = None, log: Optional[Callable[[str], None]] = None):
        super().__init__(name="voice-speaker", daemon=True)
        self.model_dir, self.player = Path(model_dir), player
        self.sid, self.speed, self.num_threads, self.vocoder = sid, speed, num_threads, vocoder
        self.cache_dir = Path(cache_dir or os.environ.get("HEXAPOD_VOICE_CACHE")
                              or Path.home() / ".cache" / "hexapod-voice")
        self.log = log or (lambda s: None)
        self._q: "queue.Queue" = queue.Queue()
        self._tts = None
        self._recent = deque(maxlen=8)     # (说过的句子, 播完时刻)——自听过滤用
        self._playing_text: Optional[str] = None
        self._lock = threading.Lock()
        self._busy = threading.Event()
        self._idle = threading.Event()
        self._idle.set()
        self._last_end = 0.0
        self._stop = False
        self.load_s = 0.0

    # ---- 对外 ----
    def say(self, text: str, block: bool = False, cache: bool = True,
            mute: bool = True) -> None:
        """mute=True：播放期间 is_muting() 为真，引擎不听麦克风（防听见自己）。
        很短的应答（唤醒后的"在"）传 mute=False，免得吃掉用户紧接着说的指令。"""
        text = (text or "").strip()
        if not text:
            return
        self._idle.clear()
        self._q.put(("say", text, cache, mute))
        if block:
            self.wait()

    def prewarm(self, phrases: Iterable[str]) -> None:
        for p in phrases:
            if p:
                self._q.put(("warm", p, True, False))

    def wait(self, timeout: Optional[float] = None) -> bool:
        return self._idle.wait(timeout)

    @property
    def busy(self) -> bool:
        return self._busy.is_set()

    def is_muting(self, tail_s: float = 0.3) -> bool:
        return self._busy.is_set() or time.time() < self._last_end + tail_s

    def recent_texts(self, window_s: float = 2.5):
        """正在说 + window_s 内说完的句子——引擎用来丢弃"听见自己"的识别结果。"""
        now = time.time()
        out = [t for t, end in list(self._recent) if now - end <= window_s]
        cur = self._playing_text
        if cur:
            out.append(cur)
        return out

    def cancel(self) -> None:
        """立即闭嘴：清掉还没播的，掐断正在播的（急停用）。"""
        try:
            while True:
                self._q.get_nowait()
        except queue.Empty:
            pass
        player_stop = getattr(self.player, "stop", None)
        if player_stop:
            player_stop()
        if not self._busy.is_set():
            self._idle.set()

    def stop(self) -> None:
        self._stop = True
        self._q.put(("quit", "", False, False))

    def render(self, text: str, cache: bool = True) -> Tuple[object, int]:
        """合成（走缓存），任意线程可调；模型访问有锁。"""
        path = self._cache_path(text)
        if cache and path.exists():
            return read_wav(path, rate=self._cached_rate(path))
        with self._lock:
            tts = self._ensure()
            t0 = time.time()
            audio = tts.generate(text, sid=self.sid, speed=self.speed)
            import numpy as np
            samples = np.asarray(audio.samples, dtype=np.float32)
            self.log(f"tts 合成 {text!r} {len(samples)/audio.sample_rate:.2f}s "
                     f"用时 {time.time()-t0:.2f}s")
        if cache:
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            write_wav(path, samples, audio.sample_rate)
        return samples, audio.sample_rate

    # ---- 内部 ----
    def _ensure(self):
        if self._tts is None:
            t0 = time.time()
            self._tts = make_tts(self.model_dir, self.num_threads, self.vocoder)
            self.load_s = time.time() - t0
            self.log(f"tts 模型 {describe_model(self.model_dir)} 加载 {self.load_s:.2f}s")
        return self._tts

    def _cache_path(self, text: str) -> Path:
        key = f"{self.model_dir.name}|{self.sid}|{self.speed}|{text}"
        return self.cache_dir / (hashlib.sha1(key.encode("utf-8")).hexdigest()[:20] + ".wav")

    @staticmethod
    def _cached_rate(path: Path) -> int:
        import wave
        with wave.open(str(path), "rb") as w:
            return w.getframerate()

    def run(self) -> None:
        while not self._stop:
            kind, text, cache, mute = self._q.get()
            try:
                if kind == "quit":
                    break
                if kind == "warm":
                    self.render(text, cache=True)
                    continue
                if mute:
                    self._busy.set()
                if kind == "say":
                    self._playing_text = text
                samples, rate = self.render(text, cache=cache)
                self.player.play(samples, rate)
            except Exception as e:                      # 说不出话不能拖死机器人
                self.log(f"⚠ tts 失败 {text!r}: {e}")
            finally:
                if kind == "say":
                    if self._playing_text is not None:
                        self._recent.append((text, time.time()))
                        self._playing_text = None
                    if mute:
                        self._last_end = time.time()
                        self._busy.clear()
                if self._q.empty():
                    self._idle.set()
