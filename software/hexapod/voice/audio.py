"""ALSA 录音/放音：走 arecord / aplay 子进程（alsa-utils 树莓派自带）。

不用 PortAudio/sounddevice：少一层依赖，plughw 自动做采样率/位深/通道转换。
声卡是 ReSpeaker Lite（USB 固件，UAC2 免驱，16kHz）：XU316 芯片在板上做完
回声消除/噪声抑制/自动增益，送出来的已是处理后的人声，这里只取第 0 通道
转 float32 给识别用（多通道时其余通道内容未必是同一路，取平均反而掺东西）。
WavSource 用 wav 文件顶替麦克风，开发机上无声卡也能把整条链路跑通。
"""
import os
import re
import subprocess
import tempfile
import time
import wave
from pathlib import Path
from typing import List, Optional, Tuple

RATE = 16000
PREFERRED_CARDS = ("respeaker", "lite", "xvf", "seeed2micvoicec", "wm8960", "seeed")


def list_cards() -> List[Tuple[int, str, str]]:
    """解析 /proc/asound/cards → [(索引, 卡名, 描述)]。"""
    cards = []
    try:
        txt = Path("/proc/asound/cards").read_text()
    except OSError:
        return cards
    for m in re.finditer(r"^\s*(\d+)\s+\[(\S+)\s*\]:\s*(.*)$", txt, re.M):
        cards.append((int(m.group(1)), m.group(2), m.group(3).strip()))
    return cards


def find_card(prefer=PREFERRED_CARDS) -> Optional[str]:
    """返回语音声卡名（环境变量 HEXAPOD_AUDIO_CARD 可强制指定）。"""
    env = os.environ.get("HEXAPOD_AUDIO_CARD")
    if env:
        return env
    cards = list_cards()
    for key in prefer:
        for _, name, desc in cards:
            if key.lower() in name.lower() or key.lower() in desc.lower():
                return name
    return None


def alsa_device(card: Optional[str]) -> str:
    """arecord/aplay 的 -D 参数；card 为 None 时用系统默认设备。"""
    if not card:
        return "default"
    if card.isdigit():
        return f"plughw:{card},0"
    return f"plughw:CARD={card},DEV=0"


def _to_float32(pcm: bytes, channels: int, pick: Optional[int] = None):
    """pick=None 多通道取平均（wav 文件）；pick=k 只取第 k 通道（麦克风）。"""
    import numpy as np
    x = np.frombuffer(pcm, dtype=np.int16)
    if channels > 1:
        n = len(x) // channels * channels
        x = x[:n].reshape(-1, channels)
        x = x[:, pick] if pick is not None else x.mean(axis=1)
    return (x.astype(np.float32) / 32768.0)


def read_wav(path, rate: int = RATE):
    """任意 wav → 单声道 float32 @rate（线性重采样够用）。"""
    import numpy as np
    with wave.open(str(path), "rb") as w:
        ch, sw, sr, n = w.getnchannels(), w.getsampwidth(), w.getframerate(), w.getnframes()
        raw = w.readframes(n)
    if sw != 2:
        raise ValueError(f"{path}: 只支持 16 位 wav（现在 {sw*8} 位）")
    x = _to_float32(raw, ch)
    if sr != rate:
        m = int(len(x) * rate / sr)
        x = np.interp(np.linspace(0, len(x) - 1, m), np.arange(len(x)), x).astype(np.float32)
    return x, rate


def write_wav(path, samples, rate: int) -> str:
    import numpy as np
    pcm = (np.clip(np.asarray(samples, dtype=np.float32), -1, 1) * 32767).astype(np.int16)
    with wave.open(str(path), "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)
        w.setframerate(int(rate))
        w.writeframes(pcm.tobytes())
    return str(path)


class ArecordSource:
    """麦克风：arecord 常驻子进程，read(n) 返回 n 个单声道 float32 样本。

    多通道只取一路（默认第 0 通道，环境变量 HEXAPOD_AUDIO_PICK 可换，
    见模块头注释；哪路干净用 voice_check.py --echo-test 判定）。
    live=True：数据随真实时间流逝，引擎在机器人说话期间可以直接丢弃。
    """
    live = True

    def __init__(self, device: str, rate: int = RATE, channels: int = 2,
                 pick: Optional[int] = None):
        if pick is None:
            pick = int(os.environ.get("HEXAPOD_AUDIO_PICK", "0"))
        self.rate, self.channels, self.device, self.pick = rate, channels, device, pick
        cmd = ["arecord", "-q", "-D", device, "-f", "S16_LE", "-r", str(rate),
               "-c", str(channels), "-t", "raw"]
        self._p = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, bufsize=0)
        time.sleep(0.2)
        if self._p.poll() is not None:
            err = self._p.stderr.read().decode(errors="replace").strip()
            raise RuntimeError(f"arecord 启动失败（{device}）：{err or '无输出'}\n"
                               f"  查：arecord -l 有没有 ReSpeaker Lite；当前用户在 audio 组吗")

    def read(self, n: int):
        need = n * self.channels * 2
        buf = bytearray()
        while len(buf) < need:
            chunk = self._p.stdout.read(need - len(buf))
            if not chunk:
                return None                      # arecord 退出
            buf += chunk
        return _to_float32(bytes(buf), self.channels, pick=self.pick)

    def close(self):
        if self._p.poll() is None:
            self._p.terminate()
            try:
                self._p.wait(1.0)
            except subprocess.TimeoutExpired:
                self._p.kill()


class WavSource:
    """用 wav 文件顶替麦克风。realtime=True 时按真实时间节奏吐数据。"""

    def __init__(self, path, rate: int = RATE, realtime: bool = False):
        self.samples, self.rate = read_wav(path, rate)
        self.realtime, self.pos = realtime, 0
        self.live = realtime      # 非实时喂法：说话期间不能丢数据，引擎改为等说完

    def read(self, n: int):
        import numpy as np
        if self.pos >= len(self.samples):
            return None
        x = self.samples[self.pos:self.pos + n]
        self.pos += n
        if len(x) < n:
            x = np.concatenate([x, np.zeros(n - len(x), np.float32)])
        if self.realtime:
            time.sleep(n / self.rate)
        return x

    def close(self):
        pass


class AplayPlayer:
    """aplay 阻塞播放；tmp_dir 放临时 wav（默认系统临时目录）。"""

    def __init__(self, device: str, tmp_dir: Optional[str] = None):
        self.device = device
        self.tmp_dir = tmp_dir or tempfile.gettempdir()

    def play(self, samples, rate: int) -> None:
        fd, path = tempfile.mkstemp(prefix="hexvoice-", suffix=".wav", dir=self.tmp_dir)
        os.close(fd)
        try:
            write_wav(path, samples, rate)
            self.play_file(path)
        finally:
            try:
                os.unlink(path)
            except OSError:
                pass

    def play_file(self, path) -> None:
        subprocess.run(["aplay", "-q", "-D", self.device, str(path)],
                       check=False, stderr=subprocess.DEVNULL)


class NullPlayer:
    """不出声；save_dir 给了就把每段合成音存成 wav（开发机检查用）。"""

    def __init__(self, save_dir: Optional[str] = None):
        self.save_dir, self.count = save_dir, 0

    def play(self, samples, rate: int) -> None:
        self.count += 1
        if self.save_dir:
            Path(self.save_dir).mkdir(parents=True, exist_ok=True)
            write_wav(Path(self.save_dir) / f"say-{self.count:03d}.wav", samples, rate)

    def play_file(self, path) -> None:
        self.count += 1
