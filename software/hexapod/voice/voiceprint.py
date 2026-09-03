"""声纹锁：指令只听注册过的人（"谁在说"是"说了什么"之后的第二道闸）。

模型：3D-Speaker CAM++ 中文声纹（sherpa-onnx，28MB，192 维嵌入；Pi 5 上一段
2~4s 语音提取 ~0.1s）。开发机用多说话人 TTS 验证过区分度：同人不同句余弦
0.68~0.85，不同人 0.11~0.48，阈值 0.5 分得很开；真麦克风有噪声，注册脚本
（scripts/voice_enroll.py）会按注册段的自相似度给出建议阈值并存进档案。

范围纪律（安全 > 便利，engine 里执行）：
- 急停**永不拦**：KWS 急停词和整句识别出的 stop 意图，谁喊都停；
- 唤醒不拦：KWS 是流式的，做声纹要攒整段，低延迟做不了，且唤醒本身无害
  ——陌生人唤醒后说的指令会被这里拒掉；
- 其余指令段：先过自听过滤，再过声纹，不是主人 → denied 事件，不动不回话
  （连"没听懂"都不回，旁人聊天机器人不搭话）。

注册档案：npz（embs = N×192 注册段声纹、threshold、name），默认路径
$HEXAPOD_VOICEPRINT 或 <模型根>/voiceprint_owner.npz。
"""
import os
from pathlib import Path
from typing import Callable, Optional, Tuple


def default_profile(models_root=None) -> Path:
    env = os.environ.get("HEXAPOD_VOICEPRINT")
    if env:
        return Path(env)
    from .engine import ModelPaths
    root = Path(models_root) if models_root else ModelPaths.default_root()
    return root / "voiceprint_owner.npz"


def make_extractor(model_path, num_threads: int = 2):
    import sherpa_onnx as so
    try:
        return so.SpeakerEmbeddingExtractor(so.SpeakerEmbeddingExtractorConfig(
            model=str(model_path), num_threads=num_threads))
    except RuntimeError as e:
        size = Path(model_path).stat().st_size if Path(model_path).exists() else 0
        raise RuntimeError(
            f"{e}\n  {model_path} 当前 {size} 字节（campplus 正确应为 28281138）。"
            f"多半是下载坏了（镜像错误页/断传）：删掉该文件后重跑 "
            f"scripts/voice_setup.sh models（国内镜像失败就去掉 SHERPA_ONNX_MIRROR 直连）") from e


def embed(extractor, samples, rate: int = 16000):
    """一段单声道 float32 → L2 归一化声纹向量。"""
    import numpy as np
    st = extractor.create_stream()
    st.accept_waveform(sample_rate=rate, waveform=samples)
    st.input_finished()
    e = np.asarray(extractor.compute(st), dtype=np.float32)
    return e / (np.linalg.norm(e) + 1e-9)


def best_score(embs, e) -> float:
    """e 与各注册段声纹的最大余弦（都已归一化，点积即余弦）。"""
    return float((embs @ e).max())


def trim_voiced(samples, rate: int = 16000, margin_s: float = 0.1, rel: float = 0.1):
    """按能量裁出有人声的区间（注册短词用：2s 录音里"退出"只占 0.5s，
    整段直接提声纹会被前后静音稀释）。门限=峰值 RMS 的 rel 倍，下限 0.008。"""
    import numpy as np
    win = max(1, int(0.03 * rate))
    n = len(samples) // win
    if n == 0:
        return samples
    rms = np.sqrt((samples[:n * win].reshape(n, win) ** 2).mean(axis=1))
    th = max(0.008, float(rms.max()) * rel)
    idx = np.nonzero(rms >= th)[0]
    if not len(idx):
        return samples
    m = int(margin_s * rate)
    a = max(0, int(idx[0]) * win - m)
    b = min(len(samples), (int(idx[-1]) + 1) * win + m)
    return samples[a:b]


def append_profile(path, new_embs) -> int:
    """向已有档案追加声纹（阈值/名字不动），返回追加后的总条数。
    补录短词（voice_enroll.py --append）用，不必整套重注册。"""
    import numpy as np
    data = np.load(str(path), allow_pickle=False)
    old = np.asarray(data["embs"], dtype=np.float32)
    embs = np.vstack([old, np.asarray(new_embs, dtype=np.float32)])
    th = float(data["threshold"]) if "threshold" in data.files else 0.5
    name = str(data["name"]) if "name" in data.files else "owner"
    save_profile(path, embs, th, name)
    return len(embs)


def save_profile(path, embs, threshold: float, name: str = "owner") -> str:
    import numpy as np
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    np.savez(p, embs=np.asarray(embs, dtype=np.float32),
             threshold=np.float32(threshold), name=str(name))
    return str(p)


SHORT_S = 0.7          # 只给极短段折让（"退出"≈0.5s，向量确实抖）。窗口不能放到
                       # 1.0s：陌生人 0.8~0.9s 的指令会 0.43~0.46 贴线混过（开发机实测）；
SHORT_DISCOUNT = 0.05  # 多尺度注册后主人短句已 0.58+，折让只是给极短句留余量


class VoiceGate:
    """score(samples) = 与主人声纹的最大余弦；accept() 按阈值判（短段有折让）。
    模型懒加载。"""

    def __init__(self, model_path, profile_path, *, threshold: Optional[float] = None,
                 num_threads: int = 2, log: Optional[Callable[[str], None]] = None):
        import numpy as np
        data = np.load(str(profile_path), allow_pickle=False)
        self.embs = np.asarray(data["embs"], dtype=np.float32)
        if self.embs.ndim != 2 or not len(self.embs):
            raise ValueError(f"{profile_path}: 声纹档案为空或格式不对，重跑 voice_enroll.py")
        stored = float(data["threshold"]) if "threshold" in data.files else 0.5
        self.threshold = float(threshold) if threshold is not None else stored
        self.name = str(data["name"]) if "name" in data.files else "owner"
        self.model_path = model_path
        self.num_threads = num_threads
        self.log = log or (lambda s: None)
        self._extractor = None

    def _ensure(self):
        if self._extractor is None:
            import time
            t0 = time.time()
            self._extractor = make_extractor(self.model_path, self.num_threads)
            self.log(f"[spk] 声纹模型加载 {time.time()-t0:.1f}s（{Path(self.model_path).name}）")
        return self._extractor

    def score(self, samples, rate: int = 16000) -> float:
        return best_score(self.embs, embed(self._ensure(), samples, rate))

    def effective_threshold(self, dur_s: float) -> float:
        return self.threshold - (SHORT_DISCOUNT if dur_s < SHORT_S else 0.0)

    def accept(self, samples, rate: int = 16000) -> Tuple[bool, float]:
        s = self.score(samples, rate)
        return s >= self.effective_threshold(len(samples) / rate), s
