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
    return so.SpeakerEmbeddingExtractor(so.SpeakerEmbeddingExtractorConfig(
        model=str(model_path), num_threads=num_threads))


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


def save_profile(path, embs, threshold: float, name: str = "owner") -> str:
    import numpy as np
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    np.savez(p, embs=np.asarray(embs, dtype=np.float32),
             threshold=np.float32(threshold), name=str(name))
    return str(p)


class VoiceGate:
    """score(samples) = 与主人声纹的最大余弦；accept() 按阈值判。模型懒加载。"""

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

    def accept(self, samples, rate: int = 16000) -> Tuple[bool, float]:
        s = self.score(samples, rate)
        return s >= self.threshold, s
