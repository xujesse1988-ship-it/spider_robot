"""声纹锁纯逻辑（不需要 sherpa/声纹模型，numpy 即可）。"""
import numpy as np
import pytest

from hexapod.voice.voiceprint import (VoiceGate, append_profile, best_score,
                                      default_profile, save_profile, trim_voiced)


def _unit(v):
    v = np.asarray(v, dtype=np.float32)
    return v / np.linalg.norm(v)


def test_best_score_is_max_cosine():
    embs = np.stack([_unit([1, 0, 0]), _unit([0, 1, 0])])
    assert best_score(embs, _unit([1, 0, 0])) == pytest.approx(1.0)
    assert best_score(embs, _unit([1, 1, 0])) == pytest.approx(0.7071, abs=1e-3)
    assert best_score(embs, _unit([0, 0, 1])) == pytest.approx(0.0, abs=1e-6)


def test_profile_roundtrip_and_threshold(tmp_path):
    p = tmp_path / "owner.npz"
    save_profile(p, np.stack([_unit([1, 0]), _unit([0, 1])]), 0.47, "shaopeng")
    g = VoiceGate("model.onnx", p)                       # 模型懒加载，不会碰文件
    assert g.embs.shape == (2, 2)
    assert g.threshold == pytest.approx(0.47)
    assert g.name == "shaopeng"
    g2 = VoiceGate("model.onnx", p, threshold=0.6)       # 显式覆盖
    assert g2.threshold == pytest.approx(0.6)


def test_profile_without_threshold_defaults(tmp_path):
    p = tmp_path / "bare.npz"
    np.savez(p, embs=np.stack([_unit([1, 0])]))
    assert VoiceGate("m.onnx", p).threshold == pytest.approx(0.5)


def test_empty_profile_rejected(tmp_path):
    p = tmp_path / "bad.npz"
    np.savez(p, embs=np.zeros((0, 192), np.float32))
    with pytest.raises(ValueError):
        VoiceGate("m.onnx", p)


def test_short_segment_threshold_discount(tmp_path):
    p = tmp_path / "o.npz"
    save_profile(p, np.stack([_unit([1, 0])]), 0.50)
    g = VoiceGate("m.onnx", p)
    assert g.effective_threshold(2.0) == pytest.approx(0.50)   # 正常长度不折让
    assert g.effective_threshold(0.8) == pytest.approx(0.50)   # 0.7~1s 也不折让（防贴线混入）
    assert g.effective_threshold(0.5) == pytest.approx(0.45)   # 极短句（"退出"）折让 0.05


def test_default_profile_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HEXAPOD_VOICEPRINT", str(tmp_path / "x.npz"))
    assert default_profile() == tmp_path / "x.npz"


def test_trim_voiced_extracts_burst():
    rate = 16000
    audio = np.zeros(2 * rate, np.float32)
    t = np.arange(int(0.5 * rate)) / rate
    audio[int(0.9 * rate):int(1.4 * rate)] = 0.3 * np.sin(2 * np.pi * 220 * t)
    seg = trim_voiced(audio, rate)
    dur = len(seg) / rate
    assert 0.5 <= dur <= 0.8                      # 词 0.5s + 两侧 margin ≤0.1s（窗量化有零头）
    assert float(np.abs(seg).max()) == pytest.approx(0.3, abs=0.01)  # 词身完整保住


def test_trim_voiced_silence_and_empty_passthrough():
    rate = 16000
    silence = np.zeros(rate, np.float32)
    assert len(trim_voiced(silence, rate)) == len(silence)   # 全静音不知道裁哪，原样返回
    assert len(trim_voiced(np.zeros(0, np.float32), rate)) == 0


def test_append_profile_keeps_threshold_and_name(tmp_path):
    p = tmp_path / "o.npz"
    save_profile(p, np.stack([_unit([1, 0, 0]), _unit([0, 1, 0])]), 0.47, "shaopeng")
    total = append_profile(p, np.stack([_unit([0, 0, 1])]))
    assert total == 3
    g = VoiceGate("m.onnx", p)
    assert g.embs.shape == (3, 3)
    assert g.threshold == pytest.approx(0.47)     # 追加不动阈值
    assert g.name == "shaopeng"
    assert best_score(g.embs, _unit([0, 0, 1])) == pytest.approx(1.0)  # 新锚点生效
