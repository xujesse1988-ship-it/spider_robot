"""声纹锁纯逻辑（不需要 sherpa/声纹模型，numpy 即可）。"""
import numpy as np
import pytest

from hexapod.voice.voiceprint import VoiceGate, best_score, default_profile, save_profile


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


def test_default_profile_env(tmp_path, monkeypatch):
    monkeypatch.setenv("HEXAPOD_VOICEPRINT", str(tmp_path / "x.npz"))
    assert default_profile() == tmp_path / "x.npz"
