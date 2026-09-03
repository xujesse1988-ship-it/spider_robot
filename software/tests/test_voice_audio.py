"""audio.py 纯逻辑（Highpass 流式高通；不需要声卡）。"""
import numpy as np
import pytest

from hexapod.voice.audio import Highpass


def _tone(hz, secs=1.0, rate=16000, amp=0.3):
    t = np.arange(int(secs * rate)) / rate
    return (amp * np.sin(2 * np.pi * hz * t)).astype(np.float32)


def _rms(x):
    return float(np.sqrt((x ** 2).mean()))


def test_highpass_kills_rumble_keeps_speech_band():
    lo = Highpass(200.0).process(_tone(50))[8000:]     # 前半段是暂态，掐掉
    hi = Highpass(200.0).process(_tone(1000))[8000:]
    assert _rms(lo) < 0.1 * _rms(_tone(50))            # 50Hz（低两个八度）≥20dB 衰减
    assert _rms(hi) == pytest.approx(_rms(_tone(1000)), rel=0.05)   # 1kHz 基本不动


def test_highpass_streaming_matches_oneshot():
    rng = np.random.default_rng(3)
    x = rng.normal(0, 0.1, 16000).astype(np.float32)
    one = Highpass(200.0).process(x)
    hp = Highpass(200.0)
    two = np.concatenate([hp.process(x[:777]), hp.process(x[777:])])
    assert np.allclose(one, two, atol=1e-6)            # 跨 chunk 状态连续，无缝


def test_highpass_from_env(monkeypatch):
    monkeypatch.delenv("HEXAPOD_AUDIO_HPF", raising=False)
    assert Highpass.from_env() is None                 # 默认关
    monkeypatch.setenv("HEXAPOD_AUDIO_HPF", "0")
    assert Highpass.from_env() is None
    monkeypatch.setenv("HEXAPOD_AUDIO_HPF", "200")
    assert Highpass.from_env().cutoff_hz == 200.0
    monkeypatch.setenv("HEXAPOD_AUDIO_HPF", "垃圾")
    assert Highpass.from_env() is None                 # 非法值当没设，别炸录音链
