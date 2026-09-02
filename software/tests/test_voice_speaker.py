"""Speaker 分句播放的打断行为（render 打桩，不需要 TTS 模型）。

背景（09-02 实机）：急停打在"下一句正在合成"的窗口里，合成完照样开播——
cancel 必须在合成后、开播前再查一次。
"""
import time

import numpy as np

from hexapod.voice.tts import Speaker


class FakeRenderSpeaker(Speaker):
    def __init__(self, player, render_s: float):
        super().__init__("nonexistent-tts-dir", player)
        self._render_s = render_s

    def render(self, text, cache=True):
        time.sleep(self._render_s)
        return np.zeros(800, np.float32), 16000


class RecordingPlayer:
    def __init__(self):
        self.played = 0

    def play(self, samples, rate):
        self.played += 1
        time.sleep(0.2)


def test_cancel_stops_remaining_sentences():
    pl = RecordingPlayer()
    spk = FakeRenderSpeaker(pl, render_s=0.3)
    spk.start()
    spk.say("一。二。三。四。")          # 每句 render 0.3s + play 0.2s + gap 0.45s
    time.sleep(0.65)                      # 句 1 播完，句 2 合成/间隔中
    spk.cancel()
    assert spk.wait(3.0)                  # 很快闲下来
    assert pl.played <= 2                 # 后面的句子被跳过（无 cancel 应是 4）
    assert not spk.busy
    spk.stop()
