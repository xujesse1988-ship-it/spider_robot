"""语音交互子包（ReSpeaker Lite USB 麦克风/喇叭板 + sherpa-onnx 全离线）。

- intents.py   识别文本 → 意图（纯规则，无三方依赖）
- keywords.py  唤醒词/急停词 → KWS 关键词文件（只依赖 pypinyin）
- audio.py     arecord/aplay 子进程做录放音（不装 PortAudio）
- tts.py       离线合成 + 固定短语缓存 + 后台播放线程
- engine.py    引擎线程：KWS 常驻 → VAD 切句 → SenseVoice → 意图事件队列

sherpa_onnx / numpy 只在 audio/tts/engine 里延迟导入；开发机没装也能
`from hexapod.voice.intents import parse` 跑测试。
"""
from .intents import Intent, parse, normalize, cn_to_number, parse_duration

__all__ = ["Intent", "parse", "normalize", "cn_to_number", "parse_duration"]
