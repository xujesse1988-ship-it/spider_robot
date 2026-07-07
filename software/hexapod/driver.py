"""Servo2040 驱动 —— Chica 串口协议。

协议从固件源码逐字节确认（EddieCarrera/chica-servo2040-simpleDriver，MIT）：

  SET 包:  0xD3, startIdx, count, [低7位, 高7位] * count
  GET 包:  0xC7, startIdx, count
  GET 响应: 0xC7, startIdx, count, 然后每项 [低7位, 高7位]

索引表（cmdPins）：0..17 舵机 SERVO1..18；18..23 足底开关 TS1..TS6；
24 电流；25 电压；26 RELAY（舵机总使能，同时驱动 GPIO26 上的物理继电器）；27/28 A1/A2。

值均为 14 位。舵机值 = 脉宽 µs。换算（来自固件常量）：
  电压 V = raw / 310.3        电流 A = (raw - 512) * 0.0814

固件逐字节读取超时 100µs —— 一个数据包必须一次 write() 发完，不能分段。
上电后舵机不动，直到 SET RELAY=1；先发好全部脉宽再使能，舵机会直接到位。
"""
import time

SET_CMD = 0xD3
GET_CMD = 0xC7
IDX_SERVO1 = 0
IDX_TS1 = 18
IDX_CURR = 24
IDX_VOLT = 25
IDX_RELAY = 26
NUM_SERVOS = 18
NUM_TOUCH = 6

VOLT_RATIO = 310.3
CURR_LSB = 0.0814
TOUCH_THRESHOLD_RAW = 512  # 开关闭合到 3.3V ≈ 1023


def encode_set(start: int, values) -> bytes:
    values = list(values)
    assert 0 <= start and start + len(values) <= 29 and len(values) <= 127
    pkt = bytearray([SET_CMD, start, len(values)])
    for v in values:
        v = int(round(v)) & 0x3FFF
        pkt += bytes([v & 0x7F, (v >> 7) & 0x7F])
    return bytes(pkt)


def encode_get(start: int, count: int) -> bytes:
    return bytes([GET_CMD, start, count])


def decode_get_response(buf: bytes, start: int, count: int):
    """校验 3 字节包头并解出 count 个 14 位值。"""
    if len(buf) != 3 + 2 * count:
        raise IOError(f"GET 响应长度 {len(buf)}，期望 {3 + 2 * count}")
    if buf[0] != GET_CMD or buf[1] != start or buf[2] != count:
        raise IOError(f"GET 响应包头不符: {buf[:3].hex()}")
    return [(buf[3 + 2 * i] & 0x7F) | ((buf[4 + 2 * i] & 0x7F) << 7)
            for i in range(count)]


class Servo2040Driver:
    """通过 USB CDC 串口驱动 Servo2040（树莓派上通常是 /dev/ttyACM0）。"""

    def __init__(self, port: str = "/dev/ttyACM0", timeout: float = 0.2):
        import serial  # 延迟导入，无 pyserial 时仍可用 MockDriver
        self.ser = serial.Serial(port, baudrate=115200, timeout=timeout)
        time.sleep(0.2)  # 等固件退出 LED 等待循环
        self.ser.reset_input_buffer()

    # --- 输出 ---
    def set_pulses_us(self, start: int, values) -> None:
        self.ser.write(encode_set(start, values))

    def set_all_pulses_us(self, values18) -> None:
        assert len(values18) == NUM_SERVOS
        self.ser.write(encode_set(IDX_SERVO1, values18))

    def enable(self, on: bool) -> None:
        self.ser.write(encode_set(IDX_RELAY, [1 if on else 0]))

    # --- 读取 ---
    def _get(self, start: int, count: int):
        self.ser.reset_input_buffer()
        self.ser.write(encode_get(start, count))
        buf = self.ser.read(3 + 2 * count)
        return decode_get_response(buf, start, count)

    def read_touch_raw(self):
        return self._get(IDX_TS1, NUM_TOUCH)

    def read_touch(self):
        """6 路足底开关 [TS1..TS6]，True=触地。按腿名取用 LegConfig.touch_idx-18。"""
        return [v > TOUCH_THRESHOLD_RAW for v in self.read_touch_raw()]

    def read_voltage_v(self) -> float:
        return self._get(IDX_VOLT, 1)[0] / VOLT_RATIO

    def read_current_a(self) -> float:
        return (self._get(IDX_CURR, 1)[0] - 512) * CURR_LSB

    def close(self) -> None:
        self.enable(False)
        self.ser.close()


class MockDriver:
    """无硬件仿真/测试用：接口与 Servo2040Driver 一致，记录状态。"""

    def __init__(self):
        self.pulses = [1500.0] * NUM_SERVOS
        self.enabled = False
        self.touch_raw = [0] * NUM_TOUCH
        self.voltage = 7.4
        self.current = 1.0
        self.history = []  # [(t, pulses copy)]

    def set_pulses_us(self, start, values):
        for i, v in enumerate(values):
            if start + i < NUM_SERVOS:
                self.pulses[start + i] = float(v)
        self.history.append(list(self.pulses))

    def set_all_pulses_us(self, values18):
        self.set_pulses_us(0, values18)

    def enable(self, on):
        self.enabled = bool(on)

    def read_touch_raw(self):
        return list(self.touch_raw)

    def read_touch(self):
        return [v > TOUCH_THRESHOLD_RAW for v in self.touch_raw]

    def read_voltage_v(self):
        return self.voltage

    def read_current_a(self):
        return self.current

    def close(self):
        self.enabled = False
