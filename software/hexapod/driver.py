"""Servo2040 驱动 —— Chica 串口协议。

协议从固件源码逐字节确认（EddieCarrera/chica-servo2040-simpleDriver，MIT）：

  SET 包:  0xD3, startIdx, count, [低7位, 高7位] * count
  GET 包:  0xC7, startIdx, count
  GET 响应: 0xC7, startIdx, count, 然后每项 [低7位, 高7位]

索引表（cmdPins）：0..17 舵机 SERVO1..18；18..23 足底开关 TS1..TS6；
24 电流；25 电压；26 RELAY（固件内部舵机使能；其 GPIO26 引脚 08-15 起悬空不接）；
27/28 A1/A2。

值均为 14 位。舵机值 = 脉宽 µs。换算（来自固件常量）：
  电压 V = raw / 310.3        电流 A = (raw - 512) * 0.0814

固件逐字节读取超时 100µs —— 一个数据包必须一次 write() 发完，不能分段。
上电后舵机不动，直到固件使能（arm）；先发好全部脉宽再使能，舵机会直接到位。

物理继电器（舵机 7.4V 总闸，切正极高边）08-15 定案改由 Pi GPIO17 驱动，
不再走 Servo2040 A0/GPIO26：enable() 串口 RELAY 指令照发（保固件使能），
功率通断由 GPIO17 收口。高电平吸合（模块跳线插 H 侧）；lgpio 进程退出后
GPIO 回输入态 → 继电器释放 → 舵机断电（常闭阀保真空），天然急停兜底。

合闸拆两步（09-03）：enable(True) = power_on()（只合 GPIO17，舵机带电但固件
未使能=无脉冲不出力）→ 等 ARM_DELAY_S → arm()（固件使能，18 舵机同刻开始向
目标位出力）。原先固件使能在前、合闸在后，带电冲击与出力冲击叠在同一瞬间：
09-03 实机黑匣子（lean_20260903_135325.log）合闸后 <50ms Pi 5 整机死机（5V
输入空载仅 4.89V，余量极薄）。拆开后两次冲击各自峰值更小，且黑匣子在中间
采样能分清是哪一记把 5V 拽过线。分闸顺序不变：先断功率再撤固件使能。
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

    RELAY_PIN = 17   # Pi GPIO17（Pin11）→ 继电器 IN1，高电平吸合（跳线 H 侧）
    GPIOCHIP = 4     # 树莓派 5
    ARM_DELAY_S = 0.4   # 合闸→固件使能的间隔（模块头"合闸拆两步"）

    def __init__(self, port: str = "/dev/ttyACM0", timeout: float = 0.2):
        import serial  # 延迟导入，无 pyserial 时仍可用 MockDriver
        import lgpio   # 物理继电器在 Pi GPIO17，缺 lgpio 时禁止静默降级
        self.ser = serial.Serial(port, baudrate=115200, timeout=timeout)
        time.sleep(0.2)  # 等固件退出 LED 等待循环
        self.ser.reset_input_buffer()
        self._lg = lgpio
        try:
            self._h = lgpio.gpiochip_open(self.GPIOCHIP)
        except Exception:
            self._h = lgpio.gpiochip_open(0)
        lgpio.gpio_claim_output(self._h, self.RELAY_PIN, 0)  # 初始必断

    # --- 输出 ---
    def set_pulses_us(self, start: int, values) -> None:
        self.ser.write(encode_set(start, values))

    def set_all_pulses_us(self, values18) -> None:
        assert len(values18) == NUM_SERVOS
        self.ser.write(encode_set(IDX_SERVO1, values18))

    def power_on(self) -> None:
        """只合物理继电器（GPIO17）：舵机带电，固件未使能=无脉冲不出力。"""
        self._lg.gpio_write(self._h, self.RELAY_PIN, 1)

    def arm(self) -> None:
        """固件使能：开始输出脉冲，18 舵机同刻向目标位出力。须先 power_on。"""
        self.ser.write(encode_set(IDX_RELAY, [1]))

    def enable(self, on: bool) -> None:
        # 合闸拆两步（模块头）：合继电器 → ARM_DELAY_S → 固件使能；
        # 分闸：先断功率再撤固件使能——通断始终由 GPIO17 收口
        if on:
            self.power_on()
            time.sleep(self.ARM_DELAY_S)
            self.arm()
        else:
            self._lg.gpio_write(self._h, self.RELAY_PIN, 0)
            self.ser.write(encode_set(IDX_RELAY, [0]))

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
        self._lg.gpiochip_close(self._h)


class MockDriver:
    """无硬件仿真/测试用：接口与 Servo2040Driver 一致，记录状态。"""

    is_mock = True   # 时序敏感调用方（如 glide_to）据此跳过真实墙钟等待

    def __init__(self):
        self.pulses = [1500.0] * NUM_SERVOS
        self.enabled = False
        self.powered = False   # 物理继电器（power_on）；enabled=固件使能（arm）
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

    def power_on(self):
        self.powered = True

    def arm(self):
        self.enabled = True

    def enable(self, on):
        if on:
            self.power_on()
            self.arm()
        else:
            self.enabled = self.powered = False

    def read_touch_raw(self):
        return list(self.touch_raw)

    def read_touch(self):
        return [v > TOUCH_THRESHOLD_RAW for v in self.touch_raw]

    def read_voltage_v(self):
        return self.voltage

    def read_current_a(self):
        return self.current

    def close(self):
        self.enabled = self.powered = False
