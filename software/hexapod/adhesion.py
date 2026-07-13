"""吸附控制（P1 台架 / P2 单腿 / P4 整机）。

每足一个状态机：
  RELEASED -> (request_attach) PRESSING -> SUCKING -> ATTACHED
  ATTACHED -> (request_release) VENTING -> RELEASED
  SUCKING 超时 -> FAULT（由步态层决定重试：抬脚 5mm 重新压紧）

关键安全约定：只有 ATTACHED 状态的脚才允许承重；
爬墙步态每次抬脚前必须确认其余脚 attached_count >= 4。

硬件层通过 VacuumIO 抽象：
  - MockVacuumIO       无硬件仿真（一阶压力动力学），测试/开发用
  - Pi5VacuumIO        树莓派 5 实机实现：GPIO+MOSFET 控阀/泵，ADS1115 读
                       XGZP6847A 压力（待 P1 台架阶段按实际接线补全 TODO）
"""
import time
from enum import Enum


ATTACH_KPA = -30.0      # 达到此真空度视为吸附成功
RELEASE_KPA = -5.0      # 高于此视为已释放
SUCK_TIMEOUT_S = 0.8    # 抽气超时 -> FAULT
VENT_TIME_S = 0.4       # 放气时长
PRESS_TIME_S = 0.3      # 压紧等待（由步态层保证足端已压到位）
PUMP_ON_KPA = -45.0     # 储气罐压力高于此启动泵
PUMP_OFF_KPA = -65.0    # 低于此停泵


class FootState(Enum):
    RELEASED = "released"
    PRESSING = "pressing"
    SUCKING = "sucking"
    ATTACHED = "attached"
    VENTING = "venting"
    FAULT = "fault"


class MockVacuumIO:
    """一阶模型仿真：阀开 -> 足压力趋近罐压；阀关放气 -> 趋近 0。"""

    def __init__(self, n_feet=6, tau_suck=0.15, tau_vent=0.1):
        self.n = n_feet
        self.tau_suck, self.tau_vent = tau_suck, tau_vent
        self.valve = [False] * n_feet     # True = 接通真空
        self.pump = False
        self.tank_kpa = 0.0
        self.foot_kpa = [0.0] * n_feet
        self.sealed = [True] * n_feet     # False 模拟吸盘没贴好/漏气

    def set_valve(self, i, on):
        self.valve[i] = bool(on)

    def set_pump(self, on):
        self.pump = bool(on)

    def read_foot_kpa(self, i):
        return self.foot_kpa[i]

    def read_tank_kpa(self):
        return self.tank_kpa

    def step(self, dt):
        if self.pump:
            self.tank_kpa += (-80.0 - self.tank_kpa) * min(1.0, dt / 1.0)
        else:
            self.tank_kpa += (0.0 - self.tank_kpa) * min(1.0, dt / 60.0)  # 慢泄漏
        for i in range(self.n):
            if self.valve[i] and self.sealed[i]:
                target, tau = self.tank_kpa, self.tau_suck
            elif self.valve[i]:
                target, tau = -3.0, self.tau_suck  # 没密封，抽不下去
            else:
                target, tau = 0.0, self.tau_vent
            self.foot_kpa[i] += (target - self.foot_kpa[i]) * min(1.0, dt / tau)


class Pi5VacuumIO:
    """树莓派 5 实机 IO。P1 台架:1 阀 1 泵 1 传感器;P4 扩到 6 阀 2 泵。"""
    VALVE_PINS = [5]
    PUMP_PIN = 20
    VALVE_ON_LEVEL = 0     # set_valve(True)=吸盘接通真空 的 GPIO 电平,按实测为 0
    ADS_ADDR = 0x48
    V_DIV = 2.0            # 1:1 电阻分压
    V_ATM = 4.486          # 第 5 步实测大气点电压
    KPA_PER_V = 25.0       # 实测斜率
    GPIOCHIP = 4           # 树莓派 5

    def __init__(self, n_feet=1):
        import lgpio
        from smbus2 import SMBus
        self._lg, self.n = lgpio, n_feet
        try:
            self._h = lgpio.gpiochip_open(self.GPIOCHIP)
        except Exception:
            self._h = lgpio.gpiochip_open(0)
            
        for p in self.VALVE_PINS[:n_feet]:
            lgpio.gpio_claim_output(self._h, p, 1 - self.VALVE_ON_LEVEL)
        lgpio.gpio_claim_output(self._h, self.PUMP_PIN, 0)
        self._bus = SMBus(1)

    def set_valve(self, i, on):
        self._lg.gpio_write(self._h, self.VALVE_PINS[i],
                            self.VALVE_ON_LEVEL if on else 1 - self.VALVE_ON_LEVEL)

    def set_pump(self, on):
        self._lg.gpio_write(self._h, self.PUMP_PIN, 1 if on else 0)

    def _read_v(self, ch=0):
        import time
        self._bus.write_i2c_block_data(self.ADS_ADDR, 0x01, [0xC3 + (ch << 4), 0x83])
        time.sleep(0.01)
        hi, lo = self._bus.read_i2c_block_data(self.ADS_ADDR, 0x00, 2)
        raw = (hi << 8) | lo
        return (raw - 65536 if raw > 32767 else raw) * 4.096 / 32768

    def read_foot_kpa(self, i):
        return self.KPA_PER_V * (self._read_v(0) * self.V_DIV - self.V_ATM)

    def read_tank_kpa(self):
        return self.read_foot_kpa(0)

    def close(self):
        self._bus.close()
        self._lg.gpiochip_close(self._h)


class AdhesionController:
    """6 足吸附状态机。每个控制周期调用 update(dt)。"""

    def __init__(self, io, n_feet=6):
        self.io = io
        self.n = n_feet
        self.state = [FootState.RELEASED] * n_feet
        self._t_enter = [0.0] * n_feet
        self._now = 0.0

    # --- 步态层接口 ---
    def request_attach(self, i):
        if self.state[i] in (FootState.RELEASED, FootState.FAULT):
            self._set(i, FootState.PRESSING)

    def request_release(self, i):
        if self.state[i] == FootState.ATTACHED:
            self._set(i, FootState.VENTING)

    def is_attached(self, i):
        return self.state[i] == FootState.ATTACHED

    def attached_count(self):
        return sum(1 for s in self.state if s == FootState.ATTACHED)

    def clear_fault(self, i):
        if self.state[i] == FootState.FAULT:
            self.io.set_valve(i, False)
            self._set(i, FootState.RELEASED)

    # --- 状态机 ---
    def _set(self, i, st):
        self.state[i] = st
        self._t_enter[i] = self._now

    def update(self, dt):
        self._now += dt
        # 泵滞环控制储气罐压力
        tank = self.io.read_tank_kpa()
        if tank > PUMP_ON_KPA:
            self.io.set_pump(True)
        elif tank < PUMP_OFF_KPA:
            self.io.set_pump(False)

        for i in range(self.n):
            st = self.state[i]
            el = self._now - self._t_enter[i]
            if st == FootState.PRESSING:
                if el >= PRESS_TIME_S:
                    self.io.set_valve(i, True)
                    self._set(i, FootState.SUCKING)
            elif st == FootState.SUCKING:
                if self.io.read_foot_kpa(i) <= ATTACH_KPA:
                    self._set(i, FootState.ATTACHED)
                elif el > SUCK_TIMEOUT_S:
                    self.io.set_valve(i, False)
                    self._set(i, FootState.FAULT)
            elif st == FootState.ATTACHED:
                # 吸附中漏气报警（压力回升说明脱吸风险）
                if self.io.read_foot_kpa(i) > ATTACH_KPA + 10:
                    self.io.set_valve(i, True)  # 保持抽气尝试挽救
            elif st == FootState.VENTING:
                self.io.set_valve(i, False)
                if el >= VENT_TIME_S and self.io.read_foot_kpa(i) >= RELEASE_KPA:
                    self._set(i, FootState.RELEASED)

        if hasattr(self.io, "step"):
            self.io.step(dt)  # 仿真 IO 推进物理
