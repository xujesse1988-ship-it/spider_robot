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
    """树莓派 5 实机 IO（P1 台架阶段完成）。

    接线（按 docs/CLIMBING-DESIGN.md 气路图，引脚号在此处集中定义）：
      - 6 路阀 + 1 路泵经 8 路 MOSFET 板接 GPIO（lgpio 驱动）
      - XGZP6847A 模拟输出接 ADS1115（I2C 0x48/0x49），转换公式见传感器手册
    """

    VALVE_PINS = [5, 6, 13, 16, 19, 26]   # TODO: 按实际接线修改
    PUMP_PIN = 20                          # TODO
    ADS1115_ADDRS = (0x48, 0x49)           # TODO

    def __init__(self):
        raise NotImplementedError("P1 台架阶段按实际接线实现（lgpio + smbus2）")


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
