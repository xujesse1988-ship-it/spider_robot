"""吸附控制（P1 台架 / P2 单腿 / P4 整机）。

每足一个状态机：
  RELEASED -> (request_attach) PRESSING -> SUCKING -> ATTACHED
  ATTACHED -> (request_release) VENTING -> RELEASED
  SUCKING 超时 -> FAULT（由步态层决定重试：抬脚 5mm 重新压紧）
  ATTACHED 漏气（压力回升）-> is_leaking(i)=True，保持抽气挽救；
           挽救超时的"全机冻结"由步态层（climb.ClimbEngine）执行

关键安全约定：只有 ATTACHED 状态的脚才允许承重；
爬墙步态每次抬脚前必须确认其余脚全部 ATTACHED（P4 首爬用 5/5，见 climb.py）。

判据防毛刺（P4-GUIDE 第 2 步之 4）：足压是判 ATTACHED 的唯一依据，一次假的
深负压读数会让状态机放行抬腿、五足支撑变四足。所以 ATTACHED / 漏气 / 恢复
都改成"条件连续成立一段时间"才翻状态，单点毛刺（静电注入、泵阀开关瞬变）
被时间窗滤掉。

硬件层通过 VacuumIO 抽象：
  - MockVacuumIO       无硬件仿真（一阶压力动力学），测试/开发用
  - Pi5VacuumIO        树莓派 5 实机：GPIO+YYNMOS-8 控 6 阀+泵，
                       2×ADS1115 读 6 足压 + 1 罐压（XGZP6847A）
"""
import time
from enum import Enum


ATTACH_KPA = -30.0      # 达到此真空度视为吸附成功
RELEASE_KPA = -5.0      # 高于此视为已释放
SUCK_TIMEOUT_S = 0.8    # 抽气超时 -> FAULT
VENT_TIME_S = 0.4       # 放气时长
PRESS_TIME_S = 0.3      # 压紧等待（由步态层保证足端已压到位）
PUMP_ON_KPA = -60.0     # 储气罐压力高于此启动泵
PUMP_OFF_KPA = -75.0    # 低于此停泵。⚠ 555 泵标称极限 -75~-85（BOM），本线顶着
                        # 标称下限；且无罐滞环作用在盘压上，每足单向阀装机后
                        # 盘压下限 = 泵极限 + 阀开启压差。08-18 地面无罐实测
                        # 收口：-75 可达且泵正常停，抽气频次反比 -55/-65 更低
                        # （带宽 15 + 低漏率），本线可用；换泵/改气路后需复验
                        # ——届时过不了线就回调到 plateau 浅侧 ≥5kPa 处（带宽
                        # 15 保持，双线一起挪）
# 防毛刺时间窗：条件连续成立这么久才翻状态（比计次判据更稳——不依赖采样率）。
# Pi5VacuumIO 有 CACHE_S=0.05 读数缓存 + 每周期突发限流（最坏陈旧 ~0.11s），
# 窗必须显著大于最坏陈旧期才保证跨 ≥2 次真实采样。
# 超时判据只在压力未达标时计时（见 SUCKING 分支），确认窗不挤占超时预算。
ATTACH_CONFIRM_S = 0.2   # SUCKING -> ATTACHED 确认窗
LEAK_DELTA_KPA = 10.0    # ATTACHED 后压力回升超过此幅度算疑似漏气
LEAK_CONFIRM_S = 0.25    # 漏气报警 / 漏气恢复确认窗
# 罐压合理区间：出界视为传感器失效（未接时分压点被拉到地 ≈ -112kPa，
# 会骗过滞环让泵永不启动——P4-GUIDE 第 2 步之 5）
TANK_KPA_MIN, TANK_KPA_MAX = -100.0, 10.0


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
    """树莓派 5 实机 IO（P4 整机：6 阀 + 泵 A + 7 路压力）。

    传感器映射与 html/p4-divider-board.html、scripts/p4_sensor_check.py 一致：
      0x48: AIN0=S1 罐压, AIN1..3 = S2..S4 足 L1 L2 L3
      0x49: AIN0..2 = S5..S7 足 R1 R2 R3, AIN3 = S8 备用（只留焊盘）
    足序与 VALVE_PINS 同为 L1 L2 L3 R1 R2 R3，和 config.LEG_NAMES 对齐。
    """
    VALVE_PINS = [5, 6, 13, 16, 19, 21]   # L1 L2 L3 R1 R2 R3,08-14 点动实测通过
    PUMP_PIN = 20                          # 泵 A;泵 B(GPIO26)V0 空置
    PUMP_B_PIN = 26                        # 只定义不驱动（首飞单泵，P4-GUIDE 第 2 步之 1）
    # GPIO17 已被舵机继电器占用（driver.Servo2040Driver.RELAY_PIN），扩展别用
    VALVE_ON_LEVEL = 0     # set_valve(True)=吸盘接通真空 的 GPIO 电平,按实测为 0
    V_DIV = 2.0            # 1:1 电阻分压
    V_ATM = 4.486          # 大气点电压（2026-08-11 六路实测极差 0.3kPa，统一用一个值）
    KPA_PER_V = 25.0       # 实测斜率
    GPIOCHIP = 4           # 树莓派 5
    FOOT_ADC = [(0x48, 1), (0x48, 2), (0x48, 3),   # L1 L2 L3
                (0x49, 0), (0x49, 1), (0x49, 2)]   # R1 R2 R3
    TANK_ADC = (0x48, 0)
    # 同一通道两次真实采样的最小间隔：7 路 × 每读 ~2-3ms，不加节流会拖垮 50Hz 主环。
    # 判据端的确认窗（ATTACH_CONFIRM_S 等）必须大于最坏陈旧期，保证跨多次真实采样。
    CACHE_S = 0.05
    # 突发限流：各通道缓存会在同一控制周期集中过期（同刻回填→同刻失效），
    # 不限流的话每第 3 个周期约 15-21ms 全花在 I2C 上，舵机指令流周期性抖动。
    # 限每 BURST_WINDOW 内至多 MAX_BURST 次真实转换，其余通道先用旧值，
    # 刷新自然摊成轮转；7 路最坏陈旧 ≈ 3 个周期 + CACHE_S ≈ 0.11s。
    MAX_BURST = 3
    BURST_WINDOW = 0.015

    def __init__(self, n_feet=6):
        import lgpio
        from smbus2 import SMBus
        self._lg, self.n = lgpio, n_feet
        try:
            self._h = lgpio.gpiochip_open(self.GPIOCHIP)
        except Exception:
            self._h = lgpio.gpiochip_open(0)

        for k, p in enumerate(self.VALVE_PINS[:n_feet]):
            # claim 电平 = 排气位 = 线圈**通电**（隔离吸盘与歧管：单向阀
            # 只挡罐→盘方向，通罐位下泵预抽会把站在地上的六盘吸住）。
            # 足间垫 0.2s 把六线圈上电从同刻阶跃摊成串行（与 ESC 退出
            # 同口径）——12V 轨不再吃 ~25W 阶跃（08-18 掉电疑似扳机）
            if k:
                time.sleep(0.2)
            lgpio.gpio_claim_output(self._h, p, 1 - self.VALVE_ON_LEVEL)
        lgpio.gpio_claim_output(self._h, self.PUMP_PIN, 0)
        self._bus = SMBus(1)
        self._cache = {}      # (addr, ch) -> (kpa, 采样时刻)
        self._burst = []      # 最近 BURST_WINDOW 内真实转换的时间戳
        self.read_faults = 0  # 读失败计数（转换超时/总线失联皆记；联调
                              # 脚本 TLM IO错= 可显示，持续失败会上抛）
        # 逻辑镜像（True=接通真空 / 泵开）：黑匣子与状态显示用，不回读 GPIO。
        # 初值与上面 claim 的电平一致（阀=排气位断真空、泵停）
        self.valve = [False] * n_feet
        self.pump = False

    def set_valve(self, i, on):
        self.valve[i] = bool(on)
        self._lg.gpio_write(self._h, self.VALVE_PINS[i],
                            self.VALVE_ON_LEVEL if on else 1 - self.VALVE_ON_LEVEL)

    def set_pump(self, on):
        self.pump = bool(on)
        self._lg.gpio_write(self._h, self.PUMP_PIN, 1 if on else 0)

    def _read_v(self, addr, ch):
        # Config MSB: 0xC3+ch<<4 = OS=1 | MUX=AINch-GND | PGA=±4.096V | 单次
        # PGA 必须 ±4.096V：分压后最高 2.25V，设 ±2.048 会在放气确认区间削顶。
        # LSB: 0xE3 = 860SPS + 比较器关闭；OS 位轮询等转换完（~1.2ms），
        # 比旧版固定 sleep(0.01)@128SPS 快 5 倍——7 路轮询才喂得起 50Hz 主环。
        self._bus.write_i2c_block_data(addr, 0x01, [0xC3 + (ch << 4), 0xE3])
        deadline = time.time() + 0.05
        while time.time() < deadline:
            if self._bus.read_i2c_block_data(addr, 0x01, 2)[0] & 0x80:
                hi, lo = self._bus.read_i2c_block_data(addr, 0x00, 2)
                raw = (hi << 8) | lo
                return (raw - 65536 if raw > 32767 else raw) * 4.096 / 32768
            time.sleep(0.0005)
        # OS 位始终没置位 = 转换没完成。此时 0x00 里躺着**上一个通道**的旧值
        # （同芯片共用转换寄存器），绝不能读——罐压/足压互串会喂坏吸附判据。
        return None

    def _kpa(self, adc):
        now = time.time()
        hit = self._cache.get(adc)
        if hit and now - hit[1] < self.CACHE_S:
            return hit[0]
        # 突发限流：本窗口配额用完就先用旧值（见 MAX_BURST 注释）
        self._burst = [t for t in self._burst if now - t < self.BURST_WINDOW]
        if hit and len(self._burst) >= self.MAX_BURST:
            return hit[0]
        self._burst.append(now)
        bus_err = None
        try:
            v = self._read_v(*adc)
        except OSError as e:
            # 总线级失联（如 Errno 121 从机不应答）。08-23 实测（lean_20260823
            # _165641.log）：三次 121 全部落在泵/阀通电后 0.05~0.6s 内且均
            # <1s 自愈（EMI/电源瞬态；IO错 全场 0 = 芯片失联前完全健康）——
            # 单次 NACK 直接上抛会把瞬态毛刺放大成全机冻结。与"转换超时"
            # 同款容忍：隔 1ms 重试一次，再用 0.5s 内旧值顶，持续失联才
            # 上抛（降落伞冻结语义不变）。治本在硬件：泵/阀续流二极管、
            # ADC 板去耦、I2C 走线离 12V 束（P4-GUIDE 排障表）
            time.sleep(0.001)
            try:
                v = self._read_v(*adc)
            except OSError:
                v, bus_err = None, e
        if v is None:                       # 读失败：短期容忍用旧值，持续必须上抛
            self.read_faults += 1
            if hit and now - hit[1] < 0.5:
                return hit[0]
            if bus_err is not None:
                raise IOError(f"ADS1115 0x{adc[0]:02x} AIN{adc[1]} 总线失联"
                              f"（{bus_err}）") from bus_err
            raise IOError(f"ADS1115 0x{adc[0]:02x} AIN{adc[1]} 转换持续超时")
        kpa = self.KPA_PER_V * (v * self.V_DIV - self.V_ATM)
        self._cache[adc] = (kpa, now)
        return kpa

    def read_foot_kpa(self, i):
        return self._kpa(self.FOOT_ADC[i])

    def read_tank_kpa(self):
        return self._kpa(self.TANK_ADC)

    def close(self):
        self._bus.close()
        self._lg.gpiochip_close(self._h)


class AdhesionController:
    """6 足吸附状态机。每个控制周期调用 update(dt)。"""

    def __init__(self, io, n_feet=6, pump_without_tank=False,
                 tankless=False, suck_timeout_s=SUCK_TIMEOUT_S):
        self.io = io
        self.n = n_feet
        # 罐压传感器失效时的降级泵策略（架空联调、罐压未接时用）：
        # False = 停泵置 fault 等上层报警；True = 有脚在抽气就开泵
        self.pump_without_tank = pump_without_tank
        # 无罐模式（储气罐未装时用）：完全不读罐压传感器，
        # 泵改按"抽气需求 + 已吸附足最差压滞环"直抽歧管。没有储备真空：
        # 挽救能力弱、断电不保真空（地面/上墙均已多次实测可用）。
        self.tankless = tankless
        # 无罐预抽请求（步态层在首次抽气前置位几秒）：阀全关时先把歧管抽空，
        # 否则首足 SUCK 独扛整段大气歧管，健康系统也可能白烧重试
        self.precharge = False
        # 取机窗口（climb_walk 'oo' 放开吸盘保持站立）：置位后泵不许再开——
        # 罐模式滞环否则会继续维持罐压。一次性置位，进程退出前不复位；
        # 置位期间 tank_fault 不再刷新（吸盘已放，罐压不再是安全条件）
        self.pump_inhibit = False
        # SUCK 超时可调：无罐时没有罐的存量"瞬间咬住"，泵实时抽更慢
        self.suck_timeout_s = suck_timeout_s
        self.state = [FootState.RELEASED] * n_feet
        # 黑匣子接口（hexapod.runlog）：状态跳变钩子 + 最近读数镜像。
        # on_event(i, old, new, elapsed_s) 在 _set 内同步调用——订阅方必须轻
        # 且不抛（RunLog 自带写盘熔断）。last_* 只在状态机真实读过时更新，
        # 遥测端读镜像即可，零额外传感器 IO（不抢 I2C 突发配额）
        self.on_event = None
        self.last_kpa = [None] * n_feet
        self.last_tank_kpa = None
        # 仲裁位占位（远期双墙面混合足，P4-GUIDE 第 2 步之 6）：本阶段恒为 suction
        self.mode = ["suction"] * n_feet
        self.leaking = [False] * n_feet
        self.tank_fault = False
        self._t_enter = [0.0] * n_feet
        self._good_since = [None] * n_feet   # 压力满足吸附判据的起始时刻
        self._bad_since = [None] * n_feet    # 压力回升出漏气判据的起始时刻
        self._leak_since = [0.0] * n_feet
        self._now = 0.0

    # --- 步态层接口 ---
    def request_attach(self, i):
        if self.state[i] in (FootState.RELEASED, FootState.FAULT):
            self._set(i, FootState.PRESSING)

    def request_release(self, i):
        if self.state[i] == FootState.ATTACHED:
            self._set(i, FootState.VENTING)

    def force_release(self, i):
        """退出收尾专用：无论当前状态（含 FAULT/SUCKING/PRESSING）一律转入
        VENTING 走正常放气确认。request_release 只认 ATTACHED——从冻结态
        （FAULT）或抽气中途（SUCKING）退出的足走不到放气：白等预算、吸盘
        留在抽空态，无罐模式还会被 SUCKING 超时分支跨时隙反手通电。
        注：尚未轮到收尾时隙的 SUCKING 足仍可能先自行超时转 FAULT 并翻到
        排气位（短暂多一路线圈），轮到它时本方法照常接手收口。"""
        if self.state[i] not in (FootState.VENTING, FootState.RELEASED):
            self._set(i, FootState.VENTING)

    def abandon_release(self, i):
        """放气确认超时的弃疗出口：直接记为 RELEASED（压力未确认，调用方
        自行断阀并留日志）。必须先走这一步再断阀电——否则 VENTING 分支
        每周期无条件 set_valve(i, False)，会把调用方刚断电的阀反手重新
        通电，日志"线圈已断"与实际电平相反（审核发现：退出序列超时足）。"""
        if self.state[i] == FootState.VENTING:
            self._set(i, FootState.RELEASED)

    def is_attached(self, i):
        return self.state[i] == FootState.ATTACHED

    def attached_count(self):
        return sum(1 for s in self.state if s == FootState.ATTACHED)

    def is_leaking(self, i):
        return self.leaking[i]

    def leak_time(self, i):
        """当前这次漏气已持续多久（秒）；没漏返回 0。挽救超时判据用。"""
        return self._now - self._leak_since[i] if self.leaking[i] else 0.0

    def clear_fault(self, i):
        if self.state[i] == FootState.FAULT:
            self.io.set_valve(i, False)
            self._set(i, FootState.RELEASED)

    # --- 状态机 ---
    def _set(self, i, st):
        old = self.state[i]
        elapsed = self._now - self._t_enter[i]
        self.state[i] = st
        self._t_enter[i] = self._now
        self._good_since[i] = self._bad_since[i] = None
        self.leaking[i] = False
        if self.on_event is not None and st != old:
            self.on_event(i, old, st, elapsed)

    def _foot_kpa(self, i):
        """读足压并留镜像（黑匣子遥测取 last_kpa，不再发起 IO）。"""
        kpa = self.io.read_foot_kpa(i)
        self.last_kpa[i] = kpa
        return kpa

    def _held(self, since, window):
        """since 时刻起条件是否已连续保持 window 秒。"""
        return since is not None and self._now - since >= window

    def update(self, dt):
        self._now += dt
        if self.pump_inhibit:
            self.io.set_pump(False)
            if not self.tankless:
                # 只禁泵不禁采样：退出/取机窗口的 TLM 还在记罐压（正是
                # 08-18 事故窗口），跳过读取会让黑匣子整段记 ESC 前的陈旧
                # 值，验尸拿假数据。读失败纯遥测损失，不许打断收尾期状态机
                # （阀还要靠下面的循环驱动）。tank_fault 仍不刷新（吸盘已
                # 放，罐压不再是安全条件）
                try:
                    self.last_tank_kpa = self.io.read_tank_kpa()
                except Exception:
                    pass
        elif self.tankless:
            # 无罐：泵按需求直抽歧管——预抽期/有脚在抽气必开；否则按已吸附
            # 足的最差（最接近大气）压力做滞环维持。罐压传感器完全不参与。
            if self.precharge or \
                    any(s == FootState.SUCKING for s in self.state):
                self.io.set_pump(True)
            else:
                att = [self._foot_kpa(i) for i in range(self.n)
                       if self.state[i] == FootState.ATTACHED]
                if not att:
                    self.io.set_pump(False)
                elif max(att) > PUMP_ON_KPA:
                    self.io.set_pump(True)
                elif max(att) < PUMP_OFF_KPA:
                    self.io.set_pump(False)
        else:
            # 泵滞环控制储气罐压力；罐压出合理区间 = 传感器失效/未接，
            # 停泵置 fault 交由步态层报警（假读数 -112kPa 会骗过滞环让泵永不启动）
            tank = self.io.read_tank_kpa()
            self.last_tank_kpa = tank
            if not (TANK_KPA_MIN <= tank <= TANK_KPA_MAX):
                self.tank_fault = True
                if self.pump_without_tank:   # 降级：抽气需求驱动泵，不看罐压
                    self.io.set_pump(any(s == FootState.SUCKING
                                         for s in self.state))
                else:
                    self.io.set_pump(False)
            else:
                self.tank_fault = False
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
                if self._foot_kpa(i) <= ATTACH_KPA:
                    if self._good_since[i] is None:
                        self._good_since[i] = self._now
                    if self._held(self._good_since[i], ATTACH_CONFIRM_S):
                        self._set(i, FootState.ATTACHED)
                else:
                    self._good_since[i] = None
                    # 超时只在压力未达标时计：已经吸到 -30 只是还在确认窗里的
                    # 脚，不能被超时打成 FAULT 放掉（那是物理上密封好的脚）
                    if el > self.suck_timeout_s:
                        self.io.set_valve(i, False)
                        self._set(i, FootState.FAULT)
            elif st == FootState.ATTACHED:
                kpa = self._foot_kpa(i)
                good = kpa <= ATTACH_KPA
                bad = kpa > ATTACH_KPA + LEAK_DELTA_KPA
                if good:
                    if self._good_since[i] is None:
                        self._good_since[i] = self._now
                else:
                    self._good_since[i] = None
                if bad:
                    if self._bad_since[i] is None:
                        self._bad_since[i] = self._now
                else:
                    self._bad_since[i] = None
                if not self.leaking[i] and self._held(self._bad_since[i],
                                                      LEAK_CONFIRM_S):
                    self.leaking[i] = True
                    self._leak_since[i] = self._now
                elif self.leaking[i] and self._held(self._good_since[i],
                                                    LEAK_CONFIRM_S):
                    self.leaking[i] = False
                if bad:
                    self.io.set_valve(i, True)  # 保持抽气尝试挽救
            elif st == FootState.VENTING:
                self.io.set_valve(i, False)
                if el >= VENT_TIME_S and self._foot_kpa(i) >= RELEASE_KPA:
                    self._set(i, FootState.RELEASED)

        if hasattr(self.io, "step"):
            self.io.step(dt)  # 仿真 IO 推进物理
