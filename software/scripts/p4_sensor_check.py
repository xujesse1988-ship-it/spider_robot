#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P4 分压汇接板 + XGZP6847A 气压传感器 上电自检 / 读数验证（树莓派 5 上跑）。

对应图纸：
  html/p4-divider-board.html   7 路 10k/10k 分压 + ADS1115 ×2（0x48 / 0x49）
  html/p4-pi-wiring.html       Pi 物理脚 1(3.3V) / 2(5V) / 3(SDA1) / 5(SCL1) / 9(GND) → J1

安全边界：本脚本**只读 I2C，一个 GPIO 都不碰**——阀和泵在这里绝不会动作。
所以气路没装、MOSFET 板没接的时候也能安全跑。

量程方向（P1 台架实测，adhesion.py:84）：
  传感器 4.5V = 大气 0kPa，0.5V = -100kPa，斜率 25 kPa/V。
  经 1:1 分压后 → 大气点 ADC ≈ 2.243V；未接的通道被下臂 10k 拉到地 ≈ 0.00V。
  （注意：分压板图纸里"大气 ≈0.25V"那句描述写反了，以本文件与实测为准。）

用法：
  python3 scripts/p4_sensor_check.py              # 一次性自检报告
  python3 scripts/p4_sensor_check.py --live       # 连续监视（吸气测试用这个）
  python3 scripts/p4_sensor_check.py --wiggle 7   # 锁定 7 路高速抓瞬断（晃线时读数跳）
  python3 scripts/p4_sensor_check.py --zero       # 逐路标定大气点 V_ATM，存表
  python3 scripts/p4_sensor_check.py --live --fixed-atm   # 用固定 4.486 而非自动基线
"""
import argparse
import json
import math
import sys
import time
from pathlib import Path

# ---- 电学常数 ----
V_DIV = 2.0             # 10k/10k 分压，还原系数
KPA_PER_V = 25.0        # (4.5-0.5)V ↔ 100kPa
V_ATM_NOMINAL = 4.486   # P1 实测大气点（传感器侧，未分压）
PGA_FSR = 4.096         # ADS1115 PGA=±4.096V（±2.048 会在放气确认区削顶）
V_ADC_ATM = V_ATM_NOMINAL / V_DIV   # ≈2.243V，分压板上应该看到的大气点

# ---- 通道映射：与 html/p4-divider-board.html 的映射表一致 ----
# (通道号, S口, 名称, ADS 地址, AIN, 类别)
# 类别 spare = 板图里"只留焊盘"的第 8 路：分压电阻没装，AIN 悬空，读数飘是正常的
CHANNELS = [
    (1, "S1", "罐压",   0x48, 0, "tank"),
    (2, "S2", "足 L1",  0x48, 1, "foot"),
    (3, "S3", "足 L2",  0x48, 2, "foot"),
    (4, "S4", "足 L3",  0x48, 3, "foot"),
    (5, "S5", "足 R1",  0x49, 0, "foot"),
    (6, "S6", "足 R2",  0x49, 1, "foot"),
    (7, "S7", "足 R3",  0x49, 2, "foot"),
    (8, "—",  "备用",   0x49, 3, "spare"),
]

VATM_TABLE_PATH = Path(__file__).resolve().parents[2] / "docs" / "data" / "p4_vatm.json"


class ADS1115:
    """单次转换模式读 ADS1115，PGA 固定 ±4.096V、128SPS。"""

    def __init__(self, bus, addr):
        self.bus, self.addr = bus, addr

    def present(self):
        try:
            self.bus.read_i2c_block_data(self.addr, 0x01, 2)
            return True
        except OSError:
            return False

    def read_v(self, ch):
        # Config MSB: OS=1 | MUX=100+ch (AINch-GND) | PGA=001(±4.096V) | MODE=1(单次)
        # Config LSB: 0x83 = 128SPS + 比较器关闭
        self.bus.write_i2c_block_data(self.addr, 0x01, [0xC3 + (ch << 4), 0x83])
        deadline = time.time() + 0.2
        while time.time() < deadline:
            time.sleep(0.002)
            if self.bus.read_i2c_block_data(self.addr, 0x01, 2)[0] & 0x80:
                break   # OS 位回 1 = 转换完成
        return self._raw_to_v(self.bus.read_i2c_block_data(self.addr, 0x00, 2))

    @staticmethod
    def _raw_to_v(b):
        raw = (b[0] << 8) | b[1]
        return (raw - 65536 if raw > 32767 else raw) * PGA_FSR / 32768

    def start_continuous(self, ch):
        """连续转换 860SPS，锁定单通道——抓毫秒级瞬断用。

        单次模式下 6 路轮采、每路 0.15s 才看一眼，1ms 量级的接触不良基本采不到。
        锁定一路 + 连续模式后采样率能到几百 Hz，才有资格说"没抓到就是没有"。
        """
        # MSB: OS=1 | MUX=100+ch | PGA=001(±4.096V) | MODE=0(连续)
        # LSB: DR=111(860SPS) | 比较器关闭
        self.bus.write_i2c_block_data(self.addr, 0x01, [0xC2 + (ch << 4), 0xE3])
        time.sleep(0.01)

    def read_last(self):
        return self._raw_to_v(self.bus.read_i2c_block_data(self.addr, 0x00, 2))

    def stop_continuous(self):
        self.bus.write_i2c_block_data(self.addr, 0x01, [0xC3, 0x83])


def to_kpa(v_adc, v_atm):
    return KPA_PER_V * (v_adc * V_DIV - v_atm)


def classify(v_adc, kind="foot"):
    """按"当前处于大气压"的前提判定一路的接线状态。"""
    if kind == "spare":
        return "空位", ("板图第 8 路只留焊盘、分压电阻没装，AIN 悬空 —— "
                        "电压飘在中间值、噪声比其他路大一个量级都是正常的")
    if v_adc < 0.10:
        if kind == "tank":
            return "未接", "罐压传感器还没接（储气罐没到货），预期如此；" \
                           "读数贴地也说明该路下臂 R_B 是好的"
        return "未接", "分压点被下臂 10k 拉到地——传感器没插 / S 口断线 / 没给 5V"
    if v_adc < 0.90:
        return "异常低", "≈满真空才该这么低。查：传感器 5V 供电、S 口三根线顺序、传感器本身"
    if abs(v_adc - V_ADC_ATM) <= 0.10:
        return "正常", ""
    if v_adc < 2.40:
        return "偏低", "在线但偏离大气点 >0.1V(2.5kPa)。管口有残余负压？还是该路分压电阻偏差大"
    if v_adc < 3.30:
        return "偏高", "疑似下臂 R_B 虚焊/开路——分压没分成，ADS 正被灌高压"
    return "危险", "已接近/超过 ADS 的 3.3V 供电，立刻断电查该路分压，别再上电"


def sample(readers, n=16):
    """每路采 n 次，返回 {通道号: (均值, 峰峰值)}。"""
    acc = {ch[0]: [] for ch in CHANNELS}
    for _ in range(n):
        for num, _s, _name, addr, ain, _k in CHANNELS:
            r = readers.get(addr)
            if r is not None:
                acc[num].append(r.read_v(ain))
    out = {}
    for num, vs in acc.items():
        if vs:
            out[num] = (sum(vs) / len(vs), max(vs) - min(vs))
    return out


def open_bus(busnum):
    try:
        from smbus2 import SMBus
    except ImportError:
        sys.exit("缺 smbus2。在树莓派的 venv 里装：pip install smbus2\n"
                 "（本仓库：pip install -e '.[pi]'）")
    try:
        return SMBus(busnum)
    except (OSError, PermissionError) as e:
        sys.exit(f"打不开 I2C bus {busnum}：{e}\n"
                 "查：raspi-config 里 I2C 是否已启用；当前用户是否在 i2c 组。")


def find_readers(bus):
    """探测 0x48 / 0x49，返回 {addr: ADS1115}。"""
    readers, missing = {}, []
    for addr in sorted({c[3] for c in CHANNELS}):
        r = ADS1115(bus, addr)
        if r.present():
            readers[addr] = r
        else:
            missing.append(addr)
    return readers, missing


# ---------------- 报告模式 ----------------

def report(bus, samples):
    readers, missing = find_readers(bus)
    print("=" * 78)
    print("P4 分压汇接板自检   （量程方向：大气 4.5V / -100kPa 0.5V，大气点 ADC 应 ≈%.3fV）"
          % V_ADC_ATM)
    print("=" * 78)
    for addr in (0x48, 0x49):
        tag = "✅ 在线" if addr in readers else "❌ 未应答"
        who = "#1(ADDR→GND)" if addr == 0x48 else "#2(ADDR→VDD)"
        print(f"  ADS1115 0x{addr:02X} {who:14s} {tag}")
    if missing:
        print("\n  ⚠ 有模块没应答，先查：3.3V/GND 是否到位、SDA/SCL 有没有对调、"
              "0x49 的 ADDR→VDD 飞线有没有焊上。")
        print("    命令行交叉验证：i2cdetect -y 1")
    if not readers:
        return None

    data = sample(readers, samples)
    print(f"\n  每路采样 {samples} 次：\n")
    print("  通道 S口  名称    ADC(V)  传感器(V)  p-p(mV)    kPa   判定")
    print("  " + "-" * 68)
    online, faulty, notconn, spare = [], [], [], []
    for num, s, name, addr, ain, kind in CHANNELS:
        if num not in data:
            print(f"   {num}   {s:3s} {name:6s}   ——  该路的 ADS(0x{addr:02X}) 未应答")
            continue
        mean, pp = data[num]
        tag, note = classify(mean, kind)
        kpa = ("    ——" if tag in ("未接", "空位")
               else f"{to_kpa(mean, V_ATM_NOMINAL):6.1f}")
        print(f"   {num}   {s:3s} {name:6s} {mean:6.3f}   {mean*V_DIV:6.3f}   "
              f"{pp*1000:6.1f}  {kpa}   {tag}")
        if note:
            print(f"        └ {note}")
        if tag in ("正常", "偏低"):
            online.append((num, name, mean, pp))
        elif tag == "未接":
            notconn.append((num, name))
        elif tag == "空位":
            spare.append(num)
        else:
            faulty.append((num, name, tag))

    print("\n  —— 小结 ——")
    if faulty:
        print("  ❌ 有问题的路：" + "、".join(f"{n}({nm}) {t}" for n, nm, t in faulty))
        print("     先断电，用万用表复查这些路：S口OUT→分压点 ≈10k、分压点→GND ≈10k。")
    if notconn:
        print("  ○ 未接 " + "、".join(f"{n}({nm})" for n, nm in notconn) +
              " —— 读数贴地是好事，说明这些路的下臂 R_B 有效")
    if spare:
        print("  ○ 空位 通道 " + "、".join(str(n) for n in spare) +
              " —— 只留焊盘、没装分压，AIN 悬空，读数飘属正常，不用管")
    if not online:
        print("  没有任何一路读到正常的传感器。先按上面的提示排线。")
        return data
    print(f"  ✅ 在线 {len(online)} 路：" + "、".join(f"{n}({name})" for n, name, _, _ in online))

    vs = [m for _, _, m, _ in online]
    spread = (max(vs) - min(vs)) * V_DIV
    print(f"  大气点一致性：在线各路传感器电压极差 {spread*1000:.0f} mV "
          f"(≈{spread*KPA_PER_V:.1f} kPa)  ", end="")
    print("✅ 一致" if spread <= 0.10 else "⚠ 偏大，逐只标 V_ATM（--zero）后按表换算")

    worst = max(online, key=lambda x: x[3])
    print(f"  噪声最大一路：通道 {worst[0]}({worst[1]}) p-p {worst[3]*1000:.1f} mV "
          f"(≈{worst[3]*V_DIV*KPA_PER_V:.2f} kPa)  ", end="")
    print("✅ 干净" if worst[3] < 0.005 else
          "⚠ 偏抖，查 100nF 有没有焊上 / GND 走线 / 模拟线是否贴着功率线走")
    print("\n  下一步：python3 scripts/p4_sensor_check.py --live   然后对着传感器管口吸气")
    return data


# ---------------- 实时监视模式 ----------------

def bar(kpa, width=22, full=-60.0):
    n = int(max(0.0, min(1.0, kpa / full)) * width)
    return "█" * n + "·" * (width - n)


def live(bus, fixed_atm, samples):
    readers, _missing = find_readers(bus)
    if not readers:
        sys.exit("没有 ADS1115 应答，先跑一次不带参数的自检。")

    base = sample(readers, samples)
    active, bad = [], []
    for c in CHANNELS:
        if c[0] not in base:
            continue
        tag, _ = classify(base[c[0]][0], c[5])
        if tag in ("正常", "偏低"):
            active.append(c)
        elif tag not in ("未接", "空位"):
            bad.append((c[0], c[2], tag))
    if bad:
        print("⚠ 以下路判定异常，已排除在监视之外（先跑不带参数的自检看原因）：")
        for num, name, tag in bad:
            print(f"   通道 {num} {name} — {tag}")
    if not active:
        sys.exit("没有一路检测到在线传感器，先跑一次不带参数的自检。")

    if fixed_atm:
        v_atm = {c[0]: V_ATM_NOMINAL for c in active}
        print(f"大气基线：固定值 {V_ATM_NOMINAL:.3f} V（--fixed-atm）")
    else:
        v_atm = {c[0]: base[c[0]][0] * V_DIV for c in active}
        print("大气基线：启动时自动采集（要求此刻所有传感器都通大气、没堵没吸）")
    for num, s, name, _a, _i, _k in active:
        print(f"  通道 {num} {s:3s} {name:6s} V_ATM = {v_atm[num]:.3f} V")

    lo = {c[0]: 0.0 for c in active}   # 本次最深负压
    print("\n对着某一路传感器的管口吸气（别吹！正压超量程），下面那路应该往负走。")
    print("Ctrl-C 结束。\n")
    nlines = len(active) + 2
    first = True
    tty = sys.stdout.isatty()
    try:
        while True:
            rows = []
            for num, s, name, addr, ain, _k in active:
                v = readers[addr].read_v(ain)
                kpa = to_kpa(v, v_atm[num])
                lo[num] = min(lo[num], kpa)
                mark = "  ← 负压" if kpa < -2.0 else ""
                rows.append(f"  {num} {s:3s} {name:6s} {v:6.3f}V  {kpa:7.1f} kPa  "
                            f"[{bar(kpa)}]  最深 {lo[num]:6.1f}{mark}")
            if tty and not first:
                sys.stdout.write(f"\033[{nlines}A")
            first = False
            print("  通道 S口  名称     ADC      气压          真空条          本次最深")
            print("  " + "-" * 74)
            for r in rows:
                print(r)
            sys.stdout.flush()
            time.sleep(0.15)
    except KeyboardInterrupt:
        print("\n\n本次各路最深负压：")
        ok = False
        for num, s, name, _a, _i, _k in active:
            verdict = "✅ 读到负压，该路通" if lo[num] < -5.0 else "—  没吸过 / 没读到变化"
            print(f"  通道 {num} {s:3s} {name:6s} {lo[num]:7.1f} kPa   {verdict}")
            ok = ok or lo[num] < -5.0
        if ok:
            print("\n气压读取链路（传感器 → 分压 → ADS → I2C → Pi）验证通过。")
        else:
            print("\n没有任何一路读到负压。若吸气时电压纹丝不动：查该路 S 口 OUT 线、"
                  "传感器 5V 供电、或换一路传感器交叉验证。")


# ---------------- 抓瞬断模式 ----------------

WIGGLE_STEPS = [
    "传感器本体（拿手轻轻晃）",
    "三根线在传感器根部的出线处（逐根轻捻、轻拉）",
    "线束中段",
    "线在分压板 S 口接头处（轻推轻拔，别用力）",
    "S 口接头外壳本身",
    "用指甲/塑料镊子轻敲板上该路的 R_A / R_B / 100nF 焊点",
    "轻掰一下板子（模拟机身震动下的形变）",
]


def wiggle(bus, chnum, thresh_kpa):
    """锁定一路高速采样，边晃线边抓瞬断。"""
    readers, _ = find_readers(bus)
    entry = next((c for c in CHANNELS if c[0] == chnum), None)
    if entry is None:
        sys.exit(f"没有通道 {chnum}，可选 1~8。")
    num, s, name, addr, ain, kind = entry
    if addr not in readers:
        sys.exit(f"通道 {num} 挂在 0x{addr:02X}，该模块没应答。")

    r = readers[addr]
    tag, _ = classify(r.read_v(ain), kind)
    if tag not in ("正常", "偏低"):
        sys.exit(f"通道 {num}({name}) 当前判定「{tag}」，不是稳态在线，先跑不带参数的自检。")

    print(f"抓瞬断：通道 {num} {s} {name}   （0x{addr:02X} AIN{ain}）")
    r.start_continuous(ain)
    time.sleep(0.05)

    t0 = time.time()
    base = []
    while time.time() - t0 < 1.5:
        base.append(r.read_last())
    base.sort()
    v_base = base[len(base) // 2]
    rate = len(base) / 1.5
    noise_kpa = (base[-1] - base[0]) * V_DIV * KPA_PER_V
    print(f"  基线 {v_base:.4f}V（{v_base*V_DIV:.3f}V 传感器侧）  "
          f"静态噪声 {noise_kpa:.2f} kPa  采样率 ≈{rate:.0f} Hz")
    print(f"  报警阈值 ±{thresh_kpa:.1f} kPa —— 超过就是接触问题，不是噪声\n")
    print("  按顺序逐项做，每项晃 10 秒，记下哪一项能触发：")
    for i, step in enumerate(WIGGLE_STEPS, 1):
        print(f"    {i}. {step}")
    print("\n  Ctrl-C 结束。\n")

    n = ev = 0
    worst = 0.0
    signs = []
    in_ev = False
    ev_peak = 0.0
    ev_t0 = 0.0
    calm = 0
    t_start = time.time()
    last_status = 0.0
    try:
        while True:
            v = r.read_last()
            n += 1
            dev = (v - v_base) * V_DIV * KPA_PER_V
            if abs(dev) > thresh_kpa:
                if not in_ev:
                    in_ev, ev_peak, ev_t0 = True, dev, time.time()
                elif abs(dev) > abs(ev_peak):
                    ev_peak = dev
                calm = 0
            elif in_ev:
                calm += 1
                if calm >= 5:
                    in_ev = False
                    ev += 1
                    worst = max(worst, abs(ev_peak))
                    signs.append(1 if ev_peak > 0 else -1)
                    why = ("电压掉 → OUT 线开路或 5V 丢失（分压被下臂拉向地）"
                           if ev_peak < 0 else "电压升 → GND 侧接触不良")
                    dur = (time.time() - ev_t0) * 1000
                    sys.stdout.write("\r\033[K")
                    print(f"  ⚡ 瞬断 #{ev}  t={time.time()-t_start:5.1f}s  "
                          f"峰值 {ev_peak:+7.1f} kPa  持续 {dur:.0f} ms   {why}")
            now = time.time()
            if now - last_status > 0.3:
                last_status = now
                sys.stdout.write(f"\r  已采 {n} 点 / {now-t_start:.0f}s "
                                 f"({n/max(now-t_start,1e-3):.0f} Hz)  "
                                 f"瞬断 {ev} 次  最大偏移 {worst:.1f} kPa   ")
                sys.stdout.flush()
    except KeyboardInterrupt:
        el = time.time() - t_start
        try:
            r.stop_continuous()
        except OSError:
            pass
        print(f"\n\n  —— 结果 ——")
        print(f"  通道 {num}({name})  采样 {n} 点 / {el:.0f}s  实际 {n/max(el,1e-3):.0f} Hz")
        if ev:
            alt = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
            alt_ratio = alt / max(len(signs) - 1, 1)
            if ev >= 6 and alt_ratio >= 0.7:
                print(f"  ⚠ 抓到 {ev} 次越界，但其中 {alt_ratio*100:.0f}% 是正负交替出现的，")
                print(f"     最大偏移仅 {worst:.1f} kPa —— **这是周期性干扰的特征，不是瞬断**。")
                print("     真瞬断是单向的（线断了分压点只会被拉向地）、随机的、接近满量程。")
                print("     交替 = 围绕基线上下摆的正弦波，被本工具按半周期切成了多次事件。")
                print(f"     下一步用 --scope {num} 看波形和频率定性，别去拆焊点：")
                print(f"         python3 scripts/p4_sensor_check.py --scope {num} --secs 3")
            else:
                print(f"  ❌ 抓到 {ev} 次瞬断，最大偏移 {worst:.1f} kPa")
                print("     这一路存在间歇性接触不良，必须修好再上墙——足压读数是吸附")
                print("     状态机判 ATTACHED 的依据，一次假读就可能让另一条腿提前抬起。")
                print("     断电后按刚才能触发的那一项定位：重新焊该焊点 / 重压该压接端子。")
        else:
            hz = min(n / max(el, 1e-3), 860.0)   # ADS1115 转换率封顶 860SPS
            print(f"  ✅ 本次没抓到超过 ±{thresh_kpa:.1f} kPa 的偏移")
            print(f"     能力边界：有效 {hz:.0f} 次转换/秒（轮询再快也只是重复读同一次结果），")
            print(f"     即约 {1000.0/hz:.1f} ms 一格。ADS 是 ΔΣ 型、按窗口积分，比这更短的瞬断")
            print("     会被平均成幅度变小的凹陷——仍看得见，但可能低于阈值。")
            print("     另外只有刚才真晃到的部位算验过，没晃到的不算。")


# ---------------- 波形/频谱模式 ----------------

def _pick(readers, chnum):
    entry = next((c for c in CHANNELS if c[0] == chnum), None)
    if entry is None:
        sys.exit(f"没有通道 {chnum}，可选 1~8。")
    if entry[3] not in readers:
        sys.exit(f"通道 {chnum} 挂在 0x{entry[3]:02X}，该模块没应答。")
    return entry


def _spectrum(ts, kpa, f_lo=5, f_hi=200):
    """按实际时间戳做 DFT（采样不均匀也能用），返回 [(幅度kPa, 频率Hz)] 降序。"""
    n = len(ts)
    step = max(1, n // 4000)          # 控制纯 Python 的计算量
    ts, kpa = ts[::step], kpa[::step]
    m = len(kpa)
    dc = sum(kpa) / m
    out = []
    for f in range(f_lo, f_hi + 1):
        re = im = 0.0
        w = 2 * math.pi * f
        for t, k in zip(ts, kpa):
            a = w * t
            re += (k - dc) * math.cos(a)
            im += (k - dc) * math.sin(a)
        out.append((2 * math.hypot(re, im) / m, f))
    out.sort(reverse=True)
    return out


def scope(bus, chnum, secs, csv_path):
    """抓一段原始波形，看它到底是"瞬断"还是"周期性干扰"。"""
    readers, _ = find_readers(bus)
    num, s, name, addr, ain, kind = _pick(readers, chnum)
    r = readers[addr]
    print(f"抓波形：通道 {num} {s} {name}   （0x{addr:02X} AIN{ain}）  {secs:.1f} 秒")
    print("  想复现干扰就在采集期间做那个动作（比如用手碰分压板）。\n")
    r.start_continuous(ain)
    time.sleep(0.05)

    ts, vs = [], []
    t0 = time.time()
    while time.time() - t0 < secs:
        ts.append(time.time() - t0)
        vs.append(r.read_last())
    try:
        r.stop_continuous()
    except OSError:
        pass

    srt = sorted(vs)
    v_med = srt[len(srt) // 2]
    kpa = [(v - v_med) * V_DIV * KPA_PER_V for v in vs]
    p2p = max(kpa) - min(kpa)
    rate = len(vs) / secs
    print(f"  采样 {len(vs)} 点 / {secs:.1f}s ({rate:.0f} Hz)   "
          f"基线 {v_med:.4f}V   峰峰 {p2p:.2f} kPa   "
          f"范围 {min(kpa):+.2f} ~ {max(kpa):+.2f} kPa")

    if csv_path:
        with open(csv_path, "w", encoding="utf-8") as f:
            f.write("t_s,v_adc,kpa_dev\n")
            for t, v, k in zip(ts, vs, kpa):
                f.write(f"{t:.6f},{v:.6f},{k:.4f}\n")
        print(f"  原始数据 → {csv_path}")

    peaks = _spectrum(ts, kpa)
    print("\n  频谱主峰（5~200Hz）：")
    shown, used = 0, []
    for amp, f in peaks:
        if any(abs(f - u) < 4 for u in used):
            continue
        used.append(f)
        print(f"    {f:3d} Hz   幅度 {amp:5.2f} kPa")
        shown += 1
        if shown >= 3:
            break

    top_amp, top_f = peaks[0]
    # 单一正弦：能量集中在主峰，2×幅度 ≈ 峰峰值（占比→100%）。
    # 孤立脉冲：频谱宽带铺开，任何单根谱线占比都很低。占比才是判据，不是绝对幅度。
    ratio = 200 * top_amp / max(p2p, 1e-6)
    print("\n  —— 判读 ——")
    print(f"  主峰 {top_f} Hz、幅度 {top_amp:.2f} kPa，占峰峰的 {ratio:.0f}%")
    if p2p < 0.5:
        print(f"  ✅ 峰峰仅 {p2p:.2f} kPa，就是本底噪声，波形干净。")
        print("     没复现出问题的话，是刚才那个动作没做到——干扰/瞬断都得现场复现才测得到。")
        return
    if ratio < 40:
        print(f"  ⚠ 主峰只占 {ratio:.0f}%，能量铺在整个频段 —— 不是周期信号，")
        print("     而是孤立脉冲的宽带频谱。")
        if p2p > 5.0:
            print(f"     峰峰 {p2p:.1f} kPa 的孤立大偏移，**这才像真瞬断**：")
            print(f"         python3 scripts/p4_sensor_check.py --wiggle {num}")
            print("     按七步定位到具体焊点/端子。")
        else:
            print(f"     但峰峰只有 {p2p:.2f} kPa，量级接近噪声，暂不用管。")
        return
    if top_amp < 0.3:
        print(f"  ✅ 周期成分幅度仅 {top_amp:.2f} kPa，远小于 -30kPa 判据，可以忽略。")
        return
    mains = 45 <= top_f <= 55 or 95 <= top_f <= 105
    if mains:
        print("  ❗ 这是工频（50Hz）或其二次谐波（100Hz）—— **电磁干扰，不是接触故障**。")
        print("     分压点源阻抗 10k∥10k=5k，配 100nF 的转折频率 318Hz，对 50Hz 几乎")
        print("     不衰减，所以手一碰就把市电感应耦合进来了。别去拆焊点。")
        print("     验证：改用充电宝/电池给 Pi 供电再测一次，主峰应显著变小或消失。")
    else:
        print(f"  ⚠ 周期性干扰但不在工频附近。查 {top_f}Hz 附近有什么在开关——")
        print("     泵/阀 PWM、舵机、开关电源。模拟线与功率线必须分梳走。")
    print("  无论哪种，周期性干扰都不该用拆焊点来治；真瞬断的特征是单向、随机、大幅。")


# ---------------- 大气点标定模式 ----------------

def zero(bus, samples, save):
    readers, _ = find_readers(bus)
    if not readers:
        sys.exit("没有 ADS1115 应答。")
    print(f"逐路标定大气点 V_ATM（采样 {samples} 次/路）。确认此刻所有传感器都通大气……")
    data = sample(readers, samples)
    table, rows = {}, []
    for num, s, name, addr, ain, kind in CHANNELS:
        if num not in data:
            continue
        mean, pp = data[num]
        tag, _ = classify(mean, kind)
        if tag in ("未接", "空位"):
            rows.append(f"  {num} {s:3s} {name:6s}  {tag}，跳过")
            continue
        if tag not in ("正常", "偏低"):
            rows.append(f"  {num} {s:3s} {name:6s}  {tag}（{mean:.3f}V），跳过——"
                        f"这一路先修好再标定")
            continue
        vatm = mean * V_DIV
        table[str(num)] = round(vatm, 4)
        rows.append(f"  {num} {s:3s} {name:6s}  V_ATM = {vatm:.4f} V   "
                    f"(偏离标称 {(vatm - V_ATM_NOMINAL)*1000:+.0f} mV = "
                    f"{(vatm - V_ATM_NOMINAL)*KPA_PER_V:+.2f} kPa)  p-p {pp*1000:.1f} mV")
    print("\n".join(rows))
    if not table:
        return
    print("\n粘进 Pi5VacuumIO 的形式：")
    print("    V_ATM_BY_CH = {" + ", ".join(f"{k}: {v}" for k, v in table.items()) + "}")
    if save:
        VATM_TABLE_PATH.parent.mkdir(parents=True, exist_ok=True)
        VATM_TABLE_PATH.write_text(
            json.dumps({"kpa_per_v": KPA_PER_V, "v_div": V_DIV, "v_atm_by_channel": table},
                       ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"\n已写入 {VATM_TABLE_PATH}")


def main():
    ap = argparse.ArgumentParser(description="P4 分压板 + 气压传感器自检")
    ap.add_argument("--live", action="store_true",
                    help="连续监视，吸气测试用（每路 ~0.15s 一次，抓不了毫秒级瞬断）")
    ap.add_argument("--wiggle", type=int, metavar="N",
                    help="锁定通道 N 高速采样抓瞬断，边晃线边看（怀疑接触不良时用）")
    ap.add_argument("--thresh", type=float, default=1.0, metavar="KPA",
                    help="--wiggle 的报警阈值 kPa（默认 1.0，静态噪声约 0.05）")
    ap.add_argument("--scope", type=int, metavar="N",
                    help="抓通道 N 的原始波形并测频率，分清干扰与真瞬断")
    ap.add_argument("--secs", type=float, default=3.0, help="--scope 采集秒数（默认 3）")
    ap.add_argument("--csv", metavar="PATH", help="--scope 把原始波形写成 CSV")
    ap.add_argument("--zero", action="store_true", help="逐路标定大气点 V_ATM")
    ap.add_argument("--save", action="store_true", help="--zero 时把表写进 docs/data/")
    ap.add_argument("--fixed-atm", action="store_true",
                    help="--live 时用固定 4.486V 基线，而不是启动自动取基线")
    ap.add_argument("--bus", type=int, default=1, help="I2C 总线号（默认 1）")
    ap.add_argument("--samples", type=int, default=16, help="每路采样次数（默认 16）")
    a = ap.parse_args()

    bus = open_bus(a.bus)
    try:
        if a.scope is not None:
            scope(bus, a.scope, a.secs, a.csv)
        elif a.wiggle is not None:
            wiggle(bus, a.wiggle, a.thresh)
        elif a.zero:
            zero(bus, max(a.samples, 32), a.save)
        elif a.live:
            live(bus, a.fixed_atm, a.samples)
        else:
            report(bus, a.samples)
    finally:
        bus.close()


if __name__ == "__main__":
    main()
