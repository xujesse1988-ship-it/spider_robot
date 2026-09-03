"""树莓派 5 电源侧黑匣子：5V 输入轨 / 欠压标志高频落盘 + 启动"大电流步骤"标记。

动机（2026-09-02）：body_lean / climb_walk 启动时六阀线圈逐一通电之后，概率性
整个树莓派死机——状态灯绿→红、SSH 失联。Pi 5 的红灯常亮是 PMIC 把 SoC 断电后
的停机态（与 sudo halt 之后一样，要按电源键/重新上电才回来），最像 5V 输入
瞬间跌破 PMIC 欠压门限；也不能排除感性负载开关瞬态把 SoC 打死。死机瞬间没有
任何进程能再写盘，所以只能"死前留痕"：

  - 每个大电流步骤（每路阀线圈通电、舵机继电器合闸、泵启动）之前先写一行并
    fsync（RunLog.mark）——日志停在哪一行，凶手就是它的下一步；
  - PowerWatch 后台线程按 period_s 读 PMIC 的 5V 输入电压（vcgencmd
    pmic_read_adc 的 EXT5V_V）与内核欠压标志（get_throttled），每行 fsync，
    死前最后一行就是最后的电压。PMIC ADC 只能 ~10Hz 级采样，几 ms 的深跌落
    抓不到波形，但趋势看得见，且"曾欠压"粘滞位（bit16）会把没死成的擦边
    也留下证据；
  - servo_power_on 在舵机继电器合闸后多点采样 Servo2040 读到的母线电压/电流：
    18 舵机同刻上电的冲击把电池拽到多少——5V 降压模块与舵机共电池
    （CLIMBING-DESIGN §5），母线塌陷是 5V 塌陷的上游。

非树莓派（无 vcgencmd）自动降级为只留步骤标记，开发机 --mock 不受影响。
死机后重新上电，跑 scripts/pi_forensics.sh check 看上次开机的内核线索 +
黑匣子尾巴（第一次先跑 setup 打开持久 journal，否则内核日志随死机蒸发）。
"""
import glob
import os
import re
import shutil
import subprocess
import threading
import time

# vcgencmd get_throttled 位定义（Pi 4/5 同）：低 4 位=此刻，16..19=本次开机以来
UV_NOW = 0x1
UV_EVER = 0x10000
STICKY_MASK = 0xF0000
THROTTLE_BITS = {0: "欠压中", 1: "ARM频率封顶", 2: "限频中", 3: "软温限中",
                 16: "曾欠压", 17: "曾频率封顶", 18: "曾限频", 19: "曾软温限"}
# 5V 输入告警线：PMIC 欠压门限之上留余量的经验线（标称 5.0V，降压模块正常
# 应在 5.0~5.15；跌到 4.8 以下说明降压输入/输出已经吃紧）
EXT5V_WARN_V = 4.80
# 舵机合闸后母线采样时刻（相对合闸，秒）；末点 = 原脚本合闸后 1.0s 等待
SERVO_ON_SAMPLE_S = (0.05, 0.15, 0.3, 0.6, 1.0)

_ADC_RE = re.compile(r"^\s*(\S+)\s+(?:volt|current)\(\d+\)=([0-9.]+)\s*[VA]",
                     re.M)
# 内核 firmware 节点的 get_throttled（读 sysfs 免 fork）；Pi 5 的 soc 节点名带
# 地址，用通配。没有就退回 vcgencmd get_throttled
_THROTTLED_GLOB = "/sys/devices/platform/soc*/*:firmware/get_throttled"


def parse_pmic_adc(text):
    """`vcgencmd pmic_read_adc` 输出 -> {通道名: 值}，如 {"EXT5V_V": 5.0824}。"""
    return {m.group(1): float(m.group(2)) for m in _ADC_RE.finditer(text or "")}


def parse_throttled(text):
    """'throttled=0x50000' / '50000' / '0x0' -> int；解析不了返回 None。"""
    s = (text or "").strip().split("=")[-1].strip()
    if not s:
        return None
    try:
        return int(s, 16)
    except ValueError:
        return None


def throttled_text(flags):
    if flags is None:
        return "thr=?"
    names = [n for b, n in THROTTLE_BITS.items() if (flags >> b) & 1]
    return f"thr=0x{flags:x}" + (f"[{','.join(names)}]" if names else "")


def fmt_snapshot(s):
    """一次采样 -> 日志片段：5V=5.082V 核心=0.721V/2.04A thr=0x0。"""
    parts = []
    if s.get("ext5v") is not None:
        parts.append(f"5V={s['ext5v']:.3f}V")
    if s.get("core_v") is not None:
        core = f"核心={s['core_v']:.3f}V"
        if s.get("core_a") is not None:
            core += f"/{s['core_a']:.2f}A"
        parts.append(core)
    parts.append(throttled_text(s.get("throttled")))
    if s.get("uv_alarm"):
        parts.append("hwmon欠压告警")
    return " ".join(parts)


class PiPowerSampler:
    """读一次 Pi 5 电源健康：PMIC ADC（5V 输入/核心电压电流）+ 欠压标志 +
    hwmon rpi_volt 告警位。available=False（非树莓派/无 vcgencmd）时 sample()
    恒返回 None，reason 说明原因。"""

    def __init__(self):
        self.vcgencmd = shutil.which("vcgencmd")
        self.reason = None
        self.has_adc = False
        thr = glob.glob(_THROTTLED_GLOB)
        self.throttled_path = thr[0] if thr else None
        self.alarm_path = self._find_rpi_volt_alarm()
        self.available = self.vcgencmd is not None
        if not self.available:
            self.reason = "无 vcgencmd（非树莓派？）"
            return
        probe = self._run(["pmic_read_adc"])
        self.has_adc = bool(probe) and "EXT5V_V" in probe
        if not self.has_adc:
            # Pi 4 没有 PMIC ADC；或用户不在 video 组。欠压标志仍可采
            self.reason = "pmic_read_adc 不可用（非 Pi 5 / 用户不在 video 组？）"

    @staticmethod
    def _find_rpi_volt_alarm():
        for name in glob.glob("/sys/class/hwmon/hwmon*/name"):
            try:
                with open(name) as f:
                    if f.read().strip() != "rpi_volt":
                        continue
            except OSError:
                continue
            p = name[:-len("name")] + "in0_lcrit_alarm"
            if os.path.exists(p):
                return p
        return None

    def _run(self, args, timeout=0.5):
        try:
            r = subprocess.run([self.vcgencmd, *args], capture_output=True,
                               text=True, timeout=timeout)
        except Exception:
            return None
        return r.stdout if r.returncode == 0 else None

    def _read_file(self, path):
        try:
            with open(path) as f:
                return f.read()
        except OSError:
            return None

    def sample(self):
        if not self.available:
            return None
        snap = {"t": time.monotonic(), "ext5v": None, "core_v": None,
                "core_a": None, "throttled": None, "uv_alarm": None}
        if self.has_adc:
            adc = parse_pmic_adc(self._run(["pmic_read_adc"]))
            snap["ext5v"] = adc.get("EXT5V_V")
            snap["core_v"] = adc.get("VDD_CORE_V")
            snap["core_a"] = adc.get("VDD_CORE_A")
        thr = None
        if self.throttled_path:
            thr = parse_throttled(self._read_file(self.throttled_path))
        if thr is None:
            thr = parse_throttled(self._run(["get_throttled"]))
        snap["throttled"] = thr
        if self.alarm_path:
            a = self._read_file(self.alarm_path)
            snap["uv_alarm"] = (a.strip() == "1") if a is not None else None
        if snap["ext5v"] is None and thr is None:
            return None     # 两路都没读到：本次采样作废（计失败）
        return snap


class PowerWatch:
    """后台采样线程：每 period_s 写一行 `TLM 电源 …` 并 fsync；欠压/低压/粘滞
    位变化写 EVT 沿。text() 给步骤标记行拼最近一次采样。

    sampler：默认 PiPowerSampler()；测试可注入带 sample()/available/reason 的
    对象或直接一个返回 dict|None 的 callable。"""

    def __init__(self, log, period_s=0.1, sampler=None, warn_v=EXT5V_WARN_V):
        self.log = log
        self.period_s = period_s
        self.warn_v = warn_v
        src = PiPowerSampler() if sampler is None else sampler
        self._sample = src.sample if hasattr(src, "sample") else src
        self._available = getattr(src, "available", True)
        self._reason = getattr(src, "reason", None)
        self.last = None
        self.samples = 0
        self.fails = 0
        self.min_5v = None
        self.uv_ever_at_start = False   # 起跑时粘滞位已置：本次开机早就欠压过
        self._stop = threading.Event()
        self._thr = None
        self._uv_now = False
        self._low = False
        self._alarm = False
        self._sticky = 0

    # ---- 生命周期 ----
    def start(self):
        if not self._available:
            self.log.note(f"Pi 电源监视：不可用（{self._reason}），只留步骤标记")
            return self
        snap = self._take()
        if snap is None:
            self.log.note("Pi 电源监视：首采失败，只留步骤标记")
            return self
        thr = snap.get("throttled") or 0
        self._sticky = thr & STICKY_MASK
        self._uv_now = bool(thr & UV_NOW)
        self.uv_ever_at_start = bool(thr & UV_EVER)
        warn = ("  ⚠ 本次开机以来已出现过欠压（粘滞位）——先查供电"
                if self.uv_ever_at_start else "")
        why = f"（{self._reason}）" if self._reason else ""
        self.log.note(f"Pi 电源监视{why}：每 {self.period_s:g}s 一行 TLM 电源，"
                      f"每行落盘；起始 {fmt_snapshot(snap)}{warn}")
        self._thr = threading.Thread(target=self._run, name="powerwatch",
                                     daemon=True)
        self._thr.start()
        return self

    def relax(self, period_s=0.5):
        """启动大电流阶段过了就放慢（少 fork、少 fsync）。"""
        self.period_s = period_s

    def stop(self):
        if self._thr is None:
            return
        self._stop.set()
        self._thr.join(timeout=2.0)
        self._thr = None
        lo = f" 5V 最低 {self.min_5v:.3f}V" if self.min_5v is not None else ""
        self.log.event(f"Pi 电源监视结束：采样 {self.samples} 次 失败 {self.fails}"
                       f"{lo} {throttled_text((self.last or {}).get('throttled'))}")
        self.log.sync()

    # ---- 采样 ----
    def _take(self):
        try:
            snap = self._sample()
        except Exception:
            snap = None
        if snap is None:
            self.fails += 1
            return None
        snap.setdefault("t", time.monotonic())
        self.samples += 1
        self.last = snap
        v = snap.get("ext5v")
        if v is not None and (self.min_5v is None or v < self.min_5v):
            self.min_5v = v
        return snap

    def _run(self):
        while not self._stop.wait(self.period_s):
            snap = self._take()
            if snap is None:
                continue
            self.log.row("电源 " + fmt_snapshot(snap))
            self._edges(snap)
            self.log.sync()

    def _edges(self, s):
        thr = s.get("throttled")
        if thr is not None:
            now = bool(thr & UV_NOW)
            if now != self._uv_now:
                self._uv_now = now
                self.log.event("⚠ Pi 欠压中（get_throttled bit0）"
                               if now else "Pi 欠压解除")
            new = thr & STICKY_MASK & ~self._sticky
            if new:
                self._sticky |= new
                names = [n for b, n in THROTTLE_BITS.items()
                         if b >= 16 and (new >> b) & 1]
                self.log.event(f"⚠ Pi 本次开机新增粘滞标志：{','.join(names)}"
                               f"（{throttled_text(thr)}）")
        v = s.get("ext5v")
        if v is not None:
            if not self._low and v < self.warn_v:
                self._low = True
                self.log.event(f"⚠ Pi 5V 输入 {v:.3f}V 低于 {self.warn_v:g}V")
            elif self._low and v > self.warn_v + 0.05:
                self._low = False
                self.log.event(f"Pi 5V 输入恢复 {v:.3f}V")
        a = s.get("uv_alarm")
        if a is not None and a != self._alarm:
            self._alarm = a
            self.log.event("⚠ hwmon rpi_volt 欠压告警置位" if a
                           else "hwmon rpi_volt 欠压告警清除")

    def text(self):
        """步骤标记行的后缀：最近一次采样 + 多久前（没采样返回空串）。"""
        if self.last is None:
            return ""
        age = (time.monotonic() - self.last["t"]) * 1000.0
        return f"  Pi[{fmt_snapshot(self.last)} {age:.0f}ms前]"


def startup_marker(log, pwr=None, prefix="启动"):
    """给 Pi5VacuumIO(on_step=…) 与脚本启动序列用的步骤标记：一步一行、当场
    fsync、带最近的 Pi 电源采样——死机后日志停在哪行，下一步就是扳机。"""
    def mark(text):
        log.mark(f"{prefix} {text}{pwr.text() if pwr is not None else ''}")
    return mark


def _bus_text(drv):
    try:
        return f"母线={drv.read_voltage_v():.2f}V {drv.read_current_a():.2f}A"
    except Exception as e:
        return f"母线=读失败({e.__class__.__name__})"


def servo_power_on(drv, log, pwr=None, settle_s=1.0, samples=SERVO_ON_SAMPLE_S):
    """合舵机继电器（drv.enable(True)）并把合闸前后的母线电压/电流落盘。

    合闸=18 舵机同刻上电（BOM：瞬态 ~12A），是启动序列里最大的一次电流阶跃，
    紧跟在六阀线圈通电之后不到 0.1s——用户看到的"阀逐一亮起之后死机"落在
    这两件事之间，靠本函数的标记行分辨。合闸后按 samples 各时刻读一次母线，
    整体等满 settle_s（原脚本合闸后 sleep 1.0s 的口径不变）；Mock 驱动不等。"""
    sfx = (lambda: pwr.text()) if pwr is not None else (lambda: "")
    log.mark(f"舵机继电器合闸前 {_bus_text(drv)}{sfx()}")
    drv.enable(True)
    log.mark(f"舵机继电器已合闸（18 舵机同刻上电）{sfx()}")
    if getattr(drv, "is_mock", False):
        return
    t0 = time.monotonic()
    for t in samples:
        if t > settle_s:
            break
        rem = t0 + t - time.monotonic()
        if rem > 0:
            time.sleep(rem)
        log.mark(f"合闸后 {t:.2f}s {_bus_text(drv)}{sfx()}")
    rem = t0 + settle_s - time.monotonic()
    if rem > 0:
        time.sleep(rem)
