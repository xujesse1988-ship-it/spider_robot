"""Pi 5 电源黑匣子（hexapod.powerlog）测试：vcgencmd 输出解析、采样线程落盘与
告警沿、不可用降级、舵机合闸多点采样、启动步骤标记。全部无硬件。"""
import time

from hexapod.driver import MockDriver
from hexapod.powerlog import (PowerWatch, UV_NOW, UV_EVER, fmt_snapshot,
                              parse_pmic_adc, parse_throttled, servo_power_on,
                              startup_marker, throttled_text)
from hexapod.runlog import RunLog

SAMPLE_ADC = """ 3V7_WL_SW_A current(0)=0.01806000A
 3V3_SYS_A current(1)=0.11532000A
 VDD_CORE_A current(7)=2.03750000A
 VDD_CORE_V volt(16)=0.72062000V
 EXT5V_V volt(24)=5.08240000V
 0V8_AON_V volt(28)=0.79512000V
"""


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def test_parse_pmic_adc_and_throttled():
    d = parse_pmic_adc(SAMPLE_ADC)
    assert abs(d["EXT5V_V"] - 5.0824) < 1e-6
    assert abs(d["VDD_CORE_A"] - 2.0375) < 1e-6 and "0V8_AON_V" in d
    assert parse_pmic_adc("") == {} and parse_pmic_adc(None) == {}
    assert parse_throttled("throttled=0x50005") == 0x50005
    assert parse_throttled("50000\n") == 0x50000      # sysfs 无 0x 前缀
    assert parse_throttled("0x0") == 0
    assert parse_throttled("") is None and parse_throttled("垃圾") is None
    assert throttled_text(0x50005) == "thr=0x50005[欠压中,限频中,曾欠压,曾限频]"
    assert throttled_text(None) == "thr=?"
    s = fmt_snapshot({"ext5v": 5.0824, "core_v": 0.7206, "core_a": 2.04,
                      "throttled": 0, "uv_alarm": True})
    assert s == "5V=5.082V 核心=0.721V/2.04A thr=0x0 hwmon欠压告警"


class _Seq:
    """注入采样器：按序吐 dict，吐完后返回 None（计失败）。"""
    available = True
    reason = None

    def __init__(self, seq):
        self.seq = list(seq)

    def sample(self):
        return dict(self.seq.pop(0)) if self.seq else None


def _wait(pred, timeout=3.0):
    t_end = time.monotonic() + timeout
    while time.monotonic() < t_end:
        if pred():
            return True
        time.sleep(0.005)
    return False


def test_powerwatch_rows_edges_and_footer(tmp_path):
    log = RunLog(str(tmp_path), tag="t")
    seq = [{"ext5v": 5.05, "throttled": 0},
           {"ext5v": 4.70, "throttled": UV_NOW},               # 跌落 + 欠压中
           {"ext5v": 4.72, "throttled": UV_NOW | UV_EVER},     # 粘滞位新增
           {"ext5v": 5.02, "throttled": UV_EVER}]              # 恢复
    pw = PowerWatch(log, period_s=0.005, sampler=_Seq(seq)).start()
    assert pw.uv_ever_at_start is False
    assert _wait(lambda: pw.samples >= 4 and pw.fails >= 1)
    pw.stop()
    txt = read(log.path)
    assert "# Pi 电源监视：每 0.005s 一行" in txt and "起始 5V=5.050V" in txt
    assert "TLM 电源 5V=4.700V thr=0x1[欠压中]" in txt
    assert "⚠ Pi 欠压中" in txt and "Pi 欠压解除" in txt
    assert "⚠ Pi 5V 输入 4.700V 低于 4.8V" in txt and "Pi 5V 输入恢复 5.020V" in txt
    assert "新增粘滞标志：曾欠压" in txt
    assert "Pi 电源监视结束：采样 4 次" in txt and "5V 最低 4.700V" in txt
    assert txt.index("TLM 电源 5V=4.700V") < txt.index("⚠ Pi 欠压中")  # 先行后沿
    assert pw.text().startswith("  Pi[5V=5.020V thr=0x10000[曾欠压]")


def test_powerwatch_sticky_at_start_flags_and_no_dup_event(tmp_path):
    log = RunLog(str(tmp_path), tag="t")
    seq = [{"ext5v": 5.0, "throttled": UV_EVER}] * 3
    pw = PowerWatch(log, period_s=0.005, sampler=_Seq(seq)).start()
    assert pw.uv_ever_at_start is True
    assert _wait(lambda: pw.samples >= 3)
    pw.stop()
    txt = read(log.path)
    assert "本次开机以来已出现过欠压" in txt
    assert "新增粘滞标志" not in txt          # 起跑就有的粘滞位不算"新增"


def test_powerwatch_unavailable_only_notes(tmp_path):
    log = RunLog(str(tmp_path), tag="t")

    class NoPi:
        available = False
        reason = "无 vcgencmd（非树莓派？）"

        def sample(self):
            raise AssertionError("不可用时不该采样")
    pw = PowerWatch(log, sampler=NoPi()).start()
    pw.relax()
    pw.stop()                                  # 没线程也要安静
    assert pw.text() == ""
    txt = read(log.path)
    assert "Pi 电源监视：不可用（无 vcgencmd（非树莓派？））" in txt
    assert "TLM 电源" not in txt and "监视结束" not in txt


def test_startup_marker_and_servo_power_on_mock(tmp_path):
    log = RunLog(str(tmp_path), tag="t")
    pw = PowerWatch(log, sampler=_Seq([{"ext5v": 5.1, "throttled": 0}])).start()
    step = startup_marker(log, pw)
    step("阀 1/6 线圈通电前（GPIO5）")
    drv = MockDriver()
    t0 = time.monotonic()
    servo_power_on(drv, log, pw)
    assert drv.enabled and time.monotonic() - t0 < 0.5   # mock 不等 1s
    pw.stop()
    txt = read(log.path)
    assert "EVT 启动 阀 1/6 线圈通电前（GPIO5）  Pi[5V=5.100V thr=0x0 " in txt
    assert "舵机继电器合闸前 母线=7.40V 1.00A" in txt
    assert "舵机继电器已合闸（18 舵机同刻上电）" in txt
    assert "合闸后" not in txt


def test_servo_power_on_real_driver_samples_bus(tmp_path):
    log = RunLog(str(tmp_path), tag="t")

    class Drv:                                 # 无 is_mock：走真实采样路径
        def __init__(self):
            self.v = iter([7.9, 6.9, 7.3, 7.6])
            self.enabled = False

        def enable(self, on):
            self.enabled = on

        def read_voltage_v(self):
            return next(self.v)

        def read_current_a(self):
            return 2.5
    drv = Drv()
    t0 = time.monotonic()
    servo_power_on(drv, log, None, settle_s=0.03, samples=(0.0, 0.01, 0.5))
    el = time.monotonic() - t0
    assert drv.enabled and 0.03 <= el < 0.4   # 等满 settle_s；超 settle 的采样点丢弃
    txt = read(log.path)
    assert "合闸前 母线=7.90V 2.50A" in txt
    assert "合闸后 0.00s 母线=6.90V" in txt and "合闸后 0.01s 母线=7.30V" in txt
    assert "合闸后 0.50s" not in txt
