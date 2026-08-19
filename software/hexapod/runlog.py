"""黑匣子运行日志：联调脚本的落盘飞行记录。

动机（2026-08-18 事故）：无罐+深滞环首跑疑似把电池拖到低谷，5V 瞬间跌落，
Pi 掉电死机 + PMIC 故障锁存伪装成整板烧毁。事后既无文件日志、系统 journal
又在内存里，死机瞬间机器人在干什么（泵开没开、盘压多深、电压掉到多少）
完全无从查起。本模块保证下次有尸检材料。

为"死后验尸"服务的设计：
  - 一跑一文件 logs/<tag>_YYYYmmdd_HHMMSS.log；事件（EVT）与周期遥测（TLM）
    混排在同一时间轴，行首 [+秒] 是距开跑的相对时间，绝对时刻见文件头
  - 行缓冲写盘（进程崩溃不丢已写的行）+ 至多每 FSYNC_S 一次 fsync
    （死机/断电最多丢这么久的尾巴）
  - 写盘失败（SD 满/坏）绝不能炸控制环：stderr 告警一次后静默停写
  - close() 只写 END 标记 + fsync，不真关文件——异常清理路径（finally 与
    excepthook）先后顺序不定，后到的追记仍要写得进去；fd 由进程退出收掉

ClimbWatch 是 climb_walk 的取数器：逐控制周期 poll() 内存镜像找状态跳变，
不发起任何传感器 IO（不与控制环抢 Pi5VacuumIO 的 I2C 突发配额）；吸附
状态机跳变经 AdhesionController.on_event 钩子送达（带判据现场的盘压与
在态时长——轮询会晚一帧且拿不到判决值）。
"""
import os
import subprocess
import sys
import time
import traceback

from .adhesion import FootState
from .config import LEG_NAMES

FSYNC_S = 1.0   # 强制落盘周期：死机/断电最多丢这么久

# 状态行/遥测行共用字符表（climb_walk 状态行由此导入，勿两处漂移）
PHASE_CH = {"stance": "·", "vent": "V", "lift": "L", "transfer": "T",
            "hover": "H", "descend": "D", "press": "P", "retry": "R",
            "wait": "W"}
ADH_CH = {"released": "r", "pressing": "p", "sucking": "s",
          "attached": "A", "venting": "v", "fault": "F"}


class RunLog:
    def __init__(self, log_dir, tag="climb"):
        os.makedirs(log_dir, exist_ok=True)
        self.t0 = time.time()
        # 行首 [+秒] 用单调钟：树莓派无 RTC，chrony 联网后阶跃校时会把墙钟
        # 差出发的相对时间轴撕开一个跳变，验尸时序就乱了。绝对时刻只在
        # 文件头记一次（用墙钟，校时前可能偏，argv 行旁有 git 版本可交叉）
        self._t0m = time.monotonic()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.t0))
        self.path = os.path.join(log_dir, f"{tag}_{stamp}.log")
        # 追加模式：同秒重跑不覆盖旧内容
        self._f = open(self.path, "a", buffering=1, encoding="utf-8")
        self._dead = False        # 写盘失败熔断（绝不炸控制环）
        self._ended = False       # END 标记只写一次
        self._exc_logged = False  # traceback 只写一份（close 与 excepthook 都可能到）
        self._last_sync = 0.0
        self.note(f"开跑 {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(self.t0))}"
                  f"  argv: {' '.join(sys.argv)}")
        rev = self._git_rev()
        if rev:
            self.note(f"git: {rev}")

    @staticmethod
    def _git_rev():
        try:
            r = subprocess.run(["git", "describe", "--always", "--dirty"],
                               capture_output=True, text=True, timeout=2.0,
                               cwd=os.path.dirname(os.path.abspath(__file__)))
            return r.stdout.strip() or None
        except Exception:
            return None

    def _line(self, text):
        if self._dead:
            return
        try:
            self._f.write(text + "\n")
            now = time.monotonic()
            if now - self._last_sync >= FSYNC_S:
                self._last_sync = now
                os.fsync(self._f.fileno())
        except Exception as e:
            self._dead = True
            print(f"\n⚠ 黑匣子写盘失败，已停写（不影响行走）: {e}", file=sys.stderr)

    def note(self, text):
        """文件头注记（无时间戳）：模式、参数、阈值。"""
        self._line(f"# {text}")

    def event(self, text):
        self._line(f"[{time.monotonic() - self._t0m:9.3f}] EVT {text}")

    def row(self, text):
        self._line(f"[{time.monotonic() - self._t0m:9.3f}] TLM {text}")

    def exc(self, e=None):
        """记录异常 traceback（整个运行只记第一份，后到的略过）。"""
        if e is None:
            e = sys.exc_info()[1]
        if e is None or self._exc_logged:
            return
        self._exc_logged = True
        tb = "".join(traceback.format_exception(type(e), e, e.__traceback__))
        self.event("异常退出 ↓")
        for ln in tb.rstrip().splitlines():
            self._line("    " + ln)
        self.sync()

    def sync(self):
        if self._dead:
            return
        try:
            self._f.flush()
            os.fsync(self._f.fileno())
        except Exception:
            self._dead = True

    def close(self, reason=""):
        """收尾：在场异常先落 traceback，写一次 END，强制落盘（见模块头，
        故意不 close 文件）。"""
        e = sys.exc_info()[1]
        if isinstance(e, KeyboardInterrupt):
            self.event("Ctrl-C 中断")
        elif e is not None:
            self.exc(e)
        if not self._ended:
            self._ended = True
            self.event(f"END {reason}".rstrip())
        self.sync()


class ClimbWatch:
    """climb_walk 黑匣子取数器（用法见模块头）。"""

    def __init__(self, log, eng, ctl, io, cfg):
        self.log, self.eng, self.ctl, self.io, self.cfg = log, eng, ctl, io, cfg
        ctl.on_event = self._adh_event
        self._ph = dict(eng.phase_of)
        self._frozen = eng.frozen
        self._started = eng.started
        self._leak = list(ctl.leaking)
        self._tank_fault = ctl.tank_fault
        self._pump = bool(getattr(io, "pump", False))
        self._valve = list(getattr(io, "valve", [False] * ctl.n))
        self._giveups = getattr(eng, "air_giveups", 0)
        self._v_warn = self._c_warn = False

    # ---- 吸附状态机钩子（AdhesionController._set 内同步调用，必须轻且不抛）----
    def _adh_event(self, i, old, new, elapsed):
        kpa = self.ctl.last_kpa[i]
        k = f" {kpa:.1f}kPa" if kpa is not None else ""
        self.log.event(f"吸附 {LEG_NAMES[i]} {old.value}→{new.value} "
                       f"在态{elapsed:.2f}s{k}")

    def _worst_attached(self):
        vals = [self.ctl.last_kpa[i] for i in range(self.ctl.n)
                if self.ctl.state[i] == FootState.ATTACHED
                and self.ctl.last_kpa[i] is not None]
        return max(vals) if vals else None

    def poll(self):
        """每控制周期调用一次：内存比对找跳变，零传感器 IO。"""
        eng, ctl, io, log = self.eng, self.ctl, self.io, self.log
        for n in LEG_NAMES:
            ph = eng.phase_of[n]
            if ph != self._ph[n]:
                extra = ""
                if ph.value == "vent":
                    # 抬腿事件附本步落点相对默认位的指令偏移：验尸能分清
                    # "指令没往前"还是"指令对了腿没走到"（舵机/机械/被拽）
                    lx, ly = eng.landing[n]
                    x0, y0, _ = eng.default_feet[n]
                    extra = f" 落点Δ({lx - x0:+.1f},{ly - y0:+.1f})"
                log.event(f"相位 {n} {self._ph[n].value}→{ph.value}{extra}")
                self._ph[n] = ph
        for i, n in enumerate(LEG_NAMES):
            lk = ctl.leaking[i]
            if lk != self._leak[i]:
                self._leak[i] = lk
                kpa = ctl.last_kpa[i]
                k = f" 盘压{kpa:.1f}" if kpa is not None else ""
                log.event(f"漏气 {n} {'开始(保持抽气挽救)' if lk else '结束'}{k}")
        pump = bool(getattr(io, "pump", False))
        if pump != self._pump:
            self._pump = pump
            w = self._worst_attached()
            ctx = f" 最差盘压{w:.1f}" if w is not None else ""
            if not ctl.tankless and ctl.last_tank_kpa is not None:
                ctx += f" 罐压{ctl.last_tank_kpa:.1f}"
            log.event(f"泵 {'开' if pump else '停'}{ctx}")
        valve = list(getattr(io, "valve", []))
        for i, v in enumerate(valve):
            if i < len(self._valve) and v != self._valve[i]:
                log.event(f"阀 {LEG_NAMES[i]} {'→抽气位' if v else '→排气位'}")
        self._valve = valve
        if eng.frozen != self._frozen:
            log.event(f"冻结 ⚠ {eng.frozen}" if eng.frozen else "解冻")
            self._frozen = eng.frozen
        if eng.started and not self._started:
            self._started = True
            log.event("✓ 六足全吸附，启动序列完成，相位钟开始")
        if ctl.tank_fault != self._tank_fault:
            self._tank_fault = ctl.tank_fault
            log.event("罐压传感器失效（读数出合理区间）" if ctl.tank_fault
                      else "罐压传感器恢复")
        g = getattr(eng, "air_giveups", 0)
        if g != self._giveups:
            self._giveups = g
            log.event(f"架空模式放弃吸附，累计 {g}")

    def telemetry(self, v, c, peak, cmd, note=""):
        """随状态行节拍（0.5s）调用：一行全机快照 + 电源告警沿。"""
        eng, ctl, io = self.eng, self.ctl, self.io
        toks = []
        for i, n in enumerate(LEG_NAMES):
            st = ctl.state[i]
            kpa = ctl.last_kpa[i]
            # 只有状态机正在读压的状态才显示读数，其余显示 --（防拿陈旧值当真）
            live = st in (FootState.SUCKING, FootState.ATTACHED, FootState.VENTING)
            k = f"{kpa:.0f}" if live and kpa is not None else "--"
            toks.append(f"{n}{PHASE_CH[eng.phase_of[n].value]}"
                        f"{'!' if ctl.leaking[i] else ADH_CH[st.value]}@{k}")
        tank = ("无罐" if ctl.tankless else
                f"{ctl.last_tank_kpa:.1f}" if ctl.last_tank_kpa is not None
                else "--")
        valves = "".join("1" if x else "0"
                         for x in getattr(io, "valve", [])) or "?"
        vx, vy, wz = cmd
        self.log.row(
            f"t={eng.t:6.1f} {' '.join(toks)} 罐={tank} "
            f"泵={'1' if getattr(io, 'pump', False) else '0'} 阀={valves} "
            f"速={vx:+.0f},{vy:+.0f},{wz:+.2f} 电={v:.2f}V/{c:.2f}A/峰{peak:.2f} "
            f"补余={getattr(eng, '_comp_left', 0.0):.1f} "
            f"IO错={getattr(io, 'read_faults', 0)}{note}")
        # 电源告警沿（带回差防抖）：本次事故第一嫌因是电压跌落，趋势必须留痕
        if not self._v_warn and v < self.cfg.volt_warn:
            self._v_warn = True
            self.log.event(f"⚠ 电压告警 {v:.2f}V < {self.cfg.volt_warn}V"
                           f"（切断线 {self.cfg.volt_cutoff}V）")
        elif self._v_warn and v > self.cfg.volt_warn + 0.15:
            self._v_warn = False
            self.log.event(f"电压恢复 {v:.2f}V")
        if not self._c_warn and c > self.cfg.curr_warn:
            self._c_warn = True
            self.log.event(f"⚠ 电流告警 {c:.2f}A > {self.cfg.curr_warn}A")
        elif self._c_warn and c < self.cfg.curr_warn - 0.5:
            self._c_warn = False
            self.log.event(f"电流回落 {c:.2f}A")
