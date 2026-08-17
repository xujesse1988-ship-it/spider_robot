"""P4 爬墙步态引擎：相位推进可被吸附事件暂停的事件驱动步态。

P3 的 GaitEngine 是纯开环相位表——假定触地精确发生在相位边界、触地瞬间
垂直速度最大（"拍地"）、一过边界立刻横向拖脚，对"落脚即密封"全是反的
（P4-GUIDE 第 0 步）。本引擎把摆动相拆成分段状态机（§4.1）：

  LIFT     请求放气并沿面法向退开 lift_clearance（吸盘回弹 11~13mm 会把脚
           顶回面上，必须边放气边走）；放气未确认（RELEASED）不进 TRANSFER
  TRANSFER 平移到落点上方：沿用 smoothstep + 正弦抬腿形状
  DESCEND  落点正上方竖直减速下探：XY 冻结，descend_speed 慢速——消灭拍地
  PRESS    XY 继续冻结，压入该腿 press_delta_mm（§4.2 预压行程），
           到位即 request_attach
  WAIT     等 ATTACHED（-30kPa 确认窗）；FAULT -> 抬 retry_lift_mm 重压，
           连续 max_attach_retry 次失败 -> 全机冻结报警
  （支撑）  ATTACHED 后目标保持在压入位，随支撑速度场整体反向平移——
           吸住的脚不能滑，支撑目标必须连续积分，速度突变不产生跳变

坐标约定：一切在身体系，z0 = -stand_height 是吸附面，面法向 = ±z。
身体平行于面时对地面/斜面/竖墙通用（贴墙姿态由 Hexapod.body_rpy 另调）。

相位钟规则（"事件驱动"的实现）：全局钟 t 正常推进，但
  - 摆动腿在其相位窗结束时还没吸牢 -> 钟停在窗尾等它（支撑腿随之冻结）
  - 抬腿前互锁不满足（其余 5 足没全 ATTACHED，比 adhesion 注释的 >=4 严，
    首爬用严的）-> 钟停在窗头；相邻足不同时释放由一次一腿天然保证
  - 任一支撑足漏气 -> 钟暂停（挽救窗内），超 leak_rescue_s -> 冻结报警
  - 速度指令为零 -> 该窗不抬腿，钟空转过窗（吸住不动，省吸盘寿命）

冻结（frozen）是粘滞报警态：足端目标全部保持、吸附状态机照跑（常闭阀保
真空，吸附态冻结是安全态），人工处理后 clear_freeze() 继续。

启动序列：调用方先让机器人 stand() 到默认站姿，引擎首个阶段把六足同时
压入、按腿序逐足抽气确认（六阀同开会把罐抽垮），全部 ATTACHED 后相位钟
才开始走（started=True）。
"""
import math
from dataclasses import replace
from enum import Enum

from .adhesion import FootState
from .config import RobotConfig, LEG_NAMES
from .gait import Gait, GaitEngine, CLIMB, _smoothstep
from .kinematics import leg_ik

_EPS = 1e-6
TANK_READY_KPA = -40.0      # 启动序列首次抽气前罐压须建立到此值（冷罐上电
                            # 直接抽必然重试穷尽误冻结；-40 给 -30 判据留余量）
TANKLESS_PRECHARGE_S = 3.0  # 无罐模式首次抽气前的盲抽时长：阀全关、歧管容积
                            # 小，抽几秒即空——省得首足 SUCK 独扛整段大气歧管
PRECHARGE_TIMEOUT_S = 30.0  # 罐压建立超时 -> 冻结报警（泵坏/大漏不能静默干等）
VENT_STALL_S = 2.0          # LIFT 抬到位后等放气确认（RELEASED）的额外上限，
                            # 超时冻结报警——排气堵/传感器漂移不能静默停摆
TILT_BAND_DEG = 12.0        # 落点带：压入位物理吸盘轴偏面法线的许用角。
                            # P1 台架实测容差 ±15°，留余量；CLIMBING-DESIGN §6
                            # 接受的工作带 ≤11.5°。越界步长按半径裁剪（§4.3）
_R_BRACKET = (110.0, 190.0)  # 站位半径求解区间 mm（区间内倾角随半径单调增）


def _press_tilt(cfg, r, z_press):
    """(径向 r, 压入深度 z) 姿态下，物理吸盘轴偏离面法线的带符号角（rad）。
    吸盘轴 = a_t + cup_delta（勿拿 a_t 当吸盘轴，LEG-GEOMETRY §2.13 教训）；
    倾角只依赖 (r, z)——coxa 偏摆整体旋转腿平面，不改轴线离垂直的角度。"""
    _, a, th = leg_ik(cfg, r, 0.0, z_press)
    a_t = a + th - math.pi
    return a_t + math.radians(cfg.cup_delta_deg) + math.pi / 2


def _solve_reach(cfg, z_press, tilt_rad=0.0):
    """解站位半径：压入位吸盘轴偏法线 = tilt_rad（0 = 严格垂直）。"""
    lo, hi = _R_BRACKET
    if _press_tilt(cfg, lo, z_press) >= tilt_rad:
        return lo
    if _press_tilt(cfg, hi, z_press) <= tilt_rad:
        return hi
    for _ in range(48):
        mid = (lo + hi) / 2.0
        if _press_tilt(cfg, mid, z_press) < tilt_rad:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


class LegPhase(Enum):
    STANCE = "stance"        # 支撑（含启动前的等待）：目标在压入位，随速度场平移
    LIFT = "lift"
    TRANSFER = "transfer"
    DESCEND = "descend"
    PRESS = "press"
    RETRY_LIFT = "retry"
    WAIT = "wait"


class ClimbEngine:
    """吸附联动爬墙步态。每控制周期调用 update(dt, vx, vy, wz) 取足端目标。

    update() 内部会替调用方跑 AdhesionController.update(dt)，外层循环
    只需要 move_feet(engine.update(...))。
    """

    def __init__(self, cfg: RobotConfig, ctl, gait: Gait = CLIMB,
                 ignore_tank_fault: bool = False, air_mode: bool = False):
        self.cfg = cfg
        self.ctl = ctl
        self.gait = gait
        # air_mode = 实机架空联调（P4-GUIDE 4.6.3）：吸盘悬空必然 FAULT，
        # 重试穷尽后不冻结而是当作已吸附继续走、互锁放行——阀/泵/重试路径
        # 全部真跑，只旁路"吸不住就停"。上墙严禁开。
        self.air_mode = air_mode
        self.air_giveups = 0         # 架空模式放弃吸附的次数（联调统计用）
        # 罐压传感器未接时的专用旁路，上墙严禁开
        self.ignore_tank_fault = ignore_tank_fault or air_mode
        self.z0 = -cfg.stand_height
        # 步幅/相位几何复用 GaitEngine（cycle_time/max_step 换成爬墙参数），
        # 避免公式漂移
        self._geom = GaitEngine(replace(cfg, cycle_time=cfg.climb_cycle_time,
                                        max_step=cfg.climb_max_step), gait)
        # 爬墙站位 ≠ 地面站位（foot_reach=130 时吸盘轴偏法线 ~19°，超 ±15°
        # 容差，唇口斜着接面吸不住）：逐腿解"压入位吸盘轴 ⊥ 吸附面"的半径
        # （默认参数约 176mm），落点带按 ±TILT_BAND_DEG 换算成半径区间裁剪。
        self.default_feet = {}
        self._r_band = {}
        for leg in cfg.legs:
            z_press = self.z0 - leg.press_delta_mm
            band = math.radians(TILT_BAND_DEG)
            r0 = _solve_reach(cfg, z_press)
            self._r_band[leg.name] = (_solve_reach(cfg, z_press, -band),
                                      _solve_reach(cfg, z_press, band))
            a = math.radians(leg.mount_angle_deg)
            self.default_feet[leg.name] = (leg.mount_x + r0 * math.cos(a),
                                           leg.mount_y + r0 * math.sin(a),
                                           self.z0)
        self._geom.default_feet = dict(self.default_feet)  # 速度场用同一站位
        # 当前指令足端目标（唯一事实源，机内所有分段都直接改它）
        self.foot = {n: list(p) for n, p in self.default_feet.items()}
        self.phase_of = {n: LegPhase.PRESS for n in LEG_NAMES}  # 启动即全压入
        self.retries = {n: 0 for n in LEG_NAMES}
        self.landing = {n: self.default_feet[n][:2] for n in LEG_NAMES}
        self._transfer = {}          # 腿名 -> (进度 s, 起点 xyz)
        self.frozen = None           # 冻结原因；None = 正常
        self.started = False         # 启动全吸附完成，相位钟开始走
        self.t = 0.0                 # 相位钟
        self._attach_queue = list(LEG_NAMES)
        self._slot_leg = None        # 当前相位窗属于哪条腿
        self._slot_active = False    # 本窗摆动已启动
        self._slot_skipped = False   # 本窗静止跳过
        self._block_t = 0.0          # 互锁不满足已等待时长
        self._precharge_t = 0.0      # 启动序列等罐压建立已耗时长
        self._tankless_precharged = False   # 无罐盲抽已完成（一次性）
        self._seg_t = {n: 0.0 for n in LEG_NAMES}   # 当前摆动分段停留时长
        self._last_ph = dict(self.phase_of)

    # ---------- 对外 ----------
    def update(self, dt, vx=0.0, vy=0.0, wz=0.0):
        """推进一个控制周期，返回 {腿名: (x,y,z)} 身体系足端目标。"""
        self.ctl.update(dt)
        if (getattr(self.ctl, "tank_fault", False)
                and not self.ignore_tank_fault and not self.frozen):
            self.frozen = "罐压传感器读数出合理区间（未接/失效），泵已停"
        if self.frozen:
            return self.targets()          # 冻结：目标保持，只有吸附机在跑
        leak_pause = self._leak_watch()    # 支撑足漏气：挽救窗内暂停，超时冻结
        if self.frozen:
            return self.targets()
        if not self.started:
            if not self._tank_ready():   # 冷罐上电：先建罐压再抽第一只脚
                # 计"连续"不就绪时长：抽气尝试会周期性放掉罐压再等泵恢复，
                # 累计口径会把正常的反复等待攒成假超时（--air 架空必踩）
                self._precharge_t += dt
                if self._precharge_t > PRECHARGE_TIMEOUT_S:
                    self.frozen = (f"罐压连续 {PRECHARGE_TIMEOUT_S:.0f}s 未建立到 "
                                   f"{TANK_READY_KPA:.0f}kPa（泵/气路异常）")
                    return self.targets()
            else:
                self._precharge_t = 0.0
            self._run_machines(dt)
            if not self._attach_queue and all(
                    p == LegPhase.STANCE for p in self.phase_of.values()):
                self.started = True
            return self.targets()

        vx, vy, wz = self._clamp_speed(vx, vy, wz)
        moving = abs(vx) > 1e-6 or abs(vy) > 1e-6 or abs(wz) > 1e-6

        # 1. 相位窗归属（窗切换时复位窗内一次性标志）
        cur = self._slot(self.t)
        if cur != self._slot_leg:
            self._slot_leg = cur
            self._slot_active = self._slot_skipped = False
            self._block_t = 0.0

        # 2. 窗头决策：跳过 / 启动摆动 / 互锁等待。
        #    漏气挽救期间（leak_pause）绝不放行新的抬腿——漏着的脚不算可靠支撑
        if not self._slot_active and not self._slot_skipped:
            if not moving:
                self._slot_skipped = True
            elif not leak_pause and self._interlock_ok(cur):
                self.landing[cur] = self._landing_xy(cur, vx, vy, wz)
                self.ctl.request_release(LEG_NAMES.index(cur))
                self.phase_of[cur] = LegPhase.LIFT
                self._slot_active = True
            else:
                self._block_t += dt
                if self._block_t > self.cfg.interlock_timeout_s:
                    bad = [n for n in LEG_NAMES if n != cur and
                           (not self.ctl.is_attached(LEG_NAMES.index(n))
                            or self.ctl.is_leaking(LEG_NAMES.index(n)))]
                    self.frozen = (f"互锁失败：{'/'.join(bad) or '?'} 不可靠，"
                                   f"{cur} 拒抬")

        # 3. 相位钟推进量：摆动落后于相位窗时钟停在窗尾等吸附事件
        if leak_pause:
            adv = 0.0
        elif self._slot_skipped:
            adv = dt                                   # 静止：空转过窗
        elif not self._slot_active:
            adv = 0.0                                  # 互锁等待：停在窗头
        elif self.phase_of[cur] == LegPhase.STANCE:
            adv = dt                                   # 本窗摆动已完成
        else:
            p = self._phase(cur, self.t)
            adv = max(0.0, min(dt, (1.0 - p) * self.cfg.climb_cycle_time - _EPS))
        self.t += adv

        # 4. 支撑足随速度场反向平移（只随钟走：钟停 = 全体支撑冻结）
        if adv > 0.0 and moving:
            dyaw = wz * adv
            c, s = math.cos(dyaw), math.sin(dyaw)
            for n in LEG_NAMES:
                if self.phase_of[n] == LegPhase.STANCE:
                    f = self.foot[n]
                    x, y = f[0] - vx * adv, f[1] - vy * adv
                    f[0], f[1] = c * x + s * y, -s * x + c * y
                    f[2] = self.z0 - self.cfg.leg(n).press_delta_mm

        # 5. 摆动分段状态机按真实时间推进（钟停时重试/等待照常进行）
        self._run_machines(dt)
        return self.targets()

    def targets(self):
        return {n: tuple(f) for n, f in self.foot.items()}

    def status(self):
        """{腿名: (步态段, 吸附态, 是否漏气)}，联调脚本显示用。"""
        return {n: (self.phase_of[n].value,
                    self.ctl.state[LEG_NAMES.index(n)].value,
                    self.ctl.is_leaking(LEG_NAMES.index(n)))
                for n in LEG_NAMES}

    def clear_freeze(self):
        """人工处理后解除冻结；重试/等待计时全部清零，挂着 FAULT 的腿自动
        重新压附（罐压计时不清会导致解冻后一帧不就绪立刻复冻）。"""
        self.frozen = None
        self._block_t = 0.0
        self._precharge_t = 0.0
        for n in LEG_NAMES:
            self.retries[n] = 0

    # ---------- 内部 ----------
    def _phase(self, name, t):
        return self._geom.phase(name, t)   # _geom 的 cycle_time 已换成爬墙周期

    def _tank_ready(self):
        """罐压已建立（或传感器失效被旁路时视为就绪，air 模式用）。
        无罐模式改为定时盲抽：读不到歧管压力（足压传感器在阀的吸盘侧、
        阀关时看不见歧管），先请求泵抽 TANKLESS_PRECHARGE_S 再放行首足。"""
        if getattr(self.ctl, "tankless", False):
            if not self._tankless_precharged:
                if self._precharge_t < TANKLESS_PRECHARGE_S:
                    self.ctl.precharge = True
                    return False
                self._tankless_precharged = True
                self.ctl.precharge = False
            return True
        return self.ctl.tank_fault or \
            self.ctl.io.read_tank_kpa() <= TANK_READY_KPA

    def _slot(self, t):
        """当前相位窗属于哪条腿。5/6 占空 + 等距相位恰好铺满一圈；浮点边界
        可能瞬时出现两腿都 >= duty，取相位最深（即将收尾）的那条。"""
        cands = [(self._phase(n, t), n) for n in LEG_NAMES
                 if self._phase(n, t) >= self.gait.duty]
        return max(cands)[1] if cands else self._slot_leg

    def _clamp_speed(self, vx, vy, wz):
        """把速度指令整体缩放到爬墙步幅上限内。地面步态只截落点、超速部分靠
        支撑足打滑消化；爬墙吸住的脚不能滑，**支撑相真实位移 = 速度×支撑
        时长**必须 ≤ climb_max_step，否则支撑足会被拖出工作空间/落点带。"""
        T_st = self.cfg.climb_cycle_time * self.gait.duty
        worst = 0.0
        for n in LEG_NAMES:
            x0, y0, _ = self.default_feet[n]
            worst = max(worst, math.hypot(vx - wz * y0, vy + wz * x0))
        worst *= T_st
        if worst <= self.cfg.climb_max_step:
            return vx, vy, wz
        s = self.cfg.climb_max_step / worst
        return vx * s, vy * s, wz * s

    def _interlock_ok(self, name):
        """抬 name 前其余 5 足必须全部 ATTACHED 且没在漏气。"""
        if self.air_mode:
            return True
        return all(self.ctl.is_attached(LEG_NAMES.index(n))
                   and not self.ctl.is_leaking(LEG_NAMES.index(n))
                   for n in LEG_NAMES if n != name)

    def _leak_watch(self):
        """支撑足漏气监护（启动序列与行走共用）：挽救窗内返回 True
        （相位钟应暂停），超 leak_rescue_s 置 frozen。"""
        pause = False
        for n in LEG_NAMES:
            i = LEG_NAMES.index(n)
            if self.phase_of[n] == LegPhase.STANCE and self.ctl.is_leaking(i):
                pause = True
                if self.ctl.leak_time(i) > self.cfg.leak_rescue_s:
                    # 每足支路已装单向阀（歧管—阀罐口间，只许吸盘→歧管），
                    # 盘压互相独立：报谁就是谁，不再有落脚连坐顶包
                    self.frozen = (f"{n} 漏气挽救超 "
                                   f"{self.cfg.leak_rescue_s}s"
                                   f"（查 {n} 吸盘唇口/支路密封）")
        return pause

    def _landing_xy(self, name, vx, vy, wz):
        """本步落点（身体系）：默认位 + 半步幅（速度场与限幅复用 _geom），
        再按落点带做径向裁剪——半径出带 = 压入位唇口倾角超容差，宁可缩步。"""
        ux, uy = self._geom._stride(name, vx, vy, wz)
        x0, y0, _ = self.default_feet[name]
        lx, ly = x0 + ux / 2.0, y0 + uy / 2.0
        leg = self.cfg.leg(name)
        dx, dy = lx - leg.mount_x, ly - leg.mount_y
        r = math.hypot(dx, dy)
        r_lo, r_hi = self._r_band[name]
        r_c = min(max(r, r_lo), r_hi)
        if abs(r_c - r) > _EPS:
            dx, dy = dx * r_c / r, dy * r_c / r
        return (leg.mount_x + dx, leg.mount_y + dy)

    def _may_attach(self, name):
        """启动序列里只放行队首（逐足抽气，防六阀同开抽垮罐），
        且罐压必须已建立（冷罐直接抽会重试穷尽误冻结）。"""
        if self.started:
            return True
        return bool(self._attach_queue) and self._attach_queue[0] == name \
            and self._tank_ready()

    def _run_machines(self, dt):
        for n in LEG_NAMES:
            # 分段停留计时（放气确认超时等 stall 监护用）
            if self.phase_of[n] != self._last_ph[n]:
                self._last_ph[n] = self.phase_of[n]
                self._seg_t[n] = 0.0
            else:
                self._seg_t[n] += dt
            if self.phase_of[n] != LegPhase.STANCE:
                self._step_swing(n, dt)

    def _step_swing(self, name, dt):
        cfg = self.cfg
        i = LEG_NAMES.index(name)
        f = self.foot[name]
        ph = self.phase_of[name]
        z_press = self.z0 - cfg.leg(name).press_delta_mm
        z_lift = self.z0 + cfg.lift_clearance

        if ph == LegPhase.LIFT:
            f[2] = min(z_lift, f[2] + cfg.lift_speed * dt)
            if f[2] >= z_lift - _EPS:
                if self.ctl.state[i] == FootState.RELEASED:
                    self._transfer[name] = (0.0, tuple(f))
                    self.phase_of[name] = LegPhase.TRANSFER
                else:
                    # 抬到位还没等到放气确认：排气堵/传感器漂移不能静默停摆
                    lift_t = (cfg.lift_clearance + cfg.leg(name).press_delta_mm) \
                        / cfg.lift_speed
                    if self._seg_t[name] > lift_t + VENT_STALL_S:
                        self.frozen = f"{name} 放气确认超时（排气堵/传感器漂移？）"
        elif ph == LegPhase.TRANSFER:
            s, start = self._transfer[name]
            s = min(1.0, s + dt / cfg.transfer_time)
            self._transfer[name] = (s, start)
            ss = _smoothstep(s)
            lx, ly = self.landing[name]
            f[0] = start[0] + (lx - start[0]) * ss
            f[1] = start[1] + (ly - start[1]) * ss
            arc = max(0.0, cfg.step_height - cfg.lift_clearance)
            f[2] = z_lift + arc * math.sin(math.pi * s)
            if s >= 1.0:
                f[0], f[1], f[2] = lx, ly, z_lift
                self.phase_of[name] = LegPhase.DESCEND
        elif ph == LegPhase.DESCEND:
            f[2] = max(self.z0, f[2] - cfg.descend_speed * dt)
            if f[2] <= self.z0 + _EPS:
                self.phase_of[name] = LegPhase.PRESS
        elif ph == LegPhase.PRESS:
            f[2] = max(z_press, f[2] - cfg.press_speed * dt)
            if f[2] <= z_press + _EPS and self._may_attach(name):
                self.ctl.request_attach(i)
                self.phase_of[name] = LegPhase.WAIT
        elif ph == LegPhase.RETRY_LIFT:
            top = z_press + cfg.retry_lift_mm
            f[2] = min(top, f[2] + cfg.press_speed * dt)
            if f[2] >= top - _EPS:
                self.phase_of[name] = LegPhase.PRESS
        elif ph == LegPhase.WAIT:
            st = self.ctl.state[i]
            if st == FootState.ATTACHED:
                self.retries[name] = 0
                if self._attach_queue and self._attach_queue[0] == name:
                    self._attach_queue.pop(0)
                self.phase_of[name] = LegPhase.STANCE
            elif st == FootState.FAULT:
                self.retries[name] += 1
                if self.retries[name] > cfg.max_attach_retry:
                    if self.air_mode:      # 架空空走：放弃本足，当作吸附成功
                        self.air_giveups += 1
                        self.retries[name] = 0
                        self.ctl.clear_fault(i)
                        if self._attach_queue and self._attach_queue[0] == name:
                            self._attach_queue.pop(0)
                        self.phase_of[name] = LegPhase.STANCE
                    else:
                        self.frozen = (f"{name} 连续 {cfg.max_attach_retry} 次"
                                       "吸附失败，全机冻结")
                else:
                    self.ctl.clear_fault(i)
                    self.phase_of[name] = LegPhase.RETRY_LIFT
