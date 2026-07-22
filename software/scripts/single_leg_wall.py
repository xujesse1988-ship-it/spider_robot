#!/usr/bin/env python3
"""P2 单腿爬墙循环：伸腿 -> 压紧 -> 抽气 -> 压力确认 -> 承力 -> 放气 -> 抬腿。

台架几何（见 docs/P2-GUIDE.md 第 2 步）：coxa 轴垂直玻璃，髋平面距玻璃 90mm；
足端 x=169 固定，z 只在法线方向进退（-70 抬离 / -90 触壁 / -105 压紧）。
压紧深度 15mm = 预压行程 13.5（LEG-GEOMETRY-OPEN §4.4 实测）+ 1.5 预载。
⚠ 物理吸盘轴 = a_t - 22.7°（不是 K→P 方向，§2.13）；x=169 是"压紧时吸盘轴⊥玻璃"
的解（a_t=-67.3°），全循环吸盘轴偏法线 ≤4.5°。

用法:
  python single_leg_wall.py --mock --cycles 3            # 无硬件干跑
  python single_leg_wall.py --cycles 1 --hold 3          # 真机单循环
  python single_leg_wall.py --cycles 50 --csv p2_50.csv  # 验收：50 循环
  python single_leg_wall.py --hold 300                   # 悬挂 5 分钟验收
  python single_leg_wall.py --release                    # 手动放气+回抬离位

Ctrl-C 中断：阀保持当前状态（吸附不放开，防止滑车坠落），舵机保持使能；
善后用 --release。失败重试策略：SUCKING 超时 -> 抬 5mm -> 重新压紧，最多 3 次。
"""
import argparse
import csv
import math
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.config import DEFAULT_CONFIG as CFG
from hexapod.kinematics import leg_ik
from hexapod.driver import Servo2040Driver, MockDriver
from hexapod.adhesion import AdhesionController, FootState, MockVacuumIO

LEG = CFG.leg("L1")
FOOT_X = 169.0          # 足端径向位置：压紧时物理吸盘轴⊥玻璃（a_t=-67.3°+δ，偏 0.1°）
Z_LIFT = -70.0          # 抬离（唇口离玻璃 20mm）
Z_CONTACT = -90.0       # 唇口刚触玻璃（= 髋平面距玻璃 H_WALL）
Z_PRESS = -105.0        # 压紧（预压 13.5 + 预载 1.5）
RETRY_LIFT_MM = 5.0
MAX_RETRY = 3
MOVE_HZ = 50.0
PUMP_TOPUP_KPA = -55.0  # 吸附保持中压力高于此重新补抽（离 ~-53 失控区留安全距离，2026-07-22 实测打嗝）
PUMP_STOP_KPA = -65.0
BURP_ALARM_KPA = -30.0  # hold 期压力突升过此线（=吸附确认线）→ 打嗝报警：吸附已不可信，立即上销


class PumpOverrideIO:
    """代理 VacuumIO：屏蔽 AdhesionController 的泵滞环（P1 单传感器下罐压
    读数=足压近似，滞环失准会让泵长转发热），泵由本脚本按周期策略控制。"""

    def __init__(self, io):
        self._io = io

    def set_pump(self, on):      # 控制器的调用被忽略
        pass

    def pump(self, on):          # 脚本用这个真正开关泵
        self._io.set_pump(on)

    def __getattr__(self, name):
        return getattr(self._io, name)


def leg_pulses(x, z):
    """台架坐标 (x, 0, z) -> {通道: 脉宽}。tibia 传 k=180-θ（k 基准，§2.9/§2.11）。"""
    g, a, th = leg_ik(CFG, x, 0.0, z)
    return {
        LEG.coxa.channel: LEG.coxa.joint_deg_to_us(math.degrees(g)),
        LEG.femur.channel: LEG.femur.joint_deg_to_us(math.degrees(a)),
        LEG.tibia.channel: LEG.tibia.joint_deg_to_us(180.0 - math.degrees(th)),
    }


class Rig:
    def __init__(self, drv, io, ctl, log):
        self.drv, self.io, self.ctl, self.log = drv, io, ctl, log
        self.cur = dict(leg_pulses(FOOT_X, Z_LIFT))
        self.t0 = time.time()
        self.cycle = 0
        self.phase = "init"
        self.print_accum = 0.0   # 距上次打印气压累计的时间，每满 1s 打印一次

    def send(self, target, dur):
        """当前脉宽 -> 目标脉宽 线性插值，期间持续跑状态机与采样。"""
        steps = max(1, int(dur * MOVE_HZ))
        start = dict(self.cur)
        for i in range(1, steps + 1):
            f = i / steps
            for ch in target:
                self.cur[ch] = start[ch] + (target[ch] - start[ch]) * f
            for ch, us in self.cur.items():
                self.drv.set_pulses_us(ch, [us])
            self.tick(1.0 / MOVE_HZ)

    def tick(self, dt):
        """跑一步状态机+采样，返回本步读到的足压 kPa（供保持期复用，避免重复读传感器）。"""
        self.ctl.update(dt)
        kpa = round(self.io.read_foot_kpa(0), 2)
        self.log.writerow([round(time.time() - self.t0, 2), self.cycle, self.phase,
                           kpa, round(self.drv.read_current_a(), 2)])
        self.print_accum += dt                   # 每秒打印一次气压（累计仿真/真机时间，mock 也生效）
        if self.print_accum >= 1.0:
            self.print_accum = 0.0
            print(f"  [{time.time() - self.t0:5.1f}s] 循环{self.cycle} {self.phase:<10} 气压 {kpa:7.2f} kPa")
        time.sleep(dt if isinstance(self.drv, Servo2040Driver) else 0)
        return kpa

    def wait(self, seconds, phase=None):
        if phase:
            self.phase = phase
        t_end = time.time() + seconds
        while time.time() < t_end or not isinstance(self.drv, Servo2040Driver):
            self.tick(0.05)
            if not isinstance(self.drv, Servo2040Driver):
                seconds -= 0.05
                if seconds <= 0:
                    break

    def goto(self, z, dur, phase):
        self.phase = phase
        self.send(leg_pulses(FOOT_X, z), dur)

    def run_cycle(self, hold_s):
        """一个完整吸放循环。返回 (成功?, 吸附耗时, 保持期最差 kPa)。"""
        self.cycle += 1
        self.io.pump(True)
        self.wait(1.5, "precharge")              # 预抽罐
        self.goto(Z_CONTACT, 1.0, "approach")
        z = Z_PRESS
        for attempt in range(1 + MAX_RETRY):
            self.goto(z, 0.8, "press")
            self.ctl.request_attach(0)
            t0 = time.time()
            while self.ctl.state[0] not in (FootState.ATTACHED, FootState.FAULT):
                self.tick(0.02)
            if self.ctl.is_attached(0):
                break
            self.ctl.clear_fault(0)              # 失败：抬 5mm 重压
            print(f"  循环{self.cycle} 第{attempt+1}次未吸住，抬 {RETRY_LIFT_MM}mm 重试")
            self.goto(z + RETRY_LIFT_MM, 0.5, "retry-lift")
        else:
            self.io.pump(False)
            self.goto(Z_LIFT, 1.0, "abort-lift")
            return False, 0.0, 0.0
        t_attach = time.time() - t0
        self.io.pump(False)

        # 保持期最差(最接近0)压力；泵按压力补抽。
        # worst/补抽都用「中值滤波后」的压力，单样本传感器毛刺(P1 单传感器读数飘)不算数；
        # 入 hold 头 0.5s 稳定期(泵刚停、压力未定)不计入 worst。tick 读一次同时供 CSV/worst/补抽。
        worst = -999.0
        med = []                                 # 最近 3 次读数，取中位数抗毛刺
        n_samples = max(1, int(hold_s / 0.05))
        skip = min(int(0.5 / 0.05), n_samples - 1)   # 稳定期跳过的样本数（并保证 worst 至少更新一次）
        t_end = time.time() + hold_s
        self.phase = "hold"
        n_mock = n_samples
        i = 0
        alarmed = False                          # 打嗝报警只在跌回安全带后重新武装，防刷屏
        while (time.time() < t_end) if isinstance(self.drv, Servo2040Driver) else n_mock > 0:
            n_mock -= 1
            kpa = self.tick(0.05)
            med.append(kpa)
            if len(med) > 3:
                med.pop(0)
            kpa_f = sorted(med)[len(med) // 2]   # 中值滤波
            if i >= skip:
                worst = max(worst, kpa_f)
            if i >= skip and kpa_f > BURP_ALARM_KPA:
                if not alarmed:
                    alarmed = True
                    print(f"\a\n  ⚠⚠⚠ 打嗝！hold 压力 {kpa_f:.1f} kPa 已高于吸附确认线"
                          f" {BURP_ALARM_KPA:.0f} —— 吸附不可信，立即上销！\n\a")
            elif kpa_f <= PUMP_TOPUP_KPA:
                alarmed = False                  # 回到补抽线以下算真恢复，重新武装
            if kpa_f > PUMP_TOPUP_KPA:
                self.io.pump(True)
            elif kpa_f < PUMP_STOP_KPA:
                self.io.pump(False)
            i += 1
        self.io.pump(False)

        self.ctl.request_release(0)
        self.phase = "vent"
        while self.ctl.state[0] is not FootState.RELEASED:
            self.tick(0.02)
        self.goto(Z_LIFT, 1.0, "lift")
        return True, t_attach, worst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    ap.add_argument("--cycles", type=int, default=1)
    ap.add_argument("--hold", type=float, default=3.0, help="每循环吸附保持秒数")
    ap.add_argument("--csv", default="/tmp/single_leg_wall.csv")
    ap.add_argument("--release", action="store_true", help="只做放气+回抬离位后退出")
    args = ap.parse_args()

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    if args.mock:
        io = PumpOverrideIO(MockVacuumIO(n_feet=1))
    else:
        from hexapod.adhesion import Pi5VacuumIO
        io = PumpOverrideIO(Pi5VacuumIO(n_feet=1))
    ctl = AdhesionController(io, n_feet=1)

    f = open(args.csv, "w", newline="")
    log = csv.writer(f)
    log.writerow(["t", "cycle", "phase", "kpa", "amp"])
    rig = Rig(drv, io, ctl, log)

    # 先发好起始姿态再使能（同 stand_up.py，防上电乱跳）
    for ch, us in rig.cur.items():
        drv.set_pulses_us(ch, [us])
    drv.enable(True)
    time.sleep(0 if args.mock else 1.0)

    if args.release:
        io.set_valve(0, False)
        rig.wait(1.0, "manual-vent")
        rig.goto(Z_LIFT, 1.5, "lift")
        drv.close(); io.pump(False); f.close()
        print("已放气并回抬离位。")
        return

    ok = 0
    try:
        for _ in range(args.cycles):
            good, t_at, worst = rig.run_cycle(args.hold)
            ok += good
            print(f"循环 {rig.cycle}: {'成功' if good else '失败'}"
                  + (f"  吸附 {t_at:.2f}s  保持期最差 {worst:.1f}kPa" if good else ""))
        n = args.cycles
        print(f"\n共 {n} 循环，成功 {ok}，成功率 {100*ok/n:.0f}%（验收 >95%）")
        print(f"明细已存 {args.csv}")
        drv.close(); f.close()
    except KeyboardInterrupt:
        # 不放气、不断舵机使能：吸附中的滑车不能掉。善后跑 --release。
        f.close()
        print(f"\n中断：阀/舵机保持当前状态（防滑车坠落），善后请跑 --release。"
              f"当前压力 {io.read_foot_kpa(0):.1f} kPa")


if __name__ == "__main__":
    main()
