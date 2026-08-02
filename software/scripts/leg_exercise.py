#!/usr/bin/env python3
"""单腿动作演示：装好一条腿后，不等整机就让它做站立和走路动作。

场景（P3 第 3 步装配期）：腿刚装上 frame，机身垫高在台面上、足端悬空。
流程：使能到站姿 -> 保持 -> 过渡到步态起点 -> 按步态引擎走 N 秒 -> 回站姿。
足端轨迹与整机行走时该腿的轨迹完全一致（同一个 GaitEngine），
但只发这条腿的 3 个通道，其余 15 路不发——没装的腿不受影响。
发真机前先做全轨迹自检：逐帧 IK 可达 + 脉宽不撞 500/2500 限幅。

前提：舵盘已按规矩装好（servo_center.py 回中后装）。使能瞬间舵机直接跳到
站姿，手离腿远点。Ctrl-C 立即断舵机电——腿会软掉下垂，悬空无风险。

用法:
  python leg_exercise.py --mock                  # 无硬件干跑（自检+轨迹采样打印）
  python leg_exercise.py                         # 真机：L1 站立 + 走 4 个步态周期
  python leg_exercise.py --walk-time 12 --vx 25  # 慢速多走一会
  python leg_exercise.py --gait wave             # 波浪步态（支撑相长、摆动短快）
  python leg_exercise.py --leg L2                # 之后装好的腿复用本脚本
"""
import argparse
import math
import sys
import time

sys.path.insert(0, __file__.rsplit("/", 2)[0])
from hexapod.config import DEFAULT_CONFIG as CFG, FEMUR_ATTACH, TIBIA_ATTACH
from hexapod.driver import Servo2040Driver, MockDriver
from hexapod.gait import GaitEngine, TRIPOD, WAVE
from hexapod.kinematics import leg_ik, WorkspaceError

GAITS = {"tripod": TRIPOD, "wave": WAVE}
STAND_HOLD_S = 2.0
GLIDE_S = 1.5           # 姿态间平滑过渡时间
PRINT_EVERY_S = 0.5     # 走路时状态打印间隔


def body_to_leg(leg, p):
    """身体系 -> 腿坐标系（同 robot._body_to_leg，身体姿态偏移取零）。"""
    x, y, z = p[0] - leg.mount_x, p[1] - leg.mount_y, p[2]
    a = -math.radians(leg.mount_angle_deg)
    c, s = math.cos(a), math.sin(a)
    return c * x - s * y, s * x + c * y, z


def leg_pulses(leg, p_body):
    """身体系足端目标 -> ({通道: 脉宽}, (γ°, α°, k°))。tibia 传 k=180-θ（k 基准，同 robot.pulses）。"""
    g, a, th = leg_ik(CFG, *body_to_leg(leg, p_body))
    k = 180.0 - math.degrees(th)
    return {
        leg.coxa.channel: leg.coxa.joint_deg_to_us(math.degrees(g)),
        leg.femur.channel: leg.femur.joint_deg_to_us(math.degrees(a)),
        leg.tibia.channel: leg.tibia.joint_deg_to_us(k),
    }, (math.degrees(g), math.degrees(a), k)


def plan_points(leg, engine, vx, vy, walk_time):
    """全程身体系足端点：站姿 + 走路轨迹逐帧（与执行时同一采样率）。"""
    pts = [engine.default_feet[leg.name]]
    dt = 1.0 / CFG.update_hz
    for i in range(int(walk_time * CFG.update_hz) + 1):
        pts.append(engine.foot_targets(i * dt, vx, vy, 0.0)[leg.name])
    return pts


def preflight(leg, pts):
    """逐点 IK+脉宽自检，返回三关节的 (角度min, 角度max, 脉宽min, 脉宽max)。"""
    cals = (leg.coxa, leg.femur, leg.tibia)
    jmin, jmax = [1e9] * 3, [-1e9] * 3
    umin, umax = [1e9] * 3, [-1e9] * 3
    for p in pts:
        pulses, degs = leg_pulses(leg, p)
        for i, cal in enumerate(cals):
            us = pulses[cal.channel]
            if us <= cal.min_us + 1 or us >= cal.max_us - 1:
                raise ValueError(
                    f"通道{cal.channel} 在足端 ({p[0]:.0f},{p[1]:.0f},{p[2]:.0f}) "
                    f"脉宽 {us:.0f}µs 已到限幅——减小 vx 或 step_height")
            jmin[i], jmax[i] = min(jmin[i], degs[i]), max(jmax[i], degs[i])
            umin[i], umax[i] = min(umin[i], us), max(umax[i], us)
    return jmin, jmax, umin, umax


class LegRig:
    def __init__(self, drv, leg, real):
        c = leg.coxa.channel
        assert (leg.femur.channel, leg.tibia.channel) == (c + 1, c + 2)
        self.drv, self.leg, self.real = drv, leg, real
        self.foot = None            # 当前身体系足端

    def send_foot(self, p_body):
        pulses, _ = leg_pulses(self.leg, p_body)
        c = self.leg.coxa.channel
        # 三通道连续，一个 SET 包发完（固件要求单包一次 write）
        self.drv.set_pulses_us(c, [pulses[c], pulses[c + 1], pulses[c + 2]])
        self.foot = tuple(p_body)

    def glide(self, p_to, dur):
        steps = max(2, int(dur * CFG.update_hz))
        p0 = self.foot
        for i in range(1, steps + 1):
            s = i / steps
            s = s * s * (3 - 2 * s)
            self.send_foot(tuple(a + (b - a) * s for a, b in zip(p0, p_to)))
            self.sleep(1.0 / CFG.update_hz)

    def sleep(self, dt):
        if self.real:
            time.sleep(dt)

    def power_str(self):
        if not self.real:
            return ""
        v, a = self.drv.read_voltage_v(), self.drv.read_current_a()
        warn = "  ⚠电压低" if v < CFG.volt_warn else ""
        return f"  {v:.2f}V {a:.1f}A{warn}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--mock", action="store_true", help="无硬件干跑")
    ap.add_argument("--leg", default="L1", choices=[l.name for l in CFG.legs])
    ap.add_argument("--gait", default="tripod", choices=list(GAITS))
    ap.add_argument("--vx", type=float, default=40.0, help="前进速度 mm/s")
    ap.add_argument("--vy", type=float, default=0.0, help="侧移速度 mm/s")
    ap.add_argument("--walk-time", type=float, default=6.0,
                    help="走路秒数（默认 6s = 4 个步态周期）")
    args = ap.parse_args()

    leg = CFG.leg(args.leg)
    engine = GaitEngine(CFG, GAITS[args.gait])
    c = leg.coxa.channel
    print(f"{leg.name}: 通道 coxa/femur/tibia = {c}/{c+1}/{c+2}, attach = "
          f"{leg.coxa.attach_deg:.1f}/{leg.femur.attach_deg:.1f}/{leg.tibia.attach_deg:.1f}°")
    if leg.femur.attach_deg == FEMUR_ATTACH or leg.tibia.attach_deg == TIBIA_ATTACH:
        print(f"⚠ {leg.name} 的 femur/tibia attach_deg 还是官方默认值（未实测），"
              "动作能跑但姿态可能偏一个花键齿；正式标定见 P3-GUIDE 第 4 步。")

    pts = plan_points(leg, engine, args.vx, args.vy, args.walk_time)
    try:
        jmin, jmax, umin, umax = preflight(leg, pts)
    except (WorkspaceError, ValueError) as e:
        sys.exit(f"轨迹自检失败：{e}")
    print(f"轨迹自检通过：{len(pts)} 帧全部可达（{args.gait}, vx={args.vx:.0f}mm/s）")
    for i, name in enumerate(("coxa γ", "femur α", "tibia k")):
        print(f"  {name:<8} 通道{c+i:>2}  角度 {jmin[i]:6.1f}° .. {jmax[i]:6.1f}°"
              f"  脉宽 {umin[i]:4.0f} .. {umax[i]:4.0f}µs")

    drv = MockDriver() if args.mock else Servo2040Driver(args.port)
    rig = LegRig(drv, leg, not args.mock)
    stand = engine.default_feet[leg.name]

    try:
        # 先发站姿脉宽再使能——使能瞬间直接到位（同 stand_up.py，防上电乱跳）
        rig.send_foot(stand)
        drv.enable(True)
        print(f"\n[站立] 保持 {STAND_HOLD_S:.0f}s，足端身体系 "
              f"({stand[0]:.0f}, {stand[1]:.0f}, {stand[2]:.0f})")
        held = 0.0
        while held < STAND_HOLD_S:
            rig.sleep(1.0)
            held += 1.0
            if rig.power_str():
                print(f"  t={held:.0f}s{rig.power_str()}")

        rig.glide(pts[1], GLIDE_S)   # 过渡到步态 t=0 的落点（最多差半个步幅）
        print(f"[走路] {args.walk_time:.0f}s，周期 {CFG.cycle_time}s，"
              f"抬脚 {CFG.step_height:.0f}mm")
        dt = 1.0 / CFG.update_hz
        next_pr = 0.0
        for i in range(int(args.walk_time * CFG.update_hz)):
            t = i * dt
            rig.send_foot(engine.foot_targets(t, args.vx, args.vy, 0.0)[leg.name])
            if t >= next_pr:
                ph = engine.phase(leg.name, t)
                st = "支撑" if ph < engine.gait.duty else "摆动"
                x, y, z = rig.foot
                print(f"  t={t:4.1f}s {st} 相位{ph:.2f} "
                      f"足端({x:6.1f},{y:6.1f},{z:6.1f}){rig.power_str()}")
                next_pr += PRINT_EVERY_S
            rig.sleep(dt)

        print("[站立] 回站姿")
        rig.glide(stand, GLIDE_S)
        if args.mock:
            print("mock 演示完成。")
        else:
            input("演示完成。回车断舵机电（腿会软掉，先扶稳）… ")
    except KeyboardInterrupt:
        print("\n中断：断舵机电。")
    finally:
        drv.close()


if __name__ == "__main__":
    main()
