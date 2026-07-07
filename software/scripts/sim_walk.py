#!/usr/bin/env python3
"""无硬件步态仿真：matplotlib 3D 动画验证 IK + 步态。

用法:
  python sim_walk.py                 # 弹窗动画（本机有显示器时）
  python sim_walk.py --gif out.gif   # 保存 GIF（无头环境）
  python sim_walk.py --gait wave --vx 40 --wz 0.2
"""
import argparse
import math
import sys

sys.path.insert(0, __file__.rsplit("/", 2)[0])
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter

from hexapod import Hexapod, MockDriver, TRIPOD, WAVE, CLIMB
from hexapod.gait import GaitEngine
from hexapod.kinematics import leg_joint_points

GAITS = {"tripod": TRIPOD, "wave": WAVE, "climb": CLIMB}


def leg_points_body(bot, name, target_body):
    """腿关节点（身体系），画图用。"""
    leg = bot.cfg.leg(name)
    p_leg = bot._body_to_leg(leg, target_body)
    from hexapod.kinematics import leg_ik
    g, a, th = leg_ik(bot.cfg, *p_leg)
    pts = leg_joint_points(bot.cfg, g, a, th)
    ang = math.radians(leg.mount_angle_deg)
    c, s = math.cos(ang), math.sin(ang)
    return [(leg.mount_x + c * x - s * y, leg.mount_y + s * x + c * y, z)
            for x, y, z in pts]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gait", choices=GAITS, default="tripod")
    ap.add_argument("--vx", type=float, default=40.0, help="前进速度 mm/s")
    ap.add_argument("--wz", type=float, default=0.0, help="转向 rad/s")
    ap.add_argument("--gif", default=None, help="保存为 GIF 而非弹窗")
    ap.add_argument("--seconds", type=float, default=3.0)
    args = ap.parse_args()

    if args.gif:
        matplotlib.use("Agg")
    bot = Hexapod(MockDriver(), gait=GAITS[args.gait])
    fps = 20
    frames = int(args.seconds * fps)

    fig = plt.figure(figsize=(7, 6))
    ax = fig.add_subplot(projection="3d")

    def draw(i):
        ax.cla()
        t = i / fps
        targets = bot.engine.foot_targets(t, args.vx, 0, args.wz)
        # 机身六边形
        ms = [(l.mount_x, l.mount_y) for l in bot.cfg.legs]
        order = [0, 1, 2, 5, 4, 3, 0]  # L1 L2 L3 R3 R2 R1
        ax.plot([ms[k][0] for k in order], [ms[k][1] for k in order],
                [0] * len(order), "k-", lw=2)
        for name, tgt in targets.items():
            pts = leg_points_body(bot, name, tgt)
            xs, ys, zs = zip(*pts)
            stance = bot.engine.phase(name, t) < bot.engine.gait.duty
            ax.plot(xs, ys, zs, "o-", lw=2, ms=3,
                    color="steelblue" if stance else "indianred")
        ax.set_xlim(-250, 250); ax.set_ylim(-250, 250); ax.set_zlim(-120, 60)
        ax.set_title(f"{bot.engine.gait.name}  t={t:.2f}s  blue=stance red=swing")

    anim = FuncAnimation(fig, draw, frames=frames, interval=1000 / fps)
    if args.gif:
        anim.save(args.gif, writer=PillowWriter(fps=fps))
        print(f"已保存 {args.gif}")
    else:
        plt.show()


if __name__ == "__main__":
    main()
