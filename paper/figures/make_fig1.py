#!/usr/bin/env python3
"""Fig. 1 (b)(c) 示意图:平台腿命名/并联弹簧模型/零力交接运动学。
在仓库根目录跑:
    .venv/bin/python paper/figures/make_fig1.py
输出 paper/figures/fig_platform.pdf(+ .png 预览)。

不依赖任何实验数据——纯几何示意,给没见过这台机器人的读者一个心智模型:
  (a) 俯视:六腿命名 L/R × 1/2/3,前 = 沿墙向上,重力向下;
  (b) 侧视:每条吸附腿链折叠成一根弹簧 k_i,足端锚点不滑,机体挂在弹簧下;
  (c) 交接:被抬腿 j 的足端指令上移 δ_j(弹簧归零),支撑足指令下移 w_i δ_j,
      六足指令均值不变 → 机体指令不动 → 放气时无可释放。
真机照片 (a-photo) 仍待拍,见 main.tex 图注 TODO。
配色沿用 make_figs.py:墨色/灰阶 + L1 蓝作被抬腿强调色。
"""
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Ellipse, Circle, Rectangle

OUT = os.path.dirname(os.path.abspath(__file__))
INK, MUT, LIGHT, PALE = "#1a1a1a", "#555555", "#b8b8b8", "#e6e6e6"
ACC = "#2457b0"      # 被抬腿 j(与 make_figs 的 L1 蓝一致)
ACC2 = "#c05621"     # 指令位移箭头

plt.rcParams.update({
    "font.size": 7, "axes.linewidth": 0.6, "pdf.fonttype": 42,
    "mathtext.fontset": "dejavusans",
})


# ------------------------------------------------------------- primitives
def spring(ax, p0, p1, n=7, amp=0.07, color=INK, lw=0.8, z=3):
    """p0→p1 之间画 n 个折弯的锯齿弹簧,两端各留一小段直线。"""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0
    L = np.hypot(*d)
    u = d / L
    v = np.array([-u[1], u[0]])
    lead = 0.12 * L
    pts = [p0, p0 + u * lead]
    inner = L - 2 * lead
    for i in range(1, 2 * n):
        s = lead + inner * i / (2 * n)
        pts.append(p0 + u * s + v * amp * (1 if i % 2 else -1))
    pts += [p1 - u * lead, p1]
    pts = np.array(pts)
    ax.plot(pts[:, 0], pts[:, 1], color=color, lw=lw, zorder=z,
            solid_joinstyle="miter")


def wall(ax, x, y0, y1, side="left"):
    ax.plot([x, x], [y0, y1], color=INK, lw=1.6, zorder=2)
    # 斜线阴影表示墙体
    step = 0.16
    ys = np.arange(y0, y1, step)
    sgn = -1 if side == "left" else 1
    for y in ys:
        ax.plot([x, x + sgn * 0.13], [y, y + 0.13], color=LIGHT, lw=0.5,
                zorder=1)


def cup(ax, x, y, color=INK, z=4):
    ax.add_patch(Ellipse((x + 0.05, y), 0.1, 0.24, facecolor="white",
                         edgecolor=color, lw=0.8, zorder=z))


def body(ax, x0, y0, w, h, z=3):
    ax.add_patch(FancyBboxPatch((x0, y0), w, h,
                                boxstyle="round,pad=0,rounding_size=0.06",
                                facecolor=PALE, edgecolor=INK, lw=0.8,
                                zorder=z))


def hollow(ax, x, y, color=ACC2, z=5):
    """指令足端位置(空心标记)。"""
    ax.add_patch(Circle((x, y), 0.07, facecolor="white", edgecolor=color,
                        lw=0.9, zorder=z))


def arrow(ax, p0, p1, color=INK, lw=0.7, ms=5, z=6, ls="-"):
    ax.annotate("", xy=p1, xytext=p0,
                arrowprops=dict(arrowstyle="-|>", color=color, lw=lw,
                                mutation_scale=ms, linestyle=ls,
                                shrinkA=0, shrinkB=0), zorder=z)


# ------------------------------------------------------------- panels
def panel_top(ax):
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-2.0, 2.45)
    ax.set_ylim(-1.55, 1.75)
    # 机身
    ax.add_patch(FancyBboxPatch((-0.42, -0.85), 0.84, 1.7,
                                boxstyle="round,pad=0,rounding_size=0.2",
                                facecolor=PALE, edgecolor=INK, lw=0.8,
                                zorder=3))
    legs = {  # name: (hip, knee, foot)
        "L1": ((-0.42, 0.62), (-0.95, 1.0), (-1.35, 1.0)),
        "L2": ((-0.42, 0.0), (-1.05, 0.05), (-1.5, 0.0)),
        "L3": ((-0.42, -0.62), (-0.95, -1.0), (-1.35, -1.0)),
    }
    for name, (hip, knee, foot) in list(legs.items()):
        legs["R" + name[1]] = tuple((-p[0], p[1]) for p in (hip, knee, foot))
    for name, (hip, knee, foot) in legs.items():
        col = ACC if name == "L1" else INK
        ax.plot([hip[0], knee[0], foot[0]], [hip[1], knee[1], foot[1]],
                color=col, lw=1.0, zorder=2, solid_capstyle="round")
        ax.add_patch(Circle(foot, 0.16, facecolor="white", edgecolor=col,
                            lw=0.8, zorder=3))
        ax.add_patch(Circle(foot, 0.05, facecolor=col, edgecolor=col,
                            zorder=4))
        dx = -0.16 if name[0] == "L" else 0.16
        ax.text(foot[0] + dx * 1.6, foot[1], name, ha="right" if dx < 0
                else "left", va="center", fontsize=6.5, color=col)
    # 前方 = 沿墙向上
    arrow(ax, (0, 0.95), (0, 1.55), color=INK, lw=0.8, ms=6)
    ax.text(0.08, 1.62, "front = up the wall", ha="left", va="center",
            fontsize=6, color=MUT)
    # 重力(放在最右侧,避开 R2 标签)
    arrow(ax, (2.25, 0.35), (2.25, -0.35), color=MUT, lw=0.8, ms=6)
    ax.text(2.25, 0.5, "$g$", ha="center", va="bottom", fontsize=7,
            color=MUT)
    ax.text(0, -1.45, "(a) top view, leg names", ha="center", va="center",
            fontsize=7, color=INK)


def panel_side(ax):
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-0.35, 2.45)
    ax.set_ylim(-1.55, 1.75)
    wall(ax, 0, -1.25, 1.45)
    ax.text(-0.1, 1.55, "wall", ha="center", va="bottom", fontsize=6,
            color=MUT)
    body(ax, 1.15, -0.45, 0.55, 0.9)
    ax.text(1.425, 0.0, "body", ha="center", va="center", fontsize=6.5,
            color=INK)
    legs = [(1.0, 0.33, "1"), (0.0, 0.0, "2"), (-1.0, -0.33, "3")]
    for ycup, ybody, tag in legs:
        col = ACC if tag == "1" else INK
        cup(ax, 0, ycup, color=col)
        spring(ax, (1.15, ybody), (0.12, ycup), n=6, amp=0.07, color=col)
        ax.text(0.62, (ycup + ybody) / 2 + (0.17 if tag != "3" else -0.19),
                f"$k_{tag}$", ha="center", va="center", fontsize=7,
                color=col)
    # 锚点标注(指向顶部吸盘,文字放在弹簧上方的空白区)
    ax.annotate("anchor $a_i$ (cup: no slip)", xy=(0.1, 1.08),
                xytext=(0.42, 1.42), fontsize=5.8, color=MUT,
                ha="left", va="center",
                arrowprops=dict(arrowstyle="-", color=LIGHT, lw=0.5))
    # 重力 mg
    arrow(ax, (1.425, -0.45), (1.425, -1.05), color=INK, lw=0.9, ms=6)
    ax.text(1.5, -0.8, "$mg$", ha="left", va="center", fontsize=7)
    # x 轴向下
    arrow(ax, (2.15, 1.3), (2.15, 0.5), color=MUT, lw=0.7, ms=5)
    ax.text(2.2, 0.9, "$x$", ha="left", va="center", fontsize=7, color=MUT)
    ax.text(2.13, 1.45, "down-\nwall", ha="center", va="bottom",
            fontsize=5.5, color=MUT)
    ax.text(1.05, -1.45, "(b) side view: leg chains as springs",
            ha="center", va="center", fontsize=7, color=INK)


def panel_handover(ax):
    """两个状态并排:交接前(腿 j 带力) / 交接后(腿 j 零力)。"""
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_xlim(-0.3, 5.9)
    ax.set_ylim(-1.7, 1.75)
    e_j, w_d = 0.34, 0.12          # 储存变形 / 支撑腿分得的下移量(示意)

    def state(x0, after):
        wall(ax, x0, -1.35, 1.45)
        bx = x0 + 1.15
        body(ax, bx, -0.45, 0.55, 0.9)
        legs = [(1.0, 0.33, True), (0.0, 0.0, False), (-1.0, -0.33, False)]
        for ycup, ybody, is_j in legs:
            col = ACC if is_j else INK
            cup(ax, x0, ycup, color=col)
            spring(ax, (bx, ybody), (x0 + 0.12, ycup), n=6, amp=0.07,
                   color=col)
            # 指令足端位置(空心):吸附腿都在吸盘之下 e(被绕紧);
            # 交接后 j 回到吸盘处,支撑再下移 w_i δ_j
            if is_j:
                yc = ycup - (0.0 if after else e_j)
            else:
                yc = ycup - 0.18 - (w_d if after else 0.0)
            hollow(ax, x0 + 0.12, yc)
            # 指令位移箭头贴着墙面画(x0+0.12),标签放在弹簧下方的空白处
            if is_j and not after:
                arrow(ax, (x0 + 0.12, yc + 0.08), (x0 + 0.12, ycup - 0.13),
                      color=ACC2, lw=0.8, ms=5)
                ax.text(x0 + 0.24, yc - 0.13, r"$+\delta_j$",
                        ha="left", va="center", fontsize=6, color=ACC2)
            if (not is_j) and (not after):
                arrow(ax, (x0 + 0.12, yc - 0.08), (x0 + 0.12, yc - 0.3),
                      color=ACC2, lw=0.8, ms=5)
                ax.text(x0 + 0.24, yc - 0.22, r"$-w_i\delta_j$",
                        ha="left", va="center", fontsize=6, color=ACC2)
        return bx

    bx1 = state(0.0, after=False)
    bx2 = state(3.15, after=True)
    # 状态标题
    ax.text(0.95, 1.6, "before handover: leg $j$ loaded", ha="center",
            va="center", fontsize=6.3, color=INK)
    ax.text(4.1, 1.6, "after: $f_j=0$, body unchanged", ha="center",
            va="center", fontsize=6.3, color=INK)
    # 过渡箭头
    arrow(ax, (2.2, 0.0), (2.95, 0.0), color=MUT, lw=0.9, ms=7)
    ax.text(2.57, 0.16, "10 mm/s", ha="center", va="bottom", fontsize=5.5,
            color=MUT)
    # 图例
    hollow(ax, 0.05, -1.72)
    ax.text(0.18, -1.72, "commanded foot position", ha="left", va="center",
            fontsize=5.8, color=MUT)
    ax.text(2.75, -1.72, r"$\Sigma_i w_i=1$: six-command mean unchanged",
            ha="left", va="center", fontsize=5.8, color=MUT)
    ax.text(2.8, -2.0, "(c) zero-force handover (side view; vent follows)",
            ha="center", va="center", fontsize=7, color=INK)


def main():
    fig = plt.figure(figsize=(3.5, 4.0))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.0, 1.05],
                          width_ratios=[1.0, 0.98],
                          left=0.01, right=0.99, top=0.99, bottom=0.03,
                          hspace=0.08, wspace=0.02)
    panel_top(fig.add_subplot(gs[0, 0]))
    panel_side(fig.add_subplot(gs[0, 1]))
    ax = fig.add_subplot(gs[1, :])
    panel_handover(ax)
    ax.set_ylim(-2.15, 1.75)
    for ext in ("pdf", "png"):
        fig.savefig(os.path.join(OUT, f"fig_platform.{ext}"),
                    dpi=300 if ext == "png" else None)
    print("wrote fig_platform.pdf/png")


if __name__ == "__main__":
    main()
