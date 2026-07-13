#!/usr/bin/env python3
"""
爬墙专用 3D 打印部件生成器（参数化）
====================================

生成 MakeYourPet 六足爬墙改装部件的 STL：

  1. suction_foot.stl   —— 吸盘足模块 v2：套接 9x9mm 方轴腿末端。底部卡槽块
                            从侧面滑入卡住吸盘颈部（实购件为 2.5 折波纹吸盘 +
                            直角宝塔弯头一体件，见 images/xipan.jpg；颈部轮廓
                            自下而上 Ø17x5 圆柱 / Ø15x1.5 凹槽 / Ø17x2 圆柱，
                            卡持总高 8.5mm）。上方 C 型护筒罩住金具，正面全高
                            开窗供滑入及宝塔嘴出气管。
  2. component_plate.stl—— M3 网格安装板（迷你洞洞板），用于固定真空泵/
                            电磁阀/传感器等杂件

所有关键尺寸在 PARAMS 里，按实际购买件修改后重新运行即可。
运行:  .venv/bin/python tools/generate_climbing_parts.py
输出:  hardware/climbing-parts/*.stl
依赖:  trimesh, manifold3d, numpy, scipy, shapely
"""

import os
import numpy as np
import trimesh
from trimesh.creation import box, cylinder
from trimesh.transformations import rotation_matrix

OUT = os.path.join(os.path.dirname(__file__), "..", "hardware", "climbing-parts")

PARAMS = dict(
    # --- 腿末端方轴（实测 MakeYourPet left-tibia.stl 末端为 9.0x9.0mm）---
    shaft_side=9.4,        # 套孔边长（9.0 + 0.4 装配间隙）
    socket_depth=25.0,     # 套接深度
    sleeve_wall=3.2,       # 套筒壁厚
    setscrew_d=2.8,        # M3 自攻顶丝底孔直径
    # --- 吸盘颈部卡槽（实测所购吸盘+直角宝塔一体件，images/xipan.jpg）---
    neck_d=17.0,           # 颈部圆柱直径（红标位置，凹槽上下两段相同）
    neck_lower_h=5.0,      # 凹槽以下圆柱高度
    groove_d=15.0,         # 凹槽直径
    groove_h=1.5,          # 凹槽高度
    neck_upper_h=2.0,      # 凹槽以上小圆柱高度（其上即金具）
    neck_gap=0.4,          # 颈孔装配间隙（加在直径上）
    slide_gap=0.3,         # 卡槽块高度方向滑入间隙（金具底面与块顶面之间）
    detent=0.7,            # 入口防脱凸点单边过盈（硅胶挤过后卡住）
    # --- 护筒（罩住金具，正面开窗）---
    fitting_w=17.0,        # 金具最宽处（六角对边，实测约 17）
    neck_to_elbow=35.0,    # 颈部到直角宝塔弯角的垂直距离（按从颈部上端起量留余量）
    elbow_headroom=10.0,   # 弯角以上净空
    shell_id=22.0,         # 护筒内径（须让 fitting_w 六角带角旋转，17/cos30°≈19.6）
    shell_wall=4.0,        # 护筒壁厚
    window_w=20.0,         # 正面开口宽度（整个吸盘连金具由此滑入，气管由此伸出）
    # --- 安装板 ---
    plate_w=90.0,
    plate_h=70.0,
    plate_t=3.0,
    plate_grid=10.0,
    plate_hole_d=3.2,
)

E = "manifold"  # 布尔引擎


def _cyl(r, h, at=(0, 0, 0), axis="z", sections=64):
    """圆柱：底面中心位于 at，沿 axis 正方向延伸 h。"""
    c = cylinder(radius=r, height=h, sections=sections)
    c.apply_translation([0, 0, h / 2])
    if axis == "x":
        c.apply_transform(rotation_matrix(np.pi / 2, [0, 1, 0]))
    elif axis == "y":
        c.apply_transform(rotation_matrix(-np.pi / 2, [1, 0, 0]))
    c.apply_translation(at)
    return c


def _box(ex, ey, ez, at=(0, 0, 0)):
    """长方体：几何中心位于 at。"""
    b = box(extents=[ex, ey, ez])
    b.apply_translation(at)
    return b


def _sleeve(p, z0):
    """通用方轴套筒：外壁 z0..z0+depth+floor，套孔从顶部开口向下 depth。"""
    side = p["shaft_side"] + 2 * p["sleeve_wall"]
    h = p["socket_depth"] + 3.0  # 3mm 底
    zc = z0 + h / 2
    body = _box(side, side, h, (0, 0, zc))
    hole = _box(p["shaft_side"], p["shaft_side"], p["socket_depth"] + 0.2,
                (0, 0, z0 + 3.0 + p["socket_depth"] / 2 + 0.1))
    # 两个顶丝孔（穿透 -Y 侧壁）
    screws = [
        _cyl(p["setscrew_d"] / 2, side, at=(0, -side, z0 + 3 + p["socket_depth"] * f), axis="y")
        for f in (0.35, 0.8)
    ]
    return body, [hole] + screws, z0 + h


def suction_foot(p):
    """吸盘足 v2：颈部卡槽块(+X 向侧滑入) -> C 型护筒(正面开窗) -> 方轴套筒。

    Z=0 为卡槽块底面（贴吸盘顶层波纹的肩部）。装配：吸盘连金具整体从 +X
    开口水平滑入——颈部进底部卡槽、金具走护筒开窗；内环凸筋落进 Ø15 凹槽
    锁轴向，入口两颗凸点防侧向滑出。受力：压墙经块底面传到波纹肩部；吸附
    悬挂经凸筋上表面顶住凹槽上沿（金具六角压块顶面为第二重保险）。
    """
    from trimesh.creation import revolve

    bore_r = (p["neck_d"] + p["neck_gap"]) / 2          # 8.7 颈孔
    rib_r = (p["groove_d"] + p["neck_gap"]) / 2         # 7.7 凸筋
    top_r = bore_r + 0.1                                # 上段孔加桥接下垂余量
    z_g0 = p["neck_lower_h"]                            # 凹槽下沿
    z_g1 = z_g0 + p["groove_h"]                         # 凹槽上沿
    rib_top = z_g1 - 0.1                                # 凸筋顶面（承力面，留 0.1 咬合行程）
    rib_bot = rib_top - 0.4                             # 凸筋直段下沿，其下 45° 斜面利于打印
    block_h = z_g1 + p["neck_upper_h"] - p["slide_gap"]
    shell_ri = p["shell_id"] / 2
    shell_ro = shell_ri + p["shell_wall"]
    z_ceil = p["neck_lower_h"] + p["neck_to_elbow"] + p["elbow_headroom"]
    cone_h = shell_ri - 5.0                             # 顶部 45° 收口到 Ø10，免大跨度平桥
    z_top = z_ceil + cone_h

    body = _cyl(shell_ro, z_top)
    sleeve, sleeve_cuts, _ = _sleeve(p, z_top)
    # 内腔+颈孔一体旋转体：颈孔(带凸筋) -> 块顶面台阶 -> 护筒内腔 -> 45° 锥顶
    void = revolve(np.array([
        (0.0, -0.1), (bore_r, -0.1), (bore_r, z_g0 + 0.1),
        (rib_r, rib_bot), (rib_r, rib_top), (top_r, rib_top),
        (top_r, block_h), (shell_ri, block_h), (shell_ri, z_ceil),
        (5.0, z_top), (0.0, z_top)]))
    # 侧滑通道（+X 贯通到外缘），宽度分层跟随颈部轮廓
    reach = shell_ro + 1.0
    ch = [
        _box(reach, 2 * bore_r, z_g0 + 0.2, (reach / 2, 0, z_g0 / 2)),
        _box(reach, 2 * rib_r, rib_top - z_g0 - 0.1,
             (reach / 2, 0, (z_g0 + 0.1 + rib_top) / 2)),
        _box(reach, 2 * top_r, block_h - rib_top + 0.1,
             (reach / 2, 0, (rib_top + block_h + 0.1) / 2)),
    ]
    # 护筒正面开窗：块顶面起到内腔顶，金具滑入 + 宝塔嘴气管伸出
    window = _box(reach, p["window_w"], z_ceil - block_h,
                  (reach / 2, 0, (block_h + z_ceil) / 2))
    solid = trimesh.boolean.union([body, sleeve], engine=E)
    solid = trimesh.boolean.difference(
        [solid, void, window] + ch + sleeve_cuts, engine=E)
    # 入口防脱凸点：只挤压 Ø17 两段（凹槽层通道更窄，凸点不伸入）
    bump_y = bore_r + 1.5 - p["detent"]
    bumps = [_cyl(1.5, block_h, at=(11.0, s * bump_y, 0), sections=32)
             for s in (1, -1)]
    return trimesh.boolean.union([solid] + bumps, engine=E)


def component_plate(p):
    """M3 网格安装板。"""
    plate = _box(p["plate_w"], p["plate_h"], p["plate_t"], (0, 0, p["plate_t"] / 2))
    holes = []
    nx = int((p["plate_w"] - 2 * 8) // p["plate_grid"]) + 1
    ny = int((p["plate_h"] - 2 * 8) // p["plate_grid"]) + 1
    x0, y0 = -(nx - 1) * p["plate_grid"] / 2, -(ny - 1) * p["plate_grid"] / 2
    for i in range(nx):
        for j in range(ny):
            holes.append(_cyl(p["plate_hole_d"] / 2, p["plate_t"] + 2,
                              at=(x0 + i * p["plate_grid"], y0 + j * p["plate_grid"], -1),
                              sections=24))
    return trimesh.boolean.difference([plate] + holes, engine=E)


def main():
    os.makedirs(OUT, exist_ok=True)
    parts = {
        "suction_foot": suction_foot,
        "component_plate": component_plate,
    }
    for name, fn in parts.items():
        m = fn(PARAMS)
        path = os.path.join(OUT, f"{name}.stl")
        m.export(path)
        ext = np.round(m.bounding_box.extents, 1)
        print(f"{name:18s} watertight={m.is_watertight}  "
              f"尺寸 {ext[0]}x{ext[1]}x{ext[2]}mm  体积 {m.volume/1000:.1f}cm3  -> {path}")


if __name__ == "__main__":
    main()
