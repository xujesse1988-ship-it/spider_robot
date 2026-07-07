#!/usr/bin/env python3
"""
爬墙专用 3D 打印部件生成器（参数化）
====================================

生成 MakeYourPet 六足爬墙改装部件的 STL：

  1. suction_foot.stl   —— 吸盘足模块：套接 9x9mm 方轴腿末端，底部安装
                            30mm 波纹真空吸盘（M5 通孔金具 + 弯头宝塔接嘴）
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
    # --- 吸盘足 ---
    cup_plate_d=40.0,      # 吸盘安装盘直径（适配 30mm 波纹吸盘）
    cup_plate_t=5.0,       # 安装盘厚度
    cup_stud_d=5.4,        # 吸盘金具螺柱通孔（M5 金具，含间隙）
    turret_od=24.0,        # 螺母腔外径
    turret_id=15.0,        # 螺母腔内径（容纳 M5 螺母 + 弯头旋转）
    turret_h=13.0,         # 螺母腔高度（侧面开窗，供拧螺母和出气管）
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
    """吸盘足：底盘(装吸盘) -> 开窗螺母腔 -> 方轴套筒。Z=0 为吸盘安装面。"""
    plate = _cyl(p["cup_plate_d"] / 2, p["cup_plate_t"])
    z = p["cup_plate_t"]
    turret = _cyl(p["turret_od"] / 2, p["turret_h"], at=(0, 0, z))
    cavity = _cyl(p["turret_id"] / 2, p["turret_h"] - 3.0, at=(0, 0, z))
    # 侧窗：拧 M5 螺母 + 弯头宝塔气管出口
    window = _box(p["turret_od"], p["turret_id"] - 2, p["turret_h"] - 3.0,
                  (p["turret_od"] / 2, 0, z + (p["turret_h"] - 3.0) / 2))
    z_top = z + p["turret_h"]
    sleeve, sleeve_cuts, _ = _sleeve(p, z_top)
    stud = _cyl(p["cup_stud_d"] / 2, p["cup_plate_t"] + 2, at=(0, 0, -1))
    solid = trimesh.boolean.union([plate, turret, sleeve], engine=E)
    return trimesh.boolean.difference(
        [solid, stud, cavity, window] + sleeve_cuts, engine=E)


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
