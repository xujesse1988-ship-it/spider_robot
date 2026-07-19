#!/usr/bin/env python3
"""
爬墙专用 3D 打印部件生成器（参数化）
====================================

生成 MakeYourPet 六足爬墙改装部件的 STL：

  1. left-tibia-suction.stl —— 吸盘足 v3：与 left-tibia 一体化的小腿。
       末端腔体按实购"2.5 折波纹吸盘 + 两颗六角螺母 + 直角宝塔弯头"一体件
       （images/xipan_marked.jpeg 三视图）做全形状负模：
       Ø27 折痕座圈 + 45° 肩面 → Ø17 主孔 → Ø15 凹槽凸筋 → Ø17 圈 →
       边长7.3 六角袋 → 边长6 六角袋（锁转动）→ Ø9.6 弯头腔，顶面即金属
       最高点。舵盘螺丝孔扩为 M3 自攻（Ø2.8）。前下方开口由
       独立的门盖封住（胶粘），门顶以上开放，宝塔嘴可转动出管。
       打印方向：立打（腔体朝下、tibia 朝上）——方轴轴线距 tibia 平背面
       仅 4.5mm 而腔体半径 17mm，平躺必然穿透打印床，几何上只能立打；
       建议 PETG、5~6 壁、45% 填充，开 brim；tibia 细节处可开支撑保险。
  2. suction-foot-door.stl  —— 上述腔体的门盖（含前半负模 + 4 定位销），
       外平面朝下平躺打印，无支撑。
  3. component_plate.stl    —— M3 网格安装板，固定真空泵/电磁阀/传感器。

所有关键尺寸在 PARAMS 里，按实际购买件修改后重新运行即可。
运行:  .venv/bin/python tools/generate_climbing_parts.py
输出:  hardware/climbing-parts/*.stl
依赖:  trimesh, manifold3d, numpy, scipy, shapely
"""

import os
import numpy as np
import trimesh
from trimesh.creation import box, cylinder, revolve, extrude_polygon
from trimesh.transformations import rotation_matrix

HERE = os.path.dirname(__file__)
OUT = os.path.join(HERE, "..", "hardware", "climbing-parts")
TIBIA = os.path.join(HERE, "..", "hardware", "makeyourpet-hexapod", "STL",
                     "left-tibia.stl")

PARAMS = dict(
    # --- 吸盘+金具一体件实测（自下而上；z=0 取 Ø27 第一折痕平面）---
    crease_d=27.0,        # 第一折痕直径（被包裹段的最大直径）
    shoulder_deg=45.0,    # 折痕肩部母线与底面夹角（2026-07 新吸盘件，原为 5°）
    neck_h=10.0,          # 折痕到凹槽下沿总高（45° 圆台 5.0 + Ø17 圆柱 5.0）
    groove_d=15.0,        # 凹槽直径
    groove_h=1.7,         # 凹槽高度
    ring_d=17.0,          # 凹槽上方小圆柱直径
    ring_h=2.0,           # 凹槽上方小圆柱高度
    nut1_side=7.3,        # 下六角螺母边长（对边 = 边长*√3 ≈ 12.64）
    nut1_h=3.0,
    nut2_side=6.0,        # 上六角螺母边长（对边 ≈ 10.39）
    nut2_h=9.0,
    elbow_d=9.0,          # 直角宝塔竖直部分外径
    elbow_h=13.0,         # 弯头总高（金属最高点 = 38.7）
    barb_reach=24.0,      # 宝塔嘴尖到竖直轴线的水平距离（需伸出门外）
    hose_od=6.0,          # 软管外径
    # --- 装配间隙 ---
    gap_d=0.4,            # 圆截面直径间隙
    gap_hex=0.4,          # 六角对边间隙
    gap_z=0.3,            # 关键台阶的竖直余量
    # --- 腔体外壳 / 门 ---
    wall=3.2,             # 最小壁厚
    door_h=29.5,          # 门盖高度（其上开放，供滑入和宝塔嘴转动出管）
    peg_d=2.9,            # 门定位销直径（孔 +0.8）
    # --- 与 tibia 的结合（left-tibia.stl 实测）---
    shaft_cx=1.3,         # 方轴轴心 x
    shaft_cy=-4.5,        # 方轴轴心 y（tibia 平背面为 y=0）
    horn_hole_d=2.8,      # 舵盘螺丝孔：M3 自攻（原 M1.6/Ø1.69；要 M3 过孔改 3.2）
    tibia_keep_z=-58.0,   # tibia 保留 z >= 此值（其下方轴丢弃，残段埋入颈柱）
    crease_z=-100.0,      # 折痕平面在 tibia 坐标系的 z（决定腿长）
    # --- 安装板 ---
    plate_w=90.0,
    plate_h=70.0,
    plate_t=3.0,
    plate_grid=10.0,
    plate_hole_d=3.2,
)

E = "manifold"  # 布尔引擎

# left-tibia.stl 实测 4 个舵盘螺丝孔位 (x, z)，节圆 Ø49，孔轴沿 y
HORN_HOLES = [(28.51, -29.82), (36.17, -36.22), (59.37, 6.98), (67.02, 0.53)]


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


def _hex_pocket(side, gap, z0, z1, at):
    """六角袋：对边 = side*√3 + gap，平面法向 ±X（横滑入时两壁夹持对边）。"""
    af = side * np.sqrt(3.0) + gap
    r = af / np.sqrt(3.0)          # 六边形外接圆半径
    h = cylinder(radius=r, height=z1 - z0, sections=6)
    h.apply_transform(rotation_matrix(np.pi / 6, [0, 0, 1]))  # 平边法向转到 ±X
    h.apply_translation([at[0], at[1], (z0 + z1) / 2])
    return h


def _cavity_parts(p, ax, ay, z0):
    """吸盘+金具的全形状负模（含底部导入倒角），返回待减去的实体列表。

    z 相对折痕平面 z0 向上：
      0..~0.95         Ø(crease+0.6) 浅座圈（容纳折痕圈）
      ..~6.05          45° 肩部让位面（留 ~0.5 间隙；45° 锥面立打自承，
                       无需桥接）
      ..neck_h         Ø17 主孔（真正的侧向夹持段，约 3.2mm 高）
      ..+groove_h      Ø15 凸筋（下侧斜面便于立打，上平面承拉力）
      ..+ring_h        Ø17 圈
      ..+nut1_h        六角袋 1
      ..+nut2_h        六角袋 2
      ..+elbow_h       弯头圆腔，顶面即金属最高点(+gap_z)
    """
    g = p["gap_d"]
    seat_r = (p["crease_d"] + 0.6) / 2                  # 13.8
    neck_r = (p["ring_d"] + g) / 2                      # 8.7
    rib_r = (p["groove_d"] + g) / 2                     # 7.7
    ring_r = neck_r + 0.1                               # 8.8 上段孔
    rise = (seat_r - neck_r) * np.tan(np.radians(p["shoulder_deg"]))  # 肩部升高 5.1
    lip_h = 0.95                                        # 座圈竖直段（导入斜面 0.45 + 直壁 0.5）
    zg0 = p["neck_h"]                                   # 凹槽下沿 10
    assert lip_h + rise <= zg0 - 0.75, "肩部锥面顶进凹槽段：检查 shoulder_deg/neck_h"
    zg1 = zg0 + p["groove_h"]                           # 凹槽上沿 11.7
    rib_top = zg1 - 0.15                                # 凸筋顶面，留咬合行程
    z_ring = zg1 + p["ring_h"]                          # 13.7
    z_n1 = z_ring + p["nut1_h"]                         # 16.7
    z_n2 = z_n1 + p["nut2_h"]                           # 25.7
    z_top = z_n2 + p["elbow_h"] + p["gap_z"]            # 39.0 腔顶
    # 旋转体：座圈 + 45° 肩面 + Ø17 主孔 + 凸筋 + Ø17 圈（六角袋以上另做棱柱）
    prof = np.array([
        (0.0, -0.01),
        (seat_r + 0.4, -0.01),                           # 底缘导入
        (seat_r, lip_h - 0.5),
        (seat_r, lip_h),                                 # Ø27.6 座圈
        (neck_r, lip_h + rise),                          # 45° 肩部让位面
        (neck_r, zg0 - 0.75),                            # Ø17.4 主孔
        (rib_r, zg0 + 0.45),                             # 凸筋下斜面（立打免支撑）
        (rib_r, rib_top),
        (ring_r, rib_top),                               # 凸筋顶平面（承力）
        (ring_r, z_ring + p["gap_z"]),
        (0.0, z_ring + p["gap_z"]),
    ])
    rev = revolve(prof)
    rev.apply_translation([ax, ay, z0])
    hex1 = _hex_pocket(p["nut1_side"], p["gap_hex"],
                       z0 + z_ring - 0.1, z0 + z_n1 + p["gap_z"], (ax, ay))
    hex2 = _hex_pocket(p["nut2_side"], p["gap_hex"],
                       z0 + z_n1 - 0.1, z0 + z_n2 + p["gap_z"], (ax, ay))
    elbow = _cyl((p["elbow_d"] + 0.6) / 2, z_top - (z_n2 - 0.1),
                 at=(ax, ay, z0 + z_n2 - 0.1))
    return [rev, hex1, hex2, elbow], z_top


def _peg_spots(p, ax):
    """门定位销的 (x, z_rel) 位置：避开锥腔，落在实心壁上。"""
    return [(ax - 13.5, 10.0), (ax + 13.5, 10.0),
            (ax - 13.5, 24.0), (ax + 13.5, 24.0)]


def tibia_suction(p):
    """吸盘足 v3 主体：腔体外壳(前下开口) + 45°斜顶 + 颈柱 + tibia 本体。"""
    ax, ay, z0 = p["shaft_cx"], p["shaft_cy"], p["crease_z"]
    w = p["wall"]
    cuts, z_top = _cavity_parts(p, ax, ay, z0)          # z_top = 腔顶(相对折痕)
    r_out = (p["crease_d"] + 0.6) / 2 + w               # 17.0 外壳半宽
    # 外壳箱体（z: 折痕..腔顶）
    box0 = _box(2 * r_out, 2 * r_out, z_top,
                (ax, ay, z0 + z_top / 2))
    # 顶部：后半平板(y>=轴面) + 前半 45° 斜顶（立打免支撑），共 8mm
    cap_h = 8.0
    cap = _box(2 * r_out, r_out, cap_h,
               (ax, ay + r_out / 2, z0 + z_top + cap_h / 2))
    from shapely.geometry import Polygon
    tri = Polygon([(ay, z_top), (ay, z_top + cap_h), (ay - 8.0, z_top + cap_h)])
    wedge = extrude_polygon(tri, 2 * r_out)
    # extrude_polygon 生成 (多边形xy, 挤出z)；映射 多边形x->y, 多边形y->z, 挤出->x
    m = np.eye(4)
    m[:3, :3] = np.array([[0., 0., 1.], [1., 0., 0.], [0., 1., 0.]])
    wedge.apply_transform(m)
    wedge.apply_translation([ax - r_out, 0, z0])
    # 颈柱：包住 tibia 方轴残段与加强筋根部（x -4..18, y -12.5..2）
    collar = _box(22.0, 14.5, 11.0,
                  (ax + 5.7, -12.5 + 14.5 / 2, z0 + z_top + cap_h + 5.5))
    # tibia 本体（丢弃 tibia_keep_z 以下的方轴）
    tib = trimesh.load(TIBIA)
    keep = _box(200, 200, 200, (0, 0, p["tibia_keep_z"] + 100))
    tib = trimesh.boolean.intersection([tib, keep], engine=E)
    solid = trimesh.boolean.union([box0, cap, wedge, collar, tib], engine=E)
    # 前下开口（门所在区域 + 其上宝塔嘴转动空间）：整个 y < ay 前半清空
    front = _box(2 * r_out + 2, r_out + 2, z_top + 0.001,
                 (ax, ay - (r_out + 2) / 2, z0 + (z_top + 0.001) / 2 - 0.001))
    # 门定位销孔（沿 +Y 钻入门贴合面）
    holes = [_cyl((p["peg_d"] + 0.8) / 2, 4.0,
                  at=(x, ay - 0.5, z0 + z), axis="y", sections=24)
             for x, z in _peg_spots(p, ax)]
    # 舵盘螺丝孔扩为 M3 自攻（原 Ø1.69 只够 M1.6）；沿 y 通钻，
    # 出口侧原有的 Ø3.0 让位段不受影响
    horn = [_cyl(p["horn_hole_d"] / 2, 15.0, at=(x, -14.0, z), axis="y",
                 sections=24)
            for x, z in HORN_HOLES]
    return trimesh.boolean.difference([solid, front] + cuts + holes + horn,
                                      engine=E)


def suction_door(p):
    """门盖：前半负模 + 底缘导入 + 4 定位销；外平面(y=-r_out)朝下平躺打印。"""
    ax, ay, z0 = p["shaft_cx"], p["shaft_cy"], p["crease_z"]
    r_out = (p["crease_d"] + 0.6) / 2 + p["wall"]
    cuts, _ = _cavity_parts(p, ax, ay, z0)
    blk = _box(2 * r_out, r_out, p["door_h"],
               (ax, ay - r_out / 2, z0 + p["door_h"] / 2))
    door = trimesh.boolean.difference([blk] + cuts, engine=E)
    pegs = [_cyl(p["peg_d"] / 2, 4.6, at=(x, ay - 2.0, z0 + z),
                 axis="y", sections=24)
            for x, z in _peg_spots(p, ax)]
    return trimesh.boolean.union([door] + pegs, engine=E)


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
        "left-tibia-suction": tibia_suction,
        "suction-foot-door": suction_door,
        "component_plate": component_plate,
    }
    for name, fn in parts.items():
        m = fn(PARAMS)
        path = os.path.join(OUT, f"{name}.stl")
        m.export(path)
        ext = np.round(m.bounding_box.extents, 1)
        print(f"{name:20s} watertight={m.is_watertight}  "
              f"尺寸 {ext[0]}x{ext[1]}x{ext[2]}mm  体积 {m.volume/1000:.1f}cm3  -> {path}")


if __name__ == "__main__":
    main()
