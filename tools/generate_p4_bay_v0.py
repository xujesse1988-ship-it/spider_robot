#!/usr/bin/env python3
"""Generate the P4 removable electrical and pneumatic bay V0.

V0 is deliberately modular.  The frame interface, electrical deck and every
pneumatic holder are separate prints so that one bad fit does not invalidate a
large one-piece part.  Coordinates use millimetres, with +Y toward the head.

Run:
    venv/bin/python tools/generate_p4_bay_v0.py

Output:
    hardware/climbing-parts/p4-bay-v0/

The fit template must be printed and checked against the assembled coxa wiring
before the 190 x 130 mm baseplate is accepted for service.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely.affinity import translate
from shapely.geometry import LineString, Point, box as shapely_box
from shapely.ops import unary_union
from trimesh.creation import box, cylinder, extrude_polygon
from trimesh.transformations import rotation_matrix


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "hardware" / "climbing-parts" / "p4-bay-v0"
BOOLEAN_ENGINE = "manifold"


BAY = {
    "base": {
        "size": (190.0, 130.0, 3.0),
        "corner_r": 8.0,
        "opening": (70.0, 108.0, 14.0),
        "rib_h": 5.0,
        "frame_holes": [
            (-44.0, -40.0), (44.0, -40.0),
            (-44.0, 0.0), (44.0, 0.0),
            (-44.0, 40.0), (44.0, 40.0),
        ],
        "module_hole_d": 3.4,
    },
    "deck": {
        "size": (184.0, 108.0, 3.0),
        "corner_r": 6.0,
        "post_holes": [
            (-85.0, -48.0), (85.0, -48.0),
            (-85.0, 0.0), (85.0, 0.0),
            (-85.0, 48.0), (85.0, 48.0),
        ],
    },
    "boards": {
        # side is used only by the layout report.  All holes pass through the deck.
        "pi5": {
            "side": "top", "center": (-47.0, 14.0), "size": (85.0, 56.0),
            "hole_d": 2.8, "holes": [(-29.0, -24.5), (29.0, -24.5),
                                      (-29.0, 24.5), (29.0, 24.5)],
        },
        "divider": {
            "side": "top", "center": (44.0, 8.0), "size": (91.0, 70.0),
            "hole_d": 3.4, "holes": [(-41.5, -29.0), (41.5, -29.0),
                                      (-41.5, 29.0), (41.5, 29.0)],
        },
        "buck_5v": {
            "side": "top", "center": (56.0, -41.5), "size": (46.0, 24.0),
            "hole_d": 3.4, "holes": [(-20.5, -9.5), (20.5, -9.5),
                                      (-20.5, 9.5), (20.5, 9.5)],
        },
        "yynmos8": {
            "side": "bottom", "center": (-43.0, 14.0), "size": (93.0, 54.0),
            "hole_d": 3.4,
            # Measured from the PCB lower-left corner: (3,30), (90,30),
            # (3,39), (90,41.5).  Keep these explicit; this board is asymmetric.
            "holes": [(-43.5, 3.0), (43.5, 3.0),
                      (-43.5, 12.0), (43.5, 14.5)],
        },
        "relay": {
            "side": "bottom", "center": (45.0, 15.0), "size": (72.0, 40.0),
            "hole_d": 3.4, "holes": [(-33.25, -17.25), (33.25, -17.25),
                                      (-33.25, 17.25), (33.25, 17.25)],
        },
        "xl6009_left": {
            "side": "bottom", "center": (-51.0, -38.0), "size": (50.0, 28.0),
            "hole_d": 3.4, "holes": [(-21.5, -10.5), (21.5, -10.5),
                                      (-21.5, 10.5), (21.5, 10.5)],
        },
        "xl6009_right": {
            "side": "bottom", "center": (5.0, -39.0), "size": (50.0, 28.0),
            "hole_d": 3.4, "holes": [(-21.5, -10.5), (21.5, -10.5),
                                      (-21.5, 10.5), (21.5, 10.5)],
        },
    },
    "pump": {
        "adapter": (64.0, 54.0, 4.0),
        "bracket_pitch": (48.0, 28.0),
        "bracket_hole_d": 5.4,  # M4 screw plus measured-bracket tolerance
        "pad": (58.0, 38.0, 3.0),
    },
    "valve": {"count": 6, "body": (20.0, 25.0, 16.0), "pitch": 23.0},
    "sensor": {"count": 7, "body": (19.0, 19.0, 12.0), "pitch": 21.5},
    "manifold": {"body_od": 20.0, "clip_gap_d": 20.6},
    "switch": {"cutout": (19.4, 13.4), "wall_t": 1.8},
}


PART_META = {
    "p4-bay-fit-template-v0": (1, "PLA/PETG", "先打印；验证 190×130 外廓和 6 个 frame 孔"),
    "p4-bay-baseplate-v0": (1, "PETG", "确认试装模板不碰 coxa 线束后再打印"),
    "p4-bay-electrical-deck-v0": (1, "PETG", "双面装板；元件面按 layout-report.json"),
    "p4-bay-deck-post-90mm-v0": (6, "PETG", "优先用采购的 M3×90 六角柱或组合柱"),
    "p4-bay-spacer-m25-6mm-v0": (4, "PETG/PLA", "Pi 5 用"),
    "p4-bay-spacer-m3-6mm-v0": (20, "PETG/PLA", "其余 PCB，按实际装板数量打印"),
    "p4-bay-pump-adapter-v0": (1, "PETG", "V0 单泵；第二泵复用同一件"),
    "p4-bay-pump-pad-tpu-v0": (1, "TPU 95A", "泵金属支架与适配板之间"),
    "p4-bay-valve-rail-v0": (1, "PETG", "6 条 2.5mm 扎带作为上压固定"),
    "p4-bay-sensor-block-v0": (1, "PETG/PLA", "4+3 双排；针脚向下、气嘴向上"),
    "p4-bay-manifold-clip-v0": (2, "PETG", "主干 Ø20；抬高避让阀轨，打印两只"),
    "p4-bay-switch-bracket-v0": (1, "PETG", "KCD1 卡口壁厚 1.8；XT60 用扎带固定"),
    "p4-bay-cable-comb-v0": (4, "PETG/PLA", "功率线与模拟线分开使用"),
}


def _rounded_rect(width: float, height: float, radius: float):
    """Exact-width rounded rectangle centred at the origin."""
    if radius <= 0:
        return shapely_box(-width / 2, -height / 2, width / 2, height / 2)
    core = shapely_box(
        -width / 2 + radius,
        -height / 2 + radius,
        width / 2 - radius,
        height / 2 - radius,
    )
    return core.buffer(radius, quad_segs=16)


def _slot(x: float, y: float, length: float, width: float, axis: str = "x"):
    """2D capsule slot centred at x,y."""
    straight = max(length - width, 0.01) / 2
    if axis == "x":
        line = LineString([(x - straight, y), (x + straight, y)])
    else:
        line = LineString([(x, y - straight), (x, y + straight)])
    return line.buffer(width / 2, cap_style=1, quad_segs=12)


def _extrude(poly, height: float, z0: float = 0.0) -> trimesh.Trimesh:
    if poly.geom_type == "Polygon":
        mesh = extrude_polygon(poly, height=height)
    else:
        pieces = [
            extrude_polygon(piece, height=height)
            for piece in poly.geoms
            if piece.geom_type == "Polygon" and not piece.is_empty
        ]
        if not pieces:
            raise ValueError(f"cannot extrude empty {poly.geom_type}")
        mesh = _union(pieces)
    if z0:
        mesh.apply_translation((0.0, 0.0, z0))
    return mesh


def _box(ex: float, ey: float, ez: float, at=(0.0, 0.0, 0.0)) -> trimesh.Trimesh:
    mesh = box(extents=(ex, ey, ez))
    mesh.apply_translation(at)
    return mesh


def _cyl(radius: float, height: float, at=(0.0, 0.0, 0.0), axis="z") -> trimesh.Trimesh:
    """Cylinder whose bottom centre is at *at*."""
    mesh = cylinder(radius=radius, height=height, sections=64)
    mesh.apply_translation((0.0, 0.0, height / 2))
    if axis == "x":
        mesh.apply_transform(rotation_matrix(math.pi / 2, (0, 1, 0)))
    elif axis == "y":
        mesh.apply_transform(rotation_matrix(-math.pi / 2, (1, 0, 0)))
    mesh.apply_translation(at)
    return mesh


def _union(meshes) -> trimesh.Trimesh:
    meshes = [mesh for mesh in meshes if mesh is not None]
    if len(meshes) == 1:
        return meshes[0]
    return trimesh.boolean.union(meshes, engine=BOOLEAN_ENGINE)


def _difference(solid: trimesh.Trimesh, cuts) -> trimesh.Trimesh:
    cuts = list(cuts)
    if not cuts:
        return solid
    return trimesh.boolean.difference([solid, *cuts], engine=BOOLEAN_ENGINE)


def _module_holes():
    """Shared M3 hole array on the two solid side rails of the baseplate."""
    holes = []
    for sx in (-1, 1):
        for x_abs in (45.0, 65.0, 85.0):
            for y in (-48.0, -24.0, 0.0, 24.0, 48.0):
                # (±45, 0) would merge into the extra frame attachment at ±44.
                if x_abs == 45.0 and y == 0.0:
                    continue
                holes.append((sx * x_abs, y))
    return holes


def _plate_polygon(width, height, radius, holes, opening=None):
    poly = _rounded_rect(width, height, radius)
    cuts = [Point(x, y).buffer(d / 2, quad_segs=16) for x, y, d in holes]
    if opening is not None:
        cuts.append(_rounded_rect(*opening))
    if cuts:
        poly = poly.difference(unary_union(cuts))
    return poly


def bay_fit_template() -> trimesh.Trimesh:
    base = BAY["base"]
    width, height, _ = base["size"]
    holes = [(x, y, 3.4) for x, y in base["frame_holes"]]
    poly = _plate_polygon(
        width, height, base["corner_r"], holes, opening=base["opening"]
    )
    return _extrude(poly, 1.0)


def bay_baseplate() -> trimesh.Trimesh:
    cfg = BAY["base"]
    width, height, thickness = cfg["size"]
    all_holes = list(cfg["frame_holes"]) + _module_holes()
    hole_specs = [(x, y, cfg["module_hole_d"]) for x, y in all_holes]

    plate_poly = _plate_polygon(
        width, height, cfg["corner_r"], hole_specs, opening=cfg["opening"]
    )
    plate = _extrude(plate_poly, thickness)

    outer = _rounded_rect(width, height, cfg["corner_r"])
    perimeter_rib = outer.difference(_rounded_rect(width - 8.0, height - 8.0, 4.0))
    opening_w, opening_h, opening_r = cfg["opening"]
    opening_rib = _rounded_rect(opening_w + 8.0, opening_h + 8.0,
                                opening_r + 4.0).difference(
        _rounded_rect(opening_w, opening_h, opening_r)
    )
    hole_cuts = unary_union([
        Point(x, y).buffer(cfg["module_hole_d"] / 2, quad_segs=16)
        for x, y in all_holes
    ])
    rib_poly = unary_union([perimeter_rib, opening_rib]).difference(hole_cuts)
    ribs = _extrude(rib_poly, cfg["rib_h"], z0=thickness)
    return _union([plate, ribs])


def electrical_deck() -> trimesh.Trimesh:
    deck = BAY["deck"]
    width, height, thickness = deck["size"]
    holes = [(x, y, 3.4) for x, y in deck["post_holes"]]
    for board in BAY["boards"].values():
        cx, cy = board["center"]
        for hx, hy in board["holes"]:
            holes.append((cx + hx, cy + hy, board["hole_d"]))

    # Three cable pass-throughs: Pi/Servo2040, sensor ribbon and power loom.
    slots = [
        _slot(-5.0, 48.0, 18.0, 5.0, "x"),
        _slot(0.0, -48.0, 18.0, 5.0, "x"),
        _slot(88.0, 24.0, 16.0, 4.0, "y"),
    ]
    poly = _plate_polygon(width, height, deck["corner_r"], holes)
    poly = poly.difference(unary_union(slots))
    return _extrude(poly, thickness)


def spacer(outer_d: float, inner_d: float, height: float) -> trimesh.Trimesh:
    ring = Point(0, 0).buffer(outer_d / 2, quad_segs=24).difference(
        Point(0, 0).buffer(inner_d / 2, quad_segs=24)
    )
    return _extrude(ring, height)


def pump_adapter() -> trimesh.Trimesh:
    cfg = BAY["pump"]
    width, height, thickness = cfg["adapter"]
    hx, hy = cfg["bracket_pitch"][0] / 2, cfg["bracket_pitch"][1] / 2
    cuts = [Point(x, y).buffer(cfg["bracket_hole_d"] / 2, quad_segs=16)
            for x in (-hx, hx) for y in (-hy, hy)]
    # At the nominal V0 location (-52, 22), these four slots land on the
    # left-side rail holes x=-65/-45 and y=0/48.
    cuts.extend([
        _slot(x, y, 10.0, 3.4, "y")
        for x in (-13.0, 7.0) for y in (-17.0, 21.0)
    ])
    poly = _rounded_rect(width, height, 4.0).difference(unary_union(cuts))
    return _extrude(poly, thickness)


def pump_pad_tpu() -> trimesh.Trimesh:
    cfg = BAY["pump"]
    width, height, thickness = cfg["pad"]
    hx, hy = cfg["bracket_pitch"][0] / 2, cfg["bracket_pitch"][1] / 2
    holes = [(x, y, cfg["bracket_hole_d"]) for x in (-hx, hx) for y in (-hy, hy)]
    poly = _plate_polygon(width, height, 3.0, holes)
    return _extrude(poly, thickness)


def valve_rail() -> trimesh.Trimesh:
    cfg = BAY["valve"]
    count, pitch = cfg["count"], cfg["pitch"]
    centres = [(i - (count - 1) / 2) * pitch for i in range(count)]
    base_w, base_h, base_t = 154.0, 34.0, 3.0

    cuts = []
    for cx in centres:
        cuts.append(translate(_rounded_rect(14.0, 18.0, 2.0), xoff=cx))
        # One zip tie goes under the base and over each valve, offset from top port.
        cuts.extend([_slot(cx - 5.0, y, 6.0, 2.5, "x") for y in (-13.5, 13.5)])
    cuts.extend([_slot(x, 0.0, 14.0, 3.4, "x") for x in (-69.0, 69.0)])
    base_poly = _rounded_rect(base_w, base_h, 3.0).difference(unary_union(cuts))
    base = _extrude(base_poly, base_t)

    locators = []
    for cx in centres:
        for sx in (-1, 1):
            for sy in (-1, 1):
                locators.append(_box(
                    1.6, 3.0, 7.0,
                    (cx + sx * 11.4, sy * 14.0, base_t + 3.5),
                ))
    return _union([base, *locators])


def sensor_block() -> trimesh.Trimesh:
    cfg = BAY["sensor"]
    pitch = cfg["pitch"]
    unshifted = [
        ((i - 1.5) * pitch, pitch / 2) for i in range(4)
    ] + [
        ((i - 1.0) * pitch, -pitch / 2) for i in range(3)
    ]
    centres = [(x - 8.0, y) for x, y in unshifted]
    base_w, base_h, base_t = 104.0, 64.0, 3.0
    cuts = [
        translate(_rounded_rect(13.0, 13.0, 1.5), xoff=cx, yoff=cy)
        for cx, cy in centres
    ]
    # Nominal base position (45,22): four slots match x=45/65 and y=0/48.
    cuts.extend([
        _slot(x, y, 10.0, 3.4, "y")
        for x in (0.0, 20.0) for y in (-17.0, 21.0)
    ])
    # The two right deck posts pass through Ø11 clearance holes and still seat
    # directly on the baseplate, so this holder cannot tilt the deck.
    cuts.extend([
        Point(40.0, y).buffer(5.5, quad_segs=20) for y in (-22.0, 26.0)
    ])
    base_outline = translate(_rounded_rect(base_w, base_h, 3.0), xoff=-5.0)
    base_poly = base_outline.difference(unary_union(cuts))
    base = _extrude(base_poly, base_t)

    locators = []
    for cx, cy in centres:
        for sx in (-1, 1):
            for sy in (-1, 1):
                locators.append(_box(
                    1.6, 1.6, 8.0,
                    (cx + sx * 10.3, cy + sy * 10.3, base_t + 4.0),
                ))
    return _union([base, *locators])


def manifold_clip() -> trimesh.Trimesh:
    cfg = BAY["manifold"]
    inner_r = cfg["clip_gap_d"] / 2
    outer_r = inner_r + 3.0
    # The Ø20 manifold sits above the valve bodies.  At the recommended clip
    # placement y=-16, its axis is z=34 and the valve rail occupies z<=19.
    centre_z = 34.0
    ring_outer = _cyl(outer_r, 8.0, at=(-4.0, 0.0, centre_z), axis="x")
    ring_inner = _cyl(inner_r, 10.0, at=(-5.0, 0.0, centre_z), axis="x")
    ring = _difference(ring_outer, [ring_inner])
    gap = _box(10.0, 12.0, 18.0, (0.0, 0.0, centre_z + 14.0))
    ring = _difference(ring, [gap])

    base_poly = _rounded_rect(30.0, 25.0, 3.0).difference(unary_union([
        Point(-10.0, -6.0).buffer(1.7, quad_segs=16),
        Point(10.0, -6.0).buffer(1.7, quad_segs=16),
    ]))
    base = _extrude(base_poly, 3.0)
    uprights = [
        _box(8.0, 3.0, 31.0, (0.0, sy * 10.5, 18.5)) for sy in (-1, 1)
    ]
    return _union([base, ring, *uprights])


def switch_bracket() -> trimesh.Trimesh:
    cfg = BAY["switch"]
    base_poly = _rounded_rect(50.0, 20.0, 3.0).difference(unary_union([
        _slot(-17.0, 0.0, 8.0, 3.4, "y"),
        _slot(17.0, 0.0, 8.0, 3.4, "y"),
    ]))
    base = _extrude(base_poly, 3.0)

    wall_t = cfg["wall_t"]
    wall = _box(50.0, wall_t, 30.0, (0.0, 10.0 - wall_t / 2, 18.0))
    cut_w, cut_h = cfg["cutout"]
    cuts = [
        _box(cut_w, wall_t + 2.0, cut_h, (-10.0, 10.0 - wall_t / 2, 18.0)),
        _box(3.5, wall_t + 2.0, 11.0, (12.0, 10.0 - wall_t / 2, 18.0)),
        _box(3.5, wall_t + 2.0, 11.0, (21.0, 10.0 - wall_t / 2, 18.0)),
    ]
    wall = _difference(wall, cuts)
    gussets = [
        _box(3.0, 8.0, 12.0, (sx * 23.5, 6.0, 9.0)) for sx in (-1, 1)
    ]
    return _union([base, wall, *gussets])


def cable_comb() -> trimesh.Trimesh:
    base_poly = _rounded_rect(48.0, 14.0, 2.5).difference(unary_union([
        Point(-18.0, -4.5).buffer(1.7, quad_segs=16),
        Point(18.0, -4.5).buffer(1.7, quad_segs=16),
    ]))
    base = _extrude(base_poly, 3.0)
    wall = _box(48.0, 4.0, 13.0, (0.0, 4.5, 9.5))
    notches = [
        _box(5.5, 6.0, 10.0, (x, 4.5, 13.0))
        for x in (-17.0, -8.5, 0.0, 8.5, 17.0)
    ]
    wall = _difference(wall, notches)
    return _union([base, wall])


def _absolute_board_holes():
    result = []
    for name, board in BAY["boards"].items():
        cx, cy = board["center"]
        for hx, hy in board["holes"]:
            result.append((name, cx + hx, cy + hy, board["hole_d"]))
    return result


def _validate_layout():
    deck_w, deck_h, _ = BAY["deck"]["size"]
    errors = []
    for name, board in BAY["boards"].items():
        cx, cy = board["center"]
        width, height = board["size"]
        margin = min(deck_w / 2 - abs(cx) - width / 2,
                     deck_h / 2 - abs(cy) - height / 2)
        if margin < 0:
            errors.append(f"{name} exceeds deck by {-margin:.1f} mm")

    boards = list(BAY["boards"].items())
    for i, (name_a, board_a) in enumerate(boards):
        ax, ay = board_a["center"]
        aw, ah = board_a["size"]
        for name_b, board_b in boards[i + 1:]:
            if board_a["side"] != board_b["side"]:
                continue
            bx, by = board_b["center"]
            bw, bh = board_b["size"]
            gap_x = abs(ax - bx) - (aw + bw) / 2
            gap_y = abs(ay - by) - (ah + bh) / 2
            # Rectangles are safely separated when either axis has >=2 mm gap.
            if max(gap_x, gap_y) < 2.0:
                errors.append(
                    f"same-side PCB clearance <2 mm: {name_a}/{name_b} "
                    f"(x={gap_x:.1f}, y={gap_y:.1f})"
                )

    all_holes = _absolute_board_holes()
    all_holes += [("deck_post", x, y, 3.4) for x, y in BAY["deck"]["post_holes"]]
    for i, (name_a, xa, ya, da) in enumerate(all_holes):
        for name_b, xb, yb, db in all_holes[i + 1:]:
            distance = math.hypot(xa - xb, ya - yb)
            if distance < (da + db) / 2 + 3.0:
                errors.append(
                    f"holes too close: {name_a}/{name_b}, centre distance {distance:.2f} mm"
                )
    if errors:
        raise ValueError("Invalid electrical-deck layout:\n  " + "\n  ".join(errors))


def _clean(mesh: trimesh.Trimesh) -> trimesh.Trimesh:
    if isinstance(mesh, trimesh.Scene):
        mesh = mesh.dump(concatenate=True)
    # Manifold already returns indexed, oriented solids.  trimesh's validation
    # merge can collapse deliberately coincident grid-wall edges into non-manifold
    # edges, so only remove unreachable vertices here.
    mesh.remove_unreferenced_vertices()
    trimesh.repair.fix_normals(mesh)
    if not mesh.is_watertight:
        raise ValueError("generated mesh is not watertight")
    if not np.isfinite(mesh.vertices).all() or mesh.volume <= 0:
        raise ValueError("generated mesh is invalid or has non-positive volume")
    return mesh


def _write_reports(records):
    fit_volume = records["p4-bay-fit-template-v0"]["volume_cm3_each"]
    print_set_volume = sum(
        record["volume_cm3_each"] * record["quantity"]
        for record in records.values()
    ) - fit_volume
    tpu_volume = (
        records["p4-bay-pump-pad-tpu-v0"]["volume_cm3_each"]
        * records["p4-bay-pump-pad-tpu-v0"]["quantity"]
    )
    solid_mass_upper = (print_set_volume - tpu_volume) * 1.27 + tpu_volume * 1.21
    layout = {
        "units": "mm",
        "coordinate_system": "+Y=head, deck origin=center",
        "frame_attachment_holes": BAY["base"]["frame_holes"],
        "base_module_holes": _module_holes(),
        "recommended_base_layout": {
            "valve_rail_center": [0.0, -48.0, 3.0],
            "manifold_axis": [0.0, -18.0, 37.0],
            "manifold_clip_centers": [[-55.0, -18.0], [55.0, -18.0]],
            "pump_adapter_center": [-52.0, 22.0, 3.0],
            "sensor_block_center": [45.0, 22.0, 3.0],
        },
        "electrical_deck_assembled_z": {
            "baseplate_top": 3.0,
            "post_length": 90.0,
            "deck_bottom": 93.0,
            "deck_top": 96.0,
        },
        "electrical_deck": {
            name: {
                "side": board["side"],
                "center": board["center"],
                "board_size": board["size"],
                "absolute_holes": [
                    [board["center"][0] + hx, board["center"][1] + hy, board["hole_d"]]
                    for hx, hy in board["holes"]
                ],
            }
            for name, board in BAY["boards"].items()
        },
        "parts": records,
        "print_set_estimate": {
            "excludes_fit_template": True,
            "geometric_solid_volume_cm3": round(print_set_volume, 2),
            "all_solid_mass_upper_bound_g": round(solid_mass_upper, 1),
            "density_assumptions_g_cm3": {"PETG_or_PLA": 1.27, "TPU": 1.21},
            "warning": "Slicer mass depends on walls/infill; weigh the actual V0 before climb use.",
        },
        "v0_limits": [
            "Print and physically verify the 1 mm fit template before the baseplate.",
            "Tank cradle omitted until the tank diameter, length and ear spacing are measured.",
            "V0 is single-pump; a second pump uses another identical adapter after mass review.",
            "Frame needs two added M3 self-tap points at (±44,0); drill pilot Ø2.8 mm only after fit check.",
        ],
    }
    with (OUT / "layout-report.json").open("w", encoding="utf-8") as handle:
        json.dump(layout, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def main():
    _validate_layout()
    OUT.mkdir(parents=True, exist_ok=True)
    parts = {
        "p4-bay-fit-template-v0": bay_fit_template,
        "p4-bay-baseplate-v0": bay_baseplate,
        "p4-bay-electrical-deck-v0": electrical_deck,
        "p4-bay-deck-post-90mm-v0": lambda: spacer(9.0, 3.4, 90.0),
        "p4-bay-spacer-m25-6mm-v0": lambda: spacer(7.0, 2.8, 6.0),
        "p4-bay-spacer-m3-6mm-v0": lambda: spacer(7.0, 3.4, 6.0),
        "p4-bay-pump-adapter-v0": pump_adapter,
        "p4-bay-pump-pad-tpu-v0": pump_pad_tpu,
        "p4-bay-valve-rail-v0": valve_rail,
        "p4-bay-sensor-block-v0": sensor_block,
        "p4-bay-manifold-clip-v0": manifold_clip,
        "p4-bay-switch-bracket-v0": switch_bracket,
        "p4-bay-cable-comb-v0": cable_comb,
    }

    records = {}
    for name, function in parts.items():
        mesh = _clean(function())
        path = OUT / f"{name}.stl"
        mesh.export(path)
        extents = np.round(mesh.bounding_box.extents, 3).tolist()
        quantity, material, note = PART_META[name]
        records[name] = {
            "file": path.name,
            "quantity": quantity,
            "material": material,
            "note": note,
            "watertight": bool(mesh.is_watertight),
            "extents_mm": extents,
            "volume_cm3_each": round(float(mesh.volume) / 1000.0, 3),
        }
        print(
            f"{name:36s} watertight={mesh.is_watertight!s:5s}  "
            f"size={extents[0]:7.2f}×{extents[1]:7.2f}×{extents[2]:7.2f} mm  "
            f"volume={mesh.volume / 1000:7.2f} cm³"
        )
    _write_reports(records)
    print(f"\nGenerated {len(parts)} V0 STL files in {OUT}")


if __name__ == "__main__":
    main()
