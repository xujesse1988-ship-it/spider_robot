#!/usr/bin/env python3
"""Generate the collision-checked P4 removable bay V1.

V1 preserves the 190 x 130 mm frame envelope but replaces the V0 lower
layout and electrical-deck hole pattern.  The important architectural changes
are:

* two pump positions at x=+-46 mm;
* a raised 2+2+2+1 sensor bridge over the frame opening;
* vertically mounted valves with both plastic ports clear of the baseplate;
* manifold clips mounted on the valve rail, between the two pumps;
* an electrical deck with real terminal service corridors and open-edge cable
  exits rather than connector-impossible closed slots.

Run::

    venv/bin/python tools/generate_p4_bay_v1.py

Then independently validate the exported files with::

    venv/bin/python tools/validate_p4_bay_v1.py
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import trimesh
from shapely.affinity import rotate as rotate_shape
from shapely.affinity import translate as translate_shape
from shapely.geometry import Point, box as shapely_box
from shapely.ops import unary_union
from trimesh.transformations import rotation_matrix

import generate_p4_bay_v0 as core


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "hardware" / "climbing-parts" / "p4-bay-v1"
BOOLEAN_ENGINE = "manifold"

BASE_SIZE = (190.0, 130.0, 4.0)
BASE_OPENING = (70.0, 108.0, 14.0)
FRAME_HOLES = [(-44.0, -40.0), (44.0, -40.0),
               (-44.0, 40.0), (44.0, 40.0)]
POST_HOLES = [(-85.0, -48.0), (85.0, -48.0),
              (-85.0, 0.0), (85.0, 0.0),
              (-85.0, 48.0), (85.0, 48.0)]
PUMP_HOLES = [(-60.0, 0.0), (-40.0, 0.0),
              (-60.0, 48.0), (-40.0, 48.0),
              (40.0, 0.0), (60.0, 0.0),
              (40.0, 48.0), (60.0, 48.0)]
VALVE_HOLES = [(-72.0, -56.0), (-72.0, -36.0),
               (72.0, -56.0), (72.0, -36.0)]
SENSOR_HOLES = [(0.0, -9.5), (0.0, 12.0), (0.0, 33.5)]
SWITCH_HOLES = [(-18.0, 60.0), (18.0, 60.0)]
COMB_HOLES = [(-87.0, -36.5), (-87.0, -12.5),
              (-87.0, 12.5), (-87.0, 36.5),
              (87.0, -36.75), (87.0, -12.75),
              (87.0, 12.75), (87.0, 36.75)]
BASE_MODULE_HOLES = POST_HOLES + PUMP_HOLES + VALVE_HOLES + SENSOR_HOLES + SWITCH_HOLES + COMB_HOLES
VALVE_RAIL_CLIP_HOLES = [(-25.0, -52.5), (-9.0, -52.5),
                         (9.0, -52.5), (25.0, -52.5)]

DECK_SIZE = (184.0, 108.0, 3.0)
DECK_POST_LENGTH = 100.0


def _rotate_xy(point: tuple[float, float], degrees: float) -> tuple[float, float]:
    angle = math.radians(degrees)
    x, y = point
    return (x * math.cos(angle) - y * math.sin(angle),
            x * math.sin(angle) + y * math.cos(angle))


def _board(*, side: str, center: tuple[float, float], source_size: tuple[float, float],
           holes: list[tuple[float, float]], hole_d: float, rotation_deg: float,
           ports: dict[str, str]) -> dict:
    rotated_holes = [_rotate_xy(hole, rotation_deg) for hole in holes]
    if int(abs(rotation_deg)) % 180 == 90:
        envelope = (source_size[1], source_size[0])
    else:
        envelope = source_size
    return {
        "side": side,
        "center": center,
        "source_size": source_size,
        "size": envelope,
        "rotation_deg": rotation_deg,
        "hole_d": hole_d,
        "holes": rotated_holes,
        "ports": ports,
    }


BOARDS = {
    "divider": _board(
        side="top", center=(-52.0, 0.0), source_size=(91.0, 70.0),
        holes=[(-41.5, -29.0), (41.5, -29.0), (-41.5, 29.0), (41.5, 29.0)],
        hole_d=3.6, rotation_deg=90.0,
        ports={"sensor_JST": "vertical; route to left open edge", "Pi_IDC": "+Y"},
    ),
    "pi5": _board(
        side="top", center=(15.0, 8.0), source_size=(85.0, 56.0),
        holes=[(-29.0, -24.5), (29.0, -24.5), (-29.0, 24.5), (29.0, 24.5)],
        hole_d=3.0, rotation_deg=90.0,
        ports={"GPIO": "-X", "USB_A_Ethernet": "+Y open edge", "USB_C_power": "+X"},
    ),
    "buck_5v": _board(
        side="top", center=(70.0, 2.0), source_size=(46.0, 24.0),
        holes=[(-20.5, -9.5), (20.5, -9.5), (-20.5, 9.5), (20.5, 9.5)],
        hole_d=3.6, rotation_deg=90.0,
        ports={"input": "-Y", "USB_C_output": "+Y"},
    ),
    "yynmos8": _board(
        side="bottom", center=(43.0, 24.0), source_size=(93.0, 54.0),
        holes=[(-43.5, 3.0), (43.5, 3.0), (-43.5, 12.0), (43.5, 14.5)],
        hole_d=3.6, rotation_deg=0.0,
        ports={"signal_IN": "+Y open edge", "power_OUT_DC": "-Y 20 mm corridor"},
    ),
    "relay": _board(
        side="bottom", center=(-61.0, 15.0), source_size=(72.0, 40.0),
        holes=[(-33.25, -17.25), (33.25, -17.25), (-33.25, 17.25), (33.25, 17.25)],
        hole_d=3.6, rotation_deg=90.0,
        ports={"30A_power": "-Y", "coil_control": "+Y open edge"},
    ),
    "xl6009_pump": _board(
        side="bottom", center=(-14.0, -37.0), source_size=(50.0, 28.0),
        holes=[(-21.5, -10.5), (21.5, -10.5), (-21.5, 10.5), (21.5, 10.5)],
        hole_d=3.6, rotation_deg=180.0,
        ports={"OUT_pump_rail": "-X", "IN_battery": "+X shared 20 mm corridor"},
    ),
    "xl6009_valve": _board(
        side="bottom", center=(56.0, -37.0), source_size=(50.0, 28.0),
        holes=[(-21.5, -10.5), (21.5, -10.5), (-21.5, 10.5), (21.5, 10.5)],
        hole_d=3.6, rotation_deg=0.0,
        ports={"IN_battery": "-X shared 20 mm corridor", "OUT_valve_rail": "+X open edge"},
    ),
}


PART_META = {
    "p4-bay-fit-template-v1": (1, "PLA/PETG", "first print; verifies frame envelope only"),
    "p4-bay-baseplate-v1": (1, "PETG", "flat 4 mm plate; no raised V0 opening rib"),
    "p4-bay-electrical-deck-v1": (1, "PETG", "re-laid for terminal service corridors"),
    "p4-bay-deck-post-100mm-v1": (6, "PETG trial", "use purchased M3 standoffs for climb service"),
    "p4-bay-spacer-m25-6mm-v1": (4, "PETG/PLA", "Pi 5; enlarged Ø3.0 trial hole"),
    "p4-bay-spacer-m3-6mm-v1": (24, "PETG/PLA", "six non-Pi PCBs; four spacers each"),
    "p4-bay-pump-adapter-v1": (2, "PETG", "same part; right instance rotates 180 degrees"),
    "p4-bay-pump-pad-tpu-v1": (2, "TPU 95A", "between metal bracket and adapter"),
    "p4-bay-valve-rail-v1": (1, "PETG", "six vertical valves; side nozzle faces -Y"),
    "p4-bay-sensor-bridge-v1": (1, "PETG", "raised 2+2+2+1 bridge over frame opening"),
    "p4-bay-manifold-clip-v1": (2, "PETG", "mounts on valve-rail front pads; support required"),
    "p4-bay-switch-bracket-v1": (1, "PETG", "front-edge KCD1 plus XT60/fuse tie slots"),
    "p4-bay-cable-comb-v1": (4, "PETG/PLA", "two signal-left and two power-right"),
}


def _hole_union(points: list[tuple[float, float]], diameter: float):
    return unary_union([Point(x, y).buffer(diameter / 2.0, quad_segs=20) for x, y in points])


def fit_template() -> trimesh.Trimesh:
    holes = [(x, y, 3.6) for x, y in FRAME_HOLES]
    poly = core._plate_polygon(190.0, 130.0, 8.0, holes, opening=BASE_OPENING)
    return core._extrude(poly, 1.0)


def baseplate() -> trimesh.Trimesh:
    outer = core._rounded_rect(190.0, 130.0, 8.0)
    opening = core._rounded_rect(*BASE_OPENING)
    # Three flush, same-thickness bridges support the raised sensor holder.
    # Extend 2 mm into the surrounding ring on each side.  Ending exactly at
    # x=+-35 would create a zero-area/tangent join at the rounded opening and
    # some STL triangulators would emit a non-manifold seam.
    bridges = unary_union([
        shapely_box(-37.0, -12.5, 37.0, -6.5),
        shapely_box(-37.0, 9.0, 37.0, 15.0),
        shapely_box(-37.0, 31.0, 37.0, 36.0),
    ])
    poly = outer.difference(opening).union(bridges)
    all_holes = FRAME_HOLES + BASE_MODULE_HOLES
    poly = poly.difference(_hole_union(all_holes, 3.8))
    return core._extrude(poly, BASE_SIZE[2])


def electrical_deck() -> trimesh.Trimesh:
    holes = [(x, y, 3.8) for x, y in POST_HOLES]
    for board in BOARDS.values():
        cx, cy = board["center"]
        for hx, hy in board["holes"]:
            holes.append((cx + hx, cy + hy, board["hole_d"]))
    poly = core._plate_polygon(DECK_SIZE[0], DECK_SIZE[1], 6.0, holes)
    # Open-edge notches: finished USB, left analog loom and right power loom.
    notches = unary_union([
        shapely_box(28.0, 45.0, 46.0, 55.0),
        shapely_box(-93.0, 20.0, -82.0, 36.0),
        shapely_box(83.0, -34.0, 93.0, -22.0),
    ])
    return core._extrude(poly.difference(notches), DECK_SIZE[2])


def pump_adapter() -> trimesh.Trimesh:
    cuts = [Point(x, y).buffer(2.8, quad_segs=20)
            for x in (-24.0, 24.0) for y in (-14.0, 14.0)]
    cuts.extend(core._slot(x, y, 6.0, 3.8, "x")
                for x in (-14.0, 6.0) for y in (-24.0, 24.0))
    poly = core._rounded_rect(64.0, 54.0, 4.0).difference(unary_union(cuts))
    return core._extrude(poly, 4.0)


def pump_pad() -> trimesh.Trimesh:
    holes = [(x, y, 5.6) for x in (-24.0, 24.0) for y in (-14.0, 14.0)]
    poly = core._plate_polygon(56.0, 36.0, 3.0, holes)
    return core._extrude(poly, 3.0)


VALVE_CENTRES_X = [(index - 2.5) * 23.5 for index in range(6)]


def valve_rail() -> trimesh.Trimesh:
    base_w, base_h, base_t = 154.0, 32.0, 4.0
    cuts = [Point(x, y).buffer(1.9, quad_segs=20) for x, y in [
        (-72.0, -11.5), (-72.0, 8.5), (72.0, -11.5), (72.0, 8.5),
        (-25.0, -8.0), (-9.0, -8.0), (9.0, -8.0), (25.0, -8.0),
    ]]
    for cx in VALVE_CENTRES_X:
        cuts.extend([core._slot(cx - 5.0, y, 6.0, 2.8, "x") for y in (-12.5, 12.5)])
    base = core._extrude(core._rounded_rect(base_w, base_h, 3.0).difference(unary_union(cuts)), base_t)

    supports: list[trimesh.Trimesh] = []
    seat_z = 23.0
    for cx in VALVE_CENTRES_X:
        # Two side columns and ledges support the metal body while leaving the
        # central/downward and -Y side plastic ports unobstructed.
        for sx in (-1.0, 1.0):
            # Sink the column 0.2 mm into the base so the export is one
            # manifold body rather than solids which merely share a face.
            supports.append(core._box(2.6, 6.0, seat_z - base_t + 0.2,
                                      (cx + sx * 9.2, 3.0, (seat_z + base_t - 0.2) / 2.0)))
            supports.append(core._box(3.2, 12.0, 2.0,
                                      (cx + sx * 9.0, 0.0, seat_z - 1.0)))
        # The 20 mm stop overlaps both ledges in X and reaches their rear edge
        # in Y; this makes it structural while leaving the -Y nozzle open.
        supports.append(core._box(20.0, 5.5, 8.0, (cx, 7.25, seat_z + 3.9)))
    return core._union([base, *supports])


SENSOR_CENTRES = [
    (-10.75, -32.25), (10.75, -32.25),
    (-10.75, -10.75), (10.75, -10.75),
    (-10.75, 10.75), (10.75, 10.75),
    (0.0, 32.25),
]


def sensor_bridge() -> trimesh.Trimesh:
    plate_bottom, plate_t = 12.0, 3.0
    outline = core._rounded_rect(43.2, 90.0, 3.0)
    windows = [translate_shape(core._rounded_rect(15.0, 15.0, 1.5), xoff=x, yoff=y)
               for x, y in SENSOR_CENTRES]
    mount_holes = [Point(0.0, y).buffer(1.9, quad_segs=20) for y in (-21.5, 0.0, 21.5)]
    plate = core._extrude(outline.difference(unary_union(windows + mount_holes)),
                          plate_t, z0=plate_bottom)

    feet = []
    for y in (-21.5, 0.0, 21.5):
        foot_poly = core._rounded_rect(22.0, 5.0, 1.5).difference(
            Point(0.0, 0.0).buffer(1.9, quad_segs=20)
        )
        feet.append(core._extrude(translate_shape(foot_poly, yoff=y), plate_bottom + 0.2))

    locators = []
    for cx, cy in SENSOR_CENTRES:
        # Two side tabs locate each 19 mm sensor.  V0-style four-corner tabs
        # merged directly above the bridge's three M3 holes on the inner row.
        for sx in (-1.0, 1.0):
            locators.append(core._box(
                1.5, 3.0, 8.2,
                (cx + sx * 10.55, cy, plate_bottom + plate_t + 3.9),
            ))
    return core._union([plate, *feet, *locators])


def _beam_between(start: tuple[float, float, float], end: tuple[float, float, float],
                  radius: float) -> trimesh.Trimesh:
    start_v = np.asarray(start, dtype=float)
    end_v = np.asarray(end, dtype=float)
    vector = end_v - start_v
    length = float(np.linalg.norm(vector))
    beam = trimesh.creation.cylinder(radius=radius, height=length, sections=40)
    align = trimesh.geometry.align_vectors((0.0, 0.0, 1.0), vector / length)
    beam.apply_transform(align)
    beam.apply_translation((start_v + end_v) / 2.0)
    return beam


def manifold_clip() -> trimesh.Trimesh:
    inner_r, outer_r = 10.3, 12.3
    ring = core._difference(
        core._cyl(outer_r, 8.0, at=(-4.0, 3.0, 57.0), axis="x"),
        [core._cyl(inner_r, 10.0, at=(-5.0, 3.0, 57.0), axis="x")],
    )
    ring = core._difference(ring, [core._box(10.0, 12.0, 18.0, (0.0, 3.0, 70.0))])
    base_poly = core._rounded_rect(24.0, 6.0, 2.0).difference(unary_union([
        Point(-8.0, 0.0).buffer(1.9, quad_segs=20),
        Point(8.0, 0.0).buffer(1.9, quad_segs=20),
    ]))
    base = core._extrude(base_poly, 4.0)
    # Rear-biased diagonal stem stays behind the raised sensor bridge.
    stem = _beam_between((0.0, -2.0, 4.0), (0.0, -8.5, 57.0), 2.6)
    return core._union([base, stem, ring])


def switch_bracket() -> trimesh.Trimesh:
    base_poly = core._rounded_rect(50.0, 10.0, 2.5).difference(unary_union([
        Point(-18.0, 0.0).buffer(1.9, quad_segs=20),
        Point(18.0, 0.0).buffer(1.9, quad_segs=20),
    ]))
    base = core._extrude(base_poly, 4.0)
    wall_t = 2.0
    wall = core._box(50.0, wall_t, 32.0, (0.0, 5.0 - wall_t / 2.0, 20.0))
    wall = core._difference(wall, [
        core._box(19.6, 4.0, 13.6, (-10.0, 4.0, 20.0)),
        core._box(3.8, 4.0, 14.0, (10.0, 4.0, 20.0)),
        core._box(3.8, 4.0, 14.0, (20.0, 4.0, 20.0)),
    ])
    gussets = [core._box(3.0, 7.0, 12.0, (sx * 23.0, 1.5, 10.0)) for sx in (-1.0, 1.0)]
    return core._union([base, wall, *gussets])


def cable_comb() -> trimesh.Trimesh:
    base_poly = core._rounded_rect(36.0, 14.0, 2.5).difference(unary_union([
        Point(-12.0, 0.0).buffer(1.9, quad_segs=20),
        Point(12.0, 0.0).buffer(1.9, quad_segs=20),
    ]))
    base = core._extrude(base_poly, 3.0)
    wall = core._box(36.0, 4.0, 13.0, (0.0, 4.5, 9.5))
    notches = [core._box(5.2, 6.0, 10.0, (x, 4.5, 13.0))
               for x in (-14.0, -7.0, 0.0, 7.0, 14.0)]
    return core._union([base, core._difference(wall, notches)])


def _absolute_board_holes() -> list[tuple[str, float, float, float]]:
    result = []
    for name, board in BOARDS.items():
        cx, cy = board["center"]
        result.extend((name, cx + hx, cy + hy, board["hole_d"]) for hx, hy in board["holes"])
    return result


def _validate_deck_holes() -> None:
    holes = _absolute_board_holes() + [("deck_post", x, y, 3.8) for x, y in POST_HOLES]
    errors = []
    for index, (name_a, xa, ya, da) in enumerate(holes):
        for name_b, xb, yb, db in holes[index + 1:]:
            distance = math.hypot(xa - xb, ya - yb)
            if distance < (da + db) / 2.0 + 0.8:
                errors.append(f"{name_a}/{name_b} holes only {distance:.2f} mm apart")
    if errors:
        raise ValueError("electrical deck hole conflict:\n  " + "\n  ".join(errors))


def _placed(part: str, translation: tuple[float, float, float], rotation_z: float = 0.0) -> dict:
    return {
        "id": part,
        "part": part.rsplit("-instance", 1)[0] if part.endswith("-instance") else part,
        "translation_mm": list(translation),
        "rotation_deg_xyz": [0.0, 0.0, rotation_z],
    }


def _part_record(name: str, mesh: trimesh.Trimesh) -> dict:
    quantity, material, note = PART_META[name]
    return {
        "file": f"{name}.stl",
        "quantity": quantity,
        "material": material,
        "note": note,
        "watertight": bool(mesh.is_watertight),
        "extents_mm": np.round(mesh.extents, 3).tolist(),
        "volume_cm3_each": round(float(mesh.volume) / 1000.0, 3),
    }


def _instance(instance_id: str, part: str, xyz: tuple[float, float, float], rz: float = 0.0) -> dict:
    return {"id": instance_id, "part": part, "translation_mm": list(xyz),
            "rotation_deg_xyz": [0.0, 0.0, rz]}


def _validation_metadata() -> dict:
    instances = [
        _instance("baseplate", "p4-bay-baseplate-v1", (0.0, 0.0, 0.0)),
        _instance("pump-left", "p4-bay-pump-adapter-v1", (-46.0, 24.0, 4.0)),
        _instance("pump-right", "p4-bay-pump-adapter-v1", (46.0, 24.0, 4.0), 180.0),
        _instance("pump-pad-left", "p4-bay-pump-pad-tpu-v1", (-46.0, 24.0, 8.0)),
        _instance("pump-pad-right", "p4-bay-pump-pad-tpu-v1", (46.0, 24.0, 8.0), 180.0),
        _instance("valve-rail", "p4-bay-valve-rail-v1", (0.0, -44.5, 4.0)),
        _instance("sensor-bridge", "p4-bay-sensor-bridge-v1", (0.0, 12.0, 4.0)),
        _instance("manifold-clip-left", "p4-bay-manifold-clip-v1", (-17.0, -52.5, 8.0)),
        _instance("manifold-clip-right", "p4-bay-manifold-clip-v1", (17.0, -52.5, 8.0)),
        _instance("electrical-deck", "p4-bay-electrical-deck-v1", (0.0, 0.0, 104.0)),
        _instance("switch-bracket", "p4-bay-switch-bracket-v1", (0.0, 60.0, 4.0)),
        _instance("comb-left-tail", "p4-bay-cable-comb-v1", (-87.0, -24.5, 4.0), -90.0),
        _instance("comb-left-head", "p4-bay-cable-comb-v1", (-87.0, 24.5, 4.0), -90.0),
        _instance("comb-right-tail", "p4-bay-cable-comb-v1", (87.0, -24.75, 4.0), 90.0),
        _instance("comb-right-head", "p4-bay-cable-comb-v1", (87.0, 24.75, 4.0), 90.0),
    ]
    for x in (-85.0, 85.0):
        for y in (-48.0, 0.0, 48.0):
            instances.append(_instance(f"post-{int(x)}-{int(y)}", "p4-bay-deck-post-100mm-v1", (x, y, 4.0)))

    fasteners = [
        {"name": "pump-left", "instance": "pump-left", "base_instance": "baseplate",
         "bolt_centers_world_mm": [[-60, 0], [-40, 0], [-60, 48], [-40, 48]], "screw_diameter_mm": 3.0},
        {"name": "pump-right", "instance": "pump-right", "base_instance": "baseplate",
         "bolt_centers_world_mm": [[40, 0], [60, 0], [40, 48], [60, 48]], "screw_diameter_mm": 3.0},
        {"name": "valve-rail", "instance": "valve-rail", "base_instance": "baseplate",
         "bolt_centers_world_mm": [list(point) for point in VALVE_HOLES], "screw_diameter_mm": 3.0},
        {"name": "sensor-bridge", "instance": "sensor-bridge", "base_instance": "baseplate",
         "bolt_centers_world_mm": [list(point) for point in SENSOR_HOLES], "screw_diameter_mm": 3.0},
        {"name": "switch-bracket", "instance": "switch-bracket", "base_instance": "baseplate",
         "bolt_centers_world_mm": [list(point) for point in SWITCH_HOLES], "screw_diameter_mm": 3.0},
    ]
    for instance_id, points in [
        ("comb-left-tail", [(-87, -36.5), (-87, -12.5)]),
        ("comb-left-head", [(-87, 12.5), (-87, 36.5)]),
        ("comb-right-tail", [(87, -36.75), (87, -12.75)]),
        ("comb-right-head", [(87, 12.75), (87, 36.75)]),
    ]:
        fasteners.append({"name": instance_id, "instance": instance_id, "base_instance": "baseplate",
                          "bolt_centers_world_mm": [list(point) for point in points], "screw_diameter_mm": 3.0})
    for x in (-85, 85):
        for y in (-48, 0, 48):
            post_id = f"post-{x}-{y}"
            fasteners.append({"name": post_id + "-base", "instance": post_id,
                              "base_instance": "baseplate", "bolt_centers_world_mm": [[x, y]],
                              "screw_diameter_mm": 3.0})
            fasteners.append({"name": post_id + "-deck", "instance": post_id,
                              "base_instance": "electrical-deck", "base_hole_set": "deck_post_holes",
                              "bolt_centers_world_mm": [[x, y]], "screw_diameter_mm": 3.0})
    fasteners.extend([
        {"name": "clip-left", "instance": "manifold-clip-left", "base_instance": "valve-rail",
         "base_hole_set": "valve_rail_clip_holes", "bolt_centers_world_mm": [[-25, -52.5], [-9, -52.5]],
         "screw_diameter_mm": 3.0},
        {"name": "clip-right", "instance": "manifold-clip-right", "base_instance": "valve-rail",
         "base_hole_set": "valve_rail_clip_holes", "bolt_centers_world_mm": [[9, -52.5], [25, -52.5]],
         "screw_diameter_mm": 3.0},
    ])

    service_zones = [
        {"name": "MOS power terminals", "owner": "yynmos8", "side": "bottom", "center_mm": [43, -13], "size_mm": [93, 20]},
        {"name": "MOS signal ribbon", "owner": "yynmos8", "side": "bottom", "center_mm": [43, 56], "size_mm": [93, 10], "minimum_depth_mm": 10, "allow_beyond_deck": True},
        {"name": "relay 30A terminals", "owner": "relay", "side": "bottom", "center_mm": [-61, -31], "size_mm": [40, 20]},
        {"name": "relay control", "owner": "relay", "side": "bottom", "center_mm": [-61, 56], "size_mm": [40, 10], "minimum_depth_mm": 10, "allow_beyond_deck": True},
        {"name": "pump XL output", "owner": "xl6009_pump", "side": "bottom", "center_mm": [-49, -37], "size_mm": [20, 28]},
        {"name": "pump XL input", "owner": "xl6009_pump", "side": "bottom", "center_mm": [21, -37], "size_mm": [20, 28]},
        {"name": "valve XL input", "owner": "xl6009_valve", "side": "bottom", "center_mm": [21, -37], "size_mm": [20, 28]},
        {"name": "valve XL output", "owner": "xl6009_valve", "side": "bottom", "center_mm": [91, -37], "size_mm": [20, 28], "allow_beyond_deck": True},
        {"name": "5V input", "owner": "buck_5v", "side": "top", "center_mm": [70, -28.5], "size_mm": [24, 15], "minimum_depth_mm": 15},
        {"name": "5V USB-C output", "owner": "buck_5v", "side": "top", "center_mm": [70, 32.5], "size_mm": [24, 15], "minimum_depth_mm": 15},
    ]

    probes = []
    for index, (x, y) in enumerate(SENSOR_CENTRES, start=1):
        probes.append({"name": f"sensor pin window {index}", "kind": "sensor_pin",
                       "center_world_mm": [x, y + 12.0], "diameter_mm": 13.0,
                       "z_range_world_mm": [0.0, 19.0],
                       "must_clear_instances": ["baseplate", "sensor-bridge"]})
    for index, x in enumerate(VALVE_CENTRES_X, start=1):
        probes.append({"name": f"valve downward nozzle {index}", "kind": "valve_nozzle",
                       "center_world_mm": [x, -44.5], "diameter_mm": 8.0,
                       "z_range_world_mm": [15.0, 27.0],
                       "must_clear_instances": ["valve-rail"]})

    return {
        "assembly_instances": instances,
        "required_assembly_instance_ids": [item["id"] for item in instances],
        "collision_ignore_pairs": [],
        "collision_volume_tolerance_mm3": 0.05,
        "base_hole_match_tolerance_mm": 0.25,
        "fastener_mounts": fasteners,
        "deck": {
            "instance": "electrical-deck",
            "minimum_board_edge_clearance_mm": 2.0,
            "minimum_same_side_gap_mm": 2.0,
            "minimum_terminal_service_depth_mm": 20.0,
            "maximum_service_zone_owner_gap_mm": 0.25,
            "terminal_service_zones": service_zones,
        },
        "minimum_downward_probe_counts": {"sensor_pin": 7, "valve_nozzle": 6},
        "downward_clearance_probes": probes,
    }


def _write_report(records: dict[str, dict]) -> None:
    layout = {
        "version": "V1",
        "units": "mm",
        "coordinate_system": "+Y=head, origin=base/deck center",
        "frame_attachment_holes": FRAME_HOLES,
        "base_module_holes": BASE_MODULE_HOLES,
        "deck_post_holes": POST_HOLES,
        "valve_rail_clip_holes": VALVE_RAIL_CLIP_HOLES,
        "recommended_base_layout": {
            "pump_adapter_centers": [[-46.0, 24.0, 4.0], [46.0, 24.0, 4.0]],
            "sensor_bridge_center": [0.0, 12.0, 4.0],
            "valve_rail_center": [0.0, -44.5, 4.0],
            "manifold_axis": [0.0, -49.5, 65.0],
            "manifold_clip_centers": [[-17.0, -49.5, 65.0], [17.0, -49.5, 65.0]],
            "switch_bracket_center": [0.0, 60.0, 4.0],
            "signal_combs": [[-87.0, -24.5, 4.0], [-87.0, 24.5, 4.0]],
            "power_combs": [[87.0, -24.75, 4.0], [87.0, 24.75, 4.0]],
        },
        "electrical_deck_assembled_z": {
            "baseplate_top": 4.0,
            "post_length": DECK_POST_LENGTH,
            "deck_bottom": 104.0,
            "deck_top": 107.0,
        },
        "electrical_deck": {
            name: {
                "side": board["side"],
                "center": list(board["center"]),
                "board_size": list(board["size"]),
                "source_board_size": list(board["source_size"]),
                "rotation_deg": board["rotation_deg"],
                "ports": board["ports"],
                "absolute_holes": [[board["center"][0] + hx, board["center"][1] + hy, board["hole_d"]]
                                   for hx, hy in board["holes"]],
            }
            for name, board in BOARDS.items()
        },
        "component_keepouts": {
            "pump_each_conservative": {"size_mm": [62.0, 86.0, 53.0],
                                         "warning": "measure 86 mm body offset from bracket-hole centre before climb"},
            "valve_each_vertical": {"body_mm": [20.0, 16.0, 25.0],
                                     "plastic_nozzle_keepout": "Ø8 x 12; measure actual protrusion"},
            "manifold": {"body_mm": [115.0, 20.0], "axis_world_mm": [0.0, -49.5, 65.0]},
            "pump_filter_cap": {"body_max_mm": [22.0, 13.0], "location": "beside XL6009 pump OUT"},
            "valve_filter_cap": {"body_max_mm": [16.0, 10.0], "location": "beside XL6009 valve OUT"},
        },
        "parts": records,
        "validation": _validation_metadata(),
        "v1_limits": [
            "The fit template still requires a full-leg dynamic sweep before baseplate printing.",
            "Measure pump-body fore/aft offset from its 48x28 bracket-hole centre.",
            "Measure both 0520B plastic nozzle protrusions; V1 uses a conservative 12 mm proxy.",
            "Tank cradle remains deferred until the actual tank and mounting ears arrive.",
            "Manifold clips require slicer support under the C ring.",
        ],
    }
    with (OUT / "layout-report.json").open("w", encoding="utf-8") as handle:
        json.dump(layout, handle, ensure_ascii=False, indent=2)
        handle.write("\n")


def main() -> None:
    _validate_deck_holes()
    OUT.mkdir(parents=True, exist_ok=True)
    functions = {
        "p4-bay-fit-template-v1": fit_template,
        "p4-bay-baseplate-v1": baseplate,
        "p4-bay-electrical-deck-v1": electrical_deck,
        "p4-bay-deck-post-100mm-v1": lambda: core.spacer(9.0, 3.8, DECK_POST_LENGTH),
        "p4-bay-spacer-m25-6mm-v1": lambda: core.spacer(7.0, 3.0, 6.0),
        "p4-bay-spacer-m3-6mm-v1": lambda: core.spacer(7.5, 3.6, 6.0),
        "p4-bay-pump-adapter-v1": pump_adapter,
        "p4-bay-pump-pad-tpu-v1": pump_pad,
        "p4-bay-valve-rail-v1": valve_rail,
        "p4-bay-sensor-bridge-v1": sensor_bridge,
        "p4-bay-manifold-clip-v1": manifold_clip,
        "p4-bay-switch-bracket-v1": switch_bracket,
        "p4-bay-cable-comb-v1": cable_comb,
    }
    records = {}
    for name, function in functions.items():
        mesh = core._clean(function())
        mesh.export(OUT / f"{name}.stl")
        records[name] = _part_record(name, mesh)
        print(f"{name:39s} size={np.round(mesh.extents, 2).tolist()}  volume={mesh.volume / 1000:.2f} cm3")
    _write_report(records)
    print(f"\nGenerated {len(functions)} V1 STL files in {OUT}")


if __name__ == "__main__":
    main()
