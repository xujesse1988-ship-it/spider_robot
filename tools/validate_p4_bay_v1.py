#!/usr/bin/env python3
"""Validate the generated P4 removable electrical/pneumatic bay V1.

This is intentionally independent from ``generate_p4_bay_v1.py``.  It reads
only the exported STL files and ``layout-report.json`` so that a bug in the
generator cannot make the same incorrect assertion in the validator.

Run from any directory with the repository virtual environment::

    venv/bin/python tools/validate_p4_bay_v1.py

Exit status is 0 for a clean validation, 1 for failed checks, and 2 when the
input/report is missing or cannot be parsed.  Use ``--json`` for CI output.

The report should contain a top-level ``validation`` object with:

* ``assembly_instances`` -- positioned STL instances used for collision tests;
* ``fastener_mounts`` -- selected world-space bolt centres shared by a module
  and the baseplate;
* ``deck.terminal_service_zones`` -- reserved terminal/wire-bend rectangles;
* ``downward_clearance_probes`` -- cylindrical pin/nozzle paths which must not
  intersect the baseplate or holder.

See the individual validation functions below for the compact field schema.
All coordinates and volumes are millimetres and cubic millimetres.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import warnings
from dataclasses import asdict, dataclass
from itertools import combinations
from pathlib import Path
from typing import Any, Iterable

try:
    import numpy as np
    import trimesh
except ImportError as exc:  # pragma: no cover - useful when invoked outside venv
    print(
        "ERROR: trimesh and numpy are required; run with venv/bin/python "
        f"({exc})",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUT = ROOT / "hardware" / "climbing-parts" / "p4-bay-v1"
EXPECTED_STL_COUNT = 13
BOOLEAN_ENGINE = "manifold"
DEFAULT_COLLISION_VOLUME_MM3 = 0.05
DEFAULT_VOID_VOLUME_MM3 = 0.02


@dataclass(frozen=True)
class Finding:
    check: str
    status: str
    message: str


class Results:
    def __init__(self) -> None:
        self.findings: list[Finding] = []

    def pass_(self, check: str, message: str) -> None:
        self.findings.append(Finding(check, "PASS", message))

    def fail(self, check: str, message: str) -> None:
        self.findings.append(Finding(check, "FAIL", message))

    @property
    def failed(self) -> bool:
        return any(item.status == "FAIL" for item in self.findings)


@dataclass
class Instance:
    id: str
    part: str
    file: Path
    transform: np.ndarray
    mesh: trimesh.Trimesh


def _as_xyz(value: Any, field: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (3,) or not np.isfinite(array).all():
        raise ValueError(f"{field} must be three finite numbers")
    return array


def _rotation_matrix_xyz(degrees: Iterable[float]) -> np.ndarray:
    angles = _as_xyz(list(degrees), "rotation_deg_xyz")
    result = np.eye(4)
    for angle, axis in zip(np.radians(angles), ((1, 0, 0), (0, 1, 0), (0, 0, 1))):
        result = result @ trimesh.transformations.rotation_matrix(angle, axis)
    return result


def _instance_transform(record: dict[str, Any]) -> np.ndarray:
    if "transform" in record or "transform_matrix" in record:
        raw = record.get("transform", record.get("transform_matrix"))
        matrix = np.asarray(raw, dtype=float)
        if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
            raise ValueError("transform must be a finite 4x4 matrix")
        return matrix

    matrix = _rotation_matrix_xyz(record.get("rotation_deg_xyz", (0.0, 0.0, 0.0)))
    translation = record.get(
        "translation_mm", record.get("translation", record.get("center", (0.0, 0.0, 0.0)))
    )
    matrix[:3, 3] = _as_xyz(translation, "translation_mm")
    return matrix


def _load_stl(path: Path) -> trimesh.Trimesh:
    # STL stores independent triangle vertices.  Normal import processing is
    # required to merge coincident vertices before topology/body checks.
    loaded = trimesh.load_mesh(path, file_type="stl", process=True)
    if isinstance(loaded, trimesh.Scene):
        if not loaded.geometry:
            raise ValueError("empty scene")
        loaded = loaded.to_mesh()
    if not isinstance(loaded, trimesh.Trimesh):
        raise ValueError(f"unsupported mesh type {type(loaded).__name__}")
    loaded.remove_unreferenced_vertices()
    return loaded


def _part_files(report: dict[str, Any], out_dir: Path) -> dict[str, Path]:
    result: dict[str, Path] = {}
    parts = report.get("parts", {})
    if isinstance(parts, dict):
        for name, record in parts.items():
            if isinstance(record, dict) and record.get("file"):
                path = out_dir / str(record["file"])
                result[str(name)] = path
                result[path.name] = path
                result[path.stem] = path
    for path in out_dir.glob("*.stl"):
        result.setdefault(path.name, path)
        result.setdefault(path.stem, path)
    return result


def _mesh_volume(mesh: trimesh.Trimesh | trimesh.Scene | None) -> float:
    if mesh is None:
        return 0.0
    # Manifold can return an explicit zero-volume contact surface for parts
    # which only touch.  Trimesh correctly reports volume zero but its centroid
    # integration divides by that zero and emits an irrelevant RuntimeWarning.
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        if isinstance(mesh, trimesh.Scene):
            if not mesh.geometry:
                return 0.0
            return float(sum(abs(float(item.volume)) for item in mesh.geometry.values()))
        return abs(float(mesh.volume))


def _aabb_overlap_volume(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    low = np.maximum(a.bounds[0], b.bounds[0])
    high = np.minimum(a.bounds[1], b.bounds[1])
    extents = np.maximum(high - low, 0.0)
    return float(np.prod(extents))


def _intersection_volume(a: trimesh.Trimesh, b: trimesh.Trimesh) -> float:
    if _aabb_overlap_volume(a, b) <= 0.0:
        return 0.0
    result = trimesh.boolean.intersection([a, b], engine=BOOLEAN_ENGINE)
    return _mesh_volume(result)


def validate_stls(
    report: dict[str, Any], out_dir: Path, results: Results
) -> dict[Path, trimesh.Trimesh]:
    paths = sorted(out_dir.glob("*.stl"))
    if len(paths) != EXPECTED_STL_COUNT:
        results.fail(
            "stl-count",
            f"expected {EXPECTED_STL_COUNT} STL files, found {len(paths)} in {out_dir}",
        )
    else:
        results.pass_("stl-count", f"found exactly {EXPECTED_STL_COUNT} STL files")

    parts = report.get("parts", {})
    if not isinstance(parts, dict):
        results.fail("stl-manifest", "top-level parts must be an object")
        parts = {}
    declared = {
        str(record.get("file"))
        for record in parts.values()
        if isinstance(record, dict) and record.get("file")
    }
    actual = {path.name for path in paths}
    if declared != actual:
        missing = sorted(declared - actual)
        undeclared = sorted(actual - declared)
        details = []
        if missing:
            details.append("missing: " + ", ".join(missing))
        if undeclared:
            details.append("undeclared: " + ", ".join(undeclared))
        results.fail("stl-manifest", "; ".join(details) or "manifest mismatch")
    else:
        results.pass_("stl-manifest", "report manifest matches the STL directory")

    meshes: dict[Path, trimesh.Trimesh] = {}
    for path in paths:
        try:
            mesh = _load_stl(path)
            meshes[path.resolve()] = mesh
        except Exception as exc:
            results.fail("stl-geometry", f"{path.name}: cannot load ({exc})")
            continue

        problems = []
        if not mesh.is_watertight:
            problems.append("not watertight")
        try:
            body_count = int(mesh.body_count)
        except Exception:
            body_count = len(mesh.split(only_watertight=False))
        if body_count != 1:
            problems.append(f"{body_count} connected bodies")
        volume = float(mesh.volume)
        if not math.isfinite(volume) or volume <= 0.0:
            problems.append(f"non-positive/invalid signed volume {volume!r}")
        if not np.isfinite(mesh.vertices).all():
            problems.append("non-finite vertex coordinates")

        if problems:
            results.fail("stl-geometry", f"{path.name}: " + ", ".join(problems))
        else:
            results.pass_(
                "stl-geometry",
                f"{path.name}: watertight, one body, volume={volume:.2f} mm^3",
            )
    return meshes


def _validation_block(report: dict[str, Any]) -> dict[str, Any]:
    block = report.get("validation", {})
    return block if isinstance(block, dict) else {}


def build_instances(
    report: dict[str, Any],
    out_dir: Path,
    meshes: dict[Path, trimesh.Trimesh],
    results: Results,
) -> dict[str, Instance]:
    validation = _validation_block(report)
    records = validation.get("assembly_instances", report.get("assembly_instances", []))
    if not isinstance(records, list) or not records:
        results.fail(
            "assembly-schema",
            "validation.assembly_instances is required for independent collision checks",
        )
        return {}

    known_files = _part_files(report, out_dir)
    instances: dict[str, Instance] = {}
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            results.fail("assembly-schema", f"assembly instance #{index} is not an object")
            continue
        instance_id = str(record.get("id", "")).strip()
        part = str(record.get("part", record.get("file", ""))).strip()
        if not instance_id or not part:
            results.fail(
                "assembly-schema", f"assembly instance #{index} needs id and part/file"
            )
            continue
        if instance_id in instances:
            results.fail("assembly-schema", f"duplicate assembly instance id {instance_id!r}")
            continue
        path = known_files.get(part)
        if path is None and (out_dir / part).suffix.lower() == ".stl":
            path = out_dir / part
        if path is None or not path.is_file():
            results.fail("assembly-schema", f"{instance_id}: unknown STL part {part!r}")
            continue
        try:
            transform = _instance_transform(record)
            source = meshes.get(path.resolve())
            if source is None:
                source = _load_stl(path)
            placed = source.copy()
            placed.apply_transform(transform)
        except Exception as exc:
            results.fail("assembly-schema", f"{instance_id}: invalid transform/mesh ({exc})")
            continue
        instances[instance_id] = Instance(instance_id, part, path, transform, placed)

    if instances:
        results.pass_("assembly-schema", f"loaded {len(instances)} positioned assembly instances")

    required_ids = validation.get("required_assembly_instance_ids")
    if not isinstance(required_ids, list) or not required_ids:
        results.fail(
            "assembly-schema",
            "validation.required_assembly_instance_ids is required to prevent partial collision coverage",
        )
    else:
        missing_ids = sorted({str(item) for item in required_ids} - set(instances))
        if missing_ids:
            results.fail(
                "assembly-schema",
                "required assembly instances are missing: " + ", ".join(missing_ids),
            )
        else:
            results.pass_(
                "assembly-schema",
                f"all {len(set(map(str, required_ids)))} required instances are positioned",
            )
    return instances


def validate_collisions(
    report: dict[str, Any], instances: dict[str, Instance], results: Results
) -> None:
    if not instances:
        return
    validation = _validation_block(report)
    raw_ignore = validation.get(
        "collision_ignore_pairs", report.get("collision_ignore_pairs", [])
    )
    ignored: set[frozenset[str]] = set()
    for pair in raw_ignore if isinstance(raw_ignore, list) else []:
        if isinstance(pair, (list, tuple)) and len(pair) == 2:
            ignored.add(frozenset((str(pair[0]), str(pair[1]))))

    tolerance = float(
        validation.get("collision_volume_tolerance_mm3", DEFAULT_COLLISION_VOLUME_MM3)
    )
    checked = 0
    for first, second in combinations(instances.values(), 2):
        if frozenset((first.id, second.id)) in ignored:
            continue
        if _aabb_overlap_volume(first.mesh, second.mesh) <= tolerance:
            continue
        checked += 1
        try:
            volume = _intersection_volume(first.mesh, second.mesh)
        except Exception as exc:
            results.fail(
                "assembly-collision",
                f"{first.id}/{second.id}: boolean intersection failed ({exc})",
            )
            continue
        if volume > tolerance:
            results.fail(
                "assembly-collision",
                f"{first.id}/{second.id}: positive overlap {volume:.3f} mm^3",
            )
    collision_failures = [
        item for item in results.findings if item.check == "assembly-collision" and item.status == "FAIL"
    ]
    if not collision_failures:
        results.pass_(
            "assembly-collision",
            f"no positive-volume collision across {checked} AABB-overlapping pairs",
        )


def _probe_cylinder(x: float, y: float, diameter: float, bounds_z: np.ndarray) -> trimesh.Trimesh:
    bottom = float(bounds_z[0]) - 0.5
    height = float(bounds_z[1] - bounds_z[0]) + 1.0
    probe = trimesh.creation.cylinder(radius=diameter / 2.0, height=height, sections=48)
    probe.apply_translation((x, y, bottom + height / 2.0))
    return probe


def _xy_list(record: dict[str, Any], *names: str) -> list[list[float]]:
    value: Any = None
    for name in names:
        if name in record:
            value = record[name]
            break
    if not isinstance(value, list):
        raise ValueError(f"one of {', '.join(names)} must be a list")
    result = []
    for point in value:
        array = np.asarray(point, dtype=float)
        if array.shape not in ((2,), (3,)) or not np.isfinite(array).all():
            raise ValueError("bolt centre must contain two or three finite numbers")
        result.append([float(array[0]), float(array[1])])
    if not result:
        raise ValueError("at least one bolt centre is required")
    return result


def _instance_void_at(
    instance: Instance, x: float, y: float, diameter: float
) -> float:
    probe = _probe_cylinder(x, y, diameter, instance.mesh.bounds[:, 2])
    return _intersection_volume(instance.mesh, probe)


def validate_fasteners(
    report: dict[str, Any], instances: dict[str, Instance], results: Results
) -> None:
    validation = _validation_block(report)
    records = validation.get("fastener_mounts", report.get("fastener_mounts", []))
    if not isinstance(records, list) or not records:
        results.fail(
            "fastener-schema",
            "validation.fastener_mounts is required to prove module/base hole alignment",
        )
        return

    tolerance = float(validation.get("void_volume_tolerance_mm3", DEFAULT_VOID_VOLUME_MM3))
    hole_match_tolerance = float(validation.get("base_hole_match_tolerance_mm", 0.25))
    checked = 0
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            results.fail("fastener-schema", f"fastener mount #{index} is not an object")
            continue
        name = str(record.get("name", f"fastener mount #{index}"))
        module_id = str(record.get("instance", ""))
        base_id = str(record.get("base_instance", "baseplate"))
        module = instances.get(module_id)
        base = instances.get(base_id)
        if module is None or base is None:
            results.fail(
                "fastener-schema",
                f"{name}: unknown module/base instance {module_id!r}/{base_id!r}",
            )
            continue
        try:
            centres = _xy_list(
                record, "bolt_centers_world_mm", "bolt_centres_world_mm", "bolt_centers"
            )
            diameter = float(record.get("screw_diameter_mm", 3.0))
            if diameter <= 0:
                raise ValueError("screw_diameter_mm must be positive")
        except (TypeError, ValueError) as exc:
            results.fail("fastener-schema", f"{name}: {exc}")
            continue

        hole_set_name = str(record.get("base_hole_set", "base_module_holes"))
        raw_holes = report.get(hole_set_name, [])
        base_holes = [np.asarray(point[:2], dtype=float) for point in raw_holes]
        if not base_holes:
            results.fail("fastener-schema", f"{name}: report has no {hole_set_name}")
            continue

        for x, y in centres:
            checked += 1
            nearest = min(float(np.linalg.norm(np.asarray((x, y)) - hole)) for hole in base_holes)
            if nearest > hole_match_tolerance:
                results.fail(
                    "fastener-alignment",
                    f"{name} bolt ({x:.2f},{y:.2f}) is {nearest:.2f} mm from nearest {hole_set_name}",
                )
            for role, instance in (("base", base), ("module", module)):
                try:
                    obstruction = _instance_void_at(instance, x, y, diameter)
                except Exception as exc:
                    results.fail(
                        "fastener-alignment",
                        f"{name} {role} hole at ({x:.2f},{y:.2f}): boolean failed ({exc})",
                    )
                    continue
                if obstruction > tolerance:
                    results.fail(
                        "fastener-alignment",
                        f"{name} {role} blocks Ø{diameter:.2f} screw at "
                        f"({x:.2f},{y:.2f}) by {obstruction:.3f} mm^3",
                    )

    failures = [
        item for item in results.findings if item.check == "fastener-alignment" and item.status == "FAIL"
    ]
    if checked and not failures:
        results.pass_(
            "fastener-alignment",
            f"{checked} selected bolt paths match reported base holes and both STL openings",
        )


def _rect(center: Iterable[float], size: Iterable[float], field: str) -> np.ndarray:
    c = np.asarray(list(center), dtype=float)
    s = np.asarray(list(size), dtype=float)
    if c.shape != (2,) or s.shape != (2,) or not np.isfinite(c).all() or not np.isfinite(s).all():
        raise ValueError(f"{field} center/size must each contain two finite numbers")
    if np.any(s <= 0):
        raise ValueError(f"{field} size must be positive")
    return np.asarray((c - s / 2.0, c + s / 2.0))


def _rect_gap(a: np.ndarray, b: np.ndarray) -> float:
    dx = max(float(a[0, 0] - b[1, 0]), float(b[0, 0] - a[1, 0]), 0.0)
    dy = max(float(a[0, 1] - b[1, 1]), float(b[0, 1] - a[1, 1]), 0.0)
    return math.hypot(dx, dy)


def _rect_overlap_area(a: np.ndarray, b: np.ndarray) -> float:
    extents = np.maximum(np.minimum(a[1], b[1]) - np.maximum(a[0], b[0]), 0.0)
    return float(np.prod(extents))


def validate_deck(
    report: dict[str, Any], instances: dict[str, Instance], results: Results
) -> None:
    validation = _validation_block(report)
    config = validation.get("deck", report.get("deck_validation", {}))
    boards_raw = report.get("electrical_deck", {})
    if not isinstance(config, dict) or not config:
        results.fail(
            "deck-schema",
            "validation.deck is required for board-edge and terminal service checks",
        )
        return
    if not isinstance(boards_raw, dict) or not boards_raw:
        results.fail("deck-schema", "top-level electrical_deck board map is missing")
        return

    deck_id = str(config.get("instance", "electrical-deck"))
    deck = instances.get(deck_id)
    if deck is None:
        results.fail("deck-schema", f"unknown electrical-deck instance {deck_id!r}")
        return

    # Board and service-zone coordinates are deck-local.  The rectangular deck
    # bound is conservative at the rounded corners only when layouts stay away
    # from those corners, as required by the minimum edge clearance.
    local_source = _load_stl(deck.file)
    deck_bounds = local_source.bounds[:, :2]
    edge_min = float(config.get("minimum_board_edge_clearance_mm", 2.0))
    same_side_min = float(config.get("minimum_same_side_gap_mm", 2.0))
    service_depth_min = float(config.get("minimum_terminal_service_depth_mm", 20.0))
    service_owner_gap_max = float(config.get("maximum_service_zone_owner_gap_mm", 3.0))
    boards: dict[str, tuple[str, np.ndarray]] = {}
    for name, board in boards_raw.items():
        if not isinstance(board, dict):
            results.fail("deck-schema", f"board {name!r} record is not an object")
            continue
        try:
            bounds = _rect(
                board.get("center", board.get("center_mm")),
                board.get("board_size", board.get("size_mm")),
                f"board {name}",
            )
        except (TypeError, ValueError) as exc:
            results.fail("deck-schema", str(exc))
            continue
        side = str(board.get("side", ""))
        if side not in ("top", "bottom"):
            results.fail("deck-schema", f"board {name!r} has invalid side {side!r}")
            continue
        boards[str(name)] = (side, bounds)
        margin = float(np.min(np.concatenate((bounds[0] - deck_bounds[0], deck_bounds[1] - bounds[1]))))
        if margin < edge_min:
            results.fail(
                "deck-clearance",
                f"{name}: board-edge clearance {margin:.2f} mm < {edge_min:.2f} mm",
            )

    for (name_a, (side_a, rect_a)), (name_b, (side_b, rect_b)) in combinations(
        boards.items(), 2
    ):
        if side_a != side_b:
            continue
        gap = _rect_gap(rect_a, rect_b)
        if gap < same_side_min:
            results.fail(
                "deck-clearance",
                f"{name_a}/{name_b} ({side_a}) gap {gap:.2f} mm < {same_side_min:.2f} mm",
            )

    zones = config.get("terminal_service_zones", [])
    if not isinstance(zones, list) or not zones:
        results.fail(
            "terminal-service",
            "deck.terminal_service_zones must reserve terminal and wire-bend space",
        )
    else:
        for index, zone in enumerate(zones):
            if not isinstance(zone, dict):
                results.fail("terminal-service", f"service zone #{index} is not an object")
                continue
            name = str(zone.get("name", f"service zone #{index}"))
            owner = str(zone.get("owner", ""))
            side = str(zone.get("side", ""))
            try:
                bounds = _rect(
                    zone.get("center_mm", zone.get("center")),
                    zone.get("size_mm", zone.get("size")),
                    name,
                )
            except (TypeError, ValueError) as exc:
                results.fail("terminal-service", str(exc))
                continue
            if owner not in boards:
                results.fail("terminal-service", f"{name}: unknown owner board {owner!r}")
            elif side != boards[owner][0]:
                results.fail(
                    "terminal-service",
                    f"{name}: side {side!r} differs from owner {owner!r}",
                )
            else:
                owner_gap = _rect_gap(bounds, boards[owner][1])
                if owner_gap > service_owner_gap_max:
                    results.fail(
                        "terminal-service",
                        f"{name}: is {owner_gap:.2f} mm from owner {owner!r}; "
                        f"maximum is {service_owner_gap_max:.2f} mm",
                    )
            size = bounds[1] - bounds[0]
            zone_depth_min = float(zone.get("minimum_depth_mm", service_depth_min))
            if float(np.min(size)) < zone_depth_min:
                results.fail(
                    "terminal-service",
                    f"{name}: narrow dimension {float(np.min(size)):.2f} mm "
                    f"< required {zone_depth_min:.2f} mm",
                )
            allow_beyond_deck = bool(zone.get("allow_beyond_deck", False))
            if (not allow_beyond_deck and
                    (np.any(bounds[0] < deck_bounds[0]) or np.any(bounds[1] > deck_bounds[1]))):
                results.fail("terminal-service", f"{name}: extends beyond electrical deck")
            for board_name, (board_side, board_bounds) in boards.items():
                if board_side != side:
                    continue
                overlap = _rect_overlap_area(bounds, board_bounds)
                if overlap > 1e-6:
                    results.fail(
                        "terminal-service",
                        f"{name}: overlaps {board_name} by {overlap:.2f} mm^2",
                    )

    deck_failures = [
        item
        for item in results.findings
        if item.check in ("deck-clearance", "terminal-service") and item.status == "FAIL"
    ]
    if boards and not deck_failures:
        results.pass_(
            "deck-clearance",
            f"{len(boards)} PCB envelopes satisfy edge and same-side gap limits",
        )
        results.pass_(
            "terminal-service",
            f"{len(zones)} terminal/wire-bend service zones are unobstructed",
        )


def validate_downward_clearance(
    report: dict[str, Any], instances: dict[str, Instance], results: Results
) -> None:
    validation = _validation_block(report)
    probes = validation.get(
        "downward_clearance_probes", report.get("downward_clearance_probes", [])
    )
    if not isinstance(probes, list) or not probes:
        results.fail(
            "downward-clearance-schema",
            "validation.downward_clearance_probes is required for pins and nozzles",
        )
        return

    tolerance = float(validation.get("void_volume_tolerance_mm3", DEFAULT_VOID_VOLUME_MM3))
    required_counts = validation.get(
        "minimum_downward_probe_counts", {"sensor_pin": 7, "valve_nozzle": 6}
    )
    observed: dict[str, int] = {}
    checked = 0
    for index, record in enumerate(probes):
        if not isinstance(record, dict):
            results.fail("downward-clearance-schema", f"probe #{index} is not an object")
            continue
        name = str(record.get("name", f"probe #{index}"))
        kind = str(record.get("kind", ""))
        if not kind:
            lowered = name.lower()
            kind = "sensor_pin" if "sensor" in lowered or "pin" in lowered else (
                "valve_nozzle" if "valve" in lowered or "nozzle" in lowered else "other"
            )
        try:
            xy = np.asarray(
                record.get("center_world_mm", record.get("center")), dtype=float
            )
            if xy.shape not in ((2,), (3,)) or not np.isfinite(xy).all():
                raise ValueError("center_world_mm must contain two or three finite numbers")
            diameter = float(record["diameter_mm"])
            z_range = np.asarray(
                record.get("z_range_world_mm", record.get("z_range")), dtype=float
            )
            if diameter <= 0 or z_range.shape != (2,) or z_range[1] <= z_range[0]:
                raise ValueError("diameter must be positive and z_range must increase")
            obstacles = record.get("must_clear_instances", [])
            if not isinstance(obstacles, list) or not obstacles:
                raise ValueError("must_clear_instances must contain at least one id")
        except (KeyError, TypeError, ValueError) as exc:
            results.fail("downward-clearance-schema", f"{name}: {exc}")
            continue

        count = int(record.get("count", 1))
        observed[kind] = observed.get(kind, 0) + max(count, 0)
        probe = _probe_cylinder(float(xy[0]), float(xy[1]), diameter, z_range)
        for obstacle_id in obstacles:
            obstacle = instances.get(str(obstacle_id))
            if obstacle is None:
                results.fail(
                    "downward-clearance-schema",
                    f"{name}: unknown obstacle instance {obstacle_id!r}",
                )
                continue
            checked += 1
            try:
                obstruction = _intersection_volume(obstacle.mesh, probe)
            except Exception as exc:
                results.fail(
                    "downward-clearance",
                    f"{name}/{obstacle.id}: boolean intersection failed ({exc})",
                )
                continue
            if obstruction > tolerance:
                results.fail(
                    "downward-clearance",
                    f"{name}: {obstacle.id} blocks Ø{diameter:.2f} path by "
                    f"{obstruction:.3f} mm^3",
                )

    if isinstance(required_counts, dict):
        for kind, minimum in required_counts.items():
            actual = observed.get(str(kind), 0)
            if actual < int(minimum):
                results.fail(
                    "downward-clearance-schema",
                    f"{kind}: only {actual} represented, at least {int(minimum)} required",
                )

    failures = [
        item
        for item in results.findings
        if item.check == "downward-clearance" and item.status == "FAIL"
    ]
    if checked and not failures:
        results.pass_(
            "downward-clearance",
            f"{checked} pin/nozzle-to-obstacle clearance paths are open",
        )


def _print_human(results: Results) -> None:
    for finding in results.findings:
        stream = sys.stderr if finding.status == "FAIL" else sys.stdout
        print(f"[{finding.status}] {finding.check}: {finding.message}", file=stream)
    passed = sum(item.status == "PASS" for item in results.findings)
    failed = sum(item.status == "FAIL" for item in results.findings)
    print(f"\nP4 bay V1 validation: {passed} passed, {failed} failed")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=DEFAULT_OUT,
        help=f"V1 output directory (default: {DEFAULT_OUT})",
    )
    parser.add_argument(
        "--report",
        type=Path,
        help="layout report path (default: <out-dir>/layout-report.json)",
    )
    parser.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    out_dir = args.out_dir.resolve()
    report_path = (args.report or out_dir / "layout-report.json").resolve()
    if not out_dir.is_dir():
        print(
            f"ERROR: V1 output directory does not exist: {out_dir}\n"
            "Generate the V1 STL set before running this validator.",
            file=sys.stderr,
        )
        return 2
    if not report_path.is_file():
        print(f"ERROR: V1 layout report does not exist: {report_path}", file=sys.stderr)
        return 2
    try:
        with report_path.open(encoding="utf-8") as handle:
            report = json.load(handle)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"ERROR: cannot read {report_path}: {exc}", file=sys.stderr)
        return 2
    if not isinstance(report, dict):
        print(f"ERROR: {report_path} must contain a JSON object", file=sys.stderr)
        return 2

    results = Results()
    meshes = validate_stls(report, out_dir, results)
    instances = build_instances(report, out_dir, meshes, results)
    validate_collisions(report, instances, results)
    validate_fasteners(report, instances, results)
    validate_deck(report, instances, results)
    validate_downward_clearance(report, instances, results)

    if args.json:
        print(
            json.dumps(
                {
                    "ok": not results.failed,
                    "report": str(report_path),
                    "findings": [asdict(item) for item in results.findings],
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        _print_human(results)
    return 1 if results.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
