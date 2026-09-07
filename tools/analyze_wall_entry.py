#!/usr/bin/env python3
"""Offline first wall-contact screen; never imports a hardware driver.

Body level, hip plane 90 mm above floor, wall normal along body +X.
Samples front-foot lip centres and checks the current IK, unclamped electrical
limits, cup-axis alignment and joint-centre clearance. Uses the free-cup model.
Does NOT validate six-leg equilibrium, a continuous entry path, mesh collisions,
cup compression, hose clearance, physical joint stops or loaded servo capacity.
"""
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "software"))
from hexapod.config import DEFAULT_CONFIG as CFG
from hexapod.kinematics import leg_ik, leg_joint_points, WorkspaceError


def raw_pulse(cal, angle):
    return ((cal.us_m45 + cal.us_p45) / 2
            + cal.sign * (angle - cal.attach_deg)
            * (cal.us_p45 - cal.us_m45) / 90)


def candidate(leg, distance, lateral, height):
    mount = math.radians(leg.mount_angle_deg)
    c, s = math.cos(mount), math.sin(mount)
    x, y = c * distance + s * lateral, -s * distance + c * lateral
    try:
        g, a, t = leg_ik(CFG, x, y, height - CFG.stand_height)
    except WorkspaceError:
        return None
    angles = [math.degrees(g), math.degrees(a), 180 - math.degrees(t)]
    calibrations = [leg.coxa, leg.femur, leg.tibia]
    pulses = [raw_pulse(cal, angle) for cal, angle in zip(calibrations, angles)]
    if any(not cal.min_us <= pulse <= cal.max_us
           for cal, pulse in zip(calibrations, pulses)):
        return None
    # Cup direction in body coordinates, per config's in-plane axis model.
    beta = a + t - math.pi + math.radians(CFG.cup_delta_deg)
    normal_x = math.cos(beta) * math.cos(mount + g)
    tilt = math.degrees(math.acos(max(-1, min(1, normal_x))))
    if tilt > 15:
        return None
    points = leg_joint_points(CFG, g, a, t)
    for px, py, pz in points[:-1]:
        if c * px - s * py > distance - 10 or pz + CFG.stand_height < 10:
            return None
    margin = min(min(pulse - cal.min_us, cal.max_us - pulse)
                 for cal, pulse in zip(calibrations, pulses))
    return dict(lip_height_mm=height, lateral_from_hip_mm=lateral,
                cup_tilt_deg=round(tilt, 2),
                joint_deg=[round(v, 2) for v in angles],
                pulse_us=[round(v, 1) for v in pulses],
                electrical_margin_us=round(margin, 1))


def main():
    results = []
    for name in ("L1", "R1"):
        leg = CFG.leg(name)
        for distance in (140, 160, 180):
            hits = [p for height in range(30, 321)
                    for lateral in range(-40, 41, 5)
                    if (p := candidate(leg, distance, lateral, height)) is not None]
            best = min(hits, key=lambda p: (p["cup_tilt_deg"],
                                           -p["electrical_margin_us"])) if hits else None
            results.append(dict(leg=name, hip_to_wall_mm=distance,
                                sampled_lip_height_range_mm=(
                                    [min(p["lip_height_mm"] for p in hits),
                                     max(p["lip_height_mm"] for p in hits)]
                                    if hits else None), best_alignment_sample=best))
    print(json.dumps(dict(scope="First-contact geometry only; not an executable gait",
                          body_height_mm=CFG.stand_height,
                          cup_tolerance_assumption_deg=15, results=results),
                     ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
