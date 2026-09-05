> English translation of [`docs/CLIMBING-DESIGN.md`](../CLIMBING-DESIGN.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Wall-climbing design rationale

## 1. Picking an adhesion method

| Method | Principle | Suitable walls | DIY feasibility | Verdict |
|---|---|---|---|---|
| **Vacuum suction cups + valve bank** | Pump pulls vacuum, one valve per foot controls attach/release | Glass, tile, painted steel, acrylic | ★★★★☆ all standard pneumatic parts | **Main line** |
| EDF pressing against the wall | Ducted fan presses the machine onto the wall | Almost any flat wall | ★★★★☆ off-the-shelf RC parts | Dropped: only works for platforms <800 g, incompatible with a 2 kg platform (see §3) |
| Microspine | Fine needles hook onto microscopic asperities of a rough surface | Brick, concrete, rough paint | ★★★☆☆ needles + printed parts is all it takes, but carrying load on spines alone needs a finely tuned compliant suspension | Dropped: only practical when paired with an EDF |
| Magnetic | Electromagnet / permanent magnet | Steel surfaces only | ★★★★★ | Too narrow a use case, dropped |
| Electrostatic adhesion | High-voltage electrodes polarize the wall | Many | ★☆☆☆☆ high voltage + weak force | Research grade, dropped |
| Biomimetic dry adhesive (gecko) | Van der Waals force | Smooth surfaces | ★☆☆☆☆ hard to make the material yourself, fouls easily | Research grade, dropped |
| Hot-melt glue feet | Glue on, heat to release | Many | ★★☆☆☆ slow cycle, high power | Dropped |

### Leg count: why six legs and not four

Four legs would save 6 servos + 2 foot modules (about −450 g / −¥300–500), but on a wall it is strictly more dangerous:
in swing phase a hexapod has 5 feet attached, a quadruped 3; after a single cup lets go (P2 acceptance already allows a 5% failure rate, so it will happen)
the hexapod still has 4 feet and can gracefully degrade and re-attach, while the quadruped has 2 feet taking a dynamic shock load and will most likely cascade into a fall.
On top of that a quadruped's per-foot peak load is ×1.5 (cups would have to go up to 40 mm, eating the weight advantage), its static gait on the floor has to shift the CG,
and the floor-to-wall transition only ever has "2+2" support. The academic precedent (Hirose's NINJA-I, a quadruped suction-cup wall climber) relies on research-grade
special valves and high-DOF legs, which does not suit DIY.
**Decision: six legs. Reversible fallback: the official configuration has MODE_QUADRUPED, so adding a quadruped gait to the gait engine lets us
A/B test on the same hardware; if P4 gets stuck on weight, pulling the two middle legs and going quadruped is an experiment we can run at any time.**

## 2. Force budget for the suction-cup foot design

Taking the MakeYourPet large platform + adhesion system as a whole robot at **m = 2.5 kg** (upper bound with margin):

- One 30 mm cup (with a 3D-printed anti-deformation shell) at a working vacuum of −50 kPa, measured:
  normal pull-off force = **25 N (2.5 kg)**, shear pull-off on vertical glass = **15 N (1.5 kg)** (about 60% of normal).
- At any moment in the climbing gait at least **5 cups are attached**; being extremely conservative, assume only 3 actually carry load:
  total normal pull ≥ 75 N, giving a **normal safety factor of 3.0** against 2.5 kg (25 N) of gravity;
  total shear pull ≥ 45 N, giving a **shear safety factor of 1.8** against 2.5 kg (25 N) (if all 5 are effective the shear safety factor reaches 3.0 — plenty of margin!).
- Peel moment: with the CG standing off the wall by d, the upper row of cups sees an extra pull of ≈ m·g·d/h (h = spacing between upper and lower feet).
  Every 1 cm you take off d is real margin → **keep the body as close to the wall as possible in the climbing pose, and mount heavy items such as the battery on the wall side**.

System conclusions:
1. 30 mm cups ×6 at −50 kPa works for a whole robot up to 2.5 kg; switch to 40 mm (area ×1.78) for the payload stage.
2. The pump only has to keep the vacuum tank down; the valves handle attach/release per foot → the pump can run intermittently and save power.
3. **NC valves + check valves** make the powered-off state = all feet locked on. This is the single most important passive-safety feature.

## 3. Record of a dropped route: EDF wall pressing

The common DIY approach for rough walls (brick, concrete, latex paint) is a ducted fan pressing the machine onto the wall + friction/microspine feet.
Reason for dropping it (physics vetoed it once the platform was fixed as the MakeYourPet large platform): a 70 mm 12-blade EDF at 4S gives only
12–18 N static thrust; with a friction coefficient μ≈0.6, not slipping needs F_press ≥ m·g/μ, so a 2 kg robot needs 33 N+ of continuous pressing force,
which means dual 90 mm EDFs, hundreds of watts, and it falls the moment the fan stops — in conflict with the project goals of "hangs statically, can carry a payload".
This route only works for lightweight platforms under 800 g (references: the GECO and IJERT papers in §7).
**Conclusion: this project only targets smooth walls (glass/tile/painted steel). Rough walls are out of scope.**

## 4. Pneumatic diagram

```
pump (555 ×2 parallel) → check valve → tank (PET bottle) ─────────┬→ NC 3/2-way valve #1 → foot 1 cup
      ↑                                  │ (XGZP6847A pressure    ├→ NC 3/2-way valve #2 → foot 2 cup
  MOSFET board ← Pi5 GPIO                │ feedback, via          ├→ ...
             Pi5 I2C ← ADS1115 ──────────┘ ADS1115)               └→ NC 3/2-way valve #6 → foot 6 cup
   third port of the 3/2-way valve = atmosphere (energizing switches it to the vent position, releasing that foot)
```

Control logic (adhesion state machine, one per foot):
`RELEASED → press onto the wall → valve to the suction position → wait for pressure ≤ -30kPa (on a 500ms timeout, lift the foot and retry) → ATTACHED (load bearing allowed) → when a move is needed, valve to the vent position → 300ms delay → RELEASED`

Gait constraint: **at least 5 feet ATTACHED at any moment**, and no two adjacent feet released at the same time.

> [!TIP]
> **The killer valve hookup that holds pressure with the power off (P1 stage summary)**
> Measured behavior of the 0520B-type NC 3/2-way valve we bought: with the coil off, port 1 connects to port 3 and port 2 is blocked.
> To get the passive-safety design "power off = cup keeps its vacuum", **you must wire it like this**:
> 1. Port 1 ➡️ to the suction cup
> 2. Port 3 ➡️ to the vacuum pump inlet
> 3. Port 2 ➡️ open to atmosphere
> **Why it works**: with the power off (1 connected to 3) the cup is connected to the stopped vacuum pump, and the check valves inside the diaphragm pump dead-lock the vacuum in. With the power on (1 connected to 2) the cup is opened to atmosphere, pressure is released and the leg can lift. This hookup not only gives absolute power-off anti-fall safety, it also draws 0 current from the solenoid valve while sitting attached. The code must be configured with `VALVE_ON_LEVEL=0`.

## 5. Evolution of the electrical architecture

**Single-brain architecture: the Raspberry Pi 5 is the brain from day one** (`software/` in-house software, the same code stack through all stages):

```
2S battery ─┬─ 7.4V direct → Servo2040 servo power rail (via relay) → 18 servos
            ├─ 5V/5A buck → Pi 5 (USB-C, its own supply, never shared with the servo rail)
            └─ XL6009 boost to 12V → MOSFET board → 6 valves + 2 pumps
Pi 5 ─USB──→ Servo2040 (chica serial protocol: 18 servos + 6 foot contact switches + voltage/current)
Pi 5 ─I2C──→ ADS1115 ×2 → XGZP6847A pressure sensors (from P1)
Pi 5 ─GPIO─→ MOSFET board (valve/pump switching, from P1); GPIO17 → servo relay (from P4)
Pi 5 ─CSI──→ camera module (P5, FPV)
```

- The protocol description and driver implementation are in `software/hexapod/driver.py` (verified byte by byte against the firmware source).
- Before the gait moves a step, the adhesion state machine confirms "the target foot is attached" (`software/hexapod/adhesion.py`).
- **A low-voltage alarm is mandatory**: LiPo undervoltage = pump stops = adhesion decays = fall. In software, `check_power()` throws and stops the machine below 6.0 V (the valves are NC, so it stays attached even with the power off).

## 6. Workspace analysis: can a 3DOF leg get onto a vertical wall

The analysis script is `tools/workspace_analysis.py` (computed with the real link lengths 43/80/120 mm — tibia is the L1 measured value,
see LEG-GEOMETRY-OPEN §2.11 — and the servos' electrical travel, femur −40.3° to +139.7° (L1 measured attach 49.7°,
§2.12) and knee interior angle 0°–176.4°; a 2D leg-plane model, conservative. Recomputed 2026-07-17 against the measured calibration;
**2026-07-18 the cup axis model was corrected**: the physical cup axis runs along the square shaft = a_t − 22.7°, which is not the direction of the K→P line
a_t (a fixed geometric offset, the knee axis K sitting 46.5 mm off the square shaft, LEG-GEOMETRY-OPEN §2.13); the table below has been recomputed on that basis).
The fundamental limit of a 3DOF leg: **foot position and cup orientation are coupled** — pick a foothold and the cup axis is determined,
so all you have to fall back on is the angular compliance of the bellows cup.

**A. Walking on the wall: feasible.** With the body parallel to the wall the kinematics are equivalent to floor walking; the only new constraint is
the cup axis's tilt relative to the wall normal (a bellows cup tolerates about 15°):

| Stand height (body off the wall) | Stance-phase reach | Stride | Cup axis tilt |
|---|---|---|---|
| 90mm | **170mm** | 0 / 30 / 40 / 60mm | 1.6° / ≤8.9° / ≤11.5° / ≤17.2° |
| 90mm | 130mm (floor default) | 0mm | **16.7°, over tolerance** |
| 70mm | 170mm | 0 / 30 / 40mm | 5.0° / ≤11.5° / ≤13.9° |

→ The climbing gait uses **stand height 90 mm + stance reach 170 mm + stride ≤40 mm** (tilt ≤11.5°).
The key change (2026-07-18): **you cannot carry the floor pose of reach=130 onto the wall** — that would put the cup onto the wall at a
16.7° angle; on the wall the feet have to splay out to reach≈170. "Body close to the wall" (lowering the peel moment) and
"cup squared up" pull in opposite directions; 90 mm is still the balance point.

**B. Floor-to-wall transition (front feet onto the wall): kinematically solvable.** Reachable foothold band for a front leg (15° tilt tolerance,
height range relative to the front hip, mm):

| Body pitch | Wall distance from front hip 100 | 140 | 180 |
|---|---|---|---|
| 0° (body level) | +75–+143 | +95–+153 | +95–+141 |
| 30° (nose up about the rear hips) | +96–+168 | +116–+174 | +114–+158 |

→ With the cup axis corrected the reachable band is actually **wider and higher** (about 50–70 mm tall, shifted up overall): with the body level,
feasible footholds exist at every wall distance from 80–200 mm; pitching the body 30° (`robot.py`'s `body_rpy` already supports it)
makes it wider still. Every step of the transition sequence has a solution, but sequencing it is complex, so per the roadmap it stays in P5.

**Conclusions and design decisions:**
1. No need to add a 4th ankle servo to each leg — saves the weight and complexity of 6 servos.
2. The key parameter for feasibility is the bellows cup's real angular tolerance. **P1 test-rig measurement: the tilt tolerance reached 15°!** That means we can unleash the robot's full mobility and climb with a **40 mm stride**, with no need to tighten up to 25 mm.
3. All of the above is computed from electrical travel and does not account for structural interference (femur hitting the body, tibia hitting the femur) — during the P2
   single-leg tests, measure each joint's real mechanical travel and fill it back into this analysis.

## 7. Reference projects and literature

- MakeYourPet hexapod (`hardware/makeyourpet-hexapod/` in this repo, MIT): https://github.com/MakeYourPet/hexapod
- Aecert Robotics hexapod (full Arduino tutorial series): https://aecertrobotics.com , code at https://github.com/Ryan-Mirch/Aecerts_Hexapod_V1
- GECO wall-climbing robot (Ghent University, EDF suction, reference for the dropped route in §3): https://www.instructables.com/GECO-Wall-Climbing-Robot/
- Design of a sub-1 kg EDF wall-climbing robot (IJERT, 2026, reference for the dropped route in §3): https://www.ijert.org/design-and-development-of-a-sub-1-kg-wall-climbing-robot-using-electric-ducted-fan-for-vertical-surface-operations-ijertv15is043914
- Review of wall-climbing robot technology (ScienceDirect, 2024): https://www.sciencedirect.com/science/article/pii/S2773186324000781
- Design of a wall-climbing robot with passive suction cups: https://www.researchgate.net/publication/251991762_Design_of_a_wall-climbing_robot_with_passive_suction_cups
