> English translation of [`docs/P3-GUIDE.md`](../P3-GUIDE.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# P3 whole-robot assembly and floor walking · detailed operating guide

Goal (weeks 9–15): **assemble the 18-servo robot, finish calibrating every servo, get it walking on flat ground.**
Acceptance: walk 2m in a straight line on the floor, turn 360° in place, cross a 2cm obstacle, single-leg FK/IK point-to-point error <5mm;
weigh the whole robot into `weight-log.md` (reference line ≤1.9kg without the adhesion system; start weight reduction above 2.2kg).

The P2 decision gate **passed on 2026-07-22** (acceptance data below the checklist in `P2-GUIDE.md`).
Two tracks run in parallel in this stage: the **delivery track** (order in step 1, 1–2 weeks of waiting) and the **printing track** (step 2,
keep the printer fully loaded). Assembly (from step 3 on) starts once the two tracks meet.

Prerequisite check:

- [ ] P2 decision gate passed (2026-07-22 ✅)
- [ ] Pi 5 environment healthy: `pytest tests/` all green, `get_throttled` = 0x0, independent 5V/5A supply
- [ ] L1 leg calibration values in `config.py` (tibia 89.3 k baseline / femur 49.7 / coxa official -8, to be measured in P3)

---

## Step 0 · One configuration question to settle first (half an hour, drives the print list)

The ROADMAP's staging says: **P3 walks on the floor with standard tibias** (left/right-tibia + foot contact switch +
rubber foot tip), and **only P4 switches to suction tibias for the wall** ("6 suction feet on the robot"). But that costs two things:

1. **L1 currently has a suction tibia** (the one used in P2, already calibrated). Following the ROADMAP, P3 would swap it for a
   standard tibia — swapping a structural part = the tibia calibration is void, and swapping back in P4 means calibrating again.
2. The tibia calibration of all 6 legs would be **completely void and redone** when P4 switches to suction tibias.

Two routes:

| Route | Content | Cost | Benefit |
|---|---|---|---|
| **A · Follow the ROADMAP** | All 6 legs use standard tibias + micro-switch-tip + rubber foot tip | 6 extra standard tibias to print (small parts, ~13g each); tibia calibration done once in P3 and again in P4 | Foot contact switches for touchdown detection (the mature official route); the cup lip doesn't scrub the ground |
| **B · Suction tibias directly** | All 6 legs walk on the floor with suction tibias | No touchdown switch (the end of the suction tibia is the cup cavity, there is no place for a switch); the lip acts as the sole and wears on the ground (cups are ¥5–15 consumables, 8 in stock) | The tibia is calibrated only once; L1's calibration stays valid; no part swap in P4 |
| **Issue route B must face** | The BOM marks the foot contact switch as "essential for the climbing gait", but the suction tibia has no place for a switch anyway — **P4 has to rely on pressure sensors for touchdown/adhesion confirmation regardless**. The switch really only serves P3 | | |

**✅ Route B decided (2026-07-22)**: all 6 legs walk on the floor with suction tibias directly. The
"touchdown-detection gait" in the P3 acceptance is downgraded to an open-loop tripod gait + pressure/current fallback (recorded in the acceptance checklist);
the foot contact switches are still bought in batch 2 and kept for P4 experiments.

> **✅ Suction tibia finalized (2026-07-27)**: use `left-tibia-suction.stl` with the matching
> `suction-foot-door.stl`; the `*-exp.stl` experimental version with the touchdown ring is not used. The right-leg version
> `right-tibia-suction.stl` has been mirrored the same way the official left/right tibias are, so printing can start.

## Step 1 · Order batch 2 (do it today, shipping is the critical path)

Per the "batch 2" list in `BOM.md` (about ¥1100–1900):

| Class | Content | Notes |
|---|---|---|
| Servos | DS3235 35kg 180° ×16 (to make 18 + 2 spares) | Don't buy the cheapest no-name ones; stalling is normal when climbing |
| Platform parts | Foot contact switches ×6 + spares (buy them even on route B, it's ¥5, keep them for P4 experiments), M1.6×6 ×120 and other screws, dowel pins, rubber foot tips | Check screw counts against the official fan parts list |
| Adhesion system | Pump ×1 (to make a dual-pump set), NC valves ×6 + 2 spares, suction cups ×5 (to make 8), **XGZP6847A ×6** (to make 7 = 1 per foot + 1 for the tank), ADS1115 ×1 (to make 2 boards, 8 channels total, I2C 0x48/0x49), **vacuum manifold 1-in-6-out ×1** (search `气动歧管 4mm 一进六出`, "pneumatic manifold 4mm 1-in-6-out", ~¥10-20; missing from the original BOM list — don't improvise with a chain of tees, 15 push-fit joints vs 7, and every joint is a leak point. Manifolds on the market mostly have 8mm **push-to-connect** inlets — push-to-connect can't grip soft silicone tubing, so unscrew the fitting and replace it with a G1/8 → 4mm barbed straight, keeping the whole chain on 4mm barbs + cable ties; or buy an `鱼缸气盘 一进六出` ("aquarium air splitter, 1-in-6-out") of the plain straight-through type without adjusting valves, whose barbs natively fit 4mm soft tubing — pull -70kPa and leak-test it the moment it arrives), 4mm tees ≥7 (for tapping pressure for the foot sensors), vacuum tank (first choice the plastic `汽车真空罐` ("car vacuum reservoir") ~¥30, designed for negative pressure so no crush test needed, comes with barbs and mounting ears; alternative `DIY CO2 瓶盖 双头` ("DIY CO2 bottle cap, dual port") — the plain two-port version — plus a soda bottle, ~¥5, pull -70kPa and leak-test on arrival), safety line | **The valves must be normally closed** — ask before ordering. Mount the foot pressure sensor **between the valve and the cup** (cup side) so foot pressure can be read even with the valve closed — adhesion confirmation and hiccup detection both depend on it (the single-sensor lesson from P1 + the P2 hiccup measurement where 30s leads to loss of control) |
| Power | XL6009 ×1 (to make 2; missing from the batch-2 enumeration in the BOM); **servo power switch ×1** (≥15A continuous, 3.3V trigger; first choice a `30A 继电器模块 5V 光耦` ("30A relay module, 5V, optocoupler") in the low-level-trigger version, or 2 × 15A MOSFET modules in parallel) | 12V for pump/valves; the switch controls the servo main power and **breaks the positive, high side** (ground is common through USB, so a low-side switch gets bypassed; the control pin was originally Servo2040 A0, changed to Pi GPIO17 in P4 on 08-15, see P4-GUIDE). Sizing is based on P2 measurements: 2–4A continuous / ~12A transient for the whole robot. Three checks on arrival: IN floating = open, 3.3V = closed, release = open |
| Consumables | Top PLA/PETG up to 2kg in stock; add 5m of silicone tubing if the spool is low; check your own stock of cable ties / epoxy / heat-shrink; **divider board materials**: 10k resistors ×14+, 100nF ×7, a small piece of perfboard, JST-XH or soldered wires; **P4 wiring materials**: 2×20 IDC ribbon connector + ribbon cable ×1 (clips onto the Pi 40-pin header as one piece, replacing individual Dupont jumpers), a handful of pre-crimped JST-XH pigtails (no crimping tool needed) | Confirm before the printing track starts; seal the pneumatic joints with epoxy, not hot glue. The sensor dividers (10k/10k) that sat on breadboards in P1/P2 must move to a soldered perfboard in P4 — breadboard/Dupont intermittent disconnects under vibration = state machine misjudgment = fall risk; power the ADS1115 from 3.3V and set PGA to ±4.096V (±2.048 clips in the -10kPa to 0 band, which breaks vent confirmation). **The divider board drawings (schematic / perfboard layout / assembly inspection) are in `html/en/p4-divider-board.html`** |

## Step 2 · Print scheduling and inventory (in parallel with shipping, carried over from P2)

Print order: longest first. Inventory table (tick when printed/inspected; inspected = assembly holes take the wire,
the servo horn seat fits, no warping or layer splits):

| Part | Need | Have | Short | Notes |
|---|---|---|---|---|
| frame | 1 | 0 | 1 | **Biggest part, first on the printer** |
| left-coxa2 / right-coxa2 | 3 / 3 | 1 (L1) | 2 / 3 | When several versions share a name, take the highest number |
| left-femur / right-femur | 3 / 3 | 1 (L1) | 2 / 3 | PLA, 4 walls, 40% infill |
| left-tibia-suction | 3 | 1 (L1) | 2 | **PETG, printed upright**: cup cavity facing down, 5–6 walls, 45%, brim ≥8mm (see the printing notes in the README) |
| right-tibia-suction | 3 | 0 | 3 | **PETG, printed upright**; mirrored from the finalized left leg across the XZ plane, uses the same door cover |
| suction-foot-door | 6 + 2 spares | 1 (L1) | 7 | Flat, no supports |
| top-cover4 / bottom-cover | 1 each | 0 | 1 each | Choose between bottom-cover and -flat depending on the battery layout |
| shield, servo2040-bottom-cover, battery-bar | 1 each | 0 | 1 each | Small parts, squeeze them into gaps |
| Spares | +1 each of coxa/femur/tibia | | | For quick repair after a crash; print last |
| **P4 pneumatic bay mounting parts** | 1 set | 0 | 1 set | **Design after the batch-2 parts arrive and can be measured** (extend generate_climbing_parts.py): soft TPU pump mount (a diaphragm pump vibrates; a rigid mount loosens screws and adds noise to the readings), 6-valve array beam, PET tank clamp (crush-test the tank alone at -70kPa first), 7 pressure sensor mounts + sensing tees (each foot's branch routed back into the bay), PCB standoffs, cable combs; make the whole bay a removable module and lay heavy parts flat against the body plane (CG close to the wall); leave a service loop where each leg's vacuum line crosses the coxa joint. **The electrical + pneumatic connection topology is in `html/en/p4-system-diagram.html`** |

`right-tibia-suction.stl` is generated properly by `tools/generate_climbing_parts.py`: the parametric geometry of the
finalized `left-tibia-suction.stl` is mirrored `Y → -Y`, the same left/right transform as between the official
`left-tibia.stl` / `right-tibia.stl`. The cup cavity, hex pockets and screw positions mirror
along with the body; the door cover is symmetric about its own center plane, so both legs share `suction-foot-door.stl`.

**Weigh and log to `weight-log.md` after every batch of prints**: all of the P2 acceptance rests on "2.5kg whole robot",
and the biggest variable inside that 1.8× margin is going overweight (analysis in the P2 acceptance discussion). Subtotal once all structural parts are done,
and estimate whether the whole robot (+18 servos, about 60g×18 = 1.1kg, + battery ~350g, + boards/wiring) can stay under 1.9kg.

## Step 3 · Assembly (after the parts arrive, 3–5 days)

1. Follow the official YouTube assembly video for the order (channel MakeYourPet), wire it per
   `hardware/makeyourpet-hexapod/wiring-diagram-servo2040.png`, channel mapping is in `config.py`.
2. **Rule for mounting servo horns**: run `python scripts/servo_center.py` to center every servo before mounting the horn.
   Note: servo_center sends 1500µs, but the calibration reference point is 1510µs (`(1040+1980)/2`,
   §4.6) — 1500 is fine for mounting horns (one spline tooth is ~14°, far coarser than 1°), but **when you later measure
   attach_deg you must use `servo_calib_helper.py` to send 1510**.
3. **Never force a joint by hand while the servos are enabled** — 35kg·cm stalling strips gears, and the angle you force it to is fake anyway.
4. Power rules as always: the Pi gets its own 5V/5A and never shares a rail with the servos; the servos' 7.4V goes through a relay (at the time
   Servo2040 A0/GPIO26, changed to Pi GPIO17 in P4 on 08-15); after assembly `vcgencmd get_throttled`
   must be 0x0.
5. Leave room for P4 when routing: at the root of every leg, leave a channel and cable-tie points for the vacuum tubing (no tubing yet,
   but don't let the harness block the path).

## Step 4 · Full servo calibration (mandatory after assembly, 2–3 days, the most tedious job)

The order matters (`LEG-GEOMETRY-OPEN.md` §5, remaining main line):

1. **coxa γ (the first thing after the body is assembled)**: use the baseline method — the line through the projected L1↔R1 servo-horn
   screw points is the reference; don't rely on "the body pocket is aligned to 55°" (§5 main line 1, method figure in the calibration reference figures §05).
   L1's coxa has been making do with the official -8 all along; now it can finally be measured.
2. ±45° pulse widths per joint: drive it manually (send pulse widths, don't move it by hand) to -45°/+45°, record and fill in
   `us_m45/us_p45` in `config.py`; a reversed direction gets `sign=-1`.
3. Measure femur/tibia `attach_deg` per leg: use the method proven on L1 (femur height-difference method §2.12,
   tibia trilateration method §2.11). **A leg that keeps the official 68 without measuring carries the 44° reference risk of §2.9;
   all six legs must be measured, don't cut corners.**
4. **End-to-end FK spot check (P3 acceptance item)**: send a set of joint angles to each leg, measure the actual foot tip position with a tape against
   the `leg_fk` prediction, **<5mm passes**. While at it, L1 settles the tibia_len 116 vs 120
   dispute recorded in §4.4 (the 4.6mm of tension sits exactly on the edge of the criterion; measure three different poses and take the majority).
5. Rule restated: replacing a structural part / remounting a servo horn → that joint's calibration is void, redo it.

## Step 5 · First runs in software and standing up (1–2 days)

```bash
pytest tests/                                  # all green first
python scripts/sim_walk.py --gif walk.gif      # eyeball the simulated gait
python scripts/stand_up.py                     # stand up + sensor readings
python scripts/walk_teleop.py                  # wasd/qe teleop, 1/2 switch tripod/wave
```

- **Prop the body up on blocks before stand_up**: any script exit or exception cuts servo power and the robot flops down.
- Watch three things during the first stand: no odd noises or heat on any of the 18 channels, sensible current readings, `get_throttled` still 0x0.
- With the body propped up, run walk_teleop in the air to confirm the phases are right (in the tripod gait the three legs of a group lift and land together), then set it down.

## Step 6 · Graded acceptance (1–2 days)

| Level | Content | Pass criterion |
|---|---|---|
| 1 | Walking in the air | Gait phases correct, no joint jitter |
| 2 | Standing on the ground for 60s | No overheating, no undervoltage, posture doesn't sag |
| 3 | 2m straight | Doesn't veer out of a 0.3m corridor, doesn't fall |
| 4 | 360° turn in place | Completing it passes |
| 5 | Crossing a 2cm obstacle | Completing it passes |
| 6 | FK spot check | All six legs <5mm (done in step 4, archived here) |
| 7 | Weighing | Whole robot (no adhesion) measured into `weight-log.md`; start weight reduction above 2.2kg |

**P3 acceptance checklist**:

- [ ] 2m straight / 360° turn / 2cm obstacle
- [ ] FK spot check <5mm on all six legs, tibia_len 116/120 dispute settled and written back into config
- [ ] All 18 calibration values in `config.py` (including a measured coxa γ, ending the official -8)
- [ ] Whole-robot weight in `weight-log.md` (≤2.2kg; if over, list the weight-reduction items)
- [ ] Printing inventory of the whole robot's structural parts complete (every box in the step 2 table ticked)
- [ ] Touchdown detection: route A switches installed / route B records the downgrade decision and the P4 pressure plan

## Common problems

| Symptom | What to do |
|---|---|
| One leg's posture is clearly wrong when standing | Check sign first, then attach_deg; a joint off by a multiple of ~14° = the servo horn is on the wrong spline tooth |
| Veering while walking | The gait itself is left-right symmetric (zero calibration error = zero drift); veering comes from attach residuals and stance-phase slip. **Sensitivity**: 1° of attach error on any channel → 0.5–0.8°/m of yaw, 3–17mm/m of lateral shift; the biggest lever is the middle legs L2/R2, tibia (0.81°/m) and femur (0.60°/m); the middle coxa has almost no effect (its error is absorbed as a phase offset). At the current residual level, 2–6cm off over 2m is normal. Walk 2m straight first to tell **yaw** (heading changed) from **lateral shift** (heading unchanged, whole body translated), then decide which channel to re-measure; if you don't want to touch the calibration again, put the measured values into `yaw_trim_deg_per_m` / `side_trim_mm_per_m` in `config.py` (left is positive, applied proportionally only when there is vx) |
| Servos hot after a few steps | Don't keep standing joint angles near the edge of travel; slow the gait down (lower SPEED); check whether one leg carries far more load than the others |
| Pi reboots | Undervoltage. `get_throttled` non-zero → power problem, fix it before continuing; in P4 this is a fall |
| Occasional servo jitter | Signal wires bundled too close to the servo power wires; route them separately or add a ferrite ring |
| Gait phases scrambled | Channel mapping is off; check every channel against the wiring using `config.py` |

## Safety

- Run everything in the air before any test on the ground; do the first landing on a soft mat.
- Never force any joint by hand while servos are enabled; disable them before adjusting the pose.
- LiPo rules same as P0–P2: never leave it charging unattended, keep the XT60 within reach to yank, power down before rewiring.
- Quick crash repair: keep spare legs on hand (the last batch in step 2) so a broken part can be swapped the same day.
