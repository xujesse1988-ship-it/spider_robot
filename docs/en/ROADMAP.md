> English translation of [`docs/ROADMAP.md`](../ROADMAP.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Wall-Climbing Spider Robot · Implementation Roadmap

> Overall strategy: **front-load the biggest risk — prove "one foot on the wall" with the least money possible, and only commit to the whole robot after the decision gate passes.**
> Floor walking is a mature path already validated by the MakeYourPet community, so it carries little information and is done after the adhesion validation.
> Purchasing is split into two batches: batch 1 buys only the single-leg validation kit (about ¥600–950); the big items such as the 15 servos are ordered only after the P2 decision gate passes.
> Climbing is extremely sensitive to weight, so record the weight budget at every stage.

## Technical approach

**Suction-cup feet + solenoid valve bank**: one vacuum suction cup per foot; the pump pulls vacuum to attach, a solenoid valve vents to release.
It can hold statically (normally-closed valves keep the vacuum when unpowered, the pump runs intermittently), can carry a payload, and does real "Spider-Man" style climbing;
whole-robot weight budget ~2.5kg (three 30mm cups attached at once already means >100N of normal force).
Limitation: only works on smooth walls (glass, tile, painted metal, acrylic).
Cost: the gait has to be coupled with valve switching and pressure feedback, so control is the core workload of this project (`software/` already has the skeleton).

> An EDF ducted-fan wall-pressing route (suited to rough walls) was evaluated and dropped: it only holds up for platforms <800g,
> physically incompatible with the chosen large MakeYourPet platform (~2kg). The argument is in `CLIMBING-DESIGN.md`.

---

## P0 Preparation (weeks 1–2)

> Step-by-step guide (commands, checklists, common problems): [P0-GUIDE.md](P0-GUIDE.md)

- Order the **batch 1** parts: the single-leg validation kit (see "Batched purchasing" at the top of `BOM.md`); buy only 4 servos (3 in use, 1 spare).
- Print: one complete leg — coxa/femur from `hardware/makeyourpet-hexapod/STL/`, tibia from `climbing-parts/left-tibia-suction.stl` (the suction-foot integrated tibia) + one `suction-foot-door.stl`.
- Raspberry Pi 5 setup: flash Raspberry Pi OS Lite 64-bit, enable SSH/I2C, install the `software/` package and get the `sim_walk.py` simulation and `pytest` running.
- Acceptance: one leg with 3 servos fully assembled, and the Pi can swing it through the Servo2040; simulation and tests all green on the Pi.

## P1 Adhesion system bench validation (weeks 2–5)

> Step-by-step guide (wiring, code, record templates, common problems): [P1-GUIDE.md](P1-GUIDE.md)

First get the vacuum circuit working on the bench (a horizontal tabletop is enough, no wall yet):

- Build the single-foot vacuum circuit: pump → check valve → vacuum tank (a PET bottle will do) → solenoid valve → suction cup, with an XGZP6847A pressure sensor; the Pi 5 reads pressure through an ADS1115 and drives MOSFETs from GPIO to switch pump/valve (implement `Pi5VacuumIO` in `software/hexapod/adhesion.py`; the state machine and simulation are already in place).
- Test on vertical glass/tile and record three curves:
  1. Pump-down time: cup on the wall → how long to -40kPa (target <1s)
  2. Leak rate: how fast the pressure comes back after the valve closes (target: from -40kPa, hold >60s without decaying to -20kPa)
  3. Release time: valve vents → the cup can be lifted off with no resistance (target <0.5s)
- Destructive test: how many kg of normal hanging weight and of shear hanging weight it takes to pull one 30mm cup off (theory: normal ~35N @ -50kPa, shear about 50% of that).
- **Tilt-angle adhesion test** (the key input for the workspace analysis, see `CLIMBING-DESIGN.md` §6): sealing success rate and pull-off force with the cup axis tilted 5°/10°/15°/20° from the wall normal. If the measured tolerance is ≥15°, the climbing stride can be 40mm; if it is only 10°, tighten it to ~25mm.
- Repeat the tests above with the cup installed in the `left-tibia-suction.stl` cavity and the door cover glued on (validates the clamping stiffness and strength of the printed part).
- **Acceptance: a single suction-cup foot holds a 3kg weight on vertical glass for 10 minutes; one "attach–confirm–release" cycle takes <2s.**

## P2 Single-leg wall trial (weeks 5–9) — decision gate ✅ passed 2026-07-22

> Step-by-step guide (rig drawing, geometry constants, staged testing, common problems): [P2-GUIDE.md](P2-GUIDE.md)
> Measured acceptance: 50 cycles at 100%, 1.5kg × 300s hang, 1kg side pull — all passed; data in `docs/data/`.

The most critical validation of the whole project; batch 2 is not bought until it passes:

- **Test rig**: an L-shaped frame from wood board / aluminum extrusion, holding the hip joint (coxa axis) about 130mm in front of the vertical glass (mid-range of the leg's workspace); the leg + suction foot runs the complete cycle against the wall: "extend → press → pump down → pressure confirm → take load → vent → lift".
- **Load simulation**: remount the leg base on a carriage that slides along a vertical rail, and weight the carriage to 1.5kg (≈ the 2.5kg whole robot ÷ 3 effectively load-bearing feet, plus margin). Release the carriage lock once adhesion is confirmed — this one leg has to hold the weight on its own, which is exactly its real duty when the whole robot is on the wall.
- Software deliverable: `scripts/single_leg_wall.py` (single-leg IK integrated with the `adhesion.py` state machine: the weight is transferred only when the pressure is ≤-30kPa; on failure it automatically lifts 5mm and retries), logging the pressure curve and servo current for each cycle.
- Measure the servos' real torque margin in the wall shear-load pose (35kg·cm servos are used derated to 50%).
- **Acceptance (decision gate): >95% success over 50 consecutive attach/release cycles; while attached, the carriage's 1.5kg weight hangs for 5 minutes and a 1kg lateral pull is applied without detaching.**
- Decision: pass → order batch 2 (15 servos + everything else), move to P3; fail → upgrade to 40mm cups / dual pump and retest; still failing → re-evaluate the project concept, having spent only a few hundred yuan so far.

## P3 Floor walking (weeks 9–15)

Once the single leg passes, walk the well-trodden path to the end (while the printer sits idle during P1/P2 you can already print the whole robot's structural parts):

- Print all structural parts (PLA is fine; 4 walls + 40% infill recommended for the leg parts), assemble the 18 servos, route the wiring, calibrate the servo centers.
- Order in which to pick up the control software:
  1. `scripts/servo_center.py` to center the servos, fit the servo horns, then fill in the ±45° pulse widths following the calibration procedure in `software/README.md`;
  2. `scripts/stand_up.py` to stand; `scripts/walk_teleop.py` for keyboard teleop with the tripod gait.
- Wire the foot contact switches to the Servo2040 sensor ports (official wiring), read them from the Pi over the same USB, and implement a ground-contact-detecting gait.
- **Acceptance: walk 2m in a straight line on flat ground, turn 360° in place, step over a 2cm obstacle, single-leg IK point-to-point error <5mm.**
- Milestone weight record: weigh the whole robot; over 2.2kg means it is time to start cutting weight (the climbing budget cap is 2.5kg including the adhesion system).

## P4 Whole-robot climbing integration (weeks 15–26)

> Step-by-step guide (gait engine rework, pneumatic assembly and leak check, progressive incline-to-wall ramp-up, common problems): [P4-GUIDE.md](P4-GUIDE.md)

- Six suction feet on the robot; the pneumatics use 1 pump + vacuum tank + 6 valves (the valve bank mounts on `component_plate.stl`); pick **normally-closed** solenoid valves — they hold the vacuum when unpowered, so the robot does not fall when power is lost.
- The gait changes from the floor tripod gait (3+3) to a conservative **five-foot support gait** (only one foot moves at a time), coordinated with valve switching timing and pressure confirmation.
- Body pose: keep the body close to the wall while climbing (the closer the center of mass is to the wall, the smaller the peel moment on the upper row of cups).
- **Safety line on at all times** (anchored at the top, climbing accessory cord with a little slack), and mats on the floor.
- The ground-to-wall transition (walking from the floor onto the wall) is the hardest move and is skipped at this stage: put the robot on the wall by hand.
- **Acceptance: on a vertical glass/tile wall, climb 1m upward, traverse 0.5m sideways, and hang in place with the power cut for 5 minutes without falling off.**

## P5 Payload and capability extensions (after week 26, ordered as needed)

1. **Payload**: target 20–30% of its own weight (400–600g) as carried load. Means: go up to 40mm cups (normal force ×1.8), upgrade to a dual-head pump, and step the test weights up to find the safety boundary, then take 50% of it as the rated payload.
2. **Ground-to-wall transition**: the two front feet attach to the wall first → the body leans back → the middle feet go on → the rear feet go on. The workspace analysis (`CLIMBING-DESIGN.md` §6) has confirmed there is a kinematic solution: with the body level and the wall 140–200mm from the front hips there is a footfall band about 70mm tall, and it gets wider once the body pitches 30°; the hard part is sequencing the transition and managing the center of mass, not changing the leg geometry.
3. **Sensing and teleop upgrades**: a camera module on the Pi 5 CSI port for FPV; MPU6050 attitude sensing (on detecting a slip and fall, immediately drive all valves to attach and raise an alarm).
4. **Ceiling walking**: the suction approach is feasible in theory, but the normal-force budget has to be recomputed (the entire weight becomes peel force).
5. **Autonomy**: SLAM/line following make little sense on a wall; prioritize practical automatic moves like "one-button return to charge" and "one-button off the wall".

---

## Risk list (ordered by damage)

| Risk | Mitigation |
|---|---|
| Finding out the adhesion approach is unworkable only after the whole budget is spent | Avoided by the stage ordering: only batch-1 money (about ¥600–950) is spent before the P2 single-leg decision gate passes |
| Robot too heavy, adhesion margin too thin | Weigh at every stage; over 2.2kg at the end of P3 means cut weight (drop the shell, switch to a smaller 2S battery) |
| Valve/tubing leaks, vacuum does not last long enough for one step | Nail down the leak rate on the P1 bench before going on the robot; every fitting gets a clip + cable tie + sealant |
| Servos short on torque or overheating in the wall pose | Measured on the single leg in P2; 35kg·cm servos derated 50%; slow the gait down |
| Wall dust kills the suction cups | Only test on wiped-clean glass/tile; clean the cup lips with alcohol regularly |
| Unstable Pi 5 power, reboots mid-walk / mid-climb | Power it from a dedicated 5V/5A buck converter, never sharing a rail with the servos; verify with `vcgencmd get_throttled`; if the undervoltage history is nonzero, fix the power before continuing |
| Crash damage | The safety line never comes off; test low first, at 0.3m |
| Attempting the ground-to-wall transition right at the start | Don't. In P4 the robot goes on the wall by hand; the transition waits for P5 |
