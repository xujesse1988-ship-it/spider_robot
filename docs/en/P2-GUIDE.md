> English translation of [`docs/P2-GUIDE.md`](../P2-GUIDE.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# P2 Single-Leg Wall Trial · Detailed Operating Guide

Goal (weeks 5–9): **the decision gate of the whole project — one leg, in front of vertical glass, automatically completes the
"extend → press → pump down → pressure confirm → take load → vent → lift" cycle and holds a 1.5kg weight on its own.**
By the end of this stage you should be able to shoot two videos: ① the rig running 50 attach/release cycles back to back; ② with the carriage's lock pin pulled,
the 1.5kg weight hanging entirely on the attached leg for 5 minutes without budging.

**Batch 2 is not bought** before this passes. Suggested order: step 1 (materials) goes first; step 4 (software dry run) doesn't depend on
the rig and can be finished while waiting for shipping; steps 2–3 build the rig, steps 5–6 are the main event, strictly in order.

Prerequisite check (all of these were finished in P0/P1; if one is missing, go back and do it first):

- [ ] L1 leg ±45° pulse widths + measured `attach_deg` values are in `config.py` (tibia 89.3 — re-measured with the new
      suction part on 2026-07-19, direction also swapped; femur 49.7, see `LEG-GEOMETRY-OPEN.md` §2.11/§2.12;
      the coxa can just use the official -8, this rig is insensitive to it, see step 2. ⚠ after changing a structural part or refitting a horn the calibration is void and must be redone)
- [ ] `Pi5VacuumIO` implemented and the three P1 curves meet target (<1s to -40kPa; leak doesn't pass -20 in 60s; release <0.5s)
- [ ] The cup is installed in the `left-tibia-suction.stl` cavity, the door cover is glued on, and the whole-foot hanging-weight retest passed (P1 step 9)

---

## Step 1 · Materials (1–2 days, mostly things you have around)

| # | Material | Spec / qty | Notes |
|---|---|---|---|
| 1 | Vertical glass panel | ≥400×400mm, ≥5mm thick | backed by a wood board the whole time; tape the edges so they don't chip |
| 2 | Post and base | 2020 aluminum extrusion or wood beam | weight the base down or clamp it to the table with a G-clamp |
| 3 | Drawer slide (three-section) | 300mm, rated ≥10kg | or a linear rail; must mount vertically and slide smoothly with no play |
| 4 | Carriage plate | wood/acrylic ~150×100 | mounted on the moving side of the slide; the leg base and the weights both go on it |
| 5 | Lock pin | 1 door bolt | switches the carriage between "locked" and "released"; must be pullable one-handed |
| 6 | Weights | make the total sliding mass 1.5kg | includes carriage plate + leg + base; trim with water bottles / barbell plates |
| 7 | Spring scale | 0–5kg | for the 1kg lateral pull acceptance |
| 8 | Cushion pad | towel / EVA mat | at the bottom of the slide travel, to cushion a fall if the cup lets go |
| 9 | Corner brackets and screws | as needed | stiffness is half of whether this rig works |

## Step 2 · Rig geometry (half a day to 1 day)

```
        post(vertical)                glass(vertical, backing board)
          │ ┌rail                            ║
          │ │┌carriage──coxa servo case═══╗  ║   coxa output shaft horizontal, pointing at the glass
          │ ││   +weight             femur ╲ ║
          │ ││lock pin↕                     ╲║ ← cup pressed on the glass
          │ ││                          tibia║
          ┴─┴┴──cushion──────────────────────║──
```

1. **Glass vertical**, wiped clean (alcohol), backing board clamped behind it.
2. **Fix the coxa servo case to the carriage plate**: output shaft horizontal, pointing perpendicular at the glass;
   the case's mounting face (the equivalent of the robot's body plane) sits **90mm from the glass**. Cut slots in the base for ±20mm of adjustment.
   > Why 90 and not the "about 130" written in the ROADMAP: 130 was a rough number thrown together from the "foot working radius"
   > before calibration; recomputed with the measured calibration + the physical cup axis (δ=22.7°, see step 4) (`CLIMBING-DESIGN.md`
   > §6 table A), at a stand height of 90mm and a stance-phase reach of 170mm the cup axis is off by only 1.6°.
   > The 90mm in this guide is what counts, and the constants in step 4's script are computed for it.
3. **Leg plane vertical**: with the coxa centered (1510µs), the femur/tibia swing plane should be plumb
   (eyeballing it plus a phone level against the side of the femur is enough; ±3° doesn't matter).
4. **An uncalibrated coxa doesn't affect this rig**: the coxa axis is ⊥ to the glass, so a γ error only swings the foot along an arc across the glass surface,
   leaving the cup axis's attitude relative to the glass normal completely unchanged (that is exactly the basis for pushing the γ measurement to P3,
   `LEG-GEOMETRY-OPEN.md` §5). If the leg reaching out at an angle bothers you, trim the coxa `attach_deg` in `config.py`
   to straighten it; the value doesn't have to be accurate.
5. **Stiffness acceptance**: push the leg base by hand; movement in any direction <1mm. The press-down reaction is tens of newtons,
   and if the frame is soft the preload travel gets eaten by deflection — that is suspect number one when it won't attach.
6. **Geometry self-check** (don't mount the glass yet): after confirming the software with `--mock` from step 4, command the press pose on the real hardware
   and measure with calipers from the plane of the cup lip to "the plane where the glass will be" — the lip should be **past** that plane by about 15mm
   (that is the press interference). If it is far off, use the slots to adjust that 90mm distance.

## Step 3 · Carriage load-bearing mechanism (half a day)

1. Mount the slide vertically on the post, carriage travel ≥50mm, cushion pad at the bottom.
2. The leg base + weights all mount on the carriage plate, and the **total sliding mass is weighed to 1.5kg** (≈ the 2.5kg whole robot ÷ 3
   effectively load-bearing feet, plus margin). Keep the weights as close to the slide's axis as possible; off to one side they bind the rail.
3. The lock pin has two states: **locked** (carriage fixed = a rigid rig, for running cycles) /
   **released** (carriage free to slide down = the whole weight transfers to the attached leg, for the load-bearing acceptance).
4. Rehearse "pull the pin one-handed, set the pin one-handed" once; while pulling the pin the other hand does not touch the carriage.
5. Rehearse a let-go (empty carriage, no leg): release it and let the carriage drop free onto the cushion pad, confirming nothing gets smashed at the end of travel.

## Step 4 · Software and dry run (finish it while the rig is being built)

The software deliverable is already in place: `software/scripts/single_leg_wall.py`.

**Geometry constants** (at the top of the script, computed for the 90mm rig from step 2):

| Constant | Value | Meaning |
|---|---|---|
| `FOOT_X` | 169 | foot radial position: when pressing, the **physical cup axis** is ⊥ to the glass (0.1° off) |
| `Z_LIFT` | -70 | lift-off position, lip 20mm from the glass |
| `Z_CONTACT` | -90 | lip just touching the glass (= the hip plane's distance to the glass) |
| `Z_PRESS` | -105 | press position = contact plus another 15mm (13.5 of preload travel + 1.5 of preload, measured in `LEG-GEOMETRY-OPEN.md` §4.4) |

> ⚠ **Why FOOT_X is 169 and not the 122 you get from "K→P perpendicular to the glass"**: the physical cup axis runs along the square shaft,
> and differs from the model's virtual line K→P by a fixed **δ=22.7°** (the knee axis K is not on the square shaft, `LEG-GEOMETRY-OPEN.md`
> §2.13). Press with K→P ⊥ to the glass (122) and the cup jams into the glass at a 22.5° tilt, past the 15° tolerance, and simply will not attach.
> Rig drawing: see `html/en/p2-rig-diagram.html`.

Quick reference for changing the constants when the rig distance isn't 90 (H = the measured distance from the case mounting face to the glass):
`Z_CONTACT=-H`, `Z_PRESS=-H-15`, `Z_LIFT=-H+20`; `FOOT_X`: H=80→168, 90→169, 100→169.
Across the whole cycle the physical cup axis deviates ≤4.5° from the glass normal, inside the 15° tolerance measured in P1, with ≥44° of travel margin.

1. **Mock dry run** (dev machine or Pi, either works):
   ```bash
   cd ~/spider/software && source .venv/bin/activate
   python scripts/single_leg_wall.py --mock --cycles 3
   # expect: all 3 cycles succeed, hold pressure around -40kPa, 100% success rate
   ```
2. **Options**: `--cycles N` number of cycles; `--hold SECONDS` how long each cycle holds the attachment; `--csv PATH` for the detail log
   (t/cycle/phase/kpa/amp at 20Hz — the pressure curve and the servo current are both in there); `--release` to clean up.
3. **Retry on failure** is built in: SUCKING timeout → lift 5mm → press again, up to 3 times; if all fail, that cycle counts as a failure.
4. **Ctrl-C means "freeze", not "stop"**: the valve stays where it is and the servos stay enabled — a carriage hanging on an attached cup
   must not fall just because you pressed Ctrl-C. Always clean up with:
   ```bash
   python scripts/single_leg_wall.py --release    # vent + return to the lift-off position
   ```
5. **Pump policy**: the script disables the state machine's pump hysteresis (with P1's single sensor the tank pressure reading is off, which makes the pump run long and get hot)
   and replaces it with "pre-pump 1.5s before pumping down + top up whenever the hold falls below -45kPa". The pump duty cycle is therefore very low,
   so 50 cycles back to back is no overheating worry.
6. Before the first run on real hardware, go through the old P0 rules again: jumper cut, battery polarity, `get_throttled` = 0x0.

## Step 5 · Staged ramp-up (1–2 days)

Pass each level before moving to the next; if any level fails, fix it before going on:

| Level | State | Command / action | Pass criterion |
|---|---|---|---|
| 1 | Locked + no weight | `--cycles 1 --hold 3` | succeeds first time; the attach time on screen ≈ P1's <1s |
| 2 | Same as above | `--cycles 10` | 10/10 success (built-in retries allowed) |
| 3 | Locked + weights on the carriage | `--cycles 5` | no structural deflection, success rate doesn't drop |
| 4 | **Released, taking load** | a single cycle with `--hold 300`; pull the pin by hand after the screen confirms attachment | 1.5kg hangs for 5 minutes without letting go, pressure stable |
| 5 | Lateral load | while under load, pull the carriage horizontally with the spring scale to 1kg and hold 10s | doesn't let go, doesn't slip |
| 6 | Torque margin | look at the CSV: the current peak at the instant of pressing vs the baseline during hold | the peak visibly has margin; deepen `Z_PRESS` by another 3mm and re-measure the current increase as a reference |

- Level 4 operating discipline: **never pull the pin until the screen shows attachment confirmed (≤-30kPa)**; stand to the side after pulling it;
  venting is only allowed after the pin is back in.
- Level 6 note: the Servo2040 reads the whole board's total current, which is an approximation — what matters is the relative change, not the absolute value.
  The criterion for the 35kg·cm servos derated to 50%: if the current rise stays gentle after 3mm more preload (no steep jump, no odd noise, no
  heating), the wall pose is taken to have margin; a steep jump means it is already close to the stall region.

## Step 6 · The 50-cycle acceptance run and the decision (half a day)

```bash
mkdir -p ~/spider/docs/data
python scripts/single_leg_wall.py --cycles 50 --hold 5 --csv ~/spider/docs/data/p2_50cycles.csv
```

Run it back to back with the weights on and the carriage locked (the load-bearing item was accepted separately in step 5); afterwards, spot-check 5 cycles by releasing the pin by hand to re-verify load bearing.

**P2 acceptance checklist (decision gate)**:

- [x] >95% success over 50 consecutive attach/release cycles — **100% (50/50, zero retries)**, 2026-07-22, `docs/data/p2_50cycles.csv`
- [x] While attached, the carriage's 1.5kg hangs 5 minutes without letting go — pressure within [-81, -55] kPa throughout, about 4 top-ups, no burping, `docs/data/p2_1_5_kg_hold300s.csv`
- [x] Under load, a 1kg lateral pull neither detaches nor slips (measured and passed 2026-07-22)
- [x] Pressure curve + servo current CSVs archived into `docs/data/`; the torque margin conclusion is below
- [x] Rig photos/videos archived (`images/p2_wall_suck.mp4`, `images/suck_wall.jpg`, etc.)
- [ ] Stocktake of whole-robot structural print progress (what the printer produced in its idle time during P1/P2) — moved into P3, in parallel with the batch-2 shipping

**Acceptance data summary (2026-07-22, decision gate passed)**:

| Item | Measured |
|---|---|
| 50-cycle success rate | 100% (zero retries); worst hold pressure per cycle -69.7 to -71.3 kPa |
| press→hold time | median 2.74s, longest 2.77s (highly consistent across the 50 cycles) |
| 1.5kg × 300s hang | pressure never went above -55 the whole time (the -55/-65 top-up band was active, one top-up about every 75s); with the weight on, shear stretched the bellows and the vacuum passively deepened to -81 |
| **Torque margin conclusion** | press peak 0.73A vs the no-load baseline 0.07A and the hold baseline 0.16A — the peak is only transient and low in absolute terms, so there is plenty of margin. During the loaded hold it draws a continuous 0.61A (peak 0.90A), the servo carrying a steady load against the vacuum's pull-in force plus the weight; 300s with no odd noise or overheating, so acceptable. On the whole robot in P3, watch out for this continuous term ×3 feet |
| Known leftovers | at the instant of attachment the foot gets pulled a few mm toward the wall by the vacuum (the bellows flattens); harmless geometrically (axis off by <0.2°), ignored for now; the P3 gait has to tolerate this disturbance |
| Leak lesson | on 2026-07-22, with the -50/-60 top-up band, a leak around -53 ran away and burped up to -17 — leakage accelerates exponentially as the vacuum weakens; the top-up band has been changed to -55/-65 and a burp alarm added (a beep once it rises past -30) |

**Decision**:

- **Pass** → order batch 2 (`BOM.md`: 15 servos + everything else) → move to P3 (floor walking).
  The first thing to calibrate after the body is assembled in P3 is the coxa γ (method: `LEG-GEOMETRY-OPEN.md` §5, main line 1).
- **Fail** → upgrade to 40mm cups (normal force ×1.8) / parallel the dual pump heads and retest; still failing → re-evaluate the project
  concept, having spent only batch 1's few hundred yuan.

## Common problems

| Symptom | What to do |
|---|---|
| SUCKING times out every time, all 3 retries fail | The tank never pumped down (listen to the pump, check the 12V); not enough preload — measure the 15mm interference from the step 2 geometry self-check; dirty glass or dirty lip |
| Servos buzz and draw a lot of current while pressing | H measured wrong, so press goes too deep; the frame deflects and eats the preload (go back to the step 2 stiffness acceptance); temporarily make `Z_PRESS` 3mm shallower to localize the problem |
| Attaches fine but drops as soon as the pin is pulled | The vacuum barely scraped past the -30 line — look for leaks (redo P1 curve 2); tighten the script's criterion from -30 to -40 and re-verify |
| Carriage doesn't slide down after release (fake load bearing) | The slide is binding / off-center weights are jamming the rail — rehearse step 3 item 5 with an empty carriage |
| Slow slipping while under load | Shear is right at the edge: re-wipe the glass with alcohol; push the vacuum below -50; if it still slips, that is the 30mm cup's shear limit — consider 40mm |
| Cycle success rate hovers around 90% | Go through the pressure curves of the failed cycles in the CSV one by one: won't pump down = a sealing problem (geometry/cleanliness); pumps down then comes back up = a leak |
| The current column never changes | A fixed 1.0 under mock is normal; on real hardware check whether servo power is connected |
| Want to continue after Ctrl-C | Clean up with `--release` first to get back to a known state, then start `--cycles` again |
| What happens if power is lost while attached | Depends on the unpowered valve behavior measured in P1 step 3; a valve where "unpowered ≠ hold" vents right away and it falls — don't touch the power in the middle of a test |

## Safety

- **Never put hands or feet under the carriage and weights** at any time, and keep the cushion pad in place throughout; stand to the side of the rig when releasing under load.
- Never pull the lock pin before attachment is confirmed (≤-30kPa on screen); the pin must go back in before venting.
- Backing board behind the glass, tape on the edges; wear goggles for the hanging-weight and side-pull tests (a cup letting go snaps back).
- Ctrl-C not venting is deliberate — clean up only with `--release`, never by yanking the servo power.
- Same 12V/LiPo rules as P0/P1: power off before rewiring, never charge unattended, XT60 quick disconnect within reach.
