> English translation of [`docs/P4-GUIDE.md`](../P4-GUIDE.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# P4 whole-robot wall-climbing integration · detailed operating guide

Goal (weeks 15–26): **6 suction feet + the complete pneumatic system on the robot, the gait coupled to the adhesion state machine,
climbing a vertical glass/tile wall.**
Acceptance (ROADMAP): climb 1m up, translate 0.5m sideways, hang in place for 5 minutes with the power off without coming loose; safety line throughout.
The ground-to-wall transition is **not** part of this stage: the robot is put on the wall by hand (the transition move is P5).

Two tracks run in parallel in this stage: the **hardware track** (pneumatic bay assembly + pneumatics + electrical, gated by printing and deliveries) and
the **software track** (gait engine rework + adhesion integration, which can start any time on MockVacuumIO/MockDriver).
The software track is the real bulk of the work in this stage; start there.

Prerequisite check:

- [ ] Every box of the P3 acceptance checklist ticked (2m straight / 360° turn / 2cm obstacle / FK <5mm / print inventory)
- [ ] All 18 ±45° calibrations in `config.py` (2026-08-08 ✅, including whole-robot standing verified on the real robot)
- [ ] Whole robot (no adhesion) weighed into `weight-log.md` — **this one is still blank, do it first**; start weight reduction above 2.2kg
- [ ] P4 pneumatic bay V1 has passed its standalone validation (`validate_p4_bay_v1.py` 22/22, see P4-BAY-DESIGN §6)
- [ ] Vacuum tank has arrived (the V1 tank clamp waits on the real ear spacing, see P4-BAY-DESIGN §4-11)
- [ ] Pi environment healthy: `pytest` all green, `get_throttled` = 0x0

---

## Step 0 · Three things to settle first (they shape every job that follows)

**1. The gait engine needs a structural rework, not parameter tuning.** P3's `gait.py` is a pure open-loop phase table:
it assumes the ground is always at z0 and that touchdown happens exactly on a phase boundary; vertical speed happens to be at its maximum
at the instant of touchdown (≈170mm/s with default parameters, "slapping the ground"); the moment the phase boundary passes, the foot is dragged sideways. All three are the
opposite of what "land and seal" needs. P4 must make it **event-driven**: phase advance can be paused by each leg's
adhesion events, and the end of the swing phase becomes "decelerating vertical descent → press in → wait for vacuum confirmation".
See step 4 for details.

**2. The adhesion math must be redone with the measured weight.** P4-BAY-DESIGN §2.4 already threw cold water on this: the adhesion system's
implicit budget is 0.6kg, the rough estimate is 0.94–1.11kg, and the whole robot is estimated to land at 2.6–2.8kg. Measured on a single cup on vertical
glass: 25N normal pull-off, 15N shear (CLIMBING-DESIGN §omissions). Five feet in stance carry 75N of shear,
a safety factor of ≈2.7 against 2.8kg (28N) — still alive, but every part must be weighed into
`weight-log.md` as it arrives, and before going on the wall this math must be redone with the measured whole-robot weight and written into the acceptance table of this file.
**Filled back in 09-04**: the whole robot (with adhesion) measured 3537g = 34.7N; 75N of shear on five feet → safety factor **2.16**, 60N on four feet with one lost → **1.73**. The real robot has done every wall test at this weight; see `weight-log.md` for the conclusions.

**3. Three principles of servo protection** (the conclusion of the P3 servo discussion, running through all of the gait design):

| Principle | How it is applied |
|---|---|
| Transient external force is acceptable | Occasional tugs from adhesion, retries after a press that wasn't deep enough — no need to chase zero external force |
| **Steady-state fighting must be removed by changing the command** | PWM servos have no per-channel enable, no torque mode, no position readback; the only lever is to make the command follow the physics (preload travel, section 4.2); criterion = total current falls back to baseline after attachment |
| Never exceed stall | Derate 35kg·cm by 50% (the P2 measurement basis); a slow gait beats a fast one; never force a joint by hand while enabled |

The essential difference between the wall and the floor: on the wall a stance foot carries shear load **continuously** (P2 measured 0.61A for a single leg in
hold), so five feet in stance means ~3A of continuous current plus continuous heating. This is normal, not a fault —
the `walk_teleop`-style voltage/current status line must be kept, with peak and per-foot fall-back criteria added.

## Step 1 · Hardware track: first article and assembly of the pneumatic bay (1–2 weeks, in parallel with the software track)

Execute in the order of P4-BAY-DESIGN §5.4 / §6.3; only the check items are listed here:

- [ ] `p4-bay-fit-template-v0` on the robot, all six legs swung slowly through full travel with no interference (dynamic sweep, not optional)
- [ ] Test-fit the pump adapter plate / valve rail / sensor bridge / manifold clip one by one with real parts: assembles by hand, no wobble, not forced together by screws
- [ ] Re-check three things: the fore-aft offset of the 555 pump body relative to the bracket holes, how far the 0520B plastic nozzle actually protrudes, printed hole tolerance
- [ ] Vacuum tank arrives: first pull **-70kPa as a crush test** on the tank alone, then generate the tank clamp from the measured ear spacing
- [ ] Pre-fit all PCBs on the electrical deck with no power applied; check terminals / wire bends / vibration swing (V1 left 10 service zones)
- [ ] Weigh the assembled bay into `weight-log.md`; **single pump for the first flight** (-130g), the second pump position left open
- [ ] Battery stays in the official position on the belly (the belly faces the wall while climbing = CG closest to the wall, P4-BAY-DESIGN §0-1)

Connection topology per `html/en/p4-system-diagram.html` / `html/en/p4-pneumatic-electrical.html`;
build the divider board per `html/en/p4-divider-board.html`; wire the Pi per `html/en/p4-pi-wiring.html`.

Wiring discipline (carried over from P3, all mandatory): comb 12V power wires apart from analog sensor wires; no breadboards or
Dupont jumpers anywhere in the bay (intermittent disconnects under vibration = state machine misjudgment = fall); sensor wires run on an IDC ribbon that clips onto the Pi 40-pin header as one piece.

## Step 2 · Electrical and IO: `Pi5VacuumIO` from 1 foot to 6 (2–3 days)

> **2026-08-14, software side done**: items 1–6 below are all coded into `adhesion.py` (the GPIO table was
> jogged and measured on 08-14; the 7-sensor mapping follows p4_sensor_check; glitch suppression changed to a time-window
> criterion; tank pressure on its own channel + out-of-range judged as sensor failure, which stops the pump and sets `tank_fault`; arbitration field `mode` is a placeholder).
> Still to verify on the real robot: the timing of the whole chain in the 50Hz main loop (the ADS has already been raised to 860SPS + a 0.05s reading cache).

Today's `adhesion.py` is the P1 test-rig version: `VALVE_PINS = [5]`, one pump, one ADS1115. P4 extends it:

1. **Channel allocation**: the YYNMOS-8 has exactly 8 channels = 6 valves + 2 pumps (both pump channels are defined for the first flight, only 1 is wired).
   Freeze the GPIO pin table and write it into the `Pi5VacuumIO` class constants, matching the leg order in `config.py` one for one.
2. **7 pressure channels**: 2 × ADS1115 (0x48/0x49) give 8 channels, 6 foot pressures + 1 tank pressure.
   Mount the foot pressure sensor **between the valve and the cup (cup side)** — foot pressure can then be read even with the valve closed, and adhesion confirmation and
   hiccup detection both depend on it (the single-sensor lesson from P1 + the P2 hiccup measurement where 30s goes out of control).
   PGA must be ±4.096V (±2.048 clips in the vent-confirmation band, see the comments in `adhesion.py`).
3. **Per-channel electrical verification** (pneumatics not connected): the tool is `scripts/p4_sensor_check.py`; it only reads I2C
   and never touches GPIO, so it runs even before the pneumatics and the MOSFET board are installed. With no arguments it prints a self-test report (a verdict for each of the 8
   channels + consistency of the atmospheric point + noise), `--live` verifies the read chain while sucking, and `--wiggle N`
   catches intermittent breaks (see the next item). Test the valve side separately: every valve is audible when it pulls in, the pump starts and stops.

   **2026-08-11 measurement (six foot-pressure channels; tank and spare not connected)**: spread of the atmospheric point 12mV ≈ 0.3kPa,
   noise 1.0mV ≈ 0.05kPa. The deviation is far below the -30kPa criterion, so **one common `V_ATM=4.486` is enough,
   there is no need to calibrate each sensor** (the planned "table of 7" can be dropped; if you really want to calibrate, use `--zero --save`).

   The direction of the range follows the P1 measurement: **atmosphere 4.5V, -100kPa 0.5V, so after the divider the atmospheric point reads ADC ≈2.243V**.
   The line "atmosphere ≈0.25V" in the inspection checklist of `html/en/p4-divider-board.html` is backwards; checking against it will
   make you condemn a good board. An unconnected channel is pulled to ground ≈0V by the lower 10k arm, which is how you tell it from a live channel;
   the 8th channel has pads only with no divider fitted, so a floating AIN with a drifting reading is normal.

4. **Glitch suppression (new, mandatory)**: the state machine currently uses a single-sample criterion — one reading of
   `read_foot_kpa(i) <= ATTACH_KPA` in `adhesion.py` flips it to ATTACHED. **Change it to N consecutive
   samples all satisfying the criterion, or take a median filter**; the ATTACHED leak alarm (pressure recovering by >10kPa) gets the same treatment.

   The reason: foot pressure is the only basis for judging ATTACHED, and one false deep-vacuum reading makes the state machine believe that foot
   is stuck, so it clears the next leg to lift and five feet in stance become four. The criterion has to survive a single-sample glitch.

   **A misdiagnosis on 2026-08-11; the procedure is written down for reuse.** Symptom: S7's reading jumped whenever it was touched,
   then stopped jumping later; `--wiggle 7` caught 100 "intermittent breaks". I nearly went and unsoldered the joints; the real conclusion was
   **coupling from human static electricity, the board was perfectly fine**. A three-step elimination:

   | Step | Observation | Conclusion |
   |---|---|---|
   | `--wiggle`, look at direction | Strictly alternating positive and negative, symmetric amplitude | A real intermittent break is **one-directional** (with the wire broken, the divider node is only pulled toward ground by the lower arm); alternating = swinging around the baseline, not a disconnect |
   | `--scope`, look at amplitude | Maximum excursion only -2.8kPa | A real open circuit should slam to **-112kPa** (full scale). A wire is either broken or not; it does not break "a little bit" |
   | Separate "electrical" from "mechanical" | Pressing with a plastic pen barrel (force only) does nothing; a light touch after discharging static (charge only) reproduces it | Charge injection, not mechanical/piezoelectric |

   The decisive step was **comparing against another channel**: `--scope 2` behaved exactly the same → a board-level trait,
   nothing to do with S7's soldering. **Whenever you suspect one channel, measure a second one as a control first**; it saves a lot of rework.

   In flight the robot runs on battery and nobody touches it, so the human-static path does not exist and this phenomenon is no threat on the wall.
   But it proved one real thing: **the divider node really is sensitive to injected charge** — source impedance 10k∥10k = 5k,
   the 100nF corner frequency is 318Hz, which only blocks high frequencies. The real attackers in flight are **pump/valve switching transients and
   servo current surges**, which live in the same low band, so this must be re-measured after the pump and valves are installed in step 3 (see below).

   Tool usage: `--wiggle N` catches intermittent breaks (it automatically recognizes alternating signs and suggests switching to `--scope`),
   `--scope N` produces a waveform plot + spectrum; judge it by "direction + amplitude" — only fully one-directional with >20kPa peak-to-peak
   is a real intermittent break, and only then do you localize it in the seven steps "sensor body → wire exit at the root → middle of the harness → S-port connector →
   tap the solder joints → flex the board"; the fix is re-crimping the terminal or soldering directly with heat-shrink, **not "plug it in again"**.
   Software glitch suppression is the second line of defense; it cannot replace the first.

5. **The tank pressure channel trap**: `read_tank_kpa()` currently just returns `read_foot_kpa(0)`. With the vacuum tank
   not yet delivered and S1 empty, that channel reads ≈0V, which converts to **-112kPa**, lower even than `PUMP_OFF_KPA=-65`
   → the pump hysteresis thinks the tank has plenty of vacuum → **the pump never starts**. Fix this while extending to 7 channels:
   give tank pressure its own channel number, and raise a sensor-failure alarm when a reading falls outside the sensible -100 to +10kPa range.
6. **Arbitration field** (reserved for the far-future dual-surface hybrid foot): leave a `mode` field per foot as a placeholder in the gait-layer interface of
   `AdhesionController`; in this stage it is always `suction` and implements no logic.
7. The high-side relay for the servo main power (breaking the positive) was **settled on 08-15 to be driven by Pi GPIO17 (Pin11)**,
   no longer by Servo2040 A0/GPIO26: the module's jumper goes to **H** (pull in on a high level), the coil DC+
   takes the 5V/5A buck output (same source as the Pi, ~100mA), DC− shares ground, IN1 ← GPIO17.
   `driver.enable()` still sends the serial RELAY command (to keep the firmware enable), while the power switching is owned by GPIO17.
   The benefit: when the chica firmware locks up, the Pi can still cut servo power on its own; when the Pi process exits or reboots, the GPIO returns
   to input state → the relay releases automatically, a natural emergency-stop backstop. Cutting servo power does not affect the Pi or the pneumatics — this is
   the basis for graded emergency stop on the wall: cut the servos first (the NC valves hold the vacuum, the robot stays hanging), then a human takes over.

## Step 3 · Pneumatic assembly and leak testing (2–3 days)

The chain (one run from tail to front): pump → check valve → vacuum tank → tank-port tee (tank pressure sensor) →
manifold (1-in-6-out) → 6 × NC valve → pressure-tap tee (foot pressure sensor) → service loop over the coxa → suction cup.

- [ ] The whole chain on 4mm barbs + cable ties, epoxy at the joints (no hot glue)
- [ ] Leak-test in sections: first "pump → tank" pulled to -70kPa and held, then open each foot's valve in turn and test out to the cup (lip pressed on glass)
- [ ] Re-measure the whole-chain leak rate (the P1 basis: hold -40kPa for >60s without falling to -20kPa);
      **the upper limit of the 5-minute powered-off hang acceptance is set by this curve** — if it doesn't meet spec, fix the leaks before going further
- [ ] Leave a service loop (a slack turn) where each leg's vacuum line crosses the coxa joint; no line gets tugged with all six legs swinging through full travel
- [ ] Measure pumping time: a single foot from touching the wall to -40kPa (the P1 target is <1s); measure tank recovery time, which
      decides whether the second pump goes on (criterion: tank pressure returns to the PUMP_OFF hysteresis band within one gait cycle)
- [ ] **Re-measure electromagnetic compatibility (mandatory once the pump and valves are installed)**: `p4_sensor_check.py --scope N --secs 5`,
      with the pump starting and stopping and the six valves switching one by one during the run; see how much the foot pressure channels are polluted. This is the test that really has to be passed
      — the low-frequency sensitivity of the divider node measured in step 2 has exactly these switching transients as its attacker in flight.
      Criterion: the interference amplitude must be far below the -30kPa ATTACH criterion (leave an order of magnitude of margin).
      Order of treatment if it fails: ① confirm that every solenoid valve has a **flyback diode** (the inductive kick when an inductive load is switched off
      can reach hundreds of volts; the YYNMOS-8 usually has them built in, but confirm on the physical board) → ② comb the analog wires completely apart from the 12V power
      wires, adding shielding or twisted pairs if needed → ③ only then consider larger filter capacitors.

## Step 4 · Software track: reworking the gait engine (the core workload of this stage, 2–3 weeks, Mock first)

> **2026-08-14, implementation filled back in**: 4.1–4.5 of this step are all coded — `hexapod/climb.py`
> (ClimbEngine: event-driven segmented swing + phase pause + interlock + retry + leak freeze),
> `config.py` (per-leg `press_delta_mm` + the `climb_*` parameter group), `adhesion.py`
> (time-window criteria against glitches, is_leaking/leak_time, Pi5VacuumIO with seven sensors + tank-failure
> judgment, the mode arbitration field). The integration entry point is `scripts/climb_walk.py`
> (--mock/--air/--release), visualization with `sim_walk.py --gait climb`.
> Voice shell `scripts/voice_climb.py` (09-03): climb_walk runs unmodified inside a pty, and
> voice is mapped to injected keystrokes (emergency stop "停下" ("stop") → space, "往前走" ("walk forward") → w, "开始吸附/启动" ("start adhesion / start") → p,
> "单步/落地" ("single step / land") → i, "冻结" ("freeze") → f); the interlocks and the black box are unchanged; venting-class actions such as
> exiting or taking the robot down are keyboard-only (see VOICE-GUIDE §3.9).
> Items 1 and 2 of 4.6 are done (37 pytest all green + GIF eyeball check); 3 and 4 wait for the real robot.
>
> **2026-08-16, touchdown geometry fix (§4.3 implemented, found on the real robot while suspended)**: at the default stance
> reach=130, the physical cup axis at the press-in position is ~21° off the surface normal (beyond the ±15° tolerance; it looks like
> "the K→lip-center line is perpendicular"). ClimbEngine now **solves**, per leg with `cup_delta_deg=-25.2` (the L3-DISPUTE
> ruling basis), for the stance radius that makes "the cup axis at the press-in position ⊥ the surface" (≈176mm with default parameters,
> only 1.2° off at the instant of contact), and the touchdown band is clipped to a radius interval converted from a ±12° tolerance; the velocity command is
> globally limited to `climb_max_step=40` (an attached foot must not slip, so the real stance-phase displacement must be
> ≤ the stride limit — this matches the CLIMBING-DESIGN §6 prescription "reach 170, stride ≤40").

### 4.1 The swing phase changes from "one curve" to a segmented state machine

Each leg's swing phase is split into (working with the per-foot state machine in `adhesion.py`):

```
LIFT     Lift off while venting to atmosphere: the foot tip target retreats ≥15mm along the surface normal (the cup's 11–13mm
         rebound travel would push the foot back onto the surface, so it must vent while it moves)
TRANSFER Translate to above the touchdown point: keep the existing smoothstep + sinusoidal lift shape
DESCEND  Decelerating vertical descent directly above the touchdown point: XY frozen, approach speed cut to slow (tens of mm/s,
         value measured with the robot suspended) — kills today's slap where "vertical speed is maximal at touchdown"
PRESS    Press in by the preload travel press_delta (see 4.2): XY still frozen, slow;
         at the end, adhesion.request_attach(i) → PRESSING → SUCKING
WAIT     Wait for ATTACHED (-30kPa); phase advance pauses here, and failure to pull vacuum goes to FAULT:
         lift 5mm, back to DESCEND, retry; 3 consecutive failures → whole robot stops walking and alarms
LOAD     After ATTACHED: the foot tip target switches to holding the pressed-in position (4.2), the leg goes to stance
```

The stance phase and release are symmetric: move the CG away first (unloading that leg) → `request_release` → VENTING →
RELEASED → only then LIFT is allowed.

### 4.2 Preload travel and the two-state foot tip target (the key to killing steady-state fighting)

The three states measured in §4.4 (LEG-GEOMETRY-OPEN.md), converted into gait parameters:

| State | h_cup | Meaning |
|---|---|---|
| Free | 19–21mm **measured per leg** | Swing-phase geometry; the settled l3=123.7 is based on L1's 19 |
| Pressed to seal | 7.5mm | **press_delta = h_cup_free − 7.5 ≈ 11.5–13.5mm** |
| Vacuum -40 to -50kPa | 8mm | Only 0.5mm from the sealed state — **press to seal first, then pump; pumping barely moves it further** |

Implementation basis: **do not touch the settled `tibia_len=123.7`**; add a per-leg
`press_delta_mm` in `config.py`, and have the gait add that amount along the surface normal at the touchdown point as the foot tip target for the
press-in segment and the adhesion-hold segment. A two-state l3 and "target point plus delta" are mathematically equivalent, and the latter doesn't touch the settled calibration chain.
(Related part: `CUP_DELTA=-25.2` in `workspace_analysis.py` has been revised for the attached state
— L3-DISPUTE to-do g — use it to re-check the touchdown band.)

**Acceptance criterion (check it on every run on the real robot)**: within ~2s after ATTACHED and the target switch, the total current should
fall back close to the baseline of that stance configuration. If total current stays ~0.6A higher after a leg attaches, that leg's
press_delta wasn't fully consumed (re-measure that leg's h_cup) and the servo is fighting the vacuum.

### 4.3 Touchdown point and lip attitude constraints

- Lip tilt tolerance **±15°** (measured on the P1 test rig). A 3DOF leg cannot control the cup's orientation — tilt is
  determined by the geometry of the touchdown position, so this is a **touchdown selection constraint** (workspace layer), not something the gait can
  trim away step by step. Flat ground and a vertical wall satisfy it naturally near the design stance; a step length that puts the touchdown point outside the band is simply rejected.
- Pressing along the surface normal with XY frozen is a **hard gait rule** (settled in 4.1); otherwise the lip scrubs sideways and the seal fails.

### 4.4 Touchdown criterion

The plane's position is known (both floor and glass wall are flat), so the main route is **open-loop press-in + current watch**:
the DESCEND segment goes to the known plane height and then switches to PRESS, watching the total current increment throughout for two purposes —
early-contact warning (current jumps before the expected height is reached → stop) and touchdown confirmation. Total current resolution is
81mA/LSB, and a single leg's press-in increment is on the order of a hundred mA, so it is distinguishable; the threshold is calibrated by comparing suspended vs on the ground.
The foot pressure sensor is the second criterion: if pressure doesn't drop after the valve opens at the end of PRESS, the lip never sealed at all.

### 4.5 The CLIMB gait and the safety interlocks

- Use the CLIMB reserved at `gait.py:29` (5/6 duty, wave phasing, one leg at a time). In a tripod gait two groups of legs attach and release at the same time,
  which strains both the pump and the timing; don't use it for climbing.
- Interlock (stricter than the ≥4 in the `adhesion.py` comment; use the strict one for the first climb): **before lifting any foot, the other
  5 feet must all be ATTACHED**; adjacent feet never release at the same time (the CLIMBING-DESIGN §omissions constraint).
- ATTACHED leak alarm (pressure recovering by >10kPa): re-pump that foot immediately to save it; if 2s of that doesn't work →
  freeze the whole robot in its current pose + alarm (losing one of the five feet still leaves 4; the safety factor is 1.73 at the 3.54kg measured on 09-04).
- Pump hysteresis -60/-75kPa (deepened from -55/-65 on 2026-08-17: widening the band 10→15 stretches the interval between top-up pumping,
  and going deeper overall adds adhesion margin). ⚠ -75 sits right at the lower end of the 555 pump's nominal limit (-75 to -85), and with a check valve
  on every foot the cup-pressure floor = pump limit + valve cracking pressure — the measured cup pressure plateau must actually pass -75
  for this to count; if it can't, the pump never stops, and both thresholds must be moved back together to ≥5kPa on the shallow side of the plateau.
  Set the gait cycle to 3–4s at first (cycle_time is adjustable) and speed up once it runs smoothly.
- Slip compensation (`climb_sag_comp_mm` / `--sag-comp`, default 0 = off, added after the 2026-08-17 wall
  measurements): on a vertical wall the measurement is "every time a leg lifts, the whole robot gets dragged down a notch" — load is redistributed from 6 feet to 5,
  the elastic give (lateral deformation of the bellows cup + the servos' load-induced give angle) is locked in as a ratchet by the next attachment, and
  6 times per cycle can eat the whole stride = stepping in place. The compensation, during the LIFT+TRANSFER segment of each lift event
  (the swing foot is off the surface, no cup is pressing in), translates all stance feet at a constant rate by a set amount along the body frame's "downhill"
  direction (head up against the wall = -X, rotated with the integrated heading), pushing the body back up. The actual amount per event
  = min(setting, (40−half stride)/5): once the total outward swing at the tail of stance exceeds 40mm the leg leaves the IK envelope
  (capped at 4mm at full speed; at lower speed the stride shrinks and the cap loosens automatically); if the target leaves the workspace,
  climb_walk has a parachute — freeze and hover, so an anomaly doesn't blow up the control loop. Calibration basis: first mark a dot with a whiteboard marker next to a
  stance cup and walk 3 cycles to tell the loss types apart — **the mark stays put while the body sinks = elastic ratchet
  (compensable); the mark travels with the cup = interface slip (not recoverable, check cleanliness / cup pressure)**. Once the elastic type is confirmed,
  start at 2–3mm and increase step by step, watching the net displacement per cycle; take up to eighty percent of the measured sink. ⚠ Anything beyond
  the real sink drags the attached cup downhill along the surface, so err on the low side.

### 4.6 Development order (all on Mock first)

1. Run `MockVacuumIO + MockDriver` together with the new gait state machine and add pytest cases: normal cycle,
   SUCKING timeout retry, ATTACHED leak, interlock refusing a lift;
2. `sim_walk.py` produces a GIF for an eyeball check of the segmented swing trajectory (the vertical descent segment is visible);
3. Real robot, suspended: six legs walking the CLIMB gait in the air with the valves and pump really actuating (a cup in mid-air must FAULT,
   which conveniently exercises the retry and interlock paths);
4. Glass plate on the ground: go to step 5.

## Step 5 · Adhesive walking on the ground — timing integration acceptance (1 week)

Run the complete "press – pump – confirm – walk – release" gait on a horizontal glass plate (solidly supported, wiped clean). What this step validates
is **timing**, and the risk is zero (it can't fall); every box below must be ticked before going on the wall:

- [ ] 20 consecutive steps with no FAULT (or every FAULT recovered by an automatic retry)
- [ ] Total current falls back to baseline after every step's attachment (the 4.2 criterion), no leg fighting continuously
- [ ] Pumping time / tank recovery fit inside the gait cycle (if not: slow the cycle down or add the second pump)
- [ ] No "foot push-back" on the release side (the LIFT timing of retreating while venting is right)
- [ ] Walk and stop for 10 minutes with no servo hot to the touch (corroborating evidence that the fighting is gone and the load is normal)

## Step 6 · On the wall: progressive inclines → vertical (1–2 weeks)

Attitude principle: keep the body close to the wall (the closer the CG is to the wall, the smaller the peel moment on the upper cups; settled in the ROADMAP).

| Level | Surface | Pass criterion |
|---|---|---|
| 1 | 30° inclined glass | Climb 0.5m, no FAULT cascade |
| 2 | 60° inclined glass | Climb 0.5m + hold still for 1 minute |
| 3 | Vertical, starting 0.3m above the ground | Climb 0.5m |
| 4 | Vertical | **Acceptance: climb 1m, translate 0.5m sideways** |
| 5 | Vertical | **Acceptance: 5-minute powered-off hang** (servo power + pump power cut, the NC valves hold the vacuum; a person stands by to catch it) |

Safety line throughout every level (anchor at the top, climbing accessory cord left with a little slack, load-test the anchor first) and mats on the ground.
Sideways translation and climbing up use the same gait engine, only the velocity vector differs — the CLIMB gait itself has no preferred direction.

## Step 7 · Wrap-up and archiving

- [x] Whole robot (with adhesion) weighed into `weight-log.md`, with a conclusion against the 2.5kg limit — measured **3537g** on 09-04, 1037g over the limit; conclusions and open items in weight-log
- [x] Adhesion math redone with the measured weight, numbers written into this file (filled back into step 0, item 2) — safety factor 2.16 (five feet) / 1.73 (four feet), filled in
- [ ] Measured press_delta / h_cup of each leg into `config.py` with the date noted
- [ ] Measured curves for leak rate, pumping time and the powered-off hang stored in `docs/data/`
- [ ] P5 leftovers list: ground-to-wall transition, payload, dual-pump decision, enabling the dual-surface arbitration field

## Common problems

| Symptom | What to do |
|---|---|
| One leg times out in SUCKING often | Check in order: lip tilt (is the touchdown point at the edge of the working band?) → press_delta (has this leg's h_cup actually been measured?) → a leak in that channel's tubing/tee → clean the lip with alcohol |
| Total current doesn't fall back after attachment | press_delta wasn't fully consumed and the servo is fighting the vacuum; re-measure both h_cup states on that leg (cups vary 2mm between batches, don't apply L1's value to all of them) |
| Hiccup (foot pressure periodically recovers and gets pumped back down) | The old enemy from P2: a micro-leak. Clean the lip, check the seal of that foot's cup seat; a hiccup lasting more than 30s is treated as loss of control (the lesson measured in P2) |
| One foot pressure channel jumps occasionally / jumps when its wire is touched | **Don't jump straight to "bad contact"** (a near-misdiagnosis on 2026-08-11). In order: ① `--scope N` to look at the shape — a real intermittent break is a one-directional step that slams to around -112kPa; bidirectional spikes or a few kPa are not; ② measure another channel as a control, and if it jumps the same way it is a board-level trait, not this channel's problem; ③ pressing with a plastic pen barrel vs a light touch after discharging static, separating "force" from "charge". Only if it really is an intermittent break, localize it in seven steps with `--wiggle N`; the fix is re-crimping the terminal or soldering directly with heat-shrink |
| Freezes the moment the pump/valve moves, IO keeps failing (Errno 121) | The ADS1115 is knocked off the bus by 12V switching transients (measured 08-23: all three Errno 121 events fell within 0.05–0.6s after the pump/valve was energized, all self-healed in <1s, and the IO error count was 0 all session = the chip was healthy before it dropped off the bus; a steady TLM voltage proves nothing — that is the servo bus, it cannot see 12V/3.3V transients). The software already has a 0.5s tolerance + retry to ride over the glitch; if it recurs, check the hardware: ① flyback diodes on pump/valves ② supply decoupling of the ADC divider board (100nF+10µF) and the ground topology (don't share a path with the pump current) ③ keep the I2C wires away from the 12V bundle, add a twisted ground ④ a fully charged battery (the lower the boost board's input, the more violent the switching transients — the run that started at 7.72V blew up, the one at 8.03V was fine) |
| Tank pressure can't keep up with the step rate | Slow cycle_time down first, then add the second pump; a pump that never stops running = a systemic leak, go back to step 3 and leak-test |
| The foot is "stuck" to the surface and dragged during LIFT | Lifting before venting is finished (the VENT_TIME/RELEASE_KPA criterion was not waited out), or that channel's valve exhaust is blocked |
| On the ground walk_teleop lifts the legs barely at all, the cups don't leave the ground (only after the pneumatics were installed) | Not a weight problem: a de-energized valve = the tank-connected position, and the cup reaches the manifold through each foot's check valve, so pressing the body down squeezes air into the manifold and on lift-off the check valve won't let it flow back → the foot is locked by passive vacuum (about -40 to -60kPa on a 30mm cup ≈ 30–40N, more than the femur's pull at the foot tip). walk_teleop/voice_teleop `--vent auto` (default): the six valves are energized to exhaust while standing up and walking, and are de-energized 0.5s after walking stops and the robot settles (standing still shouldn't burn ≈25W in six coils); on start-up they are energized serially, so the feet don't lift for about 1s; `on` keeps them energized; `off` doesn't touch the valves, as a control (the v key cycles between them); any similar ground script that doesn't run the adhesion state machine but still lifts legs falls into the same trap |
| Stepping in place on the wall (the whole robot slips down a notch each time a leg lifts) | First tell where the loss is: mark a dot next to a stance cup on the glass and walk 3 cycles — the mark stays put while the body sinks = elastic ratchet: the 08-20 quantification put 83% of the slip in the instant of seal rupture during venting (venting slowly doesn't help), so the first choice is `--handover` (zero-force handover before the vent; starting value = measured single bounce × 0.8, docs/en/HANDOVER-DESIGN.md, calibrate δ with body_lean A/B in place); `--sag-comp` is a symptomatic layer that catches up after the drop (§4.5), and only one of the two runs during the handover A/B; the mark travelling with the cup = interface slip, clean the glass and the lip with alcohol and watch the "盘差" (worst cup pressure) field in the status line (cup pressure that can't be pushed past -75 with the pump never stopping = shallow vacuum, and both normal force and lateral stiffness are discounted); in tankless mode attachment is slow and creep takes a large share of the time, which improves noticeably once the tank is installed |
| The whole robot rocks at the instant a foot lifts | The CG wasn't moved far enough before release and that leg is still carrying load; check the support polygon computation |
| One servo gets hot after a few steps | Continuous load on the wall is normal, but one leg clearly hotter = uneven load sharing or residual fighting; find the leg by comparing current curves |
| Pi reboots on the wall | Undervoltage = a crash-level accident. Stop immediately, check the independent 5V supply and `get_throttled`, and stay off the wall until it is fixed |
| The whole robot dies at startup after the valve coils light up one by one (LED green → solid red, SSH drops, intermittent) | **Settled on the real robot 09-03 (lean_20260903_135325.log): the trigger is the servo relay closing, not the valves** — while the six coils were energized one by one, the 5V stayed flat at 4.88–4.89V with thr=0x0; `servo relay closed` (`舵机继电器已合闸`) is the last line, and it died <50ms later. Solid red = the halt state where the Pi 5's PMIC has cut power to the SoC (same as after `halt`; it needs a power cycle). The root cause is margin: the Pi's 5V input is only 4.89V unloaded (a healthy buck should give 5.05–5.15; `get_throttled=0` cannot show this kind of "not over the line but no margin left"), and 18 servos powering up at the same instant tug the shared 2S battery over the line. The root fix is in the supply: measure under load at the Pi's USB-C end and tune the buck output to 5.1–5.2V; swap in a short thick USB-C cable; put ≥1000µF electrolytics across both the buck's input and its output; verify that the servo return ground goes straight back to the battery negative and does not detour through the Servo2040's USB ground. Software mitigation (done): `enable(True)` is split in two — close the relay (powered, no torque) → 0.4s → firmware enable (torque), with the black box sampling the bus in each segment (`合闸后 x s` / `使能后 x s`, "x s after relay closure" / "x s after enable"), so the next death can be told apart as an inrush from powering up versus from producing torque. Forensics tools: run `bash scripts/pi_forensics.sh setup` once (it makes kernel logs persistent and must be run **before** the crash), then power-cycle after a reproduction and run `pi_forensics.sh check`; the black box writes `TLM 电源 5V=…` ("TLM power 5V=…") every 0.1s (`vcgencmd pmic_read_adc`; the user must be in the video group). The Pi 5 has no RTC battery, so the start time in the boot list is a fake clock saved last time (it can be off by hours); `check` back-computes the real boot time from the true timestamp of the last log line. **Added 09-04**: after the buck was swapped for an adjustable module set to 5.19V (the PMIC reads 5.105V, already at the level of the official supply), walk_teleop still died at startup (walk_teleop had no black box then; it has since been added, tag=walk/voice) — the static voltage is not the root cause; it is a sub-millisecond dip / ground bounce at the instant of relay closure, invisible to 10Hz sampling and to a multimeter; `/proc/device-tree/chosen/power/power_reset`=2 was read both times, which per the Raspberry Pi engineers means the previous boot was cut by the PMIC because of low voltage (the PMIC's own testimony), and it should be 0 after a normal reboot, which gives you a control. The battery is an unprotected 2S RC LiPo with no protection board, so there is no over-current trip and battery sag is not the prime suspect; **verdict on the evening of 09-04 (case file `html/en/pi-crash-ground-loop-20260904.html`)**: ① the user found that every walk_teleop run turns the sticky bits from 0x0 to 0x50000, even when nothing dies — the intermittent fatal event now has a deterministic non-fatal stand-in, so the experimental criterion becomes "do the bits get set or not"; ② walk_20260904_215456.log: 14ms after GPIO17 went high, thr=0x50005, with the servo bus at 0.00A at that moment; after enable it drew 6.67A and the battery went 7.71→7.31V while the Pi's 5V didn't budge — it is not the servos' large current, it is the instant of relay closure; ③ an A/B within a single boot, closing the relay with the GPIO17 script only: with the Pi↔Servo2040 USB cable unplugged, ten closures all gave 0x0; plugged back in, the very first one gave 0x50000 — **the path is the USB ground loop**; ④ the new buck is an XL4015 5A CC/CV board whose back-side R050 low-side sense resistor sits in the Pi's ground return, and its blue LED flashing green at the instant of closure = the module itself also sees this pulse (an aggravating factor; the old fixed module had no such resistor and died too, so it is not the root cause). Mechanism: the contacts closing charge the capacitance of 18 servos + the Servo2040 with a hundred-microsecond, tens-of-amps pulse that creates a voltage difference along the servo return ground; part of it flows through the USB ground → the Pi's ground plane → the GND Dupont jumper → R050 back to the battery negative, and the drop across the jumper + R050 looks to the PMIC exactly like a 5V dip. Remedies (not installed yet; the criterion is still the sticky bits): a USB isolator (full-speed, ADuM3160 class) is the cleanest; pre-charge / soft start on the servo rail (a small relay in series with 4.7Ω 5W closing 100ms first, or a P-MOS soft start; a manual pre-charge with 10Ω across the contacts can verify it first); a star ground can only reduce it. A capacitor across the Pi's 5V is only symptomatic (2200µF at 0.7A holds 0.4V for about 1ms) |
| Energizing the valve makes it fail to hold | The NC valve's ports are connected backwards (inlet / outlet / exhaust) or `VALVE_ON_LEVEL` has the wrong polarity |
| Pressure drops fast during the powered-off hang | Localize it with section-by-section leak testing; add epoxy at the joints; clean the lips; re-check the tank volume |

## Safety

- **The safety line never comes off**, including the inclined levels; load-test the anchor with a human drop first; leave the line slightly slack so it doesn't affect the gait.
- Cover the whole area directly below the working surface with mats; first climb at 0.3m height.
- During the powered-off hang test a person stands beside the robot, hands ready but not touching.
- Never force any joint by hand while the servos are enabled (35kg·cm strips gears); cut servo power before adjusting the pose —
  the NC valves keep the adhesion, so the robot won't fall.
- For any anomaly on the wall (hiccup, FAULT cascade, odd noise, heat), freeze the pose and observe first; don't rush to keep walking;
  a frozen attached state is the safe state, moving around blindly is what's dangerous.
- LiPo rules same as P0–P3: never leave it charging unattended, keep the XT60 within reach to yank, power down before rewiring.
