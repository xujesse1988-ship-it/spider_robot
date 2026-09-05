> English translation of [`docs/HANDOVER-DESIGN.md`](../HANDOVER-DESIGN.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Zero-force handover before vent · implementation design

Status: **implemented** (design frozen 2026-08-20 and implemented the same day:
`hexapod/climb.py` update() step 4.7 + `LegConfig.handover_mm` + `--handover` in
both scripts; all 8 items of the §7 test list + the mock smoke test pass.
Added during implementation, outside the design: CLIMB_COLOR in
`scripts/sim_walk.py` also picks its color by phase, so the HANDOVER key was
added there too — the same family of KeyError hazard as PHASE_CH in §3.2,
missed by the design.
**To do: the §9.3 on-robot A/B acceptance** — the experiment plan and the
quantification tool are ready:
`docs/en/HANDOVER-AB-PROTOCOL.md` + `software/logs_analysis/ab_quant.py`).
2026-08-24, multi-swing generalization (docs/en/DUAL-SWING-DESIGN.md, dual-leg
crawling): `_ho_left` is now stored per leg (two handovers can overlap),
apportionment is now computed live each tick from the current STANCE set
(bit-identical to the old frozen version in the single-swing case), δ=0 also
goes through HANDOVER first (falling straight through to VENT on the same tick,
unobservable between ticks), and venting gained three gates — a window-head
order queue, VENT_STAGGER_S staggering, and a re-check of the gate. The
"one leg at a time, no per-leg storage needed" wording in §3.4/§3.5 below is
archival only; for the current implementation see step 4.7 in climb.py.
Same day, v1.1: the weights (§5 v1.5) were unlocked for dual swing — after the
single-swing measurements showed the weights clearly improve slip, shares are
now "valued by rotation distance, then normalized in place over the current
stance set" (`_share_now`, computed live each tick; with single swing there are
always 5 stance legs, so it is bit-identical to the frozen table), and the
presets are split by convention: dual swing recommends 1,1,1,0,0, and the
aggressive single-swing preset must not be copied over (DUAL-SWING-DESIGN
§3.5 / appendix C model).
Measured basis: `html/en/vent-snap-20260820.html` (quantification report of the
08-20 stepping-in-place experiment),
`software/logs_analysis/lean_20260820_102117.log`, `images/lean_20260820.mp4`.

---

## 1. The problem to solve (one paragraph)

With zero commanded motion, lifting and landing one leg at a time in place, the
robot slips 57→74mm down the wall per round; frame-by-frame quantification
shows **83% of the slip happens in the instant the vented seal ruptures**
(during the 0.4s bleed-down after the valve opens the body moves <0.15mm; after
rupture it drops all the way in one step within <100ms, with overshoot and
ringing) — the elastic energy the lifted leg has stored since it attached
(every 1mm the body sinks winds that leg up by 1mm) is released at the instant
of detachment, the body falls until the extra deflection of the other five legs
catches it, and the next attachment locks that in as a ratchet. Press touchdown
accounts for only 4% (under the vent-first ordering, press-lurch has become
secondary). Whole-robot current climbing 0.73→~1.9A cycle by cycle without ever
falling back is the electrical evidence that internal stress is accumulating
between the legs. **Slow venting has been ruled out by the data**: the force
release does not follow the bleed-down gradually, it is all compressed into the
instant the lip peels off — the only remedy is to make sure that leg carries no
force at all by the time it peels.

Single-event bounce per leg (round 2, mm): L1 20.9 / R1 19.2 / L3 13.5 / R3 11.8 /
R2 5.9 / L2 5.8. The front legs (the upper legs, where the peeling moment
concentrates) are the largest. A single leg's bounce caps out at ~21mm (servo
torque saturation limits how much one leg can store), which is why climbing
still nets 26% forward instead of going backwards.

## 2. Principle

**An attached foot cannot slip ⇒ changing the command changes force, not
position.** That is the physical basis of the whole scheme, and it is exactly
the same property as the existing VENT segment's "stay on the surface and
translate with the support field" (the engine already has the precedent,
update() step 4).

One-dimensional spring accounting (one spring k per leg, y_i = anchor −
command = the body position that leg "implies", X = mean(y_attached) −
mg/(n·k)): the necessary and sufficient condition for lifting leg j with no
jump is **f_j = 0**, i.e. y_j = mean(the others) − mg/5k. How: **move the
lifted leg's foot command δ back along the "uphill" direction (unloading
itself), and at the same time move each of the other five stance legs'
commands an extra δ/5 along the "downhill" direction (taking over the load)**
— the mean of the six y values is unchanged ⇒ the body does not move at all;
y_j alone drops by δ ⇒ f_j goes to zero. Then vent: at seal rupture there is
no energy left to release. 1-D model check (appendix A): baseline net advance
25.8% (matching the measured 26%) → 100% with the zero-force handover;
stepping in place −35.6mm/cycle → 0.

Three engineering properties:

- **The other legs never carry more force than they would have had to carry
  after the lift anyway** — the handover only moves the load transfer earlier
  and makes it lossless; there is no new worst case;
- The command geometry is self-consistent and bounded: over one cycle a leg
  takes 5 doses of δ/5 as a stance leg (−δ downhill in total), and when its own
  turn to lift comes, +δ of unloading brings it back to about its nominal
  station, and it lands at the nominal station again — commands do not diverge,
  and the largest extra outward excursion of a single leg is ≈ δ (workspace
  accounting in §4.7);
  ⚠ **"mean-invariant ⇒ the body does not move at all" only holds
  approximately under a per-leg δ table** (found by review and measurement):
  landing puts the lifted leg back at its default nominal station = its
  accumulated handover offset is thrown away, and "brings it back exactly" is
  exact only under a uniform table. Under a per-leg table (L1 17 … L2 5) a leg
  takes in ≈ mean(the other legs' δ) during its stance but gives back δ_i when
  its own turn comes, and the residual is swallowed by the reset at the instant
  of touchdown — the mean of the six commands takes a step at every touchdown
  (L1 landing ≈8mm uphill, L2 ≈6.4mm downhill). Measured on the real engine
  with the lift order of the protocol: ~4.8mm of accumulated uphill command
  artifact in round 1 (peak 10.7mm while walking), ±1.3mm of swing within a
  round (0.53mm with a uniform table); **the steady-state drift per round is
  still 0**. For the reading discipline see the note on criterion 4 in §1 of the
  A/B protocol and the round-1 branch in its §6;
- The direction follows `_down` (the downhill direction, rotating with the
  integrated heading, the same machinery as sag_comp): in this experiment
  lateral drift was only 7.6mm out of 131mm, so the single-axis longitudinal
  assumption holds.

## 3. Mechanism design

### 3.1 Configuration

- `LegConfig.handover_mm: float = 0.0` — the per-leg handover amount δ (0 = off,
  off by default; a per-leg field like `press_delta_mm`). It goes in LegConfig
  rather than RobotConfig: measurements differ by 4× between legs (L1 17 vs
  R2 5), so a single global value is unusable.
- `RobotConfig.handover_rate_mms: float = 10.0` — the rate at which the
  handover is applied (parameterized since 08-26, single source in config;
  `climb.HANDOVER_SPEED_MMS` is demoted to a default alias for tests and for
  duration accounting in the docs). The default 10 is the same order as
  LEAN_SPEED_MMS (commands to an attached foot must move slowly, to leave
  quasi-static time for the load redistribution); at δ=17 the handover segment
  is ~1.7s, and with the δ table at 22–33 it is ~2.7-3.8s. The engine
  `__init__` guards 0<rate≤50 (nan/inf rejected) — this also catches paths that
  set config directly instead of going through the CLI (same lesson as the
  weights). **D′ discrimination experiment = 20**: the T1 fit
  (html/en/stiffness-fit-20260826.html) found that ~nine tenths of the handover
  segment is "common-mode sag ≈ creep rate × window duration", but the window
  duration ≈0.5+δ/rate is perfectly collinear with δ, so raising the rate is
  the only offline discriminator between "time creep" and "per-mm loss"
  (protocol §9).

### 3.2 New phase

`LegPhase.HANDOVER = "handover"`, inserted between the window-head decision and
VENT:

```
STANCE →(window-head decision releases)→ HANDOVER →(δ fully applied, request_release)→ VENT → LIFT → …
```

⚠ `runlog.PHASE_CH` must get `"handover": "Z"` (Z for zero force) — both
status_line and the ClimbWatch telemetry look this table up, and a missing
entry = a KeyError that blows up the main loop / black box. ClimbWatch's
phase-transition log line (`phase X stance→handover`) goes through the generic
`.value` path and works automatically.

### 3.3 Window-head decision changes (update() step 2, else branch)

Everything that branch does today **stays exactly where it is**: the `landing`
computation, advancing the rotation (step_leg), the `_swung_since_go`
bookkeeping, loading sag_comp, `step_pending→step_active`,
`_slot_active = True`. The only change is that the closing action forks:

```python
if self.cfg.leg(cur).handover_mm > 0.0:
    self._ho_left = self.cfg.leg(cur).handover_mm
    self.phase_of[cur] = LegPhase.HANDOVER      # handover first, do not vent
else:
    self.ctl.request_release(LEG_NAMES.index(cur))
    self.phase_of[cur] = LegPhase.VENT          # current behavior, not one character changed
```

`request_release` is **deferred until the handover completes** — during the
handover the cup must stay sealed and attached (the ATTACHED control loop keeps
running, and the interlock and leak watchdog apply to it as usual).

### 3.4 The handover motion (new update() step 4.7, right after 4.6 body lean)

The same class of global step as 4.5 (sag compensation) and 4.6 (body lean),
which "directly modify foot targets":

```python
# 4.7 Zero-force handover: the lifted leg gives back δ uphill (unloading), the other stance legs each take δ/n downhill (taking the load),
# mean-invariant = the body command does not move; applied at constant speed in real time (load redistribution is a quasi-static process),
# paused during leak rescue (a leaking cup has little friction margin and should not be pushed). Only when fully applied does it vent into VENT.
if (not leak_pause and self._slot_active and self._slot_leg is not None
        and self.phase_of[self._slot_leg] == LegPhase.HANDOVER):
    cur = self._slot_leg
    step = min(self._ho_left, HANDOVER_SPEED_MMS * dt)
    self._ho_left -= step
    dx, dy = self._down
    sup = [n for n in LEG_NAMES if self.phase_of[n] == LegPhase.STANCE]
    self.foot[cur][0] -= dx * step          # opposite of downhill = uphill = unloading direction
    self.foot[cur][1] -= dy * step
    for n in sup:                           # use the actual stance count, not a hard-coded 5
        self.foot[n][0] += dx * step / len(sup)
        self.foot[n][1] += dy * step / len(sup)
    if self._ho_left <= _EPS:
        self.ctl.request_release(LEG_NAMES.index(cur))
        self.phase_of[cur] = LegPhase.VENT
```

`_step_swing` gets one `elif ph == LegPhase.HANDOVER: pass` branch (the motion
is driven by 4.7; `_seg_t` keeps timing as usual for diagnostics). No separate
timeout watchdog is needed: at a fixed rate it must finish, and the only thing
that can stall it is leak_pause, which has its own `leak_rescue_s` freeze as a
backstop.

### 3.5 New engine state

`__init__` gets `self._ho_left = 0.0`. The one-leg-at-a-time window order
guarantees at most one handover is in flight, so there is no need to store it
per leg.

## 4. Interaction with existing mechanisms (item by item, all of them traps)

1. **⚠ Following the support field (the biggest trap)**: today update() step 4
   only translates the `(STANCE, VENT)` feet with the support field and holds
   their press-in depth. During HANDOVER the foot is still stuck to the wall
   and, in walking mode, the body is moving, so it **must follow along too**;
   otherwise that foot is dragged across the wall frame, scuffing a lip that is
   still sealed. Change it to `(STANCE, VENT, HANDOVER)`, and the line that
   holds z at the press-in depth must cover HANDOVER as well. The handover
   displacement (4.7) is added on top of the field translation, on the same
   basis as sag_comp is added.
2. **Phase clock**: HANDOVER is not STANCE, so step 3 takes the
   `adv = min(dt, window remaining)` branch — in climbing mode the 0.6s window
   head cannot cover a 1.7s handover, so the clock stops at the end of the
   window and waits for it, and the cycle simply stretches (the same semantics
   as the existing "clock stops waiting for the attachment event"; no new code
   needed). Lifting in place (zero speed) has the clock idling anyway, so no
   effect.
3. **sag_comp (4.5)**: it is injected only while the slot leg is in
   LIFT/TRANSFER, so nothing is injected during a handover — naturally mutually
   exclusive. **Once the handover works, sag_comp should be zeroed out and
   disabled** — it is the symptomatic layer that "chases the drop after the
   fact", and running both pushes the stance system twice and burns workspace
   for nothing (during A/B only one of them is on).
4. **Body lean (4.6)**: lean only advances when all legs are STANCE/HOVER, so
   HANDOVER pauses it automatically — no code change, and the behavior is right
   (no whole-body translation should be added on top during a handover).
5. **Leaks**: 4.7 has a `not leak_pause` gate — the handover pauses without
   losing progress, the same no-go zone as 4.5/4.6; a rescue timeout goes to
   the existing freeze.
6. **Freeze / unfreeze**: when frozen, update() returns early at the top, so
   the handover is held automatically (all targets frozen). `clear_freeze()`
   does **not** cancel a handover in flight (same convention as "step_active is
   kept while swinging": this leg is past the window-head decision and must be
   carried through to closure; and the handover is a slow, reversible motion,
   so resuming and finishing it is harmless). Note the difference from "an
   unfinished lean is canceled": a lean is a purely human request and can be
   dropped, whereas the handover is part of the lift itself.
7. **Workspace accounting**: the largest extra downhill excursion a stance leg
   gets from handovers is ≈ δ (5 doses of δ/5 accumulated over one cycle, paid
   back only when its own turn to lift comes).
   - body_lean in-place experiment (zero speed, nominal station, stand 62):
     margin ≥80mm, so δ≤20 can be used freely, no extra guard.
   - climb_walk (walking): δ has to go into the stance-tail budget (today's
     budget = half stride + 5×sag_comp + the VENT tail that follows the field,
     see `comp_tail`). For v1 the docs simply declare "do not combine a large
     stride with a large δ", and the startup print puts δ_max and
     `comp_tail − half stride` side by side; folding δ into the
     `max_straight_step` formula and into the dynamic sag_comp allowance is
     left until the handover has proven itself in experiments (do not rewrite
     the budget formulas for a parameter that is not calibrated yet).
   - The lifted leg's own +δ is uphill, which is the direction it is about to
     swing anyway, so it costs nothing in the budget.
8. **Interlock / gate**: the window-head decision has already checked (the
   other 5 feet ATTACHED and their cup pressure deeper than lift_gate_kpa)
   before HANDOVER is entered; if a stance leg turns bad within the ≤2s of the
   handover it goes down the leak path (see 5). The interlock is not re-checked
   when the handover completes — same convention as today's "no re-check from
   VENT onwards".
9. **request_lift / single step / continuous walking**: all three paths go
   through the same window-head decision, so the handover covers all of them
   automatically; in `air_mode`, where nothing attaches, the handover still
   runs (harmless, just costs time), and no bypass is added.
10. **Startup sequence / RETRY_LIFT**: the startup presses each foot in without
    lifting any, and when a retry lifts a foot back up the cup is already FAULT
    and not attached — neither has stored energy to release, so neither goes
    through HANDOVER (the code path naturally does not; just confirm it).
11. **Telemetry**: add `"handover": "Z"` to `PHASE_CH` (§3.2); the black box
    records phase transitions automatically. Optional: add the remaining
    handover amount to the TLM tag (`ho_left=X.X`), decide when implementing.

## 5. δ calibration

⚠ **2026-08-24 correction to the calibration semantics (three in-place A/B
groups measured; the table below is archival, do not use it any more)**: the
three groups δ=0/12/24 (`lean_20260824_13*`, quantification pipeline +
step-based synchronization) show the per-leg bounce is strictly linear in δ,
and extrapolating to zero bounce needs **δ*≈28–42mm per leg** (L1 39 / R1 41 /
L3 37 / R3 28 / R2 28 / L2 30, fitted on round 2, on a scale-free basis; slopes
−0.57 to −0.13, a 4× spread between legs = the difference in leg-chain
stiffness). The "bounce ×0.8" starting method in the table below mistook the
bounce at the instant of venting for the stored energy — the bounce is the
stored energy read out after the other five legs' parallel stiffness has caught
it (≈ stored/5–8), systematically 4–8× too small, and that is exactly the
accounting behind δ=24 pressing against the old upper limit while every leg
still bounced down (slip per round 66→50→39mm). The new calibration method:
**run three groups δ=0/half/full, extrapolate the line, and start at δ*×0.8**,
with the parser's upper limit going 25→45 accordingly (§6).

Starting value = the 08-20 measured single bounce ×0.8 (under-shoot rather than
over-shoot, the same discipline as sag_comp):

| Leg | L1 | R1 | L3 | R3 | R2 | L2 |
|----|----|----|----|----|----|----|
| Measured bounce mm | 20.9 | 19.2 | 13.5 | 11.8 | 5.9 | 5.8 |
| **δ starting value mm** | **17** | **15** | **11** | **9** | **5** | **5** |

- These numbers are steady-state values for the **stepping in place, stand 62,
  trim 6** condition; in walking mode the support field keeps winding the legs
  up, so the front legs may really store more — calibrate first with body_lean
  in place (A/B), then fine-tune with climb_walk single steps (the `i` key),
  and only then walk continuously.
- The cost of overshoot: a δ larger than the real stored energy pre-loads the
  lifted leg in the opposite direction (the body hops up a little on release)
  — a small overshoot is harmless or even helpful, a large one burns workspace
  and kicks backwards; start at ×0.8 and add step by step.
- v2 direction (not this round): watch whole-robot current during the handover
  (the experiment shows the 0.73→1.9A signal is large enough, and 0.5s sampling
  is usable) — current dropping to a plateau ≈ unloading complete, which could
  be made into an adaptive δ.
- **v1.5 (implemented, off by default): the load shares are weighted by
  rotation distance in the window order** `--handover-weights` (5 weights:
  w1 = the stance leg whose turn to lift is farthest away = the one that just
  lifted, w5 = the one due to lift next; with no value = the aggressive preset
  0.6/0.25/0.1/0.05/0). Model numbers from appendix A: with uniform shares the
  steady-state stored energy per lift is 29.7; giving more to the legs whose
  turn comes latest — mild 24.1 (−19%), aggressive 20.2 (−32%), all on one leg
  29.7 (the benefit goes to zero, **non-monotonic**), reversed 38.7 (+30%,
  worse). Mechanism: the zero-pre-load lock-in at every touchdown re-spreads
  the force pattern — load received early is diluted back into the collective
  by the later touchdowns, while load received late stays intact and
  accumulates until that leg lifts itself, so the leg with the most dilution
  opportunities should take more. Under any normalized weights, mean invariance
  (the body command does not move) and zero slip per round still hold (verified
  numerically). ⚠ The ordering is by distance in the window order, not by "how
  recently the leg landed" — that is correct from the very first lift and does
  not depend on the attachment history (historical lesson: the first version
  sorted by landing recency, and back then the startup attachment order was
  still L1..R3 ≠ the window order, so the first round gave the shares to the
  wrong legs; the user spotted it and it was reworked. The startup attachment
  order was later also changed to the window order R3→L1→R2→L3→R1→L2, so
  landing recency and the rotation are aligned from startup on, but the point
  that the ordering is independent of the attachment order still stands); the
  cost = the convention assumes the legs are lifted in rotation.
  **Rotation-convention guard (v1.6 review hardening)**: when the lift order
  deviates from the rotation (lifting the same leg over and over to calibrate
  δ, for example), applying the weights blindly dumps the w1 share on the same
  stance leg every time and never gives it back (the touchdown reset only
  clears the lifted leg itself) — with the aggressive preset at 0.6δ per lift,
  4–6 lifts blow out the workspace in practice. At the start of each handover
  the engine checks "this lifting leg = the window-order successor of the
  previous lifting leg"; if not, that handover falls back to uniform δ/5 and
  leaves a handover_note trace (the scripts print it). ⚠ Turning the weights on
  changes the steady-state operating point, so the δ table has to be
  re-calibrated downward (expect −20 to −30%); do that A/B separately from the
  δ calibration.

## 6. Script interface

Both scripts get a `--handover` argument, in one of two formats:

- `--handover 8`: δ=8 uniformly for all six legs;
- `--handover L1:17,R1:15,L3:11,R3:9,R2:5,L2:5`: per leg (you may give only
  some legs; the ones left out stay 0).

After parsing, `replace(leg, handover_mm=…)` goes into cfg (copying the replace
pattern of `--press-delta`). Range check 0–45mm (since 2026-08-24: the
three-group extrapolation gives δ*≈28–42, plus margin; the old limit of 25 came
from "the bounce caps at 21 ⇒ anything larger is pointless", which is the
bounce-as-stored-energy mistake, see §5 ⚠ — the workspace is guarded separately
by climb_walk's hard refusal at startup and by the per-tick clipping in step
4.7). The startup print shows each leg's δ and the handover segment duration.
The body_lean doc header / key list gets one more line: "a zero-force handover
is done automatically before lifting (when --handover is on)". Same for
climb_walk.
Both scripts also have `--handover-rate` (since 08-26): the application rate in
mm/s, default 10 = the n=3 baseline convention, and it requires `--handover` to
be on as well; the CLI checks 0<r≤50 and the engine checks again, and the rate
is written into the log parameter line (`handover_rate=`) so the quantification
can trace it. D′ discrimination experiment = 20 (protocol §9).

## 7. Test list (tests/test_climb.py, following the existing style)

1. **Regression**: with `handover_mm=0` (the default CFG) the behavior is
   byte-for-byte what it is today — the existing 73 tests passing is the
   assertion itself, nothing new to write.
2. **Phase sequence**: with δ>0 the window-head decision goes into HANDOVER,
   during which `ctl.state` stays ATTACHED and the valves do not move; only
   after δ/HANDOVER_SPEED_MMS seconds (±3 DT) does request_release fire and the
   phase go to VENT (copy the structure of test_vent_before_lift).
3. **Displacement accounting**: when the handover completes, the lifted leg's
   target = start + δ uphill and each stance leg = start + δ/5 downhill, with
   the y component following the _down convention and z staying at the press-in
   position; the vector sum of the six displacements = 0 (the algebraic
   assertion of mean invariance = the body command does not move).
4. **Walking superposition**: walk with a velocity; during HANDOVER the lifted
   leg follows the support field (copy the XY-follows-field assertion of
   test_vent_before_lift, with the expected value including the unloading
   component).
5. **Leak pause**: inject a stance-leg leak during the handover; _ho_left
   freezes and does not advance, and nothing vents; after a successful rescue it
   resumes and finishes.
6. **Freeze recovery**: inject frozen during the handover; targets are held;
   after clear_freeze it resumes, completes, and finishes the lift-and-land
   normally (compare test_single_step_hover_survives_freeze).
7. **Heading rotation**: after wz has turned by θ, the handover direction
   rotates with _down (copy
   test_sag_comp_downhill_rotates_with_heading).
8. **IK solvable throughout an in-place lift-and-land**: request_lift + δ=20 at
   the limit; pulses stay solvable throughout, and the stance feet do not move
   at all apart from the handover component (compare
   test_lift_in_place_hover_and_land_back — note that this test's existing
   "stance feet do not move at all" assertion has to become "moves only by the
   handover component" when δ>0).
9. **δ parsing**: both argument formats at the script layer + rejection of
   out-of-range values (if the parser lives outside the engine, put the test on
   the script side or unit-test the parser directly).

## 8. Implementation list (file by file)

| File | Change |
|------|------|
| `hexapod/config.py` | LegConfig.handover_mm field + comment (citing this document and the report) |
| `hexapod/climb.py` | HANDOVER_SPEED_MMS constant; LegPhase.HANDOVER; `__init__` _ho_left; the window-head decision fork (§3.3); add HANDOVER to the follow set in step 4 (§4.1); the new step 4.7 (§3.4); the _step_swing pass branch; add a HANDOVER section to the module docstring |
| `hexapod/runlog.py` | add "handover": "Z" to PHASE_CH |
| `scripts/body_lean.py` | --handover parsing + startup print + doc header |
| `scripts/climb_walk.py` | same (the usual keep-both-sides-in-sync clause) |
| `tests/test_climb.py` | the 8 items of §7 |
| `docs/en/P4-GUIDE.md` | add a zero-force handover entry to the "stepping in place on the wall" row of the troubleshooting table (optional) |

## 9. Acceptance (definition of done)

1. All tests green (the existing 73 + the new ones).
2. Mock smoke test: body_lean --mock with --handover runs a full lift-and-land
   round.
3. **On-robot A/B (the step after next)**: re-run the 08-20 stepping-in-place
   experiment (same parameters + the --handover starting table) — the 74mm slip
   per round should drop by an order of magnitude (<10mm); whole-robot current
   should stop climbing cycle by cycle (staying ~0.7-1.0A); no visible bounce
   at vent in the video. Reuse the quantification pipeline (NCC tracking, see
   the "quantification pipeline" footnote in the report).
4. Once A/B passes, move on to climb_walk: single steps first, then continuous,
   fine-tune δ, then re-measure the net advance rate (26% baseline).

## 10. Review hardening (2026-08-23, v1.6)

A code review of the three v1/v1.5 commits confirmed 15 problems (7 in the
engine + 8 in the quantification pipeline). The common root cause: the HANDOVER
phase was written with VENT's treatment ("on its way out"), but it needs
STANCE's treatment (sealed, load-bearing, and being systematically loaded and
unloaded). Engine-side fixes (all of them entered into tests):

1. **The leak watchdog covers the handover leg** (_leak_watch takes
   STANCE+HANDOVER): previously the handover leg itself was unwatched for up to
   δ/rate seconds — if its lip failed midway, step 4.7 kept "giving force back"
   from a foot that was no longer carrying anything = the stored energy is
   released in an instant all the same. Now a leak on the handover leg also
   triggers leak_pause (the transfer pauses without losing progress and nothing
   vents), and a rescue timeout freezes and names it; the leak guard in
   request_lean follows the same convention.
2. **Per-tick envelope pre-check on the handover displacement, clipped when out
   of bounds**: step 4.7 was the only pusher on the stance system without a
   clamp (4.5 has the comp_tail allowance, 4.6 has _lean_room), so a too-large
   δ or accumulated drift pushes a stance leg outside the IK envelope →
   WorkspaceError freeze, and since clear_freeze keeps the handover in flight
   (semantically correct, kept), pressing `f` resumes it and re-freezes on the
   same frame = an infinite loop on the wall, `oo` is refused because the leg is
   not STANCE, and the only way out is ESC×2, which vents = a fall. Now, if any
   leg's new target for this tick leaves the safe envelope (_foot_xy_ok, the
   same convention as _lean_room), the remaining amount is clipped and it vents
   early — partial unloading = a reduced but safe bounce, with a handover_note
   trace (measured with δ=25 lifting the same leg repeatedly: it triggered on
   the 12th lift, with no freeze and IK solvable on every tick).
3. **Re-check the gate / interlock before venting**: δ/rate seconds pass
   between the window-head decision and request_release (longer with a leak
   pause or a freeze + `f`), and during that time a stance cup can leak into the
   blind spot above the −50 gate and below the −20 leak trip line — venting with
   a soft shoulder is exactly the 08-19 class of accident. Now it re-checks when
   the transfer finishes and, if the check fails, keeps the seal and waits for
   the pump (gate_wait shown externally, sharing the lift_gate_timeout_s timeout
   freeze).
4. **Rotation-convention guard for the weights** (§5, v1.5 paragraph) +
   **re-validation / normalization of the weights at the engine boundary**: the
   path that sets config directly did not normalize ((1,1,1,1,1) gives a net
   +4δ downhill per lift, breaking the zero sum), and the CLI parser did not
   reject nan/inf (a NaN share flowed all the way into joint_deg_to_us and got
   clamped to the 500µs endpoint pulse width) — both entrances now validate.
5. **δ counted in the enforced budget**: climb_walk hard-refuses at startup
   unless δmax ≤ comp_tail − half stride (at the same level as --max-step; the
   factory table's L1:17 exceeds the 16.8 margin at the default stride of 40, so
   the doc example was changed to --max-step 38); _lean_room reserves the
   handover in flight in advance (share × remaining amount).
6. **The per-leg table breaks mean invariance**: see the ⚠ note in the second
   engineering property of §2 (the touchdown reset swallows the residual,
   ~4.8mm of uphill artifact in round 1; the steady-state drift per round is
   still 0), and the A/B reading discipline went into the protocol.

The 8 quantification-pipeline (ab_quant.py) items were fixed separately:
negative offsets, climb_walk logs, the NCC search radius, --calib override, the
handover look-back window, round inference, the cache fingerprint, and zoom
geometry — see that file's header and the commit log.

## Appendix A: 1-D spring model (verification script, runs as is)

```python
def sim(mode, cycles=300, u=40.0, mg=89.0):
    """y_i=anchor-command; X=mean(y)-mg/n; attachment locks in zero pre-load, y_new=X.
    mg/k=89mm back-computed from the measured 74mm/round in place. u=stride(mm),
    commanded net advance=6u/5 per cycle."""
    y = [0.0] * 6
    xs = []
    for c in range(cycles):
        for j in range(6):
            att = [i for i in range(6) if i != j]
            if mode == "handover":          # zero-force handover before vent
                x6 = sum(y) / 6 - mg / 6
                fj = y[j] - x6
                y[j] -= fj
                for i in att:
                    y[i] += fj / 5
            for i in att:                   # the support field sweeps u/5 within the window
                y[i] += u / 5
            y[j] = sum(y[i] for i in att) / 5 - mg / 5   # re-attachment lock-in
        xs.append(sum(y) / 6 - mg / 6)
    return xs[-1] - xs[-2]                  # steady-state net advance per cycle

# baseline u=40: 12.4mm (25.8%, measured 26%); handover: 48.0 (100%)
# baseline u=0: -35.6mm/round (measured -74, model is half an order low -- the real robot also has posture nonlinearity); handover: 0
```

Known deviations of the model: on the real robot a uniform lean command loses
60% (the linear model predicts 0%) — there is a posture-dependent nonlinear
loss layer that the handover cannot fix; that needs calibration or a
mechanically stiffer structure. And a single leg's stored energy has a
torque-saturation ceiling (~21mm), which the model does not have. Neither of
the two affects the correctness of the handover mechanism itself.
