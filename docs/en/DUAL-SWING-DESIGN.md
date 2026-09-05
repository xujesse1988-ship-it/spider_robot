> English translation of [`docs/DUAL-SWING-DESIGN.md`](../DUAL-SWING-DESIGN.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Dual-leg climbing (dual-swing) · upgrade design

Status: **v1 implemented** (decided and implemented the same day, 2026-08-24:
all three points of §9 approved — A form = duty change + engine generalized to
multiple swings, B dual swing forbids sag/weights, C δ ceiling 45 unchanged.
Changes: `gait.CLIMB_DUAL` + nine surgical edits in `climb.py` (§4) + `--dual`
in two scripts + `ab_quant` landing-transfer compatibility; tests 96→111 all
green (zero regression on single swing + 15 dual-swing items), pty smoke test
passed on 5 branches (mutual-exclusion refusal ×2 / 28 lifts in continuous
walking all in ring order and always exactly 2 airborne at any instant / pair
step / body_lean pair lift). Implementation notes in §10. **To do: §8.3–8.5,
the on-machine ladder** — dual swing on a glass plate on the floor →
re-calibrate the three δ_pair groups (protocol §8.1) → A/B on the wall
(protocol §8.2).)
Measured basis: real-machine window-duration decomposition
`software/logs_analysis/climb_20260819_184038.log`
(08-19 on the wall, long-stride straight walking, 10 lift-land cycles);
zero-force handover, three-group δ calibration
`html/en/handover-delta-calib-20260824.html` (δ*≈28–42 per leg, per-leg
stiffness differs by 4×).

---

## 1. Goal and conclusion (one paragraph)

Single-swing climbing (5/6 duty, 5 feet attached at any instant) already runs
smoothly. The dual-swing upgrade = **exactly 2 legs swinging and 4 legs
attached at any instant** (4/6 duty). Gain: at the same stride, travel speed
**ideally ×2.5** (windows per cycle 6→3 × travel per cycle ×1.25); report ×2
conservatively. Cost: support redundancy drops one notch (a leak case falls
onto the design's "extreme conservative" line), **the δ table must be
re-calibrated as a whole for the dual-swing condition** (both the bounce
readings and the steady-state stored energy change), and nine surgical edits to
the engine's "one leg at a time" assumption. Core finding: **no new phase table
is needed** — the CLIMB offsets stay put, duty goes 5/6→4/6, and the
event-driven clock naturally gathers the lifts into "staggered dual swings":
~0.6 s stagger inside a pair, and every pair is opposite-side, different-row
(§2).

## 2. Form: change one duty, the pairing emerges by itself

### 2.1 Mechanism

- Swing window = (1−duty)·T: at duty 4/6 the window is 2 slots long (1.2 s
  nominal) while adjacent window heads are still 1 slot apart (0.6 s) — the
  windows overlap pairwise, so **exactly 2 legs have phase ≥ duty at any
  instant** (a direct corollary of six phases tiling one turn at equal
  spacing).
- The event-driven clock rules are unchanged (the window tail waits for a firm
  seal, the window head waits for the interlock): each leg's real swing
  duration D (≈4.3s, ≈7s with the handover) far exceeds the 1.2s nominal
  window, so the clock repeatedly stalls at a window tail and gathers the
  following window heads into a steady rhythm of "release one pair every D
  seconds, 0.6s apart inside the pair" (walk-through table in Appendix B).
- The lift ring order stays the CLIMB ring (the smaller duty shifts all window
  heads by one slot, so the first leg goes R3→L1, but the ring order is
  unchanged): **L1→R2→L3→R1→L2→R3**. The combinations airborne at the same
  instant roll as (L1,R2)(R2,L3)(L3,R1)(R1,L2)(L2,R3)(R3,L1) — the strong
  pairs (0.6s stagger, D−0.6s of shared flight) are **(L1,R2)(L3,R1)
  (L2,R3)**, with a weak 0.6s cross-pair overlap.
- The existing property of the current window order, "consecutive lifts are
  always diagonal/crossed and in different rows" (a product of the 08-19 lesson
  about lifting two same-side legs in a row), happens to guarantee: **the two
  legs airborne at the same instant are necessarily opposite-side and in
  different rows** ⇒ the front row always keeps ≥1 foot in place (somebody
  carries the peel moment), and the two middle legs (the softest leg chains)
  are never off the wall at the same time. A property designed back then for
  "adjacency in sequence" is reused by dual swing as "compatibility at the same
  instant".
- The semantics of `--leg-order` are automatically upgraded to "it sets the
  simultaneous combinations": under dual swing, a custom order with same-side
  neighbors = **same-side dual swing at the same instant**, whose cost is far
  higher than under single swing — the help text gets a stronger warning.

### 2.2 Alternative form (not recommended): 3 synchronized-pair windows

Make a new phase table (3 offset groups, same phase inside a pair), window =
pair. It needs the stagger inside the pair to be done by hand (vent/pumping/
coils all have to be scheduled) and the behavior at the window-splice
boundaries has to be re-verified, while "the window is a tidier unit of
progress" buys nothing real — the emergent behavior of duty 4/6 + the current
phase table already contains the stagger and the pairing. Dropped.

## 3. Five ledgers

### 3.1 Speed ledger (anchored to measurement)

Real-machine window decomposition (08-19 on the wall, press 20/stand 62/
no-tank, 10 windows, excellent consistency):

| vent | lift | transfer | descend | press | suck+confirm | total | inter-window gap |
|---|---|---|---|---|---|---|---|
| 0.30 | 0.88 | 0.62 | 0.50 | 1.35 | 0.61 | **4.26s** | 0.04s |

Gate/interlock waits were 0 throughout — the pump kept up the whole time, so
**window duration = pure kinematics**.

- Wall clock per cycle: single swing 6D, dual swing 3D (Appendix B) — **time
  ×1/2**;
- Travel per cycle: the clock advances T per cycle, real stance displacement =
  v·T·duty ≤ stride ceiling S ⇒ travel v·T ≤ S/duty, which at S=40 is 48→60mm
  — **travel ×1.25**;
- Together: at δ=0, 1.86→4.65mm/s; at δ≈27 (mean of the new table) the handover
  adds +2.7s per window, 1.14→2.86mm/s — **ideally ×2.5**. Losses: the vent
  stagger guard (§4-5, occasionally +0.3s), recovery in real-tank mode, gate
  waits on the wall — **report ×2 conservatively**.
- Seal-rupture events per meter: 6/48mm → 6/60mm, **−20% per meter** (the
  ratchet source gets sparser).

### 3.2 Support ledger (redundancy drops one notch, needs a nod)

Continuous walking is steady at **4 attached** (single swing: steady 5). Using
the convention of CLIMBING-DESIGN §2 (budget ceiling 2.5kg = 24.5N, shear
15N/cup @ −50kPa; at the operating point −60 to −75 there is actually another
~×1.2-1.5 of margin):

| Case | Single swing (current) | Dual swing |
|---|---|---|
| Normal, all effective | 5×15/24.5 = **3.1** | 4×15/24.5 = **2.4** |
| Conservative (1 fewer effective) | 2.4 | **1.8** (= the original design's "extreme conservative" line) |
| The instant one cup leaks | 4 attached → 2.4 | **3 attached → 1.8** |

- The whole ladder moves down one rung: half of the redundancy bought back when
  we chose "six legs vs four legs" is spent on speed. Acceptable during
  rope-protected experiments; **whether it becomes the normal convention is
  decided by the on-wall A/B data** (§8).
- The fraction of time a single front cup carries the peel moment goes
  33%→67% (there is always one front leg in the air).
- The static load share per stance leg goes mg/5→mg/4 (+25%): the servo holding
  current and the baseline of the "total current drops back after attachment"
  criterion both have to be re-read at +25%; the magnitude of the total current
  is unchanged (it is still the same body weight).
- Leak rescue self-heals well: leak_pause only stops new lifts and the stance
  field, **the two airborne legs still land and attach in real time** — within
  2–3s it is automatically back to 5–6 attached before anything is done about
  it.

### 3.3 Bounce and δ ledger (whole table re-calibrated, method unchanged)

- The catching side goes 5→4 in parallel: at the same residual stored energy,
  the bounce reading and the force step on each stance cup both go ×5/4.
- **Two legs rupturing at the same instant = worst case ×2.5** (double the
  stored energy into 4 legs) — **staggering the vent inside a pair is a hard
  requirement**, and the engine guard keeps consecutive vents ≥0.4s apart
  (§4-5; as a bonus this preserves the video distinguishability of the per-leg
  vent jumps for ab_quant). Note that the stagger cannot rely on the 0.6s
  window-head difference alone: δ differs per leg, and a later-window leg with
  a small δ finishes laying first (e.g. L1:31 vs R2:22, R2 overtakes by 0.3s),
  so an explicit guard is required.
- **δ* (= that leg's stored energy / its own stiffness) does not depend on the
  number of catchers, but the steady-state stored energy does change**: the
  static share goes mg/5→mg/4 and the event rhythm goes 6→3 windows per cycle,
  so δ*_pair is expected to shift up by +10–30% overall, and L1/R1 may approach
  the analytic ceiling of 45 (§9C). The slope (used in protocol §6 to convert
  the correction) also scales ~×5/4 with the number of catchers, so re-take it
  too.
- Calibration follows the 08-24 three-group linear extrapolation, but **the
  three groups are the current table ×0.5/0.75/1.0** (≈16/23/31 steps) rather
  than starting from δ=0 — bare bounce hurts more under dual swing, and linear
  extrapolation never needed a zero point anyway. Steady-state discipline
  unchanged: discard the first round (the cold-start first window has only a
  single swing and shares into 5 stance legs, a different condition from the
  steady-state 4), count round 2 onward.

### 3.4 Pneumatic/electrical ledger (mostly already-closed items)

- The per-foot check valves (installed and accepted 08-17) already physically
  isolate "the second open valve stealing the vacuum" — the prerequisite for
  parallel pumping under dual swing was laid long ago.
- SUCK measures 0.28–0.32s < the 0.6s stagger ⇒ **pumping is naturally near
  serial**, and the instantaneous tank dip is ≈ the single-swing one.
  Exception: a front-leg retry that deepens the draw together with a rear leg
  landing normally can pump in parallel (rare); just watch the tank pressure in
  TLM. Backup = pump B (`PUMP_B_PIN=26` is defined but not driven; the BOM
  pneumatic diagram already draws 555×2 in parallel).
- Coils: during the swing the exhaust position = coil energized, so the
  overlapping segment of a dual swing holds two coils at ~8W continuously
  (single swing ~4W), and the switching edges are naturally ≥0.4s apart — the
  "six coils stepping ~25W at the same instant" pattern that the 08-18 lesson
  was about does not recur.
- Gate (lift_gate −50): the pair that has just landed gets a whole D before the
  next window head for the pump to pull it deep; this step measured zero
  waiting on 08-19; re-check in real-tank mode on the wall.

### 3.5 Workspace ledger

- **The hard startup check δmax ≤ comp_tail − half stride is unchanged as an
  upper bound** (proof in Appendix A: a stance leg eats (sum of the pair's δ)/4
  in each of two windows, and its accumulated total before its own lift is ≤
  δmax, the same bound as single swing). The check formula in climb_walk is
  unchanged to the letter; the comment is updated.
- The VENT stance-field tail goes ×5/4 (at stride 40, 4→5mm):
  `max_straight_step` is parameterized by gait.duty and tightens by ~1mm
  automatically, no change needed.
- `--sag-comp`: the /5 apportioning arithmetic is semantically invalid under
  dual swing (the sag of 2 legs per window has to be compensated together), and
  the handover is the root fix anyway while the A/B discipline is
  one-or-the-other in the first place — **v1 dual swing simply refuses with
  ap.error**.
- `--handover-weights`: ~~refused under v1 dual swing~~ → **supported under
  dual swing from v1.1** (the revision to decision B made the same day, 08-24,
  after single-swing measurements showed the weights clearly improve slip).
  Mechanism: the share = w[5−d] by forward rotation distance d, then
  **normalized in place over the current STANCE set** (`_share_now` recomputes
  it every tick) — during single-swing laying, stance is always 5 and the
  weights sum to 1, bit-for-bit identical to the v1.5 frozen table; under dual
  swing the stance set changes dynamically with the order inside the pair and
  the landing of the previous pair, and the normalization adapts
  automatically. **The presets split by convention** (model in Appendix C,
  objective function = worst single-leg unloading force): dual swing recommends
  **1,1,1,0,0** (= the pair that just landed plus the rear leg of the next
  pair, split three ways: worst single leg 30.8→27.0 (−12%), mean 30.4→24.0
  (−21%)). Two traps: ① copying the single-swing aggressive preset
  0.6/0.25/0.1/0.05/0 over to dual swing pushes the rear leg of the pair above
  uniform sharing (32.9>30.8 — the "mean −21%" is carried entirely by the
  leading leg's 14.9, a fake ledger); ② the "best mean" 0.5,0.5,0,0,0 funnels
  the whole cycle into the last three legs (single leg 57, leading leg 0) —
  **when judging weights for dual swing, only the worst single leg counts**.
  The automatic preset with no value given splits by --dual (in the script
  layer). The rotation guard is unchanged (deviate from the rotation and it
  falls back to uniform sharing; the "stance ≠ 5" check only applies to single
  swing — a swing in flight is by design normal under dual swing). ⚠ Turning
  the weights on changes the steady-state operating point: **run the
  three-group δ_pair calibration with the weights on** (the model expects δ* to
  shift down accordingly, putting the worst leg further from the analytic
  ceiling of 45 — the weights are exactly the first relief when δ*_pair hits
  the cap, ahead of raising the ceiling as in decision C).

## 4. Engine surgery list (the nine places that assume "one leg at a time")

1. **Window membership**: `_slot()` single leg → the set of legs in a window
   (always 2 elements); the window-head decision now fires on **each leg's own
   window-head edge**; `_slot_active/_slot_skipped/_block_t/_gate_t` are stored
   per leg (cleared on window switch → cleared on the head edge).
2. **Clock advance**: adv = min over the legs in a window (not started and not
   skipped = stalls at its window head, 0; started and not back to STANCE =
   advance at most to its window tail; skipped = no constraint).
3. **Interlock/gate**: "the other 5 all attached" → "**every leg not inside a
   swing window** is attached, not leaking, and deeper than the gate" (legs in
   a window are exempt, which naturally covers the leading leg of a pair being
   airborne); the re-check before venting after the laying finishes uses the
   same convention.
4. **Zero-force handover**: `_ho_left` is stored per leg (two handovers may
   overlap); apportioning changes to **recomputing every tick over the current
   STANCE set** (dropping the freeze at start) — the leading leg's frozen table
   would hand δ/5 to the second leg of the pair that has already entered
   HANDOVER, polluting the ledger. Under the single-swing condition the STANCE
   set is constant during a handover ⇒ live computation is bit-for-bit
   identical to freezing, so the regression risk is zero (a test asserts it).
   Out-of-range truncation (_foot_xy_ok every tick) and the leak pause are
   unchanged and take effect per leg.
5. **Vent stagger guard** (new): after the later-window leg finishes laying its
   handover, it must **wait until the earlier-window leg has vented and at
   least 0.4s has passed since the last vent** before request_release; it stays
   sealed while waiting (the completed HANDOVER state), and the wait is exposed
   under the gate_wait convention.
6. **Single step 'i'**: one press = the two legs of the pair (the rotation leg
   + the window-order successor) are accepted one after the other and lifted to
   HOVER (two hovering); the next press = the two land and attach one after the
   other and stop automatically; the rotation advances by 2 in one go; the
   prompt text changes to two legs. `step_hover_leg` returns a set.
7. **Fast-forward to the rotation leg when starting**: align to the rotation
   leg's window head (the formula is generic, unchanged); the successor's
   window head arrives naturally 0.6s later.
8. **Rotation guard** (`_prev_lift`): the successor semantics are unchanged
   (the next leg in window order). From v1.1 dual swing has weights: the guard
   checks as usual (deviate from the rotation → fall back to uniform sharing);
   the rear leg of a pair lifting right after the leading leg is on the
   rotation, so the weights apply as usual. The "stance ≠ 5" check only applies
   to single swing (a swing in flight is by design normal under dual swing).
9. **sag_comp**: refused under dual swing (§3.5); the two places in
   `max_straight_step` that use `/5` and `5.0*comp` are inert at sag=0 and do
   not block dual swing; the comment marks the convention limit.

**What does not change**: the startup sequence (feet are pressed in and pumped
one at a time in series — the reaction-force seating problem has nothing to do
with the gait), adhesion.py's state machine, the exit/retrieve/--release
sequences, the leak watch (per leg; HANDOVER is already handled), the freeze
semantics (all targets are held, equally true with two legs airborne),
`comp_tail`/`_worst_stance_travel`/`_clamp_speed`/`_landing_xy` (parameterized
by duty, they follow automatically).

**Corner cases** (covered by tests): when a front leg's retry drags on, the
clock stalls at its window tail while the rear leg finishes landing in real
time (briefly back to 5 attached) — if the retries are exhausted and it
freezes, 2 legs may be airborne; targets are held and the manual handling is
the same as today; in air mode the count of 2 give-ups per window still runs.

## 5. Gait definition and script interface

- `gait.py`: `DUAL = replace(CLIMB, duty=4/6, name="climb-dual")` (same offset
  set); it composes naturally with `gait_with_slot_order` (the head-order
  formula takes the new duty).
- `climb_walk.py` / `body_lean.py`: a `--dual` switch (default off = current
  single-swing behavior); mutually exclusive with `--sag-comp` (ap.error);
  `--handover-weights` may be on at the same time from v1.1 (with no value, the
  automatic preset splits by convention: single-swing aggressive preset /
  dual-swing 1,1,1,0,0); on startup it prints "dual swing windows: ring order
  L1→R2→L3→R1→L2→R3, steady state always 4 attached, the δ table must be
  calibrated for the dual-swing condition, do not compare A/B across
  conventions"; the black-box parameter line records `dual=1`; the hard δ check
  formula is unchanged.
- `body_lean.py --dual`: **pair lift** in place (request_lift generalized to a
  pair, the number keys pick the leading leg of the pair) — the vehicle for the
  three-group dual-swing δ calibration.
- `runlog.py`: no change needed (events are per leg anyway; PHASE_CH is already
  complete).

## 6. Calibration and A/B (section added to HANDOVER-AB-PROTOCOL)

1. **The time-sync marker discipline, restated**: start the recording first →
   start body_lean → **the §4.3 body-lean marker is mandatory** (the lesson of
   all three 08-24 groups being wrong: activity contrast has no immunity
   against aligning to the cycle rhythm).
2. Three dual-swing δ groups: pair lift in place at the current table
   ×0.5/0.75/1.0, ≥2 rounds per group, discard the first round; ab_quant
   per-leg vent jumps → linear extrapolation to δ*_pair → start calibrated at
   ×0.8.
3. A/B vs single swing (continuous walking at the same parameters): net advance
   rate, rupture events per meter, displacement per event, current ratchet
   slope, worst cup-pressure distribution, gate-wait count.
4. ab_quant check items: parse_log's per-leg state machine must tolerate
   interleaving (run a mock dual-swing log through it); the handover look-back
   window is already per leg; the round index = per-leg ordinal already
   tolerates dropouts; the summary line is labeled with the dual convention.

## 7. Test list (added to tests/test_climb.py, in the existing style)

1. duty 4/6 windows tile fully: at any t exactly 2 legs are in a window; the
   ring order = the CLIMB ring (first leg L1).
2. Mock continuous walking for 2 cycles: start rhythm 0/0.6/D/D+0.6/…, all 12
   lift-land cycles in ring order, pulses resolvable with zero freezes
   throughout.
3. Interlock exemption: at the successor's window head, the leading leg being
   airborne still lets it through; a leg **outside a window** that is not
   attached → lift refused + the timeout freeze names it (the exemption does
   not spread).
4. Vent stagger guard: when the later leg's δ is smaller than the earlier
   one's (the R2:22 vs L1:31 kind), the later leg waits for the earlier leg's
   vent +0.4s; vent order = window-head order.
5. Overlapping handovers sum to zero: with two handovers in flight at once, the
   vector sum of the six legs' displacements is 0 every tick; a stance leg
   receiving two shares has them computed live over the current STANCE set.
6. Single-swing regression: live apportioning is bit-for-bit identical to the
   old frozen table (the existing 96 tests all green + a dedicated assertion).
7. Clock-stall semantics: the clock stalls at the leading leg's window tail
   until it is firmly attached; the later leg finishes landing in real time.
8. Leading leg in FAULT retry, later leg closes out normally; if the retries
   are exhausted and it freezes with two legs airborne, the targets are held.
9. The gate and the pre-vent re-check use the "legs outside a window" set.
10. Single step 'i' pair step: both hover, both land, automatic stop, rotation
    +2.
11. Rotation pair on starting: the first pair from standstill = the rotation
    leg + the successor.
12. δ budget: at δmax=45 the startup check under dual swing has the same bound
    as single swing (the algebraic assertion of Appendix A).
13. `max_straight_step(duty=4/6)`: finite, tightens by ~1mm with duty.
14. `--dual` and sag are mutually exclusive and refused (from v1.1 the weights
    are let through, normalization unchanged); the `--leg-order`+`--dual`
    combination takes effect.
15. Leak rescue: a stance leak during a dual swing → new lifts stop, both
    swings land, rescue/freeze conventions unchanged.
16. Dual-swing weight shares (v1.1): 1,1,1,0,0 lifting L1+R2 — the partner and
    the leading leg of the next pair get 0 throughout, d3/d4/d5 get δ/3 each,
    and after the rear leg unloads and it is normalized, L2/R3 get δ/2 each;
    under overlapping laying the sum is zero every tick and the guard does not
    fire.
17. Dual-swing weight rotation guard (v1.1): lifting the same pair repeatedly,
    from the second time on the leading leg falls back to uniform sharing and
    leaves a trace (a leg whose weight preset is always 0 really does get a
    share under uniform sharing = distinguishable).

## 8. Acceptance (definition of implementation complete)

1. ✅ All tests green (96 existing, zero changes and zero regressions + 15
   dual-swing items = 111, 2026-08-24).
2. ✅ Mock pty smoke test, five branches (2026-08-24): --dual+--sag-comp
   refused with exit 2 / weights without --handover refused with exit 2 (before
   v1.1 this slot was the dual+weights mutual-exclusion refusal) /
   continuous walking, 28 lifts all in ring order + always exactly 2
   airborne + clean exit / pair step i×2 (L1+R2 hover → land together → next
   pair L3+R1) / body_lean pair lift and land end to end (with the δ table).
3. Dual swing on a glass plate on the floor (start from the single-swing δ
   table, watch tank pressure/current/bounce).
4. Re-calibrate the three dual-swing δ groups (§6 / protocol §8.1), δ*_pair
   entered in the table.
5. On-wall A/B (protocol §8.2): net advance ≥1.8× single swing, displacement
   per event and current ratchet no worse than single swing at the same δ level
   — only after passing this do we discuss making it the normal convention
   (§3.2).

## 9. Decisions (2026-08-24, all approved)

- **A. Form**: ✅ "duty 4/6 + the current phase table + the engine generalized
  to multiple swings" (§2.1, the stagger emerges by itself); the alternative of
  3 synchronized-pair windows (§2.2) is dropped.
- **B. v1 scope**: ✅ dual swing forbids `--sag-comp`/`--handover-weights`
  (§3.5; CLI ap.error + a ValueError at the engine entry, double insurance);
  single step 'i' = pair step. **v1.1 revision on 08-24 (driven by the user's
  measurements)**: the weights are partly unbanned — on-wall single-swing
  measurements showed `--handover-weights` clearly improves slip, so dual swing
  is supported via the v1.1 mechanism of §3.5 (value by distance + normalize in
  place) plus the dual-swing-specific preset 1,1,1,0,0; the sag ban is
  unchanged.
- **C. Analytic ceiling for δ**: ✅ 45 unchanged; if δ*_pair breaks through it,
  discuss raising it per the room ledger (at stand 60 / trim 8, comp_tail −
  half stride ≈40) — calibrate first, then change the ledger, do not spend it
  in advance.

## 10. Implementation notes (2026-08-24, off-design issues found and handled during implementation)

1. **Start-alignment deadlock (caught on the machine; ancestor of test §7.8)**:
   besides "all legs STANCE", the original alignment condition also checked
   "no swing in flight" (the old `_slot_active`) — a leg that has just closed
   out its swing is back in STANCE but its window instance is still marked
   active until the window tail, which wastes the rising edge of want_go; when
   a pair step is accepted again on the same tick as the pair closes out, the
   rotation leg's current window has already been marked skipped and it can
   only wait for the next cycle, while its partner lifts to hover first and
   stalls the clock dead = deadlock. Fix: delete the redundant condition — "all
   legs STANCE" already implies "no swing in flight". The same narrow-window
   (1 tick) hazard under single swing is eliminated along with it.
2. **Apportioning table frozen → recomputed every tick** (§4-4 implemented):
   the leading leg's frozen table would hand a share to the second leg of the
   pair that has already entered HANDOVER (ledger pollution); under the
   single-swing condition the STANCE set is constant during laying, so live
   computation is bit-for-bit identical to the old freeze (the 96 existing
   tests all green is the proof). The weights table (single swing only) is
   still frozen at the instant of release, and the rotation-guard semantics are
   unchanged.
3. **δ=0 now goes through the HANDOVER dwell state too**: the queued wait for
   vent staggering needs a "released but not yet vented" dwell phase; at δ=0
   step 4.7 falls straight through to VENT on the same tick, which is
   unobservable between ticks and identical to the old "vent directly"
   behavior (including the pre-vent gate re-check, same tick, same data).
4. **ab_quant landing-transfer compatibility**: the second leg of a pair step
   goes hover→descend through the landing stagger queue (not transfer→descend),
   and the `\w+` in "landing: L1+R2" only captures the first leg — without
   patching the regex that leg would be dropped from the calibration table
   entirely. Added `相位 (\w+) hover→descend` ("phase (\w+) hover→descend"; in
   single-swing logs the two events coexist on the same tick, so whichever
   matches first means the same thing); verified end to end on a dual-swing
   mock log: 22 lift-land cycles in continuous walking, per-leg assembly all
   correct, 21/21 places where adjacent lift-land times overlap with zero
   cross-talk, both legs of every pair entered in the pair-step/pair-lift logs,
   and every leg in the δ table carrying handover events.
5. **Zero change to the stride budget formula**: both the hard startup check
   δmax ≤ comp_tail − half stride and `max_straight_step` are parameterized by
   gait.duty and follow automatically (the vent tail ×5/4 tightens things by
   ~1mm on its own), and the Appendix A upper-bound algebra gives the same
   bound under dual swing (the test asserts it exactly against the 2s/s/0 drift
   profile of the share geometry).
6. **v1.1 unbanning of the weights (08-24, same day, driven by the user's
   single-swing measurements)**: the `_freeze_share` frozen-table mechanism is
   replaced by `_weights_ok` (settled at the instant of release, the guard
   unchanged) + `_share_now` (value by distance every tick + normalize in place
   — under single swing stance is always 5, so it is bit-for-bit identical to
   the frozen table; under dual swing the dynamic set adapts automatically);
   `_lean_room`'s pre-deduction now takes the maximum of the live shares (the
   single-swing convention is unchanged). The Appendix C model fixes the
   preset: the objective function must use the **worst single leg** — the
   best-mean preset (0.5,0.5,0,0,0) funnels the cycle into the last three legs
   (single leg 57), and copying the single-swing aggressive preset pushes the
   rear leg of the pair past uniform sharing (32.9>30.8), while the dual-swing
   preset 1,1,1,0,0 gives −12% on the worst and −21% on the mean. Tests 113
   (+2: exact share assertion / distinguishable rotation-guard fallback); smoke
   tests +3 (single-swing automatic preset regression / dual-swing automatic
   preset / full pair lift-and-land flow with weights).

## Appendix A: the δmax upper bound is unchanged under dual swing (proof)

Between its own two lifts, stance leg i goes through two other windows and eats
a downhill share of (δ_a+δ_b)/4 in each; total eaten = (Σ_{the other four
legs} δ)/4 ≤ 4·δmax/4 = **δmax**. When its own turn to lift comes it first
unloads +δ_i uphill, and on landing it resets to the standing position — so the
maximum extra downhill outswing is ≤ δmax, the same bound as single swing (5
windows × δ/5 ≤ δmax). The property that under a per-leg table the mean
residual is swallowed by the landing reset (HANDOVER-DESIGN §2 ⚠) holds
identically for dual swing: a first-round artifact, and zero drift per round in
steady state.

## Appendix B: steady-state timeline walk-through (D = real duration of one window)

Window heads (clock): L1@0, R2@0.6, L3@1.2, R1@1.8, L2@2.4, R3@3.0; window
length 1.2. Clock rule: it cannot pass the window tail of a leg that is not
firmly attached. After the cold start:

| Real time | Event | Airborne at that instant |
|---|---|---|
| 0 | L1 window head releases (handover→vent→swing) | L1 |
| 0.6 | R2 window head (L1 exempt, inside a window) | L1, R2 |
| 0.6→D | clock stalls at L1's window tail (1.2) waiting for a firm seal | L1, R2 |
| D | L1 firmly attached, clock passes L3's window head | R2, L3 |
| D+0.6 | R2 firmly attached, clock passes R1's window head | L3, R1 |
| 2D | L3 firmly attached, L2 window head | R1, L2 |
| 2D+0.6 | R1 firmly attached, R3 window head | L2, R3 |
| 3D | L2 firmly attached, L1's next-round window head | R3, L1 |

Cycle = **3D** (single swing 6D); steady state is always 2 swinging / 4
attached; the strong pairs (L1,R2)(L3,R1)(L2,R3) are staggered by 0.6s, with a
weak 0.6s cross-pair overlap; the pumping intervals alternate 0.6s / D−0.6s.

## Appendix C: 1-D model of the dual-swing weights (verification script, runs as is; the basis for the v1.1 preset)

The Appendix A spring ledger extended to the dual-swing steady-state timeline
(Appendix B): at the window head of pair (A,B), prevB, the second leg of the
previous pair, is still airborne; A unloads first (for the first 0.6s its
partner B is still supporting and prevB is in the air), and 0.6s later B leaves
and starts unloading while prevB lands; A and B land one after the other (A
into a 4-lock, B into a 5-lock). The share policy is isomorphic to the engine's
`_share_now`: take w[5−d] by forward rotation distance d and normalize in
place.

```python
MG, PH1 = 89.0, 6.0        # mg/k mm (Appendix A convention); amount that can be laid within the window-head difference inside a pair

def share(policy, sup_d):  # sup_d: {leg: forward distance 1..5}
    if policy == "uniform":
        return {n: 1.0 / len(sup_d) for n in sup_d}
    raw = {n: policy[5 - d] for n, d in sup_d.items()}
    t = sum(raw.values())
    return {n: v / t for n, v in raw.items()} if t > 1e-12 else \
        {n: 1.0 / len(sup_d) for n in sup_d}

def land(y, j, att):
    y[j] = sum(y[i] for i in att) / len(att) - MG / len(att)

def sim_dual(policy, cycles=400, u=0.0, tail=6):
    y, peak = [0.0] * 6, {i: -1e9 for i in range(6)}
    for c in range(cycles):
        for A, B in ((0, 1), (2, 3), (4, 5)):
            O = [(A + k) % 6 for k in (2, 3, 4, 5)]
            X = (sum(y[i] for i in [A, B] + O[:3]) - MG) / 5
            fA = y[A] - X
            a1 = min(PH1, max(fA, 0.0))
            for n, s in share(policy, {B: 1, O[0]: 2, O[1]: 3,
                                       O[2]: 4}).items():
                y[n] += a1 * s              # head segment: partner still there, prevB airborne
            land(y, O[3], [A, B] + O[:3])   # prevB lands
            for n, s in share(policy, {O[0]: 2, O[1]: 3, O[2]: 4,
                                       O[3]: 5}).items():
                y[n] += (fA - a1) * s       # tail segment: partner has left
            y[A] -= fA
            Xb = (sum(y[i] for i in [B] + O) - MG) / 5
            fB = y[B] - Xb
            for n, s in share(policy, {O[0]: 1, O[1]: 2, O[2]: 3,
                                       O[3]: 4}).items():
                y[n] += fB * s
            y[B] -= fB
            for i in O:
                y[i] += u / 2               # stance field (u/2 per window)
            land(y, A, O)                   # A into a 4-lock
            land(y, B, [A] + O)             # B into a 5-lock
            if c >= cycles - tail:
                peak[A], peak[B] = max(peak[A], fA), max(peak[B], fB)
    return peak
```

Results (steady-state unloading force in mm, the same for u=0 and u=40; the
single-swing control reproduces Appendix A):

| Preset | Worst single leg | Mean | Per leg (leading/rear leg of a pair) | Verdict |
|---|---|---|---|---|
| Single-swing control: uniform / aggressive | 29.7 / 20.2 | same as left | six legs uniform | reproduces Appendix A ✓ |
| Dual swing · uniform (v1) | **30.8** | 30.4 | 29.9 / 30.8 | baseline (≈ single-swing uniform) |
| Dual swing · single-swing aggressive preset copied over | **32.9** ✗ | 23.9 | 14.9 / 32.9 | fake mean: the rear leg gets pushed higher instead |
| Dual swing · 0.5,0.5,0,0,0 | **57.0** ✗✗ | 22.2 | ≈0 / 57 | best-mean trap: the cycle funnels into the rear leg |
| Dual swing · **1,1,1,0,0** (the preset) | **27.0** | 24.0 | 21.0 / 27.0 | worst −12%, mean −21% |

A grid sweep (w monotonically decreasing × normalization invariance) shows the
whole (a,a,a,0,0) family tied for best under the worst-single-leg objective at
27.0 — i.e. "the pair that just landed plus the rear leg of the next pair,
split three ways; the leg about to lift and its partner get nothing". The model
does not include the leg-chain stiffness differences or the nonlinear loss
layer (the same limitation as Appendix A), so the relative conclusions are
usable while the absolute values wait on the three-group δ_pair measurement.
