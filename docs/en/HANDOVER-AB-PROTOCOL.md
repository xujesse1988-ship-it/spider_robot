> English translation of [`docs/HANDOVER-AB-PROTOCOL.md`](../HANDOVER-AB-PROTOCOL.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# On-robot A/B experiment plan for the zero-force handover (HANDOVER-DESIGN §9.3)

Status: **plan frozen, waiting to be run on the wall** (2026-08-20).
2026-08-24 added §8: three-group δ calibration for dual swing (`--dual`) and the
A/B convention (docs/en/DUAL-SWING-DESIGN.md §6, code already implemented).
Parent design: `docs/en/HANDOVER-DESIGN.md` (mechanism implemented, 81 tests green + mock smoke test passing).
Baseline: the 08-20 stepping-in-place experiment (`html/en/vent-snap-20260820.html`,
`software/logs_analysis/lean_20260820_102117.log`, `images/lean_20260820.mp4`).
Quantification tool: `software/logs_analysis/ab_quant.py` (the parameterized
version of the 08-20 pipeline checked into the repo, already regression-verified
against the baseline raw data: offset / scale / per-leg bounce / slip per round
all match the report).

---

## 1. Purpose and overall criteria

Re-run the 08-20 stepping-in-place experiment (same condition, same pacing),
comparing the `--handover` starting table on vs. off:

| Metric | Baseline (measured 08-20) | Pass line |
|---|---|---|
| Slip per round (round 2) | 74mm (76.4 on this pipeline's basis) | **<10mm/round** (an order of magnitude lower) |
| Per-leg vent jump (round 2) | L1 20.9 … L2 5.8mm | **per leg \|x\| ≤ 2mm**, no visible bounce |
| Whole-robot current | 0.79A at rest → climbs to ~1.8A cycle by cycle without falling back | **final rest ≤ before the first lift +0.2A**, no cycle-by-cycle climb |
| Body displacement during the handover segment (new mechanism check) | — (the baseline has no such segment) | **\|x\| ≤ 3mm, smooth with no steps** (on-robot verification of mean invariance) |

Item 4 is an independent probe of whether the mechanism is right: during the
handover the mean of the six leg commands does not change ⇒ the body should not
move at all. If the body sinks systematically during the handover segment ⇒
there is a bug in the direction or the apportionment, or δ is far too large:
**stop on the spot**.
⚠ Known artifact of the per-leg δ table (the ⚠ note in design §2, found by
review and measurement): the touchdown reset swallows the handover residual, so
the mean of the six commands takes a step at every touchdown — group B
accumulates ~5mm of **uphill** command artifact in round 1 (i.e. the round-1
slip reading is ~5mm smaller than physical reality), plus ±1.3mm of swing within
a round. This is the geometric accounting of a non-uniform table, **not** an
apportionment bug: the direction is uphill, the amount is bounded, and from
round 2 on the drift per round is 0. "Stop on the spot" only recognizes
**systematic sinking**; small steps on the uphill side are the artifact and are
let through.

## 2. Condition and command line (copied from the baseline, only --handover differs)

```
# Group A (same-day control, baseline replica)
python scripts/body_lean.py --no-tank --stand-height 62 --tilt-trim 6

# Group B (zero-force handover, δ starting table = measured bounce ×0.8)
python scripts/body_lean.py --no-tank --stand-height 62 --tilt-trim 6 \
    --handover L1:17,R1:15,L3:11,R3:9,R2:5,L2:5
```

The same glass wall, the same position as far as possible; A and B each get a
complete "mount on the wall — experiment — take down" cycle, and **the camera is
not touched** in between.

Why run A at all when 08-20 already gives a baseline: ① a same-day control on
site removes differences in glass cleanliness, battery and temperature; ② group
B moves <10mm in total, so **the mm/px scale can only be calibrated from group
A's large displacement** and group B reuses it (which makes locking the camera
from A to B a hard requirement); ③ if group A's round-2 per-leg bounces deviate
>20% from 08-20, re-derive the δ table as that day's values ×0.8 before running
B.

## 3. Site and equipment checklist

- [ ] Glass wall wiped clean with alcohol (the robot's working area + every cup
      landing spot);
- [ ] **Safety rope at all times** (the safety section of P4-GUIDE), padding
      below; leave **≥20cm** of clearance below the starting position (group A
      slips ~13cm over two rounds);
- [ ] A tape measure taped vertically on the glass, right next to the body (same
      as 08-20): the group-A scale calibration = the difference in readings of
      the same body edge against the tape in the video (the baseline was
      148→17 = 131mm);
- [ ] Phone/camera **locked on a tripod**, portrait, 30fps (the baseline
      544×960 is enough); the frame must contain all of: the body's green PCB,
      the tape measure, and one object guaranteed not to move (the window-sill
      flower pot as before, the camera-drift reference); lock the focus if you
      can; **from the end of the A recording to the end of the B recording, the
      tripod and the zoom must not be touched at all**;
- [ ] Battery starting at ≥8.0V (the baseline was 8.03V; the current comparison
      needs voltages in the same range);
- [ ] The code on the Pi synced to a version containing the zero-force
      handover, pre-flight: `pytest tests/ -q` all green +
      `body_lean --mock --handover 8` running one lift-and-land round (key
      sequence p→1→i→i→ESC×2).

## 4. Procedure for one group (run once for A and once for B, exactly the same actions)

1. Start recording → start the script → stand up slowly → check at the
   in-position pause → `p` to start full attachment.
2. ~~Rest ≥60s~~ **now built into `t` (2026-08-26)**: pressing `t` first counts
   down a 60s rest automatically (`--mark-settle` adjusts it, do not lower it
   for experiments; the remaining time is announced at 15s granularity) — pump
   hysteresis settling, a quiet drift reference for NCC, and keeping the marker
   far enough from the start of the video, all three purposes in one go. **With
   the recording running you just press t, you cannot forget it** (the C2 lesson
   made structural: the rest went from an operating discipline to a step in the
   key sequence). Space cancels = no marker was made and the whole group can be
   redone; the lean event timestamp = the moment the rest ends, so ab_quant's
   derivation of the synchronization window is unaffected.
3. **One-key experiment sequence (`t`, with automatic rotation since
   2026-08-25)**: the opening rest (above) → the synchronization marker (+10mm
   lean → hold 2s once applied → back to position → rest 20s) → **automatic
   rotation for `--auto-rounds` rounds (default 3) × 6 legs** — lifting each leg
   in the engine's rotation order, hovering 1s and landing (the order is
   **R3→L1→R2→L3→R1→L2**; between legs it waits ≥2s of pump idle and ≥10s since
   closure = the automated version of "wait for the cup pressure to come back
   to -75 and the pump to stop", capped at 30s after which it continues and
   leaves a trace; 15s of rest between rounds) → a closing 30s rest, then it
   prints "✓ experiment sequence complete". The pacing is reproducible from
   group to group and the rotation order is guaranteed by the engine (the cyclic
   rotation the guard demands when `--handover-weights` is on no longer depends
   on the operator's hands). Space cancels at any time (if you cancel after the
   marker has returned to position, the two ramps are already in the log and
   still valid, and the remaining rotation can be continued by hand); **a freeze
   aborts it automatically** (judge that group per §6); `--auto-rounds 0` =
   marker only (for cases where the same leg is lifted repeatedly to calibrate
   δ). What the marker is for: in group B the body barely moves during
   lift-and-land, so the 08-20 "motion contrast" synchronization method loses
   lock and `ab_quant.py` automatically switches to this large displacement to
   lock the offset; it is also an on-site self-check of the NCC tracking (the
   playback should show one clean out-and-back). Judgment is still **based on
   round 2** (round 1 is still ramping up), with round 3 as the cross-check (the
   08-24 three-group calibration was exactly three rounds in practice).
4. **Manual fallback** (the one-key `t` already covers this step and the next;
   use it to continue after an aborted sequence): lift and land each leg in the
   prompted rotation (after each "lift-and-land complete, next is X", press the
   matching number and then `i`; once the hover prompt appears, press `i` again
   to land). Pacing matched to the baseline: about 6s per leg lift-and-land,
   10–15s between legs (wait for the status line to show cup pressure back at
   -75 and the pump stopped before lifting the next one); ~15s of rest between
   rounds before the next one.
   Note for group B: after pressing `i` the status line first shows phase **Z**
   for that leg (the handover, 0.5–1.7s) and then turns to V — the Z segment is
   normal in the new flow, not a hang.
5. Closing rest ≥30s (the observation window for the current falling back; the
   automatic sequence includes it) → hold the body steady → `ESC`×2 to run the
   exit sequence → take the robot off the wall. **Changing the δ table means
   redoing the whole group** (δ is a startup parameter and cannot be changed on
   the wall; body_lean vents on exit, so a person must hold the robot, with the
   safety rope as backup).
6. Archive: copy the black box `software/logs/lean_*.log` into
   `logs_analysis/`, and the video into `images/` (named
   `lean_YYYYMMDD_A.mp4` / `_B.mp4`).

## 5. Quantification (run on the dev machine, ~25s per group)

```
# First extract frames and measure the templates (once per group; measure the rectangle coordinates on each group's own frame 0)
python software/logs_analysis/ab_quant.py --video images/lean_YYYYMMDD_A.mp4 \
    --log software/logs_analysis/lean_YYYYMMDD_*.log --out ~/ab/A --frames-only
# Group A: --calib T0,T1,MM = two quiet moments before the first lift / after the last landing (log seconds) + the tape-measure reading difference
python software/logs_analysis/ab_quant.py --video ... --log ... --out ~/ab/A \
    --body Y0:Y1,X0:X1 --ref Y0:Y1,X0:X1 --calib 142.3,327.5,131
# Group B: reuse the mm/px printed by group A
python software/logs_analysis/ab_quant.py --video ... --log ... --out ~/ab/B \
    --body ... --ref ... --mmppx 0.881
# Native-frame-rate rupture profile of a suspicious event (the time axis uses the frame rate measured by ffprobe, so 60fps
# phone recordings are no longer halved; N = index in the decomposition table; look at least at group B's L1#2 and R1#2)
python software/logs_analysis/ab_quant.py ... --zoom 7
```

⚠ Do not point `--out` at /tmp: the frame cache runs to several GB for a video
of a few minutes, and once the /tmp quota is full every Bash command on the
machine fails silently (already hit on this machine). Use a directory under
home, and clear the cache with `rm -r ~/ab/*/seq` when done (traj.csv and the
report are tiny, keep them).

Where to read: the "vent jump" column of the decomposition table = the δ
feedback signal (**based on round 2**); the "handover segment" column = the
mean-invariance check; "round N slip" = the overall criterion; the current
section = the ratchet criterion. The tracking-health line requires a median
body_score >0.8 and ref jitter <2px (the baseline was 0.875 / 1.76px).

## 6. Judgment and δ correction rules

Start with the four items in §1. Branches:

- **A slight upward bounce on the early legs of round 1 (-1 to -3mm) =
  expected, do not touch δ**. 1-D model: in the handover steady state the energy
  stored per lift equals the baseline steady state (calibrating δ from the
  baseline bounce is right for the steady state), but in round 1 the stored
  energy ramps from mg/6k up to steady state (14.8→36.9 in the model; the
  baseline's measured round-1 vent jumps 2.3→16.3mm show the same ramp) — the
  early legs of round 1 must overshoot slightly. **Judgment always looks at
  round 2.** Also: the touchdown-reset artifact of the per-leg δ table (the ⚠
  note on criterion 4 in §1) makes group B's round-1 slip reading **~5mm
  smaller** than physical reality — round-1 numbers looking good is expected and
  equally does not count.
- **A lift whose handover the engine clipped does not feed into the δ
  correction**: when the script prints "⚠ handover clipped at the envelope"
  (also recorded in the black box), that lift did not get its full δ and its
  bounce must be larger — that is a workspace accounting problem
  (stride/drift), not too little δ, and adding δ for the residual would pour
  fuel on the fire. In a valid A/B group the clip count should be 0; if any
  appear, check the stride and the stance position first and re-run that group.
- A leg still sinking >3mm in round 2: δ_i ← min(δ_i + 0.8×residual/|slope_i|,
  45). slope_i is that leg's measured "bounce vs δ" slope (08-24: L1 0.57 /
  R1 0.39 / L3 0.21 / R3 0.25 / R2 0.13 / L2 0.14); with no slope data,
  conservatively use 0.8×residual×5 (the uniform-stiffness approximation). The
  old clamp of 21 was the "bounce as stored energy" mistake (design §5 ⚠) and
  was abolished together with the parser limit going 25→45;
- A leg bouncing up <-2mm in round 2: δ_i ← δ_i − 0.8×|upward bounce|;
- After changing the table, re-run the whole group as B′ (back on the wall);
  budget one afternoon for the three groups A + B + B′ (~10min on the wall per
  group + ~3min of changeover).
- **Stop on the spot** if: the body sinks systematically during the handover
  segment (a mechanism bug — back to the bench and re-check the direction and
  apportionment of step 4.7); any leg bounces up >5mm (δ far too large, halve
  the whole table and retry); round-1 slip goes up instead of down (an error on
  the scale of a reversed direction).
- Freeze handling as before: a leak on a stance leg **or on the handover leg
  itself** during the handover pauses it automatically (since v1.6 the handover
  leg is under the watchdog; a longer Z segment is normal), and after a
  rescue-timeout freeze press `f` to resume;
  `clear_freeze` does not lose a handover in flight (locked in by tests §7.6).

What comes after passing (design §9.4): climb_walk, single steps (`i`) first and
then continuous, fine-tune δ, and re-measure the net advance rate (26% baseline)
— ab_quant now parses climb_walk logs directly (mixed single-step and continuous
runs are fine, the timeline is built from the phase-transition lines; the vent
of the exit sequence is filtered out and not counted as a lift-and-land — the
08-19 climb log really contains 10 lift-and-lands, and the 16 obtained by
counting vent events was wrong); once δ is stable, another group can be run with
`--handover-weights` (loads weighted by rotation distance in the window order,
design §5 v1.5; the convention assumes legs are lifted in the rotation shown by
the "next is X" prompt) as an A/B — the model expects internal stress within a
cycle to fall ~30% and the δ table to be re-calibrated downward overall, so be
sure to make it a separate group from the δ calibration and do not mix
variables; the report goes into the html (the pipeline output feeds the same
charts as `vent-snap-20260820.html`). **Keep --sag-comp at 0 during the A/B**
(running both pushes the stance system twice, design §4.3).

## 7. Record sheet to fill in (on site, on your phone)

| Item | Group A | Group B |
|---|---|---|
| Log file name | | |
| Video file name / was the camera moved? | | must be "no" |
| Battery start V | | |
| Tape-measure reading, start/end (group A) | | — |
| By eye: any visible bounce at vent | | |
| By eye: does the body move during the Z segment | — | |
| Freeze / leak / anomaly | | |

## 8. Dual-swing (--dual) calibration and A/B addendum (DUAL-SWING-DESIGN §6, 2026-08-24)

Dual swing = two swing windows (duty 4/6, always 2 legs swinging / 4 feet
attached). **The δ table is not interchangeable with the single-swing one**: the
catching side goes 5→4, so bounce readings scale ×5/4, and the steady-state
stored energy changes (the static load share goes mg/5→mg/4, and the rhythm is
3 windows), so δ*_pair is expected to be +10 to +30% above the single-swing
values. The slopes change with the number of catching legs too, so re-measure
them at the same time.

### 8.1 Three-group δ calibration for dual swing (body_lean --dual, paired lifts in place)

```
# Three groups = the current single-swing table ×0.5 / ×0.75 / ×1.0 (not starting from δ=0: a bare
# dual-swing bounce hurts more, and a linear extrapolation does not need the zero point anyway.
# Current single-swing table = 08-24 δ*×0.8: L1:31,R1:33,L3:29,R3:22,R2:22,L2:24)
python scripts/body_lean.py --no-tank --stand-height 62 --tilt-trim 6 --dual \
    --handover L1:16,R1:17,L3:15,R3:11,R2:11,L2:12     # the ×0.5 group
...  --handover L1:23,R1:25,L3:22,R3:17,R2:17,L2:18    # the ×0.75 group
...  --handover L1:31,R1:33,L3:29,R3:22,R2:22,L2:24    # the ×1.0 group
```

- Key sequence: `p` to start → **one-key `t` (§4.3: synchronization marker +
  automatic rotation; under dual swing it automatically lifts pair by pair,
  3 pairs per round, with the pair order guaranteed by the engine's cyclic
  rotation; the synchronization marker must not be skipped — the lesson of the
  three groups on 08-24 whose synchronization was all wrong: motion-contrast
  synchronization can lock a whole number of slots off the cycle rhythm and
  still score high)**. Manual fallback: after the synchronization marker, pick
  the pair's first leg with a number key
  → `i` to lift the pair (the two legs of the pair hover 0.6s apart) → once both
  hover, `i` lands them together (staggered descent). The pair order rotates
  cyclically: L1+R2 → L3+R1 → L2+R3.
- **≥2 full turns per group (3 pairs per turn), discard the first turn**: on a
  cold start the first window has only one swing leg and the apportionment lands
  on 5 stance legs, a different condition from the steady-state 4 (the same
  discipline as "look at round 2" in the single-swing protocol).
- The engine staggers the vents by ≥0.4s (the window-head offset within a pair +
  a guard): at 30fps the two ruptures are ≥12 frames apart, so per-leg vent
  jumps are still separable — the quantification pipeline's basis is unchanged.
- Extrapolate the per-leg line to δ*_pair → **start at ×0.8**. For a leg whose
  δ*_pair breaks through the parser limit of 45: the first remedy is to turn the
  weights on (next item; the model's δ* drops accordingly); if it still hits the
  cap, record the measured value per decision C and discuss raising the limit
  against the room budget (comp_tail − half stride), without borrowing against
  it in advance.
- **Weights (usable together with dual swing since v1.1, 08-24)**: if the wall
  runs are going to use `--handover-weights` (single-swing measurements show a
  clear improvement in slip), run the three δ_pair groups **with the weights
  on** (the weights change the steady-state operating point, and the δ table
  follows the condition). The dual-swing preset = with no value it automatically
  takes **1,1,1,0,0** (an equal third each to the pair that just landed and the
  trailing leg of the next pair; shares are valued by rotation distance and
  normalized in place; the model gives −12% on the worst single leg / −21% on
  the mean, DUAL-SWING-DESIGN appendix C); **do not carry over the aggressive
  single-swing preset** (model: the trailing leg of a pair is pushed above the
  uniform share, 32.9>30.8). The weights only apply if the pair order follows
  the cyclic rotation (which the key sequence in this section already does); a
  pair that deviates from the rotation is sent back to uniform shares by the
  engine guard, with a notice — seeing that notice in a calibration recording
  means the key sequence went wrong, and that group is void and must be re-run.
- ab_quant: the second leg of a pair goes hover→descend through the staggered
  landing queue, and the parser now handles that transition (2026-08-24); both
  legs of every pair should appear in the decomposition table, and a missing leg
  means checking the parser first and then the log.

### 8.2 Dual swing vs single swing walking A/B (climb_walk)

Run a stretch of continuous straight walking with the same parameters for each
(`--dual` the only difference; each uses its own δ table — single swing the
single-swing table, dual swing the table calibrated in 8.1, **this is not mixing
variables**: δ is the correct operating point for each convention):

| Metric | Notes |
|---|---|
| Net advance rate mm/s | The overall criterion: dual swing passes at ≥1.8× single swing (theory ~2.5×) |
| Seal-rupture events per meter | Theory −20% per meter (6 events per 48mm → per 60mm) |
| Displacement per event | With dual swing 4 legs catch, so the same residual δ gap reads ×5/4 — passing means no worse than single swing at the same δ level |
| Current ratchet slope | The probe for internal stress accumulating between the legs; with 4 stance legs the whole baseline is +25%, which is the static load share, so look at the slope, not the absolute value |
| Worst cup-pressure distribution / gate-wait count | Whether the pump keeps up with twice as many new cups per window (watch this especially in real-tank mode; the fallback is pump B) |

⚠ Rope protection at all times under dual swing (the support redundancy drops
one level: the instant one cup leaks = 3 feet carrying = the design's "extremely
conservative" line); `--sag-comp` is refused by the script under dual swing
(decision B); `--handover-weights` can be used together since v1.1 (the
dual-swing preset 1,1,1,0,0) — variable discipline: the weights on/off is its
own A/B group, do not compare it in the same group as the δ level or the
single/dual convention.

## 9. D′: the handover-rate discrimination experiment (opened 2026-08-26, the direct sequel to the T1 fit)

**Origin** (html/en/stiffness-fit-20260826.html): the n=3 fit of the B/C
handover segments shows ~nine tenths of the loss is "per-window common-mode sag
≈ creep rate × window duration" (B 60.2 = 6.7 redistribution + 55.5 common mode;
C 39.7 = 7.7 + 30.0), and a stiffness-aware share allocation can only move the
redistribution term (a 3–4mm gain; the original D design was rejected at the
gate). The window duration ≈0.5+δ/rate is perfectly collinear with δ ⇒ "time
creep" and "per-mm loss" cannot be told apart offline — **changing the
application rate is the only means of discrimination, and if time creep holds we
pick up ~12mm for free along the way**.

**Condition** (a replica of C + one new variable, `--handover-rate 20`):

```
# Group C (control, the 08-25/26 convention unchanged)
python scripts/body_lean.py --no-tank --stand-height 62 --tilt-trim 6 \
    --handover L1:31,R1:33,L3:29,R3:22,R2:22,L2:24 --handover-weights

# Group D′ (the only difference: application rate 10→20mm/s; δ table, weights and everything else identical)
python scripts/body_lean.py --no-tank --stand-height 62 --tilt-trim 6 \
    --handover L1:31,R1:33,L3:29,R3:22,R2:22,L2:24 --handover-weights \
    --handover-rate 20
```

**Design**: ≥3 same-day pairs in alternating order (C→D′ / D′→C / C→D′), with
the same statistical basis as the n=3 main table; each group is one-key `t` in
body_lean (the 60s opening rest is built in — with the recording running you
just press t, §4 step 2); battery ≥8.0V, the tripod untouched between groups,
and the scale taken per group from the long edge of the board.

**Reading** (quantify with ab_quant as usual, feeding the per-leg handover
segments into a fit_stiffness re-run):

| Result | Verdict | Follow-up |
|---|---|---|
| D′ handover segment drops in proportion to the window duration (39.7→~25mm per group, C total ~72.9→~58) | Time creep confirmed | The creep law goes into the model section of the paper; 30mm/s can be tried to squeeze one more level (still within the ≤50 guardrail, judge the dynamic risk yourself) |
| D′ handover segment ≈ unchanged | Per-mm loss (the loss follows the amount applied, not the time) | Rewrite the model (the common-mode term ∝δ); raising the rate gains nothing, but the discrimination itself goes into the paper |
| In between | Both mechanisms present | Split the two coefficients proportionally, and add a mixed model to fit_stiffness |

**Sanity checks alongside** (these should not change between the two groups; if
they do, raising the rate introduced new physics — record it on the spot):
per-leg vent jumps (the δ unloading ratio should not depend on the application
rate), the transfer ratio (from the marker), and the current increment;
criterion 4 (the handover segment is smooth with no steps) keeps its
stop-on-the-spot basis as usual — a step appearing after the speed-up = a
dynamic effect, so drop back to 15 and try again, or tighten the condition.

**Result (run on the wall 2026-08-31; the reading hit the second row, "per-mm
loss" — full report html/en/dprime-20260831.html)**: over three pairs, D′−C on
the handover segment was −2.9±2.1mm (p=0.14, far from time creep's predicted
−13.5), and −1.7±0.7mm (−3%) over the three-round total; aligning each handover
on its vent, D′'s ramp is twice as steep while the plateau's final value is
identical to C's — the sag is ∝ the amount applied and independent of the time
taken. A joint model race over 216 points and four conditions has M10 (common
mode = κ·δ) at AICc 0.0, beating M8 (∝ window duration) at 17.8; κ reproduces
across 5 days in the same condition (0.0550 vs 0.0529mm/mm), and with uniform
shares κ_B = 0.0988 = the rotation weights press κ down by 44%. **The
common-mode law is rewritten: Δb = κ·δ_j·(1+0.20(r−1)), with κ determined by the
share allocation; raising the rate has no practical benefit, and
--handover-rate is kept as a parameter verified to be harmless (still 10 by
default)**. All the sanity checks passed; the deviation = splitting the adjacent
handover / vent columns is sensitive at the ±0.3s level (the vent column of
D1/C3 reads high), while the merged column is unaffected.
