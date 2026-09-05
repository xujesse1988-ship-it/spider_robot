> English translation of [`docs/PAPER-PLAN.md`](../PAPER-PLAN.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Paper plan: mechanism, quantification and sensor-free elimination of the vent-snap ratchet on a wall-climbing robot

Status: **the main data set is complete (2026-08-26, n=3 Latin square); we are
in the remaining-work and writing phase**.
08-26 progress: T5 done (docs/en/RELATED-WORK.md), step 1 of T1's model fit done
(html/en/stiffness-fit-20260826.html: the gating check vetoed the original D
design, and the D′ laying-speedup discriminating experiment was set up
instead), the C2 time-sync correction (column C of the n=3 report refreshed,
deviation ①), T4a/T4b promoted to mandatory, and convention/wording
corrections (§1/T6). **08-31 progress: the D′ discriminating experiment was
completed on the wall (html/en/dprime-20260831.html) — the readout says "loss
per mm", the common-mode law is rewritten as Δb=κ·δ·(1+0.20(r−1)), κ
reproduces across 5 days and the weights cut κ by 44%; the main body of the
experimental line is closed out, leaving T4a/T4b/the long run (non-blocking),
and the focus moves to writing (T6).**
**Evening of 08-31: T6's full English draft v0 is done (`paper/main.tex` +
`refs.bib` with 45 entries + 4 real figures), and the T3 rupture-profile figure
is done — the remaining to-dos are concentrated in `paper/README.md`
(author information / weighing / the Fig.1 photo / T4a / T4b / video /
open-source package / full-text verification of references / cutting length).**
This document = the master plan for the paper effort, so a new session can pick
it up directly; background details are in the documents cited here, no need to
read back through old sessions.

---

## 1. Positioning and contribution claims

**In one sentence**: on a suction wall-climbing robot with compliant leg
chains, the leg chain releases its stored elastic energy at the instant of seal
rupture and causes ratchet-like slip (the headline-number convention was
settled on 08-31: the main number in the abstract/body = the **93%** from n=3,
with the 83% of 08-20 cited as the first observation and a note on the
difference in segmentation windows); we propose a purely kinematic,
zero-sensor "zero-force handover + rotation weights" scheme that cuts
three-round slip by −61% (p≈0.001), and we report two
experimental findings (handover sag from uneven stiffness, and a ~42%
compliance transfer ratio; use the wording "experimental finding" — finding 2
is a characterization of this machine, do not write "new physics").

Three contributions (from the reviewer's point of view):
1. **Discovery and quantification of the phenomenon**: the vent-snap ratchet
   phenomenon itself is the contribution; the phone-video pipeline (NCC
   tracking + step-basis decomposition + body-lean marker time sync) is
   positioned as low-cost methodological support and reproducibility evidence
   (188 check images + an open-source data package), and is not claimed as
   independently novel;
2. **Mechanism model and verified predictions**: the parallel-spring model —
   bounce-vs-δ linearity (the δ* extrapolation calibration method, independent
   of the scale factor), per-leg slope = stiffness difference, weight dilution
   (model −32% vs measured −33%), stiffness-weighted equilibrium (finding 1's
   first-order prediction 4.6 vs measured 6.8mm);
3. **Sensor-free solution and n=3 evidence**: δ pre-laying + share rotation +
   rotation-distance weights; open loop, purely kinematic, no force sensors;
   n=3 Latin square −43%/−61%, rupture segment −82%/−88%.
   ⚠ T5 boundary (08-26): ❌ "first to propose pre-detachment unloading" cannot
   be written — the Stickybot patent US7762362B2 and LORIS (ICRA 2024, with an
   explicit Unload step) already have force-domain closed-loop versions;
   ✅ the claim = first to convert the unloading condition into the
   **displacement domain** (per-leg offline calibration of the δ returned +
   mean-invariant apportionment), implemented **purely open loop** with zero
   sensors, and validated by quantifying rupture-segment slip
   (docs/en/RELATED-WORK.md §0).

Venue: RA-L (first choice), or arXiv first. **Deadlines verified (08-26, from
the ICRA 2027 official CFP)**: the ICRA 2027 (Seoul, 5/24-28) direct-submission
deadline for the main conference is 2026-09-15 (8 pages including references,
video ≤20MB/180s, upload windows 8/5-9/9 and 9/17-22); RA-L no longer has a
joint-submission mechanism — it is rolling submission, and **after acceptance**
you automatically get the right to present at a RAS conference, the deadline
for transferring to ICRA 2027 being 2026-12-31.
Branch rule: **RA-L by default** — do not rush 9-15; finish condition D under
the one-variable-at-a-time discipline, targeting submission in early
~October; if it is finally accepted before 12-31 the ICRA talk is a free bonus,
and if it misses, transfer to IROS 2027. Only if we clearly want a conference
paper instead would we consider direct submission on 9-15 (D would not be
finished by then = submit the A/B/C version, with D demoted to a model
prediction + future work) — do not sacrifice the one-variable discipline for a
deadline.
Literature positioning (the six-line T5 deep dive finished 08-26, details in
docs/en/RELATED-WORK.md): **no direct collision**; neither the quantification
of the phenomenon (whole-body position loss at the instant of negative-pressure
rupture) nor "open-loop unloading in the displacement domain" has a precedent;
but the idea of "pre-detachment unloading" does have force-domain closed-loop
precedents (the Stickybot patent US7762362B2, LORIS ICRA 2024), so the wording
of contribution ③ has been narrowed (see above). The Top-6 nearest neighbors,
~60 graded references and the writing skeleton are all written up; the
remaining manual check list is in RELATED-WORK §7 (the body of Kumar & Waldron
1990, the open/closed-loop details of FTFOF, the NWPU (Northwestern
Polytechnical University) motion-switching strategy, CNKI master's/doctoral
theses, etc.).

## 2. Core numbers at a glance (quote these directly when writing)

n=3 Latin square (#1 A→B→C / #2 B→C→A / #3 C→A→B; A = baseline, B = + the δ
calibration table 31/33/29/22/22/24, C = B + weights 0.6/0.25/0.1/0.05/0; the
scale factor is measured per group from the 93.0mm long edge of the YYNMOS-8
board, 0.288–0.296mm/px):

| Metric (mm) | A | B | C |
|---|---|---|---|
| Three-round total slip | 185.5±3.0 | 105.6±2.2 (−43%, p=0.0013) | 72.9±3.0 (−61%, p=0.00092) |
| Round 2 | 63.9±0.9 | 38.1±1.0 | 26.5±1.7 |
| vent rupture segment | 172.9±3.4 | 31.6±1.0 (−82%) | 20.1±3.3 (−88%) |
| Handover apportioning segment | — | 60.2±2.2 | 39.7±4.3 (−34%) |
| Current rise | +0.97±0.02A | +2.40±0.08A | +1.85±0.08A |

(Column C was refreshed on 08-26 after the deviation ① time-sync correction:
the C2 step rescue had once locked onto the wrong comb tooth by one; it was
re-decomposed with the clip-window marker method at off=59.34, the total is
conserved while the split and the per-leg attribution are corrected — see the
n=3 report.)
C−B = −32.7±2.7mm (p=0.0022). Group A's L1 round-2 bounce is 20.1±1.3,
reproducing the 08-20 baseline of 20.9 5 days later. **Finding 1**: 57–62%
of the B/C residual is in the handover apportioning segment, concentrated on
the stiffest legs L1/R1 and growing round by round (the three B replicates:
6.8→12.6 / 6.9→10.6 / 6.7→11.9mm) — the equal-stiffness assumption fails, and
the elastic equilibrium is a stiffness-weighted average. **Finding 2**: a ±10mm
marker gives a measured body displacement of 4.22±0.30mm = a compliance
transfer ratio of **42.2±3.0%** (9 markers, 3 days, 3 camera positions; the C2
marker was included after the time-sync correction).

**Statistical safeguards (do these proactively while writing)**: report effect
sizes, not just p; add a Bonferroni sentence for the pairwise comparisons (all
still <0.007 after ×3, nothing to fear); state explicitly that n=3 =
independent replicates of the whole procedure, each replicate containing
3 rounds × 6 legs = 18 event-level observations; give exact values where
p≈0.001 (C−A: t=32.9, df=2, p=0.00092).

## 3. Index of material and tools

- **Reports**: html/en/handover-ab-n3-20260826.html (the final n=3 version,
  statistics/deviation table/conclusions; column C refreshed on 08-26 per
  deviation ①), html/en/dprime-20260831.html (the D′ discriminating
  experiment: three matched pairs / model verdict M10 / aligned profile plot /
  the κ law),
  html/en/stiffness-fit-20260826.html
  (T1 model closure: k̂ cross-checked two ways / decomposition ledger of the
  handover segment / gating verdict / D′ design),
  html/en/handover-ab-20260825.html (the detailed ledger of replicate #1 +
  derivation of the scale-factor convention),
  html/en/vent-snap-20260820.html (the 83% baseline),
  html/en/press-lurch-20260819.html,
  html/en/handover-delta-calib-20260824.html (δ*/slope calibration);
- **Nine groups of raw data**: logs
  software/logs_analysis/lean_2026082{5,6}_*.log (in the repo); the original
  MOV footage in images/ (this machine only); per-group decomposition
  report.txt + traj.csv under ~/ab_cache/{0825,0825pm,0826}/{A,B,C}/ (the seq
  frame cache can be deleted, keep traj/report);
- **Tools** (software/logs_analysis/): ab_quant.py (the pipeline, including the
  rotation-metadata fix and --off injection), aggregate.py (nine-group
  aggregate statistics; it contains the RUNS table = the directory / log
  timestamp / per-group scale factor of the nine groups), fit_stiffness.py (the
  T1 fit: within-B model family + a joint common-mode mechanism race + the
  gating ledger; --d31 folds in the six 08-31 groups, re-runnable),
  aggregate_dprime.py (D′ three-pair matched statistics + readout table +
  sanity checks), marker_off_clip.py (clip-window marker
  time sync — the rescue for when the recording started late and only the tail
  of the marker was captured; use it before the step method), step_off.py
  (step-basis time sync; ⚠ when the comb teeth become periodic it locks onto
  the wrong tooth, the lesson being in deviation ① of the n=3
  report; before using it, look at the vent spacing it prints);
- **Measurement evidence chain** (188 check images, in the repo):
  images/ab-verify-202608/ — template boxes locate*_*.jpg, board scale
  board_scale*.jpg (four corners, subpixel), tape-measure manual checks
  ruler_A_f*.jpg (the operator has confirmed the readings of 4 frames), 12
  corners at 10x magnification ver_*.png. Material for the paper appendix /
  open-source package;
- **Design documents**: HANDOVER-DESIGN.md (mechanism + the Appendix A spring
  model), HANDOVER-AB-PROTOCOL.md (the protocol, §4 = the one-key t
  procedure), DUAL-SWING-DESIGN.md (Appendix C, the dual-swing model);
- **Stiffness data**: the 08-24 slope table L1 0.57/R1 0.39/L3 0.21/R3 0.25/
  R2 0.13/L2 0.14 (protocol §6); relative stiffness converted as k̂ ∝
  slope/(1+slope), normalized to L1 0.29/R1 0.22/R3 0.16/L3 0.14/L2 0.10/
  R2 0.09.

## 4. To-do (by priority; adjusted 08-26: T5 moved up, T4a/b promoted to mandatory; T1 and T2 advance together)

### T5 literature deep dive ✅ done (08-26, six lines in parallel: walking machines / suction cups + Japanese /
multi-limb internal forces / dry adhesives / cross-cutting / Chinese-language sweep, ~350 searches)
Produced docs/en/RELATED-WORK.md: ~60 references graded into five clusters +
novelty boundaries per contribution + a six-paragraph writing skeleton for
related work. Key points: no direct collision; the quantification of the
phenomenon and "open-loop unloading in the displacement domain" have no
precedent; the idea of "pre-detachment unloading" has force-domain closed-loop
precedents (the Stickybot patent / LORIS's Unload step), so the wording of
contribution ③ has been narrowed (§1); the walking-machine classics all execute
in the force domain, confirmed (Chen 1999's "sensor-free" refers only to
computing the setpoint); TITAN XI / Ota 2006 = precedents for the same means,
displacement-domain calibrated feedforward (their goal is foot-placement
accuracy / static gravity deformation; must be cited and delimited); on the
Chinese-language side there is no quantification precedent (three surveys
checked in full text; behind the CNKI wall is the only remaining blind spot).
**Remaining** (folded into the T6 writing period): the manual full-text check
list of RELATED-WORK §7
(the body of Kumar & Waldron 1990, the open/closed-loop details of FTFOF, the
NWPU motion-switching strategy, CNKI master's/doctoral theses).

### T1 stiffness model closure: step 1 ✅ done (08-26) → gating vetoed the original D, D′ set up instead
Full results in html/en/stiffness-fit-20260826.html; re-run fit_stiffness.py.
Key points:
1. **Model closure achieved**: RMSE 0.85mm within group B (CV 0.99 with
   leave-one-replicate-out), and the fitted k̂ and the slope-derived k̂
   cross-check each other at r=0.938; joint B+C selection over 108 points (after the C2
   correction) picks M8 (common mode = creep rate × window duration). k̂: L1
   0.235/R1 0.217/L3 0.143/R3 0.138/R2 0.138/L2 0.129 — the L1−R1 difference is
   far smaller than in the slope table, and the old saturation anxiety of "L1
   has no solution, everything gets pushed onto R1" disappears once k̂ corrects
   it (residual coefficient only 0.019).
2. **The decomposition ledger overturns the earlier reading**: B's handover
   segment 60.2 = redistribution 6.7 + common mode 55.5; C's 39.7 = 7.7 + 30.0.
   About nine tenths of the loss is per-window common-mode descent (ρ_B
   0.80→ρ_C 0.43mm/s — the weights are already cutting the common mode), and
   uneven stiffness only sets the shape of the distribution: the soft legs'
   negative redistribution cancels the creep pedestal, which is what makes it
   look "concentrated on L1/R1". Rewrite the mechanism statement of finding 1
   accordingly (paper material).
3. **Gating verdict: the original D (stiffness-aware shares) does not go on the
   wall** — shares only move the redistribution term, worth ~3mm in B / ~4.4mm
   in C, so "handover segment 60→10" does not hold. The share solution is
   demoted to a merged micro-optimization after D′ (the original implementation
   list is in git history; fetch it if it is ever enabled).
4. **D′ discriminating experiment ✅ done (08-31, on the wall,
   html/en/dprime-20260831.html)**: the readout hit "loss per mm" — D′−C is
   only −2.9±2.1mm on the handover segment (time creep predicted −13.5) and
   −1.7±0.7 in total; the aligned profile = D′'s ramp is twice as steep while
   the plateau matches C. A joint race over 216 points has M10 (common mode =
   κ·δ) at AICc 0.0 beating M8 (∝ window time) at 17.8 outright —
   **the common-mode law is rewritten as Δb=κ·δ_j·(1+0.20(r−1))**, with κ
   reproduced across 5 days (0.0550/0.0529) and uniform sharing 0.0988 →
   weights 0.0550 (**share allocation is the real lever on κ, cutting it 44%;
   time is not a lever, so going faster buys nothing**). κ jointly with k̂: L1
   0.265/R1 0.227/L3 0.137/R2 0.133/R3 0.125/L2 0.112. The residual handover
   segment of ~37-40mm = the inherent cost of pre-laying δ at the current κ;
   optional next step: a share-perturbation experiment to separate
   "concentration vs cup age" (squeezing κ further), which does not block the
   writing. Tools: aggregate_dprime.py, fit_stiffness --d31.
5. **Keep the 8–10 round long run** (~20 minutes, condition C): γ=0.12/β′=0.21
   and the current are all unsaturated over three rounds, and both the
   long-distance extrapolation and the validation of the ramp term rely on it.
6. **T4a hanging weight is promoted to a companion of T1**: hang the load, let
   it settle, and read the slope of y(t) = a direct measurement of creep rate
   vs load (turning ρ from a fitted parameter into an independent
   measurement), while also giving a third-party check on the absolute k and on
   the transfer ratio.

### T3 rupture-profile figure ✅ done (08-31, `ab_quant --zoom 7` @ 0825/A)
Figure `paper/figures/fig_staircase_zoom.pdf` (top = the whole staircase,
bottom = the full-frame-rate profile of L1#2; generating script
`paper/figures/make_figs.py`, the zoom frame cache
~/ab_cache/0825/A/clip_7_L1 can be deleted). Profile numbers: flat to ≤0.08mm
for 1.2s before release, a 21.5mm drop within two frames, an overshoot peak of
26.7, and 12.8mm of rebound at the released foot (max 17.3).
⚠ One convention caveat found: in this event the mechanical release sits right
on the valve opening (+0.03s) while the "cup pressure to zero" log event is
0.40s later — unlike the 08-20 "rupture ≈ cup pressure to zero"; marker time
sync cannot tell truth from fiction within ±a few tenths of a second, so the
paper's figure caption is written as "the valve opening and the cup pressure
reaching zero bracket the mechanical release, and its position relative to the
video carries the time-sync uncertainty"; the four hard conclusions — flat, the
two-frame drop, ringing, rebound — are all unaffected.

### T4 supplementary experiments (adjusted 08-26: a/b promoted to mandatory — highest persuasiveness per unit cost; c optional)
- **T4a hanging-weight stiffness (mandatory, ~10 minutes)**: all six feet
  attached and settled, mark with t --auto-rounds 0, then hang accurately
  weighed 200/500g loads step by step, settling 20s each — the absolute
  whole-robot shear stiffness k_total=Δmg/Δy turns the model's energy ledger
  from relative into absolute and independently cross-validates the transfer
  ratio and the k̂ normalization; it is the foundation of the model chapter;
- **T4b climb_walk net-advance-rate demo (mandatory, ~10 minutes per
  segment)**: continuous straight walking, one segment with the weights on and
  one with them off, comparing net advance rate (08-19 baseline 26%). The n=3
  main experiment is a stepping-in-place proxy task, and a reviewer is bound to
  ask "show me it climbing" — 26%→X% is a number that can go into the abstract,
  and it is also submission-video material;
- T4c load scaling (optional): run three rounds in condition C with 200g hung,
  to test the model's scaling prediction that stored energy ∝ load².
- **T4d target cross-validation run (added 09-01, ~1 hour, after T4a/T4b)**:
  print the Ø20 black-and-white concentric circles of
  `html/en/tracking-targets-printable.html` (at 100% scale, measure the 100mm
  check ruler first), stick two of them at a known spacing on the plane of the
  body board (the same plane as the existing scale reference), one on the glass
  as a static reference, and optionally a small target on a foot; run one group
  of the in-place baseline and track the same video segment with both the
  "board template" and the "target" templates (just crop the target area as the
  template in ab_quant, no code change), then compare the trajectories → in the
  paper's III-B write "board-template tracking and target tracking agree to
  within X mm", replacing the current phrase "the annotated frames were
  manually checked"; the spacing between the target centers incidentally gives
  a scale check independent of the board-corner readings (closing T7's second
  item at the same time); the foot target can turn "foot rebound of 9–15mm is a
  lower bound" into a true value. **From T4d on, every on-wall video (T4a/T4b/
  the long run/share perturbation/the submission video) carries targets.**
  Verdict (09-01): **do not redo n=3/D′** — the effects of 43–88% are 1–2
  orders of magnitude larger than the measurement noise, a low body_score is
  match confidence rather than displacement error, and two days of re-running
  would not change any conclusion; if there is time left after T4a/T4b, the
  "built-in 60s settle + fixed camera + recharging between groups + targets"
  version can be run as a second 3×3 Latin square alongside the existing data
  (n=3→6, the two tracking methods cross-validating), which is icing on the
  cake and not a gate.
  Related: of the 188 images in `images/ab-verify-202608/`, only board_scale×9
  + locate×15 (+12 tape-measure) are self-explanatory check images; the other
  ~150 are corner-reading process images; the open-source package ships only
  these 36 + an index README, with the process images moved into a
  subdirectory; until T4d is done, the paper's wording is changed to give no
  image count and to avoid the phrase "manually checked" (all the operator
  confirmed was 4 tape-measure frames).

### T6 paper skeleton and writing → **full English draft v0 done (08-31, `paper/`)**
`main.tex` (IEEEtran journal, ~6500 words, the red `\TODO` items are the
to-dos) + `refs.bib` (45 entries, unverified fields carry TODO(verify)) +
`figures/` (Fig.2 staircase + profile, Fig.3 bounce-vs-δ, Fig.4 waterfall +
per-leg, Fig.5 D′ aligned profile are all real figures; the Fig.1 platform
photo still has to be taken). Structure = ①–⑤ below, all implemented: the two
pre-rebuttals written into the Discussion, the three-piece statistical
safeguard into §VI-B, the deviation table into §VI-G, and all of T5's wording
red lines executed (contribution ③ narrowed / the −82% dimensional coincidence
called out / finding 2 written as a platform characterization).
The remaining to-dos and how to compile are in `paper/README.md`. The original
plan's key points, archived:
① phenomenon + quantification (the headline convention is settled: the 93% from
n=3 as the main number, the 08-20 83% cited as the first observation);
② the model (parallel springs + uneven stiffness, with a list of four verified
predictions); ③ the method (δ pre-laying / shares / weights / stiffness-aware,
zero-sensor open loop); ④ the experiments (the n=3 Latin square main table +
findings 1/2 + the current cost + the three-piece statistical safeguard of §2);
⑤ the discussion — **the two pre-rebuttals are mandatory**:
(a) "isn't this just fixing a robot you built badly yourself?" → compliance is
unavoidable in lightweight suction wall climbing, rupture-type detachment × a
compliant chain is a general combination, a pure software fix that touches no
hardware is exactly the value, and L1 having no solution gives a hardware
design rule in reverse (the stiffness upper bound of the support set decides
what can be compensated); (b) "you have current telemetry, why not close the
loop?" → whole-robot current cannot be attributed per leg, open loop needs no
estimation and no tuning, and the simpler slow vent was already ruled out by
the 08-20 data — write open loop as a reasoned choice; plus the energy
trade-off, the generalization conditions = compliant leg chain + rupture-type
detachment, and the limitations. Figure/table list: the main table, per-leg
bounce bars, the decomposition waterfall (the three-segment ledger for A/B/C),
the rupture profile (T3), the bounce-vs-δ linearity (08-24 data), the
round-by-round growth of the handover segment (finding 1), the transfer ratio.
**Submission attachment video (mandatory)**: A vs C side by side + slow motion
at the instant of rupture (RA-L/ICRA accept multimedia, and a wall-climbing
paper with no video means disarming yourself; the material is in the original
MOV footage, this machine only).
Honest disclosure (copied straight from the n=3 report's deviation table): the
C2 time-sync rescue, camera position/battery, body_score below the internal
convention (backstopped by the nine groups cross-validating at CV 1.6–4%), and
the scale-factor parallax ledger (the board long-edge convention).

### T7 precision improvements (optional, non-blocking)
Rotation-tolerant templates / per-round template refresh (improvements for the
0.39–0.76 median body_score); a third-party check of the marker scale factor
against the board scale factor (photograph a caliper laid on the body plane).

## 5. Discipline reminders (for a new session executing this)

- On-wall experiments always use body_lean's one-key t (protocol §4); **make
  sure the time-sync marker is captured in the video** — the C2 lesson has been
  made structural: the 60s pre-run settle is built into the t key
  (--mark-settle, 08-26), so with the recording running you just press t;
  battery ≥8.0V for every group; do not touch the tripod between groups;
  changing the δ table or changing the share mode = changing the condition, so
  keep the groups separate, one variable at a time;
- Put the quantification cache in ~/ab_cache (not /tmp); measure the scale
  factor per group from the board's long edge, and use the markers only to
  measure the transfer ratio, never as the scale factor; **freeze the original
  MOV footage (this machine only) and do not delete it until the submission
  video is cut (T6)**;
- Judge A/B on round 2; the first round's ramp-up and landing-reset artifacts
  do not count (protocol §6).
