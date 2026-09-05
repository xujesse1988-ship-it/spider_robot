> English translation of [`docs/L3-DISPUTE-OPEN.md`](../L3-DISPUTE-OPEN.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# The tibia_len (l3) dispute · closed (2026-08-07, config = 123.7 written in)

> Background: P3 all-servo calibration is under way (tool `software/scripts/calib_fit.py`).
> Before tibia measurement can start, the model parameter `config.py: tibia_len` (l3 from
> here on) has to be nailed down, because samples use it to convert to angles at the moment
> they are recorded. femur_len has been measured and fixed at 81 (same-edge method, config
> already changed). l3 came out with three candidate values; this document records the
> evidence chain and the pending measurements — work through them one by one and the case
> can be closed.

## 🔒 2026-08-07 ruling: knee axis K = the servo output shaft, not the centroid of the four holes

**The "three candidate values" framework below is void as a whole** — the implicit premise
shared by all three candidates, "K = the centroid of the four hub holes (47.8, −14.6)", has
been overturned. Evidence and ruling (visualization: `html/en/tibia-three-view.html` v4):

1. **The four holes are the servo flange hole pattern**: the STL-fitted hole spacing
   9.98/10.01 × 48.00 = the DS3235 flange rectangle, not the bolt circle of a Ø49 round
   servo horn (that would require 34.6 between adjacent holes). The four corners of a
   rectangle are always concyclic (r=24.5), so the "concyclic residual 0.002" in
   LEG-GEOMETRY §2.4 never was evidence for "servo-horn holes". The 55×29×1.5 flange-shaped
   recess on the outer face of the hub (center 45.9, −13.4, sharing the long axis with the
   hole rectangle) is corroborating evidence. Confirmed on the physical part ✓.
2. **Physical ruling (1)**: the servo-horn center screw (K) sits **~10mm toward the hip**
   relative to the center of the 4-screw rectangle on the outer face of the hub ✓
   (the output shaft is ~10.25 from the end of the DS3235 body, and the flange is centered
   ⇒ the shaft is ~9.75 from the hole centroid).
3. **Physical ruling (2)**: tape-measured vertical drop from the K screw to the crease plane
   is **92mm** (the output-shaft convention predicts 92.8; the old K convention gives 85.4 —
   out).

**K = the output shaft ≈ (54.0, −7.2)±1** (part coordinates; = the four-hole centroid shifted
~9.75 toward the hip along the flange long axis).

Knock-on verdicts / reversals:

- ~~l3 (h_cup=21) = 125.5±1~~ → **2026-08-07, item c done: h_cup re-measured = 19** (July's
  21 was the L1 cup; cup batch and assembly vary, so measure h_cup per leg) ⇒ geometry chain
  l3 = √(52.7²+(92.8+19)²) = **123.65**, which **meshes to 0.05mm across two paths** with the
  08-06 sticker-Pythagoras value (123.7) — "was the lateral component subtracted that day"
  is thereby settled indirectly, and pending item d is downgraded to optional. **Verdict
  convention l3 = 123.7** (the tape-measure drop of 92 gives 122.9, within ±1); write it into
  config after f.
- **123.7 (08-06) is back**: it differs from the new convention by ~1.8 (h_cup=19.1 would fit
  exactly); but whether the lateral component was subtracted that day still has to be checked
  (see pending item d) — if it was not, the corrected value is ≈120.6.
- **116 and 120 are void** (both anchored to the old K). config currently holds 120; change
  it to ~125.5 once the wrap-up is done.
- **Dispute 2 reversed**: the lateral distance between the lip center (y=−4.47) and the plane
  of the K screw / horn face (y=−32, **confirmed by measurement**: 32mm from the K screw to
  the flat back face, on the same side as the door) = **27.5 (all measured)**; the "30"
  measured on paper back then was measuring exactly this (differing by 2.5, so they confirm
  each other) — it was not a bad measurement, it is a real offset.
  Consequence: the Pythagoras method must subtract the lateral term:
  **l3 = √(d² − Δy² + h_K²), Δy≈27.5** (slant-distance correction ≈3.0mm, not negligible).
- Assembly measurements on file (blue/purple annotated photos, 2026-08-07): **the servo horn
  is on the same side as the door (−y)** — the servo goes into the hook-shaped opening of the
  hub from the door side, its flange lands in the 55×29×1.5 recess on the outer face, and the
  hub's 4×M3 are its locking holes (pending item h closed); the shaft passes through the
  clearance hole in the femur rod end, **K center screw to flat back face measured = 32
  (y=−32)**, with the servo horn pressed against its inside (further out than the door's outer
  face at −21.5), locked to the femur rod end by 4 small screws + the center K screw; the
  servo body spans y≈−26..+18, and its tail end is supported by another rod end (pivot still
  pending). The claim "flat back face (y=0) = the plane the servo horn sits against" is void;
  y=0 is only the generator's coordinate datum. The cup axis (1.30,−4.47) ⊥ crease plane is
  unaffected.

### Wrap-up to-dos (change config when they are done)

- [x] **c. Re-measure h_cup in the free state (2026-08-07)**: = **19** ⇒ l3 = 123.65, meshing
      with the Pythagoras value 123.7 to 0.05mm. Verdict convention **123.7**.
- [x] **d. Closed indirectly**: the two paths confirm each other ⇒ the 08-06 handling of the
      lateral term is equivalent to having subtracted it; if it is redone (optional):
      l3=√(d²−Δy²+h_K²), Δy≈27.5.
- [ ] **e. coxa lateral-offset check** (kept as is, see the archived pending item e).
- [x] **f. Second leg re-measured and consistent (2026-08-07)** → `config.py: tibia_len` has
      been changed to **123.7** (**if the calib REPL is open it must be restarted to take
      effect**). **The l3 dispute is hereby closed**, and batch tibia calibration is unblocked
      (height-difference method recz still to be added / trilateration method recd, with an FK
      spot check <5mm as the backstop acceptance).
- [~] **g. Knock-on doc revisions**: workspace_analysis.py `CUP_DELTA` −22.7 → **−25.2, already
      changed** (2026-08-07; the comment carries stance phase 27.6°/l3≈113.8, and P4 picks the
      value by phase). Still open: rewriting the body text of
      LEG-GEOMETRY-OPEN.md §2.4/§3.4/§2.13/§4.1/4.3/§4.4 (the banner is already posted) and
      recomputing the reach / offset-angle table in CLIMBING-DESIGN §6 — best done together
      when stance-phase IK is started in P4.
- [x] **h. Closed (2026-08-07 photos)**: the hub's four holes + the outer recess = the locking
      holes and the seat for the servo flange; the servo horn is locked to the femur rod end,
      on the same side as the door.

---

> ⚠️ **What follows is the original dispute, archived from the morning of 2026-08-07** (its
> premise has been overturned by the ruling above; kept for traceability only, do not quote
> the numbers in it).

## Definition of l3

**The "in-leg-plane" distance from K to the center of the suction cup lip** (free state, cup
hanging unloaded).

- K = the center screw of the tibia servo horn = the knee joint axis;
- "Leg plane" = the vertical swing plane of the leg; the lip center has a **lateral** offset
  Δy from K (perpendicular to the leg plane), so any method that "measures the slant distance
  directly" mixes Δy in, and l3 ≠ the 3D distance measured straight off with calipers (the
  difference is Δy²/(2·l3)).
- config currently holds 120, from the July measurement on L1, which already left a 4.6mm open
  question (`LEG-GEOMETRY-OPEN.md` §4.4: another measurement path gives 116).

## The three candidate values

| Candidate | Source | Weakness |
|---|---|---|
| **116** | Free cup height h_cup measured at 21mm, plugged into the design geometry formula (below) | Depends on identifying "the crease line = the model's crease_z" |
| **120** | July caliper measurement of KP on L1 + design model (h_cup back-computed as 25.6) | The slant distance is hard to measure, and it contradicts the measured h_cup=21 |
| **123.7** | 2026-08-06 sticker-Pythagoras method (below), measured on a newly assembled leg | The "two-screw reference line" it relies on may have projected screws on opposite sides (see Dispute 2) |

**Design geometry formula** (the printed-part portion is fixed; only the rubber cup height
h_cup is a live variable):

l3 = √(46.5² + (85.4 + h_cup)²)

- h_cup=21 → 116.1; h_cup=25.6 → 120.35; if 123.7 holds it requires h_cup=29.2.
- 46.5 = the horizontal offset between K and the cup axis **along the leg**; 85.4 = the
  vertical drop from K to the plane of the crease seat ring. Source: the
  `tools/generate_climbing_parts.py` code, where the HORN_HOLES four-hole centroid =
  (47.8,−14.6), `shaft_cx=1.3` (x of the square-shaft axis), and the crease seat ring z=−100.
  47.8−1.3=46.5; 100−14.6=85.4.

## Dispute 2: is the lateral offset Δy 30 or ~5?

- **Measured on paper (08-06)**: the perpendicular distance from the projected lip point to
  the line "projected femur screw — projected K screw" = **30mm**; and in the top-view photo
  the cup foot block looks centered across the femur's 60mm width (half width = 30, so it is
  self-consistent).
- **Generator source code**: `shaft_cy = -4.5` — the cup axis is only **4.5mm** from the
  tibia's flat back face (the plane the servo horn sits against), i.e. by design the cup is
  almost against the servo-horn side and Δy should be only ~5mm. Corroborating evidence: the
  README says this part "would go through the print bed if laid flat, so it can only be
  printed upright", exactly because the cavity radius 17 > 4.5; if the axis were 30mm from the
  back face, lying flat would cause no trouble.
- Two explanations: (1) the paper measurement projected the wrong screws — the **opposite side
  plate** of the knee carries pivot/idler screws, and if the femur and tibia points were not
  projected from the same side, the reference line is skewed sideways by tens of millimeters
  and both Δy and l3=123.7 are contaminated; (2) the "centered" look in the photo is a
  perspective illusion (the foot is at the lowest point, and top-view perspective pushes it
  toward the center of the frame).
- **Note**: this Δy (K ↔ lip) is not the same as the "lip ↔ coxa axis plane" offset that the
  coxa projection method cares about; that one has its own 5-minute check (see pending item e).

## Pending measurements (30 seconds–5 minutes each; tick them off as they are done)

- [ ] **a. Lateral check**: measure with a ruler the lateral distance from the lip edge to the
  tibia's flat back face. Design expectation: the lip (Ø30) straddles the back-face plane, with
  the two edges about 10.5 / 19.5mm away. If so, Δy≈5 and the 30 on paper was a bad
  measurement; also, seen head-on the cup should hug the side plate that carries the servo
  horn, not hang in the middle of the leg width.
- [ ] **b. Screw inventory**: what screws are on each of the two knee side plates? Confirm
  whether the two used for the 08-06 projection were on the same side (the servo-horn screw
  side).
- [ ] **c. h_cup**: with the cup hanging free, measure the height from the crease line to the
  lip plane. ≈21 → the l3=116 camp; ≈29 → the 123.7 camp. (July record: free 21 / sealed 7.5 /
  vacuum 8.)
- [ ] **d. Redo the sticker-Pythagoras measurement** (after a passes): press the whole lip ring
  lightly onto paper and trace the circle to get its center C_lip; drop a plumb projection of K
  to get P_K and measure K's height above the paper h_K; then **l3 = √(d² + h_K²)**, with
  d = the on-paper distance C_lip → P_K (when Δy≈5 no decomposition correction is needed).
  Cross-check against the value computed in c; it only counts if they mesh within 1–2mm.
- [ ] **e. coxa lateral-offset check** (an independent item, do it while you are there): hold
  the coxa still, project the lip at two extension amounts and draw the line through the two
  points, then measure the perpendicular distance from the projected hip (coxa servo-horn
  screw) to that line. ≈0 → calibrate the coxa with the hip + single-point method; clearly
  non-zero → calib_fit.py needs a two-point command.
- [ ] **f. Repeat c/d on a second leg**; only change config if the two legs agree (one leg
  measured once does not change a model parameter).

## What to do once the case is closed

1. Change `config.py: tibia_len` to the settled value; if the calib_fit REPL is open it must be
   restarted to take effect.
2. Batch tibia calibration uses the height-difference method (`recz`, to be added) or
   trilateration (`recd`); both consume l3.
3. The FK spot check (<5mm) is the final backstop acceptance.

## Related files

- `software/scripts/calib_fit.py` — calibration tool (±45 pulse width + attach fitted together;
  progress for all 18 channels stored in `docs/data/calib_pm45.json`)
- `docs/en/LEG-GEOMETRY-OPEN.md` §2.13 (design coordinates of K / the square shaft), §4.4
  (original record of the 4.6mm open question)
- `tools/generate_climbing_parts.py` — the source of the parameters 46.5/85.4/4.5
- Photos: `images/leg_top_view.jpg` (red line = femur width 60), `images/leg_side_view.jpg`
