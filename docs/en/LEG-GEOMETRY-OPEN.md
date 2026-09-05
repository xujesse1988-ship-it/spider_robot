> English translation of [`docs/LEG-GEOMETRY-OPEN.md`](../LEG-GEOMETRY-OPEN.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Single-leg geometry and calibration · verified conclusions and open questions

> Status: **not converged**. This document records the results of one round of investigation on 2026-07-17, for a later session to pick up.
> Trigger: at P0 step 5 the servo horns were not fitted exactly at the official attach angles -8°/35°/68°; chasing "do we have to take them off and redo them"
> dragged out several problems more serious than the horn angles.
>
> ⚠️ How to read this: the document separates "verified" from "open". **Do not use anything in the open section as a conclusion.**
> The investigation produced a batch of wrong conclusions along the way (see §3); they are void, take care not to pick them up again.

---

## 1. Background

- Hardware: the left front leg L1 is assembled (photos `images/leg_view1.jpg`, `images/leg_view2.jpg`).
- Change: the tibia and the suction-cup foot are **merged into one part** `hardware/climbing-parts/left-tibia-suction.stl`,
  replacing the official `left-tibia.stl` + `tip.stl`.
- Goal: pin down the true `attach_deg` and `tibia_len` for L1 in `software/hexapod/config.py`.

Coordinate and joint-angle conventions are in `software/hexapod/kinematics.py`:
- `gamma` coxa horizontal swing angle; `alpha` femur elevation above horizontal; `theta` **inner knee angle** (angle between femur and tibia, straight = 180°).

---

## 2. Verified conclusions

Each one comes with evidence and a way to reproduce it.

### 2.1 What the official ATTACH_ANGLE means

`hardware/makeyourpet-hexapod/chica-config-2040.txt:87-90`:

```
# The angle between the servo itself and the leg segment when the servo is centered.
COXA_ATTACH_ANGLE -8
FEMUR_ATTACH_ANGLE 35
TIBIA_ATTACH_ANGLE 68
```

### 2.2 -8/35/68 cannot be hit exactly in hardware, and that is not a problem

The DS3235 has a 25-tooth spline → 360/25 = **14.4° per tooth**, worst-case residual **±7.2°**.
Those three official numbers are design intent, not a tolerance requirement.

The common-problems table in `docs/en/P0-GUIDE.md` already spells out the right way to do it:
> "The servo-horn spline has a tooth pitch; fit the nearest tooth and fix the residual later with `attach_deg` in `config.py`"

**Conclusion: the servo horns do not need to come off and be redone.** What is needed is to measure `attach_deg` and put it in the config.

### 2.3 Travel margin is ample (recomputed 2026-07-17 with measured values)

Joint-angle envelope for floor walking (`max_step=60`, `step_height=40`, stance height 90),
computed with the measured config (`tibia_len=120`, tibia attach k=93.6, femur attach α=49.7):

| Joint | Travel needed relative to center | Margin to the ±90° limit |
|---|---|---|
| femur | -33.0° to +9.7° | 57.0° / 80.3° |
| tibia (k) | -6.6° to +48.0° | 83.4° / 42.0° |

The tightest spot still has 42° of margin. **Conclusion stands: the horns stay on.**

### 2.4 ★ Where the tibia's knee axis is (the hardest result of this round)

> ⚠️ **Overturned 2026-08-07**: the four hole spacings measure 9.98/10.01×48.00, which is the servo **flange** rectangle, not
> the Ø49 servo-horn bolt circle (that would need 34.6 between adjacent holes); the four corners of a rectangle are always
> concyclic, so the "concyclic residual 0.002" below is no evidence of horn holes. Knee axis = output shaft **≈(54.0,−7.2)±1**
> (the centroid sits ~9.75 toward the hip along the flange's long axis), and it has been adjudicated twice on the hardware
> (servo-horn center ~10 toward the hip ✓; K→crease drop measured 92 with a tape, this section's convention predicts 85.4 and is out).
> See the adjudication section at the top of `docs/en/L3-DISPUTE-OPEN.md`.

The 4 M1.6 servo-horn screw holes (Ø1.69) on `left-tibia.stl` are concyclic:

```
center (47.77, -14.59)   bolt-circle radius 24.52mm   fit residual 0.002mm
```

Cross-checked with two independent slices at y=-4 and y=-12; the centers differ by 0.07mm.

**→ Knee axis K = (47.8, -14.6) in the tibia frame, not the origin.**

The upper half of what looks like a "7"-shaped hook **is the servo-horn mounting disc** (the Ø49 bolt circle fills it exactly).

Reproduce:
```bash
.venv/bin/python - <<'EOF'
import numpy as np, trimesh, math
m = trimesh.load("hardware/makeyourpet-hexapod/STL/left-tibia.stl")
sec = m.section(plane_origin=[0,-4,0], plane_normal=[0,1,0])
pts=[]
for e in sec.entities:
    v = sec.vertices[e.points][:, [0,2]]          # note: must take x,z straight from the 3D data
    c = v.mean(0); r = np.hypot(*(v-c).T)         # to_2D() gives each slice a random frame, unusable
    if r.std() < 0.3 and 1.0 < r.mean()*2 < 8: pts.append(c)
P = np.array(pts)
A = np.c_[2*P[:,0], 2*P[:,1], np.ones(len(P))]
sol, *_ = np.linalg.lstsq(A, (P**2).sum(1), rcond=None)
print("knee axis =", sol[:2], " R =", math.sqrt(sol[2]+sol[0]**2+sol[1]**2))
EOF
```

### 2.5 Direction of the femur's link line

- The fork opening in `left-femur.stl` is 42mm ≈ the DS3235 body's 40mm → **fork direction = knee-axis direction**.
- The link line runs along the STL's X axis (the two axes at x=0 and x=80, matching `FEMUR_LEN 80`).
- Long straight edges **exactly parallel (0.00°)** to the link line do exist (XY projection: y=+9, length 80.4mm; y=-7, length 82.2mm).

⚠️ The femur axis's **exact coordinates in the STL were never verified by hole fitting the way §2.4 was**; they were only inferred from the x span and FEMUR_LEN=80.
The direction is trustworthy, the **position is doubtful** — it needs the same hole-circle fit.

### 2.6 tibia_len = 134 is the untouched official value

`software/hexapod/config.py:83`:
```python
tibia_len: float = 134.0    # re-measure after switching to the suction-cup foot module (about +45mm)
```
134 == `TIBIA_LEN 134` in `chica-config-2040.txt`; not one character was changed, and the comment is a TODO nobody did.

And the "about +45mm" in that comment is **a leftover from the v1/v2 era** — back then the suction foot was a separate module hanging under the tibia
(git `6cf11d8`, "total height 84→73mm"); from `143e9d3` on, v3 is integrated with left-tibia, so the +45 no longer applies.

### 2.7 The cup cavity is concentric with the square shaft

`tools/generate_climbing_parts.py`: `crease_z=-100.0` (its own comment claims it "sets the leg length") and `shaft_cx=1.3`.
The cavity shell `box0` is an **unrotated** box, half-width `r_out=17`, centered on the square shaft's axis
→ expected x_min = 1.3-17 = **-15.7**, measured STL x_min = **-15.70**. Match.

**Note**: what is concentric is the "cavity" and the "square shaft", and that does **not** mean they are parallel to the link line K→P (see §3.2).

### 2.8 The dimension that sets the leg length was never measured

`images/xipan_marked.jpeg` (three views of the cup): every dimension **above** the crease plane (the thick black line) has a measured value in red
(10 / 1.7 / 3 / 9 / 5 / 13 / 9 mm, because the negative cavity needed them); **below** it, `D_base`, `D_bellows` and
`H_total` are all marked "TBD". And it is exactly the stretch below the black line that sets the leg length.

### 2.9 The tibia basis in robot.py and config.py may differ by 44°

`software/hexapod/robot.py:63-64`:
```python
# the knee variable is the bend angle k = 180 - theta (straight = 0), same basis as TIBIA_ATTACH_ANGLE
arr[leg.tibia.channel] = leg.tibia.joint_deg_to_us(180.0 - math.degrees(th))
```
Together with `attach_deg=68` in `config.py` → **the software takes the inner knee angle at servo center to be theta = 112°** (obtuse, leg nearly straight).

FK counter-check (plugging `alpha=35°` into each of the two bases):

| Reading | Foot tip at center | Plausible? |
|---|---|---|
| 68 = theta | 138.7mm out from the hip, 84.7mm below the hip | normal stance |
| 68 = k=180-theta (current code) | 220.9mm out from the hip, **27.1mm** below the hip | belly on the floor, not a stance |

And the femur side supports "center ≈ standing": standing works out to `alpha=32.2°` vs the official 35°, 2.8° apart.

**Leaning conclusion: 68 is on the theta basis, and the current code is off by a constant 44°.** But this check used `tibia_len=134`
(the value of the official leg, which is fair for judging official intent), so still re-check against a measured theta before changing anything.

If it holds, pick one of two fixes:
- `config.py` tibia `attach_deg`: 68 → 112 (keep robot.py passing k)
- or change robot.py to pass theta itself, keep attach at 68 and take `sign` = -1

### 2.10 Atmospheric pressure squashes the cup flat

-50kPa × Ø30 = **35.3N**, far more than any preload you can push by hand.
→ After it sticks, the bellows keeps compressing: **the higher the vacuum, the shorter the leg**.
→ Strictly, "the distance from knee axis to wall" is a function of pressure, not a geometric constant.

**Closed by measurement 2026-07-18 (§4.4)**: within the working range the bellows is already bottomed out, attached h_cup holds at 7.5–8mm,
and the pressure sensitivity is negligible — a two-state constant model (free 120 / attached ≈108) is enough, no continuous function needed.

### 2.11 L1 three-side measurement and theta_center (2026-07-17, coarse, whole mm)

Preconditions: tibia channel 17 @ 1510µs, enabled throughout; P taken as the **lip center of the cup (free state)**.
Method: measure the pairwise distances between the three points F (center screw of the femur servo horn), K (center screw of the tibia servo horn) and P,
get the angle from the law of cosines; this leans on no physical edge (dodging the "no parallel face" problem of §4.2).

```
FK = 80    matches FEMUR_LEN=80 → the point-picking method self-checks out
KP = 120   the tibia_len candidate under this convention
FP = 140
theta_center = arccos((80²+120²−140²)/(2·80·120)) = arccos(0.0625) ≈ 86.4°
```

- **Precision**: whole-mm readings; 1mm off on each side moves θ by about 1.2°. Before filling in the config, re-measure with calipers to within 0.5mm.
- **Conversion**: keep robot.py passing k → L1 tibia `attach_deg = 180 − 86.4 = 93.6`;
  change robot.py to pass θ itself → `attach_deg = 86.4` (with the sign adjusted). Pick one, never mix.
- **The official-basis question of §2.9 cannot be decided here**: 86.4° is 18.4° from 68 (≈1.3 teeth) and 25.6° from 112 (≈1.8 teeth),
  and the horn was never aligned strictly to the official position in the first place. On a "nearest tooth" argument this weakly favors the θ basis (68+14.4=82.4, 4° off), but that is not a conclusion.
  For **legs that have been measured** the question no longer matters in practice — the measured value simply overrides the official 68.
- **The P convention**: this measurement turns §4.1 from "an STL derivation problem" into "a choice of convention" — once P is the free-state lip center,
  theta and tibia_len are self-consistent on one convention and the kinematics closes. The STL derivation can still be done (§4.1), as a cross-check.
- **Cross-check**: if P lies on the square shaft's axis (x=1.3, §2.7 cavity concentric with the shaft), then KP=120 ⇒ the lip is at STL
  z ≈ −125.6, i.e. 25.6mm below the crease plane (z=−100) and 14.6mm below the bottom of the printed part (z=−110.95).
  This is the first measured reference value for the free-state h_cup of §4.4 (±2–3mm).
- **2026-07-17 the user checked the three numbers, usable as is, applied**: `config.py` L1 tibia `attach_deg=93.6`
  (keeping robot.py passing k) and `tibia_len=120`; `tools/workspace_analysis.py` limits changed to
  θ∈(0,176.4); the §2.3 travel envelope and the CLIMBING-DESIGN §6 tables were recomputed, all conclusions unchanged;
  unit tests 18 passed.

### 2.12 L1 femur measured by height difference (2026-07-17)

Preconditions: the leg root (bottom face of the coxa servo case) laid flat — at P0 there is no body, so laying the case bottom flat on the table
is enough to make the coxa axis vertical; channel 16 @ 1510µs, enabled throughout; the table edge sits between K and the cup
(F and K stay above the tabletop so their heights can be measured, the cup overhangs into free air so the servo does not stall).
Measure table→F and table→K to the top of the screw (same screw type, so the head radius cancels in the difference).

```
Δh = h_K − h_F = 61
α_center = arcsin(61/80) = 49.7°
```

- **49.7 ≈ official 35 + one spline tooth 14.4 = 49.4 (0.3° apart)** — it lands right on the tooth-pitch grid,
  consistent with "assembled one tooth off the official position", so the method is self-consistent.
- Precision: Δh ±0.5mm → ±0.55° in angle (here cosα≈0.65; the larger α, the more sensitive).
- Applied: config L1 femur `attach_deg=49.7`; `ALPHA_LIM=(-40.3,139.7)`;
  the §2.3 envelope and CLIMBING-DESIGN §6 recomputed — with the lift travel going 125→139.7°,
  **the flat-body floor-to-wall transition goes from infeasible to feasible at wall distance D=100** (+25 to +85mm); everything else unchanged.
- Corroborating evidence: since the femur lands exactly on the grid, the tibia's 86.4° back-computed from it (4.0° off the θ-basis grid value 82.4) probably carries 2–4°
  of measurement error; but with ≥42° of walking margin it makes no practical difference, so no re-measurement for now — it gets checked with the P3 FK spot check.

### 2.13 ★ The physical cup axis = a_t − 22.7°, not the K→P direction (2026-07-18)

> ⚠️ **Note 2026-08-07**: the direction of the conclusion still holds (K→P is not parallel to the square shaft), but K has been corrected to
> ≈(54.0,−7.2) (see the banner in §2.4), so δ = atan(52.7/113.8) ≈ **24.9°** (the original 22.7° was based on the old K).
> Downstream numbers such as `workspace_analysis.py CUP_DELTA` await a joint recomputation under to-do g of L3-DISPUTE-OPEN.

K=(47.8,−14.6) is not on the square shaft (x=1.3) → the line K→P makes an angle
**δ = atan(46.5/111.0) = 22.7°** with the shaft / cup axis (lip taken at z=−125.6, the §2.11 convention; under the official P convention it is
the 20.3° already computed back in §4.1 — the angle was known long ago, it just never got wired into the "which way the cup points" model).
In the kinematics a_t = α+θ−180 is the K→P direction, so **the physical cup axis = a_t − 22.7°**.

- **Sign corroboration**: plug in the official floor stance (α=35, θ=68 → a_t=−77°) → cup axis −99.7°,
  only 9.7° off vertical — the cup lies nearly flat on the floor, which matches the design intent of "suction-cup feet that walk on the floor", so the sign is right.
- **Where the error came from**: another downstream victim of §3.1 ("knee axis = STL origin") — back then K ≈ the square shaft,
  K→P ≈ the shaft direction, so using a_t as the cup axis was harmless; after §2.4 corrected K nobody cleaned up this implicit assumption,
  so the tilt table in CLIMBING-DESIGN §6 was systematically off by 22.7°.
- **Downstream already fixed (2026-07-18)**:
  - `workspace_analysis.py` gained `CUP_DELTA=-22.7` and was recomputed;
  - CLIMBING-DESIGN §6: wall stance-phase reach must go 130→**170**
    (in the old pose the cup met the wall at a 16.7° tilt, past the 15° tolerance, so it would not stick); stance height 90 + reach 170 +
    stride ≤40 → tilt ≤11.5°; the landing band for the floor-to-wall transition actually comes out wider and higher;
  - `single_leg_wall.py` / P2-GUIDE: FOOT_X 122→**169** (press position a_t=−67.3°).
- **Not affected**: foot_reach=130 in the P3 floor gait (the floor does not require the cup axis to be perpendicular, and in the stance pose the cup
  already lies nearly flat). The P4 stance-phase IK needs both l3≈108 (§4.4) and the δ of this section.

---

## 3. ⚠️ Wrong conclusions produced this round (void, do not reuse)

They are recorded here so that a later session does not pick them up again.

### 3.1 ❌ "Knee axis = the origin (0,0) of left-tibia.stl"

**Wrong.** The true value is in §2.4: (47.8, -14.6).

Where it came from: at the time it was patched together out of the **corroborating evidence** "official TIBIA_LEN=134, bottom of the printed part
z=-110.95, a 23mm difference, exactly enough for the `micro-switch-tip` + `rod 2x22` + `Tip` sprung-foot assembly", and taken as a conclusion without any hole verification.

### 3.2 ❌ "K→P is parallel to the square shaft (0.56°)", ❌ "the box's side face is parallel to K→P (0.00°)"

**Wrong, and a downstream inference from §3.1.** K is at (47.8,-14.6) and the square shaft at x=1.3, 46.5mm apart in x,
so K→P cannot possibly be parallel to the shaft.

(The user caught this one: "there isn't actually a parallel physical face from K to P, is there" — the user was right.)

### 3.3 ❌ "tibia_len should be changed to 100.0 (P at the crease plane)"

**Wrong, and built on §3.1.** `crease_z=-100` is relative to the STL origin, not to the knee axis.

### 3.4 ✅ But the K marked on the photo is right

> ⚠️ **Note 2026-08-07**: the concept (servo-horn center = output shaft = knee axis) is still right, but its STL coordinates are not §2.4's
> (47.8,−14.6); they are ≈(54.0,−7.2) — the four holes are the flange spacing, and the output shaft sits ~9.75 off the hole centroid.
> See the banner in §2.4 and the adjudication section of `docs/en/L3-DISPUTE-OPEN.md`.

The K marked on `images/leg_view2.jpg` = the center screw of the tibia servo horn — **servo-horn center = servo output shaft = knee axis**,
and that much is right. The contradiction at the time was treating it and the STL's (0,0) as the same point.

---

## 4. Open questions (in priority order)

### 4.1 ★ Where is the tibia's foot tip P? What is the direction of K→P?

Known: K = (47.8, -14.6); official `TIBIA_LEN 134`; square-shaft axis at x=1.3; bottom of the printed part at z=-110.95.

If P is assumed to fall on the square shaft's axis (x=1.3):
```
(47.77-1.3)² + (-14.59-z)² = 134²   →   z_foot = -140.3
```
i.e. P is **another 29.4mm below** the bottom of the printed part. `micro-switch-tip.stl` is 29mm tall (Z: 7..36) — the numbers are close,
but **this is still corroborating evidence, not verification** (§3.1 is exactly what that kind of reasoning cost us).

If the assumption holds, K→P makes about **20.3°** with the square shaft, i.e. the shaft / box **cannot** serve as a datum face.

**This has to be settled first**, because §4.2 and §4.3 both depend on it.

Suggested ways to verify:
- find the mating features between the square shaft at the lower end of `left-tibia.stl` and `micro-switch-tip.stl` / `tip.stl`, and pin down the foot tip from them;
- or just measure on the hardware: the distance from K (the servo-horn center screw) to the foot tip, and work backwards.

### 4.2 Is there any physical face on the tibia parallel to K→P?

§4.1 is undecided → so is this. If there is none, the way theta is measured has to be redesigned (it cannot rely on laying something against a face).

### 4.3 The real tibia_len of the merged part

`tibia_len=134` is definitely wrong (§2.6), but **the right value is unknown** (it depends on §4.1).
And the modeling convention for P has to be decided first:

- **P = a rigid point** (some fixed location on the part) → `l3` is a constant and the cup's compression `h_cup` is modeled separately
- **P = the contact plane** → `l3 = constant + h_cup(vacuum)`, not a constant

The latter disguises a variable as a constant: `l3` differs by about 10mm between walking on the floor (0kPa) and stuck to the wall (-50kPa),
and no single constant can be right for both. Leaning toward the former, but the argument has to be redone once §4.1 is settled.

### 4.4 How h_cup (crease plane → cup lip) varies with vacuum (2/3 measured as of 2026-07-17)

> ⚠️ **Note 2026-08-07**: a newly assembled P3 leg re-measures the free-state h_cup at **19** (the 21 in this table came from the July
> L1 cup — cup batch and assembly differ by 2mm, so measure h_cup leg by leg). With K corrected (see the banner in §2.4), the l3 conversion becomes:
> 19 → **123.7**, which meshes with the 08-06 sticker/Pythagorean two-path result to 0.05mm and is now the settled convention
> (the adjudication section of L3-DISPUTE-OPEN.md). The vacuum / sealed-state data in this section is still valid, but the absolute values need re-measuring on the new cup.

| State | h_cup measured | Used for |
|---|---|---|
| free (no external force) | **21mm** | swing phase: telling when the wall is touched |
| pressed to a seal, not yet pumped | **7.5mm** | preload travel |
| pumped to a steady -40 to -50kPa | **8mm** (2026-07-18) | stance-phase IK (load-bearing case) |

Inferences so far:
- **Three states collapse into a two-state model**: the vacuum state (8) ≈ the hand-pressed sealed state (7.5), a 0.5mm difference within reading error
  (and a hand can push past the 35N of atmospheric force, so the hand-pressed state can be slightly flatter) — the bellows is **already bottomed out** across the working range,
  so stance-phase l3 is insensitive to vacuum. The "l3 is a function of pressure" of §2.10 actually converges to:
  **free l3=120 (swing / floor), attached l3≈108 (sealed, ~-50kPa, stance phase)**, each a constant.
- **Preload travel = 13.5mm**: in the P2 "extend leg → press → pump" sequence, after touching the wall it has to advance another 13.5mm (take 15mm to leave margin).
- **Equivalent l3 for the sealed, unpumped state ≈ 108mm** (120 → 108): the lip moves 13.5mm up along the square shaft, the straight line K→P shortens by 12mm,
  and the K→P direction swings by ~2.8° — changing l3 without changing the angle leaves a ~5mm systematic error at the foot tip.
  The P2 pressure loop is insensitive to it; **the P4 stance-phase IK has to handle it properly** (moving P along the shaft ≠ scaling along K→P, see §4.3).
- ⚠️ **Tension with §2.11**: h_cup measured directly is 21 vs the 25.6 back-computed from KP=120, a 4.6mm gap
  (equivalent to tibia_len 116 vs 120). Candidate causes: whole-mm reading error on KP / the visible crease line ≠ the model's
  crease_z=-100. The config stays at 120 for now, **to be adjudicated by the P3 FK spot check (<5mm)**.

### 4.5 The exact position of the femur axis in the STL

§2.5 only verified the direction, not the position. Verify it with the hole-circle fit of §2.4.

### 4.6 Measuring theta / attach_deg

Depends on §4.1 and §4.2 (they decide what datum face to measure against).

Known measurement constraints:
- Must send **1510µs**, not 1500µs. `attach_deg` is defined at the calibration center
  `(us_m45+us_p45)/2 = (1040+1980)/2 = 1510`, while `scripts/servo_center.py:25` sends 1500, about 1° off.
  Use `scripts/servo_calib_helper.py` and type `17 1510`.
- Keep the servo enabled the whole time; never force it by hand (35kg of stall will strip the gears, and the angle you get afterwards is fake anyway).
- Precision tiers: settling the 44° basis contradiction of §2.9 only needs ±20°; filling in the config needs ±1–2°
  (P3 acceptance is IK <5mm, which on a ~130mm lever arm is about 2.2°).

### 4.7 Knock-on effects of the corrections

The `ALPHA_LIM=(-55,125)` / `THETA_LIM=(22,180)` at `tools/workspace_analysis.py:23-24`
were hard-coded from attach 35/68 (35±90 and so on), and `THETA_LIM` is on the k basis (related to §2.9).
Change `attach_deg` or `tibia_len` and both limits have to follow, and
the "landing band" conclusion in `docs/en/CLIMBING-DESIGN.md` (the basis for the P5 floor-to-wall transition) has to be recomputed.

---

## 5. Summary of conclusions

| Question | Status |
|---|---|
| Do the servo horns have to come off and be redone | **No** (§2.2, §2.3; re-checked against the measured values and it still holds) |
| L1 tibia `attach_deg` | **Measured and written into the config**: θ_center=86.4° → 93.6 on the k basis (§2.11) |
| L1 femur `attach_deg` | **Measured and written into the config**: α_center=49.7°, landing exactly on the official+one-tooth grid (§2.12) |
| `tibia_len` | **Changed to 120** (§2.11 measurement, free-state lip convention; the official 134 is dropped) |
| Where the tibia knee axis is | **(47.8, -14.6)** (§2.4, verified) |
| Where the tibia foot tip is | Convention settled: the free-state lip center, KP=120 (§2.11); the STL cross-check has not been done (§4.1) |
| The tibia basis in robot.py | Semantics fixed as **the k basis** (robot.py keeps passing k); which basis the official 68 uses cannot be decided and no longer matters (§2.11) |
| Which way the cup axis points | **= a_t − 22.7°; stop using a_t as the cup axis** (§2.13); the wall pose and the P2 rig were recomputed accordingly |

**The §4.1 blocker has been routed around by the measured convention of §2.11** (the STL derivation is demoted to a cross-check), the L1 tibia calibration is in place,
and the knock-on updates of §4.7 (workspace_analysis limits, the CLIMBING-DESIGN §6 tables, the §2.3 envelope) are done.

Remaining main line:
1. L1's **coxa attach is still the official default -8; it was decided (2026-07-17) to defer measuring it to P3, when the body goes on**.
   Reason: at P0 there is no body, so γ's zero (the neutral direction 55°) does not physically exist yet; the P1/P2 single-leg rig does not depend on
   the absolute γ (pressure closed loop + the rig can be aimed, and a γ error only shifts the leg plane sideways by ~2.3mm/°); P3 has to do
   a full servo calibration anyway and the body will be there by then, so use the baseline method directly (the line between the projected L1↔R1 servo-horn screws as the datum — it does not rely on
   the "the body pocket is aligned to 55°" assumption, and is harder than the servo-case datum method). Method figure: §05 of the calibration reference figures.
   The femur is done (§2.12: α_center=49.7).
2. Once all three attach values are in, run an end-to-end FK spot check (send joint angles, tape-measure the foot tip vs `leg_fk`, pass at <5mm).
3. After the other five legs are assembled, measure each one by the §2.11/§2.12 method and fill in `attach_deg` leg by leg.
4. ✅ All three h_cup states measured (free 21 / sealed 7.5 / vacuum 8, §4.4): preload travel 13.5mm;
   the two-state l3 model is final (free 120 / attached ≈108; the P4 stance-phase IK uses the latter and must account for the ~2.7° direction swing).
   Note the 4.6mm tension recorded in §4.4 (tibia_len 116 vs 120), to be adjudicated at the P3 FK spot check.
   The risk that unmeasured legs keep running on the official 68 basis is still there (§2.9).
