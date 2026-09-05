> English translation of [`docs/BOM.md`](../BOM.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# Bill of materials (BOM)

Prices are the usual Taobao / Pinduoduo range (mid-2026) and are for budgeting only. The terms in parentheses are Taobao search keywords.

## Buying in batches (important)

Risk first: **do not buy anything beyond the first batch until the P2 single-leg wall decision gate has passed.**

**Batch 1 · single-leg validation kit (P0–P2, about ¥600–950)**

- From table 1: servos ×4 (3 in use, 1 spare), Servo2040 ×1, 5V/5A buck ×1, 2S battery ×1, screws / dowel pins / wire (enough for one leg)
- From table 2: vacuum pump ×1, solenoid valves ×2, suction cups ×3, XGZP6847A ×1, ADS1115 ×1, MOSFET board ×1, XL6009 ×1, silicone tubing / fittings / check valves
- Also needed: test-rig material (plywood or aluminum extrusion, a vertical rail carriage or drawer slide, 1.5 kg of ballast)

**Batch 2 · whole robot (after the P2 decision gate, about ¥1100–1900)**

- Rest of table 1: servos ×16 (making 18 + 2 spare), servo-power MOS switch module ×1, foot contact switches, remaining screws and consumables
- Rest of table 2: pump ×1, valves ×6, suction cups ×5, pressure sensors ×6 (making 7 = 1 per foot + 1 on the tank), ADS1115 ×1, vacuum manifold ×1, XL6009 ×1 (making 2), safety line; use up what is left on the silicone tubing reel and top up with 5 m if short
- Batch 2 is the full whole-robot purchase; **P4 adds exactly one item, the line filters** (added 2026-08-17 after the dust risk assessment, see table 2; 40 mm cups and two pumps in parallel are still data-triggered upgrades, not planned items)
- Table 3 as needed

## Table 1 · Walking platform (MakeYourPet; buy 4 servos at P0, the rest before P3)

| Item | Spec | Qty | Unit price ref. | Notes |
|---|---|---|---|---|
| Digital servo | 35 kg·cm coreless/iron-core, 270° limited to 180° (`DS3235 35kg 舵机`, DS3235 35 kg servo) | 18 + 2 spare | ¥45–85 | The original build uses ZOSKAY 35 kg; buy the 180° control-angle version |
| Servo control board | Pimoroni Servo2040 (`Servo2040` or import it) | 1 | ¥150–220 | Runs the community chica firmware, driven from the Pi over USB; cheap alternative: 2× PCA9685 (¥15 each, you have to modify driver.py yourself and manage the servo power rail) |
| Brain | Raspberry Pi 5 4GB + official active cooler | 1 | already owned | Runs the in-house gait software in `software/` (Raspberry Pi OS Lite 64-bit + SSH) |
| Pi supply | 5V/5A DC-DC buck module (`5V 5A 降压模块`, 5 V 5 A buck module) | 1 | ¥15–25 | **Must have its own supply, never share the rail with the servos** — the voltage drop when a servo stalls will reboot the Pi, and a reboot while climbing = a fall. Once installed, confirm with `vcgencmd get_throttled` that there is no undervoltage |
| Battery | 2S 7.4V 5000–6200mAh LiPo (`2S 航模锂电池 XT60`, 2S RC LiPo battery XT60) | 1 | ¥80–150 | Feeds the servos directly at 7.4 V; add a B3 charger, ¥25 |
| Foot contact switch | Micro limit switch (`KW10 微动开关`, KW10 micro switch) | 6 + spares | ¥0.5–1 | Ground contact detection, a hard requirement for the climbing gait |
| Screws | M1.6×6 cap head ×120, M2.5×6 ×10, assorted M3 (`M1.6 内六角 碳钢`, M1.6 hex socket, carbon steel) | 1 bag each | ¥10–20/bag | Check against the fan parts list in the official README for your STL version |
| Dowel pins | Φ3 cylindrical pin (`3mm 定位销`, 3 mm dowel pin) | 6 | ¥5/bag | Leg joint pivots |
| Rubber foot tips | 9mm ID rubber cap (`橡胶保护帽 9mm`, rubber protective cap 9 mm) | 6 | ¥5–10 | For floor walking; swapped for suction-cup feet when climbing |
| Servo power switch | Continuous ≥15A, triggerable at 3.3V: first choice a 30A relay module (`30A 继电器模块 5V 光耦`, 30 A relay module 5 V optocoupler, active-low trigger); or 15A MOS modules (`MOS管触发开关驱动模块 15A 400W`, MOSFET trigger switch driver module 15 A 400 W) ×2 in parallel | 1 | ¥5–20 | **Pi GPIO17 (Pin11) switches the servos' 7.4V main power** (changed 08-15; originally Servo2040 A0/GPIO26); the module bought has selectable H/L triggering, the jumper must go on **H**, coil DC+ from the 5V/5A buck output, DC− on common ground. **Must switch the positive high side** (ground is shared with the Pi through USB, so a low-side switch gets bypassed). Load basis, measured at P2: single-leg press peak 0.73A, whole robot 2–4A continuous / ~12A transient. Three checks on arrival: IN floating = off, 3.3V = on, released = off |
| Power switch / wire | XT60 connector, 16AWG silicone wire, heat-shrink, Dupont jumpers | assorted | ¥30 | |
| Print filament | PLA or PETG 1.75mm | 2kg | ¥50–110 | Leg parts at 4 walls, 40% infill |

**Subtotal about ¥1300–2200** (the servos dominate).

## Table 2 · Vacuum adhesion system (in use from the P1/P2 single-leg validation)

| Item | Spec | Qty | Unit price ref. | Notes |
|---|---|---|---|---|
| Miniature vacuum pump | 555 twin-head diaphragm pump 12V, ultimate -75 to -85kPa (`555 双头真空泵 12V`, 555 twin-head vacuum pump 12 V) | 2 | ¥25–60 | One main, one spare / in parallel for faster pumping |
| Solenoid valve | 3/2-way 12V **normally closed** (`0520B 三通电磁阀 12V 常闭`, 0520B 3-way solenoid valve 12 V normally closed) | 6 + 2 spare | ¥10–20 | Normally closed = holds vacuum when unpowered, the critical selection point |
| Vacuum suction cup | 30mm 2.5-fold bellows cup + **M5 through-hole fitting + barbed elbow** (`机械手真空吸盘 30mm 波纹 M5`, robot vacuum suction cup 30 mm bellows M5) | 8 | ¥5–15 | The bellows structure gives angular compliance for free; upgrade to 40 mm for the payload stage in P5 |
| Pressure sensor | XGZP6847A 0 to -100kPa analog output (`XGZP6847 真空压力传感器`, XGZP6847 vacuum pressure sensor) | 3–6 | ¥15–25 | At least 1 downstream of the pump + poll the feet; ideally 1 per foot |
| Silicone tubing | 4×6mm food-grade silicone tubing | 5m | ¥15 | Plus one bag each of 4 mm tees / unions / check valves |
| Check valve | 4mm pneumatic check valve (`气动单向阀 4mm`, pneumatic check valve 4 mm) | 2 | ¥3–8 | Between the pump and the vacuum tank |
| Line filter | 4mm inline vacuum filter, sintered or felt element (`真空过滤器 4mm`, vacuum filter 4 mm, or the SMC ZFC equivalent) | 6 + 2 spare | ¥2–10 | **Added at P4 (2026-08-17)**: one per foot, fitted between the cup and the foot-pressure tee (cup side). Keeps dust out of the lines — plugging the sensor's pressure tap seals the vacuum in and fakes ATTACHED, a grain under a check valve seat silently kills the leak isolation, and dust in the pump head degrades the ultimate vacuum so the pump runs continuously; none of the three is easy to notice. Re-measure the pump-down time after fitting them (a filter element adds flow resistance) |
| Vacuum manifold | 1-in-6-out distributor (`气动歧管 4mm 一进六出`, pneumatic manifold 4 mm 1-in-6-out) | 1 | ¥10–20 | Distributes tank → 6 valve lines; one part instead of a chain of tees saves 8 push-in joints (a joint = a leak point) |
| Vacuum tank | Empty PET bottle / printed tank + bottle-cap fitting | 1 | ~¥0 | Smooths pressure ripple, speeds up the response |
| Valve driver | 8-channel MOSFET board (`8路 MOS管 驱动模块`, 8-channel MOSFET driver module) | 1 | ¥15–30 | Drives 6 valves + 2 pumps, control pins wired straight to the Pi 5 GPIO |
| Boost module | 2S 7.4V → 12V DC-DC 3A (`XL6009 升压模块`, XL6009 boost module) | 2 | ¥8–15 | The pumps and valves are 12 V parts |
| ADC module | ADS1115 16-bit I2C 4-channel (`ADS1115 模块`, ADS1115 module) | 2 | ¥10–15 | The Pi has no analog input; needed to read the XGZP6847A. Two boards give 8 channels, I2C addresses 0x48/0x49 |
| Safety line | 3mm accessory cord 10m + 2 small carabiners + an expansion anchor | 1 set | ¥40–70 | Used at all times from P4 on |

**Subtotal about ¥350–600**.

## Table 3 · P5 extensions (buy as needed)

| Item | Purpose | Price ref. |
|---|---|---|
| 40mm bellows cups ×6 | Payload upgrade (normal force ×1.8) | ¥8–18 each |
| Raspberry Pi Camera Module 3 (CSI ribbon to the Pi 5) | FPV video stream | ¥120–180 |
| MPU6050 | Fall detection (sudden attitude change → lock all valves) | ¥8 |
| Buzzer + voltage monitor module | Low-voltage alarm (power loss = fall) | ¥10 |
| ReSpeaker Lite single board (XMOS XU316 dual mic, USB) + 4Ω 3W PH2.0 speaker | Voice interaction (**bought 2026-09**, see `docs/en/VOICE-GUIDE.md`; USB, driverless, uses no GPIO, on-board AEC/noise reduction, the speaker plugs straight into the board's SPK socket. A 2-Mics Pi HAT was bought first and returned because GPIO18–21 clashed with the pneumatics) | ¥100–150 + ¥5 |

## Tools (if you do not have them)

3D printer (build volume ≥200×200mm), soldering iron + solder, hot glue gun, multimeter, Phillips/hex screwdriver set, wire strippers, cyanoacrylate + epoxy, bench vise.

---

### Buying notes

1. **Do not buy the cheapest no-name servos** — stalling is normal when climbing, and stripped gears = the whole robot falls. In the 35 kg class buy from a shop with a reputation, e.g. ZOSKAY or DSservo.
2. **The solenoid valves must be the normally closed type** (the cups keep their vacuum when the power is off). Ask before you buy.
3. Pick cups that are a **one-piece part with a right-angle barbed elbow** (bought: 2.5-fold bellows cup, first crease Ø27, groove Ø15×1.7, twin hex-nut flats 7/5, elbow height 13, barb spout protruding 24, hose OD 6; see the three-view in `images/xipan_marked.jpeg`); the cavity in `left-tibia-suction.stl` is designed as a negative of that exact shape, so a different cup means editing `PARAMS` in `tools/generate_climbing_parts.py` and regenerating.
4. The Servo2040 takes a long time to import, so you can do the assembly tuning with a servo tester first and let the control board arrive later.
