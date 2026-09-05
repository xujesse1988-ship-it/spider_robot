> English translation of [`docs/P4-BAY-DESIGN.md`](../P4-BAY-DESIGN.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# P4 electrical + pneumatic bay mounting parts · design

Goal: pack every electrical/pneumatic part of the P4 adhesion system into **one "bay" that comes off as a unit**,
bolted to the top face of the MakeYourPet body (frame), a few M3 screws and it is off the machine. The dimensions
measured so far have already produced a **V0 trial-fit STL set**; the generator is a standalone `tools/generate_p4_bay_v0.py`,
writing into `hardware/climbing-parts/p4-bay-v0/`. V0 has to pass the 1mm outline template and a trial fit of the
real parts first; it must not go climbing without a real fit check.

Source material:
- The photo of the parts, `dianqi.jpg` (the primary source for the inventory of mounting features)
- Connection topology, `html/en/p4-system-diagram.html` (three electrical rails + the vacuum bus, which set the relative positions)
- `docs/en/P3-GUIDE.md` step 2, the "P4 pneumatic bay mounting parts" row (constraints already fixed: soft TPU pump
  mount, valve array beam, tank cradle, 7 sensor positions + sensing tees, PCB standoffs, cable combs, whole bay
  removable, heavy parts laid flat against the body plane, a service loop per leg)
- The official `component-layout.jpg` + STL measurements: frame 198×205×16mm,
  top-cover4 footprint 100×140mm, plus battery-bar (battery) and
  servo2040-bottom-cover (Servo2040 tray) on the belly side

---

## 0. Three architecture decisions settled up front

1. **The battery and the Servo2040 do not go into the bay; they stay in their official belly positions.**
   The official parts battery-bar (battery strapped to the belly) + servo2040-bottom-cover (Servo2040 tray)
   were already on the P3 print list. The belly faces the wall while climbing — the battery is the single heaviest
   item (~350g), so putting it on the belly puts it at the closest point to the wall on the whole machine, which is
   optimal for the CG; the 18 servo leads also naturally converge on the belly.
   Moving them up into the bay has nothing but downsides. All the bay needs is: a drop-through channel for the one
   USB cable between Pi and Servo2040, and a channel up for the battery's XT60 harness.
2. **"Servo2040 close to the Pi" done in 3D.** The Pi sits at the front of the bay, directly above the frame's central
   opening, and the Servo2040 sits on the belly tray directly below that opening — the two are vertically adjacent, and a
   15–20cm USB-C cable goes straight through the opening. That satisfies "one short USB link" without disturbing the
   18 servo leads.
3. **The bay is two levels: pneumatics on the floor, PCBs mounted on both faces of the deck.** The originally planned
   110×95mm mezzanine has an area of only 10450mm², while the Pi + divider board + 5V board alone already come to
   12234mm² — physically they do not fit. V0 changes it to a 184×108×3mm electrical deck on six posts:
   Pi / divider board / 5V on the top face, MOSFET / relay / 2×XL6009 on the bottom face; the pump, valves, manifold and
   sensors still sit on the baseplate. The post length becomes 90mm, leaving clearance for the pump, a future
   Ø≤70 tank, and the parts on the underside of the deck.

---

## 1. Parts list and inventory of mounting features

Mounting strategy types: **A** direct M3/M2.5 screw + standoff (board has holes) / **B** printed snap-in tray (board with no holes) /
**C** TPU pad + hold-down strip (pump) / **D** cradle clamp (cylinders) / **E** cable-tie slot (harnesses, small light parts).

| # | Item | Mounting features seen in the photo | Mounting strategy | Vibration isolation |
|---|---|---|---|---|
| 1 | Raspberry Pi 5 (4 copper heatsinks stuck on) | Standard 4×M2.5 holes (58×49mm spacing); the USB-C power port is on the same side as the micro-HDMIs, USB-A/Ethernet on the opposite side | A: M2.5 screws through the board + 6mm standoffs; top face of the electrical deck | Not needed; leave ≥15mm of air above the heatsinks, no top cover |
| 2 | 2S LiPo (silver carbon-pattern hard case, XT60 + balance lead) | No mounting holes or brackets at all, bare case | **Stays out of the bay**: official battery-bar + hook-and-loop/cable ties, belly side; removable for charging | Not needed |
| 3 | 555 vacuum pump ×2 (black pump head + silver motor) | **Comes with a metal L-bracket**, measured 48×28mm four holes, hole Ø5 | C: separate adapter plate + 4×M4 + large washers, a 3mm TPU sheet between bracket and plate, "held but not clamped dead"; V0 fits a single pump for now | **Required** (diaphragm pump vibration: a rigid mount loosens screws and puts spikes in the sensor readings, already settled in P3-GUIDE) |
| 4 | YYNMOS-8 8-channel MOSFET board | Measured 93×54mm, the four holes are at asymmetric coordinates; blue terminal blocks along both long edges | A: M3 screws through the board + 6mm standoffs; bottom face of the electrical deck, **output terminals toward the valve rail, input end toward the XL6009s** | Not needed |
| 5 | 5V buck module (USB-C output, "QC PASS" sticker) | Measured 46×24mm, four holes at 41×19mm, Ø3; USB-C centered on the short edge | A: M3 + 6mm standoffs, on the top face of the electrical deck; **the USB-C outlet aimed at the side with the Pi's power port** | Not needed |
| 6 | XL6009 boost ×2 (red board, trimpot + blue terminals) | Measured 50×28mm, four holes at 43×21mm, Ø3; runs hot | A: M3 + 6mm standoffs, on the bottom face of the electrical deck; leave hand room on the trimpot side | Not needed |
| 7 | Servo2040 | 4×M2 holes, USB-C at one end, the 18 three-pin headers along both sides | **Stays out of the bay**: official servo2040-bottom-cover, belly side | Not needed |
| 8 | 0520B solenoid valve ×6 (1 in the photo, solenoid + white barbed spouts) | No bracket, no holes — it is just a solenoid with spouts on it | B variant: **printed valve rail** (6 slots as a negative-profile seat) + hold-down strip / cable ties | Not needed (the valves are one of the excitation sources but they are light, so a rigid mount is fine) |
| 9 | Divider board (large perfboard + 2×ADS1115, half built) | Measured 91×70mm after cutting, hole spacing 83×58, Ø3 (build drawings in html/en/p4-divider-board.html) | A: M3 screws through the board + 6mm standoffs; deck top face, **right next to where the 7 sensor lines come up + IDC ribbon to the Pi's 40-pin header** | Not needed |
| 10 | XGZP6847A pressure sensor ×7 | Measured 19×19×12mm (pins excluded), Ø3 pressure port facing up, pins facing down, no mounting holes | B variant: **printed 7-slot sensor block**, pins through a window in the floor, tacked with a little neutral-cure silicone once snapped in | Light: keep the whole block away from the pump position and let the TPU pump pad cut the source |
| 11 | Vacuum manifold (1-in-6-out) | Measured trunk Ø20×115, outlet pitch 17, no mounting ears | D variant: C-shaped cradles with 20.6 ID ×2 + M3 | Not needed |
| 12 | Vacuum tank (automotive vacuum reservoir, not delivered yet) | Estimated Φ50–70 × 100–150; the BOM says it **comes with barbed spouts and mounting ears** | D: saddle cradles ×2 + TPU liner + cable ties/hook-and-loop; if it has mounting ears, bolting through them comes first | Not needed (before it goes on the machine, crush-test it alone at −70kPa) |
| 13 | Servo power switch (30A relay module) | Measured 72×40mm, hole spacing 66.5×34.5, Ø3 | A: M3 screws through the board + 6mm standoffs; bottom face of the electrical deck, **switching the positive high side**, close to the power inlet | Not needed |
| 14 | XT60 harness + main switch (KCD1 rocker) | KCD1 measured cutout 19×13; still no panel-slot dimensions for the XT60 | E + panel bracket: 1.8mm snap wall + twin cable-tie slots for the XT60 (a "grab and pull anytime" position) | Not needed |
| 15 | Check valves ×2, tees ×7, silicone tubing | All plumbing parts, no mounting features | E: anchor to the nearest slot in the baseplate's cable-tie slot array | Not needed |

---

## 2. Layout (the core)

### 2.1 Zone overview (looking down on the frame top face, head up)

```
                                    FRONT (head)
        FL leg ◤                                                     ◥ FR leg
   ┌────────────────────────────────────────────────────────────────────────────┐
   │ ══ ELECTRICAL DECK (184×108, six posts 90 tall, both faces) ══════════════ │
   │  ┌─────────────┐  ┌────────────────────┐                                   │
   │  │ 5V buck     │→ │  Pi 5              │     ┌─────────────────────┐       │
   │  │USB-C aligned│  │ (40pin to the rear)│   ←→│ divider board       │       │
   │  └─────────────┘  └──────┬─────────────┘  IDC│ + 2×ADS1115         │       │
   │                          │USB drops down     └──────────┬──────────┘       │
   │        (frame central opening: belly side below         │ 7 sensor lines   │
   │         = Servo2040 official tray)                      ↓ to lower layer   │
   ├──── ELECTRICAL DECK, BOTTOM FACE (power electronics) ──────────────────────┤
   │  [relay slot 70×30]  [XT60 + main switch ↗ right front edge, vertical face]│
   │  [XL6009 ×2 side by side]→[MOSFET board (outputs to rear)]                 │
   ├──── LOWER PLATE (middle: valve zone, aligned with ML/MR legs) ─────────────┤
 ML│ ◄3 tubes out          [sensor block, 4+3 double row]           3 tubes out►│MR
   │        [ VALVE RAIL: 0520B ×6 in a row ]                                   │
   │        [ MANIFOLD 1-in-6-out, tight behind the valve rail ]                │
   ├──── LOWER PLATE (rear half: vacuum generation chain) ──────────────────────┤
   │  [V1 tank: cradle and position TBD once it arrives and is measured]        │
   │  [pump1 ▓] L-bracket + TPU pad; second pump keeps the same adapter         │
   │      └ short tube from check valve → manifold / future tank                │
   └────────────────────────────────────────────────────────────────────────────┘
        RL leg ◣                                                     ◢ RR leg
                                     REAR (tail)
   Belly side (faces the wall when climbing): battery (battery-bar + hook-and-loop) + Servo2040 (official tray)
```

The V0 baseplate outline is **190×130mm** (the central area reuses top-cover4's 100×140 interface boundary,
and the wings extending left and right press onto the solid parts of the frame top face, keeping clear of the 6 coxa
servo zones — clearance still to be measured, §4-14). The nominal center coordinates of the V0 lower layer are:
valve rail `(0,-48)`, pump adapter plate `(-52,22)`, sensor block
`(45,22)`, the two manifold cradles `(-55,-18)/(55,-18)`; the manifold axis is at
`(0,-18,z=37)`. Origin = center of the baseplate, +Y toward the head; the complete hole positions are in
`hardware/climbing-parts/p4-bay-v0/layout-report.json`.

### 2.2 Layout principles, checked one by one

| Principle | How it is met |
|---|---|
| Heavy parts laid flat against the body plane | The battery (heaviest) is on the belly = closest to the wall; in V0 the single pump, the valves and the manifold sit on the baseplate; the deck carries only PCBs. Once the tank arrives it still goes on the baseplate by preference, but a V1 cradle must be made from measured dimensions first |
| XL6009 12V output right next to the MOSFET power end | Side by side in the front zone, terminal to terminal, power wires <50mm |
| MOSFET outputs face the valve array | The board's output side faces rearward, straight at the valve rail in the middle; the 6 valve wires are <80mm; the 2 pump wires run along the right edge through a cable comb to the tail |
| Divider board right next to where the 7 sensor lines come in | The divider board is at the rear edge of the mezzanine, **directly above the sensor block**, so the 7 lines run vertically up <60mm; the run to the Pi is a 2×20 IDC ribbon (settled in P3, no Dupont jumpers) |
| Servo2040 close to the Pi (one USB) | Vertically adjacent (§0-2), the USB-C cable passes through the frame's central opening |
| The 5V buck's USB-C outlet faces the Pi's port | The buck module is on the same side as the Pi's power port, outlet aimed at it, with a 15cm short cable |
| One continuous pneumatic run | A straight line from tail to front: pumps ×2 → check valves ×2 → tank → tee at the tank port (the tank pressure sensor = slot 7 of the block) → manifold → 6 valves; the whole chain is continuous across the rear half and the middle, with no long cross-zone tubing |
| The 6 valve outlets fan out to the six legs | The valve rail spans the middle, right at the ML/MR leg roots; 3 tubes go left and 3 right, running along the cable-tie slots at the edge of the baseplate to each leg root, **with a service loop where each leg crosses the coxa** (a slack loop); the cable-tie slot at the bay edge is the final anchor |
| Pump vibration isolation | Its own L-bracket + a 3mm TPU sheet + TPU washers, half-floating fastening (§3-3) |
| Battery removable | Hook-and-loop / cable ties on the belly, pull it out to charge without touching the bay |
| Modular | The whole bay is one baseplate, 4–6 M3 screws to get it off the machine; the electrical deck sits on 6 M3 posts and comes off on its own; the pump/valve/sensor/manifold mounts all bolt to hole arrays and can be swapped individually |

### 2.3 Heights and interfaces

- V0 baseplate 3mm, with the perimeter and opening ribs adding another 5mm, 8mm total height; electrical deck 3mm.
- The six deck posts are 90mm long, so the underside of the deck sits about 93mm above the baseplate plane. Estimating
  from the pump's mounting face + 3mm TPU + the 28mm bracket axis height + the 19mm motor radius, the top of the pump
  is at about 58–60mm; that leaves about 33mm, and after subtracting 6mm PCB standoffs, about 2mm of board
  thickness and a conservative 15mm of component height there is still about 10mm
  of static margin. The printed posts are for trial fitting only; for real climbing, prefer bought M3×90 metal/nylon hex
  posts or a two-piece stacked post.
  The gap between the pump and the PCBs on the underside of the deck still has to be checked with everything installed —
  terminals, wire bends and vibration swing — not just the static envelope.
- The interface to the frame reuses top-cover4's four holes `(±44,±40)`; V0 additionally reserves `(±44,0)`
  as two reinforcement holes. Those two may only be drilled into the frame as Ø2.8mm M3 self-tapping pilot holes after the
  1mm template confirms them; without that confirmation, do not drill.
- The central opening in the baseplate is 70×108mm, R14, smaller than the frame's minimum through-hole of 76×116mm, about R18;
  it is used to drop USB / sensor lines / servo leads through. The dynamic clearance from the 190×130mm outline to the coxa harness
  still has to be checked with `p4-bay-fit-template-v0.stl` on a frame with the legs and cable clips fitted.
- The pump exhaust points out toward the tail, and the valves' side Ø6 ports stick out of the open side of the valve rail;
  the 12V power wires and the analog sensor wires use different cable combs.
- Wiring discipline (carried over from P3): 12V power wires and 0.25–2.25V analog sensor wires **run through separate combs**,
  different combs, never bundled together; no breadboards or Dupont jumpers anywhere in the bay, JST-XH or soldered wire.

### 2.4 Weight budget — the cold water first

2.5kg total budget − the P3 no-adhesion reference line of 1.9kg = **the adhesion system's implied budget is only 0.6kg**. Rough estimate:

| Item | Est. weight g |
|---|---|
| Pumps ×2 | ~260 |
| Tank | 60–100 |
| Valves ×6 | 120–200 |
| Manifold + tubing + fittings + check valves | ~120 |
| Sensors ×7 | ~40 |
| MOSFET + XL6009 ×2 + divider board | ~140 |
| Bay printed parts + posts and screws | 200–250 |
| **Total** | **~940–1110** |

**Over budget by ~350–500g.** Countermeasures (in priority order): ① fit only one pump for the first run, keep the
second pump position and leave the spare pump on the shelf (−130g); ② pick the small 250mL tank; ③ make weight the number
one goal for the printed parts — 3mm plate + cutouts + ribs, target ≤180g; ④ weigh every part as it arrives and log it in
`weight-log.md`; start a dedicated weight-reduction effort once it goes over 2.2kg
(without adhesion) or the whole robot is projected over 2.7kg. **2.5kg most likely cannot be held;
the realistic landing point is 2.6–2.8kg — that eats straight into the adhesion margin, so the suction-cup numbers have to be redone before P4 acceptance.**

The geometric solid volume of V0 at the listed quantities (excluding the 1mm template) is about **237.2cm³**; if it were
all solid PETG/TPU, that is an upper-bound mass of about **301g**. Slicer infill will bring the real weight down, and bought posts/standoffs
will change the result too, but most of a 3mm large flat plate is still close to solid. So V0's job is to confirm hole positions,
assembly and harnesses, not to claim it hits ≤180g; once the trial fit passes, do the V1 cutout/thin-wall weight reduction and weigh it for real.

---

## 3. Design notes for each mounting part

| Part | Form and key points | Approx. size | Material / print orientation | How it attaches to the baseplate |
|---|---|---|---|---|
| bay_baseplate | Independent 20/24mm module hole arrays, 70×108 R14 drop-through opening, perimeter + opening ribs; component bosses are not fused onto the big plate | 190×130×3, 8 total height | **PETG**; flat, no supports | The original 4×M3 + optional 2×M3 reinforcement holes into the frame |
| electrical_deck | Top face Pi / divider board / 5V; bottom face MOSFET / relay / 2×XL6009; 3 wire slots kept | 184×108×3 | PETG; flat | Six posts, post holes `(±85,-48/0/48)` |
| deck_post / pcb_spacer | 90mm hollow posts for trial fitting; the PCBs use 6mm M2.5/M3 standoffs respectively | Ø9×90 / Ø7×6 | PETG; posts printed standing, trial fitting only | For the real build, prefer bought hex posts / stacked posts |
| pump_adapter + TPU pad | 48×28 four holes Ø5.4 in the adapter plate for 4×M4 + large washers; four M3 slots bolt into the left-hand hole array; 3mm TPU pad | 64×54×4 / 58×38×3 | PETG + TPU 95A; flat | V0 has one pump; the second pump reuses the same part |
| valve_rail | 6 slots at 23 pitch; a window under each valve to clear the vertical Ø6 spout, small stops at the four corners for location, the open side lets the side spout's tube out | 154×34×10 | PETG; slots facing up | M3 slots at both ends; one 2.5mm cable tie over each valve |
| sensor_block | 4+3 double row of 7 slots at 21.5 pitch; floor windows clear the pins; two extra Ø11 holes let the right-hand deck posts land directly on the baseplate | 104×64×11 | PLA/PETG; slots facing up | 4×M3 into the right-hand hole array; tacked with neutral-cure silicone |
| manifold_clip | An open C-clip for the Ø20 trunk, 20.6 ID; axis raised to z=37 (assembly coordinates), crossing over from behind and above the valve rail | 30×about 27×45.9 | PETG; bottom face down | 2×M3 per cradle, print 2 |
| tank_cradle | **Not generated in V0**; the tank still has not arrived, and you cannot make a load-bearing part from an estimated diameter | pending §4-11 | — | Generate in V1 once it arrives and is measured |
| switch_bracket | 1.8mm local wall thickness + a 19.4×13.4 KCD1 rectangular cutout; the XT60 has not been measured, so use two adjacent cable-tie slots instead of a fake-precise slot | 50×20×33 | PETG; on its side or with supports | 2×M3 slots in the bottom face |
| cable_comb | 5-slot comb part ×4; separate parts for power and analog signal | 48×14×16 | PLA/PETG | 2×M3 each |

---

## 4. Dimensions still to be measured (grab the calipers, fill in the blanks)

The photos are clickable thumbnails; front/back or multiple angles of the same part go in the same cell.

| # | Photo | Item | What to measure | Measured value (fill in here) |
|---|---|---|---|---|
| 1 | <a href="../../images/pi_5.jpg"><img src="../../images/pi_5.jpg" alt="Pi 5" width="120"></a> | Pi 5 | Recheck the 4-hole spacing (nominal 58×49), hole diameter (nominal Ø2.7), USB-C port center to board corner | _58_×_49_ / Ø2.7__ / __12__ |
| 2 | <a href="../../images/battery.jpg"><img src="../../images/battery.jpg" alt="2S battery" width="120"></a> | 2S battery | Length × width × height, XT60 lead length (it lives in the official belly position, the bay only leaves a harness channel) | _135_×__43__×__16__ / __90__ |
| 3 | <a href="../../images/YYNMOS-8.jpg"><img src="../../images/YYNMOS-8.jpg" alt="YYNMOS-8 front" width="100"></a><br><a href="../../images/YYNMOS-8_back.jpg"><img src="../../images/YYNMOS-8_back.jpg" alt="YYNMOS-8 back" width="100"></a> | YYNMOS-8 | Board length × width, coordinates of the 4 holes, distance from the terminal blocks on both sides to the board edge |  _93_×__54_ / the (X,Y) coordinates of the 4 holes relative to the lower-left corner are: hole 1 (lower left): (3, 30) hole 2 (lower right): (90, 30) hole 3 (upper left): (3, 39) hole 4 (upper right): (90, 41.5) hole diameter: Ø 3.0 / terminal blocks inset from the board edge: left 1mm, right 1mm |
| 4 | <a href="../../images/5V_voltage_reduction.jpg"><img src="../../images/5V_voltage_reduction.jpg" alt="5V buck module" width="120"></a> | 5V buck module | Board length × width × thickness, **whether it has mounting holes** (if so, spacing/diameter), which way the USB-C port faces | __46__×__24__×__17__ / __has mounting holes, spacing 41x19, diameter 3_/ USB-C port on the short edge, dead center |
| 5 | <a href="../../images/XL6009.jpg"><img src="../../images/XL6009.jpg" alt="XL6009" width="120"></a> | XL6009 ×2 | Board length × width, **whether it has holes** (spacing/diameter), whether the two boards are the same model, trimpot position (needs hand room) | _50_×_28__ / __has mounting holes, spacing 43x21, diameter 3_ |
| 6 | <a href="../../images/Voltage_divider_junction_plate.jpg"><img src="../../images/Voltage_divider_junction_plate.jpg" alt="divider board" width="120"></a> | Divider board | Length × width **after cutting**, corner hole spacing × diameter (decide the cut line first, then measure) | __91__×__70__ / __83__×__58__ Ø_3__ |
| 7 | <a href="../../images/pump_1.jpg"><img src="../../images/pump_1.jpg" alt="555 pump side" width="100"></a><br><a href="../../images/pump_2.jpg"><img src="../../images/pump_2.jpg" alt="555 pump top" width="100"></a><br><a href="../../images/pump_3.jpg"><img src="../../images/pump_3.jpg" alt="555 pump motor and bracket" width="100"></a> | 555 pump ×2 | Motor Ø, overall length, pump head width, **L-bracket: hole spacing / hole diameter / plate thickness / hole-to-pump-axis height**, which way the inlet and exhaust spouts face, whether the two units are the same model | Ø38_ overall length: 86, pump head width: _40__ / longitudinal hole spacing (front to back): _28_, transverse hole spacing (left to right): __48_,_ Ø_5, plate thickness _2__ / _hole-to-pump-axis height 28__, exhausts horizontally out the side (not upward)_ |
| 8 | <a href="../../images/valve_1.jpg"><img src="../../images/valve_1.jpg" alt="0520B valve exterior" width="100"></a><br><a href="../../images/valve_2.jpg"><img src="../../images/valve_2.jpg" alt="0520B valve interior and ports" width="100"></a> | 0520B valve | Body length × width × height, coil Ø, **whether there are mounting holes in the base**, which face the inlet/outlet/exhaust ports are on + spout OD | __25__×__20__×__16__ / _17, no mounting holes in the base_, 1 metal spout on the top face (Ø4), 1 vertical plastic spout on the bottom face (Ø6) + 1 side plastic spout (Ø6)__ |
| 9 | <a href="../../images/XGZP6847A_1.jpg"><img src="../../images/XGZP6847A_1.jpg" alt="XGZP6847A side" width="100"></a><br><a href="../../images/XGZP6847A_2.jpg"><img src="../../images/XGZP6847A_2.jpg" alt="XGZP6847A top" width="100"></a> | XGZP6847A ×7 | Overall length × width × height, pressure port OD / direction, wire exit direction, whether all 7 are identical | __19__×_19___×__12 (height excludes the pins)__ / Ø_3__ port pointing straight up_ header pins pointing straight down _ identical |
| 10 | <a href="../../images/divided_manifold.jpg"><img src="../../images/divided_manifold.jpg" alt="vacuum manifold" width="120"></a> | Manifold | Overall length, outlet spacing, inlet/outlet spout OD, whether it has mounting ears (ear hole spacing) | _115 (trunk OD Ø20)___ / _outlet spacing 17___ / inlet OD Ø10, outlet OD Ø6____ / __no mounting ears__ |
| 11 | — (not delivered yet) | Vacuum tank (once it arrives) | Outer Ø, length, spout position/OD, **mounting ear hole spacing**, weight |  not needed for now |
| 12 | <a href="../../images/relay.jpg"><img src="../../images/relay.jpg" alt="30A relay module" width="120"></a> | Relay module| Board length × width, 4-hole spacing × diameter, which way the high-current terminals face | __72__×__40__ / _66.5___×__34.5__ Ø_3___ |
| 13 | <a href="../../images/KCD1.jpg"><img src="../../images/KCD1.jpg" alt="KCD1 rocker switch dimension drawing" width="120"></a> | Main switch (KCD1) | Panel cutout length × width, whether the snap can grip a 3mm plate | rectangular _19.0_×_13.0_ (body 18.7×12.9) / _snap-fit (recommend thinning the printed part around the hole to 1.5–2mm)_ |
| 14 | <a href="../../hardware/makeyourpet-hexapod/Illustrations/body.png"><img src="../../hardware/makeyourpet-hexapod/Illustrations/body.png" alt="frame and top cover assembly sketch" width="120"></a><br>not printed yet, no photo | frame top face | The top-cover4 interface (snap slot positions / screw hole positions), usable clear length × width on the top face (out to the coxa servo cable clips), central opening dimensions | 4 countersunk self-tapping screw holes, no snaps; hole center coordinates relative to the center are (±44, ±40)mm, spacing 88×80mm; top-cover4 outline / direct replacement interface 100×140mm; frame minimum central through-hole 76×116mm with about R18 corners, widening at the top opening to about 81.4×121.4mm because of the corner radii; frame overall outline 198×205.4mm. The dynamic clearance for expanding outward to the coxa harness still needs a whole-robot assembly check. |
| 15 | <a href="../../images/check_valve.jpg"><img src="../../images/check_valve.jpg" alt="check valve" width="100"></a><br><a href="../../images/tee-junction.jpg"><img src="../../images/tee-junction.jpg" alt="tee" width="100"></a><br><a href="../../images/silicone_tube.jpg"><img src="../../images/silicone_tube.jpg" alt="silicone tubing" width="100"></a> | Check valve / tee / silicone tubing | Check valve OD × length, tee OD, recheck tube OD (nominal 4×6) | __check valve OD × length: 20x35_ tee OD 5 _tube OD 4x6 |

**15 items** in total. Right now only two cannot be confirmed from the photos/STLs on hand: the tank dimensions in §4-11 and
the dynamic clearance out to the coxa cable clips in §4-14; the latter has already been turned into the V0 outline-template
fit test.

---

## 5. The V0 generator and deliverables

### 5.1 Fixes in this version

| Original problem | V0 handling |
|---|---|
| The 110×95 mezzanine cannot hold the three top-face PCBs | Changed to a 184×108 double-sided electrical deck on six posts |
| The MOSFET's four holes are not a symmetric rectangle | Generated hole by hole from the four measured coordinates off the lower-left corner, no more guessing spacings with `hx/hy` |
| The pump bracket measures Ø5 with four holes at 48×28, while the old design said 2×M3 | Changed to a separate adapter plate with 4×Ø5.4, using 4×M4 + large washers + a 3mm TPU pad |
| After expanding to 190×130 the dynamic clearance to the coxa harness cannot be obtained from the bare STL | Generate a 1mm fit template separately; do not print the real baseplate and do not drill the new holes until the template passes |
| The tank and XT60 panel interface dimensions are still unknown | V0 does not generate the tank cradle; the XT60 becomes cable-tie slots, avoiding a fake-precise slot |
| With a one-piece baseplate, changing a single hole means reprinting the whole thing | The baseplate becomes an M3 module hole array; pump / valve / sensor / manifold are all separate replaceable parts |

### 5.2 Generation and verification

```bash
venv/bin/python tools/generate_p4_bay_v0.py
```

The script outputs 13 STLs and `layout-report.json`. During generation each part is checked for watertightness, finite vertices and
positive volume; the electrical deck also gets checked for parts going out of bounds and for any two mounting holes being too close together. Currently
13/13 pass. The existing `tools/generate_climbing_parts.py` and the old STLs are not overwritten.

### 5.3 V0 print list

| STL | Qty | Material / purpose |
|---|---:|---|
| p4-bay-fit-template-v0 | 1 | PLA/PETG, **print this first** |
| p4-bay-baseplate-v0 | 1 | PETG, print only after the template is confirmed |
| p4-bay-electrical-deck-v0 | 1 | PETG |
| p4-bay-deck-post-90mm-v0 | 6 | PETG for trial fitting; for the real build prefer bought M3×90 hex posts / stacked posts |
| p4-bay-spacer-m25-6mm-v0 | 4 | Pi 5 |
| p4-bay-spacer-m3-6mm-v0 | about 20 as needed | The other PCBs |
| p4-bay-pump-adapter-v0 | 1 | PETG, V0 single pump |
| p4-bay-pump-pad-tpu-v0 | 1 | TPU 95A |
| p4-bay-valve-rail-v0 | 1 | PETG + 2.5mm cable ties ×6 |
| p4-bay-sensor-block-v0 | 1 | PLA/PETG, 4+3 double row |
| p4-bay-manifold-clip-v0 | 2 | PETG |
| p4-bay-switch-bracket-v0 | 1 | PETG |
| p4-bay-cable-comb-v0 | 4 | Power and analog kept separate |

### 5.4 First-article build order

1. Print `p4-bay-fit-template-v0.stl` and fit it to a frame that already has the coxa servos and cable clips installed;
   swing all six legs slowly through their full travel and confirm the template does not interfere with the harnesses or the servos.
2. Fix the template with the original four `(±44,±40)` holes only. Only after you need more strength and have confirmed the landing point is
   solid should you drill Ø2.8mm pilot holes at `(±44,0)`; this step cannot be substituted with static STL dimensions.
3. Print the pump adapter plate, valve rail, sensor block and manifold clips separately and try each one on the real parts. The target for a
   snap fit is that it goes on by hand with no obvious wobble; do not use screws to force a part that does not fit dimensionally.
4. Print the electrical deck and install all the PCBs without powering anything, checking whether the components, terminals and wire bends on the
   bottom face collide; M2.5 for the Pi, M3 for the other boards, 6mm standoffs everywhere.
5. Only after the first four steps pass, print the real baseplate, assemble the whole bay and weigh it. Record it in `docs/en/weight-log.md`.
   V0 gets a single pump for now; whether to add the second pump is decided jointly by the vacuum build-up time and the whole-robot weight.

---

## 6. V1: revised after the assembly and wiring review

The 13 V0 STLs are each valid as a single mesh, but the whole-bay review found the ribs around the opening clashing with the
pump/valve/sensor footprints, module hole positions that could not form a complete fastening, downward-facing pins and valve
spouts being blocked, and not enough room on the terminal side of the electrical deck for plugging in and bending wires. V1
therefore gets its own directory and generator, and **does not overwrite V0**.

### 6.1 Structural and routing fixes

| Review finding | V1 fix |
|---|---|
| V0 had only one pump position | Two 64 × 54 mm shared adapter plates, left and right, centers `(±46,24)`; the right one rotated 180° |
| The ribs around the opening intruded into the parts | The real baseplate becomes a 4 mm flat plate; only the three carrier beams for the sensor bridge, flush with the baseplate, are kept |
| No clearance below the 7 sensors' pins | Use a raised bridge, laid out 2+2+2+1, bridge plate assembled at a height of 16 mm, each sensor with its own window |
| The valves' bottom and side spouts were obstructed | The six valves mount vertically, bottom spout down and side spout toward `-Y`; the support only touches the two sides of the body |
| The manifold clip collided with the sensor bridge | The cradles move behind the valve rail, manifold axis at about `(0,-49.5,65)` |
| V0's closed slots in the electrical deck could not pass a finished connector | Changed to three notches open to the board edge, with a 10–20 mm service area kept for every screw terminal |
| Power and analog wires crossed easily | The two left-hand combs carry GPIO/sensor only, the two right-hand combs carry battery/pump/valve power only |
| V0's 90 mm mezzanine had too little margin | Posts go up to 100 mm; the deck is assembled at `Z=104…107 mm` |
| The old design added unconfirmed intermediate frame holes | V1 uses only the four confirmed frame holes `(±44,±40)` |

The top face of the electrical deck carries the divider board, the Pi 5 and the 5 V buck board; the bottom face carries the relay,
the YYNMOS-8 and the two XL6009s. The exact coordinates, rotations, port orientations, the 10 wire-bend service areas and
the absolute coordinates of every hole are in `hardware/climbing-parts/p4-bay-v1/layout-report.json`.

### 6.2 Generation and independent validation

```bash
venv/bin/python tools/generate_p4_bay_v1.py
venv/bin/python tools/validate_p4_bay_v1.py
```

The current result is **22 passed, 0 failed**: all 13 STLs are watertight and singly connected; the 21
positioned printed-part instances have no positive-volume collisions; 41 screw paths are clear; the boundary and same-face
spacing of the 7 PCBs and the 10 terminal service areas pass; the downward paths for the 7 sensors' pins and the 6 valves' bottom spouts pass.
The complete print list, assembly directions and wiring zones are in
`hardware/climbing-parts/p4-bay-v1/README.md`.
A rotatable assembly view with filtering and per-part coordinates is at
`html/en/p4-bay-v1-assembly.html`; the real components there are envelope proxies, and the printed STL shapes are not
passed off as exact CAD of the components.

### 6.3 Limits of the validation

V1 still cannot skip the physical first article: you have to print the fit template first and do a full-travel dynamic sweep with
all six legs, then recheck the fore-aft offset of the 555 pump body relative to the bracket hole centers, how far the 0520B
plastic spouts actually stick out, and the printed hole diameter tolerance. The tank cradle stays on hold until the real ear spacing is known.
