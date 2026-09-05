> English translation of [`docs/P1-GUIDE.md`](../P1-GUIDE.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# P1 Adhesion System Bench Validation · Detailed Operating Guide

Goal (weeks 2–5): **get the whole vacuum chain "pump → tank → valve → cup + sensor → Pi" working on the bench,
and collect the first batch of measured data on whether climbing is feasible.**
By the end of this stage you should be able to shoot a video: a single suction-cup foot on vertical glass holding a 3kg weight without budging;
SSH into the Pi, type one command, and it completes an "attach–confirm–release" cycle (<2s).

Suggested order: step 1 doesn't depend on the pneumatic parts and can be done as soon as the servos are on; steps 3–7 strictly in order (each one is the prerequisite for the next);
steps 8–9 are the main event that produces the data.

---

## Step 1 · Single-leg ±45° pulse-width calibration (finish it while waiting for the pneumatic parts)

P2's `single_leg_wall.py` does single-leg IK, and calibrating leg L1 (channels 15/16/17) is the prerequisite for it.
The procedure is in "calibration procedure" in `software/README.md`; here are the extra bench details:

1. Confirm the **servo supply voltage** once more: it must not exceed the limit on the servo's label (the 6.8V version must be given 6.0–6.5V
   through a buck converter, never 2S direct).
2. Find the actual pulse widths for -45°/+45° joint by joint; pick one of two methods:
   - **Method A (no disassembly, recommended)**: stick a phone level on the leg segment. Center first and note the reference angle,
     then nudge the pulse width until the segment has turned ±45° relative to that reference. Use the level for the pitching joints (femur/tibia);
     for the coxa, which swings horizontally, use a paper protractor printed out and laid underneath.
   - **Method B (more accurate, more work)**: take the printed leg segments off, fit the `calibration-arm` printed in P0 onto the horn,
     and use it with `calibration-ruler` as shown in the official video to find the ±45° marks.
   Nudge the pulse width from an interactive Python session:
   ```bash
   cd ~/spider/software && source .venv/bin/activate && python
   >>> from hexapod.driver import Servo2040Driver
   >>> d = Servo2040Driver(); d.set_all_pulses_us([1500]*18); d.enable(True)
   >>> d.set_pulses_us(16, [1520])   # femur=16, close in slowly in ±10µs steps
   ```
3. Put the measured values into `us_m45/us_p45` for L1 in `config.py`; if the direction is reversed set `sign=-1`,
   and trim the zero-position residual with `attach_deg`. After the edits `pytest tests/` must still be all green.
4. Re-check: command femur to 0° and to +45°; a measured error of <2° on the level passes.

## Step 2 · Incoming inspection of the pneumatic parts (the day they arrive)

| Part | What to check |
|---|---|
| 3/2-way solenoid valve | 12V, labeled "normally closed" — the real behavior gets measured in step 3, don't just trust the listing |
| 30mm bellows suction cup | the fitting is an **M5 through-hole** type (air passes through the center of the stud) + a 90° barbed elbow; miss either and `suction_foot` cannot be assembled |
| XGZP6847A | the **analog-output version** (3 wires: 5V/GND/Vout), not the I2C one; range 0 to -100kPa |
| 555 dual-head pump | 12V, rated ultimate vacuum ≤-75kPa; two suction heads, which can be paralleled with a tee to pump down faster |
| Check valve | the body has a flow-direction arrow |
| 8-channel MOSFET board | trigger level compatible with 3.3V (for the opto version read the description; "high-level trigger 3-24V" is fine) |
| XL6009 | the adjustable version (with a potentiometer) |

## Step 3 · Measure the solenoid valve's behavior (half an hour; it decides the plumbing, the software polarity, and one design assumption)

The design doc assumes "an NC valve unpowered = the cup keeps its vacuum" (the passive-safety design in `CLIMBING-DESIGN.md` §2).
But plenty of the 0520B-class "normally closed" 3-way valves on the market actually **vent the outlet port to atmosphere when unpowered** — if that is what yours does, hanging
through a power loss doesn't hold up. This step is about pinning down exactly what the valve you bought does:

1. Measure the coil resistance with a multimeter (a few tens of Ω is normal); a "click" when 12V goes on means the coil works.
2. Test connectivity by **blowing through it**: label the three ports P (to the tank / vacuum side), A (to the cup side), R (exhaust).
   Blow into each port with the coil unpowered and then powered, and record which two ports are connected:

   | State | Measured connection |
   |---|---|
   | Unpowered | ___ ↔ ___ |
   | Powered | ___ ↔ ___ |

3. Compare against the conclusions:
   - **Ideal** (unpowered A↔P, or A fully sealed): holding vacuum without power works, so plumb it this way.
   - **Common** (unpowered A↔R, vented to atmosphere): the P1 bench work is unaffected and goes ahead as usual (just keep the valve powered during the data tests),
     but record the measurement in `CLIMBING-DESIGN.md` §4 — it has to be solved before P4
     (switch to a valve type / plumbing that holds when unpowered, or rewrite the safety argument for hanging through a power loss).
4. Either way, write down whether **connecting the cup to vacuum** corresponds to the coil being powered or unpowered — that decides the `VALVE_ON_LEVEL`
   constant in step 7.

## Step 4 · 12V supply and the MOSFET drive chain (half a day)

1. **Set the XL6009 voltage**: feed the input from the 2S battery (check polarity with a multimeter first), turn the potentiometer to
   12.0V output **with no load**, and only then connect the load. The P1 load (1 valve + 1 pump) is about 1A, so a 3A module is plenty.
2. **Wire the MOSFET board**: on the power side VCC = 12V, and its GND **shares a common ground** with the Pi's GND; on the control side
   IN1 ← Pi GPIO5 (valve), IN7 ← GPIO20 (pump), matching
   `VALVE_PINS[0]=5` and `PUMP_PIN=20` in `adhesion.py` (wire it differently and you change the constants to match).
3. Leave the valve and pump disconnected at first, and pulse the GPIOs while watching the board's indicator LEDs / output terminal voltage:
   ```bash
   sudo apt install -y python3-lgpio gpiod
   pip install lgpio smbus2        # i.e. the [pi] extra in pyproject
   python - <<'EOF'
   import lgpio, time
   h = lgpio.gpiochip_open(0)      # if it errors, try 4; gpioinfo | grep pinctrl-rp1 confirms the number
   for pin in (5, 20):
       lgpio.gpio_claim_output(h, pin, 0)
       lgpio.gpio_write(h, pin, 1); time.sleep(1); lgpio.gpio_write(h, pin, 0)
   lgpio.gpiochip_close(h)
   EOF
   ```
4. Once that passes, connect the valve and pump and pulse again: the valve clicks, the pump spins.

## Step 5 · Pressure sensing chain (half a day)

The XGZP6847A outputs 0.5–4.5V, above the Pi's 3.3V world, so it **must be divided down** before the ADS1115:

| Connection | Notes |
|---|---|
| XGZP 5V / GND | to Pi pins 2 (5V) / 6 (GND) |
| XGZP Vout → 10k → A0, A0 → 10k → GND | 1:1 divider, 4.5V→2.25V |
| ADS1115 VDD/GND | to Pi 3.3V / GND (**VDD goes to 3.3V, not 5V**) |
| ADS1115 SDA/SCL | Pi pins 3 / 5 (I2C1) |
| ADS1115 ADDR → GND | address 0x48 |

1. `i2cdetect -y 1` should show `48`.
2. Read the voltage:
   ```python
   from smbus2 import SMBus
   import time
   def read_a0_v(addr=0x48):
       with SMBus(1) as bus:
           bus.write_i2c_block_data(addr, 0x01, [0xC3, 0x83])  # A0 single-ended, ±4.096V, single-shot
           time.sleep(0.01)
           hi, lo = bus.read_i2c_block_data(addr, 0x00, 2)
           raw = (hi << 8) | lo
           return (raw - 65536 if raw > 32767 else raw) * 4.096 / 32768
   print(read_a0_v() * 2)   # ×2 = the sensor voltage before the divider
   ```
3. **Two-point calibration**:
   - At atmosphere, record the sensor voltage `V_ATM` (on the 0 to -100kPa version it should be close to 4.5V);
   - pull down to the pump's limit by hand (or with the pump once step 6 is built); the voltage should move monotonically toward the other end.
     Use the two points (V_ATM, 0kPa) and (limit voltage, near the pump's rated -75kPa) to confirm the sign of the slope;
     the nominal full-scale slope is 100kPa/4V = 25: `kPa = 25 × (V − V_ATM)` (for the batches whose voltage falls as it pumps down),
     and if the direction is reversed take the slope negative. The vacuum level must come out as a **negative** number.
   - Write `V_ATM` and the slope down; they get filled in at step 7.

## Step 6 · Assemble the vacuum circuit + manual verification (no software, half a day)

The P1 single-foot circuit (the single-foot subset of the pneumatic diagram in `CLIMBING-DESIGN.md` §4):

```
pump(suction port) ← check valve ← tank(PET bottle) ← 3-way valve[P] ; 3-way valve[A] → tee ┬→ suction cup
                                                      3-way valve[R] = atmosphere          └→ XGZP6847A
```

1. **Vacuum tank**: a carbonated-drink PET bottle (it takes negative pressure; a plain water bottle collapses). Drill two holes in the cap for
   4mm tubing, and double-seal with hot glue + epoxy.
2. **Check valve direction**: the arrow (flow direction) points at the pump — i.e. it allows "tank → pump" pumping and blocks air flowing back in.
   Once fitted, stop the pump; if the tank pressure doesn't recover, it is in the right way round.
3. **Sensor position**: P1 has only one XGZP, so tee it in **between valve and cup** (the pressure in the cup branch
   is what acceptance is about). If you want to watch the tank pressure at the same time, buy another one (¥20; table 2 of the BOM lists 3–6 of them anyway).
4. All fittings: pushed fully home + locked with a cable tie; a wrap of PTFE tape on the barbed fittings doesn't hurt.
5. **Manual verification** (skip the software, isolate the problem): feed the pump 12V directly, press the cup onto wiped-clean glass,
   and watch the voltage reading from step 5 — it should get below -60kPa. If it doesn't, brush soapy water on the fittings to find the leak,
   or clamp the tubing section by section and bisect.

## Step 7 · Implement Pi5VacuumIO and bring it up (1–2 days)

Replace the `Pi5VacuumIO` placeholder class in `software/hexapod/adhesion.py` with a real implementation
(fill the constants in from the measurements in steps 3 and 5):

```python
class Pi5VacuumIO:
    """Raspberry Pi 5 hardware IO. P1 bench: 1 valve, 1 pump, 1 sensor; P4 expands to 6 valves, 2 pumps."""
    VALVE_PINS = [5]
    PUMP_PIN = 20
    VALVE_ON_LEVEL = 1     # GPIO level where set_valve(True)=cup connected to vacuum, per the step 3 measurement
    ADS_ADDR = 0x48
    V_DIV = 2.0            # 1:1 resistor divider
    V_ATM = 4.50           # atmosphere-point voltage measured in step 5
    KPA_PER_V = 25.0       # slope measured in step 5 (sign included)
    GPIOCHIP = 0           # change to 4 if it won't open

    def __init__(self, n_feet=1):
        import lgpio
        from smbus2 import SMBus
        self._lg, self.n = lgpio, n_feet
        self._h = lgpio.gpiochip_open(self.GPIOCHIP)
        for p in self.VALVE_PINS[:n_feet]:
            lgpio.gpio_claim_output(self._h, p, 1 - self.VALVE_ON_LEVEL)
        lgpio.gpio_claim_output(self._h, self.PUMP_PIN, 0)
        self._bus = SMBus(1)

    def set_valve(self, i, on):
        self._lg.gpio_write(self._h, self.VALVE_PINS[i],
                            self.VALVE_ON_LEVEL if on else 1 - self.VALVE_ON_LEVEL)

    def set_pump(self, on):
        self._lg.gpio_write(self._h, self.PUMP_PIN, 1 if on else 0)

    def _read_v(self, ch=0):
        import time
        self._bus.write_i2c_block_data(self.ADS_ADDR, 0x01, [0xC3 + (ch << 4), 0x83])
        time.sleep(0.01)
        hi, lo = self._bus.read_i2c_block_data(self.ADS_ADDR, 0x00, 2)
        raw = (hi << 8) | lo
        return (raw - 65536 if raw > 32767 else raw) * 4.096 / 32768

    def read_foot_kpa(self, i):
        return self.KPA_PER_V * (self._read_v(0) * self.V_DIV - self.V_ATM)

    def read_tank_kpa(self):
        # P1's single sensor sits in the cup branch, so tank pressure uses the same reading as an approximation
        # — the pump hysteresis is therefore off; on the bench drive the pump manually or leave it on; P4 restores hysteresis once the tank has its own sensor.
        return self.read_foot_kpa(0)

    def close(self):
        self._bus.close()
        self._lg.gpiochip_close(self._h)
```

After the edit run `pytest tests/` first (the mock path must be unaffected), then run the full state machine cycle on the bench:

```python
import time
from hexapod.adhesion import AdhesionController, FootState, Pi5VacuumIO
io = Pi5VacuumIO(n_feet=1); ctl = AdhesionController(io, n_feet=1)
io.set_pump(True); time.sleep(3)            # pull the tank down first (pump under manual control)
t0 = time.time(); ctl.request_attach(0)     # the cup should already be pressed on the glass by now
while not ctl.is_attached(0):
    ctl.update(0.02); time.sleep(0.02)
    if ctl.state[0] is FootState.FAULT:
        raise SystemExit("SUCKING timed out - did not attach, check the seal")
ta = time.time() - t0
print(f"attach confirmed {ta:.2f}s  pressure {io.read_foot_kpa(0):.1f} kPa")
time.sleep(3)
t1 = time.time(); ctl.request_release(0)
while ctl.state[0] is not FootState.RELEASED:
    ctl.update(0.02); time.sleep(0.02)
tr = time.time() - t1
print(f"release {tr:.2f}s  cycle total {ta+tr:.2f}s (acceptance target <2s)")
io.set_pump(False); io.close()
```

## Step 8 · The three data curves (vertical glass, 1 day)

Wipe the glass/tile clean with alcohol first. Pressure logging script (20Hz to CSV; scp it back to the dev machine afterward to look at):

```python
import time, csv, sys
from hexapod.adhesion import Pi5VacuumIO
io = Pi5VacuumIO(n_feet=1)
with open(sys.argv[1], "w", newline="") as f:
    w = csv.writer(f); w.writerow(["t", "kpa"]); t0 = time.time()
    while time.time() - t0 < 90:
        w.writerow([round(time.time()-t0, 3), round(io.read_foot_kpa(0), 2)])
        time.sleep(0.05)
```

| Curve | Action | Target | Measured |
|---|---|---|---|
| 1 Pump-down time | cup on the wall → open the valve, time it to -40kPa | <1s | ___ |
| 2 Leak rate | after reaching -40kPa stop the pump, put the valve in "hold", watch it come back | no higher than -20kPa within 60s | ___ |
| 3 Release time | switch the valve to vent → the cup can be lifted off with no resistance | <0.5s | ___ |

Troubleshooting a miss: curve 1 slow → parallel the two pump heads / shorten the tubing / pump the tank down first; curve 2 bad → soapy water to find leaks,
re-wipe the cup lip and the glass, re-seal the fittings; curve 3 slow → is the exhaust port blocked, and what is the tube diameter.

## Step 9 · Hanging weight, tilt angle, printed-part retest (1–2 days; the data P2's decision rests on)

**Safety**: nobody stands and no feet go directly under the weight, and put something soft on the floor; add weight in steps with water bottles / calibration weights; goggles recommended
(a cup letting go snaps back).

1. **Normal pull-off**: mount the glass horizontally, attach the cup to its underside, and hang weight downward step by step until it lets go.
   Theoretical reference ~35N (3.5kg) @ -50kPa. Record: ___ kg @ ___ kPa
2. **Shear pull-off**: glass vertical, cup attached, hang weight sideways until it lets go. Theory is about 50% of normal.
   Record: ___ kg @ ___ kPa
3. **Tilt tolerance** (it sets the climbing stride; the key input for `CLIMBING-DESIGN.md` §6):
   shim with 5°/10°/15°/20° wedges (printed or wooden) and do 10 attachments at each angle:

   | Tilt | Successful seals /10 | Pull-off force |
   |---|---|---|
   | 5° / 10° / 15° / 20° | | |

   Feed the conclusion back into §6: tolerance ≥15° → stride can be 40mm; only 10° → tighten to ~25mm.
4. **suction_foot printed-part retest**: run the cup fitting through the printed baseplate and lock the M5 nut (tightened through the side window),
   route the tube out of the side window from the barbed elbow, fit the whole foot onto the tibia's square shaft (locked with a set screw), and repeat curve 2 and the hanging weight.
   Air leaking between print layers → brush thin epoxy on the nut cavity and the inner face of the baseplate / seal the side window; not strong enough → add walls and reprint.
5. **Weigh it**: record the whole foot (printed part + cup + fitting + set screw) in `docs/en/weight-log.md`.

## P1 acceptance checklist

- [ ] L1 leg ±45° calibration entered in `config.py`, `pytest` all green, measured re-check <2°
- [x] Solenoid valve unpowered/powered behavior measured and on record; if "unpowered ≠ vacuum held", it is entered in the CLIMBING-DESIGN to-do list
- [x] `Pi5VacuumIO` implemented (GPIO polarity and the pressure conversion set by two-point calibration)
- [x] Curves 1/2/3 meet target: <1s to -40kPa; no recovery past -20kPa in 60s; release <0.5s
- [x] Normal/shear pull-off force and tilt-tolerance data complete, stride conclusion fed back into §6
- [x] **1.5kg hung on vertical glass (the 30mm cup's limit) for 10 minutes** (done with the complete suction_foot printed foot)
- [x] **"attach–confirm–release" cycle <2s** (run by the state machine, not by hand)
- [x] Whole-foot weight recorded in weight-log
- [ ] P2 materials ordered: L-frame stock, linear rail / drawer slide + carriage, 1.5kg of weight, a vertical glass panel

All ticked → move on to P2 (the single-leg climbing decision gate). The printer is idle during P1/P2, so you can start printing the whole robot's structural parts.

## Common problems

| Symptom | What to do |
|---|---|
| `i2cdetect` doesn't show 0x48 | SDA/SCL swapped; ADDR left floating (0x48 is the default, but tie it to GND to be sure); I2C not enabled (it was enabled in P0 — re-check with `raspi-config`) |
| Pressure reading never changes | Divider resistors not connected / wired to the wrong channel instead of A0; the sensor's 5V supply isn't connected |
| The state machine never reaches ATTACHED | The conversion sign is wrong — vacuum has to be negative; check the sign of `KPA_PER_V` |
| The valve doesn't actuate | Measure the actual 12V (the XL6009 sags under load); MOSFET trigger level and common ground; check whether the coil is open circuit |
| The pump spins but pulls nothing | Wrong port on the dual-head pump (suction vs exhaust); check valve installed backwards; a big leak at one of the fittings |
| The PET bottle collapses | Switch to a carbonated-drink bottle; or lower the tank's vacuum target to -60kPa |
| `lgpio.gpiochip_open` errors out | The Pi 5 chip number is 0 or 4 depending on the kernel version — confirm with `gpioinfo`; add the user to the `gpio` group, or use sudo |
| The pump never stops | With P1's single sensor `read_tank_kpa` is an approximation, so the hysteresis being off is expected; drive the pump manually during the bench phase |
| The cup drops as soon as power is cut | That's the valve type issue from step 3 (unpowered = vented to atmosphere), not a leak; see that step for what to do |
| The cup slips on the glass | Dust or oil on the lip or the glass — wipe both with alcohol; the vacuum isn't reaching -40kPa |

## Safety

- Kill the power before rewiring the 12V circuit; shorting the XL6009 output blows the transistor instantly.
- Valve and pump are inductive loads, so make sure the MOSFET board has flyback diodes (opto boards usually do; bare-transistor boards need you to add them).
- Hanging-weight tests: stay out from under the weight, add weight in steps, tape the glass edges so they don't chip.
- The pump heats up when it runs continuously against a dead head; keep a single test ≤5 minutes and let it rest if it feels hot.
- Same LiPo rules as P0: never charge unattended, keep the XT60 quick disconnect within reach.
