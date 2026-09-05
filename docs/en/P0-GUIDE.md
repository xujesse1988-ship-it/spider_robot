> English translation of [`docs/P0-GUIDE.md`](../P0-GUIDE.md). The Chinese original is the maintained source; if they differ, the Chinese version wins.

# P0 Preparation Stage · Detailed Operating Guide

Goal (1–2 weeks): **batch-1 parts arrive, the Pi 5 software environment is all green, and one leg is assembled and can be swung under Pi control.**
By the end of this stage you should be able to shoot a video: SSH into the Pi, type a command, and a spider leg on the desk lifts and sets back down.

The steps can run in parallel: finish step 1 (ordering) the same day, and do step 2 (Pi environment) and step 3 (printing) while you wait for the shipping.

---

## Step 1 · Order the batch-1 parts (day 1)

Follow the specs in tables 1 and 2 of `BOM.md`. This batch's list, and the points that must be confirmed with the seller before ordering:

| # | Part | Qty | Confirm before ordering |
|---|---|---|---|
| 1 | 35kg·cm digital servo (DS3235 / ZOSKAY class) | 4 | ① the **180° control angle** version (not 270°) ② operating voltage range includes 7.4V ③ comes with horns and screws |
| 2 | Pimoroni Servo2040 | 1 | genuine (buying agent / overseas order; longest lead time, so order it first) |
| 3 | 5V/5A DC-DC buck converter | 1 | input range covers 6–8.4V; one with a USB-C port is handier for feeding the Pi directly |
| 4 | 2S LiPo 5000mAh+ with XT60 + B3 charger | 1 each | battery with a protection board, or bring your own low-voltage alarm |
| 5 | 555 dual-head vacuum pump 12V | 1 | ultimate vacuum ≤-75kPa |
| 6 | 3/2-way solenoid valve 12V | 2 | **normally-closed type** (unpowered = air path cut, vacuum held) — get it confirmed in writing |
| 7 | 30mm 2.5-fold bellows vacuum suction cup | 3 | with an **M5 through-hole fitting + 90° barbed elbow** (air passes through the center of the stud) |
| 8 | XGZP6847A pressure sensor 0 to -100kPa | 1 | the analog-output version (not the I2C version) |
| 9 | ADS1115 module | 1 | — |
| 10 | 8-channel MOSFET driver board | 1 | the opto-isolated version is better |
| 11 | XL6009 boost converter module | 1 | — |
| 12 | 4×6mm silicone tubing 5m + 4mm tees / unions / check valves | 1 set | — |
| 13 | M1.6×6 hex socket screws (pack of 100), M2.5×6, M3 | 1 pack each | M1.6 is the workhorse for horn-to-printed-part joints |
| 14 | Dupont jumpers, XT60 pairs, 16AWG silicone wire, heat-shrink | as needed | — |

Budget total about ¥600–950. Batch 2 (the 15 servos and so on) is **not bought at this point**.

## Step 2 · Pi 5 software environment (days 1–3, no hardware needed)

1. **Flash the OS**: the official Raspberry Pi Imager → pick **Raspberry Pi OS Lite (64-bit)** →
   click the gear to preconfigure: hostname `spider`, enable SSH, fill in WiFi, username and password.
2. **First login and basic setup**:
   ```bash
   ssh <username>@spider.local
   sudo apt update && sudo apt install -y git python3-venv i2c-tools
   sudo raspi-config nonint do_i2c 0        # enable I2C (for the ADS1115)
   sudo usermod -aG dialout $USER            # serial port permission (for the Servo2040)
   # log out and back in for the group membership to take effect
   ```
3. **Deploy this repo**: push the repo from your dev machine to your git remote and clone it on the Pi,
   or just `rsync -a --exclude .venv --exclude .git ~/spider/ <username>@spider.local:~/spider/`.
4. **Install and verify the package**:
   ```bash
   cd ~/spider/software
   python3 -m venv .venv && source .venv/bin/activate
   pip install -e ".[sim,dev]"
   pytest tests/                             # expect: 18 passed
   python scripts/sim_walk.py --gif /tmp/walk.gif --seconds 2
   python scripts/stand_up.py --mock         # dry run, Ctrl-C to exit
   ```
   scp `/tmp/walk.gif` back and take a look: a hexapod tripod-gait animation means it passes.
5. **Power check habit** (do it every time from now on): `vcgencmd get_throttled` must print `0x0`;
   nonzero means undervoltage/throttling, so fix the power before going on.

## Step 3 · 3D printing (days 1–4, in parallel with shipping)

Print one **left front leg L1** (left front because that is the test channel in the software's default config: 15/16/17):

| File (`hardware/makeyourpet-hexapod/STL/`) | Qty | Notes |
|---|---|---|
| `left-coxa2.stl` | 1 | when several versions share a name, take the highest number (coxa2 > coxa) |
| `left-femur.stl` | 1 | — |
| `hardware/climbing-parts/left-tibia-suction.stl` | 1 | suction-foot integrated tibia (replaces left-tibia.stl; the cavity at the tip holds the cup directly) |
| `tip.stl` | 1 | plain foot tip (for the P0 swing test; can be fitted onto the old left-tibia) |
| `calibration-arm.stl` + `calibration-ruler.stl` | 1 each | the official servo calibration tools, needed for the ±45° pulse-width calibration in P1 |
| `hardware/climbing-parts/suction-foot-door.stl` | 1 | cup cavity door cover (glued shut with 502 super glue once the cup is in) |

Parameters: coxa/femur in PLA, 4 walls, 40% infill, 0.2mm layer height; for `left-tibia-suction.stl`
use PETG, 5–6 walls, 45% infill, printed **upright** (cup cavity facing down) + a brim ≥8mm, supports are fine for the tibia details;
`suction-foot-door.stl` goes outer face down, no supports. For part orientation and whether to add supports on the leg parts, follow the official
videos (YouTube channel **MakeYourPet**, leg assembly is episode 1); print the small assembly parts (limiter,
servo-back-hole2, etc.) as needed for the single leg in the video.
Dry-fit once before anything else: push the cup with its fitting into the cavity from the front; the rib should snap into the Ø15 groove, the two nuts should drop into the
hex pockets and not turn, and the door should close with no obvious slop. Too tight or too loose → change `gap_d`/`gap_hex`
in `tools/generate_climbing_parts.py` (by ±0.2mm) and reprint. Only glue the door with 502 super glue once it all checks out.

## Step 4 · Flash the Servo2040 firmware (the day it arrives, 5 minutes)

1. Download the community firmware:
   https://github.com/EddieCarrera/chica-servo2040-simpleDriver/releases/download/v0.0.1/chica-servo2040_release.uf2
2. **Hold the BOOT button on the board** while plugging in USB → an `RPI-RP2` drive appears on the computer → drag the .uf2 onto it → it reboots on its own and is done.
3. Verify: plug it into the Pi, `ls /dev/ttyACM*` should show a device; then:
   ```bash
   cd ~/spider/software && source .venv/bin/activate
   python -c "from hexapod.driver import Servo2040Driver; d=Servo2040Driver(); print('voltage', d.read_voltage_v()); d.close()"
   ```
   Servo power is not connected yet, so the reading depends on the "Separate USB & Ext. Power" jumper on the back of the board:
   not cut (factory default) reads about 5V (that is the USB 5V rail); already cut reads close to 0.
   Either way, if it prints a number the USB communication link is working.
   ⚠️ **That jumper must be cut before** connecting the 2S battery in step 5 (marked "CUT THIS!" on the wiring diagram),
   otherwise the battery's 7.4–8.4V back-feeds through the USB cable into the Pi and may destroy the Pi.

## Step 5 · Single-leg assembly (after the servos arrive, half a day)

⚠️ Order matters: **center the servos first, then fit the horns, then the structural parts** — get it backwards and the whole calibration is wrong.

1. **Mandatory checks before connecting the battery** (all three must pass before the battery may go on; do the first one with no power connected at all):
   - [ ] **Cut** the "Separate USB & Ext. Power" jumper on the back of the board (marked "CUT THIS!" on the wiring diagram).
         Connect a 2S battery without cutting it and the battery voltage back-feeds through the USB cable, destroying the Servo2040 and the Pi's USB port.
   - [ ] **Verify the cut**: with USB only and no battery, run the voltage read command from step 4 —
         a reading **close to 0** means the cut worked; still about 5V means it isn't cut all the way through, so go back, finish the cut, and re-test.
   - [ ] **Measure with a multimeter** which battery pigtail / XT60 lead is positive and which is negative before wiring the EXT terminals; **do not go by wire color alone**
         (aftermarket pigtails occasionally have red and black swapped, and reversed polarity kills the board instantly).

   Make it a habit: unplug USB before connecting or disconnecting the battery; the two power paths are never plugged or unplugged at the same time.
2. **Wiring** (against `hardware/makeyourpet-hexapod/wiring-diagram-servo2040.png`):
   the 3 servo signal wires go to Servo2040 channels **15 (coxa) / 16 (femur) / 17 (tibia)**;
   servo power comes from the 2S battery per the official wiring diagram (in P0 the relay can be left out and wired directly, but fit an XT60 quick disconnect).
3. **Center**:
   ```bash
   python scripts/servo_center.py            # center all channels at 1500µs and enable
   ```
   Hearing the servos lock means it worked. **Do not** force the servos by hand at this point.
4. **Fit the horns**: press each horn onto the spline at the center angle from the official video and tighten the retaining screw —
   the center poses of coxa/femur/tibia correspond to the official mounting offsets -8°/35°/68° (the video shows the alignment).
5. **Fit the structural parts**: assemble coxa → femur → tibia in that order (video episode 1), with M1.6 screws joining horns to printed parts.

## Step 6 · Swing test and acceptance (1 hour)

```bash
python scripts/servo_center.py                       # center reference
python - <<'EOF'                                     # sweep each of the three joints ±20°
import time
from hexapod.driver import Servo2040Driver
d = Servo2040Driver()
d.set_all_pulses_us([1500]*18); d.enable(True); time.sleep(1)
for ch in (15, 16, 17):
    for us in (1280, 1720, 1500):
        d.set_pulses_us(ch, [us]); time.sleep(0.8)
d.close()
EOF
```

**P0 acceptance checklist**:

- [ ] All batch-1 parts arrived; NC valves / 180° servos / M5 through-hole fittings checked against the physical items
- [ ] Pi: all 18 `pytest` items pass, `sim_walk.py` produces the GIF, `get_throttled` = 0x0
- [ ] Servo2040 firmware flashed, the Pi can read the voltage
- [ ] Power jumper cut (with USB only, the voltage reads close to 0), battery polarity confirmed with a multimeter
- [ ] All three joints of the single leg swing smoothly on command — no jitter, no odd noises, nothing loose in the structure
- [ ] Cup + fitting pushed fully home into the `left-tibia-suction.stl` cavity, nuts seated in the hex pockets and not turning, door cover dry-fits with no slop
- [ ] Whole-leg weight recorded (write it into `docs/en/weight-log.md`; budget reference: one leg with servos ~230g)

All ticked → move on to P1 (adhesion bench validation).

## Common problems

| Symptom | What to do |
|---|---|
| `/dev/ttyACM0` doesn't exist | Try another cable (many USB-C cables are charge-only); confirm the firmware flashed (rainbow chase on the board LEDs = waiting for a connection) |
| Permission denied opening the serial port | The `dialout` group hasn't taken effect — log out and back in; or temporarily `sudo chmod 666 /dev/ttyACM0` |
| Board/Pi smoking or unresponsive after the battery goes on | Most likely the jumper wasn't cut and battery voltage back-fed, or the battery polarity is reversed. Disconnect the battery and unplug USB immediately; on another computer hold BOOT and see whether the board still enumerates as `RPI-RP2`; test each Pi port in turn with a known-good device |
| Servos don't move but communication is fine | Forgot `enable(True)` (the RELAY enable); or servo power isn't connected / the battery is flat |
| Servo jitter / buzzing | If the 2S voltage is below 6.8V, charge it first; route signal wires away from power wires; jitter under the load of one leg is abnormal, so swap in the spare servo to isolate it |
| Horn angle isn't accurate after centering | The horn spline is toothed, so fit the nearest tooth and correct the residual later with `attach_deg` in `config.py` |
| Pi reboots or throttles often | `get_throttled` nonzero: switch to a 5V/5A supply, don't run the Pi off a computer USB port |
| Cup won't push into the cavity / nuts won't seat in the hex pockets | The printed part's XY tolerance is too tight: add 0.2 to `gap_d`/`gap_hex` and reprint, or file it out; if it's loose and rattles, subtract 0.2 |

## Safety

- Never leave a LiPo charging unattended; use a B3/B6 balance charger. Fit an XT60 quick disconnect on the test bench and cut power at the first odd noise.
- A 35kg servo has huge stall torque: keep fingers out of the joint's range of motion while it is powered.
