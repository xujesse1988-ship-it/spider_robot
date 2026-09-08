#!/usr/bin/env python3
"""Ground-wall bench validation: plan by default, mock demo, or live commands.

Default live mode uses torso support. --self-stand rises from the floor and
 tests only one front foot at a time. Add --dual-front for a static two-wall,
 four-floor test with a catch tether/support. Add --pitch-probe for <=2 deg
 nose-up steps after pressure holds, with fixed world foot targets.
 Add --shift-probe for level body translation in <=5 mm steps, 0..30 mm.
 Cold start only: constructors vent cups and disable servo power. No mode
 provides full ground-to-wall climbing. See docs/GROUND-WALL-TEST.md.
"""
import argparse
import json
import math
import os
from pathlib import Path
import select
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from hexapod.wall_entry import Bench, Settings, EntryError
from hexapod.driver import MockDriver, Servo2040Driver
from hexapod.adhesion import MockVacuumIO
from hexapod.config import LEG_NAMES

HELP = """
Every command needs Enter; wait for SEGMENT complete before the next command.
Motion defaults to twice the original speed (--speed 20; use --speed 10 to restore).
Pressure confirmation, hold and fault timers keep their original durations.
  start             supported: extend legs; --self-stand: crouch -> stand at target height
  prepare L1/R1     lift above floor anchor to wall-target height, then approach
                    to --approach-gap before wall (default 60 mm)
  touch L1/R1       advance from hover to nominal wall plane (default 60 mm);
                    check actual gap visually; no contact sensor
  press LEG 2       add <=2 mm normal overtravel; total <= --max-press
  attach LEG        after >=2 mm press; require <=-50 kPa continuously for 0.5 s
  release LEG       vent selected foot; independently support/unload it first
  return L1/R1      after release confirmation, retract and return to floor
  pitch 1           absolute nose-up degrees; change <=1 deg each time;
                    requires both wall cups + four floor cups attached
                    --pitch-probe: <=0.5 deg per step, 0..2 deg hard limit;
                    two wall cups + four floor feet, hold before each increase
  shift 5           --shift-probe only: absolute body X in mm toward wall;
                    0..30 mm, <=5 mm per step, peak <=5 mm/s; pitch must be 0
                    hold before every increase; keep all six world foot targets fixed
  hold              --dual-front only: two wall cups <=-50 kPa for 10 s continuously;
                    pressure recovery restarts timer, 30 s total timeout; no motion
  sit               --self-stand only: after all feet return, lower body to 20 mm
  status / help     show current commanded pose and telemetry / these commands
  stop              freeze motion and stop pump, keep valve outputs; no resume
  quit              stop pump, de-energize valve coils, disable servo power;
                    NO automatic vent/return. Self-stand: sit first; bench: keep torso supported.
Supported mode floor attachment: press L2 2 (repeat as needed), attach L2; then L3, R2, R3.
In --self-stand without --dual-front: finish and return L1 before preparing R1 (or reverse);
no floor press/attachment, two-front-foot transfer or pitch. Use sit then quit.
With --self-stand --dual-front: attach first front foot before preparing second;
use hold after both attach. Release/return one fully before releasing the other.
Keep four middle/rear feet on floor. Use a catch tether/support.
Dual-front setup from an unloaded floor crouch (enter one line at a time):
  start             wait for SEGMENT complete
  prepare L1        wait for SEGMENT complete; inspect clearance
  touch L1          wait for SEGMENT complete; visually confirm wall contact
  press L1 2        wait for SEGMENT complete; execute once (2 mm total)
  attach L1         wait for ATTACH confirmed L1 before preparing R1
  prepare R1        wait for SEGMENT complete; inspect sag and clearance
  touch R1          wait for SEGMENT complete; visually confirm wall contact
  press R1 2        wait for SEGMENT complete; execute once (2 mm total)
  attach R1         wait for ATTACH confirmed R1
  hold              wait for HOLD confirmed L1 R1
  status            record baseline pressure, power and commanded pose
Without --pitch-probe, dual-front mode has no pitch. With --pitch-probe:
hold -> pitch 0.5 -> hold -> pitch 0 -> hold for the first experiment.
With --shift-probe: after pitch returns to 0, hold -> shift 5 -> hold -> shift 0 -> hold.
Only after observing that round trip, advance 5,10,15,20,25,30 mm with hold at each step.
Return via shift 25,20,15,10,5,0 (one command at a time); no fresh hold required for retreat.
Return both shift and pitch to 0 before releasing either wall foot. No middle-foot transfer.
Do not paste multiple commands: commands received while busy are rejected.
"""


def snapshot(b):
    return dict(t_s=round(b.elapsed, 3), busy=b.busy, frozen=b.frozen,
                started=b.started,
                commanded_pitch_deg=round(b.pose.pitch, 4),
                commanded_hip_height_mm=b.geom.hip_height(b.pose),
                commanded_body_origin_height_mm=b.geom.hip_height(b.pose),
                commanded_body_x_mm=b.pose.body_x,
                self_stand=b.geom.s.self_stand, dual_front=b.geom.s.dual_front,
                pitch_probe=b.geom.s.pitch_probe,
                shift_probe=b.geom.s.shift_probe,
                hold_confirmed_pitch_deg=b.hold_pose.pitch if b.hold_pose is not None else None,
                hold_confirmed_body_x_mm=b.hold_pose.body_x if b.hold_pose is not None else None,
                commanded_front_hip_wall_distance_mm={n: round(b.geom.wall_x-b.pose.body_x
                    - math.cos(math.radians(b.pose.pitch))*b.geom.cfg.leg(n).mount_x, 3)
                    for n in ('L1', 'R1')},
                commanded_hip_heights_mm={leg.name: round(b.geom.hip_height(b.pose)
                    + math.sin(math.radians(b.pose.pitch))*leg.mount_x, 3)
                    for leg in b.geom.cfg.legs},
                approach_gap_mm=b.geom.s.approach_gap,
                commanded_front_wall_gap_mm={n: round(b.geom.wall_x-b.pose.feet[n][0], 3)
                                             for n in ('L1', 'R1')},
                waiting=b.waiting,
                hold_good_s=round(b.good_elapsed, 3) if b.waiting == ('hold', None) else None,
                seated=b.seated, stage=dict(b.stage),
                pressure_kpa=[round(p, 3) for p in b.pressures],
                voltage_v=b.voltage, current_a=b.current, pump=b.io.pump,
                valve_vacuum=list(b.io.valve), attached=sorted(b.attached),
                press_mm=dict(b.depth), virtual_feet_world_mm=dict(b.pose.feet))


def demo_commands(scenario, depth=18, pitch=2, self_stand=False, dual_front=False, pitch_probe=False,
                  shift_probe=False):
    yield 'start'
    for n in ('L1', 'R1') if scenario == 'mixed' or self_stand else ('L1',):
        yield f'prepare {n}'
        yield f'touch {n}'
        remaining = depth
        while remaining > 1e-6:
            step = min(2, remaining)
            yield f'press {n} {step:g}'
            remaining -= step
        yield f'attach {n}'
        if self_stand and not dual_front:
            yield f'release {n}'
            yield f'return {n}'
    if dual_front:
        yield 'hold'
        if pitch_probe:
            angle = 0.0
            while angle < pitch:
                angle = min(pitch, angle+0.5)
                yield f'pitch {angle:g}'
                yield 'hold'
            while angle > 0:
                angle = max(0, angle-0.5)
                yield f'pitch {angle:g}'
                yield 'hold'
        if shift_probe:
            for offset in (*range(5, 31, 5), *range(25, -1, -5)):
                yield f'shift {offset}'
                yield 'hold'
        for n in ('R1', 'L1'):
            yield f'release {n}'
            yield f'return {n}'
        yield 'sit'
        return
    if self_stand:
        yield 'sit'
        return
    if scenario == 'mixed':
        for n in ('L2','L3','R2','R3'):
            remaining = depth
            while remaining > 1e-6:
                step = min(2, remaining)
                yield f'press {n} {step:g}'
                remaining -= step
            yield f'attach {n}'
        for angle in range(1, pitch+1):
            yield f'pitch {angle}'
        for angle in reversed(range(pitch)):
            yield f'pitch {angle}'
    for n in ('L1', 'R1') if scenario == 'mixed' else ('L1',):
        yield f'release {n}'
        yield f'return {n}'


def simulate(settings, scenario, pitch):
    b = Bench(MockDriver(), MockVacuumIO(), settings)
    report = dict(mode='offline ideal-seal simulation', scenario=scenario,
                  assumptions=('Level body shift probe (plus pitch if enabled), two wall seals/four nominal floor contacts; catch tether/support required; '
                               if settings.shift_probe else
                               'Small pitch probe, two wall seals/four nominal floor contacts; catch tether/support required; '
                               if settings.pitch_probe else
                               'Static dual-front test, four nominal floor contacts; catch tether/support required; '
                               if settings.dual_front else
                               'Self-standing, five nominal floor supports during each front-foot test; '
                               if settings.self_stand else 'Torso supported; ')
                              + 'no mesh collision/force/physical seal validation',
                  settings=vars(settings), segments=[])
    try:
        for cmd in demo_commands(scenario, settings.max_press, pitch, settings.self_stand,
                                 settings.dual_front, settings.pitch_probe, settings.shift_probe):
            b.command(cmd)
            ticks = 0
            while b.busy and not b.frozen:
                b.tick()
                ticks += 1
                if ticks > 30000:
                    raise EntryError('Simulation timeout')
            if b.frozen:
                raise EntryError(b.frozen)
            report['segments'].append(dict(command=cmd, state=snapshot(b)))
        report['passed'] = True
    except (ValueError, RuntimeError) as e:
        report.update(passed=False, rejected_command=cmd, reason=str(e))
    finally:
        b.shutdown()
    return report


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument('--plan', action='store_true', help='offline route validation (default)')
    mode.add_argument('--mock', action='store_true', help='interactive mock; add --demo for finite run')
    mode.add_argument('--live', action='store_true', help='real hardware; supported bench, cold start only')
    ap.add_argument('--demo', action='store_true', help='finite ideal-seal mock run; never allowed with --live')
    ap.add_argument('--self-stand', action='store_true',
                    help='stand from ground; defaults to one front foot at a time, no pitch')
    ap.add_argument('--dual-front', action='store_true',
                    help='requires --self-stand; sequential two-wall/four-floor static test; catch tether/support required')
    ap.add_argument('--pitch-probe', action='store_true',
                    help='requires --self-stand --dual-front; <=2 deg nose-up, <=0.5 deg per command, hold before increase')
    ap.add_argument('--shift-probe', action='store_true',
                    help='requires --self-stand --dual-front; level body X 0..30 mm, <=5 mm steps, hold before increase')
    ap.add_argument('--scenario', choices=['front','mixed'], default='front')
    ap.add_argument('--distance', type=float, default=160, help='level front hip to wall, mm')
    ap.add_argument('--height', type=float, default=224, help='wall lip centre above floor, mm')
    ap.add_argument('--approach-gap', type=float, default=60,
                    help='prepared cup-centre gap before wall, mm (20..80, default 60); touch traverses this gap')
    ap.add_argument('--body-height', type=float, default=90)
    ap.add_argument('--speed', type=float, default=20,
                    help='foot/body peak mm/s, also scales pitch rate (1..20, default 20; original speed 10)')
    ap.add_argument('--max-press', type=float,
                    help='total virtual overtravel limit, mm (default: 2 with --dual-front, otherwise 18)')
    ap.add_argument('--pitch-limit', type=float, default=5)
    ap.add_argument('--demo-pitch', type=int,
                    help='offline peak angle (default: 2, or a lower configured whole-degree limit)')
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--log-dir', default=str(Path(__file__).resolve().parents[1]/'logs'))
    ap.add_argument('--report', help='write offline JSON report to this path')
    args = ap.parse_args(argv)
    if args.max_press is None:
        args.max_press = 2 if args.dual_front else 18
    settings = Settings(distance=args.distance, height=args.height, body_height=args.body_height,
                        approach_gap=args.approach_gap,
                        speed=args.speed, max_press=args.max_press, pitch_limit=args.pitch_limit,
                        self_stand=args.self_stand, dual_front=args.dual_front, pitch_probe=args.pitch_probe,
                        shift_probe=args.shift_probe)
    try:
        settings.validate()
        limit = min(2, settings.pitch_limit) if settings.pitch_probe else settings.pitch_limit
        if args.demo_pitch is None:
            args.demo_pitch = min(2, math.floor(limit))
        if not 0 <= args.demo_pitch <= limit:
            raise EntryError('--demo-pitch must be between 0 and effective pitch limit')
    except EntryError as e:
        ap.error(str(e))
    if args.self_stand and args.scenario == 'mixed':
        ap.error('--self-stand cannot use mixed scenario; use --dual-front for static two-wall/four-floor test')
    if args.live and args.demo:
        ap.error('--demo cannot operate real hardware')
    offline = args.plan or not (args.mock or args.live) or args.demo
    if offline:
        report = simulate(settings, args.scenario, args.demo_pitch)
        content = json.dumps(report, ensure_ascii=False, indent=2)
        if args.report:
            Path(args.report).write_text(content+'\n', encoding='utf-8')
            print(f"{'PASS' if report['passed'] else 'REJECTED'}: {args.report}")
        else:
            print(content)
        return 0 if report['passed'] else 1
    if args.report:
        ap.error('--report is for offline runs; interactive runs use --log-dir')
    if not sys.stdin.isatty():
        ap.error('Interactive mode requires a terminal; use --mock --demo for automation')
    # Geometry preflight before opening ANY live IO. A failure requires geometry
    # adjustment, not bypassing a guard. Dual mode includes both feet and return.
    # Live shift/pitch modes always check their whole allowed envelope and return,
    # even if --demo-pitch 0 was supplied for an earlier offline run.
    preflight = simulate(settings, 'front', limit if settings.pitch_probe else 0)
    if not preflight['passed']:
        ap.error('Preflight rejected: '+preflight['reason'])
    print(HELP)
    if args.live and args.shift_probe:
        print('LIVE shift probe: cold start crouched, cups unloaded/off vacuum. '
              'Initialization vents cups and disables servo power; never restart while attached.\n'
              'Use a catch tether/support able to carry the whole robot. '
              'After both wall cups attach: hold -> shift 5 -> hold -> shift 0 -> hold.\n'
              'shift is absolute X in mm (0..30), <=5 mm per step, peak <=5 mm/s. '
              'Hold before each increase; observe floor slip, body sag and cup peeling.\n'
              'Pitch must be 0 throughout translation; keep all six foot targets fixed. '
              'If pitch-probe is enabled, finish its 0 -> 0.5 -> 0 round trip first.\n'
              'Return shift to 0 in <=5 mm steps before release/return; sit before quit. '
              'No middle-foot movement, automatic retreat or fault recovery. '
              'Commanded body position is not a measurement; pressure is not load feedback.')
    elif args.live and args.pitch_probe:
        print('LIVE pitch probe: cold start crouched, cups unloaded/off vacuum. '
              'Catch tether/support must be able to carry the whole robot.\n'
              'After both wall feet attach, hold -> pitch 0.5 -> hold -> pitch 0 -> hold. '
              'First experiment ends there; observe body, ground feet and cup peeling.\n'
              'Hold is required before every increase; <=0.5 deg steps, <=2 deg hard cap. '
              'Four floor targets stay fixed; pressure is seal feedback, not load/pose feedback.\n'
              'Return to commanded pitch 0 before release/return of either wall foot; sit before quit. '
              'stop freezes without automatic recovery; quit cuts servo power.')
    elif args.live and args.dual_front:
        print('LIVE dual-front: cold start from crouch, all cups unloaded/off vacuum. '
              'Use a catch tether or torso support able to carry the whole robot.\n'
              'Four middle/rear floor contacts and wall seal pressure do not prove load capacity. '
              'Observe floor slip, body tilt, cup peeling and whether support carries weight.\n'
              'Attach first front foot before preparing second. Use hold after both attach. '
              'Release/return one foot fully before releasing the other; sit before quit.\n'
              'No pitch or body translation. Initialization vents cups/disables servo power.')
    elif args.live and args.self_stand:
        print('LIVE self-standing: place robot crouched on a flat floor facing the wall. '
              'No torso block needed. Hip height is an open-loop target, not a measurement.\n'
              'All cups unloaded/off vacuum; no other controller running. Use start, '
              'measure actual hip height, then test ONE front foot. Use sit before quit.\n'
              'Cold initialization vents cups/disables servos; Ctrl-C/EOF cuts power. '
              'Use a slack safety tether for first tests.')
    elif args.live:
        print('LIVE cold start: ALL cups must be unloaded and off vacuum now. '
              'Torso support must carry the robot throughout; constructors vent cups.\n'
              'Place hip plane at configured height; allow legs to reach parked pose.\n'
              'Type start only after checking clearance. Ctrl-C/EOF powers servos off.')
    from hexapod.runlog import RunLog
    from hexapod.powerlog import PowerWatch, startup_marker, servo_power_on
    log = RunLog(args.log_dir, tag='wall_entry')
    log.note(json.dumps(dict(settings=vars(settings), mode='live' if args.live else 'mock')))
    pwr = PowerWatch(log) if args.live else None
    drv = io = bench = None
    def event(message):
        log.mark(message)
        print('\n'+message, flush=True)
    try:
        if pwr:
            pwr.start()
        event('OPEN driver (power disabled)')
        drv = Servo2040Driver(args.port) if args.live else MockDriver()
        event('OPEN vacuum IO (all feet initially vented)')
        if args.live:
            from hexapod.adhesion import Pi5VacuumIO
            io = Pi5VacuumIO(6, on_step=startup_marker(log, pwr))
        else:
            io = MockVacuumIO()
        bench = Bench(drv, io, settings, event,
                      power_on=(lambda: servo_power_on(drv, log, pwr)) if args.live else None)
        print('Log: '+log.path+'\nReady. Enter start, or help.\n', flush=True)
        buf = b''
        last = time.monotonic()
        status_due = 0.0
        while True:
            now = time.monotonic()
            dt = now-last
            last = now
            # Commands never catch up by jumping several trajectory frames.
            if dt > 0.5 and bench.started:
                bench.freeze(f'Control loop delayed {dt:.2f} s')
            bench.tick(max(0.001, dt))
            if bench.elapsed >= status_due:
                status_due = bench.elapsed+1
                log.note('TLM '+json.dumps(snapshot(bench), ensure_ascii=False))
            ready, _, _ = select.select([sys.stdin], [], [], max(0, bench.DT-(time.monotonic()-now)))
            if ready:
                data = os.read(sys.stdin.fileno(), 1024)
                if not data:
                    break
                buf += data
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode('utf-8', errors='replace').strip()
                    if text == 'quit':
                        return 0
                    if text == 'help':
                        print(HELP)
                    elif text == 'status':
                        print(json.dumps(snapshot(bench), ensure_ascii=False, indent=2))
                    else:
                        try:
                            bench.command(text)
                        except ValueError as e:
                            event('REJECTED '+text+': '+str(e))
                        if text == 'start':
                            # Initial split relay/arm delay occurs before any cups
                            # attach, while torso is independently supported.
                            last = time.monotonic()
    except KeyboardInterrupt:
        event('Interrupted: stop pump and disable servo power; no automatic vent/return')
        return 130
    except Exception as e:
        event('ERROR '+repr(e))
        return 1
    finally:
        if bench:
            bench.shutdown()
        else:
            # Cover failures after driver construction, including vacuum init failure.
            for obj in (drv, io):
                if obj:
                    try:
                        obj.close()
                    except Exception as e:
                        event('Cleanup error '+repr(e))
        if pwr:
            pwr.stop()
        log.close('Supported wall-entry experiment ended; servo power disabled')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
