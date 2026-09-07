#!/usr/bin/env python3
"""Ground-wall bench validation: plan by default, mock demo, or live commands.

Default live mode uses torso support. --self-stand rises from the floor and
 tests only one front foot at a time, leaving the other five on the floor.
 Cold start only: constructors vent cups and disable servo power. Neither mode
 is a full ground-to-wall climbing controller. See docs/GROUND-WALL-TEST.md.
"""
import argparse
import json
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
  start             supported: extend legs; --self-stand: crouch -> stand at target height
  prepare L1/R1     lift chosen front foot, move to 20 mm before wall
  touch L1/R1       advance to nominal wall plane (check actual gap visually)
  press LEG 2       add <=2 mm normal overtravel; total <= --max-press
  attach LEG        after >=2 mm press; require <=-50 kPa continuously for 0.5 s
  release LEG       vent selected foot; independently support/unload it first
  return L1/R1      after release confirmation, retract and return to floor
  pitch 1           absolute nose-up degrees; change <=1 deg each time;
                    requires both wall cups + four floor cups attached
  sit               --self-stand only: after all feet return, lower body to 20 mm
  status / help     show current commanded pose and telemetry / these commands
  stop              freeze motion and stop pump, keep valve outputs; no resume
  quit              stop pump, de-energize valve coils, disable servo power;
                    NO automatic vent/return. Self-stand: sit first; bench: keep torso supported.
Floor attachment: press L2 2 (repeat as needed), attach L2; then L3, R2, R3.
In --self-stand: finish and return L1 before preparing R1 (or reverse);
no floor press/attachment, two-front-foot transfer or pitch. Use sit then quit.
Do not paste multiple commands: commands received while busy are rejected.
"""


def snapshot(b):
    return dict(t_s=round(b.elapsed, 3), busy=b.busy, frozen=b.frozen,
                started=b.started,
                commanded_pitch_deg=round(b.pose.pitch, 4),
                commanded_hip_height_mm=b.geom.hip_height(b.pose),
                self_stand=b.geom.s.self_stand, seated=b.seated, stage=dict(b.stage),
                pressure_kpa=[round(p, 3) for p in b.pressures],
                voltage_v=b.voltage, current_a=b.current, pump=b.io.pump,
                valve_vacuum=list(b.io.valve), attached=sorted(b.attached),
                press_mm=dict(b.depth), virtual_feet_world_mm=dict(b.pose.feet))


def demo_commands(scenario, depth=18, pitch=2, self_stand=False):
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
        if self_stand:
            yield f'release {n}'
            yield f'return {n}'
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
                  assumptions=('Self-standing, five nominal floor supports during each front-foot test; '
                               if settings.self_stand else 'Torso supported; ')
                              + 'no mesh collision/force/physical seal validation',
                  settings=vars(settings), segments=[])
    try:
        for cmd in demo_commands(scenario, settings.max_press, pitch, settings.self_stand):
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
                    help='stand from ground without torso block; one front foot only, no pitch')
    ap.add_argument('--scenario', choices=['front','mixed'], default='front')
    ap.add_argument('--distance', type=float, default=160, help='level front hip to wall, mm')
    ap.add_argument('--height', type=float, default=224, help='wall lip centre above floor, mm')
    ap.add_argument('--body-height', type=float, default=90)
    ap.add_argument('--speed', type=float, default=10)
    ap.add_argument('--max-press', type=float, default=18)
    ap.add_argument('--pitch-limit', type=float, default=5)
    ap.add_argument('--demo-pitch', type=int, default=2)
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--log-dir', default=str(Path(__file__).resolve().parents[1]/'logs'))
    ap.add_argument('--report', help='write offline JSON report to this path')
    args = ap.parse_args(argv)
    settings = Settings(distance=args.distance, height=args.height, body_height=args.body_height,
                        speed=args.speed, max_press=args.max_press, pitch_limit=args.pitch_limit,
                        self_stand=args.self_stand)
    try:
        settings.validate()
        if not 0 <= args.demo_pitch <= settings.pitch_limit:
            raise EntryError('--demo-pitch must be between 0 and --pitch-limit')
    except EntryError as e:
        ap.error(str(e))
    if args.self_stand and args.scenario == 'mixed':
        ap.error('--self-stand cannot use mixed scenario; test front feet one at a time')
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
    # adjustment, not bypassing a guard. Only front sequence is mandatory here.
    preflight = simulate(settings, 'front', 0)
    if not preflight['passed']:
        ap.error('Preflight rejected: '+preflight['reason'])
    print(HELP)
    if args.live and args.self_stand:
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
