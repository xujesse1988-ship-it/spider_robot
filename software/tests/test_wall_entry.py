"""Supported entry experiments: geometry, sequencing and failure behaviour."""
import math
from dataclasses import replace
import pytest

from hexapod.wall_entry import Bench, Geometry, Settings, Pose, EntryError
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES
from hexapod.driver import MockDriver
from hexapod.adhesion import MockVacuumIO, FootState


def run(b, cmd):
    b.command(cmd)
    for _ in range(30000):
        if not b.busy or b.frozen:
            break
        b.tick()
    assert not b.frozen, b.frozen
    assert not b.busy


def bench():
    return Bench(MockDriver(), MockVacuumIO())


def front_attached(b, name='L1'):
    for cmd in (f'prepare {name}', f'touch {name}', f'press {name} 2', f'attach {name}'):
        run(b, cmd)


def test_front_cycle_is_reversible_and_other_feet_do_not_move():
    b = bench()
    run(b, 'start')
    original = b.pose.copy()
    front_attached(b)
    assert b.attached == {'L1'}
    assert b.pressures[0] <= -50
    for n in LEG_NAMES:
        if n != 'L1':
            assert b.pose.feet[n] == original.feet[n]
    with pytest.raises(EntryError):
        b.command('return L1')
    run(b, 'release L1')
    run(b, 'return L1')
    assert b.pose.feet == original.feet
    assert b.depth['L1'] == 0


def test_full_press_mixed_pitch_keeps_world_anchors_and_changes_outputs():
    b = bench()
    run(b, 'start')
    for n in ('L1', 'R1'):
        run(b, f'prepare {n}')
        run(b, f'touch {n}')
    for n in LEG_NAMES:
        for _ in range(9):
            run(b, f'press {n} 2')
        run(b, f'attach {n}')
    feet, pulses = dict(b.pose.feet), list(b.drv.pulses)
    run(b, 'pitch 1')
    assert b.pose.feet == feet
    assert b.drv.pulses != pulses
    run(b, 'pitch 2')
    run(b, 'pitch 1')
    run(b, 'pitch 0')
    assert b.drv.pulses == pytest.approx(pulses)


def test_nonfront_and_premature_pitch_are_rejected_without_motion():
    b = bench()
    run(b, 'start')
    count = len(b.drv.history)
    for cmd in ('prepare L2', 'pitch 1', 'attach L1', 'touch L1', 'return L1'):
        with pytest.raises(EntryError):
            b.command(cmd)
    assert len(b.drv.history) == count


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -1, 300])
def test_geometry_options_rejected_before_hardware(value):
    with pytest.raises(EntryError):
        Geometry(Settings(distance=value))


def test_whole_path_rejection_is_atomic_even_if_first_segment_valid():
    b = bench()
    run(b, 'start')
    start = b.pose.copy()
    count = len(b.drv.history)
    valid = b.target('L1', (start.feet['L1'][0], start.feet['L1'][1], 20))
    impossible = b.target('L1', (1000, 0, 100))
    with pytest.raises(EntryError):
        b.schedule([valid, impossible], dict(b.surfaces, L1=None))
    assert not b.busy
    assert b.pose == start
    assert len(b.drv.history) == count


def test_raw_pulse_guard_catches_target_that_existing_servo_cal_would_clip():
    cfg = DEFAULT_CONFIG
    l1 = cfg.leg('L1')
    narrow = replace(l1.coxa, min_us=1400, max_us=1600)
    cfg = replace(cfg, legs=(replace(l1, coxa=narrow),)+cfg.legs[1:])
    g = Geometry(cfg=cfg)
    p = Pose(dict(g.ground))
    p.feet['L1'] = (g.wall_x, l1.mount_y, 224)
    with pytest.raises(EntryError, match='raw pulse'):
        g.solve(p, dict.fromkeys(LEG_NAMES))


def test_press_limits_and_nonfinite_requests():
    b = bench()
    run(b, 'start')
    for cmd in ('press L2 nan', 'press L2 inf', 'press L2 3', 'press L2 -1'):
        with pytest.raises(EntryError):
            b.command(cmd)
    for _ in range(9):
        run(b, 'press L2 2')
    with pytest.raises(EntryError):
        b.command('press L2 2')


def test_actual_cup_orientation_is_checked_at_contact():
    g = Geometry()
    p = Pose(dict(g.ground))
    p.feet['L1'] = (g.wall_x, 63, 170)
    with pytest.raises(EntryError, match='cup tilt'):
        g.solve(p, dict(L1='wall'))


def test_failed_seal_freezes_without_automatic_retraction():
    b = bench()
    run(b, 'start')
    run(b, 'prepare L1')
    run(b, 'touch L1')
    run(b, 'press L1 2')
    b.io.sealed[0] = False
    pose = b.pose.copy()
    b.command('attach L1')
    for _ in range(250):
        b.tick()
    assert b.frozen
    assert b.pose == pose
    assert not b.io.pump
    assert not b.busy


def test_vacuum_drop_stops_before_next_motor_command():
    b = bench()
    run(b, 'start')
    front_attached(b)
    b.command('prepare R1')
    count = len(b.drv.history)
    b.io.sealed[0] = False
    b.io.foot_kpa[0] = -10
    b.tick()
    assert b.frozen
    assert len(b.drv.history) == count
    assert not b.io.pump
    assert b.io.valve[0]   # previously attached cup not deliberately vented


def test_intermediate_vacuum_pauses_then_times_out_without_motion():
    b = bench()
    run(b, 'start')
    front_attached(b)
    b.command('prepare R1')
    count = len(b.drv.history)
    b.io.step = lambda dt: None
    b.io.foot_kpa[0] = -40
    for _ in range(205):
        b.tick()
    assert b.frozen and 'gate' in b.frozen
    assert len(b.drv.history) == count


@pytest.mark.parametrize('fault', ['nan', 'io'])
def test_bad_telemetry_freezes_without_moving(fault):
    b = bench()
    run(b, 'start')
    b.command('prepare L1')
    count = len(b.drv.history)
    def read(i):
        if fault == 'io':
            raise OSError('I2C offline')
        return math.nan
    b.io.read_foot_kpa = read
    b.tick()
    assert b.frozen and len(b.drv.history) == count


def test_stop_keeps_pending_valve_outputs_unchanged_even_after_attach_timeout():
    b = bench()
    run(b, 'start')
    run(b, 'prepare L1')
    run(b, 'touch L1')
    run(b, 'press L1 2')
    b.command('attach L1')
    for _ in range(8):
        b.tick()
    valves = list(b.io.valve)
    b.command('stop')
    for _ in range(250):
        b.tick()
    assert b.io.valve == valves
    assert not b.io.pump


@pytest.mark.parametrize('fault', ['voltage', 'current'])
def test_power_fault_disables_servos(fault):
    b = bench()
    run(b, 'start')
    setattr(b.drv, fault, 5 if fault == 'voltage' else 9)
    for _ in range(7):
        b.tick()
    assert b.frozen and not b.drv.enabled


@pytest.mark.parametrize('self_stand', [False, True])
@pytest.mark.parametrize('powered_voltage', [7.4, 5.5])
def test_unpowered_bus_is_not_undervoltage_but_powered_bus_is_checked_before_motion(
        self_stand, powered_voltage):
    drv = MockDriver()
    drv.read_voltage_v = lambda: powered_voltage if drv.powered else 0.0
    b = Bench(drv, MockVacuumIO(), Settings(self_stand=self_stand))
    for _ in range(40):
        b.tick()
    assert b.voltage == 0.0
    assert not b.frozen and not b.started and not drv.powered
    assert not drv.history

    b.command('start')
    count = len(drv.history)
    pose = b.pose.copy()
    b.tick()
    if powered_voltage < b.geom.cfg.volt_cutoff:
        assert '5.50 V' in b.frozen and '6.00 V' in b.frozen
        assert not drv.enabled and not drv.powered
        assert b.pose == pose and len(drv.history) == count
        assert not b.busy
    else:
        assert not b.frozen and drv.enabled
        assert len(drv.history) == count+1


def test_shutdown_attempts_every_cleanup_even_if_valve_call_fails():
    b = bench()
    run(b, 'start')
    closed = []
    b.io.close = lambda: closed.append(True)
    def fail(i, on):
        raise OSError('GPIO')
    b.io.set_valve = fail
    b.shutdown()
    assert not b.drv.enabled and not b.io.pump and closed


def test_frequent_keyboard_ticks_do_not_speed_up_trajectory():
    b = bench()
    run(b, 'start')
    b.command('prepare L1')
    count = len(b.drv.history)
    for _ in range(49):
        b.tick(0.001)
    assert len(b.drv.history) == count
    b.tick(0.001)
    assert len(b.drv.history) == count+1


def test_self_stand_matches_stand_up_crouch_and_stand_pulses():
    from hexapod.robot import Hexapod
    b = Bench(MockDriver(), MockVacuumIO(), Settings(self_stand=True))
    bot = Hexapod(MockDriver())
    initial_feet = dict(b.pose.feet)
    pulses, _ = b.geom.solve(b.pose, dict.fromkeys(LEG_NAMES, 'floor'))
    assert pulses == pytest.approx(bot.pulses(bot.crouch_feet()))
    b.command('start')
    heights = []
    while b.busy:
        b.tick()
        assert not b.frozen
        assert b.pose.feet == initial_feet
        heights.append(b.pose.body_z)
    assert heights == sorted(heights)
    assert heights[-1] == 90
    assert b.drv.pulses == pytest.approx(bot.pulses(bot.engine.default_feet))


def test_self_stand_keeps_five_floor_feet_and_requires_return_before_other_front():
    b = Bench(MockDriver(), MockVacuumIO(), Settings(self_stand=True))
    run(b, 'start')
    floor_feet = dict(b.pose.feet)
    run(b, 'prepare L1')
    count = len(b.drv.history)
    for cmd in ('prepare R1', 'press L2 2', 'attach L2', 'pitch 1', 'sit'):
        with pytest.raises(EntryError):
            b.command(cmd)
    assert len(b.drv.history) == count
    for n in LEG_NAMES:
        if n != 'L1':
            assert b.pose.feet[n] == floor_feet[n]
    run(b, 'return L1')
    front_attached(b, 'R1')
    with pytest.raises(EntryError):
        b.command('prepare L1')
    run(b, 'release R1')
    run(b, 'return R1')
    before = dict(b.pose.feet)
    run(b, 'sit')
    assert b.pose.feet == before
    assert b.pose.body_z == 20 and b.seated
    with pytest.raises(EntryError):
        b.command('prepare L1')


def test_self_stand_geometry_rejects_pitch_even_if_called_directly():
    g = Geometry(Settings(self_stand=True))
    with pytest.raises(EntryError, match='no pitch'):
        g.solve(Pose(dict(g.ground), pitch=1, body_z=90), {})


def dual_bench():
    return Bench(MockDriver(), MockVacuumIO(), Settings(self_stand=True, dual_front=True))


def test_dual_front_requires_explicit_self_stand():
    with pytest.raises(EntryError, match='requires self_stand'):
        Geometry(Settings(dual_front=True))


@pytest.mark.parametrize('first', ['L1', 'R1'])
@pytest.mark.parametrize('return_first', ['L1', 'R1'])
def test_dual_front_full_cycle_keeps_four_ground_targets_and_body_fixed(first, return_first):
    b = dual_bench()
    run(b, 'start')
    ground = dict(b.pose.feet)
    original_solve = b.geom.solve
    def checked_solve(pose, surfaces):
        assert pose.pitch == 0 and pose.body_z == 90
        for n in ('L2', 'L3', 'R2', 'R3'):
            assert pose.feet[n] == ground[n]
        return original_solve(pose, surfaces)
    b.geom.solve = checked_solve
    second = 'R1' if first == 'L1' else 'L1'
    front_attached(b, first)
    front_attached(b, second)
    assert b.attached == {'L1', 'R1'}
    history = len(b.drv.history)
    run(b, 'hold')
    assert b.good_elapsed >= b.DUAL_HOLD_S - 1e-9
    assert len(b.drv.history) == history
    return_second = 'R1' if return_first == 'L1' else 'L1'
    run(b, f'release {return_first}')
    valves = list(b.io.valve)
    with pytest.raises(EntryError, match='other front foot'):
        b.command(f'release {return_second}')
    assert b.io.valve == valves and b.attached == {return_second}
    run(b, f'return {return_first}')
    run(b, f'release {return_second}')
    run(b, f'return {return_second}')
    assert b.pose.feet == ground
    b.geom.solve = original_solve
    run(b, 'sit')
    assert b.seated and b.pose.body_z == 20


def test_dual_front_cannot_prepare_second_before_first_is_attached():
    b = dual_bench()
    run(b, 'start')
    for cmd in ('prepare L1', 'touch L1', 'press L1 2'):
        run(b, cmd)
        count = len(b.drv.history)
        with pytest.raises(EntryError, match='other front foot'):
            b.command('prepare R1')
        assert not b.busy and len(b.drv.history) == count
    run(b, 'attach L1')
    run(b, 'prepare R1')


@pytest.mark.parametrize('cmd', ['pitch 1', 'press L2 2', 'attach L3', 'release R2', 'prepare R3', 'sit'])
def test_dual_front_rejects_body_or_ground_foot_commands(cmd):
    b = dual_bench()
    run(b, 'start')
    front_attached(b)
    count, valves = len(b.drv.history), list(b.io.valve)
    with pytest.raises(EntryError):
        b.command(cmd)
    assert len(b.drv.history) == count and b.io.valve == valves


def test_dual_front_hold_requires_two_attached_feet():
    b = dual_bench()
    run(b, 'start')
    front_attached(b)
    with pytest.raises(EntryError, match='two confirmed'):
        b.command('hold')


def test_dual_front_hold_restarts_continuous_timer_on_pressure_dip():
    b = dual_bench()
    run(b, 'start')
    front_attached(b, 'L1')
    front_attached(b, 'R1')
    b.io.step = lambda dt: None
    b.command('hold')
    count = len(b.drv.history)
    for _ in range(120):
        b.tick()
    b.io.foot_kpa[0] = -40
    b.tick()
    assert b.good_elapsed == 0 and b.busy and not b.frozen
    b.io.foot_kpa[0] = -70
    for _ in range(120):
        b.tick()
    assert b.busy
    for _ in range(80):
        b.tick()
    assert not b.busy and not b.frozen
    assert len(b.drv.history) == count


def test_dual_front_unstable_hold_times_out_without_release_or_motion():
    b = dual_bench()
    run(b, 'start')
    front_attached(b, 'L1')
    front_attached(b, 'R1')
    b.command('hold')
    b.io.step = lambda dt: None
    b.io.foot_kpa[3] = -40
    count, valves = len(b.drv.history), list(b.io.valve)
    for _ in range(602):
        b.tick()
    assert b.frozen and 'hold not stable' in b.frozen
    assert not b.io.pump and b.io.valve == valves
    assert len(b.drv.history) == count


def test_first_wall_cup_loss_during_second_prepare_freezes_before_next_frame():
    b = dual_bench()
    run(b, 'start')
    front_attached(b, 'L1')
    b.command('prepare R1')
    b.tick()
    b.io.step = lambda dt: None
    b.io.foot_kpa[0] = -20
    pose, count, valves = b.pose.copy(), len(b.drv.history), list(b.io.valve)
    b.tick()
    assert b.frozen and not b.busy and not b.io.pump
    assert b.pose == pose and len(b.drv.history) == count and b.io.valve == valves


def test_second_cup_failed_seal_keeps_first_attached_without_retraction():
    b = dual_bench()
    run(b, 'start')
    front_attached(b, 'L1')
    for cmd in ('prepare R1', 'touch R1', 'press R1 2'):
        run(b, cmd)
    b.io.sealed[3] = False
    b.command('attach R1')
    count, pose = len(b.drv.history), b.pose.copy()
    for _ in range(220):
        b.tick()
        if b.frozen:
            break
    assert b.frozen and b.attached == {'L1'} and b.io.valve[0]
    assert not b.io.pump and b.pose == pose and len(b.drv.history) == count


def test_dual_front_geometry_still_rejects_pitch():
    g = Geometry(Settings(self_stand=True, dual_front=True))
    with pytest.raises(EntryError, match='no pitch'):
        g.solve(Pose(dict(g.ground), pitch=1, body_z=90), {})


def test_dual_front_cli_defaults_and_complete_offline_sequence(tmp_path):
    import json
    import runpy
    from pathlib import Path
    script = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/ground_wall_probe.py'))
    report = tmp_path / 'dual.json'
    assert script['main'](['--self-stand', '--dual-front', '--mock', '--demo',
                           '--report', str(report)]) == 0
    data = json.loads(report.read_text())
    assert data['passed'] and data['settings']['max_press'] == 2
    assert [s['command'] for s in data['segments']] == [
        'start', 'prepare L1', 'touch L1', 'press L1 2', 'attach L1',
        'prepare R1', 'touch R1', 'press R1 2', 'attach R1', 'hold',
        'release R1', 'return R1', 'release L1', 'return L1', 'sit']
    assert data['segments'][9]['state']['attached'] == ['L1', 'R1']
    assert data['segments'][-1]['state']['seated']
    with pytest.raises(SystemExit) as e:
        script['main'](['--plan', '--dual-front'])
    assert e.value.code == 2


def test_hold_does_not_enable_dual_front_in_default_single_mode():
    b = Bench(MockDriver(), MockVacuumIO(), Settings(self_stand=True))
    run(b, 'start')
    with pytest.raises(EntryError, match='requires --self-stand --dual-front'):
        b.command('hold')


def pitch_bench():
    b = Bench(MockDriver(), MockVacuumIO(),
              Settings(self_stand=True, dual_front=True, pitch_probe=True, max_press=2))
    run(b, 'start')
    front_attached(b, 'L1')
    front_attached(b, 'R1')
    return b


@pytest.mark.parametrize('settings', [Settings(pitch_probe=True),
                                    Settings(self_stand=True, pitch_probe=True)])
def test_pitch_probe_requires_dual_front_opt_in(settings):
    with pytest.raises(EntryError, match='pitch_probe requires'):
        Geometry(settings)


def test_pitch_probe_full_sweep_keeps_world_anchors_and_returns_original_pulses():
    b = pitch_bench()
    feet, initial_pulses = dict(b.pose.feet), list(b.drv.pulses)
    run(b, 'hold')
    for angle in (0.5, 1, 1.5, 2, 1.5, 1, 0.5, 0):
        b.command(f'pitch {angle}')
        start = b.pose.pitch
        samples = []
        while b.busy:
            b.tick()
            assert not b.frozen
            assert b.pose.feet == feet and b.pose.body_z == 90
            assert b.attached == {'L1', 'R1'}
            samples.append(b.pose.pitch)
        assert samples == sorted(samples, reverse=angle < start)
        assert samples[-1] == angle
        # Default doubled speed: 0.5 deg/s peak, including smoothstep ramp.
        assert max(abs(y-x) for x, y in zip([start]+samples, samples)) <= 0.5*b.DT+1e-9
        if angle:
            assert b.drv.pulses != initial_pulses
        run(b, 'hold')
    assert b.drv.pulses == pytest.approx(initial_pulses)
    for n in ('R1', 'L1'):
        run(b, f'release {n}')
        run(b, f'return {n}')
    run(b, 'sit')
    assert b.seated


def test_pitch_probe_requires_fresh_hold_for_every_increase_but_allows_return():
    b = pitch_bench()
    with pytest.raises(EntryError, match='Use hold'):
        b.command('pitch 0.5')
    run(b, 'hold')
    run(b, 'pitch 0.5')
    with pytest.raises(EntryError, match='Use hold'):
        b.command('pitch 1')
    # Geometrically valid retreat never waits for a new 10 s hold.
    run(b, 'pitch 0')
    with pytest.raises(EntryError, match='Use hold'):
        b.command('pitch 0.5')


@pytest.mark.parametrize('angle', ['nan', 'inf', '-0.5', '0', '1', '2.5'])
def test_pitch_probe_rejects_bad_angle_and_oversized_steps_without_output(angle):
    b = pitch_bench()
    run(b, 'hold')
    pose, count = b.pose.copy(), len(b.drv.history)
    with pytest.raises(EntryError, match='Pitch probe:'):
        b.command('pitch '+angle)
    assert b.pose == pose and len(b.drv.history) == count and not b.busy


@pytest.mark.parametrize('cmd', ['release L1', 'release R1', 'return L1', 'prepare R1', 'sit'])
def test_pitch_probe_blocks_release_and_sit_until_level(cmd):
    b = pitch_bench()
    run(b, 'hold')
    run(b, 'pitch 0.5')
    count, valves = len(b.drv.history), list(b.io.valve)
    with pytest.raises(EntryError):
        b.command(cmd)
    assert len(b.drv.history) == count and b.io.valve == valves


def test_pitch_probe_hold_invalidated_by_pressure_dip_even_after_recovery():
    b = pitch_bench()
    run(b, 'hold')
    b.io.step = lambda dt: None
    b.io.foot_kpa[0] = -40
    b.tick()
    assert b.hold_pose is None and not b.frozen
    b.io.foot_kpa[0] = -70
    b.tick()
    with pytest.raises(EntryError, match='Use hold'):
        b.command('pitch 0.5')
    run(b, 'hold')
    run(b, 'pitch 0.5')


def test_pitch_probe_pressure_loss_stops_before_next_motion_frame():
    b = pitch_bench()
    run(b, 'hold')
    b.command('pitch 0.5')
    b.tick()
    b.io.step = lambda dt: None
    b.io.foot_kpa[3] = -40
    pose, count = b.pose.copy(), len(b.drv.history)
    b.tick()
    assert not b.frozen and b.busy and b.pose == pose and len(b.drv.history) == count
    b.io.foot_kpa[3] = -20
    b.tick()
    assert b.frozen and not b.busy and b.pose == pose and len(b.drv.history) == count
    assert b.io.valve[0] and b.io.valve[3] and not b.io.pump


def test_pitch_probe_return_preflight_failure_does_not_change_command_state():
    b = pitch_bench()
    run(b, 'hold')
    pose, count = b.pose.copy(), len(b.drv.history)
    original_path = b.geom.path
    def path(start, goals, surfaces, dt=0.05):
        if start.pitch > 0 and goals[-1].pitch == 0:
            raise EntryError('return route unavailable')
        return original_path(start, goals, surfaces, dt)
    b.geom.path = path
    with pytest.raises(EntryError, match='return route unavailable'):
        b.command('pitch 0.5')
    assert not b.busy and b.pose == pose and len(b.drv.history) == count
    assert b.hold_pose == pose


@pytest.mark.parametrize('fault', ['over_cap', 'floor_target', 'body_height', 'wall_surface'])
def test_pitch_probe_geometry_guards_apply_without_command_layer(fault):
    b = pitch_bench()
    pose, surfaces = b.pose.copy(), dict(b.surfaces)
    pose.pitch = 0.5
    if fault == 'over_cap':
        pose.pitch = 2.01
    elif fault == 'floor_target':
        pose.feet['L2'] = (*pose.feet['L2'][:2], 1)
    elif fault == 'body_height':
        pose.body_z = 89
    else:
        surfaces['L1'] = None
    with pytest.raises(EntryError):
        b.geom.solve(pose, surfaces)


def test_pitch_probe_cli_preflights_whole_envelope_before_live_io(tmp_path, monkeypatch):
    import json
    import runpy
    from pathlib import Path
    script = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/ground_wall_probe.py'))
    report = tmp_path / 'pitch.json'
    flags = ['--self-stand', '--dual-front', '--pitch-probe']
    assert script['main'](flags+['--mock', '--demo', '--report', str(report)]) == 0
    data = json.loads(report.read_text())
    angles = [s['state']['commanded_pitch_deg'] for s in data['segments'] if s['command'].startswith('pitch ')]
    assert angles == [0.5, 1, 1.5, 2, 1.5, 1, 0.5, 0]
    assert data['segments'][-1]['state']['seated']
    # Reject deliberately at preflight and ensure no driver can open. Passing
    # --demo-pitch 0 must not shrink the live preflight to a static-only run.
    def rejected(settings, scenario, pitch):
        assert settings.pitch_probe and pitch == 2
        return {'passed': False, 'reason': 'test full-envelope preflight rejected'}
    monkeypatch.setitem(script['main'].__globals__, 'simulate', rejected)
    monkeypatch.setattr(script['sys'].stdin, 'isatty', lambda: True)
    def forbidden(*args, **kwargs):
        pytest.fail('Live driver opened before successful preflight')
    monkeypatch.setitem(script['main'].__globals__, 'Servo2040Driver', forbidden)
    with pytest.raises(SystemExit) as e:
        script['main'](flags+['--live', '--demo-pitch', '0'])
    assert e.value.code == 2


@pytest.mark.parametrize('self_stand', [False, True])
@pytest.mark.parametrize('name', ['L1', 'R1'])
def test_front_transfer_lifts_away_from_wall_and_retracts_before_lowering(self_stand, name):
    b = Bench(MockDriver(), MockVacuumIO(),
              Settings(self_stand=self_stand, dual_front=self_stand, max_press=2))
    run(b, 'start')
    other = 'R1' if name == 'L1' else 'L1'
    front_attached(b, other)
    anchors = dict(b.pose.feet)
    ground = b.geom.ground[name]
    min_gap = min(b.geom.wall_x-ground[0], b.geom.s.approach_gap)

    b.command(f'prepare {name}')
    while b.busy:
        b.tick()
        assert not b.frozen
        x, y, z = b.pose.feet[name]
        # All lateral/forward swing occurs only after reaching target height.
        if z < b.geom.s.height-1e-6:
            assert (x, y) == pytest.approx(ground[:2])
        assert b.geom.wall_x-x >= min_gap-1e-6
        for n in LEG_NAMES:
            if n != name:
                assert b.pose.feet[n] == anchors[n]
        assert b.attached == {other}
    assert b.geom.wall_x-b.pose.feet[name][0] == pytest.approx(60)
    run(b, f'touch {name}')
    assert b.pose.feet[name][0] == b.geom.wall_x
    run(b, f'press {name} 2')
    run(b, f'attach {name}')
    run(b, f'release {name}')
    b.command(f'return {name}')
    while b.busy:
        b.tick()
        assert not b.frozen
        x, y, z = b.pose.feet[name]
        if z < b.geom.s.height-1e-6:
            assert (x, y) == pytest.approx(ground[:2])
        for n in LEG_NAMES:
            if n != name:
                assert b.pose.feet[n] == anchors[n]
        assert b.attached == {other}
    assert b.pose.feet[name] == ground


@pytest.mark.parametrize('name', ['L1', 'R1'])
def test_new_hover_moves_knee_back_and_adds_cup_clearance(name):
    from hexapod.kinematics import leg_ik, leg_joint_points
    g = Geometry(Settings(self_stand=True))
    leg = g.cfg.leg(name)
    a = math.radians(leg.mount_angle_deg)
    def knee_x(gap):
        dx, dy, z = g.wall_x-gap-leg.mount_x, 0, g.s.height-g.s.body_height
        angles = leg_ik(g.cfg, math.cos(a)*dx+math.sin(a)*dy,
                        -math.sin(a)*dx+math.cos(a)*dy, z)
        x, y, _ = leg_joint_points(g.cfg, *angles)[2]
        return leg.mount_x+math.cos(a)*x-math.sin(a)*y
    assert knee_x(20)-knee_x(g.s.approach_gap) == pytest.approx(32.3, abs=0.1)
    assert g.s.approach_gap-20 == 40


@pytest.mark.parametrize('gap', [float('nan'), float('inf'), 0, 19, 81])
def test_invalid_approach_gap_is_rejected_before_hardware(gap):
    with pytest.raises(EntryError, match='approach_gap'):
        Geometry(Settings(approach_gap=gap))


def test_double_speed_halves_motion_duration_preserving_targets_and_hold_time():
    slow = Bench(MockDriver(), MockVacuumIO(),
                 Settings(self_stand=True, dual_front=True, pitch_probe=True, max_press=2, speed=10))
    fast = Bench(MockDriver(), MockVacuumIO(),
                 Settings(self_stand=True, dual_front=True, pitch_probe=True, max_press=2))
    for cmd in ('start', 'prepare L1', 'touch L1', 'press L1 2', 'attach L1',
                'prepare R1', 'touch R1', 'press R1 2', 'attach R1', 'hold',
                'pitch 0.5', 'pitch 0', 'release R1', 'return R1',
                'release L1', 'return L1', 'sit'):
        frames = []
        elapsed = []
        for b in (slow, fast):
            b.command(cmd)
            frames.append(len(b.frames))
            ticks = 0
            while b.busy and not b.frozen:
                b.tick()
                ticks += 1
                assert ticks < 10000
            assert not b.frozen
            elapsed.append(ticks*b.DT)
        if frames[0]:
            # Up to three independently rounded waypoints per command.
            assert frames[0]/2 <= frames[1] <= frames[0]/2+3
        if cmd == 'hold':
            assert elapsed == pytest.approx([10, 10])
        assert fast.pose == slow.pose
        assert fast.drv.pulses == pytest.approx(slow.drv.pulses)
        assert fast.attached == slow.attached


@pytest.mark.parametrize('speed', [0, 21, float('nan'), float('inf')])
def test_invalid_motion_speed_rejected(speed):
    with pytest.raises(EntryError, match='speed'):
        Geometry(Settings(speed=speed))


def shift_bench():
    b = Bench(MockDriver(), MockVacuumIO(),
              Settings(self_stand=True, dual_front=True, pitch_probe=True,
                       shift_probe=True, max_press=2))
    run(b, 'start')
    front_attached(b, 'L1')
    front_attached(b, 'R1')
    return b


def test_shift_sweep_preserves_world_anchors_with_fk_and_returns_original_outputs():
    from hexapod.kinematics import leg_fk
    b = shift_bench()
    initial = b.pose.copy()
    pulses = list(b.drv.pulses)
    run(b, 'hold')
    run(b, 'pitch 0.5')
    run(b, 'pitch 0')
    run(b, 'hold')
    for offset in (*range(5, 31, 5), *range(25, -1, -5)):
        previous = b.pose.body_x
        b.command(f'shift {offset}')
        samples = [previous]
        while b.busy:
            b.tick()
            assert not b.frozen
            assert b.pose.feet == initial.feet
            assert b.pose.pitch == 0 and b.pose.body_z == 90
            assert b.attached == {'L1', 'R1'}
            samples.append(b.pose.body_x)
            # Decode actual commanded pulses and use FK, independently of the
            # body's inverse transform, to recover all six fixed world targets.
            for leg in b.geom.cfg.legs:
                angles = []
                for cal in (leg.coxa, leg.femur, leg.tibia):
                    angle = cal.attach_deg + (b.drv.pulses[cal.channel]
                        - (cal.us_m45+cal.us_p45)/2) * 90 / (cal.sign*(cal.us_p45-cal.us_m45))
                    angles.append(math.radians(angle))
                x, y, z = leg_fk(b.geom.cfg, angles[0], angles[1], math.pi-angles[2])
                a = math.radians(leg.mount_angle_deg)
                world = (b.pose.body_x+leg.mount_x+math.cos(a)*x-math.sin(a)*y,
                         leg.mount_y+math.sin(a)*x+math.cos(a)*y, b.pose.body_z+z)
                assert world == pytest.approx(initial.feet[leg.name], abs=1e-8)
        assert samples == sorted(samples, reverse=offset < previous)
        assert samples[-1] == offset
        assert max(abs(y-x) for x,y in zip(samples,samples[1:])) <= 5*b.DT+1e-9
        if offset > previous:
            run(b, 'hold')
    assert b.pose == initial and b.drv.pulses == pytest.approx(pulses)
    for n in ('R1', 'L1'):
        run(b, f'release {n}')
        run(b, f'return {n}')
    run(b, 'sit')
    assert b.seated


def test_shift_requires_opt_in_and_fresh_hold_but_retreat_does_not():
    for settings in (Settings(shift_probe=True), Settings(self_stand=True, shift_probe=True)):
        with pytest.raises(EntryError, match='shift_probe requires'):
            Geometry(settings)
    b = pitch_bench()
    with pytest.raises(EntryError, match='requires --shift-probe'):
        b.command('shift 5')
    b = shift_bench()
    with pytest.raises(EntryError, match='Use hold'):
        b.command('shift 5')
    run(b, 'hold')
    run(b, 'shift 5')
    with pytest.raises(EntryError, match='Use hold'):
        b.command('shift 10')
    run(b, 'shift 0')
    with pytest.raises(EntryError, match='Use hold'):
        b.command('shift 5')


@pytest.mark.parametrize('offset', ['nan', 'inf', '-1', '0', '6', '30', '31'])
def test_invalid_shift_does_not_output_or_mutate_state(offset):
    b = shift_bench()
    run(b, 'hold')
    pose, count = b.pose.copy(), len(b.drv.history)
    with pytest.raises(EntryError, match='Shift probe:'):
        b.command('shift '+offset)
    assert b.pose == pose and len(b.drv.history) == count and not b.busy
    assert b.hold_pose == pose


@pytest.mark.parametrize('cmd', ['pitch 0.5', 'prepare L1', 'touch L1', 'press L1 2',
                                'attach L1', 'release L1', 'return R1', 'sit'])
def test_shift_blocks_foot_operations_and_pitch_until_origin(cmd):
    b = shift_bench()
    run(b, 'hold')
    run(b, 'shift 5')
    pose, count, valves = b.pose.copy(), len(b.drv.history), list(b.io.valve)
    with pytest.raises(EntryError):
        b.command(cmd)
    assert b.pose == pose and len(b.drv.history) == count and b.io.valve == valves and not b.busy


def test_shift_rejects_pitch_and_missing_support():
    b = shift_bench()
    run(b, 'hold')
    run(b, 'pitch 0.5')
    with pytest.raises(EntryError, match='Return pitch to 0'):
        b.command('shift 5')
    run(b, 'pitch 0')
    run(b, 'release R1')
    with pytest.raises(EntryError, match='two confirmed wall cups'):
        b.command('shift 5')


@pytest.mark.parametrize('fault', ['disabled', 'over_cap', 'negative', 'nan', 'pitch',
                                 'body_height', 'floor_target', 'wall_surface'])
def test_shift_geometry_guards_cannot_be_bypassed_by_scheduling(fault):
    b = shift_bench()
    pose, surfaces = b.pose.copy(), dict(b.surfaces)
    pose.body_x = 5
    if fault == 'disabled':
        b.geom = Geometry(replace(b.geom.s, shift_probe=False))
    elif fault in ('over_cap', 'negative', 'nan'):
        pose.body_x = {'over_cap':31, 'negative':-1, 'nan':float('nan')}[fault]
    elif fault == 'pitch':
        pose.pitch = 0.5
    elif fault == 'body_height':
        pose.body_z = 89
    elif fault == 'floor_target':
        pose.feet['L2'] = (*pose.feet['L2'][:2], 1)
    else:
        surfaces['L1'] = None
    with pytest.raises(EntryError):
        b.geom.solve(pose, surfaces)


def test_shift_recovery_invalidates_hold_and_pressure_loss_freezes_before_motion():
    b = shift_bench()
    run(b, 'hold')
    b.io.step = lambda dt: None
    b.io.foot_kpa[0] = -40
    b.tick()
    assert b.hold_pose is None and not b.frozen
    b.io.foot_kpa[0] = -70
    b.tick()
    with pytest.raises(EntryError, match='Use hold'):
        b.command('shift 5')
    run(b, 'hold')
    b.command('shift 5')
    b.tick()
    pose, count = b.pose.copy(), len(b.drv.history)
    b.io.foot_kpa[3] = -40
    b.tick()
    assert not b.frozen and b.busy and b.pose == pose and len(b.drv.history) == count
    b.io.foot_kpa[3] = -20
    b.tick()
    assert b.frozen and not b.busy and b.pose == pose and len(b.drv.history) == count
    assert b.io.valve[0] and b.io.valve[3] and not b.io.pump
    with pytest.raises(EntryError, match='Frozen'):
        b.command('shift 0')


def test_shift_return_preflight_failure_is_atomic():
    b = shift_bench()
    run(b, 'hold')
    pose, count = b.pose.copy(), len(b.drv.history)
    original_path = b.geom.path
    def path(start, goals, surfaces, dt=0.05):
        if start.body_x > 0 and goals[-1].body_x == 0:
            raise EntryError('return route unavailable')
        return original_path(start, goals, surfaces, dt)
    b.geom.path = path
    with pytest.raises(EntryError, match='return route unavailable'):
        b.command('shift 5')
    assert not b.busy and b.pose == pose and len(b.drv.history) == count and b.hold_pose == pose


def test_shift_cli_checks_full_sweep_and_retreat_before_opening_live_io(tmp_path, monkeypatch):
    import json
    import runpy
    from pathlib import Path
    script = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/ground_wall_probe.py'))
    report = tmp_path / 'shift.json'
    flags = ['--self-stand', '--dual-front', '--pitch-probe', '--shift-probe', '--approach-gap', '60']
    assert script['main'](flags+['--mock', '--demo', '--report', str(report)]) == 0
    data = json.loads(report.read_text())
    shifts = [s for s in data['segments'] if s['command'].startswith('shift ')]
    assert [s['state']['commanded_body_x_mm'] for s in shifts] == [5,10,15,20,25,30,25,20,15,10,5,0]
    assert all(s['state']['commanded_pitch_deg'] == 0 for s in shifts)
    assert shifts[5]['state']['commanded_front_hip_wall_distance_mm'] == {'L1':130, 'R1':130}
    assert data['segments'][-1]['state']['seated']
    original = script['simulate']
    def rejected(settings, scenario, pitch):
        result = original(settings, scenario, pitch)
        assert settings.shift_probe and settings.pitch_probe and pitch == 2
        assert any(s['command'] == 'shift 30' for s in result['segments'])
        assert result['segments'][-1]['state']['commanded_body_x_mm'] == 0
        return {'passed':False, 'reason':'test full shift preflight rejected'}
    monkeypatch.setitem(script['main'].__globals__, 'simulate', rejected)
    monkeypatch.setattr(script['sys'].stdin, 'isatty', lambda: True)
    def forbidden(*args, **kwargs):
        pytest.fail('Live driver opened before successful preflight')
    monkeypatch.setitem(script['main'].__globals__, 'Servo2040Driver', forbidden)
    with pytest.raises(SystemExit) as e:
        script['main'](flags+['--live', '--demo-pitch', '0'])
    assert e.value.code == 2
