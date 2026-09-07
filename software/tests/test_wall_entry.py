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
