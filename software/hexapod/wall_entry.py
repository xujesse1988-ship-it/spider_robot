"""Ground/wall experiments with supported, single-front or dual-front entry.

World +X faces the wall, +Z is up; positive pitch raises the nose.
Feet are virtual free-cup targets (pressure overtravel included), not measured
contact coordinates. No mesh collision, force or actual servo-position model.
"""
import math
from dataclasses import dataclass

from .config import DEFAULT_CONFIG, LEG_NAMES
from .kinematics import leg_ik, leg_joint_points, WorkspaceError
from .climb import _solve_reach
from .adhesion import AdhesionController, FootState


class EntryError(ValueError):
    pass


@dataclass(frozen=True)
class Settings:
    distance: float = 160.0       # front hip to wall with body level
    height: float = 224.0        # wall lip centre above floor
    approach_gap: float = 60.0   # lip-centre distance from wall at prepared hover
    body_height: float = 90.0
    speed: float = 10.0          # virtual foot mm/s
    tilt_limit: float = 15.0
    max_press: float = 18.0
    pitch_limit: float = 5.0      # experimental envelope, not a validated limit
    self_stand: bool = False     # stand_up geometry, one front foot off floor at a time
    dual_front: bool = False     # opt-in static two-wall/four-floor test; no pitch
    pitch_probe: bool = False    # opt-in <=2 deg probe after dual-front hold

    def validate(self):
        if not isinstance(self.self_stand, bool):
            raise EntryError('self_stand must be boolean')
        if not isinstance(self.dual_front, bool) or (self.dual_front and not self.self_stand):
            raise EntryError('dual_front requires self_stand')
        if not isinstance(self.pitch_probe, bool) or (self.pitch_probe and not self.dual_front):
            raise EntryError('pitch_probe requires self_stand and dual_front')
        bounds = dict(distance=(120, 200), height=(150, 280),
                      approach_gap=(20, 80),
                      body_height=(70, 100), speed=(1, 15),
                      tilt_limit=(1, 15), max_press=(2, 18), pitch_limit=(1, 10))
        for name, (lo, hi) in bounds.items():
            v = getattr(self, name)
            if not math.isfinite(v) or not lo <= v <= hi:
                raise EntryError(f"{name} must be finite and in [{lo}, {hi}]")


@dataclass
class Pose:
    feet: dict
    pitch: float = 0.0
    body_z: float = None        # None retains the fixed-height supported mode

    def copy(self):
        return Pose(dict(self.feet), self.pitch, self.body_z)


class Geometry:
    def __init__(self, settings=Settings(), cfg=DEFAULT_CONFIG):
        settings.validate()
        self.s, self.cfg = settings, cfg
        self.wall_x = cfg.leg('L1').mount_x + settings.distance
        self.ground = {}
        for leg in cfg.legs:
            r = (cfg.foot_reach if settings.self_stand else
                 _solve_reach(cfg, -settings.body_height - settings.max_press))
            a = math.radians(leg.mount_angle_deg)
            self.ground[leg.name] = (leg.mount_x + r * math.cos(a),
                                     leg.mount_y + r * math.sin(a), 0.0)

    def hip_height(self, pose):
        return self.s.body_height if pose.body_z is None else pose.body_z

    def front_waypoints(self, name):
        """Raise away from wall before approaching; reverse before lowering.

        These are free-cup centre targets, not sensed lip/mesh clearances.
        Keeping the lift above the floor anchor also avoids the tight IK
        configurations encountered by folding inward while still low.
        """
        return ((*self.ground[name][:2], self.s.height),
                (self.wall_x-self.s.approach_gap, self.cfg.leg(name).mount_y, self.s.height))

    @property
    def pitch_limit(self):
        return min(2.0, self.s.pitch_limit) if self.s.pitch_probe else self.s.pitch_limit

    def solve(self, pose, surfaces):
        """Check RAW pulses before clamp, cup normals and joint-centre clearance."""
        body_z = self.hip_height(pose)
        if not math.isfinite(body_z) or not 20 <= body_z <= self.s.body_height:
            raise EntryError('Body height outside crouch/stand envelope')
        if self.s.self_stand and not self.s.pitch_probe and pose.pitch != 0:
            raise EntryError('Self-standing mode has no pitch motion')
        if set(pose.feet) != set(LEG_NAMES):
            raise EntryError('Six feet required')
        if not math.isfinite(pose.pitch) or not 0 <= pose.pitch <= self.pitch_limit:
            raise EntryError('Pitch outside experiment envelope')
        if self.s.pitch_probe and pose.pitch != 0:
            if body_z != self.s.body_height:
                raise EntryError('Pitch probe requires standing body-origin height')
            # Four unsealed floor targets and both wall targets stay fixed.
            for n in LEG_NAMES:
                p = pose.feet[n]
                if n in ('L1', 'R1'):
                    if (surfaces.get(n) != 'wall' or len(p) != 3
                            or not self.wall_x <= p[0] <= self.wall_x+self.s.max_press
                            or p[1:] != (self.cfg.leg(n).mount_y, self.s.height)):
                        raise EntryError('Pitch probe requires both front targets on wall')
                elif surfaces.get(n) != 'floor' or p != self.ground[n]:
                    raise EntryError('Pitch probe requires fixed middle/rear floor targets')
        c, s = math.cos(math.radians(pose.pitch)), math.sin(math.radians(pose.pitch))
        pulses = [None] * 18
        tilts = {}
        for leg in self.cfg.legs:
            p = pose.feet[leg.name]
            if len(p) != 3 or not all(math.isfinite(v) for v in p):
                raise EntryError(f'{leg.name}: invalid target')
            x, y, z = p[0], p[1], p[2] - body_z
            bx, bz = c*x + s*z, -s*x + c*z
            a = math.radians(leg.mount_angle_deg)
            ca, sa = math.cos(a), math.sin(a)
            dx, dy = bx-leg.mount_x, y-leg.mount_y
            try:
                g, f, t = leg_ik(self.cfg, ca*dx+sa*dy, -sa*dx+ca*dy, bz)
            except WorkspaceError as e:
                raise EntryError(f'{leg.name}: {e}') from e
            for cal, angle in zip((leg.coxa, leg.femur, leg.tibia),
                                  (math.degrees(g), math.degrees(f), 180-math.degrees(t))):
                us = ((cal.us_m45+cal.us_p45)/2 + cal.sign*(angle-cal.attach_deg)
                      * (cal.us_p45-cal.us_m45)/90)
                # 50 us reserve is an electrical screen, not a physical-stop model.
                if not cal.min_us+50 <= us <= cal.max_us-50:
                    raise EntryError(f'{leg.name}/ch{cal.channel}: raw pulse {us:.1f} outside reserved range')
                pulses[cal.channel] = us
            beta = f+t-math.pi + math.radians(self.cfg.cup_delta_deg)
            nx, ny, nz = math.cos(beta)*math.cos(a+g), math.cos(beta)*math.sin(a+g), math.sin(beta)
            world_n = (c*nx-s*nz, ny, s*nx+c*nz)
            surface = surfaces.get(leg.name)
            if ((surface == 'wall' and p[0] >= self.wall_x-1e-6)
                    or (surface == 'floor' and not self.s.self_stand and p[2] <= 1e-6)):
                dot = world_n[0] if surface == 'wall' else -world_n[2]
                tilt = math.degrees(math.acos(max(-1, min(1, dot))))
                tilts[leg.name] = tilt
                if tilt > self.s.tilt_limit:
                    raise EntryError(f'{leg.name}: {surface} cup tilt {tilt:.1f} > {self.s.tilt_limit} deg')
            # Point skeleton only. Hardware mesh/hoses require observation.
            for lx, ly, lz in leg_joint_points(self.cfg, g, f, t)[:-1]:
                jx, jy = leg.mount_x+ca*lx-sa*ly, leg.mount_y+sa*lx+ca*ly
                wx, wz = c*jx-s*lz, s*jx+c*lz+body_z
                if wx > self.wall_x-10 or wz < 10:
                    raise EntryError(f'{leg.name}: joint centre near floor/wall')
            # Only virtual pressure overtravel is allowed through a surface.
            if p[0] > self.wall_x + (self.s.max_press if surface == 'wall' else -0.01) + 1e-6:
                raise EntryError(f'{leg.name}: foot crosses wall')
            if p[2] < -(self.s.max_press if surface == 'floor' else 0)-1e-6:
                raise EntryError(f'{leg.name}: foot crosses floor')
        return pulses, tilts

    def path(self, start, waypoints, surfaces, dt=0.05):
        frames = []
        prev = start
        self.solve(start, surfaces)
        for goal in waypoints:
            distance = max(math.dist(prev.feet[n], goal.feet[n]) for n in LEG_NAMES)
            pitch_speed = 0.25 if self.s.pitch_probe else 0.5
            duration = max(distance/self.s.speed, abs(goal.pitch-prev.pitch)/pitch_speed,
                           abs(self.hip_height(goal)-self.hip_height(prev))/self.s.speed, dt)
            # smoothstep peak speed is 1.5 times mean
            steps = max(1, math.ceil(1.5*duration/dt))
            for k in range(1, steps+1):
                u = k/steps
                u = u*u*(3-2*u)
                p = Pose({n: tuple(a+(b-a)*u for a, b in zip(prev.feet[n], goal.feet[n]))
                          for n in LEG_NAMES}, prev.pitch+(goal.pitch-prev.pitch)*u,
                         (self.hip_height(prev)+(self.hip_height(goal)-self.hip_height(prev))*u)
                         if prev.body_z is not None or goal.body_z is not None else None)
                pulses, _ = self.solve(p, surfaces)
                frames.append((p, pulses))
            prev = goal
        return frames


class Bench:
    DT = 0.05
    DUAL_HOLD_S = 10.0
    DUAL_HOLD_TIMEOUT_S = 30.0

    def __init__(self, driver, io, settings=Settings(), event=None, power_on=None):
        self.geom = Geometry(settings)
        self.drv, self.io = driver, io
        self.power_on = power_on or (lambda: self.drv.enable(True))
        self.event = event or (lambda message: None)
        self.ctl = AdhesionController(io, tankless=True, suck_timeout_s=2.5)
        self.pose = Pose({n: (p[0], p[1], settings.body_height-20)
                          for n, p in self.geom.ground.items()})
        if settings.self_stand:
            self.pose = Pose(dict(self.geom.ground), body_z=20.0)
        self.surfaces = {n: None for n in LEG_NAMES}
        self.stage = {n: 'park' for n in LEG_NAMES}
        self.depth = {n: 0.0 for n in LEG_NAMES}
        self.attached = set()
        self.frames = []
        self.finish = None
        self.elapsed = 0.0
        self.waiting = None
        self.wait_elapsed = self.good_elapsed = 0.0
        self.pressures = [0.0]*6
        self.voltage = self.current = 0.0
        self.overcurrent_s = 0.0
        self.frozen = None
        self.started = False
        self.seated = False
        self.gate_wait = 0.0
        self.motion_elapsed = 0.0
        self.hold_pose = None

    @property
    def busy(self):
        return bool(self.frames) or self.waiting is not None

    def freeze(self, reason):
        self.hold_pose = None
        self.frames.clear()
        self.finish = None
        self.waiting = None
        if self.frozen is None:
            self.event('FREEZE '+str(reason))
        self.frozen = str(reason)
        self.ctl.pump_inhibit = True
        try:
            self.io.set_pump(False)
        except Exception:
            pass

    def schedule(self, goals, surfaces=None, done=None):
        surfaces = dict(self.surfaces if surfaces is None else surfaces)
        # Entire route validated before changing command state or sending pulses.
        frames = self.geom.path(self.pose, goals, surfaces, self.DT)
        self.hold_pose = None
        self.frames = frames[::-1]
        self.motion_elapsed = 0.0
        self.surfaces = surfaces
        self.finish = done

    def target(self, name, xyz):
        p = self.pose.copy()
        p.feet[name] = tuple(xyz)
        return p

    def strong(self, names):
        return all(self.ctl.is_attached(LEG_NAMES.index(n))
                   and self.pressures[LEG_NAMES.index(n)] <= -50 for n in names)

    def floor_ready(self, name):
        """Command/pressure checks only: floor contact and load are not sensed."""
        idx = LEG_NAMES.index(name)
        return (self.stage[name] == 'floor' and self.surfaces[name] == 'floor'
                and self.depth[name] == 0 and name not in self.attached
                and self.pressures[idx] >= -5 and self.ctl.state[idx] == FootState.RELEASED)

    def dual_support_ready(self, name):
        other = 'R1' if name == 'L1' else 'L1'
        return (all(self.floor_ready(n) for n in ('L2', 'L3', 'R2', 'R3'))
                and (self.floor_ready(other)
                     or (self.stage[other] == 'wall' and self.surfaces[other] == 'wall'
                         and other in self.attached and self.strong([other]))))

    def two_wall_four_floor(self):
        return (self.attached == {'L1', 'R1'} and self.strong(('L1', 'R1'))
                and all(self.stage[n] == 'wall' and self.surfaces[n] == 'wall'
                        for n in ('L1', 'R1'))
                and all(self.floor_ready(n) for n in ('L2', 'L3', 'R2', 'R3')))

    def command(self, text):
        words = text.strip().split()
        if not words:
            return
        cmd = words[0].lower()
        if cmd == 'stop':
            self.freeze('operator stop; no automatic return or release')
            return
        if self.frozen:
            raise EntryError('Frozen: support robot, then quit. No automatic resume.')
        if self.busy:
            raise EntryError('Current segment not complete')
        if not self.strong(self.attached):
            raise EntryError('An attached cup is above -50 kPa; wait for recovery')
        s, geo = self.geom.s, self.geom
        if cmd == 'start' and len(words) == 1:
            if self.started:
                raise EntryError('Already started')
            goals = [Pose(dict(geo.ground), body_z=s.body_height if s.self_stand else None)]
            self.schedule(goals, {n: 'floor' for n in LEG_NAMES},
                          lambda: self.stage.update({n: 'floor' for n in LEG_NAMES}))
            # Self-standing uses stand_up's crouch -> stand, with fixed floor
            # anchors and a rising body; supported mode extends airborne legs.
            self.event('ARM crouch -> self-standing' if s.self_stand else
                       'ARM supported bench; initial target is parked legs')
            self.drv.set_all_pulses_us(geo.solve(self.pose, {n: None for n in LEG_NAMES})[0])
            try:
                self.power_on()
            except Exception as e:
                self.freeze(f'Arm failed: {e}')
                raise
            self.started = True
            return
        if not self.started:
            raise EntryError('Use start first')
        if self.seated:
            raise EntryError('Seated; quit and restart for another experiment')
        if cmd == 'hold' and len(words) == 1:
            if not s.dual_front:
                raise EntryError('hold requires --self-stand --dual-front')
            if not self.two_wall_four_floor():
                raise EntryError('hold requires two confirmed wall cups and four floor feet')
            self.hold_pose = None
            self.waiting = ('hold', None)
            self.wait_elapsed = self.good_elapsed = 0.0
            self.event('HOLD begin: both wall cups <= -50 kPa for 10 s; observe body/floor feet')
            return
        if cmd == 'sit' and len(words) == 1:
            if not s.self_stand:
                raise EntryError('sit is for self-standing mode')
            if (self.pose.pitch != 0 or self.attached or any(self.stage[n] != 'floor' for n in LEG_NAMES)
                    or any(self.depth.values()) or any(p < -5 for p in self.pressures)):
                raise EntryError('Return both front feet; all six feet must be released on floor')
            goal = self.pose.copy()
            goal.body_z = 20.0
            self.schedule([goal], done=lambda: setattr(self, 'seated', True))
            self.event('COMMAND sit; body descends with fixed floor targets')
            return
        if cmd == 'pitch' and len(words) == 2:
            if s.pitch_probe:
                pitch = float(words[1])
                if (not math.isfinite(pitch) or not 0 <= pitch <= geo.pitch_limit
                        or not 0 < abs(pitch-self.pose.pitch) <= 0.500001):
                    raise EntryError('Pitch probe: 0..2 deg (or lower configured limit), step >0 and <=0.5 deg')
                if not self.two_wall_four_floor():
                    raise EntryError('Pitch probe requires two confirmed wall cups and four floor feet')
                if pitch > self.pose.pitch and self.hold_pose != self.pose:
                    raise EntryError('Use hold at current pose before increasing pitch')
                goal, level = self.pose.copy(), self.pose.copy()
                goal.pitch, level.pitch = pitch, 0.0
                # Validate a geometrical route back to level before committing
                # the next segment. Return is still operator-commanded.
                geo.path(goal, [level], self.surfaces, self.DT)
                self.schedule([goal])
                self.event('COMMAND '+text+'; fixed world targets, body-origin height unchanged')
                return
            if s.self_stand:
                raise EntryError('Self-standing modes have no pitch; pitch needs supported mode')
            pitch = float(words[1])
            if self.attached != set(LEG_NAMES) or not self.strong(LEG_NAMES):
                raise EntryError('Pitch requires six confirmed cups: two wall, four floor')
            if any(self.surfaces[n] != ('wall' if n in ('L1','R1') else 'floor') for n in LEG_NAMES):
                raise EntryError('Mixed support surfaces not established')
            if not math.isfinite(pitch) or abs(pitch-self.pose.pitch) > 1.000001:
                raise EntryError('Pitch changes limited to 1 deg per command')
            goal = self.pose.copy()
            goal.pitch = pitch
            self.schedule([goal])
            self.event('COMMAND '+text)
            return
        if len(words) < 2 or words[1].upper() not in LEG_NAMES:
            raise EntryError('Expected prepare/touch/press/attach/release/return LEG, or pitch DEG')
        name = words[1].upper()
        idx = LEG_NAMES.index(name)
        if s.dual_front and name not in ('L1', 'R1'):
            raise EntryError('Dual-front mode keeps all four middle/rear feet on floor')
        if s.dual_front and cmd in ('prepare', 'release', 'return') and not self.dual_support_ready(name):
            raise EntryError('Keep four floor feet; other front foot must be back on floor or confirmed on wall')
        if self.pose.pitch != 0:
            raise EntryError('Return pitch to 0 before individual foot operations')
        if cmd in ('prepare','touch','return') and name not in ('L1','R1'):
            raise EntryError('Only L1/R1 may move to/from wall')
        if cmd == 'prepare' and len(words) == 2:
            if s.self_stand and not s.dual_front and any(self.stage[n] != 'floor' or self.depth[n] != 0
                                    or n in self.attached for n in LEG_NAMES if n != name):
                raise EntryError('Self-standing mode requires the other five feet on floor; return first foot before testing another')
            if (self.stage[name] != 'floor' or name in self.attached or self.depth[name] != 0
                    or self.pressures[idx] < -5 or self.ctl.state[idx] != FootState.RELEASED):
                raise EntryError('Prepare requires unpressed, released floor foot')
            lift_xyz, hover_xyz = geo.front_waypoints(name)
            lift = self.target(name, lift_xyz)
            hover = self.target(name, hover_xyz)
            surfaces = dict(self.surfaces, **{name: None})
            self.schedule([lift, hover], surfaces, lambda: self.stage.update({name:'hover'}))
        elif cmd == 'touch' and len(words) == 2:
            if self.stage[name] != 'hover':
                raise EntryError('Use prepare first')
            goal = self.target(name, (geo.wall_x, geo.cfg.leg(name).mount_y, s.height))
            self.schedule([goal], dict(self.surfaces, **{name:'wall'}),
                          lambda: self.stage.update({name:'wall'}))
        elif cmd == 'press' and len(words) == 3:
            if s.self_stand and self.stage[name] != 'wall':
                raise EntryError('Self-standing mode does not press/attach floor feet')
            step = float(words[2])
            depth = self.depth[name]+step
            if not math.isfinite(step) or not 0 < step <= 2 or depth > s.max_press+1e-6:
                raise EntryError('Press increment 0 < mm <= 2; total <= max-press')
            if self.stage[name] not in ('wall','floor') or name in self.attached:
                raise EntryError('Press only before attachment, on a contact surface')
            xyz = list(self.pose.feet[name])
            xyz[0 if self.surfaces[name]=='wall' else 2] += step if self.surfaces[name]=='wall' else -step
            self.schedule([self.target(name, xyz)], done=lambda: self.depth.update({name:depth}))
        elif cmd == 'attach' and len(words) == 2:
            if s.self_stand and self.stage[name] != 'wall':
                raise EntryError('Self-standing mode does not attach floor feet')
            if self.stage[name] not in ('wall','floor') or self.depth[name] < 2 or name in self.attached:
                raise EntryError('Touch surface and press at least 2 mm before attach')
            self.hold_pose = None
            self.ctl.request_attach(idx)
            self.waiting = ('attach', name)
            self.wait_elapsed = self.good_elapsed = 0.0
        elif cmd == 'release' and len(words) == 2:
            if self.stage[name] not in ('wall','floor'):
                raise EntryError('Foot is not on a contact surface')
            self.hold_pose = None
            self.attached.discard(name)
            self.ctl.force_release(idx)
            # RELEASED state may still have residual vacuum; explicitly vent.
            self.io.set_valve(idx, False)
            self.waiting = ('release', name)
            self.wait_elapsed = self.good_elapsed = 0.0
        elif cmd == 'return' and len(words) == 2:
            if self.stage[name] not in ('hover','wall') or name in self.attached:
                raise EntryError('Release cup before returning')
            if self.ctl.state[idx] != FootState.RELEASED or self.pressures[idx] < -5:
                raise EntryError('Release not confirmed')
            lift_xyz, hover_xyz = geo.front_waypoints(name)
            hover = self.target(name, hover_xyz)
            lift = self.target(name, lift_xyz)
            floor = self.target(name, geo.ground[name])
            def returned():
                self.stage[name], self.surfaces[name], self.depth[name] = 'floor','floor',0.0
            self.schedule([hover, lift, floor], dict(self.surfaces, **{name:self.surfaces[name]}), returned)
        else:
            raise EntryError('Unknown command or wrong argument count')
        self.event('COMMAND '+text)

    def tick(self, dt=None):
        dt = self.DT if dt is None else dt
        self.elapsed += dt
        try:
            if not self.frozen:
                self.ctl.update(dt)
            self.pressures = [self.io.read_foot_kpa(i) for i in range(6)]
            self.voltage, self.current = self.drv.read_voltage_v(), self.drv.read_current_a()
            if not all(math.isfinite(v) for v in self.pressures+[self.voltage,self.current]):
                raise EntryError('Nonfinite telemetry')
            if any(not -100 <= p <= 10 for p in self.pressures):
                raise EntryError('Pressure outside plausible range')
            # Servo2040 reports the switched servo bus. Before start, the
            # relay is open: a low bus reading is not a battery diagnosis.
            # Once power_on returns, check before sending any motion frame.
            if self.started and self.voltage < self.geom.cfg.volt_cutoff:
                self.drv.enable(False)
                raise EntryError(f'Servo bus {self.voltage:.2f} V below cutoff '
                                 f'{self.geom.cfg.volt_cutoff:.2f} V after power-on; servo power disabled')
            self.overcurrent_s = (self.overcurrent_s+dt
                                  if self.started and self.current > self.geom.cfg.curr_warn else 0.0)
            if self.overcurrent_s >= 0.3:
                self.drv.enable(False)
                raise EntryError('Current above configured warning for 0.3 s; servo power disabled')
            if any(self.pressures[LEG_NAMES.index(n)] > -30 for n in self.attached):
                raise EntryError('Attached cup lost vacuum (> -30 kPa)')
            if not self.strong(self.attached):
                self.hold_pose = None
        except Exception as e:
            self.freeze(e)
            # Sensors unavailable: no blind pumping or motion. Mechanical support required.
            return
        if self.frozen:
            return
        if self.waiting:
            mode, name = self.waiting
            if mode == 'hold':
                self.wait_elapsed += dt
                self.good_elapsed = self.good_elapsed+dt if self.strong(('L1', 'R1')) else 0.0
                if self.good_elapsed + 1e-9 >= self.DUAL_HOLD_S:
                    self.hold_pose = self.pose.copy()
                    self.event(f'HOLD confirmed L1 R1: both <= -50 kPa continuously for 10 s at '
                               f'commanded pitch {self.pose.pitch:g} deg; seal only, not load proof')
                    self.waiting = None
                elif self.wait_elapsed >= self.DUAL_HOLD_TIMEOUT_S:
                    self.freeze('Dual-front hold not stable within 30 s; no automatic release/return')
                return
            idx = LEG_NAMES.index(name)
            self.wait_elapsed += dt
            good = (self.ctl.is_attached(idx) and self.pressures[idx] <= -50) if mode=='attach' else (
                self.ctl.state[idx] == FootState.RELEASED and self.pressures[idx] >= -5)
            self.good_elapsed = self.good_elapsed+dt if good else 0.0
            if self.good_elapsed >= 0.5:
                if mode == 'attach':
                    self.attached.add(name)
                self.event(f'{mode.upper()} confirmed {name}')
                self.waiting = None
            elif self.ctl.state[idx] == FootState.FAULT or self.wait_elapsed > 10:
                self.freeze(f'{name} {mode} failed/timed out; no automatic withdrawal')
            return
        if self.frames and not self.strong(self.attached):
            self.gate_wait += dt
            if self.gate_wait > 10:
                self.freeze('Motion vacuum gate not recovered within 10 s')
            return
        self.gate_wait = 0.0
        if self.frames:
            self.motion_elapsed += dt
            if self.motion_elapsed + 1e-9 < self.DT:
                return
            # At most one sample per 50 ms; keyboard traffic cannot speed up
            # playback, and a delayed loop never emits a catch-up burst.
            self.motion_elapsed = 0.0
            pose, pulses = self.frames[-1]
            try:
                self.drv.set_all_pulses_us(pulses)
            except Exception as e:
                self.freeze(f'Servo write: {e}')
                return
            self.frames.pop()
            self.pose = pose
            if not self.frames:
                if self.finish:
                    self.finish()
                    self.finish = None
                self.event('SEGMENT complete')

    def shutdown(self):
        """No commanded vent/return. Power off requires support or completed sit."""
        errors = []
        actions = [lambda: self.io.set_pump(False)]
        actions += [lambda i=i: self.io.set_valve(i, True) for i in range(6)]
        actions += [self.drv.close, self.io.close]
        for action in actions:
            try:
                action()
            except Exception as e:
                errors.append(str(e))
        if errors:
            self.event('SHUTDOWN errors: '+'; '.join(errors))
