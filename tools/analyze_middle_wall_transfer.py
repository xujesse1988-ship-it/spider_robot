#!/usr/bin/env python3
"""Offline audit of animation steps 8/9/10; never opens hardware.

Animation geometry, current positive-knee IK branch and calibrated raw pulses.
Point-skeleton clearance and free-cup axis only; no meshes, loads or compliance.
"""
import argparse
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'software'))
from hexapod.config import DEFAULT_CONFIG as CFG
from hexapod.kinematics import leg_ik, leg_fk, leg_joint_points, WorkspaceError

WALL = 243.5
GROUND = dict(L1=(158.1, 169.5, 0), L2=(0, 211.5, 0), L3=(-158.1, 169.5, 0),
              R1=(158.1, -169.5, 0), R2=(0, -211.5, 0), R3=(-158.1, -169.5, 0))
ANIMATION_BODY = (50.0, 100.0, 25.0)  # world body-origin x,z,pitch
TILT = 15.0


def evaluate(name, foot, body, surface=None):
    leg = CFG.leg(name)
    ox, oz, pitch = body
    c, s = math.cos(math.radians(pitch)), math.sin(math.radians(pitch))
    ca, sa = math.cos(math.radians(leg.mount_angle_deg)), math.sin(math.radians(leg.mount_angle_deg))
    x, y, z = foot[0]-ox, foot[1], foot[2]-oz
    bx, bz = c*x+s*z, -s*x+c*z
    dx, dy = bx-leg.mount_x, y-leg.mount_y
    local = (ca*dx+sa*dy, -sa*dx+ca*dy, bz)
    result = dict(leg=name, foot_world_mm=foot, body_x_z_pitch=body, failures=[])
    try:
        gamma, alpha, theta = leg_ik(CFG, *local)
    except WorkspaceError as e:
        result['failures'].append('IK: '+str(e))
        return result
    angles = [math.degrees(gamma), math.degrees(alpha), 180-math.degrees(theta)]
    cals = (leg.coxa, leg.femur, leg.tibia)
    pulses = [(cal.us_m45+cal.us_p45)/2+cal.sign*(angle-cal.attach_deg)
              * (cal.us_p45-cal.us_m45)/90 for cal, angle in zip(cals, angles)]
    for cal, pulse in zip(cals, pulses):
        if not cal.min_us+50 <= pulse <= cal.max_us-50:
            result['failures'].append(f'ch{cal.channel} raw {pulse:.2f} us outside reserved range')
    beta = alpha+theta-math.pi+math.radians(CFG.cup_delta_deg)
    yaw = math.radians(leg.mount_angle_deg)+gamma
    nx, ny, nz = math.cos(beta)*math.cos(yaw), math.cos(beta)*math.sin(yaw), math.sin(beta)
    normal = (c*nx-s*nz, ny, s*nx+c*nz)
    tilt = math.degrees(math.acos(max(-1, min(1, normal[0]))))
    if surface == 'wall' and tilt > TILT:
        result['failures'].append(f'wall cup tilt {tilt:.2f} deg > {TILT:g} deg')
    points = []
    for lx, ly, lz in leg_joint_points(CFG, gamma, alpha, theta):
        jx, jy = leg.mount_x+ca*lx-sa*ly, leg.mount_y+sa*lx+ca*ly
        points.append((ox+c*jx-s*lz, jy, oz+s*jx+c*lz))
    if any(p[0] > WALL-10 or p[2] < 10 for p in points[:-1]):
        result['failures'].append('joint centre clearance below 10 mm')
    result.update(joint_deg=angles, raw_us=pulses, channels=[cal.channel for cal in cals],
                  wall_tilt_deg=tilt, wall_normal=normal,
                  raw_margin_us=min(min(p-cal.min_us, cal.max_us-p) for cal, p in zip(cals, pulses)),
                  joint_points_world_mm=points, fk_error_mm=math.dist(local, leg_fk(CFG, gamma, alpha, theta)))
    return result


def initial_feet(press=0):
    feet = dict(GROUND)
    feet.update(L1=(WALL+press, 63, 224), R1=(WALL+press, -63, 224))
    return feet


def pose_checks(feet, body, wall_names):
    return [evaluate(n, p, body, 'wall' if n in wall_names else None) for n,p in feet.items()]


def path_audit(name, body, waypoints, support_feet, wall_names, spacing=1.0):
    first_failure, failed_samples, count = None, 0, 0
    max_tilt, min_margin, max_fk_error = 0, math.inf, 0
    for seg, (a,b) in enumerate(zip(waypoints, waypoints[1:])):
        steps = max(1, math.ceil(math.dist(a,b)/spacing))
        for k in range(steps+1):
            u = k/steps
            foot = tuple(x+(y-x)*u for x,y in zip(a,b))
            # All support feet remain fixed; final moving foot is wall contact.
            feet = dict(support_feet, **{name:foot})
            contact = set(wall_names)
            if foot[0] >= WALL-1e-9:
                contact.add(name)
            checks = pose_checks(feet, body, contact)
            failures = {r['leg']:r['failures'] for r in checks if r['failures']}
            if failures:
                failed_samples += 1
                if first_failure is None:
                    first_failure = dict(segment=seg, u=u, foot=foot, failures=failures)
            for r in checks:
                max_fk_error = max(max_fk_error, r.get('fk_error_mm', 0))
                min_margin = min(min_margin, r.get('raw_margin_us', math.inf))
                if r['leg'] in contact:
                    max_tilt = max(max_tilt,r.get('wall_tilt_deg',0))
            count += 1
    return dict(passed=not failed_samples, sample_spacing_mm=spacing, samples=count, failed_samples=failed_samples,
                first_failure=first_failure, min_raw_margin_us=min_margin,
                max_support_wall_tilt_deg=max_tilt, max_fk_error_mm=max_fk_error)


def body_audit(target, press=0):
    start=(0,90,0)
    n=max(1,math.ceil(max(abs(target[0]), abs(target[1]-90), abs(target[2])*10)))
    worst,first=0,None
    for k in range(n+1):
        body=tuple(a+(b-a)*k/n for a,b in zip(start,target))
        checks=pose_checks(initial_feet(press),body,{'L1','R1'})
        failures={r['leg']:r['failures'] for r in checks if r['failures']}
        worst=max(worst,max((r.get('wall_tilt_deg',0) for r in checks if r['leg'] in ('L1','R1')),default=0))
        if failures and first is None:
            first=dict(body_x_z_pitch=body,failures=failures)
    return dict(passed=first is None,first_failure=first,max_front_tilt_deg=worst,samples=n+1)


def search():
    """Finite nearby grid, not a proof of global feasibility/impossibility."""
    hits=[]
    for pitch in range(0,31,5):
        for x in range(0,81,10):
            for z in (80,90,100,110):
                body=(x,z,pitch)
                if any(r['failures'] for r in pose_checks(initial_feet(2),body,{'L1','R1'})):
                    continue
                for y in (85,95,110,130,150):
                    for height in range(140,281,5):
                        # Screen touch and the experimentally used 2 mm overtravel.
                        legs=[evaluate(n,(WALL+d,sign*y,height),body,'wall')
                              for n,sign in (('L2',1),('R2',-1)) for d in (0,2)]
                        if any(r['failures'] for r in legs):
                            continue
                        score=max(r['wall_tilt_deg'] for r in legs)
                        hits.append(dict(body=body,y=y,height=height,max_middle_tilt=score,
                                         min_raw_margin=min(r['raw_margin_us'] for r in legs)))
    # Prefer nearby body adjustments rather than minimum middle tilt alone.
    hits.sort(key=lambda v:(abs(v['body'][0])+abs(v['body'][1]-90)+2*v['body'][2],v['max_middle_tilt']))
    candidates=[]
    seen=set()
    for h in hits:
        if h['body'] in seen:
            continue
        seen.add(h['body'])
        h['body_route']=body_audit(h['body'],2)
        feet=initial_feet(2)
        paths=[]
        for name,sign in (('L2',1),('R2',-1)):
            points=[GROUND[name],(*GROUND[name][:2],h['height']),
                    (WALL-60,sign*h['y'],h['height']),(WALL,sign*h['y'],h['height'])]
            paths.append(path_audit(name,h['body'],points,feet,
                                   {'L1','R1'} if name=='L2' else {'L1','R1','L2'}))
            feet[name]=(WALL+2,sign*h['y'],h['height'])
        h['simple_transfer_paths']=paths
        candidates.append(h)
        if len(candidates)>=8:
            break
    return dict(grid=dict(body_x_mm=[0,80,10],body_z_mm=[80,90,100,110],pitch_deg=[0,30,5],
                          middle_abs_y_mm=[85,95,110,130,150],height_mm=[140,280,5]),
                endpoint_pair_hits=len(hits),nearby_candidates=candidates)


def candidate_audit():
    """One nearby candidate with more pulse reserve; not a global optimum."""
    body=(30,90,0)
    height,y=190,110
    feet=initial_feet(2)
    walls={'L1','R1'}
    moves=[]
    endpoints=[]
    for name,sign in (('L2',1),('R2',-1)):
        points=[GROUND[name],(*GROUND[name][:2],height),(WALL-60,sign*y,height),
                (WALL,sign*y,height),(WALL+2,sign*y,height)]
        moves.append(dict(command='transfer '+name,
                          checks=path_audit(name,body,points,feet,walls,0.25)))
        endpoints.extend(evaluate(name,p,body,'wall') for p in points[-2:])
        feet[name]=points[-1]
        walls.add(name)
    four_wall_pose=pose_checks(feet,body,walls)
    for name,sign in (('R2',-1),('L2',1)):
        walls.remove(name)  # released cup still checked against wall normal until withdrawn
        points=[feet[name],(WALL-60,sign*y,height),(*GROUND[name][:2],height),GROUND[name]]
        moves.append(dict(command='released return '+name,
                          checks=path_audit(name,body,points,feet,walls,0.25)))
        feet[name]=GROUND[name]
    return dict(body_x_z_pitch=body,middle_abs_y_mm=y,middle_height_mm=height,
                touch_x_mm=WALL,pressed_x_mm=WALL+2,approach_gap_mm=60,
                body_adjustment=body_audit(body,2),middle_contact_and_press=endpoints,
                four_wall_pose=four_wall_pose,moves=moves,
                body_return='Reverse of the same checked straight body adjustment after both middle feet return',
                limitations=['No mesh collisions, cup-edge envelope, friction, support force, torque or loaded compliance model',
                             '0.25 mm sampled middle-foot paths, 1 mm sampled body translation; no continuous-path proof',
                             'Existing live script has no body-translation or middle-foot-transfer commands'])


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--report',type=Path)
    ap.add_argument('--search',action='store_true')
    args=ap.parse_args()
    feet=initial_feet()
    output=dict(scope='Animation steps 8/9/10; current IK branch, 50 us reserve, 15 deg cup-axis gate; no hardware/load proof',
                wall_x_mm=WALL,body_x_z_pitch=ANIMATION_BODY,
                animation_step8=body_audit(ANIMATION_BODY),
                step8_with_2mm_press=body_audit(ANIMATION_BODY,2),steps=[])
    for number,name,sign in ((9,'L2',1),(10,'R2',-1)):
        points=[GROUND[name],(*GROUND[name][:2],200),(WALL-60,sign*95,200),(WALL,sign*95,200)]
        endpoint=evaluate(name,points[-1],ANIMATION_BODY,'wall')
        moving_only=path_audit(name,ANIMATION_BODY,points,{name:GROUND[name]},set())
        all_feet=path_audit(name,ANIMATION_BODY,points,feet,{'L1','R1'} if number==9 else {'L1','R1','L2'})
        output['steps'].append(dict(step=number,leg=name,endpoint=endpoint,
                                    moving_leg_path=moving_only,all_feet_path=all_feet))
        feet[name]=points[-1]
    if args.search:
        output['nearby_search']=search()
    output['nearby_candidate']=candidate_audit()
    text=json.dumps(output,ensure_ascii=False,indent=2,allow_nan=False)
    if args.report:
        args.report.write_text(text+'\n')
        print(args.report)
    else:
        print(text)


if __name__=='__main__':
    main()
