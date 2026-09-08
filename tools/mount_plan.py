#!/usr/bin/env python3
"""地-墙过渡离线规划器（2026-09-06）：用 hexapod/mount.py 的几何与判据，从"地面
爬墙站位、前髋距墙 D"起搜一条完整关键帧序列——前足上墙 → 后足指正后 →
中腿收起 → 四足扶梯（固定接触抬头到极限→换步→再抬）→ 后足上墙 → 中腿上墙
→ 身体转到平行墙面 → 六足换回爬墙站位，每一步都过 MountEngine 同一套判据
（IK 余量/关节行程/coxa 偏摆/吸盘倾角/膝与机身不撞面）。

输出：JSON（给 html/wall-mount-20260906.html 的动画回放）+ 阶段摘要。
运行（项目根）：.venv/bin/python tools/mount_plan.py [--json out.json] [--wall-dist 140]
只算运动学，不算载荷（载荷估算见 tools/mount_analysis.py 四节）。
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "software"))
from hexapod.config import DEFAULT_CONFIG, LEG_NAMES          # noqa: E402
from hexapod.mount import (MountEngine, MountPhase, FLOOR, b2w, w2b, _add,   # noqa: E402
                           PITCH_RATE_DPS, LIN_RATE_MMS, TRANSFER_SPEED_MMS,
                           BELLY_MM, BODY_HALF_MM, TILT_BAND_DEG, HOLD_TILT_DEG)

d2r, r2d = math.radians, math.degrees
PITCH_STEP = 2.5
NEIGH = tuple(range(-40, 41, 10))


class PlanError(RuntimeError):
    pass


class Planner:
    def __init__(self, cfg, wall_dist):
        self.cfg = cfg
        self.e = MountEngine(cfg, None, front_hip_to_wall=wall_dist)
        self.e.started = True
        self.segs = []
        self.log = []

    # ---------- 记录 ----------
    def say(self, txt):
        e = self.e
        self.log.append(f"[φ={e.pitch_deg:5.1f}° 高{e.pose[1]:4.0f} 前髋距墙"
                        f"{e.front_hip_to_wall():4.0f}] {txt}")

    def _state(self):
        e = self.e
        return {n: ({"surf": e.surf[n].name, "pw": list(e.pw[n]), "depth": e.depth[n]}
                    if e.surf[n] is not None else
                    {"surf": None, "pb": list(e.air_pb[n])}) for n in LEG_NAMES}

    # ---------- 动作 ----------
    def move(self, name, surf, p_w, label):
        e, cfg = self.e, self.cfg
        p_w = surf.project(p_w)
        why = e._check_landing(name, p_w, surf, e.pose)
        if why:
            raise PlanError(f"{label}: {name} 落点不可行 {why}")
        start = e._liftoff_world(name)
        n_a = e.surf[name].n if e.surf[name] else surf.n
        end = _add(p_w, surf.n, cfg.lift_clearance)
        arc, why = e._check_path(name, start, end, n_a, surf.n)
        if why:
            raise PlanError(f"{label}: {name} 路径不可行 {why}")
        leg = cfg.leg(name)
        sol = e.geom[name].solve(w2b(_add(p_w, surf.n, -leg.press_delta_mm), e.pose),
                                 w2b_dir_neg(surf.n, e.pose))
        self.segs.append(dict(
            type="move", leg=name, label=label, pose=list(e.pose),
            from_w=list(e._foot_world(name)), n_a=list(n_a), start_w=list(start),
            end_w=list(end), n_b=list(surf.n), arc=arc, dst=surf.name,
            p_to=list(p_w), depth=leg.press_delta_mm,
            gamma=round(sol["gamma"], 1), tilt=round(sol["tilt"], 1)))
        e.surf[name], e.pw[name], e.depth[name] = surf, p_w, leg.press_delta_mm
        e.phase_of[name] = MountPhase.STANCE
        self.say(f"{label}：{name} → {surf.name} ({p_w[0]:.0f},{p_w[1]:.0f},{p_w[2]:.0f}) "
                 f"coxa {sol['gamma']:+.0f}° 倾角 {sol['tilt']:.1f}° 弧 {arc:g}")

    def tuck(self, name, label):
        e, cfg = self.e, self.cfg
        pb = e.tuck_point(name)
        why = e._check_air(name, pb, e.pose)
        if why:
            raise PlanError(f"{label}: {name} 收腿点不可行 {why}")
        start = e._liftoff_world(name)
        n_a = e.surf[name].n if e.surf[name] else (0.0, 0.0, 1.0)
        end = b2w(pb, e.pose)
        arc, why = e._check_path(name, start, end, n_a, n_a, arcs=(0.0,))
        if why:
            raise PlanError(f"{label}: {name} 收腿路径不可行 {why}")
        self.segs.append(dict(
            type="move", leg=name, label=label, pose=list(e.pose),
            from_w=list(e._foot_world(name)), n_a=list(n_a), start_w=list(start),
            end_w=list(end), n_b=list(n_a), arc=0.0, dst="air", pb_to=list(pb)))
        e.surf[name], e.air_pb[name] = None, pb
        e.phase_of[name] = MountPhase.AIR
        self.say(f"{label}：{name} 收起悬空")

    def glide(self, waypoints, label):
        """waypoints: 已逐个预检过的位姿列表（含起点之外的点）。"""
        e = self.e
        if not waypoints:
            return
        self.segs.append(dict(type="pose", label=label, path=[list(e.pose)]
                              + [list(p) for p in waypoints]))
        e.pose = waypoints[-1]

    # ---------- 抬头探测 ----------
    def _contacts_in_band(self, pose):
        """接触足在 pose 下是否全在落点带内（12°，比引擎 HOLD 15° 严）——
        抬头段用它，后面换步/落点判据才不会被卡在带外。"""
        e = self.e
        for n in LEG_NAMES:
            if e.surf[n] is not None and e._check_contact(
                    n, e.pw[n], e.surf[n], pose, e.depth[n], TILT_BAND_DEG):
                return False
        return True

    def _next_pose(self, pose, dphi, strict=True):
        xb, zb, phi = pose
        nphi = phi + d2r(dphi)
        best = None
        for dx in NEIGH:
            for dz in NEIGH:
                cand = (xb + dx, zb + dz, nphi)
                if self.e._check_pose(cand) is None and \
                        (not strict or self._contacts_in_band(cand)):
                    cost = abs(dx) + abs(dz)
                    if best is None or cost < best[0]:
                        best = (cost, cand)
        return best[1] if best else None

    def probe(self, phi_max_deg, pose=None, stop=None, max_steps=40, strict=True):
        """从 pose 起逐 PITCH_STEP 抬头，返回可达位姿序列（不改状态）。
        stop(pose) 为真时提前停（如"后腿已能落墙"）。"""
        pose = pose or self.e.pose
        out = []
        for _ in range(max_steps):
            if r2d(pose[2]) + PITCH_STEP > phi_max_deg + 1e-6:
                break
            nxt = self._next_pose(pose, PITCH_STEP, strict)
            if nxt is None:
                break
            out.append(nxt)
            pose = nxt
            if stop is not None and stop(pose):
                break
        return out

    # ---------- 落墙扫描 ----------
    def scan_wall(self, name, pose=None):
        """该腿在 pose 下能落墙的最正落点 (p_w, tilt, gamma)，无则 None。
        除落点判据外还要求：coxa 留 3° 余量、从当前抬离点到落点前悬停点的
        摆动路径可行（悬停点比压入位更外、coxa/femur 可能恰好越界）。"""
        e, cfg = self.e, self.cfg
        pose = pose or e.pose
        saved = e.pose
        e.pose = pose
        try:
            hip = e.hip_world(name, pose)
            sign = 1.0 if cfg.leg(name).mount_y > 0 else -1.0
            start = e._liftoff_world(name)
            n_a = e.surf[name].n if e.surf[name] else e.wall.n
            best = None
            for h in range(int(hip[2]) - 230, int(hip[2]) + 61, 10):
                if h < 20:
                    continue
                for dy in (0, 15, 30, 45, 60, 80, 100):
                    p = e.wall.project((0.0, hip[1] + sign * dy, float(h)))
                    if e._check_landing(name, p, e.wall, pose):
                        continue
                    sol = e.geom[name].solve(
                        w2b(_add(p, e.wall.n, -cfg.leg(name).press_delta_mm), pose),
                        w2b_dir_neg(e.wall.n, pose))
                    if abs(sol["gamma"]) > e.geom[name].coxa_max - 3.0:
                        continue
                    if best is not None and sol["tilt"] >= best[1]:
                        continue
                    arc, why = e._check_path(name, start, _add(p, e.wall.n, cfg.lift_clearance),
                                             n_a, e.wall.n)
                    if why:
                        continue
                    best = (p, sol["tilt"], sol["gamma"])
            return best
        finally:
            e.pose = saved

    def can_land_wall(self, names, pose):
        return all(self.scan_wall(n, pose) is not None for n in names)

    # ---------- 状态快照（换步候选回滚用）----------
    def _snapshot(self):
        e = self.e
        return (e.pose, dict(e.surf), dict(e.pw), dict(e.depth), dict(e.air_pb),
                dict(e.phase_of))

    def _restore(self, st):
        e = self.e
        e.pose, surf, pw, depth, air, ph = st
        e.surf, e.pw, e.depth, e.air_pb, e.phase_of = (dict(surf), dict(pw),
                                                       dict(depth), dict(air), dict(ph))

    # ---------- 阶梯搜索（最少换步，Dijkstra）----------
    def _cells(self):
        return [(float(x), float(z)) for x in range(-380, -50, 10)
                for z in range(50, 271, 10)]

    def _feas_maps(self, phis, hws, rears, mid_air):
        """预计算两套格图：hold（接触足在位姿下可保持：倾角≤HOLD 15°、压深 press，
        与引擎 _check_pose 同判据）与 land（可作为换步落点：_check_landing，
        12° 带、压深 0/press/加深，与 move() 同判据）。
        body[pi] = 机身+中腿收腿点可行的格。"""
        e, cfg = self.e, self.cfg
        cells = self._cells()
        body, fh, fl, rh, rl = {}, {}, {}, {}, {}

        def hold_ok(n, p, surf, pose):
            """可保持：压入位过 HOLD 倾角，且抬离点（面上 lift_clearance）IK/
            行程/膝都过——否则这条腿在此位姿下抬不起来，换不了步。"""
            if e._check_contact(n, p, surf, pose, cfg.leg(n).press_delta_mm,
                                HOLD_TILT_DEG):
                return False
            return e._check_contact(n, p, surf, pose, -cfg.lift_clearance, 1e9) is None
        for pi, phi in enumerate(phis):
            ph = d2r(phi)
            ok = []
            for (x, z) in cells:
                pose = (x, z, ph)
                if e._check_body(pose):
                    continue
                if mid_air and any(e._check_air(n, e.tuck_point(n), pose)
                                   for n in ("L2", "R2")):
                    continue
                ok.append((x, z))
            body[pi] = frozenset(ok)
            for hi, hw in enumerate(hws):
                pts = {n: e.wall_target(n, hw) for n in ("L1", "R1")}
                fh[pi, hi] = frozenset(
                    c for c in ok if all(hold_ok(n, pts[n], e.wall, (c[0], c[1], ph))
                                         for n in pts))
                fl[pi, hi] = frozenset(
                    c for c in fh[pi, hi] if not any(
                        e._check_landing(n, pts[n], e.wall, (c[0], c[1], ph))
                        for n in pts))
            for ri, spec in enumerate(rears):
                if spec[0] == "floor":
                    pts = {"L3": (spec[1], 63.0, 0.0), "R3": (spec[1], -63.0, 0.0)}
                    surf = FLOOR
                else:
                    pts = {"L3": spec[1][0], "R3": spec[1][1]}
                    surf = e.wall
                rh[pi, ri] = frozenset(
                    c for c in ok if all(hold_ok(n, pts[n], surf, (c[0], c[1], ph))
                                         for n in pts))
                rl[pi, ri] = frozenset(
                    c for c in rh[pi, ri] if not any(
                        e._check_landing(n, pts[n], surf, (c[0], c[1], ph))
                        for n in pts))
        return body, fh, fl, rh, rl

    @staticmethod
    def _near(A, B):
        if A & B:
            return True
        for x, z in A:
            for dx in (-10.0, 0.0, 10.0):
                for dz in (-10.0, 0.0, 10.0):
                    if (x + dx, z + dz) in B:
                        return True
        return False

    def ladder(self, hws, rears, mid_air, goal, hi0, ri0, phi0, label):
        """从 (phi0, hws[hi0], rears[ri0]) 起搜最少换步路径到 goal(pi, phi, cells)
        为真。返回 (phis, [((pi,hi,ri), act)], feas)；act = pitch|front|rear。"""
        import heapq
        phis = [phi0 + PITCH_STEP * i for i in range(int((90.0 - phi0) / PITCH_STEP) + 1)]
        body, fh, fl, rh, rl = self._feas_maps(phis, hws, rears, mid_air)

        def feas(pi, hi, ri):            # 可保持
            return fh[pi, hi] & rh[pi, ri]

        def can_land(pi, hi, ri, act, k2):  # 换步：同格下旧接触可保持且新落点可落
            if act == "front":
                return fh[pi, hi] & rh[pi, ri] & fl[pi, k2]
            return fh[pi, hi] & rh[pi, ri] & rl[pi, k2]

        INF = 10 ** 9
        dist, prev = {}, {}
        s0 = (0, hi0, ri0)
        if not feas(*s0):
            raise PlanError(f"{label}: 起点不可行")
        dist[s0] = 0
        pq = [(0, s0)]
        goal_state = None
        while pq:
            d, st = heapq.heappop(pq)
            if d > dist.get(st, INF):
                continue
            pi, hi, ri = st
            S = feas(pi, hi, ri)
            if goal(pi, phis[pi], S, hi, ri):
                goal_state = st
                break
            if pi + 1 < len(phis):
                S2 = feas(pi + 1, hi, ri)
                if S2 and self._near(S, S2):
                    k = (pi + 1, hi, ri)
                    if d < dist.get(k, INF):
                        dist[k] = d; prev[k] = (st, "pitch"); heapq.heappush(pq, (d, k))
            for hi2 in range(len(hws)):
                if hi2 != hi:
                    if can_land(pi, hi, ri, "front", hi2):
                        k = (pi, hi2, ri)
                        if d + 1 < dist.get(k, INF):
                            dist[k] = d + 1; prev[k] = (st, "front"); heapq.heappush(pq, (d + 1, k))
            for ri2 in range(len(rears)):
                if ri2 != ri and rears[ri2][0] == "floor":
                    if can_land(pi, hi, ri, "rear", ri2):
                        k = (pi, hi, ri2)
                        if d + 1 < dist.get(k, INF):
                            dist[k] = d + 1; prev[k] = (st, "rear"); heapq.heappush(pq, (d + 1, k))
        if goal_state is None:
            mx = max(dist, key=lambda k: (k[0], -dist[k]))
            raise PlanError(f"{label}: 未达目标，最高 φ={phis[mx[0]]}°（换步 {dist[mx]}）")
        path, k = [], goal_state
        while k in prev:
            pk, act = prev[k]
            path.append((k, act))
            k = pk
        path.reverse()
        return phis, path, feas, can_land

    def _goto_cell(self, cell, label):
        """同俯仰下把身体直线平移到目标格（逐 5mm 预检）。"""
        e = self.e
        cur = e.pose
        if abs(cur[0] - cell[0]) < 1e-6 and abs(cur[1] - cell[1]) < 1e-6:
            return
        path = self._linear_path(cur, (cell[0], cell[1], cur[2]))
        if path is None:
            raise PlanError(f"{label}: 同俯仰平移到 {cell} 不可行")
        self.glide(path, f"{label} 平移")

    @staticmethod
    def _pick_cell(S, near):
        return min(S, key=lambda c: abs(c[0] - near[0]) + abs(c[1] - near[1]))

    def _snap(self, label):
        """把当前位姿吸附到搜索格（xb/zb 10mm、φ 2.5°）。"""
        e = self.e
        cell = self._pick_cell(set(self._cells()), e.pose[:2])
        self._goto_cell(cell, label)
        phi_snap = round(e.pitch_deg / PITCH_STEP) * PITCH_STEP
        if abs(phi_snap - e.pitch_deg) > 1e-6:
            p = (e.pose[0], e.pose[1], d2r(phi_snap))
            if e._check_pose(p):
                raise PlanError(f"{label} 俯仰吸附到格失败")
            self.glide([p], f"{label} 对齐")

    def run_ladder(self, hws, rears, mid_air, goal, hi0, ri0, tag, restep_front, restep_rear):
        """执行 ladder 搜出的路径：抬头段合并成一段 glide，换步段先平移到
        新旧都可行的格再挪腿。"""
        e = self.e
        phis, path, feas, can_land = self.ladder(hws, rears, mid_air, goal, hi0, ri0,
                                                 e.pitch_deg, tag)
        hi, ri = hi0, ri0
        pending = []
        stage = 0
        for (pi, hi2, ri2), act in path:
            if act == "pitch":
                S = feas(pi, hi, ri)
                cell = self._pick_cell(S, pending[-1][:2] if pending else e.pose[:2])
                pending.append((cell[0], cell[1], d2r(phis[pi])))
                continue
            if pending:
                self._flush_pitch(pending, f"{tag}{stage}")
                pending = []
            both = can_land(pi, hi, ri, act, hi2 if act == "front" else ri2)
            # 换步：格图只保证落点可行，摆动路径还要在具体位姿下过；先试
            # 离当前最近的格，路径不过就换下一格（快照回滚）
            near = e.pose[:2]
            cells = sorted(both, key=lambda c: abs(c[0] - near[0]) + abs(c[1] - near[1]))
            done = False
            last_err = None
            for cell in cells[:12]:
                snap = (self._snapshot(), len(self.segs), len(self.log))
                try:
                    self._goto_cell(cell, f"{tag}{stage}")
                    if act == "front":
                        restep_front(hws[hi2], f"{tag}{stage}")
                    else:
                        restep_rear(rears[ri2][1], f"{tag}{stage}")
                    done = True
                    break
                except PlanError as ex:
                    last_err = ex
                    st, ns, nl = snap
                    self._restore(st)
                    del self.segs[ns:]
                    del self.log[nl:]
            if not done:
                raise PlanError(f"{tag}{stage} 换步在 {len(cells)} 个候选格都失败：{last_err}")
            if act == "front":
                hi = hi2
            else:
                ri = ri2
            stage += 1
        if pending:
            self._flush_pitch(pending, f"{tag}{stage}")
        gc = getattr(self, "_goal_cell", None)
        if gc is not None:
            self._goto_cell(gc, f"{tag}{stage} 靠墙")
            self._goal_cell = None
        return hi, ri

    def _piece_ok(self, a, b):
        """a→b 直线细分（5mm/1°）逐点过引擎 _check_pose。"""
        n = max(1, int(math.ceil(max(abs(b[0] - a[0]) / 5.0, abs(b[1] - a[1]) / 5.0,
                                     r2d(abs(b[2] - a[2])) / 1.0))))
        for k in range(1, n + 1):
            t = k / n
            q = tuple(x + (y - x) * t for x, y in zip(a, b))
            if self.e._check_pose(q):
                return False
        return True

    def _flush_pitch(self, poses, label):
        """格点位姿序列铺成一段 glide：相邻格之间先试直线，不过再试"先平移后
        俯仰"/"先俯仰后平移"两种折线（格点本身可行，直线插值可能擦过 HOLD
        倾角边界）。全部用引擎 _check_pose 复核。"""
        e = self.e
        cur = e.pose
        out = []
        for p in poses:
            routes = ([p], [(p[0], p[1], cur[2]), p], [(cur[0], cur[1], p[2]), p])
            chosen = None
            for r in routes:
                a, ok = cur, True
                for q in r:
                    if not self._piece_ok(a, q):
                        ok = False
                        break
                    a = q
                if ok:
                    chosen = r
                    break
            if chosen is None:
                raise PlanError(f"{label}: 抬头路径复核失败 {tuple(round(v, 1) for v in cur)}"
                                f" → φ={r2d(p[2]):.1f}° 三种走法都不过")
            out.extend(chosen)
            cur = p
        self.glide(out, f"{label} 抬头到 {r2d(cur[2]):.1f}°")
        self.say(f"{label} 抬头到 {e.pitch_deg:.1f}°")

    # ---------- 主流程 ----------
    def run(self, hw0=None, phi_rear=60.0, phi_mid=75.0):
        e, cfg = self.e, self.cfg
        init = dict(pose=list(e.pose), legs=self._state())
        # A. 前足上墙
        band = e.wall_band("L1")
        if band is None:
            raise PlanError("起始位姿前腿无可落足带")
        if hw0 is None:
            hw0 = (band[0] + band[1]) / 2.0
        self.say(f"A 起点：前腿可落足带 离地 {band[0]:.0f}~{band[1]:.0f}，取 {hw0:.0f}")
        self.move("L1", e.wall, e.wall_target("L1", hw0), "A1 左前足上墙")
        self.move("R1", e.wall, e.wall_target("R1", hw0), "A2 右前足上墙")
        self.move("L3", FLOOR, e.floor_back("L3"), "A3 左后足指正后")
        self.move("R3", FLOOR, e.floor_back("R3"), "A4 右后足指正后")
        # B0. 中腿还在地上先能抬多少抬多少（六接触最稳），再收中腿
        poses = self.probe(90.0, strict=False)
        if poses:
            self.glide(poses, f"B0 六足抬头到 {r2d(poses[-1][2]):.1f}°")
            self.say(f"B0 六接触抬头到 {e.pitch_deg:.1f}°")
        self.tuck("L2", "B0 左中腿收起")
        self.tuck("R2", "B0 右中腿收起")
        # B. 四足扶梯：最少换步搜索（前对换墙高 / 后对换地面位置）
        hw0 = e.pw["L1"][2]
        hws = sorted({hw0} | {float(h) for h in range(150, 481, 10)})
        rx0 = e.pw["L3"][0]
        rears = [("floor", x) for x in sorted(
            {rx0} | {float(x) for x in range(int(rx0) - 40, -160, 10)})]
        ri0 = [i for i, r in enumerate(rears) if r[1] == rx0][0]
        hi0 = hws.index(hw0)

        land_cache = {}

        def with_state(hi, ri, fn):
            """把引擎接触状态临时置为搜索态 (hw, rear) 再算（scan_wall 的摆动
            路径起点取自当前接触点）。"""
            snap = self._snapshot()
            try:
                for n in ("L1", "R1"):
                    e.surf[n], e.pw[n] = e.wall, e.wall_target(n, hws[hi])
                    e.depth[n] = cfg.leg(n).press_delta_mm
                spec = rears[ri]
                for n, y in (("L3", 63.0), ("R3", -63.0)):
                    if spec[0] == "floor":
                        e.surf[n], e.pw[n] = FLOOR, (spec[1], y, 0.0)
                    else:
                        e.surf[n], e.pw[n] = e.wall, spec[1][0 if n == "L3" else 1]
                    e.depth[n] = cfg.leg(n).press_delta_mm
                return fn()
            finally:
                self._restore(snap)

        def goal_rear(pi, phi, S, hi, ri):
            """φ 达标且 S 里存在某个位姿让后两腿都能落墙（只查后髋距墙
            ≤150 的格）。"""
            if phi < phi_rear - 1e-6:
                return False
            ph = d2r(phi)
            cand = []
            for c in S:
                d = e.wall.height(e.hip_world("L3", (c[0], c[1], ph)))
                if d <= 150.0:
                    cand.append((d, c))
            for _, c in sorted(cand):
                key = (round(phi, 1), c, hi, ri)
                if key not in land_cache:
                    land_cache[key] = with_state(
                        hi, ri, lambda: self.can_land_wall(("L3", "R3"), (c[0], c[1], ph)))
                if land_cache[key]:
                    self._goal_cell = c
                    return True
            return False

        def rf(hw, lab):
            for n, t in (("L1", "左前"), ("R1", "右前")):
                self.move(n, e.wall, e.wall_target(n, hw), f"{lab} {t}足换到墙高 {hw:.0f}")

        def rr(rx, lab):
            for n, t in (("L3", "左后"), ("R3", "右后")):
                self.move(n, FLOOR, (rx, e.hip_world(n)[1], 0.0),
                          f"{lab} {t}足换到距墙 {-rx:.0f}")
        self._snap("B1")
        hi, ri = self.run_ladder(hws, rears, True, goal_rear, hi0, ri0, "B", rf, rr)
        # C. 后足上墙
        for n, lab in (("L3", "C1 左后足上墙"), ("R3", "C2 右后足上墙")):
            best = self.scan_wall(n)
            if best is None:
                raise PlanError(f"{lab}: 无可落点")
            self.move(n, e.wall, best[0], lab)
        # C3. 四盘在墙继续抬头到中腿能落墙（后对固定、前对可换）
        rears_c = [("wall", (e.pw["L3"], e.pw["R3"]))]

        land_cache2 = {}

        def goal_mid(pi, phi, S, hi, ri):
            if phi < phi_mid - 1e-6:
                return False
            ph = d2r(phi)
            cand = []
            for c in S:
                d = e.wall.height(e.hip_world("L2", (c[0], c[1], ph)))
                if d <= 170.0:
                    cand.append((d, c))
            for _, c in sorted(cand):
                key = (round(phi, 1), c, hi)
                if key not in land_cache2:
                    land_cache2[key] = with_state_c(
                        hi, lambda: self.can_land_wall(("L2", "R2"), (c[0], c[1], ph)))
                if land_cache2[key]:
                    self._goal_cell = c
                    return True
            return False

        def with_state_c(hi, fn):
            snap = self._snapshot()
            try:
                for n in ("L1", "R1"):
                    e.surf[n], e.pw[n] = e.wall, e.wall_target(n, hws[hi])
                    e.depth[n] = cfg.leg(n).press_delta_mm
                return fn()
            finally:
                self._restore(snap)
        hw_c = e.pw["L1"][2]
        hws = sorted({hw_c} | {float(h) for h in range(150, 481, 10)})
        hi = hws.index(hw_c)
        self._snap("C3")
        self.run_ladder(hws, rears_c, True, goal_mid, hi, 0, "C", rf, None)
        # D. 中腿上墙
        for n, lab in (("L2", "D1 左中腿上墙"), ("R2", "D2 右中腿上墙")):
            best = self.scan_wall(n)
            if best is None:
                raise PlanError(f"{lab}: 无可落点")
            self.move(n, e.wall, best[0], lab)
        # D3+. 六盘在墙：转到平行墙面。转不动就逐足向"当前位姿下的爬墙站位
        # 投影点"换步（腿站得更正，俯仰就能再转），最多 6 轮
        for k in range(6):
            poses = self.probe(90.0, strict=False)
            if poses:
                self.glide(poses, f"D{3 + k} 六盘在墙转到 {r2d(poses[-1][2]):.1f}°")
                self.say(f"D{3 + k} 六盘在墙转到 {e.pitch_deg:.1f}°")
            if e.pitch_deg >= 89.9:
                break
            moved = 0
            for n in e.slot_order:
                p = e.wall.project(b2w(e.default_feet[n], e.pose))
                cur = e.pw[n]
                if math.dist(p, cur) < 15.0:
                    continue
                try:
                    self.move(n, e.wall, p, f"D{3 + k} {n} 换向站位")
                    moved += 1
                except PlanError as ex:
                    self.say(f"D{3 + k} {n} 换向站位不可行：{ex}")
            if moved == 0 and not poses:
                self.say(f"D{3 + k} 转不动也换不了步，停在 {e.pitch_deg:.1f}°")
                break
        # E. 换回爬墙站位：身体向离墙 stand_height 滑（能滑多少滑多少）与
        # 逐足换到"当前距离下的站位投影点"交替，直到离墙 90 且六足都在站位
        final_ok = False
        if e.pitch_deg >= 89.9:
            for k in range(5):
                target = (-cfg.stand_height, e.pose[1], e.pose[2])
                path = self._linear_path(e.pose, target, partial=True)
                if path:
                    self.glide(path, f"E{k} 身体向离墙站高滑到 {e.front_hip_to_wall(path[-1]):.0f}")
                    self.say(f"E{k} 身体滑到离墙 {e.front_hip_to_wall():.0f}")
                at_dist = abs(e.pose[0] + cfg.stand_height) < 1.0
                moved = 0
                for n in e.slot_order:
                    p = e.wall.project(b2w(e.default_feet[n], e.pose))
                    if math.dist(p, e.pw[n]) < 2.0:
                        continue
                    try:
                        self.move(n, e.wall, p, f"E{k} {n} 换到站位")
                        moved += 1
                    except PlanError as ex:
                        self.say(f"E{k} {n} 换到站位不可行：{ex}")
                if at_dist and all(math.dist(e.wall.project(b2w(e.default_feet[n], e.pose)),
                                             e.pw[n]) < 2.0 for n in LEG_NAMES):
                    final_ok = True
                    break
                if not path and moved == 0:
                    self.say(f"E{k} 滑不动也换不了步，停在离墙 {e.front_hip_to_wall():.0f}")
                    break
        return dict(init=init, segs=self.segs, log=self.log, final_stance=final_ok)

    def _linear_path(self, a, b, step_mm=5.0, partial=False):
        """a→b 直线逐 step_mm 预检；partial=False 时任一点不过返回 None，
        partial=True 返回能走到的前缀（可能为空列表）。"""
        n = max(1, int(math.ceil(max(abs(b[0] - a[0]), abs(b[1] - a[1])) / step_mm)))
        out = []
        for k in range(1, n + 1):
            s = k / n
            p = tuple(x + (y - x) * s for x, y in zip(a, b))
            if self.e._check_pose(p):
                return out if partial else None
            out.append(p)
        return out


def w2b_dir_neg(n, pose):
    from hexapod.mount import w2b_dir
    return w2b_dir((-n[0], -n[1], -n[2]), pose)


def params(cfg):
    legs = {l.name: dict(mx=l.mount_x, my=l.mount_y, ang=l.mount_angle_deg,
                         press=l.press_delta_mm) for l in cfg.legs}
    return dict(coxa=cfg.coxa_len, femur=cfg.femur_len, tibia=cfg.tibia_len,
                cup_delta=cfg.cup_delta_deg, stand=cfg.stand_height,
                lift_clearance=cfg.lift_clearance, lift_speed=cfg.lift_speed,
                transfer_speed=TRANSFER_SPEED_MMS, transfer_min=cfg.transfer_time,
                descend_speed=cfg.descend_speed, press_speed=cfg.press_speed,
                pitch_rate=PITCH_RATE_DPS, lin_rate=LIN_RATE_MMS,
                belly=BELLY_MM, body_half=BODY_HALF_MM, tilt_band=TILT_BAND_DEG,
                legs=legs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wall-dist", type=float, default=140.0)
    ap.add_argument("--wall-height", type=float, default=None)
    ap.add_argument("--json", default=None, help="输出 JSON 路径")
    args = ap.parse_args()
    pl = Planner(DEFAULT_CONFIG, args.wall_dist)
    try:
        res = pl.run(hw0=args.wall_height)
    finally:
        for l in pl.log:
            print(l)
    n_move = sum(1 for s in res["segs"] if s["type"] == "move")
    n_pose = sum(1 for s in res["segs"] if s["type"] == "pose")
    print(f"\n单腿动作 {n_move} 次，位姿铺设 {n_pose} 段，终态换回爬墙站位="
          f"{res['final_stance']}，终态 φ={pl.e.pitch_deg:.1f}°")
    if args.json:
        out = dict(params=params(DEFAULT_CONFIG), wall_dist=args.wall_dist, **res)
        with open(args.json, "w") as f:
            json.dump(out, f, ensure_ascii=False, separators=(",", ":"))
        print(f"JSON → {args.json}")


if __name__ == "__main__":
    main()
