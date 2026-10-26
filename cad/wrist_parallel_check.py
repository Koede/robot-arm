"""Check the parallel (spherical five-bar) wrist proxies over the serial W2 -70..80 / W3 -20..30 grid. New file; reads kin.json, writes check.json.
(a) minimum clearance per non-adjacent body pair (python-fcl), (b) colliding poses and worst overlap, (c) claw roll vs the serial wrist at the same axis,
(d) cable crossings of moving joints. Usage: python3 wrist_parallel_check.py [--fingers] [--params path] [--psi A B] [--tag name]
One core, a few minutes."""
import os, sys, json, itertools, time, numpy as np, trimesh
from trimesh.transformations import translation_matrix as tr
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import wrist_parallel as wp, wrist_parallel_kin as wk
FIXED = ["servoA", "servoB", "brA", "brB", "forearm", "tube"]; MOVING = ["crank1", "float1", "float2", "crank2", "tray"]
ADJ = {("servoA", "crank1"), ("crank1", "float1"), ("float1", "float2"), ("float2", "crank2"), ("crank2", "servoB"), ("float2", "tray"),
       ("servoA", "brA"), ("servoB", "brB")}
W2S = list(range(-70, 81, 10)); W3S = list(range(-20, 31, 10))

def pairs():
    out = []
    for a, b in itertools.combinations(FIXED + MOVING, 2):
        if a in FIXED and b in FIXED: continue
        if (a, b) in ADJ or (b, a) in ADJ: continue
        if "tube" in (a, b) and "forearm" in (a, b): continue
        out.append((a, b))
    return out

def forearm_meshes():
    f = trimesh.load(os.path.join(wp.ROOT, "distal", "forearm_dist_R.stl"))
    try: tube = f.slice_plane([0, 0, 388.0], [0, 0, -1], cap=False)
    except Exception: tube = f
    return f, tube

def frame2(a, b):
    z = wk.unit(np.cross(a, b)); return np.stack([a, np.cross(z, a), z], 1)

def link_T(Fnow, Fref):
    T = np.eye(4); T[:3, :3] = Fnow @ Fref.T; return tr(wp.C) @ T @ tr(-wp.C)

def circ_range(v):
    v = np.sort(np.mod(v, 360.0)); gaps = np.diff(np.concatenate([v, [v[0] + 360.0]])); return float(360.0 - gaps.max())

def interior_angle(P, Q, R_):
    """angle at point Q between arcs Q->P and Q->R (deg), on the unit sphere."""
    tp = P - (P @ Q) * Q; tr_ = R_ - (R_ @ Q) * Q
    return float(np.degrees(np.arctan2(np.cross(tp, tr_) @ Q, tp @ tr_)))

def run(params, psiA=0.0, psiB=0.0, fingers=False, verbose=True, w2s=W2S, w3s=W3S):
    t0 = time.time(); sol = wp.solve_grid(params, w2s, w3s)
    wp.FINGERS = fingers; wp._HAND.clear(); ref = sol[(0, 0)]; B, P = wp.build_ref(params, ref["th1"], ref["th2"], ref["sg"], psiA, psiB)
    wp.FINGERS = fingers; wp._HAND.clear()
    forearm, tube = forearm_meshes(); B["forearm"], B["tube"] = forearm, tube
    Fref = dict(crank1=frame2(P["J0"], P["J1"]), float1=frame2(P["J1"], P["J2"]), float2=frame2(P["J2"], P["J3"]), crank2=frame2(P["J3"], P["J4"]))
    Fref["tray"] = Fref["float2"]
    mgr = {}
    for k, m in B.items():
        mgr[k] = trimesh.collision.CollisionManager(); mgr[k].add_object(k, m)
    PR = pairs(); res = {}; rolls = {}; angles = {}; jumps = {}; sig = {}
    for (w2, w3), s in sol.items():
        Q = wp.one(params, s["th1"], s["th2"], s["sg"]); J = dict(J0=Q["J0"], J1=Q["J1"], J2=Q["J2"], J3=Q["J3"], J4=Q["J4"])
        Fn = dict(crank1=frame2(J["J0"], J["J1"]), float1=frame2(J["J1"], J["J2"]), float2=frame2(J["J2"], J["J3"]), crank2=frame2(J["J3"], J["J4"]))
        Fn["tray"] = Fn["float2"]
        for k in MOVING: mgr[k].set_transform(k, link_T(Fn[k], Fref[k]))
        d = {}
        for a, b in PR:
            dist = mgr[a].min_distance_other(mgr[b])
            if dist <= 0.0 or mgr[a].in_collision_other(mgr[b]):
                _, data = mgr[a].in_collision_other(mgr[b], return_data=True)
                dep = max([c.depth for c in data] + [0.0]); d[(a, b)] = -max(dep, 1e-3)
            else: d[(a, b)] = float(dist)
        res[(w2, w3)] = d
        Rc = Fn["float2"] @ Fref["float2"].T; Rs = wp.serial_frame(w2, w3)
        M = Rs.T @ Rc; rolls[(w2, w3)] = float(np.degrees(np.arctan2(M[1, 0], M[0, 0])))
        angles[(w2, w3)] = dict(J3=interior_angle(J["J2"], J["J3"], J["J4"]), J2=interior_angle(J["J1"], J["J2"], J["J3"]), J1=interior_angle(J["J0"], J["J1"], J["J2"]),
                                th1=s["th1"], th2=s["th2"])
        jumps[(w2, w3)] = s.get("jump", 0.0); sig[(w2, w3)] = s["sigma"]
    # ---- summaries
    pair_min = {f"{a}|{b}": min(res[k][(a, b)] for k in res) for a, b in PR}
    pair_worst_pose = {f"{a}|{b}": min(res, key=lambda k: res[k][(a, b)]) for a, b in PR}
    coll = {k: {f"{a}|{b}": v for (a, b), v in d.items() if v < 0} for k, d in res.items()}; coll = {k: v for k, v in coll.items() if v}
    pose_min = {k: min(d.values()) for k, d in res.items()}
    r = np.array(list(rolls.values())); rk = list(rolls)
    out = dict(n_poses=len(res), s_min=min(sig.values()), branch_jumps_max_deg=max(jumps.values()), n_err_gt_1p5=int(sum(s["err"] > 1.5 for s in sol.values())),
               pair_min_clearance=pair_min, pair_worst_pose={k: list(v) for k, v in pair_worst_pose.items()},
               n_colliding_poses=len(coll), colliding_poses={f"{k[0]},{k[1]}": v for k, v in coll.items()},
               worst_overlap_mm=float(min(pose_min.values())) if min(pose_min.values()) < 0 else 0.0,
               overall_min_clearance=float(min(pose_min.values())),
               roll_deg=dict(min=float(r.min()), max=float(r.max()), span=float(r.max() - r.min()), abs_max_about_ref0=float(np.abs(r).max()),
                             centred_halfspan=float((r.max() - r.min()) / 2), at_min=list(rk[int(r.argmin())]), at_max=list(rk[int(r.argmax())]),
                             frac_within_10=float((np.abs(r) <= 10).mean()), frac_within_10_centred=float((np.abs(r - (r.max() + r.min()) / 2) <= 10).mean())),
               joint_angle_range_deg={j: [float(min(a[j] for a in angles.values())), float(max(a[j] for a in angles.values()))] for j in ("J1", "J2", "J3", "th1", "th2")},
               cable=dict(servo_cables_crossing_moving_joints=0, claw_G_cable_min_joints_crossed=2, claw_G_cable_route="hand->J3->crank2->J4 (servo B shaft axis)",
                          J3_angle_span_deg=float(max(a["J3"] for a in angles.values()) - min(a["J3"] for a in angles.values()))),
               psi=[psiA, psiB], fingers_included=fingers, seconds=round(time.time() - t0, 1))
    out["crank_travel_deg"] = dict(th1=circ_range([a["th1"] for a in angles.values()]), th2=circ_range([a["th2"] for a in angles.values()]),
                                   J3_fold=circ_range([a["J3"] for a in angles.values()]), J2_fold=circ_range([a["J2"] for a in angles.values()]))
    out["_rolls"] = {f"{k[0]},{k[1]}": v for k, v in rolls.items()}
    return out, res

def summary(out):
    print(f"poses {out['n_poses']}  sigma_min {out['s_min']:.3f}  branch jump max {out['branch_jumps_max_deg']:.1f} deg  unreached {out['n_err_gt_1p5']}")
    print(f"colliding poses {out['n_colliding_poses']}/{out['n_poses']}  worst overlap {out['worst_overlap_mm']:.2f} mm  min clearance {out['overall_min_clearance']:.2f} mm")
    for k, v in sorted(out["pair_min_clearance"].items(), key=lambda kv: kv[1]): print(f"  {k:18s} {v:8.2f} mm  worst at {out['pair_worst_pose'][k]}")
    print("roll", {k: (round(v, 1) if isinstance(v, float) else v) for k, v in out["roll_deg"].items()})
    print("joint angles", {k: [round(x, 1) for x in v] for k, v in out["joint_angle_range_deg"].items()}); print("cable", out["cable"], "t", out["seconds"], "s")

if __name__ == "__main__":
    a = sys.argv; params = wp.load_params(a[a.index("--params") + 1] if "--params" in a else None)
    psi = (float(a[a.index("--psi") + 1]), float(a[a.index("--psi") + 2])) if "--psi" in a else (0.0, 0.0)
    tag = a[a.index("--tag") + 1] if "--tag" in a else ""
    out, _ = run(params, *psi, fingers="--fingers" in a); summary(out)
    json.dump(out, open(os.path.join(wp.OUT, f"check{tag}.json"), "w"), indent=1)
