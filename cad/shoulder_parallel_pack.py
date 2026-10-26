"""Parallel spherical shoulder: packaging / collision check of the 3-RRR design from shoulder_parallel_kin.py.   (new file, 2026-10-26)

Proxies (all axes through P, sphere radius R_S = 80 mm):
  * 3 x AK70-10 cylinders (dia 89, length 50.25): flange face at radial distance R_S + 12 along the motor axis u_i, body outward.
  * proximal link i: tube (r 5) along the great-circle arc u_i -> v_i at R_S plus a 12 mm stalk to the motor flange; rotates with theta_i about u_i.
  * distal link i: tube (r 5) along the arc v_i -> w_i at R_S (rigid body, moved by the IK solution).
  * platform: hub ball r 28 at P, three spokes P -> R_S w_i (r 5), arm stub cylinder r 36 along the arm axis z (0..113); the upper arm (arm_kin M3 group, z >= 113) rides on it.
Collision (python-fcl distance on BVH): moving = platform+stub, proximal, distal links, upper arm M3 (and optionally ME..MW3 forearm/claw) over the serial pose grid,
fixed = 3 motors, chassis plate, clasp frame, pauldron, wearer skin (gate 2 mm), wearer head/neck (gate 60 mm). Gates: 3 mm part-part and to the harness (SPEC_collision_method).
Writes proxy STLs (park pose) and stl/shoulder_parallel/pack_result.json.

Usage: python3 shoulder_parallel_pack.py <kin_search json or kin_result json with design.x> [--step 10] [--forearm]"""
import os, sys, json, time
import numpy as np, trimesh, fcl
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, ".."); OUT = os.path.join(ROOT, "stl", "shoulder_parallel"); STL = os.path.join(ROOT, "stl")
sys.path.insert(0, HERE)
import shoulder_parallel_kin as K
R_S = 80.0; R_LEG = [62.0, 80.0, 98.0]; FLANGE = R_LEG[2] + 12.0;   # nested shells: one sphere radius per leg
LINK_R = 5.0; AK_D, AK_L = 89.0, 50.25
GATE_PART, GATE_SKIN, GATE_HEAD = 3.0, 2.0, 60.0
def seg(p0, p1, r):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float); v = p1 - p0; L = np.linalg.norm(v)
    c = trimesh.creation.cylinder(radius=r, height=L, sections=14); z = np.array([0, 0, 1.0]); ax = np.cross(z, v / L); s = np.linalg.norm(ax)
    T = np.eye(4)
    if s > 1e-9: T = trimesh.transformations.rotation_matrix(np.arctan2(s, z @ (v / L)), ax / s)
    elif (v / L)[2] < 0: T = trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])
    T[:3, 3] = (p0 + p1) / 2; c.apply_transform(T); return c
def arc_mesh(a, b, r=LINK_R, rad=R_S, n=10):
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b); om = np.arccos(np.clip(a @ b, -1, 1)); pts = []
    for t in np.linspace(0, 1, n):
        pts.append(rad * (np.sin((1 - t) * om) * a + np.sin(t * om) * b) / np.sin(om))
    parts = [seg(pts[i], pts[i + 1], r) for i in range(n - 1)] + [trimesh.creation.uv_sphere(radius=r * 1.25, count=[8, 8]).apply_translation(pts[0]), trimesh.creation.uv_sphere(radius=r * 1.25, count=[8, 8]).apply_translation(pts[-1])]
    return trimesh.util.concatenate(parts)
def motor_mesh(u, fl=None):
    fl = FLANGE if fl is None else fl
    c = trimesh.creation.cylinder(radius=AK_D / 2, height=AK_L, sections=48)          # axis z, centred
    z = np.array([0, 0, 1.0]); ax = np.cross(z, u); s = np.linalg.norm(ax); T = np.eye(4)
    if s > 1e-9: T = trimesh.transformations.rotation_matrix(np.arctan2(s, z @ u), ax / s)
    T[:3, 3] = u * (fl + AK_L / 2); c.apply_transform(T); return c
def rotmat_about(u, ang):
    k = u / np.linalg.norm(u); Kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); return np.eye(3) + np.sin(ang) * Kx + (1 - np.cos(ang)) * Kx @ Kx
def frame_vw(v, w):
    n = w - (w @ v) * v; n /= np.linalg.norm(n); return np.stack([v, n, np.cross(v, n)], 1)
def T4(Rm, t=None):
    T = np.eye(4); T[:3, :3] = Rm
    if t is not None: T[:3, 3] = t
    return T
class FObj:
    def __init__(self, mesh):
        self.mesh = mesh; m = fcl.BVHModel(); m.beginModel(len(mesh.vertices), len(mesh.faces)); m.addSubModel(np.asarray(mesh.vertices, float), np.asarray(mesh.faces, np.int32)); m.endModel()
        self.o = fcl.CollisionObject(m, fcl.Transform())
    def set(self, T):
        self.o.setTransform(fcl.Transform(T[:3, :3], T[:3, 3]))
def dist(a, b, cap=1e9):
    req = fcl.DistanceRequest(); res = fcl.DistanceResult(); d = fcl.distance(a.o, b.o, req, res)
    if d < 0: return 0.0
    return d
def load_world():
    w = trimesh.load(os.path.join(STL, "shoulder/proxies/wearer_makehuman.stl")); lm = json.load(open(os.path.join(STL, "shoulder/proxies/wearer_landmarks.json")))
    zneck = lm["neck_joint_rel_P"][2]; cz = w.vertices[w.faces].mean(1)[:, 2]
    head = trimesh.Trimesh(w.vertices, w.faces[cz > zneck], process=False); skin = trimesh.Trimesh(w.vertices, w.faces[cz <= zneck], process=False)
    head.remove_unreferenced_vertices(); skin.remove_unreferenced_vertices()
    return dict(skin=skin, head=head, plate=trimesh.load(os.path.join(STL, "shoulder/chassis_plate_R.stl")), clasp=trimesh.load(os.path.join(STL, "shoulder/clasp_frame_R.stl")),
                pauldron=trimesh.load(os.path.join(STL, "shoulder/pauldron/pauldron_R.stl")))
def build(x, forearm=False):
    m = K.decode(x); th0, V0, W0, _ = m.ik(np.eye(3)[None]); Rhome = K.R3_of(45.0, 25.0, 0.0)
    th_h, V_h, W_h, vi = m.ik(Rhome[None]); th_h = th_h[0]; V_h = V_h[0]; W_h = W_h[0]
    mot = [motor_mesh(m.u[i], R_LEG[i] + 12.0) for i in range(3)]
    prox = []; dist_l = []
    for i in range(3):
        stalk = seg(m.u[i] * R_LEG[i], m.u[i] * (R_LEG[i] + 12.0), LINK_R * 1.4)
        prox.append(trimesh.util.concatenate([arc_mesh(m.u[i], V_h[i], rad=R_LEG[i]), stalk]))       # reference: theta_home
        dist_l.append(arc_mesh(V_h[i], W_h[i], rad=R_LEG[i]))                                         # reference: home orientation
    plat_parts = [trimesh.creation.uv_sphere(radius=28, count=[16, 16])]
    for i in range(3): plat_parts.append(seg([0, 0, 0], m.w0[i] * R_LEG[i], LINK_R * 1.2))
    plat_parts.append(trimesh.creation.cylinder(radius=36, height=113, sections=32).apply_translation([0, 0, 56.5]))
    plat = trimesh.util.concatenate(plat_parts)
    import arm_kin as ak
    G = ak.groups(g_pct=50.0); arm = trimesh.util.concatenate(list(G["M3"].values()))
    far = {g: trimesh.util.concatenate([mm for mm in G[g].values()]) for g in ("ME", "MW1", "MW2", "MW3")} if forearm else {}
    return m, dict(th_h=th_h, V_h=V_h, W_h=W_h, Rhome=Rhome), mot, prox, dist_l, plat, arm, far
def export_park(x, tag):
    os.makedirs(OUT, exist_ok=True); m, h, mot, prox, dist_l, plat, arm, far = build(x)
    for i in range(3):
        mot[i].export(os.path.join(OUT, f"ak70_proxy_{tag}_{i+1}.stl")); prox[i].export(os.path.join(OUT, f"link_prox_{tag}_{i+1}.stl")); dist_l[i].export(os.path.join(OUT, f"link_dist_{tag}_{i+1}.stl"))
    pp = plat.copy(); pp.apply_transform(T4(h["Rhome"])); pp.export(os.path.join(OUT, f"platform_{tag}_park.stl"))
    full = trimesh.util.concatenate(mot + prox + dist_l + [pp]); full.export(os.path.join(OUT, f"assembly_{tag}_park.stl"))
def check(x, step=10.0, forearm=False, tag="best"):
    t0 = time.time(); world = load_world(); m, h, mot, prox, dist_l, plat, arm, far = build(x, forearm)
    fixed = {k: FObj(v) for k, v in world.items()}; M = [FObj(mm) for mm in mot]
    gate = dict(skin=GATE_SKIN, head=GATE_HEAD, plate=GATE_PART, clasp=GATE_PART, pauldron=GATE_PART)
    res = dict(tag=tag, R_LEG=R_LEG, flange=[r + 12.0 for r in R_LEG])
    # ---- static: motors vs fixed structure
    st = {}
    for i in range(3):
        st[f"motor{i+1}"] = {k: round(dist(M[i], fixed[k]), 2) for k in fixed}
        st[f"motor{i+1}"]["centre_xyz"] = (m.u[i] * (R_LEG[i] + 12.0 + AK_L / 2)).round(1).tolist()
        st[f"motor{i+1}"]["axis"] = m.u[i].round(3).tolist()
    for i in range(3):
        for j in range(i + 1, 3): st[f"motor{i+1}_vs_motor{j+1}"] = round(dist(M[i], M[j]), 2)
    res["static_motor_distances_mm"] = st
    # ---- over the serial set
    q, R = K.serial_set(step); th, V, W, viol = m.ik(R); ok = viol.max(axis=1) < 1e-6
    q = q[ok]; R = R[ok]; th = th[ok]; V = V[ok]; W = W[ok]; N = len(q)
    P = FObj(plat); A = FObj(arm); PR = [FObj(p) for p in prox]; DL = [FObj(d) for d in dist_l]; FA = {g: FObj(mm) for g, mm in far.items()}
    names = ["platform", "arm_upper", "prox1", "prox2", "prox3", "dist1", "dist2", "dist3"] + list(far)
    objs = [P, A] + PR + DL + list(FA.values())
    fixed_all = {**fixed, **{f"motor{i+1}": M[i] for i in range(3)}}
    mind = {n: {k: 1e9 for k in fixed_all} for n in names}; worst = {n: {k: None for k in fixed_all} for n in names}
    cnt = {n: {k: 0 for k in fixed_all} for n in names}
    selfd = {}                                                                          # moving-moving pairs (links vs links/arm/platform)
    ia = [(0, 1)]  # platform-arm are rigidly joined: excluded
    pair_list = [(a, b) for a in range(len(names)) for b in range(a + 1, len(names))]
    skip = {("platform", "arm_upper")}
    for n_ in range(N):
        Rp = R[n_]; T_p = T4(Rp)
        P.set(T_p); A.set(T_p)
        for i in range(3):
            PR[i].set(T4(rotmat_about(m.u[i], th[n_, i] - h["th_h"][i])))
            DL[i].set(T4(frame_vw(V[n_, i], W[n_, i]) @ frame_vw(h["V_h"][i], h["W_h"][i]).T))
        # distal link rides with the platform end: rigid-body from (v,w): same construction
        if forearm:
            import arm_kin as ak
            Tq = ak.frames(dict(S1=q[n_, 0], S2=q[n_, 1], S3=q[n_, 2], E=forearm_E, W1=0, W2=0, W3=0, G=50))
            for g, o in FA.items(): o.set(Tq[g])
        for ni, nm in enumerate(names):
            for k, fo in fixed_all.items():
                if k.startswith("motor") and nm.startswith("prox") and nm[-1] == k[-1]: continue          # own motor: designed contact
                if k == "skin" and nm.startswith(("prox", "dist")) : pass
                d = dist(objs[ni], fo)
                if d < mind[nm][k]: mind[nm][k] = d; worst[nm][k] = q[n_].tolist()
                if d < gate.get(k, GATE_PART): cnt[nm][k] += 1
        for a, b in pair_list:
            na, nb = names[a], names[b]
            if (na, nb) in skip: continue
            if na.startswith("prox") and nb.startswith("dist") and na[-1] == nb[-1]: continue           # own leg: joined at v
            if na.startswith("dist") and nb == "platform" and na[-1] != "": pass
            key = f"{na}|{nb}"; d = dist(objs[a], objs[b]); selfd.setdefault(key, [1e9, None, 0])
            if d < selfd[key][0]: selfd[key][0] = d; selfd[key][1] = q[n_].tolist()
            if d < GATE_PART: selfd[key][2] += 1
    res["n_poses"] = N
    res["moving_vs_fixed_min_mm"] = {n: {k: round(v, 2) for k, v in mind[n].items()} for n in names}
    res["moving_vs_fixed_n_below_gate"] = cnt
    res["moving_vs_fixed_worst_pose"] = worst
    res["moving_vs_moving"] = {k: dict(min_mm=round(v[0], 2), worst=v[1], n_below_3mm=v[2]) for k, v in selfd.items()}
    res["gates_mm"] = gate; res["seconds"] = round(time.time() - t0, 1)
    json.dump(res, open(os.path.join(OUT, f"pack_result_{tag}{'_forearm' if forearm else ''}.json"), "w"), indent=1)
    return res
forearm_E = 0.0
if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument("src"); ap.add_argument("--step", type=float, default=10.0); ap.add_argument("--forearm", type=float, default=-1.0); ap.add_argument("--tag", default="best")
    a = ap.parse_args(); d = json.load(open(a.src)); x = np.array(d["x"] if "x" in d else d["design"]["x"])
    export_park(x, a.tag)
    if a.forearm >= 0: forearm_E = a.forearm
    r = check(x, a.step, a.forearm >= 0, a.tag)
    print("static", json.dumps(r["static_motor_distances_mm"])); print("N", r["n_poses"], "s", r["seconds"])
    for n_, d_ in r["moving_vs_fixed_min_mm"].items(): print("min", n_, d_, "below gate", {k: v for k, v in r["moving_vs_fixed_n_below_gate"][n_].items() if v})
    print("mov-mov", {k: (v["min_mm"], v["n_below_3mm"]) for k, v in r["moving_vs_moving"].items() if v["n_below_3mm"]})
