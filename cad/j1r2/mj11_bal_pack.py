"""MJ11 balancer option (a): refit per anchor sector + packaging census (2026-11-03, CONCEPT).  Robot frame, mm.
1. DE refit of the world-anchored zero-length spring (A on the pad, B on the upper arm) over the MJ11 soft free region, anchor pinned to a sector psi (deg from +y toward +z about the S1' axis), cord force capped.
2. For each solution: cord clearance over the soft region (skin, S1' group, arm, fixed MJ11 parts) and tube homes:
     T_rad  : radial tube from A (the J1 rev 2 geometry);  T_down : vertical tube below A on the lower back at y_t (cord redirected by a 2nd sheave, +friction PLACEHOLDER);
   tube D44.5 x t2.5, length from spring_design (2:1).  Gates: part-part 3 mm, skin 5 mm (MJ11 rail rule), mirror tube 3 mm.
usage: J1R2_ROOT=<root> python3 mj11_bal_pack.py  -> stl/j1r2_mounts/MJ11/balancer_pack.json"""
import os, sys, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import mj11_balancer as MB, mj_eval as E, mj_concepts as C, mj_concepts3 as C3, mj_frame as M, j1r2_params as P, j1r2_geom as G, clib as L
t0 = time.time(); log = lambda *a: print("[%4.0fs]" % (time.time() - t0), *a, flush=True)
TD, TT = 44.5, 2.5
FIXED = {k: p["mesh"] for k, p in C3.mj11_geo(None).items()}
FIX_PTS = np.vstack([E.samp(m, 2.5) for k, m in FIXED.items() if k != "liner"])
FIX_TREE = cKDTree(FIX_PTS)
SOFT_CELLS = [(i, j) for i in range(len(MB.S1V)) for j in range(len(MB.S2V)) if MB.SOFT[i, j]]
def skin_d(pts):
    d, i = E.TK.query(pts); return np.sign(np.einsum("ij,ij->i", pts - E.TP[i], E.TN[i])) * d
def tube_pts(s0, u, length):
    u = np.asarray(u, float) / np.linalg.norm(u); n = int(length / 4) + 2; ax = np.asarray(s0) + np.outer(np.linspace(0, length, n), u)
    e1 = np.cross(u, [1, 0, 0]); e1 = e1 / np.linalg.norm(e1) if np.linalg.norm(e1) > 1e-6 else np.array([0, 1.0, 0]); e2 = np.cross(u, e1); ring = [TD / 2 * (math.cos(a) * e1 + math.sin(a) * e2) for a in np.linspace(0, 2 * math.pi, 16, endpoint=False)]
    return np.vstack([ax + r for r in ring]), ax
def census_tube(s0, u, length, near_A=None):
    pts, ax = tube_pts(s0, u, length); res = {}
    res["skin_mm"] = round(float(skin_d(pts).min()), 1)
    mir = pts.copy(); mir[:, 1] = 2 * M.SPINE_Y - mir[:, 1]; res["mirror_tube_mm"] = round(float(cKDTree(pts).query(mir)[0].min()), 1)
    d = FIX_TREE.query(pts)[0]
    if near_A is not None: d = d[np.linalg.norm(pts - near_A, axis=1) > 14.0] if (np.linalg.norm(pts - near_A, axis=1) > 14.0).any() else d
    res["fixed_mj11_parts_mm"] = round(float(d.min()), 1)
    for nm, T in E.HARN.items(): res[nm + "_mm"] = round(float(T.query(pts, distance_upper_bound=40)[0].min()), 1)
    T = cKDTree(pts); b1 = b2 = 1e9
    for (i, j) in SOFT_CELLS[::2]:
        th, ph = MB.S1V[i], MB.S2V[j]
        if j == SOFT_CELLS[0][1] or True:
            Mx = G.pose_matrix("M1", E.DER, th, 0.0); b1 = min(b1, float(T.query(E.MOV1 @ Mx[:3, :3].T + Mx[:3, 3], distance_upper_bound=60)[0].min()))
            Mm = G.pose_matrix("M2", E.DER, th, ph); b2 = min(b2, float(T.query(E.MOV2 @ Mm[:3, :3].T + Mm[:3, 3], distance_upper_bound=60)[0].min()))
    res["S1p_group_mm"] = round(min(b1, 60.0), 1); res["arm_sweep_mm"] = round(min(b2, 60.0), 1)
    res["top_z"] = round(float(pts[:, 2].max()), 0); res["bottom_z"] = round(float(pts[:, 2].min()), 0); res["head_mm"] = E.head_clear([trimesh.Trimesh(*[pts, []], process=False)]) if False else round(float(cKDTree(E.HEAD).query(pts)[0].min()), 0)
    res["pass"] = bool(res["skin_mm"] >= 5 and res["mirror_tube_mm"] >= 3 and res["S1p_group_mm"] >= 3 and res["arm_sweep_mm"] >= 3 and res["fixed_mj11_parts_mm"] >= 3 and res["wing_rail_mm"] >= 3 and res["spine_bridge_mm"] >= 3)
    return res
def cord_census(x):
    A = MB.P1 + np.array(x[0:3]); Bl = np.array(x[3:6]); worst = dict(skin=1e9, S1=1e9, arm=1e9, fixed=1e9)
    for (i, j) in SOFT_CELLS:
        th, ph = MB.S1V[i], MB.S2V[j]; F0, FA, s, a, d = P.shoulder_frames(MB.B.D0, th, ph); Bw = (FA @ np.r_[Bl, 1.0])[:3]; seg = A + np.outer(np.linspace(0, 1, 30), Bw - A)
        worst["skin"] = min(worst["skin"], float(skin_d(seg[1:]).min()))
        keep = np.linalg.norm(seg - A, axis=1) > 14.0; worst["fixed"] = min(worst["fixed"], float(FIX_TREE.query(seg[keep])[0].min()) if keep.any() else 99)
        Mx = G.pose_matrix("M1", E.DER, th, 0.0); q1 = E.MOV1 @ Mx[:3, :3].T + Mx[:3, 3]; worst["S1"] = min(worst["S1"], float(cKDTree(q1).query(seg[2:])[0].min()))
        Mm = G.pose_matrix("M2", E.DER, th, ph); q2 = E.MOV2 @ Mm[:3, :3].T + Mm[:3, 3]; far = np.linalg.norm(q2 - Bw, axis=1) > 18.0; worst["arm"] = min(worst["arm"], float(cKDTree(q2[far]).query(seg[1:-3])[0].min()))
    return {k: round(v, 1) for k, v in worst.items()}
if __name__ == "__main__":
    pr = MB.make_problem(MB.SOFT); out = []
    for fcap in (130.0, 90.0):
        for psi in (100, 110, 120, 130, 140, 150):
            r = MB.refit(pr, psi, fcap, rlim=(70.0, 100.0)); r["fcap"] = fcap; r["spring"] = MB.spring_design(r["x"][6], r["cord_min"], r["cord_max"], r["F_cord_max"], L0=r["x"][7])
            A = MB.P1 + np.array(r["x"][0:3]); a = math.radians(psi); r["A_world"] = [round(float(v), 1) for v in A]
            r["cord_census"] = cord_census(r["x"]); u = np.array([0.0, math.cos(a), math.sin(a)]); L_t = r["spring"]["tube_len_mm"]
            r["tube_radial"] = census_tube(A + u * 6.0, u, L_t, near_A=A)
            tdn = {}
            for yt in (-128.0, -134.0, -140.0):
                xs = max(float(M.back_x(yt, z)) for z in np.arange(A[2] - 230, A[2] + 1, 20)) + 8.0 + TD / 2
                tdn[str(yt)] = dict(x_axis=round(xs, 1), **census_tube(np.array([xs, yt, A[2] - 20.0]), [0, 0, -1.0], L_t))
            r["tube_down"] = tdn; out.append(r)
            log("fcap", fcap, "psi", psi, "res", round(r["S1p_res"], 2), round(r["S2_res"], 2), "F", round(r["F_cord_max"], 1), "cord", r["cord_census"], "rad", {k: r["tube_radial"][k] for k in ("skin_mm", "arm_sweep_mm", "S1p_group_mm", "top_z", "pass")}, "down", {y: (v["skin_mm"], v["arm_sweep_mm"], v["pass"]) for y, v in tdn.items()})
    json.dump(out, open(os.path.join(MB.OUTD, "balancer_pack.json"), "w"), indent=1, default=float); log("done")
