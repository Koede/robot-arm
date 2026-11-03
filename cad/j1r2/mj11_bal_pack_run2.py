"""MJ11 balancer packaging scan (2026-11-03): where can the J1 rev 2 pad-anchored spring tube (R22.25 + cap R28, anchor A on the pad rim) live on the LOWER BACK for pad (-60, -190)?  Scans tube direction (angle in the yz plane from +y toward +z)
and tackle ratio (tube length 230 / 175 / 151 mm at 2:1 / 3:1 / 4:1, SPEC_j1r2_balancer section 3).  Checks: skin gap by zone (mj_eval rules rail 5, blade 5 (MJ11), top 10), head clearance + neck ROM, wing rail / spine bridge, MJ11 parts,
arm-group (M1 yoke + S2 motor, M2 arm tube) sweep clearance over the passive-stop box +-5 deg and the reach-free cells, cord vs M1 (A -> B).  usage: J1R2_ROOT=<root> python3 mj11_bal_pack.py [x-offset-json]  -> stl/j1r2_mounts/MJ11/bal_pack.json"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import mj_eval as E, mj_frame as M, mj_concepts as C, mj_concepts3 as C3, j1r2_geom as G, j1r2_params as P, clib as L
A_OFF = np.array([-10.018, -19.708, 73.434]); B_LOC = np.array([-15.629, -42.256, 118.385]); DER = E.DER; P1 = np.array([DER["PX"], DER["y1"], DER["z1"]])
mask = json.load(open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "workspace_mask.json")))
BOXH = [(th, ph) for th in np.arange(-100.0, 10.1, 5.0) for ph in list(np.arange(-20.0, 20.1, 5.0)) + [22.4]]                                   # passive-stop box (-95..5, -15..17.4) + 5 deg hard-stop expansion
REACH = [(mask["S1p"][i], mask["S2"][j]) for i in range(len(mask["S1p"])) for j in range(len(mask["S2"])) if mask["reach"][i][j]]
def tube_parts(A, u, Lt):
    u = u / np.linalg.norm(u); tube = trimesh.creation.cylinder(22.25, segment=[A, A + u * (Lt + 6.0)], sections=24); cap = trimesh.creation.cylinder(28.0, segment=[A - u * 14.0, A], sections=24)
    post = trimesh.creation.cylinder(13.0, segment=[np.array([DER["XB"], A[1], A[2]]), np.array([A[0] - 14.0, A[1], A[2]])], sections=16)
    return tube, cap, post
def parts_dict(tube, cap, post): return {"tube": dict(mesh=tube, kind="printed", mat="PETGCF"), "cap": dict(mesh=cap, kind="printed", mat="PETGCF"), "post": dict(mesh=post, kind="printed", mat="PETGCF")}
MJ = C3.mj11_geo(None); MJT = {k: cKDTree(E.samp(p["mesh"], 1.5)) for k, p in MJ.items()}
_POSED = {}
def posed(cells, key):
    if key in _POSED: return _POSED[key]
    out = []
    for th, ph in cells:
        Mx = G.pose_matrix("M1", DER, th, 0.0); Mm = G.pose_matrix("M2", DER, th, ph); F0, FA, s, a, d = P.shoulder_frames(DER, th, ph); B = (FA @ np.r_[B_LOC, 1.0])[:3]
        out.append((th, ph, E.MOV1[::2] @ Mx[:3, :3].T + Mx[:3, 3], E.MOV2[::2] @ Mm[:3, :3].T + Mm[:3, 3], B))
    _POSED[key] = out; return out
def arm_clear(pts, cells, key):
    T = cKDTree(pts); b1 = (1e9, None); b2 = (1e9, None)
    for th, ph, p1, p2, B in posed(cells, key):
        d1 = float(T.query(p1, distance_upper_bound=60)[0].min())
        if d1 < b1[0]: b1 = (d1, (th, ph))
        d2 = float(T.query(p2, distance_upper_bound=60)[0].min())
        if d2 < b2[0]: b2 = (d2, (th, ph))
    return dict(M1_min=round(min(b1[0], 60), 1), M1_at=b1[1], M2_min=round(min(b2[0], 60), 1), M2_at=b2[1])
def cord_clear(A, cells, key):
    best = (1e9, None); w = np.linspace(0.05, 0.95, 40)[:, None]
    for th, ph, p1, p2, B in posed(cells, key):
        q = A[None] + (B - A)[None] * w; d_ = float(cKDTree(p1).query(q)[0].min())
        if d_ < best[0]: best = (d_, (th, ph))
    return dict(cord_vs_M1_min=round(best[0], 1), at=best[1])
def evaluate(A, ang, Lt, cells_arm=(("box_hard", BOXH), ("reach", REACH))):
    u = np.array([0.0, math.cos(math.radians(ang)), math.sin(math.radians(ang))]); tube, cap, post = tube_parts(A, u, Lt); pd = parts_dict(tube, cap, post); pts = np.vstack([E.samp(m, 2.0) for m in (tube, cap, post)])
    sk = E.skin_zones(pd); rule = dict(E.RULE, blade=5.0); viol = [f"{k}:{z} {v}<{rule[z]}" for k, zz in sk.items() for z, v in zz.items() if z in rule and v < rule[z]]
    hc = E.head_clear([tube, cap, post]); hw = {h: round(float(T.query(pts, distance_upper_bound=30)[0].min()), 1) for h, T in E.HARN.items()}; tp = E.samp(tube, 2.0); mjd = {k: round(float(T.query(tp, distance_upper_bound=30)[0].min()), 1) for k, T in MJT.items()}          # tube only (cap + post are the designed attachment to the pad rim)
    rom = {k: v for k, v in E.wearer_rom(pd).items() if v is not None and k.startswith("neck")} if (not viol and hc >= 60) else {"skipped": "stage-1 fail"}
    ac = {n: arm_clear(pts, c, n) for n, c in cells_arm}; cc = cord_clear(A, BOXH, "box_hard")
    return dict(angle=ang, L_tube=Lt, skin=sk, skin_violations=viol, head_mm=hc, neck_rom_first_contact=rom, harness=hw, mj11_parts=mjd, arm=ac, cord=cc, tube_end=[round(float(v), 0) for v in (A + u * Lt)], z_span=[round(float(pts[:, 2].min())), round(float(pts[:, 2].max()))], y_min=round(float(pts[:, 1].min())))
def main():
    off = A_OFF if len(sys.argv) < 2 else np.array(json.loads(sys.argv[1])); A = P1 + off; out = dict(A_world=A.tolist(), A_off=off.tolist(), rows=[])
    for Lt in (230.0, 175.0, 151.0):
        for ang in range(0, 360, 15):
            r = evaluate(A, ang, Lt); out["rows"].append(r)
            ok = (not r["skin_violations"]) and r["head_mm"] >= 60 and not r["neck_rom_first_contact"] and min(r["harness"].values()) >= 3 and min(r["mj11_parts"].values()) >= 3 and r["arm"]["box_hard"]["M1_min"] >= 3 and r["arm"]["box_hard"]["M2_min"] >= 3 and r["cord"]["cord_vs_M1_min"] >= 3
            r["pass_box"] = bool(ok); r["pass_reach"] = bool(ok and r["arm"]["reach"]["M1_min"] >= 3 and r["arm"]["reach"]["M2_min"] >= 3)
            print(Lt, ang, "box" if r["pass_box"] else "-", "reach" if r["pass_reach"] else "-", "viol", r["skin_violations"][:2], "head", r["head_mm"], "M1", r["arm"]["box_hard"]["M1_min"], "M2", r["arm"]["box_hard"]["M2_min"], "cord", r["cord"]["cord_vs_M1_min"], "harn", r["harness"], "mj", min(r["mj11_parts"].values()), flush=True)
    json.dump(out, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "bal_pack.json"), "w"), indent=1, default=float)
if __name__ == "__main__": main()
