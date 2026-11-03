"""MJ11 clasp bracket search (2026-11-03, SHOULDER/CLASP).  Gates for the bracket (feet F5/F4 -> hub F1 -> pad rim): >= 3 mm to the arm sweep over the passive box +-5 deg hard-stop
expansion (target; < 1 mm failure), >= 3 mm to wing rail and spine bridge, >= 5 mm skin gap (MJ11 rule).  Feet positions and the ~270 deg clasp coverage are NOT changed.
usage: J1R2_ROOT=<root> python3 mj11_clasp_search.py [baseline|search]"""
import os, sys, json, math, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import clib as L, mj_frame as M, mj_concepts as C, mj_concepts3 as C3, mj_eval as E, j1r2_geom as G
from clib import U
BOX = dict(S1p=(-95.0, 5.0), S2=(-15.0, 17.4)); HARD = 5.0                      # passive box of mj11 sweep.json; +-5 deg hard-stop expansion
TH = np.arange(BOX["S1p"][0] - HARD, BOX["S1p"][1] + HARD + 0.1, 2.5); PH = np.arange(BOX["S2"][0] - HARD, BOX["S2"][1] + HARD + 0.1, 2.5)
def sweep_cloud():
    """arm (M2) and S1' group (M1) sample points swept over the expanded passive box."""
    c2 = []; c1 = []
    for th in TH:
        Mx = G.pose_matrix("M1", E.DER, th, 0.0); c1.append(E.MOV1 @ Mx[:3, :3].T + Mx[:3, 3])
        for ph in PH:
            Mm = G.pose_matrix("M2", E.DER, th, ph); c2.append(E.MOV2 @ Mm[:3, :3].T + Mm[:3, 3])
    return cKDTree(np.vstack(c1)), cKDTree(np.vstack(c2))
T1, T2 = sweep_cloud()
def bracket(psi, w, t, via=None):
    return C.clasp_bracket_geo(C.pad_pt(psi, C.PAD_R - 8, x=C.XB - 3.0), via=via, w=w, t=t)      # foot bars stay 16 x 6; only hub -> anchor bar changes
def metrics(m):
    s = E.samp(m, 1.5); d1 = float(T1.query(s, distance_upper_bound=40)[0].min()); d2 = float(T2.query(s, distance_upper_bound=40)[0].min())
    h = {k: float(T.query(s, distance_upper_bound=30)[0].min()) for k, T in E.HARN.items()}
    sk = E.skin_zones({"b": dict(mesh=m, kind="printed")})["b"]
    return dict(arm=round(d2, 2), M1=round(d1, 2), rail=round(h["wing_rail"], 2), bridge=round(h["spine_bridge"], 2), skin=sk)
if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "baseline"
    if mode == "baseline":
        m = bracket(C3.CLASP_PSI, 24.0, 6.0); print(metrics(m), "watertight", m.is_watertight, "vol", round(m.volume))
def margin(r):
    sk = min([v for z, v in r["skin"].items() if z in ("rail", "blade")] or [99]) - 5.0
    return min(r["M1"] - 3.0, r["arm"] - 3.0, r["rail"] - 3.0, r["bridge"] - 3.0, sk)
def search():
    rows = []
    for psi in np.arange(105, 140.1, 2.5):
        for w in range(14, 25, 2):
            for t in (4.0, 5.0, 6.0):
                try: r = metrics(bracket(psi, float(w), t))
                except Exception as e: continue
                r.update(psi=float(psi), w=w, t=t, margin=round(margin(r), 2)); rows.append(r)
    rows.sort(key=lambda r: -r["margin"]); json.dump(rows, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "clasp_search.json"), "w"), indent=0, default=float)
    for r in rows[:15]: print(r)
if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "search": search()
