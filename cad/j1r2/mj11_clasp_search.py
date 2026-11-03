"""MJ11 clasp bracket search (2026-11-03, SHOULDER/CLASP).  Bars: F5/F4 -> hub F1 (16x6, unchanged feet) then hub -> [via] -> pad-rim anchor (width w x thickness t).
Gates: >= 3 mm to the arm sweep (M1 + M2 groups) over the passive box +-5 deg hard-stop expansion (S1p -100..10, S2 -20..22.4), >= 3 mm to wing rail + spine bridge, >= 5 mm skin gap (MJ11 rule).
usage: J1R2_ROOT=<root> python3 mj11_clasp_search.py   -> stl/j1r2_mounts/MJ11/clasp_search.json"""
import os, sys, json, math, itertools, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.spatial import cKDTree
import mj_eval as E, mj_concepts as C, mj_concepts3 as C3, mj_frame as M, clib as L, j1r2_geom as G
BOX = dict(S1p=(-100.0, 10.0), S2=(-20.0, 22.4))
def samp(m, sp): return E.samp(m, sp)
def build(psi, w, t, via=None, r=None):
    return C.clasp_bracket_geo(C.pad_pt(psi, (C.PAD_R - 8) if r is None else r, x=C.XB - 3.0), via=via, w=w, t=t)
_MV = {}
def movers(sp=1.5):
    if sp not in _MV:
        m1 = np.vstack([samp(E.BASE[k]["mesh"], sp) for k in E.M1N]); m2 = np.vstack([samp(E.BASE[k]["mesh"], sp) for k in E.M2N]); _MV[sp] = (cKDTree(m1), cKDTree(m2))
    return _MV[sp]
def arm_clear(mesh, step=2.5, sp=1.5, mp=1.5):
    """min distance (mm) bracket -> M1 / M2 over BOX.  Distance is rigid-invariant: move the bracket by the inverse pose, query the fixed mover tree."""
    p = samp(mesh, mp); t1, t2 = movers(sp); b1 = b2 = 1e9; a1 = a2 = None
    for th in np.arange(BOX["S1p"][0], BOX["S1p"][1] + 1e-6, step):
        Mx = np.linalg.inv(G.pose_matrix("M1", E.DER, th, 0.0)); q = p @ Mx[:3, :3].T + Mx[:3, 3]; d = float(t1.query(q, distance_upper_bound=40)[0].min())
        if d < b1: b1, a1 = d, float(th)
        for ph in np.arange(BOX["S2"][0], BOX["S2"][1] + 1e-6, step):
            Mm = np.linalg.inv(G.pose_matrix("M2", E.DER, th, ph)); q = p @ Mm[:3, :3].T + Mm[:3, 3]; d = float(t2.query(q, distance_upper_bound=40)[0].min())
            if d < b2: b2, a2 = d, (float(th), float(ph))
    return min(b1, 40.0), a1, min(b2, 40.0), a2
def skin_min(mesh):
    s_ = samp(mesh, 1.5); d, i = E.TK.query(s_); sd = np.sign(np.einsum("ij,ij->i", s_ - E.TP[i], E.TN[i])) * d; return float(sd.min())
def evaluate(mesh, fast=False):
    a1, th1, a2, th2 = arm_clear(mesh, 5.0 if fast else 2.5)
    hc = {h: float(T.query(samp(mesh, 1.5), distance_upper_bound=30)[0].min()) for h, T in E.HARN.items()}
    sk = E.skin_zones({"b": dict(mesh=mesh, kind="printed")})["b"]
    return dict(arm=min(a1, a2), arm_M1=a1, arm_M2=a2, at=(th1, th2), rail=hc["wing_rail"], bridge=hc["spine_bridge"], skin=min(sk.values()), skin_zone=sk, vol=float(abs(mesh.volume)))
def ok(r, a=3.0, h=3.0, s=5.0): return r["arm"] >= a and r["rail"] >= h and r["bridge"] >= h and r["skin"] >= s
if __name__ == "__main__":
    t0 = time.time(); base = build(120.0, 24.0, 6.0); r0 = evaluate(base); print("baseline psi120 w24 t6:", {k: (round(v, 1) if isinstance(v, float) else v) for k, v in r0.items()}, "%.0fs" % (time.time() - t0), flush=True)
    rows = []
    for psi in (100, 110, 119, 130, 140):
        for w in (14, 18, 20, 24):
            for t in (6.0,):
                m = build(psi, w, t); r = evaluate(m, fast=True); r.update(psi=psi, w=w, t=t); rows.append(r)
                print(psi, w, t, "arm %.1f (M1 %.1f M2 %.1f @%s) rail %.1f bridge %.1f skin %.1f" % (r["arm"], r["arm_M1"], r["arm_M2"], r["at"][1], r["rail"], r["bridge"], r["skin"]), flush=True)
    json.dump(dict(baseline=r0, grid=rows), open(os.path.join(E.OUT, "MJ11", "clasp_search.json"), "w"), indent=1, default=float)
