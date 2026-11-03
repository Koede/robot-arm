"""MJ11 whole-arm first-contact sweep (2026-11-03): all joints S1p,S2,S3,E,W1,W2,W3 against the real MJ11 fixed parts, rail-only wing, clasp bracket (CLASP_PSI) and the wearer incl. head/neck.
Steps as j1r2_integrate full mode.  usage: J1R2_ROOT=<work root> python3 mj11_whole_arm.py  -> stl/j1r2_sweep/mj11_whole_arm.json (+ sweep_/tables_ files via j1r2_sweep.OUT)"""
import os, sys, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import clib as L, j1r2_params as P, j1r2_geom as G, j1r2_kin as K, j1r2_sweep as S, j1r2_opt as O, mj_frame as M, mj_concepts3 as C3
import mj11_sweep as W
STEPS = {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}
STEPS_R = {"S1p": 10.0, "S2": 10.0, "S3": [-30.0, 0.0, 30.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-30.0, 0.0, 30.0], "W3": [-15.0, 0.0, 15.0]}   # RESTRICT tier box (control.h R_S3 30, R_W2 30, R_W3 15)
def scene_full(mj_parts):
    parts, info = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True); D = info["derived"]; fixed, m1, m2 = {}, [], []
    for k, p in parts.items():
        g = p["group"]
        if p.get("kind") == "cable" or k.startswith("cable") or k in W.REPLACED: continue
        if g == "F": fixed[k] = p["mesh"]
        elif g == "M1": m1.append(p["mesh"])
        elif g == "M2": m2.append(p["mesh"])
    for k, p in mj_parts.items(): fixed["mj11_" + k] = p["mesh"]
    w = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); fixed["wing"] = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100))
    sh = dict(M1=L.U(*m1), M2=L.U(*m2)); return S.scene_from_kin(sh, fixed, S.make_wearer_parts(), kin=K.Kin(D), name="MJ11_whole", joints_upto="W3"), info
if __name__ == "__main__":
    t0 = time.time(); parts = C3.mj11_geo(None); sc, info = scene_full(parts); print("scene", round(time.time() - t0, 1), list(sc.fixed), list(sc.groups), flush=True)
    R = "--restricted" in sys.argv; out = S.run_sweep(sc, dict(steps=STEPS_R if R else STEPS, procs=2, label="mj11_whole_arm_restricted" if R else "mj11_whole_arm")); print(S.table(out), flush=True)
    ok, vals, worst = O.allowed_map(out)
    json.dump(dict(pad=[M.Y1, M.Z1, M.DER["standoff"]], clasp_psi=C3.CLASP_PSI, steps=STEPS_R if R else STEPS, worst=worst, timing=out["timing"]), open(os.path.join(M.REPO, "stl", "j1r2_sweep", "mj11_whole_arm%s_meta.json" % ("_restricted" if R else "")), "w"), indent=1, default=float)
    print("done", round(time.time() - t0, 1))
