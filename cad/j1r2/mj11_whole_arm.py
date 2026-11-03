"""MJ11 whole-arm first-contact sweep (S1p,S2,S3,E,W1,W2,W3; HAND rev B/C proxies from j1r2_kin) against the real MJ11 fixed parts (pad, seat dish, liner, spacers, clasp bracket @ CLASP_PSI),
the rail-only wing and the wearer incl. head/neck (60 mm).  Same scene builder as mj11_sweep.scene, joints_upto=W3; tier grid as j1r2_integrate full mode.
usage: J1R2_ROOT=<work root> python3 mj11_whole_arm.py [full|restricted|straight|shoulder5]  -> stl/j1r2_sweep/sweep_<label>.json, tables_<label>.npz"""
import os, sys, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import clib as L, j1r2_geom as G, j1r2_kin as K, j1r2_sweep as S, mj_frame as M, mj_concepts3 as C3
from mj11_sweep import REPLACED
STEPS = {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}
STEPS_RESTRICT = {"S1p": 10.0, "S2": 10.0, "S3": [-30.0, 0.0, 30.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-30.0, 0.0, 30.0], "W3": [-15.0, 0.0, 15.0]}   # RESTRICT tier box (control.h R_S3 30, R_W2 30, R_W3 15; W1 free)
STEPS_STRAIGHT = {"S1p": 2.5, "S2": 2.5, "S3": [0.0], "E": [0.0], "W1": [0.0], "W2": [0.0], "W3": [0.0]}    # straight-arm clearance map (firmware keep-out table)
STEPS_SH5 = {"S1p": 5.0, "S2": 5.0, "S3": [-70.0, 0.0, 90.0], "E": [0.0], "W1": [0.0], "W2": [0.0], "W3": [0.0]}                  # shoulder-level 5 deg grid (hard stops sit 5 deg beyond soft limits)
MODES = {"shoulder5": (STEPS_SH5, "mj11_shoulder5"), "full": (STEPS, "mj11_whole_arm"), "restricted": (STEPS_RESTRICT, "mj11_whole_arm_restricted"), "straight": (STEPS_STRAIGHT, "mj11_straight_arm")}
def scene(mj_parts, name="MJ11_whole"):
    parts, info = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True); D = info["derived"]; fixed, m1, m2 = {}, [], []
    for k, p in parts.items():
        g = p["group"]
        if p.get("kind") == "cable" or k.startswith("cable") or k in REPLACED: continue
        if g == "F": fixed[k] = p["mesh"]
        elif g == "M1": m1.append(p["mesh"])
        elif g == "M2": m2.append(p["mesh"])
    for k, p in mj_parts.items(): fixed["mj11_" + k] = p["mesh"]
    w = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); fixed["wing"] = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100))
    sh = dict(M1=L.U(*m1), M2=L.U(*m2)); return S.scene_from_kin(sh, fixed, S.make_wearer_parts(), kin=K.Kin(D), name=name, joints_upto="W3"), info
if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "full"; steps, label = MODES[mode]; t0 = time.time()
    sc, info = scene(C3.mj11_geo(None)); print("scene", round(time.time() - t0, 1), "s fixed", list(sc.fixed), "groups", list(sc.groups), "clasp psi", C3.CLASP_PSI, "pad", [M.Y1, M.Z1, M.DER["standoff"]], flush=True)
    out = S.run_sweep(sc, dict(steps=steps, procs=2, label=label)); print(S.table(out)); print("timing", out["timing"])
