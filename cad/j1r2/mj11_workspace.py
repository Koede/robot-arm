"""MJ11 workspace masks (2026-11-03): reruns the mj11_sweep shoulder-level first-contact sweep (5 deg) and saves the collision-free map (ok), the reachable-from-park map (reach) and the
best passive-stop box over (S1', S2).  Used by mj11_loads.py / mj11_torque.py to evaluate loads and gravity torque over the REAL workspace instead of the J1 rev 2 grid.
usage: J1R2_ROOT=<work root> J1_PAD=-60,-190,12 python3 mj11_workspace.py   -> stl/j1r2_mounts/MJ11/workspace_mask.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import mj11_sweep as SW, j1r2_opt as O, mj_frame as M, mj_concepts3 as C3
if __name__ == "__main__":
    parts = C3.mj11_geo(None); r, out = SW.run(parts, {"S1p": 5.0, "S2": 5.0, "S3": [0.0]}); ok, vals, worst = O.allowed_map(out)
    reach = out["_internal"]["reach"]; reach = reach.reshape(reach.shape[0], reach.shape[1], -1).any(axis=2)
    okp = ok.reshape(ok.shape[0], ok.shape[1], -1).all(axis=2)
    d = dict(S1p=[float(v) for v in vals["S1p"]], S2=[float(v) for v in vals["S2"]], ok=okp.astype(int).tolist(), reach=reach.astype(int).tolist(), box=r["box"], reach_cov=r["reach_cov"], pad=r["pad"])
    json.dump(d, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "workspace_mask.json"), "w"), indent=0)
    print("S1p", d["S1p"][0], d["S1p"][-1], "S2", d["S2"][0], d["S2"][-1], "ok cells", int(okp.sum()), "reach cells", int(reach.sum()), "box", r["box"])
