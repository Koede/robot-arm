"""MJ11 shoulder-level passive-box sweep with S3 {-70,0,90} (5 deg S1p x S2) and the straight-arm clearance map (E=0, S3=0, wrist 0) on a 2.5 deg (S1p, S2) grid, both with the real MJ11 parts.
usage: J1R2_ROOT=<work root> python3 mj11_shoulder_ko.py -> stl/j1r2_sweep/mj11_shoulder_s3.json, keepout_map_mj11.npz"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, j1r2_sweep as S, j1r2_opt as O, mj_frame as M, mj_concepts3 as C3, mj11_sweep as W, mj11_whole_arm as WA
OUT = os.path.join(M.REPO, "stl", "j1r2_sweep")
parts = C3.mj11_geo(None)
r, out = W.run(parts, {"S1p": 5.0, "S2": 5.0, "S3": [-70.0, 0.0, 90.0]}, label="mj11_shoulder_s3")
json.dump(r, open(os.path.join(OUT, "mj11_shoulder_s3.json"), "w"), indent=1, default=float); print("shoulder S3 triple:", json.dumps({k: v for k, v in r.items()}, default=float)); print(S.table(out))
sc, info = WA.scene_full(parts)
st = {"S1p": 2.5, "S2": 2.5, "S3": [0.0], "E": [0.0], "W1": [0.0], "W2": [0.0], "W3": [0.0]}
o = S.run_sweep(sc, dict(steps=st, procs=2, label="mj11_straight_arm")); I = o["_internal"]; vals = I["vals"]; J = list(vals); GS = tuple(len(vals[j]) for j in J)
i0 = [int(np.argmin(abs(vals[j] - (0.0))) ) for j in J]; sel = tuple(0 if j != "S1p" and j != "S2" else slice(None) for j in J)
# E / S3 / W index 0 are the requested singleton values (park values are appended to the grid): pick the zero entries
sel = tuple(slice(None) if j in ("S1p", "S2") else int(np.argmin(abs(vals[j]))) for j in J)
n1, n2 = len(vals["S1p"]), len(vals["S2"]); Cskin = np.full((n1, n2), np.inf); Chead = Cskin.copy(); Cfix = Cskin.copy(); Cany = Cskin.copy()
for pi, (p, d) in enumerate(zip(I["pairs"], I["designed"])):
    t = np.asarray(I["tabs"][pi]); full = np.broadcast_to(S._bcast(t, p.dims, J), GS)[sel]
    if p.key.endswith("wearer_skin"): Cskin = np.minimum(Cskin, full)
    elif p.key.endswith("wearer_head"): Chead = np.minimum(Chead, full)
    elif not d and p.kind == "obj": Cfix = np.minimum(Cfix, full - p.MM)
    if not d: Cany = np.minimum(Cany, full - p.MM)
np.savez_compressed(os.path.join(OUT, "keepout_map_mj11.npz"), S1p=vals["S1p"], S2=vals["S2"], C_skin=Cskin, C_head=Chead, C_fixed_margin=Cfix, C_any_margin=Cany)
print("straight-arm map", Cskin.shape, "min skin", float(Cskin.min()), "min head", float(Chead.min()))
