"""stall worst case (4 poses x 8 sign combos, caps 8.3/8.3/4.8) for the bolt layouts L0, L3, L6 (2026-11-03)"""
import os, sys, json, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mj11_loads as K, mj_frame as M, j1r2_params as P
poses = [(P.PARK["S1p"], P.PARK["S2"], 0.5), (-90.0, 0.0, 0.5), (-50.0, 17.4, 0.5), (-20.0, 35.0, 0.5)]; out = {}
for lab in [l for l in K.LAYOUTS if l[:2] in ("L0", "L3", "L6")]:
    w = 0; 
    for p in poses:
        for sg in itertools.product((-1, 1), repeat=3):
            r = K.solve_lc(lab, *p, stall=dict(S1=sg[0] * 8.3, S2=sg[1] * 8.3, S3=sg[2] * 4.8), dz=10.0); w = max(w, r["bolt_pair_tension_N"])
    out[lab] = round(w, 0); print(lab, w, flush=True)
json.dump(out, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "loads_stall_layouts.json"), "w"), indent=1)
