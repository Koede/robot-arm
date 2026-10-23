"""Offset through-channel study: smallest radius r at which a line parallel to each joint axis (fixed in the parent frame) clears all solids by >=3 mm.
(a) park pose only (a hole through the housing); (b) over legal poses (child rotating through the line). Usage: python3 axis_offset.py"""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_keepout as K, arm_wiring_lib as L
from axis_corridor import AX, PARK, F0, FR
OUT = {}
for j, (g, p0, d) in AX.items():
    p0 = np.array(p0, float); d = np.array(d, float)
    a = np.cross(d, [1, 0, 0] if abs(d[0]) < .9 else [0, 1, 0]); a /= np.linalg.norm(a); b = np.cross(d, a)
    S = np.arange(-50, 50.1, 5.0); res = {}
    for r in range(4, 61, 4):
        okp = okq = 0
        for al in np.deg2rad(np.arange(0, 360, 15)):
            P = p0 + S[:, None] * d + r * (np.cos(al) * a + np.sin(al) * b)
            v, _ = L.sdf_world((np.c_[P, np.ones(len(P))] @ F0[g].T)[:, :3], F0)
            if v.min() >= 3.0:
                okp += 1; m = 1e9
                for F in FR[::2]:
                    vv, _ = L.sdf_world((np.c_[P, np.ones(len(P))] @ F[g].T)[:, :3], F); m = min(m, vv.min())
                    if m < 3.0: break
                if m >= 3.0: okq += 1
        res[r] = (okp, okq)
    rp = next((r for r, (p, q) in res.items() if p), None); rq = next((r for r, (p, q) in res.items() if q), None)
    OUT[j] = dict(r_min_park_hole=rp, r_min_all_poses=rq, table={str(k): v for k, v in res.items()})
    print(j, "r_min park-pose hole:", rp, "mm | r_min clear over legal poses:", rq, "mm", flush=True)
json.dump(OUT, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "electronics", "axis_offset.json"), "w"), indent=1)
