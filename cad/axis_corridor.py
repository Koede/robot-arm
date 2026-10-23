"""Through-axis routing study: for each joint axis, which solids sit ON the axis (so a cable cannot pass along it) and how far the nearest solid is from the axis points elsewhere.
Axis points are fixed in the parent group's frame. Part identification at the park pose (trimesh contains), clearance over legal poses (SDF).  Usage: python3 axis_corridor.py"""
import os, sys, json, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_keepout as K, arm_wiring_lib as L
AX = {"S1": ("F", (0,0,0), (0,0,1)), "S2": ("M1", (0,0,0), (0,1,0)), "S3": ("M2", (0,0,0), (0,0,1)),
      "E": ("M3", (0,0,230), (0,1,0)), "W1": ("ME", (0,57,230), (0,0,1)), "W2": ("MW1", (0,57,410), (0,1,0)), "W3": ("MW2", (0,57,455), (1,0,0))}
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)
G = ak.groups(include_fingers=True, g_pct=50.0); F0 = ak.frames(PARK)
meshes = {(g, n): m for g, d in G.items() for n, m in d.items()}
FR, nl = K.legal_poses(K.frames_list(K.pose_list(60, 5)), 3.0); print("legal poses", nl)
out = {}
for j, (g, p0, d) in AX.items():
    p0 = np.array(p0, float); d = np.array(d, float)
    S = np.arange(-60, 60.1, 4.0); P = p0 + S[:, None] * d
    # parts containing each axis point at park
    Pw = (np.c_[P, np.ones(len(P))] @ F0[g].T)[:, :3]; hit = {}
    for (gg, n), m in meshes.items():
        Pl = (np.c_[Pw, np.ones(len(Pw))] @ np.linalg.inv(F0[gg]).T)[:, :3]
        try: c = m.contains(Pl)
        except Exception: continue
        for s, v in zip(S, c):
            if v: hit.setdefault(n, []).append(float(s))
    # clearance of each axis point over legal poses (all groups, own pose)
    clr = np.full(len(P), 1e9)
    for F in FR:
        Pw2 = (np.c_[P, np.ones(len(P))] @ F[g].T)[:, :3]
        v, _ = L.sdf_world(Pw2, F); clr = np.minimum(clr, v)
    blocked = {n: (min(v), max(v), len(v)) for n, v in hit.items()}
    free = S[clr >= 4.0]
    out[j] = dict(frame=g, blocked_by_part={n: [round(a), round(b), c] for n, (a, b, c) in blocked.items()}, axis_pts_free_ge4mm=[round(float(s)) for s in free],
                  max_free_run=0)
    # longest contiguous free run
    best = 0; cur = 0; bs = None; cs = None
    for s, c in zip(S, clr):
        if c >= 4.0: cur += 4; cs = s if cur == 4 else cs; 
        else: cur = 0
        if cur > best: best = cur; bs = cs
    out[j]["max_free_run_mm"] = best; out[j]["run_start"] = None if bs is None else float(bs)
    print(j, "frame", g, "| solids on axis at park (s range mm, n pts):", out[j]["blocked_by_part"], "| longest free (>=4 mm) run along axis: %d mm" % best, flush=True)
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "electronics", "axis_corridor.json"), "w"), indent=1)
