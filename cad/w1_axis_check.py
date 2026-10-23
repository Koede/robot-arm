"""W1 roll axis over its real length (z 230..410 at y=57 in the ME frame): solids on the axis at park, clearance over legal poses."""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_keepout as K, arm_wiring_lib as L
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50); F0 = ak.frames(PARK)
G = ak.groups(include_fingers=True, g_pct=50.0); meshes = {(g, n): m for g, d in G.items() for n, m in d.items()}
FR, nl = K.legal_poses(K.frames_list(K.pose_list(60, 5)), 3.0)
Z = np.arange(230, 411, 4.0); P = np.c_[np.zeros(len(Z)), np.full(len(Z), 57.0), Z]
Pw = (np.c_[P, np.ones(len(P))] @ F0["ME"].T)[:, :3]; hit = {}
for (g, n), m in meshes.items():
    Pl = (np.c_[Pw, np.ones(len(Pw))] @ np.linalg.inv(F0[g]).T)[:, :3]
    try: c = m.contains(Pl)
    except Exception: continue
    for z, v in zip(Z, c):
        if v: hit.setdefault(n, []).append(float(z))
print("solids on W1 axis (part: z range):", {n: (min(v), max(v)) for n, v in hit.items()})
clr = np.full(len(P), 1e9)
for F in FR:
    v, _ = L.sdf_world((np.c_[P, np.ones(len(P))] @ F["ME"].T)[:, :3], F); clr = np.minimum(clr, v)
print("min clearance over legal poses by z:"); 
for z, c in zip(Z[::5], clr[::5]): print(int(z), round(float(c), 1))
json.dump(dict(solids={n: [min(v), max(v)] for n, v in hit.items()}, z=Z.tolist(), clr=clr.tolist()), open("../stl/electronics/w1_axis.json", "w"))
