"""Straight-arm head/body keep-out map. For every (S1, S2) on a 2.5 deg grid: minimum distance between a straight arm
capsule (P -> fingertip + 15 mm, radius 32 mm: upper arm + forearm + hand fully extended, worst case; rev 2026-10-13 the
capsule followed the old 500 mm tip, it now reads Z_TIP 575 from stl/distal/distal.json) and the wearer mesh (incl. head
and neck). Used by rom_limits (passive box check) and control_map (mesh-based keep-out). Output stl/shoulder/keepout_map.npz."""
import numpy as np, trimesh, os, sys, json
from trimesh.transformations import rotation_matrix as rot
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sweep_shoulder as ss, rom_limits as rl
body = trimesh.load(os.path.join(HERE, "..", "stl", "shoulder", "proxies", "wearer_makehuman.stl"))
_lay = json.load(open(os.path.join(HERE, "..", "stl", "distal", "distal.json")))["_layout"]
CAP_R, CAP_L = 32.0, float(_lay["Z_TIP"]) + 15.0
mB = ss.mgr({"b": body}); cap = trimesh.creation.capsule(height=CAP_L - 2 * CAP_R, radius=CAP_R); cap.apply_translation([0, 0, CAP_L / 2]); mA = ss.mgr({"a": cap})
S1 = np.arange(-30.0, 155.1, 2.5); S2 = np.arange(-70.0, 103.1, 2.5); C = np.zeros((len(S1), len(S2)))
for i, a in enumerate(S1):
    for j, b in enumerate(S2):
        mA.set_transform("a", rot(np.radians(-a), [0, 0, 1]) @ rot(np.radians(-b), [0, 1, 0])); C[i, j] = mA.min_distance_other(mB)
np.savez(os.path.join(HERE, "..", "stl", "shoulder", "keepout_map.npz"), S1=S1, S2=S2, C=C)
(a0, a1), (b0, b1) = rl.SOFT["S1"], rl.SOFT["S2"]
(h0, h1), (g0, g1) = rl.HARD["S1"], rl.HARD["S2"]
soft = (S1[:, None] >= a0) & (S1[:, None] <= a1) & (S2[None] >= b0) & (S2[None] <= b1)
hard = (S1[:, None] >= h0) & (S1[:, None] <= h1) & (S2[None] >= g0) & (S2[None] <= g1)
ix = np.unravel_index(np.where(soft, C, 1e9).argmin(), C.shape)
print("capsule length", CAP_L, "radius", CAP_R, "| soft-box minimum at S1 %.1f S2 %.1f" % (S1[ix[0]], S2[ix[1]]))
print(json.dumps(dict(min_clearance_soft_box_mm=round(float(C[soft].min()), 1), min_clearance_hard_box_mm=round(float(C[hard].min()), 1)), indent=1))
