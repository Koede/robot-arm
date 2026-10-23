"""Render: arm at park pose, joint axes (green = free run, red = blocked by solids) and min through-channel circles. Output ../routing_axes.png"""
import os, sys, json, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak
from axis_corridor import AX, PARK, F0
cor = json.load(open("../stl/electronics/axis_corridor.json")); off = json.load(open("../stl/electronics/axis_offset.json"))
G = ak.groups(include_fingers=True, g_pct=50.0); M = []
for g, d in G.items():
    for n, m in d.items():
        mm = m.copy(); mm.apply_transform(F0[g]); M.append(mm)
fig, axs = plt.subplots(1, 2, figsize=(15, 8))
for ax_, (u, v, nm) in zip(axs, [(0, 2, "side (x-z)"), (1, 2, "back (y-z)")]):
    for mm in M:
        f = mm.faces[::3]; tri = mm.vertices[f][:, :, [u, v]]
        nz = np.cross(mm.vertices[f][:, 1] - mm.vertices[f][:, 0], mm.vertices[f][:, 2] - mm.vertices[f][:, 0])
        ax_.add_collection(PolyCollection(tri, facecolor="#c9ced6", edgecolor="none", alpha=0.55))
    for j, (g, p0, d) in AX.items():
        p0 = np.array(p0, float); d = np.array(d, float); S = np.arange(-60, 60.1, 4.0); fr = set(cor[j]["axis_pts_free_ge4mm"])
        Pw = (np.c_[p0 + S[:, None] * d, np.ones(len(S))] @ F0[g].T)[:, :3]
        for k in range(len(S) - 1):
            col = "#1a9850" if (round(S[k]) in fr and round(S[k + 1]) in fr) else "#d73027"
            ax_.plot(Pw[k:k + 2, u], Pw[k:k + 2, v], color=col, lw=3)
        c = Pw[len(S) // 2]; r = off[j]["r_min_all_poses"] or 60
        ax_.add_patch(plt.Circle((c[u], c[v]), r, fill=False, ec="#4575b4", ls="--", lw=1.2)); ax_.text(c[u] + 4, c[v] + r + 3, j, fontsize=10, weight="bold")
    B=np.vstack([m.vertices for m in M]); ax_.set_xlim(B[:,u].min()-40,B[:,u].max()+40); ax_.set_ylim(B[:,2].min()-30,B[:,2].max()+30); ax_.set_aspect("equal"); ax_.set_title(nm)
fig.suptitle("Joint axes: green = free for a through-cable, red = blocked by actuator; dashed = smallest clear offset-channel radius (mm)")
plt.tight_layout(); plt.savefig("../routing_axes.png", dpi=110)
