"""Shaded whole-arm renders (painter's algorithm), 4 views at park + 2 reach poses, with cable loops. Output ../whole_arm.png"""
import os, sys, json, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L
d = json.load(open("../stl/electronics/arm_chain_loops.json")); G = ak.groups(include_fingers=True, g_pct=50.0)
POSES = {"park": dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50),
         "reach forward": dict(S1=20, S2=70, S3=0, E=20, W1=0, W2=0, W3=0, G=100),
         "arm raised": dict(S1=60, S2=15, S3=0, E=100, W1=40, W2=30, W3=10, G=20)}
def scene(q):
    F = ak.frames(q); tris = []
    for g, dd in G.items():
        for n, m in dd.items():
            mm = m.copy(); mm.apply_transform(F[g]); tris.append(mm)
    loops = []
    for j, c in d["joints"].items():
        gA, gB = d["frm"][j], d["to"][j]; od = d["od"][j]
        Pa = (F[gA] @ np.r_[c["A"], 1])[:3]; Pb = (F[gB] @ np.r_[c["B"], 1])[:3]
        x, info = L.rod_relax(Pa, ta := F[gA][:3, :3] @ np.array(c["tA"]), Pb, F[gB][:3, :3] @ np.array(c["tB"]), c["L"], F, od / 2, 0.8); loops.append(np.asarray(x))
    return tris, loops
def draw(ax, tris, loops, az, el):
    a, e = np.deg2rad(az), np.deg2rad(el)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]); Rx = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]])
    R = Rx @ Rz; Lt = np.array([-0.4, -0.5, 0.75]); Lt /= np.linalg.norm(Lt)
    P, C, Dp = [], [], []
    for mm in tris:
        v = mm.vertices @ R.T; f = mm.faces[::2]; t = v[f]; n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
        s = np.abs(n @ Lt) * 0.7 + 0.3; P.append(t[:, :, [0, 2]]); C.append(np.c_[s * 0.62, s * 0.66, s * 0.72]); Dp.append(t[:, :, 1].mean(1))
    P = np.concatenate(P); C = np.concatenate(C); Dp = np.concatenate(Dp); o = np.argsort(-Dp)
    ax.add_collection(PolyCollection(P[o], facecolors=C[o], edgecolors="none"))
    for x in loops:
        y = x @ R.T; ax.plot(y[:, 0], y[:, 2], color="#d7191c", lw=2)
    allv = np.vstack([P.reshape(-1, 2)] + [(x @ R.T)[:, [0, 2]] for x in loops]); ax.set_xlim(allv[:, 0].min() - 20, allv[:, 0].max() + 20); ax.set_ylim(allv[:, 1].min() - 20, allv[:, 1].max() + 20); ax.set_aspect("equal"); ax.axis("off")
fig, axs = plt.subplots(2, 3, figsize=(18, 12))
S = {k: scene(q) for k, q in POSES.items()}
views = [("park", 35, 20, "park, rear-right 3/4"), ("park", 125, 20, "park, front-right 3/4"), ("park", 90, 0, "park, from the right"),
         ("reach forward", 35, 20, "reach forward"), ("arm raised", 35, 20, "arm raised, elbow bent"), ("arm raised", 125, 15, "arm raised, front 3/4")]
for ax, (k, az, el, t) in zip(axs.ravel(), views): draw(ax, *S[k], az, el); ax.set_title(t)
fig.suptitle("Third Shoulder right arm: shoulder stage to claw, with cable loops (red)"); plt.tight_layout(); plt.savefig("../whole_arm.png", dpi=90)
