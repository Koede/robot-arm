"""Assembly renders with wearer + harness + arm: park and first-contact frontier poses. Output ../assembly_*.png  (painter's algorithm, matplotlib)."""
import os, sys, json, numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak
S = "../stl/shoulder/"; G = ak.groups(include_fingers=True, g_pct=50.0)
def ld(p, f=1): m = trimesh.load(S + p); return m
wear = ld("proxies/wearer_makehuman.stl"); wear = wear.slice_plane([0, 0, -700], [0, 0, 1], cap=False)   # torso, head, arm down to z -700
HAR = {k: ld(p) for k, p in dict(clasp="clasp_frame_R.stl", plate="chassis_plate_R.stl", wing="scapular/wing_R.stl", spine="scapular/spine_bridge.stl", pauldron="pauldron/pauldron_R.stl").items()}
COL = dict(wear=(0.93, 0.80, 0.68), clasp=(0.25, 0.55, 0.85), plate=(0.7, 0.7, 0.75), wing=(0.25, 0.7, 0.45), spine=(0.25, 0.7, 0.45), pauldron=(0.9, 0.45, 0.2), arm=(0.55, 0.58, 0.64), cowl=(0.9, 0.45, 0.2))
POSES = {"park": dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50),
         "head_contact": dict(S1=10, S2=30, S3=50, E=120, W1=0, W2=0, W3=0, G=50),
         "skin_contact": dict(S1=10, S2=15, S3=50, E=100, W1=0, W2=0, W3=0, G=50)}
def parts(q):
    F = ak.frames(q); out = [(wear, COL["wear"], 0.45)]
    for k, m in HAR.items(): out.append((m, COL[k], 1.0))
    for g, dd in G.items():
        for n, m in dd.items():
            mm = m.copy(); mm.apply_transform(F[g]); out.append((mm, COL["cowl"] if n in ("cowl", "head") else COL["arm"], 1.0))
    return out
def draw(ax, pts, az, el, lim):
    a, e = np.deg2rad(az), np.deg2rad(el)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]); Rx = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]])
    R = Rx @ Rz; Lt = np.array([-0.4, -0.5, 0.75]); Lt /= np.linalg.norm(Lt); P, C, D = [], [], []
    for m, col, al in pts:
        v = m.vertices @ R.T; f = m.faces[:: (3 if len(m.faces) > 40000 else 1)]; t = v[f]; n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0]); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-9
        s = (np.abs(n @ Lt) * 0.7 + 0.3)[:, None]; c = s * np.array(col)
        P.append(t[:, :, [0, 2]]); C.append(np.clip(c * (1 if al == 1 else 1.0), 0, 1)); D.append(t[:, :, 1].mean(1))
    P = np.concatenate(P); C = np.concatenate(C); D = np.concatenate(D); o = np.argsort(-D)
    ax.add_collection(PolyCollection(P[o], facecolors=C[o], edgecolors="none")); ax.set_xlim(*lim[0]); ax.set_ylim(*lim[1]); ax.set_aspect("equal"); ax.axis("off")
VIEWS = [(270, 8, "from behind"), (300, 15, "rear 3/4"), (0, 0, "side"), (90, 8, "from the front")]
for pk, q in POSES.items():
    pts = parts(q); fig, axs = plt.subplots(1, 4, figsize=(22, 7))
    for ax, (az, el, t) in zip(axs, VIEWS):
        draw(ax, pts, az, el, ((-420, 420), (-760, 560))); ax.set_title(t)
    fig.suptitle("%s   S1 %g S2 %g S3 %g E %g" % (pk, q["S1"], q["S2"], q["S3"], q["E"])); plt.tight_layout(); plt.savefig("../assembly_%s.png" % pk, dpi=70); plt.close()
    print("wrote", pk, flush=True)
