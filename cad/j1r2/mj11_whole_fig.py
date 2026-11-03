"""Figure: MJ11 shoulder-level contact map (5 deg, per S3), passive box, and the proposed FULL / RESTRICT elbow-limit tables.  -> renders_j1r2/mj11_whole_arm_sweep.png"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import j1r2_sweep as S, mj11_sweep as W, mj_concepts3 as C3
R = os.path.join(S.OUT, ".."); sc, _ = W.scene(C3.mj11_geo(None)); vals = S.grid_values(sc, dict(steps={"S1p": 5.0, "S2": 5.0, "S3": [-70.0, 0.0, 90.0]})); pairs = S.build_pairs(sc); J = sc.joints
js = json.load(open(os.path.join(S.OUT, "sweep_mj11_sh5.json"))); z = np.load(os.path.join(S.OUT, "tables_mj11_sh5.npz")); rep = json.load(open(os.path.join(S.OUT, "mj11_whole_report.json"))); sb = json.load(open(os.path.join(S.OUT, "mj11_shoulder5_report.json")))
S1, S2 = vals["S1p"], vals["S2"]; cls = np.zeros((len(S1), len(S2)), int); names = {1: "head", 2: "skin", 3: "MJ11 clasp bracket", 4: "wing/rail", 5: "M1|M3"}
for pi, p in enumerate(pairs):
    if js["pairs"][p.key]["designed"]: continue
    b = (np.broadcast_to(S._bcast(z[p.key.replace("|", "__")], p.dims, J), (len(S1), len(S2), 3)) < p.MM)[:, :, 1]     # S3 = 0 slice
    c = 1 if "head" in p.obj else 2 if "skin" in p.obj else 3 if "clasp" in p.obj else 5 if p.kind == "grp" else 4
    cls[b & (cls == 0)] = c
fig, ax = plt.subplots(1, 3, figsize=(19, 6.2), dpi=130)
cm = matplotlib.colors.ListedColormap(["#e8f3e8", "#c0392b", "#e67e22", "#2980b9", "#8e44ad", "#16a085"])
ax[0].pcolormesh(np.r_[S2 - 2.5, S2[-1] + 2.5], np.r_[S1 - 2.5, S1[-1] + 2.5], cls, cmap=cm, vmin=0, vmax=5, shading="flat")
b = sb["box"]["ranges"]; ax[0].add_patch(plt.Rectangle((b["S2"][0], b["S1p"][0]), b["S2"][1] - b["S2"][0], b["S1p"][1] - b["S1p"][0], fill=False, ec="k", lw=2.2)); ax[0].plot([17.4], [-18.2], "k*", ms=14)
ax[0].set_xlabel("S2 (deg)"); ax[0].set_ylabel("S1' (deg)"); ax[0].set_title("shoulder level @ S3=0, 5 deg: first-contact cause\nbox S1' %g..%g, S2 %g..%g (cov %.3f); star = park" % (*b["S1p"], *b["S2"], sb["box"]["cov"]))
from matplotlib.patches import Patch; ax[0].legend(handles=[Patch(color=cm(i), label=n) for i, n in [(0, "free")] + list(names.items())], loc="upper right", fontsize=7)
t = rep["tiers"]; a1, a2 = t["axes"]["S1p"], t["axes"]["S2"]
for k, (nm, key) in enumerate((("FULL (any S3 / wrist)", "EL_F"), ("RESTRICT (|S3|<=30, |W2|<=30, |W3|<=15)", "EL_R"))):
    m = np.array(t[key]); m = np.where(np.array(t["shoulder_reach_free_S3_0"]), m, np.nan); im = ax[k + 1].imshow(m, cmap="viridis", origin="lower", aspect="auto", vmin=-1, vmax=145)
    ax[k + 1].set_xticks(range(len(a2))); ax[k + 1].set_xticklabels([f"{v:g}" for v in a2], fontsize=7, rotation=60); ax[k + 1].set_yticks(range(len(a1))); ax[k + 1].set_yticklabels([f"{v:g}" for v in a1], fontsize=7)
    for i in range(m.shape[0]):
        for j in range(m.shape[1]):
            if not np.isnan(m[i, j]): ax[k + 1].text(j, i, f"{m[i, j]:.0f}", ha="center", va="center", fontsize=6.5, color="w" if m[i, j] < 80 else "k")
    ax[k + 1].set_title(f"elbow limit E_max (deg), {nm}\n10 deg grid, shoulder-reach-free cells only (-1 = NEUTRAL only)", fontsize=9); ax[k + 1].set_xlabel("S2 (deg)"); ax[k + 1].set_ylabel("S1' (deg)")
plt.colorbar(im, ax=ax[2], shrink=0.8); plt.tight_layout(); os.makedirs(os.path.join(S.OUT, "..", "..", "renders_j1r2"), exist_ok=True); out = os.path.join(S.OUT, "..", "..", "renders_j1r2", "mj11_whole_arm_sweep.png"); plt.savefig(out); print(os.path.abspath(out))
