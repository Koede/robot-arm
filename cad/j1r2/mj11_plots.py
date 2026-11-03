"""Figures for the MJ11 whole-arm sweep: tier tables and shoulder keep-out map.  usage: J1R2_ROOT=<work root> python3 mj11_plots.py -> stl/j1r2_sweep/mj11_whole_arm_tiers.png"""
import os, sys, json, numpy as np, matplotlib; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__))); matplotlib.use("Agg"); import matplotlib.pyplot as plt
import clib as L
R = os.path.join(L.ROOT, "stl", "j1r2_sweep"); S = json.load(open(os.path.join(R, "mj11_whole_arm_summary.json"))); T = S["tiers"]; s1, s2 = np.array(T["axes"]["S1p"]), np.array(T["axes"]["S2"])
K = json.load(open(os.path.join(R, "mj11_shoulder_box.json"))); k = K["keepout"]; box = K["S3_0"]["box"]["ranges"]
fig, ax = plt.subplots(1, 3, figsize=(17, 5.6), dpi=130)
for a, nm, key in ((ax[0], "FULL tier  E_max (deg)", "EL_F"), (ax[1], "RESTRICT tier  E_max (deg)", "EL_R")):
    M = np.array(T[key]); im = a.imshow(M, cmap="viridis", vmin=-1, vmax=145, origin="lower", aspect="auto"); a.set_title(nm)
    a.set_xticks(range(len(s2))); a.set_xticklabels([f"{v:.0f}" for v in s2], fontsize=7); a.set_yticks(range(len(s1))); a.set_yticklabels([f"{v:.0f}" for v in s1], fontsize=7); a.set_xlabel("S2 (deg)"); a.set_ylabel("S1' (deg)")
    for i in range(M.shape[0]):
        for j in range(M.shape[1]): a.text(j, i, f"{int(M[i, j])}", ha="center", va="center", fontsize=6, color="w" if M[i, j] < 70 else "k")
    plt.colorbar(im, ax=a, fraction=0.04)
a = ax[2]; fr = np.array(k["free_reachable"]); fa = np.array(k["free_any"]); S1, S2 = np.array(k["S1p"]), np.array(k["S2"])
cnt = fr.sum(axis=2) / fr.shape[2]; a.pcolormesh(S2, S1, cnt, cmap="Greens", shading="nearest", vmin=0, vmax=1)
a.add_patch(plt.Rectangle((box["S2"][0], box["S1p"][0]), box["S2"][1] - box["S2"][0], box["S1p"][1] - box["S1p"][0], fill=False, ec="r", lw=2, label="passive-stop box (S1' -95..5, S2 -15..17.4)"))
a.plot([17.4], [-18.2], "k*", ms=12, label="park"); a.set_title("shoulder level: reach-free from park (S3 -70/0/90 fraction)"); a.set_xlabel("S2 (deg)"); a.set_ylabel("S1' (deg)"); a.legend(fontsize=7, loc="lower right")
plt.tight_layout(); plt.savefig(os.path.join(R, "mj11_whole_arm_tiers.png")); print("ok")
