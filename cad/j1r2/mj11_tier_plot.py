"""Heat maps of the MJ11 tier tables (fw15 gate) over (S2, S1') with the passive-stop box.  usage: python3 mj11_tier_plot.py [sweep dir] -> mj11_tiers.png"""
import os, sys, json, numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
D = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else "."); t = json.load(open(os.path.join(D, "mj11_tiers.json"))); S1, S2 = t["grid"]["S1p"], t["grid"]["S2"]
fig, ax = plt.subplots(1, 3, figsize=(17, 6.2), constrained_layout=True)
for a, n in zip(ax, ("FULL", "RESTRICT", "NEUTRAL")):
    M = np.array(t["fw15"][n]); im = a.imshow(np.where(M < 0, np.nan, M), origin="lower", aspect="auto", cmap="viridis", vmin=0, vmax=145)
    a.set_xticks(range(len(S2))); a.set_xticklabels([f"{v:.0f}" for v in S2], fontsize=7); a.set_yticks(range(len(S1))); a.set_yticklabels([f"{v:.0f}" for v in S1], fontsize=7)
    for i in range(len(S1)):
        for j in range(len(S2)): a.text(j, i, "x" if M[i][j] < 0 else f"{M[i][j]:.0f}", ha="center", va="center", fontsize=6.5, color="w" if M[i][j] < 70 else "k")
    i0, i1 = np.searchsorted(S1, -95), np.searchsorted(S1, 5); j0, j1 = np.searchsorted(S2, -15) - .5, np.argmin(abs(np.array(S2) - 17.4))
    a.add_patch(plt.Rectangle((np.searchsorted(S2, -10) - 1.5, i0 - .5), j1 - (np.searchsorted(S2, -10) - 1) + 1.0, i1 - i0 + 1, fill=False, ec="r", lw=2)); a.set_xlabel("S2 (deg)"); a.set_ylabel("S1' (deg)"); a.set_title(f"{n}: largest E (deg), x = closed")
fig.suptitle("MJ11 whole-arm sweep: elbow tier tables, 15 mm arm gate (candidate); red = passive-stop box + 5 deg hard-stop expansion (S1p -100..10, S2 -20..22.4)")
fig.savefig(os.path.join(D, "mj11_tiers.png"), dpi=110)
