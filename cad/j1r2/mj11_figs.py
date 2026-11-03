"""Charts for specs/SPEC_j1r2_mj11_loads_balancer.md. usage: J1R2_ROOT=/home/claude/ts python3 mj11_figs.py"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import j1r2_params as P, j1r2_bal_model as BM, mj11_torque_options as TQ
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))); OUT = os.path.join(REPO, "renders_j1r2", "mounts"); os.makedirs(OUT, exist_ok=True)
box = TQ.grid(TQ.BOX_TH, TQ.BOX_PH, pays=(0.0, 0.25, 0.5)); T1, T2 = TQ.torques(box); th = np.array([p[0] for p in box]); pay = np.array([p[4] for p in box])
ths = sorted(set(th)); op = json.load(open(os.path.join(REPO, "stl", "j1r2_bal", "mj11_torque_options.json"))); ld = json.load(open(os.path.join(REPO, "stl", "j1r2_mounts", "MJ11", "loads.json")))
fig, ax = plt.subplots(1, 3, figsize=(17, 5.2), dpi=130)
a = ax[0]
for pp, c in ((0.0, "#4c78a8"), (0.5, "#e45756")):
    lo = [T1[(th == t) & (pay == pp)].min() for t in ths]; hi = [T1[(th == t) & (pay == pp)].max() for t in ths]; a.fill_between(ths, lo, hi, color=c, alpha=0.30, label=f"S1' gravity torque, payload {pp} kg (all phi, E, S3)")
xl = op["th1_only_compensator"]["linear_torsion_spring"]["coeff_const_Nm_and_slope_Nm_per_rad"]; a.plot(ths, [-(xl[0] + xl[1] * math.radians(t)) for t in ths], "k--", lw=1.4, label="S1' torsion spring (best linear fit), sign flipped")
a.axhline(4.15, color="g", ls=":"); a.axhline(-4.15, color="g", ls=":", label="SF 2 gate +-4.15 N.m"); a.axhline(8.3, color="r", ls=":"); a.axhline(-8.3, color="r", ls=":", label="rated +-8.3 N.m")
a.set_xlabel("S1' angle th1 (deg)"); a.set_ylabel("N.m"); a.set_title("S1' gravity torque over the MJ11 box (unbalanced)"); a.legend(fontsize=7); a.grid(alpha=0.3)
a = ax[1]; names = list(ld["layouts"]); lab = [n.split(" ")[0] + "\n" + (n.split("(")[1][:26] if "(" in n else "") for n in names]
x = np.arange(len(names)); w = 0.2
for i, (key, c, nm) in enumerate((("reach", "#4c78a8", "reach x1"), ("reach2", "#e45756", "reach x2"), ("stallS1(-)", "#f58518", "S1' stall 8.3 N.m"), ("S1stop(-)", "#b279a2", "S1' stop 20 N.m"))):
    v = []
    for n in names:
        L_ = ld["layouts"][n]
        v.append(L_["x1"]["reach"]["bolts"]["beam_tension_max_N"] if key == "reach" else L_["x2"]["reach"]["bolts"]["beam_tension_max_N"] if key == "reach2" else L_["x1"]["reach+" + key]["bolts"]["beam_tension_max_N"])
    a.bar(x + (i - 1.5) * w, v, w, color=c, label=nm)
a.axhline(1000, color="r", ls="--", label="M4 insert pull-out 1 kN (UNVERIFIED)"); a.axhline(500, color="orange", ls=":", label="SF 2 on 1 kN"); a.set_xticks(x); a.set_xticklabels([n.split(" ")[0] for n in names]); a.set_ylabel("worst bolt tension (N), beam-group model")
a.set_title("Rail bolt tension by layout"); a.legend(fontsize=7); a.grid(alpha=0.3, axis="y")
a = ax[2]; ls = op["link_scale_S1p_max_Nm"]; lam = [float(k) for k in ls]
for pp, c in (("0.0", "#4c78a8"), ("0.25", "#54a24b"), ("0.5", "#e45756")): a.plot(lam, [ls[str(l)][pp] if str(l) in ls else ls[k][pp] for l, k in zip(lam, ls)], "o-", color=c, label=f"payload {pp} kg")
a.axhline(4.15, color="g", ls=":", label="SF 2 gate"); a.invert_xaxis(); a.set_xlabel("distal link-length scale (1 = 230/180/90 mm)"); a.set_ylabel("worst S1' gravity torque (N.m)"); a.set_title("Option (c): shorter links, no balancer"); a.legend(fontsize=7); a.grid(alpha=0.3)
plt.tight_layout(); plt.savefig(os.path.join(OUT, "MJ11_loads_balancer_charts.png")); print("ok")
