"""MJ11 torque-gate options, part 3 (2026-11-03): spring/tube/mass sizing of the balancer variants and the firmware-cap tables.  usage: J1R2_ROOT=<root> python3 mj11_bal_sizing.py -> stl/j1r2_mounts/MJ11/torque_options3.json"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import mj11_torque as T, mj11_balancer_fit as BF
OUT = {}; G_SHEAR = 80000.0
def size(k_cord, L0, dmin, dmax, tackle=2):
    xs_max = max((dmax - L0) / tackle, 0.0); xs_min = max((dmin - L0) / tackle, 0.0); Fm = tackle * k_cord * (dmax - L0)
    for dwi in np.arange(3.0, 7.01, 0.25):
        for Dm in (36.0, 40.0, 44.0):
            C = Dm / dwi
            if not (5.0 <= C <= 12.0): continue
            KB = (4 * C + 2) / (4 * C - 3); tau = KB * 8 * Fm * Dm / (np.pi * dwi ** 3); Sut = 2211.0 / dwi ** 0.145
            if tau <= 0.31 * Sut:
                ks = tackle ** 2 * k_cord; Na = G_SHEAR * dwi ** 4 / (8 * Dm ** 3 * ks); Ls = (Na + 2) * dwi; Lf = Ls + 2.12 * xs_max; Lt = Lf + 8 + 10 + 6 + 8; ms = math.pi * Dm * (Na + 2) * math.pi / 4 * dwi ** 2 * 7.85e-3
                mass = 0.407 * Lt + 149.0 + ms + 52.0
                return dict(tackle=tackle, F_spring_max_N=round(Fm, 0), F_cord_max_N=round(Fm / tackle, 0), wire_d=float(dwi), D_mean=Dm, spring_mass_g=round(ms, 0), L_free_mm=round(Lf, 0), tube_mm=round(Lt, 0), OD_tube_mm=round(Dm + dwi + 10, 1), total_mass_g=round(mass, 0), tau_over_Sut=round(tau / Sut, 3))
    return None
VAR = {}
x_j1 = np.r_[BF.A_OFF0, BF.B_LOC0, BF.K0, BF.L00]
for lab, ws, fn in (("a1 J1 rev 2 design unchanged", "BOX", None), ("a2 BOX refit (min residual)", "BOX", "bal_fit_BOX_best.json"), ("a3 BOX light (residual<=3.5, min force)", "BOX", "bal_fit_BOX_light.json"),
                    ("a4 REACH refit", "REACH", "bal_fit_REACH_best.json"), ("a5 REACH light", "REACH", "bal_fit_REACH_light.json")):
    F = BF.Fit(ws)
    if fn: d = json.load(open(os.path.join(T.OUTD, fn))); x = np.r_[d["x"]["A_off"], d["x"]["B_loc"], d["x"]["k"], d["x"]["L0"]]
    else: x = x_j1
    st = F.cost(x, ret=True); s_ = {}
    for tk in (2, 3, 4): s_[f"{tk}:1"] = size(x[6], x[7], st["d_min"], st["d_max"], tk)
    # residual with the arm-side 52 g collar mass added (as built)
    VAR[lab] = dict(ws=ws, A_off=[round(float(v), 2) for v in x[0:3]], B_loc=[round(float(v), 2) for v in x[3:6]], k_cord=round(float(x[6]), 4), L0=round(float(x[7]), 2), residual_S1p=round(st["r1"], 2), residual_S2=round(st["r2"], 2),
                    SF_S1p=round(T.RATED / st["r1"], 2), SF_S2=round(T.RATED / st["r2"], 2), cord_len=[round(st["d_min"], 0), round(st["d_max"], 0)], sizing=s_)
OUT["balancer_variants"] = VAR
# firmware cap tables: 4-D coverage at payload p for cap c, BOX / REACH; allowed fraction per (th1 bin, E) at 0.5 kg for the SF-2 cap
cells, mask = T.ws_cells(); caps = (4.15, 5.0, 6.0, 7.0, 7.17, 8.3); cov = {}
for name in ("BOX", "REACH"):
    TT = T.table(cells[name]); row = {}
    for pay in (0.0, 0.25, 0.35, 0.5):
        a1 = TT[0.0][0] + (TT[0.5][0] - TT[0.0][0]) * (pay / 0.5); a2 = TT[0.0][1] + (TT[0.5][1] - TT[0.0][1]) * (pay / 0.5); w = np.maximum(abs(a1), abs(a2))
        row[str(pay)] = {str(c): round(float((w <= c).mean()), 3) for c in caps}
    cov[name] = row
OUT["cap_coverage_4D"] = cov
TT = T.table(cells["BOX"]); box = cells["BOX"]; ths = sorted(set(c[0] for c in box)); tab = {}
for ie, (E, S3) in enumerate(T.ECONF):
    pass
Es = (0, 45, 90, 145); tab = {}
for E in Es:
    rows = []
    for th in ths[::2]:
        ix = [i for i, c in enumerate(box) if c[0] == th]; js = [j for j, (E_, S3) in enumerate(T.ECONF) if E_ == E]
        w = max(abs(TT[0.5][0][js][:, ix]).max(), abs(TT[0.5][1][js][:, ix]).max()); rows.append((float(th), round(float(w), 2)))
    tab[str(E)] = rows
OUT["BOX_worst_tau_by_E_and_th1_pay0p5"] = tab
json.dump(OUT, open(os.path.join(T.OUTD, "torque_options3.json"), "w"), indent=1, default=float)
for k, v in VAR.items(): print(k, v["A_off"], v["B_loc"], v["k_cord"], v["L0"], "res", v["residual_S1p"], v["residual_S2"], "SF", v["SF_S1p"], v["SF_S2"], v["cord_len"], {t: (s["tube_mm"], s["total_mass_g"], s["F_spring_max_N"]) for t, s in v["sizing"].items() if s})
print(json.dumps(cov, indent=0)[:1400]); print(tab)
