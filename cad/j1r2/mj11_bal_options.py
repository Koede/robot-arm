"""MJ11 torque-gate options, part 2 (2026-11-03): ideal-balancer floors, S1'-only spiral/cam fits, arm-mass and payload scaling, 4-D coverage under a firmware torque cap.  Same mass model and workspaces as mj11_torque.py.
usage: J1R2_ROOT=<root> python3 mj11_bal_options.py -> stl/j1r2_mounts/MJ11/torque_options2.json"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import mj11_torque as T
from scipy.optimize import minimize
cells, mask = T.ws_cells(); R = {}; CAP = T.CAP
def tab(cl, pay=(0.0, 0.5), lam=1.0, mscale=1.0):
    FA, S = T.frames(cl); out = {}
    for p in pay:
        a1 = np.zeros((len(T.ECONF), len(cl))); a2 = np.zeros_like(a1)
        for j, (E, S3) in enumerate(T.ECONF):
            pts = T.local_masses(E, S3, p, lam); pts = [(m * (mscale if i < len(pts) - (1 if p else 0) else 1.0), l) for i, (m, l) in enumerate(pts)] if mscale != 1.0 else pts
            a1[j], a2[j] = T.grav(FA, S, pts)
        out[p] = (a1, a2)
    return out
# --- 1. S2 at the hard-stop edge of the box, and per-th1 max |tau| tables (BOX), payload 0 / 0.5
hard = [(th, ph) for th in np.arange(-100.0, 10.1, 5.0) for ph in (-20.0, 22.4)]; Th = tab(hard)
R["S2_at_box_hard_stops"] = dict(tau_S2_max=round(float(abs(Th[0.5][1]).max()), 2), SF=round(T.RATED / float(abs(Th[0.5][1]).max()), 2), tau_S1p_max=round(float(abs(Th[0.5][0]).max()), 2))
box = cells["BOX"]; TB = tab(box); th_vals = sorted(set(c[0] for c in box)); rows = []
for th in th_vals:
    ix = [i for i, c in enumerate(box) if c[0] == th]; rows.append(dict(th1=float(th), S1p_pay0=round(float(abs(TB[0.0][0][:, ix]).max()), 2), S1p_pay05=round(float(abs(TB[0.5][0][:, ix]).max()), 2), S2_pay05=round(float(abs(TB[0.5][1][:, ix]).max()), 2)))
R["BOX_by_th1"] = rows
# --- 2. ideal pose-only balancer floor (function of th1, phi only, blind to E, S3, payload): residual = half the spread over (E, S3, payload) at each cell
for name in ("BOX", "REACH"):
    TT = tab(cells[name]); fl = {}
    for ax, k in (("S1p", 0), ("S2", 1)):
        allv = np.concatenate([TT[0.0][k], TT[0.5][k]], axis=0); lo, hi = allv.min(axis=0), allv.max(axis=0); fl[ax] = round(float(((hi - lo) / 2).max()), 2)
        # same but payload known (learned): spread over E, S3 only, balancer set for 0.25 kg
    R[f"ideal_pose_only_floor_{name}_Nm"] = fl; R[f"ideal_pose_only_floor_{name}_SF"] = {ax: round(T.RATED / v, 2) for ax, v in fl.items()}
# --- 3. S1'-only fits on the BOX: (i) linear spiral tau = k (th - th0) + c, (ii) pure th1-only cam (any function of th1)
TT = tab(cells["BOX"]); ths = np.array([c[0] for c in cells["BOX"]]); allt = np.concatenate([TT[0.0][0], TT[0.5][0]], axis=0); thr = np.radians(np.tile(ths, allt.shape[0] // len(ths)))
def res(x): return float(np.abs(allt.ravel() + (x[0] * (thr - x[1]))).max()) if False else float(np.abs(allt.reshape(-1, len(ths)) + x[0] * (np.radians(ths)[None] - x[1])).max())
best = None
for k0 in (2.0, 3.0, 4.0, 5.0):
    for t0 in (-0.8, -1.0, -1.4):
        r = minimize(res, [k0, t0], method="Nelder-Mead", options=dict(xatol=1e-4, fatol=1e-6, maxiter=2000))
        if best is None or r.fun < best.fun: best = r
k_, th0_ = best.x; R["spiral_S1p_BOX"] = dict(k_Nm_per_rad=round(float(k_), 3), th0_deg=round(math.degrees(th0_), 1), residual_Nm=round(float(best.fun), 2), SF=round(T.RATED / float(best.fun), 2),
                                               torque_at_ends_Nm=[round(float(k_ * (math.radians(t) - th0_)), 2) for t in (-95, 5)], note="spring torque on the arm = k (th1 - th0); sign chosen to oppose gravity")
# cam (function of th1 only): the floor over th1 with E, S3, payload blind
fl_cam = 0.0
for th in th_vals:
    ix = [i for i, c in enumerate(cells["BOX"]) if c[0] == th]; v = allt[:, ix]; fl_cam = max(fl_cam, (v.max() - v.min()) / 2)
R["cam_th1_only_floor_BOX"] = dict(residual_Nm=round(float(fl_cam), 2), SF=round(T.RATED / float(fl_cam), 2))
# spiral sizing (spring steel strip E 200 GPa, b x t, modulus-limited length L = pi E b t^3 theta/(6 T) ; sigma = 6T/(b t^2)) for k and the peak spring torque
def spiral(k, Tmax, b=25.0, t=1.0, E_=200e3, rho=7.85e-3):
    L = math.pi * E_ * b * t ** 3 / (6 * k * 1e3 / 1.0) if False else math.pi * E_ * b * t ** 3 / (6 * (k * 1e3))      # k N.m/rad -> N.mm/rad ; L in mm
    sig = 6 * Tmax * 1e3 / (b * t ** 2); return dict(strip=f"{b} x {t} mm", length_mm=round(L), sigma_MPa=round(sig), mass_g=round(rho * b * t * L, 0))
sp = R["spiral_S1p_BOX"]; Tmax = max(abs(v) for v in sp["torque_at_ends_Nm"]); R["spiral_S1p_BOX"]["sizing_b25_t1"] = spiral(sp["k_Nm_per_rad"], Tmax); R["spiral_S1p_BOX"]["sizing_b25_t0p8"] = spiral(sp["k_Nm_per_rad"], Tmax, 25.0, 0.8)
# --- 4. arm-mass scale and link scale for SF 2 (BOX / REACH), payload 0.5 / 0.35 / 0.25
ms_need = {}
for name in ("BOX", "REACH"):
    for pay in (0.5, 0.35, 0.25, 0.0):
        lo, hi = 0.2, 1.0
        for _ in range(25):
            mid = 0.5 * (lo + hi); TT_ = tab(cells[name], (0.0, pay) if pay else (0.0,), 1.0, mid) if pay else tab(cells[name], (0.0,), 1.0, mid); t1 = TT_[pay][0]; t2 = TT_[pay][1]
            if max(abs(t1).max(), abs(t2).max()) <= CAP: lo = mid
            else: hi = mid
        ms_need[f"{name}_pay{pay}"] = round(lo, 3)
R["arm_mass_scale_for_SF2"] = ms_need
# --- 5. 4-D coverage under the torque cap vs payload (BOX, REACH) and by th1
cov = {}
for name in ("BOX", "REACH"):
    TT = tab(cells[name]); row = {}
    for pay in (0.0, 0.1, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5):
        a1 = TT[0.0][0] + (TT[0.5][0] - TT[0.0][0]) * (pay / 0.5) if False else TT[0.0][0] + (TT[0.5][0] - TT[0.0][0]) * (pay / 0.5); a2 = TT[0.0][1] + (TT[0.5][1] - TT[0.0][1]) * (pay / 0.5)
        ok = np.maximum(abs(a1), abs(a2)) <= CAP; row[str(pay)] = dict(config4D_fraction=round(float(ok.mean()), 3), cell_fraction_any_config=round(float(ok.any(axis=0).mean()), 3))
    cov[name] = row
R["coverage_under_cap_by_payload"] = cov
json.dump(R, open(os.path.join(T.OUTD, "torque_options2.json"), "w"), indent=1, default=float); print(json.dumps(R, indent=1, default=float))
