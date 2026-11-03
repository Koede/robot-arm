"""MJ11 gravity-torque gate study (2026-11-03, SHOULDER / CONTROL).  Robot frame, mm / N / N m / deg.  Answers: with the MJ11 pad (-60, -190) and NO balancer, what do S1' / S2 carry over the REAL MJ11 workspace, and what do the
options cost?  No decision is made here (Forest's call): this script produces the numbers for specs/SPEC_j1r2_mj11_loads_balancer.md.
Mass model = j1r2_bal_model.py (arm beyond S2 1489 g CG ~247 mm, hand/forearm offset 57, AK70 + yoke on S1', payload at the 575 mm tip).  Torques are translation invariant, so the J1 rev 2 numbers (S1' 7.31, S2 6.59) carry over exactly on the same grid.
Workspaces: FULL = J1 rev 2 grid (th1 -95..35, phi -30..88); REACH = collision-free cells reachable from park (stl/j1r2_mounts/MJ11/workspace_mask.json); BOX = best passive-stop box (S1' -95..5, S2 -15..17.4).  E 0/45/90/145, S3 -70/0/90, payload 0/0.5.
usage: J1R2_ROOT=<work root> J1_PAD=-60,-190,12 python3 mj11_torque.py   -> stl/j1r2_mounts/MJ11/torque_options.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import j1r2_params as P, j1r2_bal_model as BM
G_ = 9.81; RATED = 8.3; SF_GATE = 2.0; CAP = RATED / SF_GATE
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))); OUTD = os.path.join(REPO, "stl", "j1r2_mounts", "MJ11")
D0 = BM.D0; P1 = BM.P1
ECONF = [(E, S3) for E in (0, 45, 90, 145) for S3 in (-70, 0, 90)]
def frames(cells):
    FA = np.zeros((len(cells), 4, 4)); S = np.zeros((len(cells), 3))
    for i, (th, ph) in enumerate(cells):
        F0, FA_, s, a, d = P.shoulder_frames(D0, th, ph); FA[i] = FA_; S[i] = s
    return FA, S
def local_masses(E, S3, pay, lam=1.0, extra=()):
    pts = [(m, np.asarray(p, float) * lam) for m, p in BM.arm_points(E, S3, 0.0)]
    if pay: pts.append((pay, BM.rotS3(BM.rotE(np.asarray(BM.TIP, float), E), S3) * lam))
    return pts + [(m, np.asarray(l, float)) for m, l in extra]
def grav(FA, S, pts):
    """gravity moments (N m) about the S1' axis (x) and the S2 axis (s) for the arm masses pts=[(kg, local mm)] at every cell; M1 masses added to S1' only (as j1r2_bal_model)."""
    t1 = np.zeros(len(FA)); t2 = np.zeros(len(FA))
    for m, loc in pts:
        p = FA[:, :3, :3] @ loc + FA[:, :3, 3]; r = p - P1; mx = -r[:, 1] * m * G_; my = r[:, 0] * m * G_
        t1 += mx; t2 += my * S[:, 1]
    for m, ds in BM.M1_MASSES:
        # M1 masses sit on the s axis: P1 + ds*s -> r = ds * s
        r = ds * S; t1 += -r[:, 1] * m * G_
    return t1 * 1e-3, t2 * 1e-3
def table(cells, payloads=(0.0, 0.5), lam=1.0):
    FA, S = frames(cells); T = {}
    for pay in payloads:
        a1 = np.zeros((len(ECONF), len(cells))); a2 = np.zeros_like(a1)
        for j, (E, S3) in enumerate(ECONF): a1[j], a2[j] = grav(FA, S, local_masses(E, S3, pay, lam))
        T[pay] = (a1, a2)
    return T
def ws_cells():
    cells = {}
    cells["FULL_J1grid"] = [(th, ph) for th in np.arange(-95, 35.1, 5.0) for ph in np.arange(-30, 88.1, 6.0)]
    m = json.load(open(os.path.join(OUTD, "workspace_mask.json"))); S1, S2 = m["S1p"], m["S2"]
    cells["REACH"] = [(S1[i], S2[j]) for i in range(len(S1)) for j in range(len(S2)) if m["reach"][i][j]]
    cells["BOX"] = [(th, ph) for th in np.arange(-95, 5.1, 5.0) for ph in list(np.arange(-15, 15.1, 5.0)) + [17.4]]
    return cells, m
def stats(T, pay_list=(0.0, 0.5)):
    out = {}
    for pay in pay_list:
        a1, a2 = T[pay]; out[pay] = dict(S1p=round(float(abs(a1).max()), 2), S2=round(float(abs(a2).max()), 2), SF_S1p=round(RATED / float(abs(a1).max()), 2), SF_S2=round(RATED / float(abs(a2).max()), 2))
    return out
def alpha_max_payload(T, cap=CAP):
    """largest payload (kg) such that |tau| <= cap in EVERY cell and arm configuration (tau linear in payload)."""
    best = {}
    for ax, k in (("S1p", 0), ("S2", 1)):
        t0 = T[0.0][k]; t5 = T[0.5][k]; slope = (t5 - t0) / 0.5; hi = np.inf
        # |t0 + slope*x| <= cap for x>=0 : solve per element
        with np.errstate(divide="ignore", invalid="ignore"):
            xp = np.where(slope > 1e-9, (cap - t0) / slope, np.where(slope < -1e-9, (-cap - t0) / slope, np.inf)); xp = np.where(abs(t0) > cap, -1.0, xp)
        best[ax] = float(xp.min())
    return best
if __name__ == "__main__":
    cells, mask = ws_cells(); R = dict(rated_Nm=RATED, sf_gate=SF_GATE, cap_for_sf2_Nm=CAP, workspaces={}); TT = {}
    # --- 1. unbalanced torque per workspace (reproduce J1 rev 2: S1' 7.31, S2 6.59 on its own grid; its balancer arm-side 52 g excluded)
    for name, cl in cells.items():
        T = table(cl); TT[name] = T; st = stats(T); a1, a2 = T[0.5]
        i1 = np.unravel_index(np.argmax(abs(a1)), a1.shape); i2 = np.unravel_index(np.argmax(abs(a2)), a2.shape)
        R["workspaces"][name] = dict(n_cells=len(cl), stats=st, worst_S1p=dict(cell=[float(v) for v in cl[i1[1]]], E_S3=ECONF[i1[0]], tau=round(float(a1[i1]), 2)), worst_S2=dict(cell=[float(v) for v in cl[i2[1]]], E_S3=ECONF[i2[0]], tau=round(float(a2[i2]), 2)),
                                     payload_for_SF2_kg=alpha_max_payload(T))
    # --- 2. per-cell worst over E, S3 at payload 0.5: area share under the SF-2 cap, and payload capability map (b: firmware cap)
    capres = {}
    for name in ("REACH", "BOX"):
        T = TT[name]; cl = cells[name]; w1 = abs(T[0.5][0]).max(axis=0); w2 = abs(T[0.5][1]).max(axis=0); w = np.maximum(w1, w2); w0 = np.maximum(abs(T[0.0][0]).max(axis=0), abs(T[0.0][1]).max(axis=0))
        cap_ok = w <= CAP; cap_ok0 = w0 <= CAP
        # payload capability per cell (largest payload kg with |tau|<=CAP for all E,S3), both axes
        pc = []
        for i in range(len(cl)):
            x = np.inf
            for k in (0, 1):
                t0 = T[0.0][k][:, i]; t5 = T[0.5][k][:, i]; sl = (t5 - t0) / 0.5
                with np.errstate(divide="ignore", invalid="ignore"): xp = np.where(sl > 1e-9, (CAP - t0) / sl, np.where(sl < -1e-9, (-CAP - t0) / sl, np.inf))
                xp = np.where(abs(t0) > CAP, -1.0, xp); x = min(x, xp.min())
            pc.append(max(min(x, 0.5), -1.0))
        pc = np.array(pc)
        # 4-D configuration fraction (th1, phi, E, S3) that stays under the cap at 0.5 kg
        frac4 = float(np.mean(np.maximum(abs(T[0.5][0]), abs(T[0.5][1])) <= CAP))
        capres[name] = dict(cells=len(cl), cell_fraction_all_E_S3_ok_at_0p5kg=round(float(cap_ok.mean()), 3), cell_fraction_ok_empty=round(float(cap_ok0.mean()), 3), config4D_fraction_ok_at_0p5kg=round(frac4, 3),
                            payload_cap_quantiles_kg={q: round(float(np.quantile(pc, q)), 3) for q in (0.0, 0.1, 0.25, 0.5, 0.75, 1.0)}, cells_with_payload_ge_0p5=round(float(np.mean(pc >= 0.5 - 1e-9)), 3),
                            cells_empty_arm_still_over_cap=int((~cap_ok0).sum()), worst_empty_Nm=round(float(w0.max()), 2))
    R["firmware_cap"] = capres
    # --- 3. payload and link-length trade (SF vs worst torque), per workspace
    trade = {}
    for name in ("REACH", "BOX", "FULL_J1grid"):
        cl = cells[name]; rows = []
        for lam in (1.0, 0.9, 0.8, 0.7):
            T = table(cl, (0.0, 0.5), lam) if lam != 1.0 else TT[name]
            for pay in (0.0, 0.25, 0.35, 0.5):
                a1 = T[0.0][0] + (T[0.5][0] - T[0.0][0]) * (pay / 0.5); a2 = T[0.0][1] + (T[0.5][1] - T[0.0][1]) * (pay / 0.5)
                rows.append(dict(link_scale=lam, payload_kg=pay, S1p=round(float(abs(a1).max()), 2), S2=round(float(abs(a2).max()), 2), SF=round(RATED / float(max(abs(a1).max(), abs(a2).max())), 2)))
        trade[name] = rows
    R["payload_link_trade"] = trade
    # link scale for SF 2 at 0.5 kg
    lam_need = {}
    for name in ("REACH", "BOX", "FULL_J1grid"):
        lo, hi = 0.3, 1.0
        for _ in range(30):
            mid = 0.5 * (lo + hi); T = table(cells[name], (0.0, 0.5), mid); w = max(abs(T[0.5][0]).max(), abs(T[0.5][1]).max())
            if w <= CAP: lo = mid
            else: hi = mid
        lam_need[name] = round(lo, 3)
    R["link_scale_for_SF2_at_0p5kg"] = lam_need
    # --- 4. thermal duty at the hold torques (thermal.h AK70 constants: kt 1.153, reff 0.204, rc 7.6 K/W, C 150 J/K, derate 95 C, cut 110 C, ambient 35 C assumed)
    kt, reff, rc, C, t_der, t_cut, amb = 1.153, 0.204, 7.6, 150.0, 95.0, 110.0, 35.0
    th = {}
    for tau in (2.0, 3.0, 4.15, 5.0, 6.0, 7.0, 7.31, 8.3):
        Pw = reff * (tau / kt) ** 2; dT = Pw * rc; tc_ss = amb + dT; tau_th = rc * C
        def tt(tl):
            if amb + dT <= tl: return None
            return float(-tau_th * np.log(1 - (tl - amb) / dT))
        th[f"{tau}"] = dict(I_A=round(tau / kt, 2), P_W=round(Pw, 2), T_case_ss_C=round(tc_ss, 1), t_to_derate_s=(None if tt(t_der) is None else round(tt(t_der), 0)), t_to_cut_s=(None if tt(t_cut) is None else round(tt(t_cut), 0)))
    R["thermal_hold_AK70"] = th
    tau_ss_der = float(np.sqrt((t_der - amb) / rc / reff) * kt); tau_ss_cut = float(np.sqrt((t_cut - amb) / rc / reff) * kt)
    R["thermal_steady_torque_limits_Nm"] = dict(derate_95C=round(tau_ss_der, 2), cut_110C=round(tau_ss_cut, 2), note="lumped model from thermal.h (ASSUMED constants, T1 bench fit pending); rated 8.3 N m is the vendor figure, the lumped case-node model reaches the 110 C cut below it")
    json.dump(R, open(os.path.join(OUTD, "torque_options.json"), "w"), indent=1, default=float)
    print(json.dumps({k: R[k] for k in ("workspaces", "firmware_cap", "link_scale_for_SF2_at_0p5kg", "thermal_hold_AK70", "thermal_steady_torque_limits_Nm")}, indent=1, default=float))
