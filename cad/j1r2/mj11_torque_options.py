"""MJ11 torque gate options (2026-11-03, SHOULDER). Gravity torque of S1' / S2 for the MJ11 pad (-60,-190) and what each way of reaching SF >= 2 costs.
Gate: |tau| <= 8.3/2 = 4.15 N.m on AK70-10 (rated 8.3). The gravity torque is translation invariant (uniform g, frames relative to the S1' axis), so the pad move does not change it;
what MJ11 changes is the REACHABLE pose box (passive box S1' -95..5, S2 -15..17.4 from stl/j1r2_mounts/mj11 sweep) - S2 never gets near the forward-horizontal worst pose.
Model: j1r2_bal_model (arm 1489 g beyond S2, CG ~249 mm, payload at 575 mm, S1' stage 0.621 + 0.150 kg).
usage: J1R2_ROOT=/home/claude/ts python3 mj11_torque_options.py  -> stl/j1r2_bal/mj11_torque_options.json"""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import j1r2_params as P, j1r2_bal_model as M
from scipy.optimize import linprog
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))); OUT = os.path.join(REPO, "stl", "j1r2_bal"); os.makedirs(OUT, exist_ok=True)
RATED, GATE = 8.3, 8.3 / 2.0
BOX_TH = (-95.0, 5.0); BOX_PH = (-15.0, 17.4)          # MJ11 passive box (SPEC_j1r2_blade_seat)
E_SET, S3_SET, PAY_SET = (0, 45, 90, 145), (-70, 0, 90), (0.0, 0.25, 0.5)
BOOT_G = 99.0                                            # sensor boots per arm (DESIGN_STATE 12.4), at the wrist/hand, ~ +480 mm

def grid(th_rng, ph_rng, dth=5.0, dph=6.0, pays=PAY_SET, es=E_SET, s3s=S3_SET):
    TH = np.arange(th_rng[0], th_rng[1] + 1e-6, dth); PH = list(np.arange(ph_rng[0], ph_rng[1] + 1e-6, dph))
    if abs(PH[-1] - ph_rng[1]) > 1e-6: PH.append(ph_rng[1])
    return [(t, p, E, S3, pay) for t in TH for p in PH for E in es for S3 in s3s for pay in pays]

def torques(poses, extra_arm=(), scale=1.0):
    """(tau1, tau2) N.m arrays. scale: link-length factor on the distal geometry (positions along the arm; masses unchanged: conservative)."""
    t1 = np.zeros(len(poses)); t2 = np.zeros(len(poses)); seg0 = list(M.SEG); tip0 = M.TIP; elb0 = M.ELB.copy()
    if scale != 1.0:
        M.SEG[:] = [(n, m, (p[0], p[1] * scale, p[2] * scale), re) for n, m, p, re in seg0]; M.TIP = (0, M.TIP[1] * scale, M.TIP[2] * scale); M.ELB = M.ELB * scale
    try:
        for i, p in enumerate(poses):
            a, b = M.grav_moments(*p, extra_arm=list(extra_arm)); t1[i], t2[i] = a * 1e-3, b * 1e-3
    finally:
        M.SEG[:] = seg0; M.TIP = tip0; M.ELB = elb0
    return t1, t2

def summ(poses, t1, t2, tag):
    a = np.array([[p[0], p[1], p[2], p[3], p[4]] for p in poses]); i1, i2 = int(np.argmax(abs(t1))), int(np.argmax(abs(t2)))
    return dict(tag=tag, n=len(poses), S1p_max=round(float(abs(t1).max()), 2), S2_max=round(float(abs(t2).max()), 2), SF_S1p=round(RATED / float(abs(t1).max()), 2), SF_S2=round(RATED / float(abs(t2).max()), 2),
                worst_S1p_pose=a[i1].tolist(), worst_S2_pose=a[i2].tolist())

if __name__ == "__main__":
    res = {}
    full = grid((-95, 35), (-30, 88), pays=(0.0, 0.5)); box = grid(BOX_TH, BOX_PH, pays=(0.0, 0.25, 0.5))
    T1f, T2f = torques(full); res["full_soft_range"] = summ(full, T1f, T2f, "J1 rev 2 soft range (reproduces SPEC_j1r2_balancer 7.31 / 6.59)")
    T1b, T2b = torques(box); res["mj11_box"] = summ(box, T1b, T2b, "MJ11 passive box S1' -95..5, S2 -15..17.4")
    pay = np.array([p[4] for p in box]); res["mj11_box_by_payload"] = {str(pp): dict(S1p=round(float(abs(T1b[pay == pp]).max()), 2), S2=round(float(abs(T2b[pay == pp]).max()), 2)) for pp in PAY_SET}
    # sensitivity: boots +99 g at the hand (local z 480, y 57) on each arm
    T1bb, T2bb = torques(box, extra_arm=[(BOOT_G * 1e-3, (0, 57.0, 480.0))]); res["mj11_box_with_boots"] = summ(box, T1bb, T2bb, "box + 99 g boot at the hand (not rotated with E: upper bound)")
    # ---------------- (b) firmware cap: which fraction of the reachable pose space is below the SF-2 gate (4.15 N.m)
    A = np.array([[p[0], p[1], p[2], p[3], p[4]] for p in box]); ok = (abs(T1b) <= GATE) & (abs(T2b) <= GATE)
    cap = {}
    for pp in PAY_SET:
        m_ = A[:, 4] == pp; cap[f"payload {pp}"] = dict(frac_poses_ok=round(float(ok[m_].mean()), 3), frac_th_phi_cells_ok_all_E_S3=None)
        cells = {}
        for (t, ph) in {(p[0], p[1]) for p in box}:
            mm = m_ & (A[:, 0] == t) & (A[:, 1] == ph); cells[(t, ph)] = bool(ok[mm].all())
        cap[f"payload {pp}"]["frac_th_phi_cells_ok_all_E_S3"] = round(sum(cells.values()) / len(cells), 3)
        # E restriction: smallest E bend that makes the cell safe (E in 0,45,90,145), worst S3
        needE = {}
        for (t, ph) in cells:
            for E in E_SET:
                mm = m_ & (A[:, 0] == t) & (A[:, 1] == ph) & (A[:, 2] >= E)
                if ok[mm].all(): needE[(t, ph)] = E; break
            else: needE[(t, ph)] = None
        vals = [v for v in needE.values()]; cap[f"payload {pp}"]["frac_cells_ok_with_E_floor"] = round(sum(v is not None for v in vals) / len(vals), 3)
        cap[f"payload {pp}"]["E_floor_hist"] = {str(k): sum(1 for v in vals if v == k) for k in (0, 45, 90, 145, None)}
    res["firmware_cap_4p15"] = cap
    # duty/thermal: continuous holding torque as a fraction of rated; copper loss ~ (tau/tau_rated)^2
    res["holding_fraction_of_rated"] = dict(worst_box=round(float(abs(T1b).max()) / RATED, 3), copper_loss_rel=round((float(abs(T1b).max()) / RATED) ** 2, 3))
    # ---------------- (c) payload / link length for SF >= 2 in the box (E, S3 worst)
    p0 = np.array([abs(T1b[(pay == 0.0)]).max(), abs(T2b[(pay == 0.0)]).max()]); p5 = np.array([abs(T1b[(pay == 0.5)]).max(), abs(T2b[(pay == 0.5)]).max()])
    slope = (p5 - p0) / 0.5                                  # N.m per kg at the worst pose (linear in payload): conservative using the worst poses of each end
    pm = [(GATE - p0[k]) / slope[k] for k in range(2)]
    # exact: payload is linear in tau at fixed pose -> max payload such that all poses OK
    t1_0 = np.array([T1b[i] for i in range(len(box)) if box[i][4] == 0.0]); t1_5 = np.array([T1b[i] for i in range(len(box)) if box[i][4] == 0.5])
    lam = np.where(abs(t1_5 - t1_0) > 1e-9, (np.sign(t1_5 - t1_0) * GATE - t1_0) / (t1_5 - t1_0) * 0.5, 9.9)      # payload at which tau hits the +/- gate (kg), per pose
    cand = []
    for i in range(len(t1_0)):
        d = (t1_5[i] - t1_0[i]) / 0.5
        if abs(t1_0[i]) > GATE: cand.append(-1.0); continue
        if abs(d) < 1e-9: cand.append(9.9); continue
        cand.append(min(((s * GATE) - t1_0[i]) / d for s in (1, -1) if (((s * GATE) - t1_0[i]) / d) >= 0))
    res["payload_for_SF2"] = dict(S1p_empty_arm_max_Nm=round(float(p0[0]), 2), S1p_per_kg_Nm=round(float(slope[0]), 2), max_payload_kg_all_poses=round(float(min(cand)), 3), note="min over the box poses; negative = the empty arm alone is over the gate")
    # link scale factor (all distal positions scaled, masses fixed)
    ls = {}
    for lam_ in (1.0, 0.9, 0.8, 0.7, 0.6, 0.5):
        T1s, T2s = torques(box, scale=lam_); pyy = np.array([p[4] for p in box])
        ls[str(lam_)] = {str(pp): round(float(abs(T1s[pyy == pp]).max()), 2) for pp in PAY_SET}
    res["link_scale_S1p_max_Nm"] = ls
    res["payload_limit_kg_by_link_scale"] = {k: (round(0.25 * (GATE - v["0.0"]) / (v["0.25"] - v["0.0"]), 3) if v["0.0"] < GATE else 0.0) for k, v in ls.items()}; res["link_scale_gate_4p15_ok_when_below"] = GATE
    # ---------------- (d) mechanism bounds for a th1-only compensator (cassette / clockspring / cam): best possible residual over the box
    th_vals = sorted({p[0] for p in box}); bound = 0.0; per_th = {}
    for t in th_vals:
        idx = [i for i, p in enumerate(box) if p[0] == t]; lo, hi = float(T1b[idx].min()), float(T1b[idx].max()); per_th[str(t)] = (round(lo, 2), round(hi, 2)); bound = max(bound, (hi - lo) / 2.0)
    res["th1_only_compensator"] = dict(ideal_cam_residual_Nm=round(bound, 2), SF=round(RATED / bound, 2), note="ideal cam / arbitrary tau(th1): residual = max over th1 of half the spread over (phi, E, S3, payload); any cassette, clockspring, cam-cord on S1' cannot beat it")
    # best linear torsion spring tau_s = a + b th1 (minimax residual, LP) and best sinusoid K sin(th1 - th0) + c
    ths = np.array([p[0] for p in box]); n = len(box); r = np.radians(ths)
    def minimax(basis):
        Bm = np.array(basis).T; k = Bm.shape[1]; c = np.r_[np.zeros(k), 1.0]; Aub = np.vstack([np.c_[Bm, -np.ones(n)], np.c_[-Bm, -np.ones(n)]]); bub = np.r_[-T1b, T1b]   # |T + B x| <= t
        sol = linprog(c, A_ub=Aub, b_ub=-(-bub) if False else np.r_[-T1b, T1b], bounds=[(None, None)] * k + [(0, None)], method="highs"); return sol.x[:k], sol.x[-1]
    # |T1 + Bx| <= t  ->  Bx - t <= -T1 ; -Bx - t <= T1
    def mm(basis):
        Bm = np.array(basis).T; k = Bm.shape[1]; c = np.r_[np.zeros(k), 1.0]; Aub = np.vstack([np.c_[Bm, -np.ones(n)], np.c_[-Bm, -np.ones(n)]]); bub = np.r_[-T1b, T1b]
        sol = linprog(c, A_ub=Aub, b_ub=bub, bounds=[(None, None)] * k + [(0, None)], method="highs"); return sol.x[:k].tolist(), float(sol.x[-1])
    xl, tl = mm([np.ones(n), r]); xs_, ts_ = mm([np.ones(n), np.cos(r), np.sin(r)])
    res["th1_only_compensator"]["linear_torsion_spring"] = dict(coeff_const_Nm_and_slope_Nm_per_rad=[round(v, 3) for v in xl], residual_Nm=round(tl, 2), SF=round(RATED / tl, 2))
    res["th1_only_compensator"]["sinusoidal_K_sin"] = dict(coeff_c_cos_sin=[round(v, 3) for v in xs_], residual_Nm=round(ts_, 2), SF=round(RATED / ts_, 2))
    # fixed-payload assumption (learned payload, 0.5 kg only / empty only) for the ideal cam
    for pp in (0.0, 0.5):
        b2 = 0.0
        for t in th_vals:
            idx = [i for i, p in enumerate(box) if p[0] == t and p[4] == pp]; b2 = max(b2, (float(T1b[idx].max()) - float(T1b[idx].min())) / 2)
        res["th1_only_compensator"][f"ideal_cam_payload_{pp}_only"] = dict(residual_Nm=round(b2, 2), SF=round(RATED / b2, 2))
    # same fits with the 99 g boots on the hand (boots are a design fact, DESIGN_STATE 12.4)
    T1_save = T1b; T1b = T1bb
    xl2, tl2 = mm([np.ones(n), r]); xs2, ts2 = mm([np.ones(n), np.cos(r), np.sin(r)]); T1b = T1_save
    res["th1_only_compensator"]["with_boots"] = dict(linear=dict(coeff=[round(v, 3) for v in xl2], residual_Nm=round(tl2, 2), SF=round(RATED / tl2, 2)), sinusoid=dict(coeff=[round(v, 3) for v in xs2], residual_Nm=round(ts2, 2), SF=round(RATED / ts2, 2)))
    # linear torsion spring: torque at the ends of the box and stored energy over the stroke (for a negator / clock spring estimate)
    th_lo, th_hi = math.radians(BOX_TH[0]), math.radians(BOX_TH[1]); tq = lambda t: xl[0] + xl[1] * t
    res["th1_only_compensator"]["linear_torsion_spring"].update(torque_at_th_min_Nm=round(tq(th_lo), 2), torque_at_th_max_Nm=round(tq(th_hi), 2), energy_J=round(abs(0.5 * (tq(th_lo) + tq(th_hi)) * (th_hi - th_lo)), 2))
    json.dump(res, open(os.path.join(OUT, "mj11_torque_options.json"), "w"), indent=1, default=float); print(json.dumps(res, indent=1, default=float))
