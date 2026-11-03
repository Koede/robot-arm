"""MJ11 gravity torque and balancer / torque-gate options (2026-11-03).  Robot frame, N.m, mm.  Pad (-60,-190), standoff 12.
Inputs: free joint region of the MJ11 sweep (stl/j1r2_mounts/MJ11/free_region.npz, written by mj11_free_region.py), the J1 rev 2 mass model (j1r2_bal_model.py: arm beyond S2 1489 g, payload at the 575 mm tip).
Sections: (1) unbalanced gravity torque over four envelopes; (2) option (a) zero-length-spring refit per anchor sector (DE) + spring/tube sizing;
(3) option (b) torque-cap envelope and thermal estimate; (4) option (c) payload / link-length scaling; (5) option (d) th1-only / phi-only ideal cassette lower bounds.
usage: J1R2_ROOT=<root> python3 mj11_balancer.py [sections]  -> stl/j1r2_mounts/MJ11/balancer_options.json"""
import os, sys, json, time, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy import ndimage as ndi
from scipy.optimize import differential_evolution
import j1r2_params as P, j1r2_bal_model as B
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))); OUTD = os.path.join(REPO, "stl", "j1r2_mounts", "MJ11")
RATED = 8.3; TARGET = RATED / 2.0
B.D0 = P.derive(-60.0, -190.0, 12.0); B.P1 = np.array([B.D0["PX"], B.D0["y1"], B.D0["z1"]]); P1 = B.P1
Z = np.load(os.path.join(OUTD, "free_region.npz")); REACH, S1V, S2V = Z["reach"], Z["S1"], Z["S2"]
SOFT = ndi.binary_erosion(REACH, structure=np.ones((3, 3), bool), border_value=1)
BOX = np.array([[(-95 <= t <= 5) and (-15 <= p <= 17.4 + 1e-6) for p in S2V] for t in S1V]); FULL = np.ones_like(REACH)
ENV = {"full domain (old gravity study)": FULL, "reach-free (hard, S2 -25..40)": REACH, "soft free region (1 cell inside)": SOFT, "passive box S1' -95..5, S2 -15..17.4": BOX}
VAR = [(E, S3, pay) for E in (0, 45, 90, 145) for S3 in (-70, 0, 90) for pay in (0.0, 0.5)]
def grav_table(mask, seg_scale=1.0, pays=(0.0, 0.5)):
    """returns arrays over (cell, variant): th, ph, E, S3, pay, u1, u2 (N.m)."""
    rows = []
    for i, t in enumerate(S1V):
        for j, p in enumerate(S2V):
            if not mask[i, j]: continue
            for (E, S3, pay) in VAR:
                if pay not in pays: continue
                t1, t2 = B.grav_moments(t, p, E, S3, pay); rows.append((i, j, t, p, E, S3, pay, t1 * 1e-3, t2 * 1e-3))
    return np.array(rows)
def summarize(T):
    a1, a2 = np.abs(T[:, 7]), np.abs(T[:, 8]); i1, i2 = int(a1.argmax()), int(a2.argmax())
    return dict(S1p_max=round(float(a1[i1]), 2), S1p_at=[float(v) for v in T[i1, 2:7]], S2_max=round(float(a2[i2]), 2), S2_at=[float(v) for v in T[i2, 2:7]], SF_S1p=round(RATED / float(a1[i1]), 2), SF_S2=round(RATED / float(a2[i2]), 2), n_poses=int(len(T)))
# ------------------------------------------------------------------ (a) zero-length-spring refit
def cell_frames(mask):
    idx = [(i, j) for i in range(len(S1V)) for j in range(len(S2V)) if mask[i, j]]; FA = []; Ss = []
    for i, j in idx:
        F0, FA_, s, a, d = P.shoulder_frames(B.D0, S1V[i], S2V[j]); FA.append(FA_); Ss.append(s)
    return idx, np.array(FA), np.array(Ss)
def make_problem(mask):
    idx, FA, Ss = cell_frames(mask); cid = {c: n for n, c in enumerate(idx)}; T = grav_table(mask); cell = np.array([cid[(int(r[0]), int(r[1]))] for r in T])
    return dict(idx=idx, FA=FA, S=Ss, T=T, cell=cell, u1=T[:, 7], u2=T[:, 8])
def spring_resid(x, pr, ret=False, extra_g=0.052):
    A = P1 + np.array(x[0:3]); Bl = np.array(x[3:6]); k, L0 = x[6], x[7]; FA, S = pr["FA"], pr["S"]
    Bw = np.einsum("nij,j->ni", FA[:, :3, :3], Bl) + FA[:, :3, 3]; v = A[None] - Bw; d = np.linalg.norm(v, axis=1); F = (k * (d - L0))[:, None] * v / d[:, None]
    mom = np.cross((Bw - P1) * 1e-3, F); m1, m2 = mom[:, 0], np.einsum("ni,ni->n", mom, S)
    g = np.cross((Bw - P1) * 1e-3, np.array([0, 0, -extra_g * 9.81])); g1, g2 = g[:, 0], np.einsum("ni,ni->n", g, S)          # arm-side hardware (collar + pin + cord) at B
    c = pr["cell"]; r1 = np.abs(pr["u1"] + g1[c] + m1[c]); r2 = np.abs(pr["u2"] + g2[c] + m2[c])
    if ret: return r1, r2, d, k * (d - L0)
    return r1, r2, d, k * (d - L0)
def cost(x, pr, psi, fcap, rlim):
    r1, r2, d, Fc = spring_resid(x, pr); pen = 0.0
    pen += max(0, d.max() - 200.0) * 0.05 + max(0, (d.max() - d.min()) - 140.0) * 0.05 + max(0, x[7] - d.min() + 2.0) * 0.5 + max(0, Fc.max() - fcap) * 0.03
    rr = math.hypot(x[1], x[2]); pen += max(0, rr - rlim[1]) * 0.2 + max(0, rlim[0] - rr) * 0.2
    ang = math.degrees(math.atan2(x[2], x[1])) % 360; da = abs((ang - psi + 180) % 360 - 180); pen += max(0, da - 8.0) * 0.1           # anchor pinned to a sector (+-8 deg)
    return max(r1.max(), r2.max()) + pen
def refit(pr, psi, fcap=130.0, seed=1, rlim=(70.0, 100.0), maxiter=120):
    a = math.radians(psi); r0 = 85.0
    bnds = [(-52, -10), (-110, 110), (-110, 110), (-40, 40), (-30, 30), (20, 160), (0.05, 3.0), (0, 80)]
    bnds[1] = (r0 * math.cos(a) - 30, r0 * math.cos(a) + 30); bnds[2] = (r0 * math.sin(a) - 30, r0 * math.sin(a) + 30)
    r = differential_evolution(cost, bnds, args=(pr, psi, fcap, rlim), seed=seed, popsize=14, maxiter=maxiter, tol=1e-7, polish=False, mutation=(0.5, 1.0), recombination=0.8)
    r1, r2, d, Fc = spring_resid(r.x, pr); return dict(psi=psi, x=[float(v) for v in r.x], S1p_res=float(r1.max()), S2_res=float(r2.max()), SF=float(RATED / max(r1.max(), r2.max())), cord_min=float(d.min()), cord_max=float(d.max()), F_cord_max=float(Fc.max()), F_cord_min=float(Fc.min()), cost=float(r.fun))
def spring_design(k_cord, d_min, d_max, F_cord_max, tackle=2, d_w=3.5, Dm=36.0, L0=0.0):
    """compression spring in a tube, 2:1 tackle, A228 (Gm 80 GPa, same recipe as j1r2_balancer.py)."""
    ks = tackle ** 2 * k_cord; Gm = 80000.0; xs_max = max((d_max - L0) / tackle, 0.0); Na = Gm * d_w ** 4 / (8 * Dm ** 3 * ks); Nt = Na + 2; Ls = Nt * d_w; L_free = Ls + 1.12 * xs_max + xs_max; C = Dm / d_w
    KB = (4 * C + 2) / (4 * C - 3); Fs = ks * xs_max; tau = KB * 8 * Fs * Dm / (math.pi * d_w ** 3); Sut = 2211.0 / d_w ** 0.145; mass = math.pi * Dm * Nt * math.pi / 4 * d_w ** 2 * 7.85e-3
    return dict(k_spring=round(ks, 3), Na=round(Na, 2), L_free=round(L_free, 1), F_spring_max=round(Fs, 1), tau_over_Sut=round(tau / Sut, 3), mass_g=round(mass, 1), tube_len_mm=round(L_free + 12.0, 0))
# ------------------------------------------------------------------ (c) scaling: arm links, payload
def scaled_arm(f):
    """scale the distal geometry (everything beyond the S3 stage) along the arm axis by f; masses unchanged (conservative: shorter links are lighter)."""
    old = (list(B.SEG), B.TIP, B.ELB.copy())
    B.SEG = [(n, m, (p[0], p[1], p[2] if i == 0 else p[2] * f), re) for i, (n, m, p, re) in enumerate(old[0])]; B.TIP = (0, 57.0, (230.0 + 345.0) * f if False else 230.0 * f + 345.0 * f); B.ELB = np.array([0, 0, 230.0 * f])
    return old
def restore(old): B.SEG, B.TIP, B.ELB = old
if __name__ == "__main__":
    sec = sys.argv[1] if len(sys.argv) > 1 else "1,2,3,4,5"; out = {}; t0 = time.time()
    if "1" in sec:
        out["envelopes"] = {k: summarize(grav_table(m)) for k, m in ENV.items()}; print("1", json.dumps(out["envelopes"], indent=0)[:1500], flush=True)
    if "2" in sec:
        pr = make_problem(SOFT); res = []
        for psi in range(0, 360, 30):
            r = refit(pr, psi, 130.0); r["spring"] = spring_design(r["x"][6], r["cord_min"], r["cord_max"], r["F_cord_max"], L0=r["x"][7]); res.append(r); print("2 psi", psi, round(r["S1p_res"], 2), round(r["S2_res"], 2), "F", round(r["F_cord_max"], 1), "t", round(time.time() - t0), flush=True)
        out["refit_by_sector"] = res
    if "3" in sec:
        T = grav_table(SOFT); b3 = {}
        for lim in (4.15, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.3):
            row = {}
            for pay in (0.0, 0.5):
                m = (T[:, 6] == pay); ok = (np.abs(T[:, 7]) <= lim) & (np.abs(T[:, 8]) <= lim); row[f"pose_fraction_ok_pay{pay}"] = round(float(ok[m].mean()), 3)
                cells_ok = set(); allc = set();
                for r_, o in zip(T[m], ok[m]): allc.add((int(r_[0]), int(r_[1])));
                bad = {(int(r_[0]), int(r_[1])) for r_, o in zip(T[m], ok[m]) if not o}; row[f"cell_fraction_all_E_S3_ok_pay{pay}"] = round(1 - len(bad) / len(allc), 3)
            row["hold_heat_vs_rated"] = round((lim / RATED) ** 2, 2); b3[str(lim)] = row
        out["torque_cap_envelope"] = b3; print("3", b3, flush=True)
    if "4" in sec:
        c4 = {}
        def maxtq(f, pay):
            old = scaled_arm(f); m1 = m2 = 0.0
            for i, t in enumerate(S1V):
                for j, p in enumerate(S2V):
                    if not SOFT[i, j]: continue
                    for E in (0, 45, 90, 145):
                        for S3 in (-70, 0, 90):
                            t1, t2 = B.grav_moments(t, p, E, S3, pay); m1 = max(m1, abs(t1) * 1e-3); m2 = max(m2, abs(t2) * 1e-3)
            restore(old); return [round(m1, 2), round(m2, 2)]
        for f in (1.0, 0.9, 0.8, 0.7, 0.6):
            row = {f"pay{pay}": maxtq(f, pay) for pay in (0.0, 0.25, 0.5)}; c4[str(f)] = row; print("4", f, row, flush=True)
        out["links_payload_scaling"] = c4
    if "5" in sec:
        T = grav_table(SOFT); c5 = {}
        # th1-only element: at each th1 the best constant = mid-range of u1 over every other coordinate -> residual = max half-range
        th = np.unique(T[:, 2]); hr1 = max((T[T[:, 2] == t_, 7].max() - T[T[:, 2] == t_, 7].min()) / 2 for t_ in th)
        ph = np.unique(T[:, 3]); hr2 = max((T[T[:, 3] == p_, 8].max() - T[T[:, 3] == p_, 8].min()) / 2 for p_ in ph)
        # S2 element that may also see th1 (e.g. gimballed or world-anchored): half-range over (E, S3, pay) at fixed (th1, phi)
        hr2b = 0.0; hr1b = 0.0
        for (i, j) in {(int(r[0]), int(r[1])) for r in T}:
            m = (T[:, 0] == i) & (T[:, 1] == j); hr2b = max(hr2b, (T[m, 8].max() - T[m, 8].min()) / 2); hr1b = max(hr1b, (T[m, 7].max() - T[m, 7].min()) / 2)
        # best sinusoid K sin(th1 + d) fit for S1' and best K sin(phi) for S2 (simple spiral/cam with the usual law)
        best = (9e9, None)
        for K in np.arange(0, 9, 0.1):
            for dl in np.arange(-90, 91, 3):
                r = np.abs(T[:, 7] - K * np.sin(np.radians(T[:, 2] + dl))).max()
                if r < best[0]: best = (r, (round(float(K), 1), int(dl)))
        best2 = (9e9, None)
        for K in np.arange(0, 9, 0.1):
            r = np.abs(T[:, 8] - K * np.sin(np.radians(T[:, 3]))).max()
            if r < best2[0]: best2 = (r, round(float(K), 1))
        c5 = dict(S1p_th1_only_ideal_cam_residual=round(float(hr1), 2), S2_phi_only_ideal_cam_residual=round(float(hr2), 2), S1p_ideal_if_pose_known_not_E_S3_halfrange=round(float(hr1b), 2), S2_ideal_if_th1_phi_known=round(float(hr2b), 2),
                  S1p_sinusoid_K_sin_th_plus_d=dict(residual=round(float(best[0]), 2), K_d=best[1]), S2_K_sin_phi=dict(residual=round(float(best2[0]), 2), K=best2[1]))
        out["cassette_bounds"] = c5; print("5", c5, flush=True)
    p = os.path.join(OUTD, "balancer_options.json"); old = json.load(open(p)) if os.path.exists(p) else {}; old.update(out); json.dump(old, open(p, "w"), indent=1, default=float); print("done", round(time.time() - t0), "s")
