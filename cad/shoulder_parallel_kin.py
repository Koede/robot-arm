"""Parallel spherical shoulder study: 3-RRR spherical parallel mechanism (SPM), all axes through P.   (new file, 2026-10-26)

Leg i: motor axis u_i (fixed on the chassis, unit vector from P to the motor), proximal link arc alpha1_i from u_i to the
middle joint axis v_i, distal link arc alpha2_i from v_i to the platform axis w_i = R w0_i (w0_i fixed in the arm frame).
Constraint per leg: v_i . w_i = cos(alpha2_i) with v_i = cos(a1) u + sin(a1)(cos(th) e1 + sin(th) e2).
Closed-form IK: A cos th + B sin th = c, A = e1.w, B = e2.w, c = (cos a2 - cos a1 (u.w)) / sin a1  -> th = atan2(B,A) +- acos(c/rho).
Leg feasible iff |a1 - a2| <= angle(u,w) <= a1 + a2 (annulus); the violation in degrees is the coverage error proxy.
Jacobian: theta_dot_i = (v_i x w_i).omega / ((u_i x v_i).w_i)  ->  A omega = Bd theta_dot,  omega = G theta_dot, G = A^-1 Bd.
Static torques: tau_motor = G^T M  (M = moment the platform must resist about P).
Required orientations: R3 = Rz(-S1) Ry(-S2) Rz(S3) (arm_kin.frames), S1 10..140, S2 -30..88, S3 -70..90.

Usage:  python3 shoulder_parallel_kin.py [--search SECONDS] [--seed N] [--report]
Writes stl/shoulder_parallel/kin_search_<seed>.json (search) and stl/shoulder_parallel/kin_result.json (report)."""
import os, sys, json, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "shoulder_parallel")
sys.path.insert(0, HERE)
D2R = np.pi / 180.0
SOFT = dict(S1=(10, 140), S2=(-30, 88), S3=(-70, 90))          # rom_limits.SOFT (single source, copied to keep this file import-light)
AK70 = dict(dia=89.0, length=50.25, mass_g=621.0, rated=8.3, peak=24.8)
XM540_STALL = 10.6

# ---------------------------------------------------------------------------------------------- serial orientation set
def Rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def R3_of(S1, S2, S3): return Rz(-S1 * D2R) @ Ry(-S2 * D2R) @ Rz(S3 * D2R)
def grid_axes(step=10.0):
    ax = {}
    for k, (a, b) in SOFT.items():
        v = list(np.arange(a, b + 1e-9, step));
        if abs(v[-1] - b) > 1e-9: v.append(float(b))
        ax[k] = np.array(v, float)
    return ax
def serial_set(step=10.0):
    ax = grid_axes(step); S1, S2, S3 = np.meshgrid(ax["S1"], ax["S2"], ax["S3"], indexing="ij")
    q = np.stack([S1.ravel(), S2.ravel(), S3.ravel()], 1)
    R = np.array([R3_of(*p) for p in q]); return q, R
def check_against_arm_kin():
    import arm_kin as ak
    rng = np.random.default_rng(0); e = 0.0
    for _ in range(20):
        p = [rng.uniform(*SOFT[k]) for k in ("S1", "S2", "S3")]
        T = ak.frames(dict(S1=p[0], S2=p[1], S3=p[2], E=0, W1=0, W2=0, W3=0, G=50))["M3"][:3, :3]
        e = max(e, np.abs(T - R3_of(*p)).max())
    return e

# ---------------------------------------------------------------------------------------------- SPM
def unit(x): x = np.asarray(x, float); return x / np.linalg.norm(x, axis=-1, keepdims=True)
def lonlat(lon, lat):
    lon = np.asarray(lon) * D2R; lat = np.asarray(lat) * D2R
    return np.stack([np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)], -1)
class SPM:
    """u (3,3) base axes, w0 (3,3) platform axes in the arm frame, a1 (3,), a2 (3,) arcs in rad, sgn (3,) +-1 assembly branch."""
    def __init__(self, u, w0, a1, a2, sgn=(1, 1, 1)):
        self.u = unit(u); self.w0 = unit(w0); self.a1 = np.asarray(a1, float); self.a2 = np.asarray(a2, float); self.sgn = np.asarray(sgn, float)
        self.e1 = np.zeros((3, 3)); self.e2 = np.zeros((3, 3))
        for i in range(3):
            t = np.array([0, 0, 1.0]) if abs(self.u[i, 2]) < 0.9 else np.array([1.0, 0, 0])
            self.e1[i] = unit(np.cross(self.u[i], t)); self.e2[i] = np.cross(self.u[i], self.e1[i])
    def ik(self, R):
        """R (N,3,3). Returns theta (N,3), v (N,3,3), w (N,3,3), viol_deg (N,3)."""
        N = R.shape[0]; th = np.zeros((N, 3)); V = np.zeros((N, 3, 3)); W = np.einsum("nij,kj->nki", R, self.w0); viol = np.zeros((N, 3))
        for i in range(3):
            w = W[:, i]; u = self.u[i]; a1, a2 = self.a1[i], self.a2[i]
            cg = np.clip(w @ u, -1, 1); g = np.arccos(cg)
            viol[:, i] = np.maximum(0, np.maximum(abs(a1 - a2) - g, g - (a1 + a2))) / D2R
            A = w @ self.e1[i]; B = w @ self.e2[i]; rho = np.hypot(A, B)
            c = (np.cos(a2) - np.cos(a1) * cg) / np.sin(a1)
            r = np.clip(c / np.maximum(rho, 1e-12), -1, 1)
            th[:, i] = np.arctan2(B, A) + self.sgn[i] * np.arccos(r)
            V[:, i] = np.cos(a1) * u + np.sin(a1) * (np.cos(th[:, i])[:, None] * self.e1[i] + np.sin(th[:, i])[:, None] * self.e2[i])
        return th, V, W, viol
    def jac(self, V, W):
        """G (N,3,3): omega = G theta_dot.  Also det(A), b (N,3) (serial-singularity factors)."""
        N = V.shape[0]; A = np.cross(V, W)                                          # rows v_i x w_i
        b = np.einsum("nic,nic->ni", np.cross(self.u[None], V), W)
        Bd = np.zeros((N, 3, 3)); Bd[:, [0, 1, 2], [0, 1, 2]] = b
        detA = np.linalg.det(A)
        with np.errstate(all="ignore"):
            G = np.linalg.solve(A, Bd)
        return G, detA, b
    def fk(self, theta, R0, iters=40):
        """Newton FK from an initial guess R0: finds R with v_i . (R w0_i) = cos a2_i for the given motor angles."""
        R = R0.copy()
        v = np.array([np.cos(self.a1[i]) * self.u[i] + np.sin(self.a1[i]) * (np.cos(theta[i]) * self.e1[i] + np.sin(theta[i]) * self.e2[i]) for i in range(3)])
        for _ in range(iters):
            w = R @ self.w0.T; h = np.array([v[i] @ w[:, i] - np.cos(self.a2[i]) for i in range(3)])
            if np.abs(h).max() < 1e-13: break
            Jm = np.array([np.cross(w[:, i], v[i]) for i in range(3)])            # dh = v.(dw) = omega . (w x v)
            dw = np.linalg.solve(Jm, -h); ang = np.linalg.norm(dw)
            if ang > 1e-15:
                k = dw / ang; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); dR = np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K
                R = dR @ R
        return R
def rot_angle_deg(Ra, Rb):
    M = np.einsum("...ji,...jk->...ik", Ra, Rb); c = np.clip((np.trace(M, axis1=-2, axis2=-1) - 1) / 2, -1, 1); return np.degrees(np.arccos(c))

# ---------------------------------------------------------------------------------------------- design vector
# x = [u lon,lat x3 (6)] [w0 lon,lat x3 (6)] [a1 x3 (deg)] [a2 x3 (deg)] [sgn bits x3]  (21)
def decode(x):
    u = lonlat(x[0:6:2], x[1:6:2]); w0 = lonlat(x[6:12:2], x[7:12:2])
    return SPM(u, w0, np.array(x[12:15]) * D2R, np.array(x[15:18]) * D2R, np.where(np.array(x[18:21]) > 0.5, 1, -1))
def arc_min_angle_to_axis(m, nsamp=5):
    """smallest angle (deg) between the arm axis (platform +z) and the distal-link arcs v_i -> w0_i, evaluated at the home orientation only (arm frame, R=I);
    the distal arc is rigid with respect to the middle joint, so its angle to the arm axis varies with pose: evaluated per pose in the caller."""
    raise NotImplementedError   # (kept for API note; the per-pose version is inside pose_metrics)
def pose_metrics(m, R, with_arc=True):
    th, V, W, viol = m.ik(R); G, detA, b = m.jac(V, W)
    out = dict(th=th, viol=viol, detA=detA, b=b)
    sv = np.linalg.svd(np.nan_to_num(G, nan=0.0, posinf=1e6, neginf=-1e6), compute_uv=False)
    out["smin"] = sv[:, -1]; out["smax"] = sv[:, 0]; out["G"] = G
    if with_arc:
        # distal arc points: slerp v_i -> w_i; angular distance of the arc to the arm axis (z of the platform frame = R e_z)
        ez = R[:, :, 2]; amin = np.full(R.shape[0], 180.0)
        for i in range(3):
            v = V[:, i]; w = W[:, i]; om = np.arccos(np.clip(np.einsum("ni,ni->n", v, w), -1, 1))
            for t in np.linspace(0, 1, 6):
                s = np.sin(om)
                p = (np.sin((1 - t) * om)[:, None] * v + np.sin(t * om)[:, None] * w) / np.maximum(s, 1e-9)[:, None]
                amin = np.minimum(amin, np.degrees(np.arccos(np.clip(np.einsum("ni,ni->n", p, ez), -1, 1))))
        out["arm_ang"] = amin
    return out
def objective(x, Rs, packaging=True, verbose=False):
    m = decode(x); met = pose_metrics(m, Rs)
    vmax = met["viol"].max(); vmean = met["viol"].mean()
    f = 50.0 * vmax + 5.0 * vmean
    # singularities: sign constancy of det A and of the serial factors b_i
    sA = np.sign(met["detA"]); f += 200.0 * min((sA > 0).mean(), (sA < 0).mean())
    for i in range(3):
        sb = np.sign(met["b"][:, i]); f += 200.0 * min((sb > 0).mean(), (sb < 0).mean())
    smin = np.nanmin(met["smin"]); smax = np.nanmax(met["smax"])
    f += 80.0 * max(0.0, 0.30 - smin) + 8.0 * max(0.0, smax - 3.0)
    # arm clearance: the distal links must stay >= 38 deg from the arm axis (arm radius ~ 40 + tube at R 80)
    f += 6.0 * max(0.0, 38.0 - met["arm_ang"].min())
    ang = lambda a, b: np.degrees(np.arccos(np.clip(a @ b, -1, 1)))
    if packaging:
        for i in range(3):
            # motor axes in the lower-rear region behind/below P (z <= -0.25, x >= -0.35); spacing >= 60 deg between motor axes
            f += 300.0 * max(0.0, m.u[i, 2] + 0.25) + 300.0 * max(0.0, -0.35 - m.u[i, 0])
            for j in range(i + 1, 3):
                f += 1.0 * max(0.0, 60.0 - ang(m.u[i], m.u[j]))
                f += 1.0 * max(0.0, 45.0 - ang(m.w0[i], m.w0[j]))
    if verbose: return f, dict(vmax=vmax, smin=smin, smax=smax)
    return f
def bounds():
    b = []
    for _ in range(3): b += [(-180, 180), (-90, 90)]
    for _ in range(3): b += [(-180, 180), (-90, 90)]
    b += [(35, 100)] * 6 + [(0, 1)] * 3
    return b

def search(seconds=240, seed=0, step=20.0, packaging=True):
    from scipy.optimize import differential_evolution
    q, R = serial_set(step); t0 = time.time(); best = [1e18, None]
    def cb(xk, convergence=None):
        return time.time() - t0 > seconds
    def f(x): return objective(np.where(np.arange(21) >= 18, np.round(x), x), R, packaging)
    r = differential_evolution(f, bounds(), seed=seed, popsize=18, maxiter=100000, tol=0, mutation=(0.4, 0.9), recombination=0.8, callback=cb, polish=False, updating="immediate", init="sobol")
    x = r.x.copy(); x[18:] = np.round(x[18:])
    return x, r.fun
def local_polish(x, Rs, packaging=True, iters=3000):
    from scipy.optimize import minimize
    xs = x.copy(); sg = xs[18:].copy()
    g = lambda y: objective(np.concatenate([y, sg]), Rs, packaging)
    r = minimize(g, xs[:18], method="Nelder-Mead", options=dict(maxiter=iters, xatol=1e-3, fatol=1e-4, adaptive=True))
    return np.concatenate([r.x, sg]), r.fun

# ---------------------------------------------------------------------------------------------- nearest reachable orientation (true coverage error)
def nearest_reachable_error(m, Rbad):
    """For each infeasible target, minimise the rotation angle to a reachable orientation (3-parameter constrained problem)."""
    from scipy.optimize import minimize
    errs = []
    for R0 in Rbad:
        def rot(p):
            a = np.linalg.norm(p)
            if a < 1e-12: return np.eye(3)
            k = p / a; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); return np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
        def viol(p): return m.ik((rot(p) @ R0)[None])[3].max()
        def cost(p): return np.degrees(np.linalg.norm(p)) + 1e3 * viol(p)
        best = np.inf
        for s in range(6):
            p0 = np.random.default_rng(s).normal(size=3) * 0.1
            r = minimize(cost, p0, method="Nelder-Mead", options=dict(maxiter=500, xatol=1e-4, fatol=1e-5))
            if viol(r.x) < 1e-3: best = min(best, np.degrees(np.linalg.norm(r.x)))
        errs.append(best)
    return np.array(errs)

# ---------------------------------------------------------------------------------------------- gravity / torque model
def gravity_moments(S12, S3s, E_list=(0, 30, 60, 90, 120, 145), W1=(-80, 0, 80), W2=(-70, 0, 80), W3=(-20, 0, 30), pay=(0.0, 0.5)):
    """Gravity moment vectors about P (N m, world frame) for every (S1,S2) in S12, S3 in S3s and the arm configs (E, wrist, payload).
    Uses the balancer first-moment model (balancer_calc.c_vec: kg m in the S2-output frame, S2 = 0). Returns dict with arrays."""
    import balancer_calc as bc
    G9 = 9.81
    cfg = [(e, w1, w2, w3, p) for e in E_list for w1 in W1 for w2 in W2 for w3 in W3 for p in pay]
    return cfg, G9, bc
def moment_set(q_list, cfg, G9, bc):
    """q_list (N,3) -> M (N, ncfg, 3) world gravity moments about P, also c0 (first moment in the S2-output frame)."""
    N = len(q_list); M = np.zeros((N, len(cfg), 3))
    cs = {}
    for s3 in np.unique(q_list[:, 2]):
        cs[s3] = np.array([bc.c_vec(float(e), p, float(s3), (float(w1), float(w2), float(w3))) for (e, w1, w2, w3, p) in cfg])      # (ncfg,3) in R2 frame
    for n, (s1, s2, s3) in enumerate(q_list):
        R2 = Rz(-s1 * D2R) @ Ry(-s2 * D2R); cw = cs[s3] @ R2.T                      # world first moments
        M[n, :, 0] = -G9 * cw[:, 1]; M[n, :, 1] = G9 * cw[:, 0]                       # M = c x (0,0,-g)
    return M

# ---------------------------------------------------------------------------------------------- report
def report(x, step=10.0, tag="best"):
    os.makedirs(OUT, exist_ok=True)
    m = decode(x); q, R = serial_set(step); met = pose_metrics(m, R)
    th, viol = met["th"], met["viol"]; vmax = viol.max(axis=1)
    infeas = vmax > 1e-6
    res = dict(tag=tag, step_deg=step, n_poses=int(len(q)), arm_kin_match_max_abs=float(check_against_arm_kin()))
    res["design"] = dict(u_motor_axes=m.u.round(5).tolist(), w0_platform_axes_arm_frame=m.w0.round(5).tolist(), alpha1_deg=(m.a1 / D2R).round(2).tolist(),
                         alpha2_deg=(m.a2 / D2R).round(2).tolist(), assembly_sign=m.sgn.tolist(), x=[float(v) for v in x],
                         motor_axis_separation_deg=[float(np.degrees(np.arccos(np.clip(m.u[i] @ m.u[j], -1, 1)))) for i, j in ((0, 1), (0, 2), (1, 2))],
                         platform_axis_separation_deg=[float(np.degrees(np.arccos(np.clip(m.w0[i] @ m.w0[j], -1, 1)))) for i, j in ((0, 1), (0, 2), (1, 2))],
                         platform_axis_angle_to_arm_z_deg=[float(np.degrees(np.arccos(m.w0[i, 2]))) for i in range(3)])
    res["coverage"] = dict(n_infeasible=int(infeas.sum()), frac_covered=float(1 - infeas.mean()), max_leg_violation_deg=float(viol.max()))
    if infeas.any():
        idx = np.where(infeas)[0]; sel = idx[np.argsort(-vmax[idx])][:60]
        err = nearest_reachable_error(m, R[sel]); res["coverage"]["max_rotation_error_deg_worst60"] = float(np.nanmax(err)); res["coverage"]["mean_rotation_error_deg_worst60"] = float(np.nanmean(err))
        res["coverage"]["worst_poses_S1S2S3"] = q[sel[:8]].tolist()
        # which joints cause the misses
        res["coverage"]["infeasible_S1_hist"] = {str(int(a)): int((q[idx, 0] == a).sum()) for a in np.unique(q[:, 0])}
        res["coverage"]["infeasible_S2_hist"] = {str(int(a)): int((q[idx, 1] == a).sum()) for a in np.unique(q[:, 1])}
        res["coverage"]["infeasible_S3_hist"] = {str(int(a)): int((q[idx, 2] == a).sum()) for a in np.unique(q[:, 2])}
    else: res["coverage"]["max_rotation_error_deg"] = 0.0
    ok = ~infeas
    sA = np.sign(met["detA"][ok])
    res["jacobian"] = dict(sigma_min_G=float(met["smin"][ok].min()), sigma_max_G=float(met["smax"][ok].max()), median_sigma_min=float(np.median(met["smin"][ok])),
                           cond_max=float((met["smax"][ok] / np.maximum(met["smin"][ok], 1e-12)).max()), detA_sign_changes=bool(sA.min() != sA.max()), min_abs_detA=float(abs(met["detA"][ok]).min()),
                           min_abs_b=[float(abs(met["b"][ok][:, i]).min()) for i in range(3)], note="G maps motor rate to platform rate (omega = G theta_dot); sigma_min is platform rad per motor rad in the weakest direction, sigma_max sets the torque amplification (tau_m = G^T M)")
    # assembly modes: how many of the 8 sign branches are feasible at all poses + same-determinant sign
    modes = []
    for s in range(8):
        sg = np.array([1 if (s >> i) & 1 else -1 for i in range(3)]); mm = SPM(m.u, m.w0, m.a1, m.a2, sg); mt = pose_metrics(mm, R, with_arc=False)
        okm = mt["viol"].max() < 1e-6; sa = np.sign(mt["detA"]); sb = [np.sign(mt["b"][:, i]) for i in range(3)]
        sel_ok = ~(mt["viol"].max(axis=1) > 1e-6)
        modes.append(dict(sgn=sg.tolist(), poses_ok=int(sel_ok.sum()), min_sigma_min=float(np.nanmin(mt["smin"][sel_ok])) if sel_ok.any() else None,
                          detA_sign_flips=bool(sa[sel_ok].min() != sa[sel_ok].max()) if sel_ok.any() else None))
    res["assembly_modes_of_8_branches"] = modes
    # motor travel
    tt = th.copy(); ref = th[ok].mean(0) if ok.any() else th.mean(0); tw = (tt - ref + np.pi) % (2 * np.pi) - np.pi + ref
    sel = ok
    res["motor_travel_deg"] = dict(per_leg_min=(tw[sel].min(0) / D2R).round(1).tolist(), per_leg_max=(tw[sel].max(0) / D2R).round(1).tolist(), per_leg_range=((tw[sel].max(0) - tw[sel].min(0)) / D2R).round(1).tolist(),
                                   note="angle of the proximal link about the motor axis, theta = 0 at e1 (arbitrary reference); the range is what matters. Wrapped about the mean; a range > 340 deg would indicate a branch jump")
    res["arm_clearance_deg"] = dict(min_angle_distal_arc_to_arm_axis=float(met["arm_ang"][ok].min()))
    # gravity torque transmission
    q_ok = q[ok]
    S12 = np.unique(q_ok[:, :2], axis=0)
    cfg, G9, bc = gravity_moment_cfg()
    sub = q_ok[(q_ok[:, 2] % 20 == 10) | (q_ok[:, 2] % 10 == 0)]
    # use the full ok set for torque (vectorised per pose over configs)
    Msub = moment_set(q_ok, cfg, G9, bc)                                              # (N,ncfg,3)
    Gk = met["G"][ok]
    tau = np.einsum("nji,ncj->nci", Gk, Msub)                                          # tau_m = G^T M
    tmax = np.abs(tau).max(axis=(0, 1)); tpeak = np.abs(tau).max()
    Mn = np.linalg.norm(Msub, axis=2)
    res["torque"] = dict(n_configs=len(cfg), max_gravity_moment_Nm=float(Mn.max()), max_motor_torque_per_leg_Nm=tmax.round(2).tolist(), max_motor_torque_Nm=float(tpeak),
                         SF_rated_AK70=float(AK70["rated"] / tpeak), SF_peak_AK70=float(AK70["peak"] / tpeak), amplification_max=float((np.abs(tau).max(axis=2) / np.maximum(Mn, 1e-9)).max()),
                         median_motor_torque_worst_cfg=float(np.median(np.abs(tau).max(axis=2).max(axis=1))))
    # serial comparison at the same poses (S2 axis and S3 axis components)
    t2 = []; t3 = []; t1 = []
    for n, (s1, s2, s3) in enumerate(q_ok):
        R1 = Rz(-s1 * D2R); R2 = R1 @ Ry(-s2 * D2R); a2 = R1 @ np.array([0, 1.0, 0]); a3 = R2 @ np.array([0, 0, 1.0])
        t1.append(np.abs(Msub[n] @ np.array([0, 0, 1.0])).max()); t2.append(np.abs(Msub[n] @ a2).max()); t3.append(np.abs(Msub[n] @ a3).max())
    res["serial_gravity_torque_Nm"] = dict(S1_max=float(max(t1)), S2_max=float(max(t2)), S3_max=float(max(t3)), balancer_calc_S2_peak=6.554)
    # spatial zero-length-spring balancer: M_b = K (e x z); minimise max |M + M_b|
    ez_arm = R[ok][:, :, 2]; cr = np.cross(ez_arm, np.array([0, 0, 1.0]))               # (N,3)
    from scipy.optimize import minimize_scalar
    def resid(K): return np.linalg.norm(Msub + K * cr[:, None, :], axis=2).max()
    r = minimize_scalar(resid, bounds=(0.0, 12.0), method="bounded")
    Kopt = float(r.x); Rb = Msub + Kopt * cr[:, None, :]
    taub = np.einsum("nji,ncj->nci", Gk, Rb)
    res["spatial_balancer"] = dict(K_Nm=Kopt, max_residual_moment_Nm=float(np.linalg.norm(Rb, axis=2).max()), max_motor_torque_with_balancer_Nm=float(np.abs(taub).max()),
                                   SF_rated_with_balancer=float(AK70["rated"] / np.abs(taub).max()), max_gravity_moment_no_balancer=float(Mn.max()),
                                   note="zero-length spring from a base point A on the +z axis through P to a point B on the arm axis gives M_b = K e x z for any S1/S2 (S1-independent); same residual physics as the S2 balancer plus the S3/E/wrist offsets")
    json.dump(res, open(os.path.join(OUT, f"kin_result_{tag}.json"), "w"), indent=1)
    return res
_CFG = None
def gravity_moment_cfg():
    global _CFG
    if _CFG is None: _CFG = gravity_moments(None, None)
    return _CFG
if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument("--search", type=float, default=0); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--report", default=""); ap.add_argument("--nopack", action="store_true")
    a = ap.parse_args(); os.makedirs(OUT, exist_ok=True)
    print("arm_kin match (max abs):", check_against_arm_kin())
    if a.search > 0:
        q, R = serial_set(20.0)
        x, f = search(a.search, a.seed, step=30.0, packaging=not a.nopack)
        x2, f2 = local_polish(x, R, packaging=not a.nopack, iters=1500)
        if f2 < f: x, f = x2, f2
        _, info = objective(x, R, not a.nopack, True)
        json.dump(dict(x=[float(v) for v in x], f=float(f), seed=a.seed, info={k: float(v) for k, v in info.items()}, packaging=not a.nopack), open(os.path.join(OUT, f"kin_search_{a.seed}{'_free' if a.nopack else ''}.json"), "w"), indent=1)
        print("search seed", a.seed, "f", f, info)
    if a.report:
        d = json.load(open(a.report)); r = report(np.array(d["x"]), 10.0, tag=os.path.basename(a.report).replace("kin_search_", "").replace(".json", ""))
        print(json.dumps({k: r[k] for k in ("coverage", "jacobian", "motor_travel_deg", "torque", "spatial_balancer")}, indent=1))
