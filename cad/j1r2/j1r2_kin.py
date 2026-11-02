"""J1 rev 2: whole-arm kinematics (screw theory), groups, distal proxies, limits study, singularity guard, speed caps, DLS shoulder map,
gravity torques, head/neck screen, workspace comparison.  Robot frame, mm, deg.  Public API at the bottom of this docstring.
  Kin(D=None).group_transforms(q) -> {M1,M2,M3,ME,MW1,MW2,MW3,M4(=ME),M5(=MW3)}  4x4 (robot frame) of each rigid group relative to its HOME-built pose
  Kin.tip(q), Kin.arm_frame(q), Kin.proxies() -> {group: {name: mesh}} (home pose, robot frame), Kin.axes_now(q)
  limits_study(), guard(q_dict, omega) / singularity_margin(q), dls_shoulder(th, phi, omega, lam), gravity_torques(th, phi, payload),
  speed_caps(n), headneck_scan(), workspace_compare(n), BASE (baseline chain) for comparison.
q = dict(S1p, S2, S3, E, W1, W2, W3) in deg.  Home = th1 = phi = 0 (arm points +z, S2 axis +y).  Joint axes below are in the HOME configuration (product of exponentials):
  T(q) = e^{xi_S1p th} e^{xi_S2 phi} e^{xi_S3 S3} e^{xi_E E} e^{xi_W1 W1} e^{xi_W2 W2} e^{xi_W3 W3} M,   e^{xi q} = Tr(p) Rot(w, q) Tr(-p).
  S1' : w = +x through (PX, y1, z1);  S2 : w = -y through (PX, y1, z1)   (d = a cos(phi) - xhat sin(phi), baseline-like 'positive swings forward = -x');
  S3 : +z through O0 = (PX, y1 + T_ARM, z1);  E : -y through O0 + (0,0,230) (E>0 swings the forearm toward -x);  W1 : +z through O0 + (0,D,0);
  W2 : -y through O0 + (0,D,Z_W2);  W3 : +x through O0 + (0,D,Z_W3).   Hand: wrist-to-tip 214 mm (rev B = rev C), tip at local z 624."""
import sys, os, json, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clib as L, j1r2_params as P
from clib import cyl, box, sph, U
from scipy.spatial import cKDTree
OUT = os.path.join(L.ROOT, "stl", "j1r2_kin"); os.makedirs(os.path.join(OUT, "proxies"), exist_ok=True)
DOFF, Z_E, Z_W2, Z_W3, Z_TIP = 57.0, 230.0, 410.0, 455.0, 624.0           # hand rev B/C: W2 at 410, tip 214 mm beyond (SPEC_hand_C: tip z 624.0)
HAND = dict(x=(-27.5, 27.5), y=(DOFF - 60.0, DOFF + 60.0), z=(491.6, Z_TIP))   # rev C envelope x -26.8..27.5, u -45..60 (sign of u unknown -> symmetric +-60)
JN = ["S1p", "S2", "S3", "E", "W1", "W2", "W3"]
def rodrigues(w, ang):
    w = np.asarray(w, float); K = np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]]); c, s = np.cos(ang), np.sin(ang)
    return np.eye(3) * c + s * K + (1 - c) * np.outer(w, w)
def expo(w, p, deg):
    R = rodrigues(w, np.radians(deg)); T = np.eye(4); T[:3, :3] = R; T[:3, 3] = np.asarray(p, float) - R @ np.asarray(p, float); return T
class Chain:
    """product-of-exponentials chain: screws = [(w, p)] in HOME configuration, M = home pose of the tip point."""
    def __init__(self, screws, tip_home, names=JN): self.sc = screws; self.tip_home = np.asarray(tip_home, float); self.names = list(names)
    def cumulative(self, q):
        T = np.eye(4); out = {}
        for (w, p), n in zip(self.sc, self.names): T = T @ expo(w, p, q[n]); out[n] = T.copy()
        return out
    def tip(self, q): return (self.cumulative(q)[self.names[-1]] @ np.r_[self.tip_home, 1.0])[:3]
    def axes_now(self, q):
        """(point, unit dir) of every joint axis in the current pose (axis i is moved by the joints before it)."""
        T = np.eye(4); out = {}
        for (w, p), n in zip(self.sc, self.names):
            out[n] = ((T @ np.r_[p, 1])[:3], T[:3, :3] @ np.asarray(w, float)); T = T @ expo(w, p, q[n])
        return out
class Kin(Chain):
    def __init__(self, D=None):
        self.D = D or P.derive(); y1, z1, PX = self.D["y1"], self.D["z1"], self.D["PX"]
        self.P1 = np.array([PX, y1, z1]); self.O0 = self.P1 + np.array([0, P.T_ARM, 0.0]); O0 = self.O0
        sc = [((1, 0, 0), self.P1), ((0, -1, 0), self.P1), ((0, 0, 1), O0), ((0, -1, 0), O0 + [0, 0, Z_E]), ((0, 0, 1), O0 + [0, DOFF, 0]),
              ((0, -1, 0), O0 + [0, DOFF, Z_W2]), ((1, 0, 0), O0 + [0, DOFF, Z_W3])]
        super().__init__(sc, O0 + [0, DOFF, Z_TIP])
    def group_transforms(self, q):
        c = self.cumulative(q); g = dict(M1=c["S1p"], M2=c["S2"], M3=c["S3"], ME=c["E"], MW1=c["W1"], MW2=c["W2"], MW3=c["W3"]); g["M4"] = g["ME"]; g["M5"] = g["MW3"]; return g
    def arm_frame(self, q):
        T = self.cumulative(q)["S2"].copy(); T[:3, 3] = (T @ np.r_[self.O0, 1])[:3]; return T          # (w, s, d) columns, origin on the S2 axis T_ARM along s
    def proxies(self):
        """distal proxy meshes, HOME pose, robot frame (conservative envelopes).  M3: upper arm + elbow ball; ME: elbow bridge + forearm; MW1: forearm distal + wrist ball;
        MW2: W2 yoke + W3 servo; MW3: hand envelope (rev B/C, wrist-to-tip 214)."""
        o = self.O0; sh = lambda m: (m.apply_translation(o), m)[1]
        M3 = dict(upper_arm=sh(cyl("z", 32.0, 68.0, Z_E - 38.0, c=(0, 0))), elbow_ball=sh(sph(44.0, (0, 0, Z_E))))
        ME = dict(elbow_bridge=sh(box(-30, 30, 0, DOFF + 28, Z_E + 2, Z_E + 60)), forearm_a=sh(cyl("z", 28.0, Z_E + 30, 330.0, c=(0, DOFF))))
        MW1 = dict(forearm_b=sh(cyl("z", 28.0, 330.0, 405.0, c=(0, DOFF))), wrist_ball=sh(sph(26.0, (0, DOFF, Z_W2))))
        MW2 = dict(w2_yoke=sh(box(-30, 30, DOFF - 30, DOFF + 30, 395.0, HAND["z"][0] + 1)))
        MW3 = dict(hand=sh(box(HAND["x"][0], HAND["x"][1], HAND["y"][0], HAND["y"][1], HAND["z"][0], HAND["z"][1])))
        return dict(M3=M3, ME=ME, MW1=MW1, MW2=MW2, MW3=MW3)
def baseline_chain():
    sc = [((0, 0, -1), (0, 0, 0)), ((0, -1, 0), (0, 0, 0)), ((0, 0, 1), (0, 0, 0)), ((0, -1, 0), (0, 0, Z_E)), ((0, 0, 1), (0, DOFF, 0)), ((0, -1, 0), (0, DOFF, Z_W2)), ((1, 0, 0), (0, DOFF, Z_W3))]
    return Chain(sc, (0, DOFF, Z_TIP), names=["S1", "S2", "S3", "E", "W1", "W2", "W3"])
SOFT = dict(P.SOFT); PARK = dict(P.PARK)
# ------------------------------------------------------------------------------------------------ shoulder Jacobian / DLS / gravity
def J_omega(th, phi):
    """angular-velocity Jacobian [d omega / d (th1', phi, S3)]: columns x, -s(th), d (robot frame)."""
    return np.c_[[1.0, 0, 0], -P.s_of(th), P.dirJ1(th, phi)]
def dls_inverse(J, lam=0.08): return J.T @ np.linalg.inv(J @ J.T + lam ** 2 * np.eye(J.shape[0]))
def dls_shoulder(th, phi, omega, lam=0.08):
    """task-space angular velocity omega (rad/s, robot frame; the SAME body-axis commands as the baseline map) -> (th1', phi', S3') rad/s."""
    return dls_inverse(J_omega(th, phi), lam) @ np.asarray(omega, float)
def J_baseline(S1, S2):
    c1 = np.array([0, 0, -1.0]); R1 = rodrigues([0, 0, -1], np.radians(S1)); c2 = R1 @ np.array([0, -1.0, 0]); R2 = R1 @ rodrigues([0, -1, 0], np.radians(S2)); return np.c_[c1, c2, R2[:, 2]]
def singularity_margin(q): return 90.0 - abs(q["S2"])                    # deg between the arm and the pole (arm along +-x): det J = -cos(phi)
def guard(q, omega_demand, phi_soft_max, slow_deg=10.0, lam_ex=0.08):
    """singularity guard (reference): returns (omega_scaled, info).  Slow zone = the last slow_deg before the soft S2 limit (the hard stop sits 5 deg beyond):
    task-space speed scaled by s = (phi_soft - phi)/slow_deg when the demand moves phi outward; amplification 1/cos(phi) is reported."""
    th, phi = q["S1p"], q["S2"]; qd = dls_shoulder(th, phi, omega_demand, lam_ex); s = 1.0
    if qd[1] * np.sign(phi) > 0 and abs(phi) > phi_soft_max - slow_deg: s = max(0.0, (phi_soft_max - abs(phi)) / slow_deg)
    return np.asarray(omega_demand) * s, dict(scale=s, amplification=1.0 / max(np.cos(np.radians(phi)), 1e-3))
g = 9.81
def gravity_torques(th_deg, phi_deg, payload=0.5):
    """S1' / S2 gravity torques (N.m) with the project masses (joint_j1.py): arm beyond S2 1.489 kg CG 0.249 m, payload at 0.575 m, S2 AK70 0.621 kg, yoke 0.15 kg."""
    t, p = np.radians(th_deg), np.radians(phi_deg); TARM, TS, AKL = P.T_ARM / 1000, P.TS / 1000, P.AKL / 1000
    sy = np.cos(t); ay = -np.sin(t) * np.cos(p)
    ya = TARM * sy + 0.249 * ay; yp = TARM * sy + 0.575 * ay; ym = (TS + AKL / 2) * sy; yy = 0.03 * sy
    tau1 = g * (1.489 * ya + payload * yp + 0.621 * ym + 0.15 * yy); tau2 = g * (1.489 * 0.249 + payload * 0.575) * np.sin(p) * np.cos(t); return tau1, tau2
CAPS_NM = dict(S1p=8.3, S2=8.3, S3=4.8)
# ------------------------------------------------------------------------------------------------ limits study (tilt alpha, S2 max, S1' range)
def _dirs(alpha, th_rng, ph_rng, step=1.0):
    al = np.radians(alpha); a = np.array([np.cos(al), 0, np.sin(al)]); u0 = np.cross(a, np.cross([0, 0, 1.0], a)); u0 /= np.linalg.norm(u0); out = []
    for th in np.arange(th_rng[0], th_rng[1] + 1e-9, step):
        u = rodrigues(a, np.radians(th)) @ u0
        for ph in np.arange(ph_rng[0], ph_rng[1] + 1e-9, step): c, s = np.cos(np.radians(ph)), np.sin(np.radians(ph)); out.append(c * u - s * a)
    return np.array(out), a
def limits_study():
    Bd = L.baseline_dir_set(2.0); rows = []
    for alpha in (0.0, 10.0):
        for phmax in (88, 80, 75, 70, 65, 60):
            best = None
            for t0 in range(-140, -19, 5):
                for t1 in range(-5, 91, 5):
                    d, a = _dirs(alpha, (t0, t1), (-30, phmax), 2.0); c = L.coverage(d, 3.0)
                    if best is None or c > best[0] + 1e-9 or (abs(c - best[0]) < 1e-9 and t1 - t0 < best[2] - best[1]): best = (c, t0, t1, t1 - t0)
            a_ = np.array([np.cos(np.radians(alpha)), 0, np.sin(np.radians(alpha))]); ang = lambda v: float(np.degrees(np.arccos(np.clip(Bd @ v, -1, 1))).min())
            rows.append(dict(alpha_deg=alpha, S2_max=phmax, coverage=round(best[0], 4), S1p_range=[best[1], best[2]], pole_to_baseline_set_deg=[round(ang(-a_), 1), round(ang(a_), 1)],
                             arm_to_pole_at_S2_max_deg=90 - phmax, depth_cost_mm=round(44.5 * np.sin(np.radians(alpha)), 1)))
    return rows
# ------------------------------------------------------------------------------------------------ speed caps, head/neck, workspace
def sample_q(n, seed=0, soft=None, S1p=None):
    r = np.random.default_rng(seed); soft = soft or SOFT; return [dict(zip(JN, [r.uniform(*soft[k]) for k in JN])) for _ in range(n)]
def speed_caps(K=None, n=6000, V=0.5):
    K = K or Kin(); lev = {k: [] for k in JN}
    for q in sample_q(n, 1):
        tip = K.tip(q); ax = K.axes_now(q)
        for k in JN: p, w = ax[k]; v = tip - p; lev[k].append(np.linalg.norm(v - (v @ w) * w))
    mx = {k: float(np.max(v)) / 1000 for k, v in lev.items()}; caps = {k: round(float(np.degrees(V / m)), 1) for k, m in mx.items()}
    return dict(lever_max_m={k: round(m, 3) for k, m in mx.items()}, cap_deg_s=caps, cap_with_project_hand_165=dict(S1p=49.5, S2=49.5, S3=81.5, E=81.5, W1=90.0, W2=90.0, W3=90.0))
HEAD = {}
def head_tree():
    if "t" not in HEAD:
        b = L.body(); v = b.vertices; sel = (v[:, 2] > -70) & (np.abs(v[:, 1] + 150) < 120) & (v[:, 0] < -90); HEAD["t"] = cKDTree(v[sel])
    return HEAD["t"]
def headneck_scan(K=None, length=590.0, radius=32.0):
    """straight-arm capsule (arm base -> +length along d, r 32) clearance to head/neck over the soft (S1', S2) box; baseline: P -> 590 mm r 32 gave >= 64.9 mm in the baseline box."""
    K = K or Kin(); tr_ = head_tree(); res = {}; worst = (1e9, None)
    for th in np.arange(SOFT["S1p"][0], SOFT["S1p"][1] + .1, 5.0):
        for ph in np.arange(SOFT["S2"][0], SOFT["S2"][1] + .1, 5.0):
            F = K.arm_frame(dict(S1p=th, S2=ph, S3=0, E=0, W1=0, W2=0, W3=0)); o = F[:3, 3]; d = F[:3, 2]; pts = o + np.outer(np.arange(0, length + 1, 8.0), d)
            c = float(tr_.query(pts)[0].min() - radius)
            if c < worst[0]: worst = (c, (th, ph))
            res[(th, ph)] = c
    return dict(min_clearance_mm=round(worst[0], 1), at_S1p_S2=worst[1], n_below_60=int(sum(1 for v in res.values() if v < 60)), n=len(res)), res
def workspace_compare(step=5.0):
    """same-direction comparison (the baseline direction set D, joint space is not uniform so voxel overlap of random tips is NOT used): for every baseline (S1,S2) pose solve J1 (th1, phi) giving the SAME arm direction,
    keep it if inside the J1 soft box, and compare the fully extended tip (E=W=0, S3=0) and the arm base: displacement = what moving the arm base from P to the J1 arm base does to the reach."""
    K = Kin(); B = baseline_chain(); disp = []; base = []; n = 0; inside = 0
    for S1 in np.arange(10, 140.1, step):
        for S2 in np.arange(-30, 88.1, step):
            n += 1; d = L.baseline_dir(S1, S2); ph = np.degrees(np.arcsin(-d[0])); th = np.degrees(np.arctan2(-d[1], d[2]))
            if not (SOFT["S1p"][0] <= th <= SOFT["S1p"][1] and SOFT["S2"][0] <= ph <= SOFT["S2"][1]): continue
            inside += 1; qj = dict(S1p=th, S2=ph, S3=0, E=0, W1=0, W2=0, W3=0); qb = dict(S1=S1, S2=S2, S3=0, E=0, W1=0, W2=0, W3=0)
            tj, tb = K.tip(qj), B.tip(qb); disp.append(tj - tb); base.append(K.arm_frame(qj)[:3, 3])
    disp = np.array(disp); m = np.linalg.norm(disp, axis=1)
    return dict(baseline_directions=n, reachable_by_J1_soft_box=inside, fraction=round(inside / n, 3), extended_tip_displacement_vs_baseline_mm=dict(mean=disp.mean(0).round(0).tolist(), mean_norm=round(float(m.mean())), p95_norm=round(float(np.percentile(m, 95))), max_norm=round(float(m.max()))),
                arm_base_range_mm=[np.array(base).min(0).round(0).tolist(), np.array(base).max(0).round(0).tolist()], baseline_P=[0, 0, 0])
def passive_box_study(ys=(-70, -50, -30, -10, 10), zs=(-35, -70, -105), thr=60.0):
    """pad-position study: for a straight arm (590 mm, r 32) the best PASSIVE box (S1' <= t1, S2 <= pm) that keeps >= thr mm to head/neck, and the baseline-direction coverage of that box."""
    tr_ = head_tree(); THs = np.arange(-95, 35.1, 5.0); PHs = np.arange(-30, 88.1, 5.0); rows = []
    for y1 in ys:
        for z1 in zs:
            D = P.derive(y1, z1); K = Kin(D)
            def clr(th, ph):
                F = K.arm_frame(dict(S1p=th, S2=ph, S3=0, E=0, W1=0, W2=0, W3=0)); pts = F[:3, 3] + np.outer(np.arange(0, 591, 10.0), F[:3, 2]); return float(tr_.query(pts)[0].min() - 32)
            C = np.array([[clr(t, p) for p in PHs] for t in THs]); dirs = lambda m: np.array([P.dirJ1(t, p) for i, t in enumerate(THs) for j, p in enumerate(PHs) if m[i, j]]); best = (0, None)
            for t1 in np.arange(-30, 35.1, 5.0):
                for pm in np.arange(40, 88.1, 5.0):
                    sel = (THs <= t1)[:, None] & (PHs <= pm)[None, :]
                    if (C[sel] >= thr).all():
                        cv = L.coverage(dirs(sel), 3.0)
                        if cv > best[0]: best = (cv, (float(t1), float(pm)))
            rows.append(dict(y1=y1, z1=z1, x0=round(D["x0"], 1), min_clr_full_soft_box=round(float(C.min()), 1), best_passive_box_coverage=round(best[0], 3), best_box_S1p_max_S2_max=best[1]))
    return rows
def baseline_levers(n=6000):
    B = baseline_chain(); lev = {k: [] for k in B.names}
    r = np.random.default_rng(2); SB = dict(S1=(10, 140), S2=(-30, 88), S3=(-70, 90), E=(0, 145), W1=(-80, 80), W2=(-70, 80), W3=(-20, 30))
    for _ in range(n):
        q = {k: r.uniform(*SB[k]) for k in B.names}; tip = B.tip(q); ax = B.axes_now(q)
        for k in B.names: p, w = ax[k]; v = tip - p; lev[k].append(np.linalg.norm(v - (v @ w) * w))
    return {k: round(float(np.max(v)) / 1000, 3) for k, v in lev.items()}
def conditioning(phmax=88):
    r = np.random.default_rng(5); sj = []; sb = []
    for _ in range(20000):
        th, ph = r.uniform(*SOFT["S1p"]), r.uniform(SOFT["S2"][0], phmax); sj.append(np.linalg.svd(J_omega(th, ph), compute_uv=False)[-1])
        s1, s2 = r.uniform(10, 140), r.uniform(-30, 88); sb.append(np.linalg.svd(J_baseline(s1, s2), compute_uv=False)[-1])
    sj, sb = np.array(sj), np.array(sb); return dict(sigma_min_J1=dict(min=round(sj.min(), 3), p5=round(np.percentile(sj, 5), 3), frac_below_0_3=round(float((sj < .3).mean()), 4)),
                                                  sigma_min_baseline=dict(min=round(sb.min(), 3), p5=round(np.percentile(sb, 5), 3), frac_below_0_3=round(float((sb < .3).mean()), 4)))
# ------------------------------------------------------------------------------------------------ tests
def tests(K=None):
    K = K or Kin(); r = np.random.default_rng(7); res = {}
    # (1) FK vs the explicit nested chain (arm_kin.py form transplanted into the arm frame FA)
    def nested(q):
        FA = P.shoulder_frames(K.D, q["S1p"], q["S2"])[1]; R = lambda ax, a, pt=(0, 0, 0): expo(ax, pt, a)
        R3 = R((0, 0, 1), q["S3"]); RE = R3 @ R((0, -1, 0), q["E"], (0, 0, Z_E)); RW1 = RE @ R((0, 0, 1), q["W1"], (0, DOFF, 0))
        RW2 = RW1 @ R((0, -1, 0), q["W2"], (0, DOFF, Z_W2)); RW3 = RW2 @ R((1, 0, 0), q["W3"], (0, DOFF, Z_W3)); Tn = np.eye(4); Tn[:3, 3] = -K.O0
        return FA @ RW3 @ Tn, FA
    e = []; ef = []
    for _ in range(200):
        q = {k: r.uniform(*SOFT[k]) for k in JN}; Tn, FA = nested(q); Tp = K.cumulative(q)["W3"]
        e.append(np.abs(Tn - Tp).max()); ef.append(np.abs(FA - K.arm_frame(q)).max())
    res["FK_vs_nested_max_abs_err"] = float(max(e)); res["arm_frame_vs_P.shoulder_frames_max_abs_err"] = float(max(ef))
    # (2) park: arm direction equals the baseline park direction S1 45 / S2 25; tip position
    d = K.arm_frame(PARK)[:3, 2]; res["park_dir_err_deg"] = float(np.degrees(np.arccos(np.clip(d @ L.baseline_dir(45, 25), -1, 1)))); res["tip_park_xyz"] = K.tip(PARK).round(1).tolist()
    res["tip_home_check"] = bool(np.allclose(K.tip(dict(S1p=0, S2=0, S3=0, E=0, W1=0, W2=0, W3=0)), K.O0 + [0, DOFF, Z_TIP]))
    # (3) coverage of the baseline direction set by the shoulder FK over the soft box
    dirs = np.array([K.arm_frame(dict(S1p=a, S2=b, S3=0, E=0, W1=0, W2=0, W3=0))[:3, 2] for a in np.arange(SOFT["S1p"][0], SOFT["S1p"][1] + .1, 1.0) for b in np.arange(SOFT["S2"][0], SOFT["S2"][1] + .1, 1.0)])
    res["coverage_soft_box"] = round(L.coverage(dirs, 3.0), 4)
    # (4) group transforms consistent: rigid, and S3 / E / W about the right axes (axes_now vs group origin motion)
    q = {k: r.uniform(*SOFT[k]) for k in JN}; G = K.group_transforms(q); res["groups_rigid"] = bool(all(abs(np.linalg.det(T[:3, :3]) - 1) < 1e-9 for T in G.values()))
    # (5) DLS/Jacobian: omega recovered; amplification at park and singular direction
    th, ph = PARK["S1p"], PARK["S2"]; w = np.array([0.3, -0.2, 0.1]); qd = np.linalg.solve(J_omega(th, ph), w); res["J_det_equals_minus_cos_phi"] = bool(abs(np.linalg.det(J_omega(th, ph)) + np.cos(np.radians(ph))) < 1e-9)
    res["amplification_park"] = round(float(1 / np.cos(np.radians(ph))), 3); res["omega_roundtrip_err"] = float(np.abs(J_omega(th, ph) @ qd - w).max())
    # (6) finite-difference angular velocity of the arm direction equals J omega columns
    eps = 1e-5; d0 = P.dirJ1(th, ph); d1 = P.dirJ1(th + eps, ph); res["dd_dth_eq_x_cross_d"] = bool(np.allclose((d1 - d0) / np.radians(eps), np.cross([1, 0, 0], d0), atol=1e-4))
    return res
if __name__ == "__main__":
    K = Kin(); out = dict(D=K.D); out["tests"] = tests(K); print(json.dumps(out["tests"], indent=1))
    out["limits_study"] = limits_study(); [print(r) for r in out["limits_study"]]
    out["speed"] = speed_caps(K); print(out["speed"])
    hn, grid = headneck_scan(K); out["headneck_straight_arm"] = hn; print(hn); hn2, _ = headneck_scan(K, 590.0, 60.0); out["headneck_straight_arm_r60"] = hn2; print(hn2)
    out["workspace"] = workspace_compare(); print(out["workspace"]); out["baseline_levers_same_hand"] = baseline_levers(); print(out["baseline_levers_same_hand"]); out["passive_box_study"] = passive_box_study(); json.dump(out["passive_box_study"], open(os.path.join(OUT, "passive_box_study.json"), "w"), indent=1); out["conditioning"] = {phm: conditioning(phm) for phm in (88, 75)}; print(out["conditioning"])
    t1, t2 = gravity_torques(np.arange(-95, 36, 2.5)[:, None], np.arange(-30, 89, 2.5)[None, :]); out["gravity_unbalanced_Nm"] = dict(S1p=round(float(np.abs(t1).max()), 2), S2=round(float(np.abs(t2).max()), 2))
    for g_, d_ in K.proxies().items():
        for n_, m in d_.items(): m.export(os.path.join(OUT, "proxies", f"{g_}_{n_}.stl"))
    json.dump(out, open(os.path.join(OUT, "j1r2_kin_report.json"), "w"), indent=1, default=float)
