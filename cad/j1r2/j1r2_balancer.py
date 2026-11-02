"""J1 rev 2 gravity balancer + stops/free-fall energy (pancake S1', side-hung S2).  Robot frame, mm / N / N m / J / deg.  Run: python3 -I cad/j1r2_balancer.py
DESIGN (found by search over the full pose grid, cad/j1r2_bal_search*.py): ONE zero-length-type spring (cord over a swivel sheave, 2:1 tackle, compression spring in a guide tube) between a point A
FIXED on the pad (world-fixed; sits ~70 mm from the S1' axis at 105 deg in the yz plane, outside the S1' motor) and a pin B on the upper arm (arm frame).  For a spring from a world-fixed point to a point on the arm
the moment about the S1' axis and about the (rotating) S2 axis are r x F with r = B - P1; with A above P1 this reproduces BOTH gravity terms (cos th1 sin phi on S2 and the y-lever on S1'), so one spring balances both axes
and S2 needs no cos(th1) gimbal.  A yoke-anchored S2 spring (K sin phi, baseline style) cannot follow cos th1 and leaves S1' unbalanced: decoupled result in stl/j1r2_bal/search.json.
API: balancer_parts(D, th1, phi, tube_dir='lateral') -> {name: dict(mesh, group, mass_g)};  tau_balancer(th1, phi) -> (tau_S1p, tau_S2) N m on the arm;  GEOM, SPRING, ANCHORS."""
import sys, os, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, clib as L
from clib import cyl, box, U, D as DIFF, tr, rot
import j1r2_params as P, j1r2_bal_model as M
OUT = os.path.join(L.ROOT, "stl", "j1r2_bal"); os.makedirs(OUT, exist_ok=True)
g = 9.81; RATED = 8.3; CAP_S3 = 4.8
D0 = M.D0; P1 = M.P1
# ---------------------------------------------------------------- design constants (from the search, rounded)
A_OFF = np.array([-10.018, -19.708, 73.434])            # A = P1 + A_OFF  (world-fixed, on the pad rim side); |A_yz| = 76.0 mm, angle 105 deg
B_LOC = np.array([-15.629, -42.256, 118.385])           # B on the upper arm, arm frame (x=w, y=s, z=d): on a lug 13 mm off the R32 tube, z 118.4
K_CORD, L0 = 0.7520, 54.587                             # N/mm, mm :  F_cord = K_CORD (|AB| - L0)
TACKLE = 2                                              # 2:1 purchase: spring force = 2 F_cord, spring compression = (|AB| - L0)/2
ARM_SIDE_G = 52.0                                       # collar + pin + half cord, at B
GEOM = dict(A_world=(P1 + A_OFF).tolist(), A_off_from_S1p_axis_mm=A_OFF.tolist(), B_arm_local=B_LOC.tolist(), k_cord_N_per_mm=K_CORD, L0_mm=L0, tackle=TACKLE)
def A_w(): return P1 + A_OFF
def B_w(th, ph):
    F0, FA, s, a, d = P.shoulder_frames(D0, th, ph); return (FA @ np.r_[B_LOC, 1.0])[:3]
def cord_len(th, ph): return float(np.linalg.norm(A_w() - B_w(th, ph)))
def tau_balancer(th, ph, k=K_CORD, l0=L0):
    """moments of the spring on the arm about the S1' axis (x) and the S2 axis (s), N m"""
    F0, FA, s, a, d = P.shoulder_frames(D0, th, ph); B = (FA @ np.r_[B_LOC, 1.0])[:3]; v = A_w() - B; n = np.linalg.norm(v); F = k * (n - l0) * v / n
    mom = np.cross((B - P1) * 1e-3, F); return float(mom[0]), float(mom @ s)
# ---------------------------------------------------------------- torque study over the FULL pose grid, balancer arm-side mass included
poses = M.grid_poses()
def evaluate(k=K_CORD, l0=L0, off_mm=0.0, extra_g=ARM_SIDE_G):
    r1 = np.zeros(len(poses)); r2 = np.zeros(len(poses)); u1 = np.zeros(len(poses)); u2 = np.zeros(len(poses))
    for i, p in enumerate(poses):
        t1, t2 = M.grav_moments(*p, extra_arm=[(extra_g * 1e-3, B_LOC)]); s1, s2 = tau_balancer(p[0], p[1], k, l0 - off_mm)
        u1[i], u2[i] = t1 * 1e-3, t2 * 1e-3; r1[i], r2[i] = u1[i] + s1, u2[i] + s2
    return u1, u2, r1, r2
u1, u2, r1, r2 = evaluate()
pp = np.array([p[4] for p in poses]); E_ = np.array([p[2] for p in poses])
res = dict(unbalanced_max_Nm=dict(S1p=round(float(abs(u1).max()), 2), S2=round(float(abs(u2).max()), 2)),
           balanced_max_Nm=dict(S1p=round(float(abs(r1).max()), 2), S2=round(float(abs(r2).max()), 2)),
           SF_vs_rated=dict(S1p=round(RATED / float(abs(r1).max()), 2), S2=round(RATED / float(abs(r2).max()), 2)),
           worst_pose_S1p=[float(v) for v in poses[int(np.argmax(abs(r1)))]], worst_pose_S2=[float(v) for v in poses[int(np.argmax(abs(r2)))]],
           payload0_range_Nm=dict(S1p=[round(float(r1[pp == 0].min()), 2), round(float(r1[pp == 0].max()), 2)], S2=[round(float(r2[pp == 0].min()), 2), round(float(r2[pp == 0].max()), 2)]),
           payload05_range_Nm=dict(S1p=[round(float(r1[pp == .5].min()), 2), round(float(r1[pp == .5].max()), 2)], S2=[round(float(r2[pp == .5].min()), 2), round(float(r2[pp == .5].max()), 2)]),
           n_poses=len(poses), grid="th1 -95..35 step 5, phi -30..88 step 6, E 0/45/90/145, S3 -70/0/90, payload 0/0.5 kg (12960 poses, straight wrist)")
# sensitivity: k +-5 %, L0 +-2 mm (cord-length calibration)
sens = {}
for lab, kk, ll in (("k -5 %", .95 * K_CORD, L0), ("k +5 %", 1.05 * K_CORD, L0), ("L0 +2 mm", K_CORD, L0 + 2), ("L0 -2 mm", K_CORD, L0 - 2)):
    _, _, a_, b_ = evaluate(kk, ll); sens[lab] = dict(S1p=round(RATED / float(abs(a_).max()), 2), S2=round(RATED / float(abs(b_).max()), 2))
res["sensitivity_SF"] = sens
fr = 0.05; fric = {}
F_all = np.array([K_CORD * (cord_len(p[0], p[1]) - L0) for p in poses]); fmax = float(F_all.max())
res["friction_5pct_hysteresis_SF"] = round(RATED / (float(abs(r1).max()) + fr * 4.0), 2)
res["cord"] = dict(F_cord_max_N=round(fmax, 1), F_spring_max_N=round(TACKLE * fmax, 1), cord_len_min_mm=round(float(min(cord_len(p[0], p[1]) for p in poses)), 1), cord_len_max_mm=round(float(max(cord_len(p[0], p[1]) for p in poses)), 1))
# the decoupled (baseline-style) alternative, for the record
try: res["decoupled_alt"] = json.load(open(os.path.join(OUT, "search.json")))["D1"]
except Exception: pass
# ---------------------------------------------------------------- spring (2:1 tackle) design, ASTM A228 as the baseline
kc = K_CORD; ks = TACKLE ** 2 * kc; Gm = 80000.0
dmin, dmax = min(cord_len(p[0], p[1]) for p in poses[::7]), max(cord_len(p[0], p[1]) for p in poses[::7]); xs = lambda dd: max((dd - L0) / TACKLE, 0.0)
xs_min, xs_max = xs(dmin), xs(dmax)
d_w, Dm = 3.5, 36.0; Na = Gm * d_w ** 4 / (8 * Dm ** 3 * ks); Nt = Na + 2; Ls = Nt * d_w; L_free = Ls + 1.12 * xs_max + xs_max; C = Dm / d_w
KB = (4 * C + 2) / (4 * C - 3); Fs_max = ks * xs_max; tau = KB * 8 * Fs_max * Dm / (np.pi * d_w ** 3); Sut = 2211.0 / d_w ** 0.145
mass_spring = np.pi * Dm * Nt * np.pi / 4 * d_w ** 2 * 7.85e-3
SPRING = dict(wire_d=d_w, D_mean=Dm, OD=Dm + d_w, N_active=round(Na, 2), N_total=round(Nt, 2), k_spring_N_per_mm=round(ks, 3), compression_min_mm=round(xs_min, 1), compression_max_mm=round(xs_max, 1),
              L_solid=round(Ls, 1), L_free=round(L_free, 1), F_max_N=round(Fs_max, 1), tau_max_MPa=round(tau, 0), tau_over_Sut=round(tau / Sut, 3), SF_vs_0p45Sut=round(0.45 * Sut / tau, 2), mass_g=round(mass_spring, 1),
              note="PLACEHOLDER vendor: custom wound A228, squared+ground, shot-peened; same spec family as the baseline S2 spring (k 3.66, F 195 N). Cord Dyneema 1.5 mm: SF = 1156 N / %.0f N = %.1f" % (fmax, 1156 / fmax))
# ---------------------------------------------------------------- free fall / stops (each axis released from rest at the worst start, others frozen)
HARD_S1 = (-100.0, 40.0); HARD_S2 = (-35.0, 91.0)
def net(th, ph, E, S3, pay, bal=True):
    t1, t2 = M.grav_moments(th, ph, E, S3, pay, extra_arm=[(ARM_SIDE_G * 1e-3, B_LOC)]); t1 *= 1e-3; t2 *= 1e-3
    if bal: s1, s2 = tau_balancer(th, ph); t1 += s1; t2 += s2
    return t1, t2
def impact(T, grid, i0):
    d = np.radians(grid[1] - grid[0]); Ti = T[i0]
    if abs(Ti) < 1e-6: return 0.0, None
    st = 1 if Ti > 0 else -1; W = 0.0; i = i0
    while 0 <= i + st < len(T):
        W += 0.5 * (T[i] + T[i + st]) * d * st; i += st
        if W < 0: return 0.0, None
    return W, ("+" if st > 0 else "-")
ff = {}
for bal in (True, False):
    best = {"S1p": {"+": (0, None), "-": (0, None)}, "S2": {"+": (0, None), "-": (0, None)}}
    for E in (0, 45, 90, 145):
        for S3 in (-70, 0, 90):
            for pay in (0.0, 0.5):
                g1 = np.arange(HARD_S1[0], HARD_S1[1] + .01, 2.5)
                for ph in (-30, 0, 30, 60, 88):
                    T = np.array([net(t, ph, E, S3, pay, bal)[0] for t in g1])
                    for i0 in range(len(g1)):
                        W, w = impact(T, g1, i0)
                        if w and W > best["S1p"][w][0]: best["S1p"][w] = (W, (float(g1[i0]), ph, E, S3, pay))
                g2 = np.arange(HARD_S2[0], HARD_S2[1] + .01, 2.5)
                for th in (-90, -60, -30, 0, 30):
                    T = np.array([net(th, p, E, S3, pay, bal)[1] for p in g2])
                    for i0 in range(len(g2)):
                        W, w = impact(T, g2, i0)
                        if w and W > best["S2"][w][0]: best["S2"][w] = (W, (th, float(g2[i0]), E, S3, pay))
    ff["balanced" if bal else "unbalanced"] = {ax: {w: dict(W_J=round(v[0], 2), start=v[1]) for w, v in d_.items()} for ax, d_ in best.items()}
# bumpers (baseline crush-blade recipe, plateau force ~ t^3): S2 as built (+ 22.1 N m 1.00 J, - 16.8 N m 0.98 J) ; S1' lever 54 mm (posts at r54, TPU 95A blades 14 mm wide x3)
def blade(M_plateau, lever_mm, usable_mm, t_ref=1.19, F_ref=293.0):
    F = M_plateau / (lever_mm * 1e-3); t = t_ref * (F / F_ref) ** (1 / 3); return dict(M_plateau_Nm=M_plateau, F_N=round(F, 0), blade_t_mm=round(t, 2), usable_crush_mm=usable_mm, energy_J=round(0.9 * F * usable_mm * 1e-3, 2))
bump = dict(S2="as built, unchanged geometry/angles (+91 / -35 first contact, bottomed +94 / -39): +22.1 N m 1.00 J, -16.8 N m 0.98 J",
            S1p_pos=blade(20.0, 54.0, 4.0), S1p_neg=blade(20.0, 54.0, 4.0),
            note="S1' posts at r 54 -> 0.94 mm of arc per degree; first contact at the HARD limits (-100 / +40), 4 mm usable crush = ~4 deg of bottoming margin; plateau 20 N m must stay below M_allow(S1' stop) = NOT re-run (body_loads needs the project solver/mesh)")
Wp = ff["balanced"]; need = {}
for ax, eb in (("S1p", {"+": bump["S1p_pos"]["energy_J"], "-": bump["S1p_neg"]["energy_J"]}), ("S2", {"+": 1.00, "-": 0.98})):
    for w in "+-":
        W = Wp[ax][w]["W_J"]; ER_ALLOW = 0.07                        # baseline pass boundary: f 0.772, 4.69 J, 1.0 J bumper -> residual 0.07 J
        need[f"{ax}{w}"] = dict(W_unbraked_J=W, bumper_J=eb[w], Er_unbraked_J=round(max(W - eb[w], 0), 2), f_needed=None if W <= eb[w] + ER_ALLOW else round(1 - (eb[w] + ER_ALLOW) / W, 3))
ffs = dict(energies=ff, bumpers=bump, braking_need=need, criterion="Er = W(1-f) - E_bumper <= 0.07 J = the baseline's pass boundary (SPEC_balancer 15: f 0.772, W 4.69 J, bumper 1.0 J); pad pressure itself NOT re-run (body_loads solver + mesh are in the repo, not in the project knowledge)",
           plan="unchanged: normally-closed phase-short on BOTH AK70s (hardware, MCU-independent); T12 must now measure f on S1' as well (S1' is gravity-loaded)")
# ---------------------------------------------------------------- spring failure (spring_check): threshold = model band + 0.6 N m + 10 %
tb = np.array([tau_balancer(p[0], p[1]) for p in poses]); thr_full = 0.6 + 2.82 + 0.1 * np.maximum(np.abs(u1), np.abs(u2)); thr_learn = 0.6 + 0.1 * np.maximum(np.abs(u1), np.abs(u2))
det_full = (np.abs(tb[:, 0]) > thr_full) | (np.abs(tb[:, 1]) > thr_full); det_learn = (np.abs(tb[:, 0]) > thr_learn) | (np.abs(tb[:, 1]) > thr_learn)
sf = dict(snap_torque_max_Nm=dict(S1p=round(float(abs(tb[:, 0]).max()), 2), S2=round(float(abs(tb[:, 1]).max()), 2)), detect_coverage_full_payload_band=round(float(det_full.mean()), 3), detect_coverage_learned_payload=round(float(det_learn.mean()), 3),
          consequence="a snapped spring drops up to %.1f N m onto S1' and %.1f N m onto S2 at the same instant; both axes see the step, so detection can use either; the arm sags under gravity: at the 49.5 deg/s cap about %.1f N m excess is inside the %.1f N m AK70 cap" % (abs(tb[:, 0]).max(), abs(tb[:, 1]).max(), abs(tb).max(), RATED),
          firmware="grav_model.h: add s1p_grav_eval(th1, phi, E, S3, pay) and the pad-spring term (cord length |AB|(th1, phi) -> F = %.4f (|AB| - %.2f)); S2 model changes from K sin(S2) to the th1-coupled r x F; spring_check runs on both axes; BALANCER_FITTED constants: A (%s), B (%s), k, L0 in protocols.h; keep-out tables must be regenerated (the balancer cord/tube is in the lateral/up bay)" % (K_CORD, L0, np.round(A_off_world := (P1 + A_OFF), 1).tolist(), B_LOC.tolist()))
# ---------------------------------------------------------------- mesh proxies + static clearance
def tangent(dirname):
    r = np.array([A_OFF[1], A_OFF[2]]); r = r / np.linalg.norm(r); lat = np.array([r[1], -r[0]])
    return {"lateral": lat, "medial": -lat, "radial": r}[dirname]
def frame_from_u(origin, u):
    u = u / np.linalg.norm(u); z = u; x = np.cross([1, 0, 0.0], z); x /= np.linalg.norm(x); y = np.cross(z, x); M_ = np.eye(4); M_[:3, 0], M_[:3, 1], M_[:3, 2], M_[:3, 3] = x, y, z, origin; return M_
def seg(p0, p1, r): return trimesh.creation.cylinder(r, segment=[p0, p1], sections=16)
def balancer_parts(D, th1, phi, tube_dir="lateral", L_tube=150.0):
    """posed meshes: fixed-side parts (pad frame) + arm-side parts (arm frame). D must equal j1r2_params.derive() at the y1,z1 of the model."""
    t2 = tangent(tube_dir); u = np.array([0.0, t2[0], t2[1]]); A = A_w(); Tf = frame_from_u(A, u)       # local z along the tube, origin at A (cap outer face)
    parts = {}
    def pl(m, M_): m = m.copy(); m.apply_transform(M_); return m
    tube = DIFF(cyl("z", 22.25, 0, L_tube + 6), cyl("z", 19.75, 8.0, L_tube))
    parts["bal_tube"] = dict(mesh=pl(tube, Tf), group="joint", mass_g=96.0, mat="PETGCF")
    cap = U(cyl("z", 28.0, -8.0, 0.0), cyl("z", 12.0, -14, -8.0)); parts["bal_tubecap"] = dict(mesh=pl(cap, Tf), group="joint", mass_g=31.0, mat="PETGCF")
    post = seg(np.array([D["XB"] - 0.0, A[1], A[2]]), np.array([A[0] - 14.0, A[1], A[2]]), 13.0); parts["bal_post"] = dict(mesh=post, group="joint", mass_g=34.0, mat="PETGCF")
    c = cord_len(th1, phi); xs_ = xs(c); Lsp = SPRING["L_free"] - xs_
    spr = DIFF(cyl("z", 19.3, 8.0, 8.0 + Lsp), cyl("z", 15.8, 7.0, 9.0 + Lsp)); parts["bal_spring"] = dict(mesh=pl(spr, Tf), group="joint", mass_g=SPRING["mass_g"], mat="STEEL")
    pis = cyl("z", 19.0, 8.0 + Lsp, 8.0 + Lsp + 10.0); parts["bal_piston"] = dict(mesh=pl(pis, Tf), group="joint", mass_g=28.0, mat="PETGCF")
    e = (B_w(th1, phi) - A); e /= np.linalg.norm(e)
    parts["bal_sheave"] = dict(mesh=seg(A - 2.5 * np.cross(e, u) / max(np.linalg.norm(np.cross(e, u)), 1e-6) + 0 * e, A + 2.5 * np.cross(e, u) / max(np.linalg.norm(np.cross(e, u)), 1e-6), 8.5), group="joint", mass_g=9.0, mat="AL")
    parts["bal_cord"] = dict(mesh=seg(A + 6.75 * e, B_w(th1, phi), 1.25), group="joint", mass_g=1.0, mat="TPU")
    F0, FA, s, a, d = P.shoulder_frames(D0, th1, phi)
    col = cyl("z", 38.0, 104.0, 126.0, r_in=32.05); lug = seg(np.array([-12.0, -33.0, 118.4]), B_LOC, 8.0); col = U(col, lug)
    col.apply_transform(FA); parts["bal_collar"] = dict(mesh=col, group="joint", mass_g=ARM_SIDE_G - 12, mat="PETGCF", arm=True)
    pin = seg((FA @ np.r_[B_LOC + np.array([-8, 0, 0]), 1])[:3], (FA @ np.r_[B_LOC + np.array([8, 0, 0]), 1])[:3], 3.0); parts["bal_pin"] = dict(mesh=pin, group="joint", mass_g=12.0, mat="STEEL", arm=True)
    return parts
# static clearance of the balancer vs the J1 stage proxies over the soft + hard grid (fcl)
def stage_proxies(th, ph):
    F0, FA, s, a, d = P.shoulder_frames(D0, th, ph); XO = D0["XO"]; R = {}
    disc = cyl("x", 47.0, XO, XO + 12.0, c=(D0["y1"], D0["z1"])); R0 = rot(np.radians(th), [1, 0, 0]); Tt = tr([0, D0["y1"], D0["z1"]]) @ R0 @ tr([0, -D0["y1"], -D0["z1"]])
    disc = disc.copy(); disc.apply_transform(Tt); bar = box(-6, 6, 0, P.TS + 3, -26, 26); bar.apply_transform(F0)
    mot = cyl("y", P.AKR, P.TS, P.TS + P.AKL); mot.apply_transform(F0); clamp = cyl("y", P.AKR + 4, P.TS + 4, P.TS + 34, r_in=P.AKR + .1); clamp.apply_transform(F0)
    R["M1"] = U(disc, bar, mot, clamp)
    ad = cyl("y", 40.0, P.TS + P.AKL, P.T_ARM); ad.apply_transform(FA); br = box(-24, 24, P.TS + P.AKL, P.T_ARM + 12, 6, 70); br.apply_transform(FA); xm = box(-29.25, 29.25, P.T_ARM - 16.75, P.T_ARM + 16.75, 68, 112); xm.apply_transform(FA)
    ua = cyl("z", 32.0, 112, 190, c=(0, P.T_ARM)); ua.apply_transform(FA); eb = L.sph(44.0, (0, P.T_ARM, 230)); eb.apply_transform(FA)
    R["M2"] = U(ad, br, xm, ua, eb)
    s1m = cyl("x", P.AKR, D0["XB"], D0["XO"], c=(D0["y1"], D0["z1"])); R["S1motor"] = s1m
    return R
import fcl
def fcl_obj(m): bv = fcl.BVHModel(); bv.beginModel(len(m.vertices), len(m.faces)); bv.addSubModel(m.vertices, m.faces); bv.endModel(); return fcl.CollisionObject(bv, fcl.Transform())
def mind(a, b):
    r = fcl.DistanceResult(); fcl.distance(fcl_obj(a), fcl_obj(b), fcl.DistanceRequest(), r); return float(r.min_distance)
def clearance(tube_dir, ths, phs):
    worst = {}
    for th in ths:
        for ph in phs:
            bp = balancer_parts(D0, th, ph, tube_dir); st = stage_proxies(th, ph)
            fixed = trimesh.util.concatenate([bp[k]["mesh"] for k in ("bal_tube", "bal_tubecap", "bal_post", "bal_sheave", "bal_piston", "bal_spring")]); cord = bp["bal_cord"]["mesh"]; arms = trimesh.util.concatenate([bp["bal_collar"]["mesh"], bp["bal_pin"]["mesh"]])
            for lab, a_, b_ in (("fixed_vs_M1(yoke,S2 motor)", fixed, st["M1"]), ("fixed_vs_M2(arm)", fixed, st["M2"]), ("cord_vs_M1", cord, st["M1"]), ("cord_vs_M2", cord, st["M2"]),
                                ("armside_vs_M1", arms, st["M1"]), ("armside_vs_fixed", arms, fixed), ("fixed_vs_S1motor", fixed, st["S1motor"])):
                d_ = mind(a_, b_)
                if lab not in worst or d_ < worst[lab][0]: worst[lab] = (round(d_, 1), (float(th), float(ph)))
    return {k: dict(min_mm=v[0], at_th1_phi=v[1]) for k, v in worst.items()}
SOFT_T = list(np.arange(-95, 35.1, 10)) + [35.0]; SOFT_P = list(np.arange(-30, 88.1, 14)) + [88.0]; HARD_T = [-100.0, 40.0]; HARD_P = [-35.0, 93.0]
clr = {}
for td in ("lateral", "radial", "medial"):
    clr[td] = dict(soft_box=clearance(td, SOFT_T, SOFT_P), hard_corners=clearance(td, HARD_T, HARD_P))
    bpd = balancer_parts(D0, th_p0 := P.PARK["S1p"], P.PARK["S2"], td); clr[td]["skin_gap_mm"] = {k: round(L.skin_gap(bpd[k]["mesh"], 2000)[0], 1) for k in ("bal_tube", "bal_tubecap", "bal_post")}

# ---------------------------------------------------------------- spring packaging options (tackle ratio n): free length / tube length / force
opts = []
dsp = [(abs(dd - L0)) for dd in []]
xs_cord = (dmax - L0)
for n in (2, 3, 4):
    ks_n = n ** 2 * K_CORD; xm_ = xs_cord / n; Fm_ = n * K_CORD * xs_cord
    for dwi in np.arange(3.0, 7.01, 0.25):
        for Dm_ in (36.0, 40.0, 44.0):
            C_ = Dm_ / dwi
            if not (5.0 <= C_ <= 12.0): continue
            KB_ = (4 * C_ + 2) / (4 * C_ - 3); tau_ = KB_ * 8 * Fm_ * Dm_ / (np.pi * dwi ** 3); Sut_ = 2211.0 / dwi ** 0.145
            if tau_ <= 0.31 * Sut_:
                Na_ = Gm * dwi ** 4 / (8 * Dm_ ** 3 * ks_n); Ls_ = (Na_ + 2) * dwi; Lf_ = Ls_ + 2.12 * xm_
                opts.append(dict(tackle=n, k_spring=round(ks_n, 2), F_spring_max_N=round(Fm_, 0), stroke_mm=round(xm_, 1), wire_d=float(dwi), D=Dm_, L_free_mm=round(Lf_, 0), tube_length_mm=round(Lf_ + 8 + 10 + 6 + 8, 0), OD_tube=round(Dm_ + dwi + 5 + 5, 1)))
                break
        else: continue
        break
res["spring_options_by_tackle"] = opts
# ---------------------------------------------------------------- finer collision census, radial tube (soft box only): where does the cord / arm-side hit the J1 stage or the tube
cen = dict(poses=0, cord_vs_M1=0, armside_vs_fixed=0, fixed_vs_M1=0, fixed_vs_M2=0); first = {}
for th in np.arange(-95, 35.1, 5.0):
    for ph in np.arange(-30, 88.1, 9.0):
        bpr = balancer_parts(D0, th, ph, "radial"); st = stage_proxies(th, ph); fx = trimesh.util.concatenate([bpr[k]["mesh"] for k in ("bal_tube", "bal_tubecap", "bal_post", "bal_sheave", "bal_piston", "bal_spring")])
        arms = trimesh.util.concatenate([bpr["bal_collar"]["mesh"], bpr["bal_pin"]["mesh"]]); cen["poses"] += 1
        for lab, a_, b_ in (("cord_vs_M1", bpr["bal_cord"]["mesh"], st["M1"]), ("armside_vs_fixed", arms, fx), ("fixed_vs_M1", fx, st["M1"]), ("fixed_vs_M2", fx, st["M2"])):
            if mind(a_, b_) < 3.0: cen[lab] += 1; first.setdefault(lab, []).append((float(th), float(ph)))
cen["th1_range_with_hits"] = {k: [min(t for t, _ in v), max(t for t, _ in v)] for k, v in first.items()}

# ---------------------------------------------------------------- export posed park meshes + report
th_p, ph_p = P.PARK["S1p"], P.PARK["S2"]; bp = balancer_parts(D0, th_p, ph_p, "lateral")
for k, v in bp.items(): v["mesh"].export(os.path.join(OUT, f"{k}_R.stl"))
masses = {k: v["mass_g"] for k, v in bp.items()}; fixed_g = sum(v for k, v in masses.items() if k not in ("bal_collar", "bal_pin")); arm_g = masses["bal_collar"] + masses["bal_pin"]
bom = dict(balancer_total_g=round(fixed_g + arm_g + 24.4 + 22.7, 1), note="parts: 1 spring, 2 x sheave (fixed swivel + movable tackle pulley on the piston) with DU bush, 2 x 6800ZZ swivel bearings, ~0.9 m Dyneema 1.5 mm, collar + pin; hardware as baseline (+24 g bearings, +23 g fasteners)",
           baseline_two_balancers_g=2 * 329, baseline_S2_balancer_g=329, delta_vs_two_baseline_balancers_g="%.0f" % (fixed_g + arm_g + 47.1 - 2 * 329),
           vendor_PLACEHOLDERS=["spring (custom A228)", "Dyneema SK78/SK99 MBL + creep", "sheave DU bush 0404DU + 4 mm pin", "6800ZZ bearing ratings", "AK70 S2 output-bearing tilt capacity (cord pull acts on the arm: radial %.0f N)" % (TACKLE * fmax)])
rep = dict(collision_census_radial_soft=cen, rev="J1r2", design=GEOM, torque=res, spring=SPRING, free_fall=ffs, spring_failure=sf, static_clearance_vs_J1_stage=clr, masses_g=masses, fixed_side_g=round(fixed_g, 1), arm_side_g=round(arm_g, 1), bom=bom,
           ax_note="S2 and S1' both balanced by the single pad-anchored spring (cos th1 coupling is automatic); yoke-anchored S2 (baseline) would need the decoupled pair: see torque.decoupled_alt")
json.dump(rep, open(os.path.join(OUT, "j1r2_balancer.json"), "w"), indent=1, default=lambda o: float(o) if isinstance(o, (np.floating,)) else (int(o) if isinstance(o, np.integer) else str(o)))
print(json.dumps(dict(torque=res, spring=SPRING, free_fall={k: v for k, v in ffs.items() if k in ("energies", "braking_need")}, spring_failure={k: v for k, v in sf.items() if k != "firmware"}, clearance=clr, bom=bom), indent=1, default=float))
