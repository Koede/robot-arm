"""S2 gravity balancer (open item I1b), rev e 2026-10-15: ZERO-LENGTH spring.  Torque model, spring / cord / sheave design, residual over the full
soft grid, free-fall energy.  Right arm, robot frame, mm / N / N m / J / deg.  Single source of numbers for balancer.py and every check script.

WHY (S2 reconciliation, specs/SPEC_balancer.md section 12): a spring between the S2 stator (M1) and the S2 output (M2) only sees S2, so the best it can do is
the per-S2 midpoint of the gravity envelope over S3, E, W and payload.  A true zero-length spring gives  tau_b = K sin(S2)  with K = k a b, which is within
0.2 N m of that bound; the rev d extension spring (L_eff 33.7 mm, phase-shifted anchor, tuned to E 0 + payload only) gave SF 1.29 on the mesh grid.

MECHANISM (zero-length by construction).
  * A  (stator, M1 frame) = point on the swivel axis, (A_X, Y_B, A_Z) = (0, 50, 22): a = 22 mm above P.
  * B  (arm, M2 frame)    = centre of a steel shoulder pin, axis y, (B_X, Y_B, B_Z) = (0, 50, 47): b = 47 mm along the arm axis.  The spliced cord eye pivots on it,
                            so the cord line passes through the pin centre.
  * Cord: B -> sheave (pitch radius R_P) -> along the swivel axis (+y) -> through the spring -> piston.  The sheave sits in a fork that swivels freely about the
    y-parallel axis through A (two 6800 bearings in the tube's medial cap).  The cord arrives along that axis, wraps 90 deg and leaves from
    T2 = A + R_P e - R_P y^  in the direction e (unit, in the x-z plane, towards B).  B lies in the plane y = Y_B = y(T2), so the free cord is perpendicular to y and
    its line passes exactly through A (T2 - A = R_P e is along e).  Cord length B..piston = |B - A|_xz - R_P + pi R_P / 2 + const  ->  spring compression
    delta = |B - A|_xz + c0, and the assembly preload is set so that c0 = 0 (cord-length adjuster, calibration at S2 = 0: F = k (b - a)).
  * Spring: music wire ASTM A228 compression spring, squared and ground ends, in the guide tube along +y (lateral) from the swivel cap, cord through its bore.
    The cord pulls the piston medially and compresses the spring against the cap; the axial cord pull on the swivel and the spring push on the cap cancel
    inside the tube, so the bracket only carries F e at A.
  Force on the arm at B:  F = k (A - B)_xz  (zero y component).  Moment about +y (restoring for S2 > 0):
        tau_b = k (B x A)_y = k a b sin(S2)        [exact for A = (0, ., a), B = Ry(-S2) (0, ., b)]
  Net forward-falling moment  T = T_g - tau_b ; AK70 S2 holding torque = |T| (+ friction hysteresis, below).
Run:  python3 -I balancer_calc.py   (writes stl/balancer/balancer_calc.json)"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "..", "stl", "balancer"); os.makedirs(OUT, exist_ok=True)
G = 9.81
S2_SOFT = (-30.0, 88.0); S2_POS_STOP, S2_NEG_STOP = 91.0, -35.0       # 91 = body_mitigations.S2_STOP (bumper hard stop); -35 = rom_limits HARD
S2_BACKSTOP = 94.0                                                    # rigid yoke backstop: the spring must not go solid before this
RATED, PEAK = 8.3, 24.8                                              # AK70-10 N m (verified vendor page, DESIGN_STATE section 3)

# ---------------------------------------------------------------------------------------------- geometry (single source for balancer.py)
A_X, A_Z = 0.0, 22.0        # swivel axis point A in the M1 (yoke) frame, x-z (mm).  a = |A| = 22
B_X, B_Z = 0.0, 47.0        # cord pin B in the arm (M2) frame, x-z.  b = 47.  B - A = 25 mm at S2 = 0: fork cheek tip (16.25 from A) + pin head r 5 + 3.75 clearance
Y_B = 50.0                  # cord plane (y of B and of the sheave departure point T2); lateral of the dome apex (46.3)
Y0 = Y_B                    # legacy name used by older scripts
R_P = 6.75                  # sheave pitch radius: groove root dia 12.0 for the 1.5 mm cord (D/d = 8), flange radius 8.5
SHEAVE_R, SHEAVE_W = 8.5, 5.0
Y_C = Y_B + R_P             # sheave centre plane (y) = 56.75; swivel axis passes through (A_X, ., A_Z)
a_mm, b_mm = float(np.hypot(A_X, A_Z)), float(np.hypot(B_X, B_Z))
PHASE_DEG = float(np.degrees(np.arctan2(B_X, B_Z) - np.arctan2(A_X, A_Z)))     # 0: pure K sin(S2)

# ---------------------------------------------------------------------------------------------- spring: ASTM A228 music wire, compression, squared + ground
G_STEEL = 80000.0           # N/mm2 shear modulus A228, d > 3.18 mm (Shigley table 10-5: 11.6 Mpsi = 80.0 GPa); the maker trims N_a to hit k on test
E_STEEL = 196500.0
WIRE_D = 3.2                # mm (standard A228 size).  3.2 / N_a 7.5 keeps the tube end inside the two-arm reach limit (verify_balancer: tubes at S1 150 both sides)
UTS = lambda d: 2211.0 / d ** 0.145      # MPa, A228 (Shigley table 10-4: A = 2211 MPa mm^m, m = 0.145)
K_TARGET = float(os.environ.get("BAL_K", 3.78))   # N m; refreshed below by the grid minimax (optimise_K) and written to balancer_calc.json
N_ACTIVE = 7.5              # active coils (quarter turns); D is solved so that k a b = K exactly
N_END = 2.0                 # squared and ground: total coils = N_a + 2
CLASH = 0.12                # fractional overrun to closure (of the working deflection at S2 88; Shigley suggests >= 0.15, 0.12 keeps the tube inside the two-arm reach; the 94 deg backstop still leaves > 4 mm to solid)
def k_needed(K=None): return (K_TARGET if K is None else K) / (a_mm * b_mm * 1e-6) / 1e3      # N/mm
def coil_D_for(k, N=None, d=None):
    N = N_ACTIVE if N is None else N; d = WIRE_D if d is None else d; return (G_STEEL * d ** 4 / (8.0 * N * k)) ** (1.0 / 3.0)
def spring_k(N=N_ACTIVE, d=WIRE_D, D=None): return G_STEEL * d ** 4 / (8 * D ** 3 * N)
def bergstrasser(C): return (4 * C + 2) / (4 * C - 3)
def spring_tau(F, d=WIRE_D, D=None):
    C = D / d; return bergstrasser(C) * 8 * F * D / (np.pi * d ** 3)

# ---------------------------------------------------------------------------------------------- cord + sheave + swivel (bought parts; PLACEHOLDER vendor data)
CORD = dict(type="Dyneema SK78 12-strand braid, 1.5 mm (low-creep SK99/DM20 preferred)", d=1.5, MBL_N=1700.0,
            splice_eff=0.85, bend_eff_Dd8=0.80, note="PLACEHOLDER: MBL, splice and bend efficiencies from typical vendor tables; confirm with the chosen cord")
SHEAVE = dict(part="6061-T6 sheave OD 17, groove root dia 12, width 5, PTFE-composite bush GGB 0404DU (4 x 5.5 x 4) on a 4 mm h7 hardened pin", pitch_r=R_P,
              bush_p_static_MPa=250.0, bush_p_dyn_MPa=140.0, note="a 693ZZ (C0 ~ 0.18 kN) cannot carry F sqrt 2; DU values from the GGB catalogue, PLACEHOLDER until purchased")
SWIVEL_BRG = dict(part="2 x 6800ZZ (10 x 19 x 5): No 1 in the cap flange (y 70..75, locating), No 2 at the end of the cap hub inside the spring bore (y 93..98, floating)",
                  centres_y=(72.5, 95.5), C0_N=1040.0, C_N=1720.0, note="catalogue values (NSK 6800), PLACEHOLDER until purchased")
FRICTION_FRAC = 0.05        # hysteresis: DU-bush sheave (mu 0.08: 3.4 % of F), swivel ball bearings, coil/tube rub; fraction of the balancer torque (estimate; bench T12b)

def spring_design(K=None):
    K = K_TARGET if K is None else K
    k = k_needed(K); D = coil_D_for(k); d = WIRE_D; C = D / d
    dmin, d88, d91, d94, dm35 = (dxz(t) for t in (0.0, 88.0, S2_POS_STOP, S2_BACKSTOP, S2_NEG_STOP))
    Nt = N_ACTIVE + N_END; Ls = Nt * d                                  # solid length (squared + ground)
    y_work = d88; L_at_88 = Ls + CLASH * y_work                           # closest working length (Shigley overrun xi = 0.15)
    L_free = L_at_88 + d88                                                # zero-length: compression from free = |AB|_xz
    L_inst = L_free - dmin                                                # longest installed length (S2 = 0)
    L_94 = L_free - d94
    F = lambda dd: k * dd
    tmin, tmax = spring_tau(F(dmin), d, D), spring_tau(F(d94), d, D)
    Sut = UTS(d); Ssu = 0.67 * Sut; Ssy = 0.45 * Sut                      # Shigley: torsional ultimate 0.67 Sut, set-removed yield 0.65 / as-wound 0.45 Sut
    fat = {}
    for lab, Ssa, Ssm in (("unpeened", 241.0, 379.0), ("shot_peened", 398.0, 534.0)):      # Zimmerli 1e7 (Shigley eq. 10-28)
        Sse = Ssa / (1 - (Ssm / Ssu) ** 2); ta = (tmax - tmin) / 2; tm = (tmax + tmin) / 2
        nf = 0.5 * (Ssu / tm) ** 2 * ta / Sse * (-1 + np.sqrt(1 + (2 * tm * Sse / (Ssu * ta)) ** 2))     # Gerber
        fat[lab] = dict(S_se_MPa=round(Sse, 0), tau_a=round(ta, 0), tau_m=round(tm, 0), n_f_Gerber=round(float(nf), 2))
    tau_solid = spring_tau(k * (L_free - Ls), d, D)
    # buckling: guided in the tube; free-standing check for information (alpha 0.5 both ends fixed: L0 < 2.63 D / alpha)
    pitch = (L_free - 2 * d) / N_ACTIVE                                   # squared+ground: L0 = p Na + 2d
    OD_free = D + d; OD_solid = np.sqrt(D ** 2 + (pitch ** 2 - d ** 2) / np.pi ** 2) + d       # Shigley eq. 10-? diameter growth on closure
    mass = np.pi * D * Nt * np.pi / 4 * d ** 2 * 7.85e-3
    U = lambda dd: 0.5 * k * dd ** 2 * 1e-3
    return dict(K_Nm=round(K, 4), k_N_per_mm=round(k, 4), wire_d=d, D_mean=round(D, 2), OD=round(OD_free, 2), OD_at_solid=round(OD_solid, 2), ID=round(D - d, 2), C=round(C, 2),
                N_active=N_ACTIVE, N_total=Nt, pitch_free=round(pitch, 2), L_free=round(L_free, 1), L_solid=round(Ls, 1), L_installed_S2_0=round(L_inst, 1),
                L_at_S2_88=round(L_at_88, 1), L_at_S2_94=round(L_94, 1), margin_to_solid_at_94_mm=round(L_94 - Ls, 1),
                deflection_from_free_mm=dict(S2_0=round(dmin, 1), S2_m35=round(dm35, 1), S2_88=round(d88, 1), S2_91=round(d91, 1), S2_94=round(d94, 1)),
                stroke_mm=round(d94 - dmin, 1), force_N=dict(S2_0=round(F(dmin), 1), S2_m35=round(F(dm35), 1), S2_88=round(F(d88), 1), S2_94=round(F(d94), 1)),
                tau_min_MPa=round(tmin, 0), tau_max_MPa=round(tmax, 0), tau_solid_MPa=round(tau_solid, 0), Sut_MPa=round(Sut, 0), tau_max_over_Sut=round(tmax / Sut, 3),
                SF_static_vs_0p45Sut=round(Ssy / tmax, 2), SF_solid_vs_0p45Sut=round(Ssy / tau_solid, 2), fatigue_1e7=fat,
                windable=bool(4.0 <= C <= 12.0), Bergstrasser_KB=round(bergstrasser(C), 4),
                buckling_free_standing_L0_over_D=round(L_free / D, 2), buckling_absolutely_stable=bool(L_free / D < 2.63 / 0.5),
                buckling_note="squared + ground ends on flat seats (alpha 0.5): absolutely stable if L0/D < 2.63/alpha = 5.26 (Shigley eq. 10-10); it also runs guided in the tube bore (radial clearance 1.25 at solid)",
                stored_energy_J=dict(S2_0=round(U(dmin), 2), S2_88=round(U(d88), 2), S2_94=round(U(d94), 2)), mass_g=round(mass, 1),
                spec_note="PLACEHOLDER vendor: custom wound A228, squared and ground, shot-peened, set removed; or the nearest stock spring re-checked with this script")

# ---------------------------------------------------------------------------------------------- kinematics of the zero-length pair
def Ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
A_PT = np.array([A_X, Y_B, A_Z]); B_ARM = np.array([B_X, Y_B, B_Z])
def B_at(t): return Ry(-np.radians(t)) @ B_ARM
def dxz(t): v = A_PT - B_at(t); return float(np.hypot(v[0], v[2]))
def e_dir(t):
    """unit vector A -> B in the x-z plane (the swivel / sheave direction)"""
    v = B_at(t) - A_PT; v[1] = 0.0; return v / np.linalg.norm(v)
def k_spring(): return SPRING["k_N_per_mm"]
def tau_b(t, k=None, offset_mm=0.0):
    """balancer moment on the arm about +y (N m), restoring = positive for t > 0.  offset_mm = preload calibration error (spring compression - |AB|)"""
    k = k_spring() if k is None else k; B = B_at(t); v = A_PT - B; v[1] = 0.0; d = np.linalg.norm(v)
    F = k * (d + offset_mm) * v / d
    return (B[2] * F[0] - B[0] * F[2]) * 1e-3
def cord_force(t, k=None): k = k_spring() if k is None else k; return k * dxz(t)

# ---------------------------------------------------------------------------------------------- arm gravity: mesh mass model (s2_massmodel.py / s2_grid.py)
_MM = None
def _mm():
    global _MM
    if _MM is None: _MM = json.load(open(os.path.join(OUT, "s2_massmodel.json")))
    return _MM
EXTRA_UPPER_G = 40.0        # wiring / fastener allowance on the upper arm (z 150, M3 frame), same as harness_loads.py and s2_grid.py
_CV = {}
def c_vec(e, pay_kg, s3=0.0, w=(0.0, 0.0, 0.0)):
    """first moment (kg m) of everything beyond S2 (+ payload at the claw tip) in the S2-output frame, S2 = 0"""
    key = (e, pay_kg, s3, w)
    if key in _CV: return _CV[key]
    import arm_kin as ak
    MM = _mm(); q = dict(S1=0.0, S2=0.0, S3=s3, E=e, W1=w[0], W2=w[1], W3=w[2], G=50.0); T = ak.frames(q); c = np.zeros(3)
    for g in ("M2", "M3", "ME", "MW1", "MW2", "MW3"):
        c += T[g][:3, :3] @ (np.array(MM[g]["first_moment_gmm"]) * 1e-6) + MM[g]["mass_g"] * 1e-3 * T[g][:3, 3] * 1e-3
    c += EXTRA_UPPER_G * 1e-3 * (T["M3"][:3, :3] @ np.array([0, 0, 0.150]) + T["M3"][:3, 3] * 1e-3)
    if pay_kg: c += pay_kg * (T["MW3"] @ np.array([0, ak.LAY["D_OFF"], ak.LAY["Z_TIP"], 1.0]))[:3] * 1e-3
    _CV[key] = c; return c
def Tg(t, e, pay=500.0, extra=None):
    """forward-falling gravity moment about the S2 axis (N m), straight wrist, S3 0.  extra = [(m_g, x_mm, z_mm)] arm-side added masses (arm frame)"""
    c = c_vec(float(e), pay * 1e-3); th = np.radians(t)
    T = G * (c[2] * np.sin(th) - c[0] * np.cos(th))
    for m, ux, uz in (extra or []): T += G * m * 1e-6 * (uz * np.sin(th) - ux * np.cos(th))
    return float(T)

# ---------------------------------------------------------------------------------------------- full soft grid (s2_grid.npz) residual
def grid():
    z = np.load(os.path.join(OUT, "s2_grid.npz")); return z["A"], z["S2"], z["TG"]
def grid_residual(tb_arr, extra=None, A=None, S2=None, TG=None):
    if TG is None: A, S2, TG = grid()
    th = np.radians(S2); dT = np.zeros_like(th)
    for m, ux, uz in (extra or []): dT += G * m * 1e-6 * (uz * np.sin(th) - ux * np.cos(th))
    R = TG + dT[None] - np.asarray(tb_arr)[None]
    i, j = np.unravel_index(np.abs(R).argmax(), R.shape)
    return R, float(abs(R).max()), dict(S2=float(S2[j]), S3=float(A[i, 0]), E=float(A[i, 1]), W1=float(A[i, 2]), W2=float(A[i, 3]), W3=float(A[i, 4]), payload_kg=float(A[i, 5]), residual_Nm=round(float(R[i, j]), 3))
def optimise_K(extra=None):
    from scipy.optimize import minimize_scalar
    A, S2, TG = grid(); s = np.sin(np.radians(S2))
    r = minimize_scalar(lambda K: grid_residual(K * s, extra, A, S2, TG)[1], bounds=(2.0, 6.0), method="bounded", options=dict(xatol=1e-4))
    return float(r.x), float(r.fun)

# ---------------------------------------------------------------------------------------------- free-fall energy with the net potential (API kept from rev d)
S_GRID = np.arange(S2_NEG_STOP, S2_POS_STOP + 1e-9, 0.5)
_TGC = {}
def Tg_arr(e, pay, extra=None, grid=S_GRID):
    key = (e, pay, tuple(map(tuple, extra or [])), len(grid))
    if key not in _TGC: _TGC[key] = np.array([Tg(s, e, pay, extra) for s in grid])
    return _TGC[key]
def T_net(e, pay, tb_fun, extra=None, grid=S_GRID):
    return Tg_arr(e, pay, extra, grid) - np.array([tb_fun(s) for s in grid])
def impact(T, s0, grid=S_GRID):
    """Rest at s0, move in the direction of T(s0).  Returns (W_at_stop J, '+'|'-'|None).  W = int T dt (rad); a turning point (W < 0) means a well: no stop impact."""
    i0 = int(np.argmin(abs(grid - s0))); ds = np.radians(grid[1] - grid[0]); Ti = T[i0]
    if abs(Ti) < 1e-6: return 0.0, None
    step = 1 if Ti > 0 else -1; W = 0.0; i = i0
    while 0 <= i + step < len(grid):
        W += 0.5 * (T[i] + T[i + step]) * ds * step; i += step
        if W < 0: return 0.0, None
    return W, ('+' if step > 0 else '-')
POSES = [(0, 500.0), (0, 0.0), (45, 500.0), (90, 500.0), (90, 0.0), (145, 500.0), (145, 0.0)]     # (E deg, payload g)
STARTS = list(range(-30, 89, 3))
def energies(tb_fun, extra=None, poses=POSES):
    wp = wn = 0.0; arg = {}
    for (e, pay) in poses:
        T = T_net(e, pay, tb_fun, extra)
        for s0 in STARTS:
            W, w = impact(T, s0)
            if w == '+' and W > wp: wp = W; arg['+'] = (e, pay, s0)
            if w == '-' and W > wn: wn = W; arg['-'] = (e, pay, s0)
    return wp, wn, arg

# ---------------------------------------------------------------------------------------------- arm-side added mass (refreshed from verify_balancer.json)
EXTRA_ARM = [(60.0, 0.0, 70.0)]       # g, x, z in the arm frame: collar + cap + lug + pin + half the cord (estimate until verify_balancer has run)
_vj = os.path.join(OUT, "verify_balancer.json")
if os.path.exists(_vj):
    try:
        _v = json.load(open(_vj))
        if _v.get("rev") == "e": EXTRA_ARM = [(float(_v["mass"]["arm_side_g"]), float(_v["mass"]["arm_side_cg_xz"][0]), float(_v["mass"]["arm_side_cg_xz"][1]))]
    except Exception: pass

# the spring is sized for the K that minimises the grid residual WITH the balancer's own arm-side mass
_kj = os.path.join(OUT, "balancer_calc.json")
if "BAL_K" not in os.environ and os.path.exists(_kj):
    try:
        _k = json.load(open(_kj))
        if _k.get("rev") == "e": K_TARGET = float(_k["K_design_Nm"])
    except Exception: pass
SPRING = spring_design(K_TARGET)

# ---------------------------------------------------------------------------------------------- tube / piston layout along y (M1 frame), used by balancer.py
TUBE = dict(cap_y0=70.0, cap_len=8.0, hub_y1=100.0, hub_OD=23.0, piston_t=5.0, end_t=6.0, buffer_t=6.0, buffer_gap=1.0, wall=2.5, radial_clr=1.25,
            buffer="TPU 85A ring 6 mm (cord-break catch: 4.3 J at 17 m/s -> ~4 kN peak, see balancer_struct.py)")
def tube_layout(sp=None):
    sp = sp or SPRING; T = dict(TUBE)
    T["seat_y"] = T["cap_y0"] + T["cap_len"]                               # spring seat (medial)
    T["piston_y_S2_0"] = T["seat_y"] + sp["L_installed_S2_0"]              # piston medial face at S2 = 0 (spring longest)
    T["piston_y_S2_94"] = T["seat_y"] + sp["L_at_S2_94"]
    T["ID"] = round(sp["OD_at_solid"] + 2 * T["radial_clr"], 2); T["OD"] = round(T["ID"] + 2 * T["wall"], 2)
    T["inner_end_y"] = T["piston_y_S2_0"] + T["piston_t"] + T["buffer_gap"] + T["buffer_t"]   # TPU buffer ring catches the piston only if the cord breaks (1 mm free at S2 = 0)
    T["y_end"] = T["inner_end_y"] + T["end_t"]
    T["length"] = T["y_end"] - T["cap_y0"]
    return T

# ---------------------------------------------------------------------------------------------- part poses (shared by verify_balancer.py and render_balancer.py)
PART_GROUP = {"bal_anchor": "M1", "bal_tubecap": "M1", "tpu_bal_buffer": "M1", "bal_piston": "PISTON", "bal_swivel": "SWIVEL", "bal_sheave": "SWIVEL",
              "bal_collar": "M2", "bal_collar_cap": "M2", "bal_pin_proxy": "M2", "tpu_s2_pos_bump": "M1", "tpu_s2_neg_bump": "M1"}
def swivel_angle(t):
    e = e_dir(t); return float(np.arctan2(e[0], e[2]))          # rotation about +y that takes the built +z fork direction onto e
def _about_axis(phi):
    c, s = np.cos(phi), np.sin(phi); T = np.eye(4); T[:3, :3] = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    A = np.eye(4); A[0, 3], A[2, 3] = A_X, A_Z; Ai = np.eye(4); Ai[0, 3], Ai[2, 3] = -A_X, -A_Z
    return A @ T @ Ai
def piston_shift(t): return -(dxz(t) - dxz(0.0))               # mm along +y relative to the S2 = 0 export
def part_transforms(frames_M1, frames_M2, t):
    """robot-frame 4x4 per balancer part, given the arm_kin M1 / M2 frames and S2 = t (deg)"""
    Tsw = frames_M1 @ _about_axis(swivel_angle(t)); Tp = frames_M1.copy(); Tp = Tp @ np.array([[1, 0, 0, 0], [0, 1, 0, piston_shift(t)], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    G = {"M1": frames_M1, "M2": frames_M2, "SWIVEL": Tsw, "PISTON": Tp}
    return {k: G[g] for k, g in PART_GROUP.items()}
def cord_points(frames_M1, frames_M2, t):
    """robot-frame end points of the free cord (T2 on the sheave -> B pin centre) and of the axial run (T1 -> piston)"""
    e = e_dir(t); T2 = A_PT + R_P * e; T2[1] = Y_B; Bp = B_ARM
    p = lambda M, v: (M @ np.r_[v, 1.0])[:3]
    T1 = np.array([A_X, Y_C, A_Z]); Pis = np.array([A_X, SPRING_SEAT_Y() + SPRING["L_free"] - dxz(t), A_Z])
    return p(frames_M1, T2), p(frames_M2, Bp), p(frames_M1, T1), p(frames_M1, Pis)
def SPRING_SEAT_Y(): return tube_layout()["seat_y"]

def report():
    out = dict(rev="e", design="zero-length compression spring + cord over a swivelling sheave (Track B concept), see SPEC_balancer.md section 12")
    # 1. K: minimax over the full soft grid including the balancer's arm-side mass
    Kopt, rK = optimise_K(EXTRA_ARM)
    out["K_design_Nm"] = round(Kopt, 4); out["grid_minimax_residual_Nm"] = round(rK, 3)
    sp = spring_design(Kopt); out["spring"] = sp
    out["geometry"] = dict(A_stator=[A_X, Y_B, A_Z], B_arm=[B_X, Y_B, B_Z], a_mm=a_mm, b_mm=b_mm, phase_deg=PHASE_DEG, cord_plane_y=Y_B, sheave_centre_y=Y_C, sheave_pitch_r=R_P,
                           AB_xz_mm=dict(S2_m35=round(dxz(-35), 1), S2_0=round(dxz(0), 1), S2_88=round(dxz(88), 1), S2_94=round(dxz(94), 1)),
                           swivel_angle_range_deg=[round(float(np.degrees(np.arctan2(e_dir(t)[2], e_dir(t)[0]))), 1) for t in (S2_NEG_STOP, 0.0, S2_BACKSTOP)],
                           note="swivel angle = direction of e in the x-z plane, atan2(z, x); it rotates monotonically through the range")
    out["tube"] = tube_layout(sp)
    # 2. torque over the grid
    A, S2, TG = grid(); k = sp["k_N_per_mm"]
    tb = np.array([tau_b(x, k) for x in S2])
    R, rmax, worst = grid_residual(tb, EXTRA_ARM, A, S2, TG)
    _, rnone, wnone = grid_residual(0 * tb, None, A, S2, TG)
    fr = FRICTION_FRAC * np.abs(tb)[None]; rfric = float((np.abs(R) + fr).max())
    out["grid"] = dict(n_poses=int(len(A)), S2_values=int(len(S2)), axes="S3 -70..90 (20), E 0..145 (5), W1 -80/0/80, W2 -70..80 (15), W3 -20/0/30, payload 0/0.5 kg, G 50, +40 g upper-arm allowance",
                       no_balancer_max_Nm=round(rnone, 3), no_balancer_worst=wnone, SF_rated_no_balancer=round(RATED / rnone, 2),
                       with_balancer_max_Nm=round(rmax, 3), worst=worst, SF_rated=round(RATED / rmax, 2), SF_peak=round(PEAK / rmax, 2),
                       with_friction_hysteresis_max_Nm=round(rfric, 3), SF_rated_with_friction=round(RATED / rfric, 2), friction_frac=FRICTION_FRAC,
                       payload0_range_Nm=[round(float(R[A[:, 5] == 0].min()), 2), round(float(R[A[:, 5] == 0].max()), 2)],
                       payload05_range_Nm=[round(float(R[A[:, 5] == 0.5].min()), 2), round(float(R[A[:, 5] == 0.5].max()), 2)],
                       balancer_Nm=dict(m30=round(tau_b(-30, k), 3), zero=round(tau_b(0, k), 3), p45=round(tau_b(45, k), 3), p88=round(tau_b(88, k), 3)),
                       gate_SF_ge_2=bool(RATED / rmax >= 2.0))
    # sensitivity: preload calibration error and K tolerance (spring rate +-5 %)
    sens = {}
    for lab, kk, off in (("k -5 %", 0.95 * k, 0.0), ("k +5 %", 1.05 * k, 0.0), ("preload +2 mm", k, 2.0), ("preload -2 mm", k, -2.0)):
        tb2 = np.array([tau_b(x, kk, off) for x in S2]); sens[lab] = round(RATED / grid_residual(tb2, EXTRA_ARM, A, S2, TG)[1], 2)
    out["grid"]["SF_sensitivity"] = sens
    # 3. residual curves (straight wrist, S3 0) for the spec table
    s = np.arange(S2_SOFT[0], S2_SOFT[1] + 1e-9, 1.0); curves = {"S2_deg": [float(x) for x in s[::6]]}; summ = {}
    for nm, e, pay in (("E0_pay0.5", 0, 500.0), ("E90_pay0.5", 90, 500.0), ("E145_pay0.5", 145, 500.0), ("E0_pay0", 0, 0.0), ("E145_pay0", 145, 0.0)):
        T = np.array([Tg(x, e, pay, EXTRA_ARM) for x in s]); B = np.array([tau_b(x, k) for x in s]); Rr = T - B
        curves[nm] = dict(gravity_Nm=[round(float(v), 2) for v in T[::6]], balancer_Nm=[round(float(v), 2) for v in B[::6]], residual_Nm=[round(float(v), 2) for v in Rr[::6]])
        summ[nm] = dict(peak_gravity_Nm=round(float(abs(T).max()), 2), peak_residual_Nm=round(float(abs(Rr).max()), 2), cancel_peak_pct=round(100 * (1 - abs(Rr).max() / abs(T).max()), 1))
    out["curves"] = curves; out["summary"] = summ
    # 4. cord
    Fmax = sp["force_N"]["S2_94"]; cap = CORD["MBL_N"] * CORD["splice_eff"] * CORD["bend_eff_Dd8"]
    out["cord"] = dict(CORD, F_max_N=Fmax, usable_N=round(cap, 0), SF=round(cap / Fmax, 2), gate="SF >= 5 (cordage, cyclic)", passes=bool(cap / Fmax >= 5.0),
                       stretch_mm_at_Fmax=round(Fmax * 160.0 / (0.6 * np.pi / 4 * CORD["d"] ** 2 * 100000.0), 2),
                       stretch_note="EA = 0.6 x (pi d^2/4) x 100 GPa (fibre fill 0.6), free length ~160 mm; series compliance error = stretch / |AB| < 0.5 % of k")
    # 5. AK70 S2 output bearing: cord force on the arm at B (y = Y_B), moment about the AK70 output-bearing centre (PLACEHOLDER y = -10: inside the motor)
    Y_BRG = -10.0; rows = []
    for t in np.arange(S2_NEG_STOP, S2_BACKSTOP + 1e-9, 1.0):
        B = B_at(t); v = A_PT - B; v[1] = 0; F = k * v
        Mo = np.cross((B - np.array([0, Y_BRG, 0])) * 1e-3, F)
        rows.append((t, float(np.linalg.norm(F)), float(np.hypot(Mo[0], Mo[2]))))
    out["AK70_S2_bearing"] = dict(max_radial_N=round(max(r[1] for r in rows), 1), max_tilting_moment_Nm=round(max(r[2] for r in rows), 2), lever_mm=Y_B - Y_BRG,
                                  bearing_centre_y_PLACEHOLDER=Y_BRG, at_S2=float(max(rows, key=lambda r: r[1])[0]),
                                  arm_gravity_radial_N=round((sum(v["mass_g"] for kk, v in _mm().items() if not kk.startswith("_")) + 40 + 500) * 1e-3 * G, 1),
                                  rev_d_for_comparison=dict(max_radial_N=233.6, max_moment_Nm=16.8),
                                  note="internal force pair between M1 (at A) and M2 (at B), collinear: S1 and the harness see no net load from the spring")
    # 6. free fall (unbraked, energy into the +91 / -35 stops), with and without the balancer
    tb = lambda x: tau_b(x, k)
    wp, wn, arg = energies(tb, EXTRA_ARM, POSES)
    env = {}
    for pays, nm in (((500.0,), "loaded_0.5kg"), ((0.0,), "empty")):
        a_ = b_ = 0.0
        for e in (0, 45, 90, 145):
            for pay in pays:
                T = T_net(e, pay, tb, EXTRA_ARM)
                for s0 in STARTS:
                    W, w = impact(T, s0)
                    if w == '+': a_ = max(a_, W)
                    if w == '-': b_ = max(b_, W)
        env[nm] = dict(pos_J=round(a_, 2), neg_J=round(b_, 2))
    base_p = 0.0
    for e in (0, 45, 90, 145):
        T = Tg_arr(e, 500.0, None)
        for s0 in STARTS:
            W, w = impact(T, s0)
            if w == '+': base_p = max(base_p, W)
    out["freefall_energy"] = dict(E_pos_stop_J=round(wp, 2), E_neg_stop_J=round(wn, 2), worst_args=arg, envelope=env, baseline_no_balancer_loaded_pos_J=round(base_p, 2),
                                  note="unbraked, no friction; impact model and pad pressures in balancer_freefall.py")
    return out

if __name__ == "__main__":
    r = report(); json.dump(r, open(os.path.join(OUT, "balancer_calc.json"), "w"), indent=1, default=float)
    print(json.dumps({k: v for k, v in r.items() if k not in ("curves",)}, indent=1, default=float))
