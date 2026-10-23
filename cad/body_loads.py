"""Body-load simulation of the Third Shoulder (HARNESS x CONTROL x SAFETY), rev 2026-10-14.  Robot frame, right arm; left arm mirrored about y = SP.

WHAT IT IS: a quasi-static 3-D rigid-body + foam-spring model.  The whole rig (both clasps, plates, wings, spine bridge, pods, both arms) is ONE rigid body
(the wings join across the spine) resting on unilateral foam springs: pad cells on F1..F4, a 4 x 16 cell grid on each wing rail, and lateral cradle cells.
Each cell: normal stiffness k = E a / t (t = foam thickness measured on the wearer mesh, E uniform), shear stiffness k/3, preload d0.  Equilibrium of
gravity (x dyn) + external wrench is solved for the 6-DOF rig displacement (damped Newton, finite-difference Jacobian).  Because E is uniform the pressure
DISTRIBUTION does not depend on E; only the rig displacement does.  It is NOT a finite-element tissue model: it shows where load goes and how big it is.

CASES  1 gravity grid (canonical + random pose pairs, both arms, 0.5 kg payload each, dyn 1.3 like harness_loads.py)
       2 validation against the 2-D sagittal model of harness_loads.py
       3 actuator-stall / blocked-arm wrench (S1, S2, S3 stall torques, 26 sign combinations) on top of gravity
       4 E-stop / power-loss free fall of S2 into the stop (energy, impulse, head sweep from keepout_map.npz)
       5 trunk loads (CG, % body weight, moment at C7 and L5) and nerve-site distances (estimated landmarks)
Writes stl/shoulder/body_loads.json and renders/body_loads.png."""
import os, sys, json, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import biomimetic_shoulder as bs, harness_loads as hl
S = os.path.join(HERE, "..", "stl", "shoulder"); RND = os.path.join(HERE, "..", "renders"); os.makedirs(RND, exist_ok=True)
G = 9.81; DYN = 1.3
SP = -150.0
E_FOAM_KPA = float(os.environ.get("E_FOAM_KPA", 200.0))     # EVA/PE foam effective compressive modulus (assumed; distribution is independent of it)
PRELOAD_MM = float(os.environ.get("PRELOAD_MM", 0.1))       # foam squeeze at donning
MU = float(os.environ.get("MU", 0.4))                                                    # textile-on-skin friction cap used only for the slip flag (assumed)
FOAM_T = 7.0                                                # clasp foam centre thickness (SPEC_harness)
RAIL_T = 6.0                                                # wing rail foam strip (SPEC_harness)
body = trimesh.load(os.path.join(S, "proxies", "wearer_makehuman.stl"))
# rev 2026-10-14 (strap/facet sprint): pad cell model switches.  PAD_MODEL auto = contoured pads on raster cells from stl/shoulder/pad_foam.npz (the built plate's
# foam-thickness field), other pads on the original 7 x 5 cells; legacy = original 7 x 5 cells everywhere (reproduces the 2026-10-14 numbers); raster = every pad on
# PAD_CELL mm raster cells (flat pads get their foam thickness from the same skin raster, so flat and contoured are compared on identical cell size).
PAD_MODEL = os.environ.get("PAD_MODEL", "auto"); PAD_CELL = float(os.environ.get("PAD_CELL", 5.0)); SKIRT_ETA = float(os.environ.get("SKIRT_ETA", 0.5))
SITES_JSON = os.environ.get("SITES_JSON", os.path.join(S, "clasp_sites.json")); PAD_NPZ = os.environ.get("PAD_NPZ", os.path.join(S, "pad_foam.npz"))
STRAP_T_N = float(os.environ.get("STRAP_T", 10.0))      # donning-tension mark on the front strap (was 12 N assumed)
sites = json.load(open(SITES_JSON))
hl.sites = sites                     # harness_loads.area()/contact() must see the same site set as the cell model (baseline controls)
frame = json.load(open(os.path.join(S, "scapular", "scapular_frame.json")))
rail = frame["zones"]["RAIL"]; cradle = json.load(open(os.path.join(S, "clasp_cradle.json")))
lm = json.load(open(os.path.join(S, "proxies", "wearer_landmarks.json")))
atl = json.load(open(os.path.join(S, "emg_atlas.json")))["landmarks"]

def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v)
def mirror(p): p = np.array(p, float).copy(); p[..., 1] = 2 * SP - p[..., 1]; return p

# ------------------------------------------------------------------ contact cells (right side), then mirrored
cells = []        # dict(name, group, r, n, a, t, d0)
def add_cell(name, group, r, n, a, t, d0, eta=1.0, skirt=False):
    cells.append(dict(name=name, group=group, r=np.array(r, float), n=unit(n), a=float(a), t=float(np.clip(t, 1.5, 20.0)), d0=float(d0), eta=float(eta), skirt=bool(skirt)))
PADS = ["F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion"]
def pad_cells(k, nu=7, nv=5):
    s = sites[k]; p, n, u = map(np.array, (s["p"], s["n"], s["u"])); n = unit(n); u = unit(u - n * (u @ n)); v = np.cross(n, u)
    w, h = s["plate"]; f = min(s.get("fillet", 12.0), min(w, h) / 2 - 1.0)
    A = hl.area(k); aa, bb = (np.arange(nu) + 0.5) / nu * w - w / 2, (np.arange(nv) + 0.5) / nv * h - h / 2
    pts, idx = [], []
    for a in aa:
        for b in bb:
            if abs(a) > w / 2 - f and abs(b) > h / 2 - f:      # rounded corner mask
                if np.hypot(abs(a) - (w / 2 - f), abs(b) - (h / 2 - f)) > f: continue
            pts.append((a, b))
    area_c = A / len(pts)
    for a, b in pts:
        q = p + n * FOAM_T + a * u + b * v
        loc, _, _ = body.ray.intersects_location([q], [-n]); t = FOAM_T
        if len(loc): t = float(np.min(np.linalg.norm(loc - q, axis=1)))
        add_cell(k, k, p + n * 0.0 + a * u + b * v + n * (FOAM_T - t) * 0 , n, area_c, t, PRELOAD_MM)
def pad_cells_raster(k):
    """raster cells (PAD_CELL mm) on the pad's bearing area; stiffness of a cell = E sum(a_i / t_i) over its 1 mm pixels (parallel springs) -> t_eff = A / sum(1/t_i)"""
    import facet as fc
    from shapely import contains_xy
    s = sites[k]; p, n, u = map(np.array, (s["p"], s["n"], s["u"])); n = unit(n); u = unit(u - n * (u @ n)); v = np.cross(n, u)
    w, h = s["plate"]; con = s.get("contour"); cu, cv = (con["centre_uv"] if con else s.get("centre_uv", (0.0, 0.0)))
    z = np.load(PAD_NPZ) if (con and os.path.exists(PAD_NPZ) and (k + "_t") in np.load(PAD_NPZ).files) else None
    if z is not None:
        aa, bb, t_f, ins = z[k + "_a"], z[k + "_b"], z[k + "_t"], z[k + "_inside"]
        if os.environ.get("AC_FILL", "0") == "1" and (k + "_inside0") in z.files: ins = z[k + "_inside0"]                    # ablation: F4 without the AC foam relief
        if os.environ.get("PAD_ROLL", "1") == "0" and (k + "_rise") in z.files: t_f = t_f - z[k + "_rise"]      # ablation: same contoured plate without the rim roll
        sk_m, sk_t = (z[k + "_skirt_mask"], z[k + "_skirt_t"]) if (k + "_skirt_mask") in z.files else (None, None)
        zin = None
    else:
        P0 = fc.outline_poly(w, h, min(s.get("fillet", 12.0), min(w, h) / 2 - 1.0), cu, cv)
        aa, bb, zs = fc.skin_height_grid(body, p, n, u, v, cu - w / 2 - 2, cu + w / 2 + 2, cv - h / 2 - 2, cv + h / 2 + 2, 1.0)
        AA_, BB_ = np.meshgrid(aa, bb, indexing="ij"); ins = contains_xy(P0, AA_, BB_); t_f = FOAM_T - zs; sk_m = sk_t = None
    AA, BB = np.meshgrid(aa, bb, indexing="ij")
    # local surface normal of the inner face for contoured plates: from the skin raster gradient (flat plates: the plate normal)
    if z is not None:
        _, _, zs_ = fc.skin_height_grid(body, p, n, u, v, aa[0], aa[-1], bb[0], bb[-1], 1.0)
        zs_f = fc.gaussian_filter(zs_, 4.0, mode="nearest"); ga, gb = np.gradient(zs_f, 1.0, 1.0)
    def emit(mask, tt, grp_skirt):
        for ca in np.arange(aa[0] + PAD_CELL / 2, aa[-1], PAD_CELL):
            for cb in np.arange(bb[0] + PAD_CELL / 2, bb[-1], PAD_CELL):
                m = mask & (AA >= ca - PAD_CELL / 2) & (AA < ca + PAD_CELL / 2) & (BB >= cb - PAD_CELL / 2) & (BB < cb + PAD_CELL / 2)      # half-open: every 1 mm pixel in exactly one cell
                A = float(m.sum())
                if A < 0.2 * PAD_CELL ** 2: continue
                t_eff = A / float(np.sum(1.0 / np.clip(tt[m], 1.5, 20.0))); a_c, b_c = AA[m].mean(), BB[m].mean()
                nn = n if z is None else unit(n - ga[m].mean() * u - gb[m].mean() * v)
                add_cell(k, k, p + a_c * u + b_c * v, nn, A, t_eff, PRELOAD_MM, eta=(SKIRT_ETA if grp_skirt else 1.0), skirt=grp_skirt)
    emit(ins, t_f, False)
    if sk_m is not None and SKIRT_ETA > 0: emit(sk_m, sk_t, True)
for k in PADS:
    if PAD_MODEL == "raster" or (PAD_MODEL == "auto" and sites[k].get("contour")): pad_cells_raster(k)
    else: pad_cells(k)
# wing rail: y 6..50 from the spine, z -400..-80; cells on the skin
def skin_x(y, z):
    loc, _, _ = body.ray.intersects_location([[200.0, y, z]], [[-1.0, 0, 0]]); return float(loc[:, 0].max()) if len(loc) else None
# rev 2026-10-14 (rail pressure study): optional RAIL_CFG (JSON env) overrides the rail footprint and adds a per-cell foam map; DEFAULT (no env) = unchanged behaviour.
#   {"y":[y0,y1], "z":[z0,z1], "ny":int, "nz":int,
#    "relief":{"w":mm, "E_f":factor at the medial edge (1 = none), "t_med":foam thickness at the medial edge (mm), "ramp":"lin"}}
# E factor and thickness blend linearly from the medial-edge value to (1, RAIL_T) at y0 + w (a compliance gradient: soft strip / chamfered or thicker foam next to the spine).
CFG = json.loads(os.environ.get("RAIL_CFG", "{}"))
rail = dict(rail); rail["y"] = CFG.get("y", rail["y"]); rail["z"] = CFG.get("z", rail["z"])
# default: the relief recorded in scapular_frame.json (zones.RAIL.relief: w, extra gap mm, E_f); RAIL_CFG overrides; no relief key = old uniform foam
RELIEF = CFG["relief"] if "relief" in CFG else ({"w": rail["relief"]["w"], "E_f": rail["relief"]["E_f"], "t_med": RAIL_T + rail["relief"]["extra"]} if rail.get("relief") else None)
_fine = bool(RELIEF) or "ny" in CFG     # a compliance gradient needs ~4 mm columns; the legacy uniform rail keeps its 4 x 16 grid
NYR = int(CFG.get("ny", round((rail["y"][1] - rail["y"][0]) / 4.0) if _fine else 4)); NZR = int(CFG.get("nz", round((rail["z"][1] - rail["z"][0]) / 20.0) if _fine else 16))
def relief_profile(y_rel):
    if not RELIEF: return 1.0, RAIL_T
    s_ = float(np.clip((y_rel - rail["y"][0]) / RELIEF["w"], 0, 1))
    return RELIEF["E_f"] + (1 - RELIEF["E_f"]) * s_, RELIEF["t_med"] + (RAIL_T - RELIEF["t_med"]) * s_
yc = SP + np.linspace(rail["y"][0], rail["y"][1], NYR + 1); yc = 0.5 * (yc[:-1] + yc[1:]); zc = np.linspace(rail["z"][0], rail["z"][1], NZR + 1); zc = 0.5 * (zc[:-1] + zc[1:])
dy, dz = (rail["y"][1] - rail["y"][0]) / NYR, (rail["z"][1] - rail["z"][0]) / NZR
for z in zc:
    for y in yc:
        x0 = skin_x(y, z); xa, xb = skin_x(y + 5, z), skin_x(y - 5, z); xc, xd = skin_x(y, z + 5), skin_x(y, z - 5)
        if None in (x0, xa, xb, xc, xd): continue
        n = unit([1.0, -(xa - xb) / 10.0, -(xc - xd) / 10.0]); ef, tt = relief_profile(y - SP)
        add_cell("rail", "rail", [x0, y, z], n, dy * dz, tt, PRELOAD_MM); cells[-1]["ef"] = ef; cells[-1]["yrel"] = float(y - SP)
# axilla cradle: lateral locating pads (chest side and arm side), 3 mm foam rim, clearances 6.0 - 3 and 4.4 - 3
tilt = np.radians(cradle["TILT"]); A0 = np.array(cradle["A"])
for sgn, gap, nm in ((+1, 6.0 - 3.0, "cradle_chest"), (-1, cradle["arm_gap"] - 3.0 if False else 4.4 - 3.0, "cradle_arm")):
    nrm = unit([0.0, sgn * np.cos(tilt), sgn * np.sin(tilt)])
    for dx in np.linspace(-cradle["CL"] / 2 + 8, cradle["CL"] / 2 - 8, 3):
        for dzz in np.linspace(-cradle["CH"] / 2 + 10, cradle["CH"] / 2 - 10, 3):
            add_cell(nm, "cradle", A0 + [dx, -sgn * 2.0, dzz], nrm, cradle["CL"] * cradle["CH"] / 9, 3.0, -gap)
nR = len(cells)
# mirrored (left) cells
for c in list(cells):
    m = dict(c); m["r"] = mirror(c["r"]); m["n"] = c["n"] * np.array([1, -1, 1]); m["name"] = c["name"] + "_L"; m["group"] = c["group"] + "_L"; cells.append(m)
NC = len(cells)
R = np.array([c["r"] for c in cells]); N = np.array([c["n"] for c in cells]); AR = np.array([c["a"] for c in cells]); T = np.array([c["t"] for c in cells]); D0 = np.array([c["d0"] for c in cells])
EF = np.array([c.get("ef", 1.0) for c in cells]); YREL = np.array([c.get("yrel", np.nan) for c in cells])
ETA = np.array([c.get("eta", 1.0) for c in cells]); SK = np.array([c.get("skirt", False) for c in cells])
FOAM_GRADE = {k: float(v) for k, v in (kv.split("=") for kv in os.environ.get("FOAM_GRADE", "").split(",") if kv)}     # per-pad foam modulus factor (soft grade < 1), e.g. FOAM_GRADE=F2_ant_delt=0.7
for _i, _c in enumerate(cells):
    _g = _c["group"].replace("_L", "") if _c["group"].endswith("_L") else _c["group"]
    if _g in FOAM_GRADE: ETA[_i] *= FOAM_GRADE[_g]
KN = ETA * EF * E_FOAM_KPA * 1e3 * AR * 1e-6 / (T * 1e-3)        # N/m  (TPU skirt cells: effectiveness ETA)
KS = KN / 3.0
GR = np.array([c["group"] for c in cells])
MU_RAIL = float(os.environ.get("MU_RAIL", MU))                    # rev f: friction of the sliding rail facing only (SPEC_rail_facing 8); default = global MU (no change)
MUC = np.where(np.char.startswith(GR.astype(str), "rail"), MU_RAIL, MU)

# ------------------------------------------------------------------ equilibrium solver
def cell_forces(x):
    u, th = x[:3], x[3:]; disp = u[None] + np.cross(th[None], R)    # R in mm, u in mm, th in rad -> mm
    dn = np.einsum("ij,ij->i", N, disp); d = D0 - dn                                                   # compression, mm
    act = d > 0; fn = np.where(act, KN * d * 1e-3, 0.0)                                                 # N
    dt = disp - dn[:, None] * N; fs = np.where(act[:, None], -KS[:, None] * dt * 1e-3, 0.0)
    mag = np.linalg.norm(fs, axis=1); cap = MUC * fn; sc = np.where(mag > cap, cap / np.maximum(mag, 1e-12), 1.0); fs = fs * sc[:, None]      # Coulomb cap
    return fn[:, None] * N + fs, fn, fs, act
def wrench_cells(x):
    f = cell_forces(x)[0]; F = f.sum(0); M = np.cross(R * 1e-3, f).sum(0); return np.r_[F, M]
def solve(ext, x0=None, iters=60):
    x = np.zeros(6) if x0 is None else x0.copy()
    for it in range(iters):
        res = wrench_cells(x) + ext; r0 = np.linalg.norm(res)
        if r0 < 1e-6: break
        J = np.zeros((6, 6)); eps = 1e-4
        for j in range(6):
            dx = np.zeros(6); dx[j] = eps; J[:, j] = (wrench_cells(x + dx) - wrench_cells(x - dx)) / (2 * eps)
        try: step = np.linalg.solve(J + 1e-6 * np.eye(6), -res)
        except np.linalg.LinAlgError: step = np.linalg.lstsq(J, -res, rcond=None)[0]
        lam = 1.0
        while lam > 1e-3 and np.linalg.norm(wrench_cells(x + lam * step) + ext) > r0 * (1 - 1e-4 * lam): lam *= 0.5
        x = x + lam * step
    res = np.linalg.norm(wrench_cells(x) + ext); return x, res

# ------------------------------------------------------------------ mass model (per arm, right-arm frame)
fore_m = float(json.load(open(os.path.join(HERE, "..", "stl", "distal", "verify_distal.json")))["forearm_hand_mass_g"]["total"])
fore_cgz = float(json.load(open(os.path.join(HERE, "..", "stl", "distal", "verify_distal.json")))["forearm_hand_mass_g"]["cg_z_mm"])
tip_z = float(json.load(open(os.path.join(HERE, "..", "stl", "distal", "verify_distal.json")))["torque_Nm"]["tip_z_mm"])
upper_m = hl.ARM_MASS_G - fore_m; PAY = 500.0
upper_cgz = float(getattr(hl, "UPPER_CG_MM", 150.0))     # rev 2026-10-15: mesh upper-arm CG from harness_loads (was a flat 150 mm)
Mfix, CGfix = hl.cg_fixed(); Mm1 = sum(hl.m1_mass_g.values()); cm1 = np.array([0.0, -20.0, -20.0])
def arm_points(s1, s2, e, payload=PAY):
    Ra = bs.fk((s1, s2, 0.0)); ez = Ra @ np.array([0, 0, 1.0]); fd = Ra @ np.array([-np.sin(np.radians(e)), 0, np.cos(np.radians(e))])
    elbow = ez * 230.0
    pts = [(Mm1, bs.Rz(-np.radians(s1)) @ cm1), (upper_m, ez * upper_cgz), (fore_m, elbow + fd * (fore_cgz - 230.0))]
    if payload > 0: pts.append((payload, elbow + fd * (tip_z - 230.0)))
    return pts, elbow + fd * (tip_z - 230.0)
def gravity_wrench(poseR, poseL, payR=PAY, payL=PAY, dyn=DYN):
    pts = [(Mfix, CGfix), (Mfix, mirror(CGfix))]
    pr, tipR = arm_points(*poseR, payR); pl, tipL = arm_points(*poseL, payL); pts += pr + [(m, mirror(p)) for m, p in pl]
    m = np.array([a for a, _ in pts]); p = np.array([b for _, b in pts]); W = m.sum() * 1e-3 * G * dyn
    F = np.array([0, 0, -W]); M = np.cross(p * 1e-3, np.outer(m * 1e-3 * G * dyn, [0, 0, -1])).sum(0)
    cg = (m[:, None] * p).sum(0) / m.sum()
    return np.r_[F, M], cg, m.sum() * 1e-3, pts

def summarise(x):
    f, fn, fs, act = cell_forces(x); out = {}
    p_cell = fn / AR * 1e3                                           # kPa
    for g in sorted(set(GR)):
        i = GR == g; A = AR[i].sum(); out[g] = dict(N=round(float(fn[i].sum()), 1), mean_kPa=round(float(fn[i].sum() / A * 1e3), 2),
                                                    peak_kPa=round(float(p_cell[i].max()), 2), contact_frac=round(float(act[i].mean()), 2))
        if (SK & i).any():
            ip = i & ~SK; out[g].update(mean_plate_kPa=round(float(fn[ip].sum() / AR[ip].sum() * 1e3), 2), peak_plate_kPa=round(float(p_cell[ip].max()), 2), skirt_load_share=round(float(fn[SK & i].sum() / max(fn[i].sum(), 1e-9)), 3))
    shear = np.linalg.norm(fs, axis=1); ratio = np.where(fn > 0.05, shear / np.maximum(fn, 1e-9), 0.0)
    return out, p_cell, ratio, fn
def top_front_rail(out, side=""):
    g = lambda *ks: sum(out[k + side]["N"] for k in ks) / sum(hl.area(k) for k in ks if k in PADS) * 1e3 if all((k + side) in out for k in ks) else 0
    return dict(top=g("F1_upper_trap", "F4_acromion"), front=g("F2_ant_delt", "F3_pec"))

if __name__ == "__main__":
    import rom_limits as rl
    res = dict(model=dict(cells=NC, E_foam_kPa=E_FOAM_KPA, preload_mm=PRELOAD_MM, mu_slip=MU, mu_rail=MU_RAIL, dyn=DYN,
                          pad_foam_t_mm={k: [round(float(min(T[GR == k])), 1), round(float(max(T[GR == k])), 1)] for k in PADS}))
    # ------------------------------------------------ case 2 validation vs the 2-D model (symmetric poses, both arms)
    poses2d = {"park (S1 45, S2 25)": (45, 25), "arm up (S2 0)": (45, 0), "forward horizontal (S1 15, S2 88)": (15, 88), "lateral horizontal (S1 90, S2 88)": (90, 88),
               "back-lateral (S1 140, S2 60)": (140, 60), "back lean (S1 45, S2 -30)": (45, -30)}
    val = {}; hl.RAIL_Z = (float(rail["z"][1]), float(rail["z"][0])); hl.RAIL_W = float(rail["y"][1] - rail["y"][0])
    for nm, (a, b) in poses2d.items():
        W, cg, mt, _ = gravity_wrench((a, b, 0.0), (a, b, 0.0)); ext = W
        # 2-D model has the arm straight (E 0) in its CG formula; use E = 0 here for the comparison
        x, r = solve(ext); out, pc, rat, fn = summarise(x); two = hl.solve((a, b), {}, dyn=DYN)
        tfr = top_front_rail(out)
        rail_peak = max(out["rail"]["peak_kPa"], out["rail_L"]["peak_kPa"])
        val[nm] = dict(sim_top=round(tfr["top"], 1), sim_front=round(tfr["front"], 1), sim_rail_peak=round(rail_peak, 1), sim_rail_mean=round(out["rail"]["mean_kPa"], 1),
                       two_d_top=two["p_top_kPa"], two_d_front=two["p_front_kPa"], two_d_rail_peak=two["p_rail_kPa"], residual_N=round(float(r), 6))
    res["validation_2d"] = val
    # ------------------------------------------------ case 1 gravity grid
    canon = {"park": (45, 25, 45), "arm up": (45, 0, 45), "fwd horiz, E0": (15, 88, 0), "fwd horiz, E90": (15, 88, 90), "lateral horiz": (90, 88, 0),
             "back-lateral": (140, 60, 0), "back lean": (45, -30, 45), "reach low fwd": (45, 80, 100), "side raise E45": (100, 60, 45), "overhead back": (30, -30, 0)}
    names = list(canon); worst = {}
    def evaluate(pR, pL, payR=PAY, payL=PAY):
        W, cg, mt, _ = gravity_wrench(pR, pL, payR, payL); x, r = solve(W)
        out, pc, rat, fn = summarise(x); return dict(out=out, pc=pc, rat=rat, fn=fn, cg=cg, mass=mt, res=r, x=x)
    rows = []; best = {}
    for a in names:
        for b in names:
            ev = evaluate(canon[a], canon[b]); o = ev["out"]
            row = dict(R=a, L=b, residual=round(float(ev["res"]), 4), cg_y_off_spine_mm=round(float(ev["cg"][1] - SP), 1), mass_kg=round(ev["mass"], 2))
            for g in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion", "rail", "cradle"):
                for sd in ("", "_L"):
                    row[g + sd] = (o[g + sd]["mean_kPa"], o[g + sd]["peak_kPa"])
            row["max_shear_over_N"] = round(float(ev["rat"].max()), 2); rows.append(row)
    def key_max(g): return max(rows, key=lambda r: max(r[g][1], r[g + "_L"][1]))
    res["canonical_pairs"] = len(rows)
    res["canonical_worst"] = {g: dict(pair=(lambda r: f"R {r['R']} / L {r['L']}")(key_max(g)), mean_peak_kPa=max((key_max(g)[g], key_max(g)[g + "_L"]), key=lambda t: t[1])) for g in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion", "rail", "cradle")}
    res["max_cradle_kPa_over_pairs"] = max(max(r["cradle"][1], r["cradle_L"][1]) for r in rows)
    res["max_shear_over_N_pairs"] = max(r["max_shear_over_N"] for r in rows)
    res["canonical_asym_example"] = [r for r in rows if r["R"] == "fwd horiz, E0" and r["L"] == "back-lateral"][0]
    # random pairs inside the soft box and the straight-arm keep-out (>= 60 mm)
    km = np.load(os.path.join(S, "keepout_map.npz")); K1, K2, KC = km["S1"], km["S2"], km["C"]
    def clr(s1, s2): return KC[np.abs(K1 - s1).argmin(), np.abs(K2 - s2).argmin()]
    rng = np.random.default_rng(7); samples = []
    def rpose():
        while True:
            q = (rng.uniform(*rl.SOFT["S1"]), rng.uniform(*rl.SOFT["S2"]), rng.uniform(*rl.SOFT["E"]))
            if clr(q[0], q[1]) >= 60: return q
    NR = int(os.environ.get("NRAND", 600)); worst_i = {}
    mx = dict(pad_mean=0, pad_peak=0, rail_peak=0, cradle=0, shear=0); lifted = 0
    agg = {g: 0.0 for g in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion", "rail", "cradle")}; arg = {}
    for i in range(NR):
        pR, pL = rpose(), rpose(); payR, payL = (PAY if rng.random() < 0.7 else 0.0), (PAY if rng.random() < 0.7 else 0.0)
        ev = evaluate(pR, pL, payR, payL)
        if ev["res"] > 1e-2: lifted += 1; continue
        o = ev["out"]
        for g in agg:
            v = max(o[g]["peak_kPa"], o[g + "_L"]["peak_kPa"])
            if v > agg[g]: agg[g] = v; arg[g] = (tuple(round(float(a), 0) for a in pR), tuple(round(float(a), 0) for a in pL), payR, payL)
        mx["shear"] = max(mx["shear"], float(ev["rat"].max()))
    res["random"] = dict(n=NR, nonconverged=lifted, worst_peak_kPa={g: round(v, 1) for g, v in agg.items()}, worst_pose={g: arg.get(g) for g in agg}, max_shear_over_N=round(mx["shear"], 2))
    # ------------------------------------------------ pose-pair store for the render (mu as set)
    def store(pR, pL):
        ev = evaluate(pR, pL); return dict(R=list(pR), L=list(pL), p=[round(float(v), 2) for v in ev["pc"]])
    res["render"] = dict(cells=[dict(r=[round(float(v), 1) for v in c["r"]], g=c["group"]) for c in cells],
                         lateral_overheadback=store(canon["lateral horiz"], canon["overhead back"]), back_lateral=store(canon["back-lateral"], canon["back-lateral"]),
                         park=store(canon["park"], canon["park"]))
    # ------------------------------------------------ case 3: blocked arm / actuator stall wrench on top of gravity (right arm blocked, left arm at park)
    TAU = {"rated (AK 8.3, XM 10.6)": (8.3, 8.3, 10.6), "peak (AK 24.8, XM 10.6)": (24.8, 24.8, 10.6)}
    poses3 = {"park": (45, 25, 45), "lateral horiz": (90, 88, 0), "back-lateral": (140, 60, 0), "fwd horiz E0": (15, 88, 0)}
    c3 = {}
    for tn, (t1, t2, t3) in TAU.items():
        worst = dict(F1_upper_trap=0, F2_ant_delt=0, F3_pec=0, F4_acromion=0, rail=0, cradle=0); wcase = {}; noeq = 0; tot = 0; shear = 0; mnet = 0
        for pn, q in poses3.items():
            s1, s2 = np.radians(q[0]), np.radians(q[1]); ax1 = np.array([0, 0, 1.0]); ax2 = bs.Rz(-s1) @ np.array([0, 1.0, 0]); ax3 = bs.fk((q[0], q[1], 0)) @ np.array([0, 0, 1.0])
            W0, _, _, _ = gravity_wrench(q, canon["park"])
            for a in (-1, 0, 1):
                for b in (-1, 0, 1):
                    for c in (-1, 0, 1):
                        if a == b == c == 0: continue
                        Mx = a * t1 * ax1 + b * t2 * ax2 + c * t3 * ax3
                        ext = W0.copy(); ext[3:] += Mx; x, r = solve(ext); tot += 1
                        if r > 0.5: noeq += 1; continue
                        out, pc, rat, fn = summarise(x); shear = max(shear, float(rat.max()))
                        for g in worst:
                            v = max(out[g]["peak_kPa"], out[g + "_L"]["peak_kPa"])
                            if v > worst[g]: worst[g] = v; wcase[g] = f"{pn} S1:{a:+d} S2:{b:+d} S3:{c:+d}"
        c3[tn] = dict(worst_peak_kPa={g: round(v, 1) for g, v in worst.items()}, worst_case=wcase, no_equilibrium=f"{noeq}/{tot}", max_shear_over_N=round(shear, 2))
    res["stall_cases"] = c3
    # ------------------------------------------------ torque-limit finder: largest stall-torque fraction lam keeping axilla cradle <= 4.3 kPa and every pad cell <= 44 kPa
    lams = np.linspace(0.05, 1.0, 20) if not os.environ.get("LAM_FINE") else np.round(np.arange(*[float(x) for x in os.environ["LAM_FINE"].split(",")]), 4)      # LAM_FINE=lo,hi,step refines the finder (rev f)
    first_bad = {"cradle_4.3": 1.01, "pad_44": 1.01}; whobad = {}
    for pn, q in poses3.items():
        ax1 = np.array([0, 0, 1.0]); ax2 = bs.Rz(-np.radians(q[0])) @ np.array([0, 1.0, 0]); ax3 = bs.fk((q[0], q[1], 0)) @ np.array([0, 0, 1.0]); W0, _, _, _ = gravity_wrench(q, canon["park"])
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                for c in (-1, 0, 1):
                    if a == b == c == 0: continue
                    for lam in lams:
                        ext = W0.copy(); ext[3:] += lam * (a * 24.8 * ax1 + b * 24.8 * ax2 + c * 10.6 * ax3); x, r = solve(ext)
                        if r > 0.5: break
                        out, pc, rat, fn = summarise(x); cr = max(out["cradle"]["peak_kPa"], out["cradle_L"]["peak_kPa"]); pd = max(out[g + sd]["peak_kPa"] for g in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion", "rail") for sd in ("", "_L"))
                        if cr > 4.3 and lam < first_bad["cradle_4.3"]: first_bad["cradle_4.3"] = float(lam); whobad["cradle_4.3"] = f"{pn} S1:{a:+d} S2:{b:+d} S3:{c:+d}"
                        if pd > 44 and lam < first_bad["pad_44"]: first_bad["pad_44"] = float(lam); whobad["pad_44"] = f"{pn} S1:{a:+d} S2:{b:+d} S3:{c:+d}"
                        if cr > 4.3 and pd > 44: break
    res["torque_limit_finder"] = dict(lam_first_exceed=first_bad, case=whobad, S1_S2_Nm_at_lam=({k: round(v * 24.8, 1) for k, v in first_bad.items()}), note="lam scales (S1 24.8, S2 24.8, S3 10.6 N m) together; first lam in 0.05 steps that exceeds the threshold")
    # ------------------------------------------------ case 4: E-stop / power loss, S2 free fall into the stop
    S2_STOP = 91.0; E_PAD_J = 0.6                      # TPU 95A S2+ pad capacity estimate (SPEC_safety: S1 pad ~0.9 J at 11 mm; S2+ pad is 7.5 mm)
    def pe_J(s1, s2, e, pay=PAY):
        pts, _ = arm_points(s1, s2, e, pay); return sum(m * 1e-3 * G * p[2] * 1e-3 for m, p in pts)
    def inertia(axis, s1, s2, e, pay=PAY):
        pts, _ = arm_points(s1, s2, e, pay); a = unit(axis); return sum(m * 1e-3 * (np.linalg.norm(p * 1e-3) ** 2 - (p * 1e-3 @ a) ** 2) for m, p in pts)
    fall = {}; kth_cache = {}
    def k_theta(axis, Emod):
        a = unit(axis); W0 = gravity_wrench((45, 25, 45), (45, 25, 45))[0]; x0, _ = solve(W0)
        M0 = 0.2; ext = W0.copy(); ext[3:] += a * M0; x1, r1 = solve(ext, x0); dth = (x1[3:] - x0[3:]) @ a
        return M0 / max(dth, 1e-9)
    rows4 = []
    for s1 in (10, 45, 90, 140):
        for s2s in (-30, 0, 30, 60):
            for e in (0, 90, 145):
                dE = pe_J(s1, s2s, e) - pe_J(s1, S2_STOP, e)
                if dE <= 0: continue
                ax2 = bs.Rz(-np.radians(s1)) @ np.array([0, 1.0, 0]); Ia = inertia(ax2, s1, S2_STOP, e)
                # rig inertia about the same axis through P (fixed masses of both sides + the other arm parked)
                Ir = 0.0
                for pos in (CGfix, mirror(CGfix)): Ir += Mfix * 1e-3 * (np.linalg.norm(pos * 1e-3) ** 2 - (pos * 1e-3 @ ax2) ** 2)
                pk, _ = arm_points(45, 25, 45)
                Ir += sum(m * 1e-3 * (np.linalg.norm(mirror(p) * 1e-3) ** 2 - (mirror(p) * 1e-3 @ ax2) ** 2) for m, p in pk)
                Er = max(dE - E_PAD_J, 0.0); wa = np.sqrt(2 * Er / Ia) if Er > 0 else 0.0
                Lm = Ia * wa; KEc = Lm ** 2 / (2 * (Ia + Ir)); kt = k_theta(ax2, E_FOAM_KPA)
                Mpk = np.sqrt(2 * KEc * kt) if KEc > 0 else 0.0
                rows4.append(dict(S1=s1, S2_start=s2s, E=e, dE_J=round(dE, 2), I_arm=round(Ia, 3), I_rig=round(Ir, 3), omega_hit=round(wa, 2), KE_to_body_J=round(KEc, 3), k_theta_Nm_per_rad=round(kt, 0), M_peak_Nm=round(Mpk, 1)))
    res["freefall_rows"] = rows4
    worst4 = max(rows4, key=lambda r: r["M_peak_Nm"]); res["freefall_worst"] = worst4
    # static pad response to the worst impulse moment (about the S2 axis of the right arm) on top of gravity, and for 1/2 and 1/4 of it (sensitivity)
    ax2 = bs.Rz(-np.radians(worst4["S1"])) @ np.array([0, 1.0, 0]); q4 = (worst4["S1"], worst4["S2_start"], worst4["E"])
    W0 = gravity_wrench((worst4["S1"], S2_STOP, worst4["E"]), canon["park"])[0]; resp = {}
    for fr in (1.0, 0.5, 0.25):
        best = None
        for sgn in (-1, 1):
            ext = W0.copy(); ext[3:] += sgn * fr * worst4["M_peak_Nm"] * ax2; x, r = solve(ext)
            if r > 0.5: best = dict(frac=fr, no_equilibrium=True); continue
            out, pc, rat, fn = summarise(x); pk = {g: round(max(out[g]["peak_kPa"], out[g + "_L"]["peak_kPa"]), 0) for g in ("F1_upper_trap", "F2_ant_delt", "F3_pec", "F4_acromion", "rail")}
            if best is None or best.get("no_equilibrium") or max(pk.values()) > max(best["peak_kPa"].values()): best = dict(frac=fr, peak_kPa=pk, M_Nm=round(fr * worst4["M_peak_Nm"], 1), max_shear_over_N=round(float(rat.max()), 2))
        resp[str(fr)] = best
    res["freefall_pad_response"] = resp
    # head sweep: straight-arm capsule clearance along the fall S2_start -> 91 at fixed S1 (keepout_map.npz; the map is the worst case, E = 0)
    sweep = []
    for s1 in np.arange(10, 141, 10):
        for s2s in (-30, 0, 30, 60):
            path = [clr(s1, v) for v in np.arange(s2s, S2_STOP + 0.1, 2.5)]; sweep.append(dict(S1=float(s1), S2_start=s2s, min_clear_mm=round(float(min(path)), 1)))
    res["freefall_head_sweep"] = dict(min_over_all=min(r["min_clear_mm"] for r in sweep), rows_below_60=[r for r in sweep if r["min_clear_mm"] < 60][:12], n_below_60=sum(r["min_clear_mm"] < 60 for r in sweep), n_below_0=sum(r["min_clear_mm"] < 0 for r in sweep), n=len(sweep))
    # ------------------------------------------------ case 5: trunk loads and nerve-site distances
    Hh = lm["height_mm"]; z_floor = lm["acromion_top_rel_P"][2] - 0.818 * Hh; L5 = np.array([-165.0, SP, z_floor + 0.55 * Hh]); C7 = np.array([atl["C7"][0], SP, atl["C7"][2]])
    tr = dict(L5_estimate=[round(float(v), 0) for v in L5], C7=[round(float(v), 0) for v in C7]); worst_l5 = (0, None); worst_c7 = (0, None); worst_lat = (0, None); mass_rig = None
    rng2 = np.random.default_rng(3)
    wsag = {"L5": (0, None), "C7": (0, None)}; wlat = {"L5": (0, None), "C7": (0, None)}
    for i in range(400):
        pR, pL = rpose(), rpose(); W, cg, mt, pts = gravity_wrench(pR, pL, dyn=1.0); mass_rig = mt
        for nm, O in (("L5", L5), ("C7", C7)):
            Mo = np.cross(cg - O, [0, 0, -mt * G]) * 1e-3; sag, lat = abs(Mo[1]), abs(Mo[0])
            if sag > wsag[nm][0]: wsag[nm] = (sag, (round(float(sag), 1), [round(float(a)) for a in pR], [round(float(a)) for a in pL]))
            if lat > wlat[nm][0]: wlat[nm] = (lat, (round(float(lat), 1), [round(float(a)) for a in pR], [round(float(a)) for a in pL]))
    worst_l5 = dict(sagittal=wsag["L5"][1], lateral=wlat["L5"][1]); worst_c7 = dict(sagittal=wsag["C7"][1], lateral=wlat["C7"][1])
    W1, cg1, mt1, _ = gravity_wrench(canon["park"], canon["park"], dyn=1.0)
    tr.update(total_mass_kg=round(mt1, 2), axial_N=round(mt1 * G, 0), cg_park=[round(float(v), 0) for v in cg1],
              moment_L5_park_Nm=[round(float(v) * 1e-3, 1) for v in np.cross(cg1 - L5, [0, 0, -mt1 * G])][:2], worst_L5=worst_l5, worst_C7=worst_c7, erector_force_est_N=round(wsag['L5'][0] / 0.05 + mt1 * G, 0),
              pct_body_weight={f"{bw} kg": round(100 * mt1 / bw, 1) for bw in (60, 70, 80, 90)})
    res["trunk"] = tr
    # nerve and bone sites (estimated landmarks): clavicle segment, plexus proxy, axilla apex, spinous line
    NOTCH, ACR = np.array(atl["NOTCH"]), np.array(atl["ACR_top"]); cl = ACR - NOTCH
    def seg_dist(P): t = np.clip(((P - NOTCH) @ cl) / (cl @ cl), 0, 1); return np.linalg.norm(P - (NOTCH + t[:, None] * cl), axis=1), t
    pc_ref = evaluate(canon["lateral horiz"], canon["overhead back"])["pc"]; near = {}
    for k in PADS:
        i = np.array([j for j in range(nR) if cells[j]["group"] == k]); d, t = seg_dist(R[i])
        near[k] = dict(min_dist_to_clavicle_mm=round(float(d.min()), 0), frac_along_from_notch=round(float(t[d.argmin()]), 2), peak_kPa_within_25mm=round(float(pc_ref[i][d < 25].max()) if (d < 25).any() else 0.0, 1))
    pp = NOTCH + 0.65 * cl
    for k in PADS:
        i = np.array([j for j in range(nR) if cells[j]["group"] == k]); near[k]["min_dist_to_plexus_proxy_mm"] = round(float(np.linalg.norm(R[i] - pp, axis=1).min()), 0)
    res["nerve_sites"] = dict(clavicle_pads=near, plexus_proxy_point=[round(float(v), 0) for v in pp],
        note="NOTCH is an ESTIMATED landmark (atlas); the plexus proxy is the clavicle point at 65 % from the notch, tolerance about +-20 mm")
    med = [j for j in range(nR) if cells[j]["group"] == "rail" and YREL[j] <= rail["y"][0] + 11.0]
    w17 = [j for j in range(nR) if cells[j]["group"] == "rail" and YREL[j] <= 17.0]
    res["spinous"] = dict(rail_medial_edge_mm_from_midline=rail["y"][0], medial_column_peak_kPa=round(float(pc_ref[med].max()), 1), within17mm_peak_kPa=round(float(pc_ref[w17].max()), 1) if w17 else 0.0, rail_cfg=CFG, note="spinous-process tip width ~5-10 mm; target 4.3 kPa on nerve/bony sites")
    # strap line pressure: p = T kappa / w along the strap path (capstan line load).  rev 2026-10-14: webbing width and tension mark come from strap_axilla.json
    # (32 mm, 10 N); the legacy polyline-curvature number is kept for comparability, plus the skin-normal curvature of the on-skin centre line (chassis_axilla.py).
    sj = json.load(open(os.path.join(S, "strap_axilla.json"))); W_MM = float(sj.get("strap_w_mm", 20.0)); T_N = float(sj.get("tension_spec_N", 12.0))
    st = np.array(sj["strap_front"]); seg = np.diff(st, axis=0); L = np.linalg.norm(seg, axis=1); tg = seg / L[:, None]
    kap = [np.arccos(np.clip(tg[i] @ tg[i + 1], -1, 1)) / (0.5 * (L[i] + L[i + 1])) for i in range(len(tg) - 1)]
    pl = lambda T_, w_: dict(max=round(T_ * max(kap) * 1e3 / (w_ * 1e-3) * 1e-3, 1), mean=round(T_ * float(np.mean(kap)) * 1e3 / (w_ * 1e-3) * 1e-3, 1))
    res["strap"] = dict(path_points=len(st), length_mm=round(float(L.sum()), 0), curvature_per_m_max=round(float(max(kap)) * 1e3, 1), strap_w_mm=W_MM, tension_N=T_N,
                        line_pressure_kPa_max=pl(T_N, W_MM)["max"], line_pressure_kPa_mean=pl(T_N, W_MM)["mean"],
                        legacy_12N_20mm=pl(12.0, 20.0), legacy_12N_at_this_width=pl(12.0, W_MM), legacy_at_tension_mark_20mm=pl(T_N, 20.0),
                        skin_normal_method=sj.get("clearances", {}).get("line_pressure_normal_kPa"), clearances=sj.get("clearances"),
                        note="p = T kappa / w; legacy = 7-point routed polyline, 3-D turning angle (upper bound); skin_normal_method = curvature toward the body of the 2 mm-resampled skin-following centre line, "
                             "width_averaged = averaged over one strap width (webbing bridges narrower features)")
    # ------------------------------------------------ keep for stage 2
    json.dump(res, open(os.path.join(S, "body_loads_mu%g%s%s.json" % (MU, "" if E_FOAM_KPA == 200.0 else "_E%g" % E_FOAM_KPA, os.environ.get("TAG", ""))), "w"), indent=1, default=lambda o: float(o) if isinstance(o, (np.floating,)) else str(o))
    print(json.dumps(res, indent=1, default=lambda o: float(o) if isinstance(o, (np.floating,)) else str(o)))
