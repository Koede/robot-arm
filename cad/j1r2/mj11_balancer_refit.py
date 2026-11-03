"""MJ11 balancer refit + packaging (2026-11-03, SHOULDER). Option (a) of the torque-gate decision.
1. Torque: the pad-anchored zero-length-type spring (cord over sheave A fixed on the pad, 2:1 tackle, tube) is translation invariant, so the rev 2 design (A_OFF, B_LOC, k, L0)
   keeps its residual at the MJ11 pad (checked by re-evaluating with D0 = derive(-60,-190,12)). New: re-fit A, B, k, L0 to the MJ11 passive box only (smaller spring / lower force).
2. Packaging on the lower back: tube direction scan (angle psi in the yz plane about A) with the real wearer mesh, MJ11 pad/dish/liner/spacers/clasp bracket, rail + spine bridge,
   the S1'/S2 stage hard sweep, the mirrored left tube, and the wearer's own arm range (mj_eval.wearer_rom).
usage: J1R2_ROOT=/home/claude/ts python3 mj11_balancer_refit.py -> stl/j1r2_bal/mj11_balancer_refit.json"""
import os, sys, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from scipy.optimize import differential_evolution
from scipy.spatial import cKDTree
import j1r2_params as P, j1r2_bal_model as BM, clib as L
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))); OUT = os.path.join(REPO, "stl", "j1r2_bal"); os.makedirs(OUT, exist_ok=True)
t0 = time.time(); log = lambda *a: print("[%5.0fs]" % (time.time() - t0), *a, flush=True)
RATED = 8.3
DER = P.derive(-60.0, -190.0, 12.0); BM.D0 = DER; BM.P1 = np.array([DER["PX"], DER["y1"], DER["z1"]]); P1 = BM.P1
A_OFF0 = np.array([-10.018, -19.708, 73.434]); B0 = np.array([-15.629, -42.256, 118.385]); K0, L00 = 0.7520, 54.587          # rev 2 design
ARM_SIDE_KG = 0.052
BOX_TH = np.arange(-95.0, 5.1, 5.0); BOX_PH = np.array([-15, -9, -3, 3, 9, 15, 17.4])
import mj11_torque_options as TQ
poses = TQ.grid(TQ.BOX_TH, TQ.BOX_PH, pays=(0.0, 0.5)); T1, T2 = TQ.torques(poses, extra_arm=[(ARM_SIDE_KG, tuple(B0))])      # N.m, with the balancer arm-side mass at B
pairs = sorted({(p[0], p[1]) for p in poses}); pidx = {pr: i for i, pr in enumerate(pairs)}; ii = np.array([pidx[(p[0], p[1])] for p in poses])
g1lo = np.full(len(pairs), 1e9); g1hi = np.full(len(pairs), -1e9); g2lo = g1lo.copy(); g2hi = g1hi.copy()
for k, i in enumerate(ii):
    g1lo[i] = min(g1lo[i], T1[k]); g1hi[i] = max(g1hi[i], T1[k]); g2lo[i] = min(g2lo[i], T2[k]); g2hi[i] = max(g2hi[i], T2[k])
FA = np.array([P.shoulder_frames(DER, th, ph)[1] for th, ph in pairs]); S = np.array([P.shoulder_frames(DER, th, ph)[2] for th, ph in pairs])
def spring_torque(Aoff, Bl, k, l0):
    B = np.einsum("nij,j->ni", FA[:, :3, :3], Bl) + FA[:, :3, 3]; v = (P1 + Aoff)[None] - B; d = np.linalg.norm(v, axis=1); F = (k * (d - l0))[:, None] * v / d[:, None]
    mom = np.cross((B - P1) * 1e-3, F); return mom[:, 0], np.einsum("ni,ni->n", mom, S), d
def residual(Aoff, Bl, k, l0):
    s1, s2, d = spring_torque(Aoff, Bl, k, l0); r1 = np.maximum(abs(g1lo + s1), abs(g1hi + s1)); r2 = np.maximum(abs(g2lo + s2), abs(g2hi + s2)); return r1, r2, d
# --- 1. invariance check: rev 2 design at the MJ11 pad, box grid and full grid
r1, r2, d = residual(A_OFF0, B0, K0, L00)
res = dict(rev2_design_in_box=dict(S1p=round(float(r1.max()), 2), S2=round(float(r2.max()), 2), SF_S1p=round(RATED / float(r1.max()), 2), SF_S2=round(RATED / float(r2.max()), 2), cord_len=[round(float(d.min()), 1), round(float(d.max()), 1)],
           F_cord_max_N=round(float(K0 * (d.max() - L00)), 1)))
log("rev2 design in the MJ11 box:", res["rev2_design_in_box"])
# --- 2. refit to the box (A radius 70..110 from the S1' axis, sector widened to 40..250 deg because the S2 motor now only sweeps -95..5 deg; cord <= 200; cord force <= 130 N)
def cost(x, sector=(40.0, 250.0), ret=False):
    Aoff = np.array([x[0], x[1], x[2]]); Bl = np.array(x[3:6]); k, l0 = x[6], x[7]; r1, r2, d = residual(Aoff, Bl, k, l0)
    rr = math.hypot(x[1], x[2]); ang = math.degrees(math.atan2(x[2], x[1])) % 360.0; rb = math.hypot(x[3], x[4]); pen = 0.0
    pen += max(0, sector[0] - ang) + max(0, ang - sector[1]) + 0.5 * max(0, rr - 110) + 0.5 * max(0, 70 - rr)
    pen += 0.05 * max(0, d.max() - 200) + 0.05 * max(0, (d.max() - d.min()) - 140) + 0.5 * max(0, l0 - d.min() + 2.0) + 0.05 * max(0, k * (d.max() - l0) - 130.0)
    pen += 0.3 * max(0, rb - 52) + 0.3 * max(0, 38 - rb) + 0.1 * max(0, 100 - x[5]) + 0.1 * max(0, x[5] - 160)
    if ret: return r1, r2, d, pen
    return max(r1.max(), 0.7 * r2.max()) + pen
bnds = [(-40, 40), (-110, 110), (-110, 110), (-52, 52), (-52, 52), (95, 165), (0.2, 2.0), (20, 120)]
fits = {}
for name, sector in (("same_sector_105_230", (105.0, 230.0)), ("wide_sector_40_250", (40.0, 250.0))):
    best = None
    for seed in (1, 2, 3):
        r = differential_evolution(cost, bnds, args=(sector,), seed=seed, maxiter=400, popsize=24, tol=1e-7, polish=True, init="sobol")
        if best is None or r.fun < best.fun: best = r
    x = best.x; r1, r2, d, pen = cost(x, sector, True)
    fits[name] = dict(x=x.tolist(), A_off=x[:3].round(2).tolist(), A_radius=round(math.hypot(x[1], x[2]), 1), A_angle_deg=round(math.degrees(math.atan2(x[2], x[1])) % 360, 1), B_loc=x[3:6].round(2).tolist(), k_cord=round(float(x[6]), 4), L0=round(float(x[7]), 2),
                      S1p=round(float(r1.max()), 2), S2=round(float(r2.max()), 2), SF_S1p=round(RATED / float(r1.max()), 2), SF_S2=round(RATED / float(r2.max()), 2), penalty=round(float(pen), 4),
                      cord_len=[round(float(d.min()), 1), round(float(d.max()), 1)], F_cord_max_N=round(float(x[6] * (d.max() - x[7])), 1), F_spring_2to1_N=round(float(2 * x[6] * (d.max() - x[7])), 1))
    log(name, {k: v for k, v in fits[name].items() if k != "x"})
res["refit"] = fits
# --- 3. spring sizing (A228, same method as j1r2_balancer): tube length by tackle ratio for the chosen design
def spring_opts(kc, d_max, d_min, l0, Gm=80000.0):
    out = []
    for n in (2, 3, 4):
        ks = n ** 2 * kc; xm = max((d_max - l0) / n, 0.0); Fm = n * kc * (d_max - l0); ok = None
        for dwi in np.arange(2.5, 7.01, 0.25):
            for Dm in (30.0, 34.0, 36.0, 40.0, 44.0):
                C = Dm / dwi
                if not (5 <= C <= 12): continue
                KB = (4 * C + 2) / (4 * C - 3); tau = KB * 8 * Fm * Dm / (math.pi * dwi ** 3); Sut = 2211.0 / dwi ** 0.145
                if tau <= 0.31 * Sut:
                    Na = Gm * dwi ** 4 / (8 * Dm ** 3 * ks); Ls = (Na + 2) * dwi; Lf = Ls + 2.12 * xm; mass = math.pi * Dm * (Na + 2) * math.pi / 4 * dwi ** 2 * 7.85e-3
                    ok = dict(tackle=n, k_spring=round(ks, 2), F_spring_max_N=round(Fm, 0), stroke_mm=round(xm, 1), wire_d=float(dwi), D_mean=Dm, tau_over_Sut=round(tau / Sut, 3), L_free=round(Lf, 0), tube_len=round(Lf + 8 + 10 + 6 + 8, 0), OD_tube=round(Dm + dwi + 10, 1), spring_g=round(mass, 0)); break
            if ok: break
        out.append(ok)
    return out
for name, f in list(fits.items()) + [("rev2", dict(k_cord=K0, L0=L00, cord_len=res["rev2_design_in_box"]["cord_len"]))]:
    f["spring_options"] = spring_opts(f["k_cord"], f["cord_len"][1], f["cord_len"][0], f["L0"])
log("spring options done")
json.dump(res, open(os.path.join(OUT, "mj11_balancer_refit.json"), "w"), indent=1, default=float)
if os.environ.get("SKIP_PACK"): sys.exit(0)
# --- 4. packaging scan on the real MJ11 geometry
import mj_eval as E, mj_concepts as C
import mj_frame as M
parts11 = C.CONCEPTS["MJ11"]["geo"](E.BASE); mount_trees = {k: cKDTree(E.samp(p["mesh"], 2.0)) for k, p in parts11.items() if p["kind"] != "cord" and k != "liner"}
def tube_parts(A, psi, Ltube, Dt=44.5):
    a = math.radians(psi); u = np.array([0.0, math.cos(a), math.sin(a)]); cap = C.tubebeam(A - u * 8.0, A, 56.0, None); tube = C.tubebeam(A + u * 6.0, A + u * (6.0 + Ltube), Dt, 2.5)
    post = C.tubebeam(np.array([DER["XB"], A[1], A[2]]), A - np.array([14.0, 0, 0]), 26.0, None)
    return u, {"bal_tube": dict(mesh=tube, mat="PETGCF", kind="printed"), "bal_cap": dict(mesh=cap, mat="PETGCF", kind="printed"), "bal_post": dict(mesh=post, mat="PETGCF", kind="printed")}
def mirror_pts(pts): q = pts.copy(); q[:, 1] = 2 * M.SPINE_Y - q[:, 1]; return q
HARD_TH = np.arange(-100.0, 10.1, 5.0); HARD_PH = (-20.0, 0.0, 17.4, 22.4)        # MJ11 passive box + 5 deg stop margin
def box_clear(prs):
    """min distance of the pieces to the S1' group (all th1 of the box) and to the arm group (th1 x S2 of the box + margin), no designed-proximity exclusion (the anchor hardware must clear)."""
    pts = np.vstack([E.samp(p["mesh"], 2.0) for p in prs.values()]); T = cKDTree(pts); b1 = b2 = 1e9
    for th in HARD_TH:
        Mx = E.G.pose_matrix("M1", E.DER, th, 0.0); b1 = min(b1, float(T.query(E.MOV1 @ Mx[:3, :3].T + Mx[:3, 3], distance_upper_bound=40)[0].min()))
        for ph in HARD_PH:
            Mm = E.G.pose_matrix("M2", E.DER, th, ph); b2 = min(b2, float(T.query(E.MOV2 @ Mm[:3, :3].T + Mm[:3, 3], distance_upper_bound=40)[0].min()))
    return round(min(b1, 40.0), 1), round(min(b2, 40.0), 1)
def pack_eval(fit_name, f, tackle_len, psi):
    A = P1 + np.array(f["A_off"]); u, prs = tube_parts(A, psi, tackle_len); tube = {"bal_tube": prs["bal_tube"]}
    sk = E.skin_zones(prs); skin_min = min(min(v.values()) for v in sk.values() if v)
    m1, m2 = box_clear(tube); hc = E.harness_clear(prs); head = E.head_clear([p["mesh"] for p in prs.values()])
    pts = np.vstack([E.samp(p["mesh"], 2.0) for p in prs.values()]); mt = min(float(T.query(pts, distance_upper_bound=40)[0].min()) for T in mount_trees.values())
    lr = float(cKDTree(pts).query(mirror_pts(pts), distance_upper_bound=80)[0].min()); rom = {k: v for k, v in E.wearer_rom(prs).items() if v is not None}
    tip = A + u * (6 + tackle_len)
    return dict(psi=psi, tube_len=tackle_len, skin_min=skin_min, tube_vs_S1p_group=m1, tube_vs_arm=m2, rail_min=min(v["wing_rail"] for v in hc.values()), bridge_min=min(v["spine_bridge"] for v in hc.values()),
                mount_parts_min=round(mt, 1), left_right_min=round(lr, 1), head_mm=head, wearer_rom_contacts=rom, tube_end_yz=[round(float(tip[1]), 0), round(float(tip[2]), 0)])
pack = {}; A_rev2 = {"A_off": A_OFF0.tolist()}
_, anchor = tube_parts(P1 + A_OFF0, 90.0, 10.0); anchor.pop("bal_tube"); res["anchor_hardware_vs_stage"] = {}
for nm, prs_ in anchor.items(): res["anchor_hardware_vs_stage"][nm] = dict(zip(("vs_S1p_group_mm", "vs_arm_mm"), box_clear({nm: prs_})))
log("anchor hardware vs stage:", res["anchor_hardware_vs_stage"])
for Lt, tag in ((230.0, "2:1"), (175.0, "3:1")):
    rows = []
    for psi in range(0, 360, 30):
        r = pack_eval("rev2_A", A_rev2, Lt, psi); rows.append(r); log("rev2_A", tag, psi, {k: r[k] for k in ("skin_min", "tube_vs_S1p_group", "tube_vs_arm", "rail_min", "bridge_min", "mount_parts_min", "left_right_min", "head_mm")}, r["wearer_rom_contacts"])
    pack[f"rev2_A {tag}"] = rows
res["packaging_scan"] = pack
json.dump(res, open(os.path.join(OUT, "mj11_balancer_refit.json"), "w"), indent=1, default=float)
