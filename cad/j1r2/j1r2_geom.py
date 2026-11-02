"""J1 rev 2: pancake / side-hung shoulder, GEOMETRY + CONNECTIONS (proxy, robot frame, mm).  Parametric:  build(y1, z1, standoff, ...) -> parts dict.
Every part is built at the REFERENCE POSE th1 = 0, phi = 0 in WORLD coordinates (there the yoke frame F0 is a pure translation to P1 = (PX, y1, z1), s = +y, a = +z, arm along +z).
Group tags:  F  fixed to the wing           M1  turns with S1' (rotation about +x through (y1, z1))
             M2 turns with S1' and S2 (S2 axis = +y through P1 at the reference pose)   ARM  upper arm tube + elbow ball (M2 kinematically)   bought  = purchased (also carries its group in `grp`)
pose(parts, th1, phi): world = Rx(th1 about P1) . Ry(-phi about P1) . geometry  (M2/ARM);  Rx(th1 about P1) . geometry (M1);  identity (F).  Verified equal to P.shoulder_frames.
NOTE: cad/joint_j1.py offset the whole arm side by an extra T_ARM (it built in FA = P1 + T_ARM s with local y = T_ARM); fixed here (arm axis is T_ARM along s from P1)."""
import sys, os, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, shapely.geometry as sg
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter
import clib as L, j1r2_params as P
from clib import cyl, box, U, I, prism, hull, tr, rot, COL
DIFF = L.D
OUT = os.path.join(L.ROOT, "stl", "j1r2"); os.makedirs(OUT, exist_ok=True)
AKR, AKL, TS, T_ARM = P.AKR, P.AKL, P.TS, P.T_ARM
Rx = lambda t: rot(np.radians(t), [1, 0, 0]); Ry = lambda t: rot(np.radians(t), [0, 1, 0])
BLUE = (0.20, 0.34, 0.52)
def _p(mesh, mat, grp, colour, frame="world", mass_g=None, fill=1.0, kind="printed"):
    d = dict(mesh=mesh, mat=mat, group=grp, colour=colour, frame=frame, fill=fill, kind=kind)
    if mass_g is not None: d["mass_g"] = mass_g
    return d
def seg_tube(pts, r, n=16):
    """swept tube proxy (concatenated cylinders + joint spheres) along a polyline."""
    ms = []
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float); h = np.linalg.norm(b - a)
        if h < 1e-6: continue
        c = trimesh.creation.cylinder(r, h, sections=n); z = np.array([0, 0, 1.0]); v = (b - a) / h
        ax = np.cross(z, v); sn = np.linalg.norm(ax)
        M = np.eye(4)
        if sn > 1e-9: M = rot(math.atan2(sn, float(z @ v)), ax / sn)
        M[:3, 3] = (a + b) / 2; c.apply_transform(M); ms.append(c)
    for q in pts[1:-1]: s = trimesh.creation.icosphere(1, r); s.apply_translation(q); ms.append(s)
    return trimesh.util.concatenate(ms)
def tube_solid(pts, r, n=16):
    """watertight union of cylinders + joint spheres (for boolean cuts)."""
    ms = []
    for a, b in zip(pts[:-1], pts[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float); h = np.linalg.norm(b - a)
        if h < 1e-6: continue
        c = trimesh.creation.cylinder(r, h, sections=n); z = np.array([0, 0, 1.0]); v = (b - a) / h; ax = np.cross(z, v); sn = np.linalg.norm(ax); M = np.eye(4)
        if sn > 1e-9: M = rot(math.atan2(sn, float(z @ v)), ax / sn)
        M[:3, 3] = (a + b) / 2; c.apply_transform(M); ms.append(c)
    for q in pts[:]:
        sp_ = trimesh.creation.icosphere(1, r); sp_.apply_translation(q); ms.append(sp_)
    return U(*ms)
def mincirc(pts):
    """min circumradius (mm) of consecutive point triples of a polyline (bend radius proxy)."""
    pts = np.asarray(pts, float); rmin = 1e9
    for a, b, c in zip(pts[:-2], pts[1:-1], pts[2:]):
        ab, bc, ca = np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c); ar = np.linalg.norm(np.cross(b - a, c - a))
        if ar > 1e-9: rmin = min(rmin, ab * bc * ca / (2 * ar))
    return rmin
def path_clearance(path, mesh, r, end_skip=6.0, n_surf=20000):
    """min (signed distance of the centreline to the part surface) - r, over the densified path (mm); negative = the tube penetrates. the last end_skip mm are excluded (designed terminations)."""
    p = np.asarray(path, float); seg = np.linalg.norm(np.diff(p, axis=0), axis=1); sarr = np.r_[0, np.cumsum(seg)]; tot = sarr[-1]
    t = np.arange(end_skip, tot - end_skip, 1.5); q = np.c_[[np.interp(t, sarr, p[:, i]) for i in range(3)]].T
    if np.any(mesh.bounds[1] + r + 1 < q.min(0)) or np.any(mesh.bounds[0] - r - 1 > q.max(0)): return 999.0
    sp = trimesh.sample.sample_surface(mesh, n_surf)[0]; d = cKDTree(sp).query(q)[0]
    inside = mesh.contains(q) if mesh.is_watertight else np.zeros(len(q), bool); sd = np.where(inside, -d, d); return float(sd.min() - r)
def mindist(A, B, n=5000):
    """approx min distance (mm) between two surfaces (sampled, +-~0.8 mm); 0 if their bounding boxes overlap and a sample falls inside the other."""
    a = trimesh.sample.sample_surface(A, n)[0]; b = trimesh.sample.sample_surface(B, n)[0]
    return float(cKDTree(b).query(a)[0].min())
def exact_min(A, B, n=20000):
    """min distance (mm) from the surface of A to the watertight mesh B (point samples of A, exact point-to-triangle distance)."""
    pts = trimesh.sample.sample_surface(A, n)[0]; _, d, _ = trimesh.proximity.closest_point(B, pts); return float(d.min())
def inter_vol(A, B):
    if np.any(A.bounds[1] < B.bounds[0]) or np.any(B.bounds[1] < A.bounds[0]): return 0.0
    try: return float(abs(I(A, B).volume))
    except Exception: return float("nan")

def fillet_path(pts, R, n_arc=10):
    """replace every interior corner of a 3D polyline by a circular arc of radius R (clamped to half of the shorter adjacent segment); returns (points, min radius used)."""
    P_ = [np.asarray(p, float) for p in pts]; out = [P_[0]]; rused = 1e9
    for i in range(1, len(P_) - 1):
        A, B, C = P_[i - 1], P_[i], P_[i + 1]; d1 = (B - A) / np.linalg.norm(B - A); d2 = (C - B) / np.linalg.norm(C - B); cs = float(np.clip(d1 @ d2, -1, 1)); phi = math.acos(cs)
        if phi < 1e-3: out.append(B); continue
        la, lb = np.linalg.norm(B - A), np.linalg.norm(C - B); Ri = R; t = Ri * math.tan(phi / 2); lim = 0.5 * min(la, lb) if 1 < i < len(P_) - 2 else min(la, lb) if i in (1, len(P_) - 2) and len(P_) == 3 else 0.5 * min(la, lb)
        if i == 1: lim = min(la - 0.0, 0.5 * lb if len(P_) > 3 else lb)
        if i == len(P_) - 2: lim = min(0.5 * la if len(P_) > 3 else la, lb)
        if t > lim: t = lim; Ri = t / math.tan(phi / 2)
        rused = min(rused, Ri); P1 = B - d1 * t; n = d2 - d1; n /= np.linalg.norm(n); Cc = B + n * Ri / math.cos(phi / 2); ax = np.cross(d1, d2); ax /= np.linalg.norm(ax)
        v0 = P1 - Cc
        for k in range(n_arc + 1):
            ang = phi * k / n_arc; v = v0 * math.cos(ang) + np.cross(ax, v0) * math.sin(ang) + ax * (ax @ v0) * (1 - math.cos(ang)); out.append(Cc + v)
    out.append(P_[-1]); return np.array(out), (rused if rused < 1e8 else float("inf"))
def wing_proxy(front_x, z_top, thick=3.5):
    """wing = same blend as clib.env_parts() (rail edge -> tongue) but the tongue plane is at front_x (x of the body-side face) and the top edge is clipped at z_top."""
    Y, Z, Xs = L.rear_surface(-106, -44, -165, z_top, 4.0, 6.0)
    s_ = np.clip((Y + 104.0) / 26.0, 0, 1); s_ = s_ * s_ * (3 - 2 * s_)
    xb = (Xs + 10.0) * (1 - s_) + front_x * s_; xb = np.maximum(xb, np.where(Y > -96, Xs + 20.0, -1e9)); xb = np.where(Y > -80, front_x, xb)
    xb = gaussian_filter(xb, 1.2, mode="nearest"); xb = np.where(Y > -80, front_x, xb)
    return L.height_solid(Y, Z, xb, xb + thick, "x")
def _radial_cyl(y1, z1, psi, r_out, r_in, rad, xc):
    """cylinder of radius rad whose axis lies along the radial direction psi (deg from +y toward +z) in the plane x = xc, from r_in to r_out about (y1, z1)."""
    u = np.array([math.cos(math.radians(psi)), math.sin(math.radians(psi))]); v = np.array([0.0, u[0], u[1]]); ax = np.cross([0, 0, 1.0], v); sn = np.linalg.norm(ax)
    m = trimesh.creation.cylinder(rad, abs(r_out - r_in), sections=24); M = rot(math.atan2(sn, v[2]), ax / sn); M[:3, 3] = [xc, *(np.array([y1, z1]) + u * (r_out + r_in) / 2)]; m.apply_transform(M); return m
def _s_curve(Q, hQ, E0, h0, R, l0=0.0):
    """planar S-curve: arc1 (CW) + arc2 (CCW) of radius R from heading h0 to heading hQ, ending at Q with the start x-coordinate E0[0] fixed and the start height solved (E0[1] is returned in info).
    Returns (polyline (N,2), info)."""
    h0 = np.asarray(h0, float); hQ = np.asarray(hQ, float); a_h0 = math.atan2(h0[1], h0[0]); a_hQ = math.atan2(hQ[1], hQ[0]); turn = a_hQ - a_h0
    def arcpts(p, ang, da, n=18):
        sgn = 1.0 if da > 0 else -1.0; cen = p + R * np.array([-math.sin(ang), math.cos(ang)]) * sgn; out = []
        for k in range(1, n + 1):
            a = ang + da * k / n; out.append(cen + R * np.array([math.sin(a), -math.cos(a)]) * sgn)
        return out
    def walk(a1):
        pts = [np.zeros(2)]; o = arcpts(pts[0], a_h0, -a1); pts += o; ang = a_h0 - a1; o = arcpts(pts[-1], ang, turn + a1); pts += o; return np.array(pts)
    lo, hi = 0.01, 2.2; tgt = Q[0] - E0[0]
    for _ in range(60):
        mid = 0.5 * (lo + hi); y_end = walk(mid)[-1][0]
        if y_end < tgt: lo = mid
        else: hi = mid
    a1 = 0.5 * (lo + hi); w = walk(a1); off = np.asarray(Q, float) - w[-1]; pts = w + off
    return pts, dict(alpha1_deg=round(math.degrees(a1), 1), alpha2_deg=round(math.degrees(turn + a1), 1), start_xy=[round(float(pts[0][0]), 1), round(float(pts[0][1]), 1)], radius=round(R, 1))
def build(y1=10.0, z1=-30.0, standoff=22.0, fair_gap=3.2, wall=1.8, th_range=None, ext_y=(-146.0, -103.0), ext_z=(-93.0, -54.0), psi_col=165.0, lap_z=(-86.0, -68.0), pad_r=66.0, beam_d=9.5, psi_gland=190.0, full=False):
    """MOUNT = RAIL TOP LAP: pad + hollow box-beam riser (pad -> 4 standoffs -> 4 bosses on the top end of the paraspinal rail). rev 2b (2026-11-02): default pad (+10, -30);
    the pad and the beam are ONE connected body (the old extension box was a separate component once y1 > -52); lap rows raised (z -86/-68) so the riser clears the skin by >= 20 mm;
    the cable is a planar S-curve + ring in the plane x = -94 behind the beam (round bundle R >= 49.2) into a hollow column clamp, then the ribbon coil; fixed fairing + lap skirt."""
    t0 = time.time(); Dv = P.derive(y1, z1, standoff); x0, XB, XO, PX = Dv["x0"], Dv["XB"], Dv["XO"], Dv["PX"]
    P1 = np.array([PX, y1, z1]); c1 = (y1, z1)
    th_lo, th_hi = (P.HARD["S1p"][0], P.HARD["S1p"][1]) if th_range is None else th_range
    parts = {}; anchors = {}; info = dict(params=dict(y1=y1, z1=z1, standoff=standoff, fair_gap=fair_gap, wall=wall, ext_y=ext_y, ext_z=ext_z, psi_col=psi_col, lap_z=lap_z, pad_r=pad_r, beam_d=beam_d, psi_gland=psi_gland), derived=Dv)
    PAD_R = pad_r; tb = 1.5; s2_inner = PX - AKR; info["s2_inner_face_x"] = round(float(s2_inner), 2)
    # ---------------------------------------------------------------- rail (env) + 4 bosses on the rail top end
    Yr, Zr, Xr = L.rear_surface(-138, -100, -420, -56, 4.0, 6.0); from scipy.interpolate import RegularGridInterpolator
    rail_x = RegularGridInterpolator((Yr[:, 0], Zr[0, :]), Xr, bounds_error=False, fill_value=None)
    rail = L.draped_back_plate(-138, -100, -420, -60, 10.0, 3.5, 4.0, smooth=6.0)
    lap = [(y, z) for y in (-132.0, -112.0) for z in lap_z]; boss_h = 4.5; bosses = []
    for (y, z) in lap:
        xr = float(rail_x([[y, z]])[0]) + 10.0 + 3.5
        bosses.append((y, z, xr)); rail = U(rail, cyl("x", 6.5, xr - 0.5, xr + boss_h, c=(y, z), n=32)); rail = DIFF(rail, cyl("x", 2.8, xr + boss_h - 4.5, xr + boss_h + 0.5, c=(y, z), n=24))
    parts["rail"] = _p(rail, "PACF", "F", (0.30, 0.34, 0.42), kind="env")
    xrail = lambda y, z: float(rail_x([[y, z]])[0]) + 13.5
    # ---------------------------------------------------------------- F: pad disc + HOLLOW BOX-BEAM riser (one connected body)
    zb, zt = ext_z; y0e, y1e = ext_y; bpoly = [(y0e, zt), (y0e, zb), (y1e, zb), (-45.0, -76.0), (y1 - 42.0, -76.0), (y1 - 42.0, zt)]
    bshape = sg.Polygon(bpoly); bin_ = bshape.buffer(-wall)
    beam = DIFF(prism(bpoly, "x", x0, x0 + beam_d), prism(list(bin_.exterior.coords)[:-1], "x", x0 + wall, x0 + beam_d - wall))
    for (y, z, xr) in bosses: beam = U(beam, cyl("x", 6.0, x0 + wall - 0.3, x0 + beam_d - wall + 0.3, c=(y, z), n=32))
    pad = cyl("x", PAD_R, x0, XB, c=c1, n=96); pr = U(pad, beam)
    fpsi = (135.0, 215.0)
    for psi in fpsi:
        u = np.array([math.cos(math.radians(psi)), math.sin(math.radians(psi))]); cc_ = np.array(c1) + u * (PAD_R - 6.0); pr = U(pr, cyl("x", 6.0, x0 - 6.0, x0, c=tuple(cc_), n=24))
        pr = DIFF(pr, _radial_cyl(y1, z1, psi, PAD_R + 0.5, PAD_R + 0.5 - 5.7, 2.3, x0 - 3.0))
    r_st = 54.0; dpsi = 2 * math.degrees(math.asin((12.0 + 2 * tb) / (2 * r_st))); post_ang = (th_lo - dpsi, th_hi + dpsi)
    def S1pt(th, r): return np.array([y1 - r * math.sin(math.radians(th)), z1 + r * math.cos(math.radians(th))])
    for th in post_ang: pr = DIFF(pr, cyl("x", 2.25, x0 - 1.0, XB + 1.0, c=tuple(S1pt(th, r_st)), n=24))
    for (y, z, xr) in bosses:
        pr = DIFF(pr, cyl("x", 2.25, x0 - 1, x0 + beam_d + 1, c=(y, z), n=24)); pr = DIFF(pr, cyl("x", 4.4, x0 + beam_d - 2.4, x0 + beam_d + 0.5, c=(y, z), n=32))     # M4 button head counterbore on the rear face
    parts["pad_riser"] = _p(pr, "PACF", "F", COL["pacf"], fill=0.55)
    info["pad_riser_components"] = len(pr.split(only_watertight=False))
    sto = []
    for (y, z, xr) in bosses: sto.append(DIFF(cyl("x", 5.0, xr + boss_h, x0, c=(y, z), n=32), cyl("x", 2.25, xr + boss_h - 1, x0 + 1, c=(y, z), n=24)))
    parts["standoffs"] = _p(U(*sto), "PETGCF", "F", COL["steel"], fill=0.8)
    info["standoff_length_mm"] = [round(x0 - (xr + boss_h), 1) for (_, _, xr) in bosses]
    # ---------------------------------------------------------------- F: S1' motor, stop posts, bumpers, hollow cable column, fixed fairing
    parts["S1_AK70"] = _p(cyl("x", AKR, XB, XO, c=c1, n=96), "AL", "F", COL["motor"], mass_g=621.0, kind="bought")
    x_post = PX - 48.5 - 3.5; posts, bump = [], []
    for th in post_ang:
        c = S1pt(th, r_st); post = cyl("x", 6.0, XB, x_post, c=tuple(c), n=32); post = DIFF(post, cyl("x", 2.8, XB - 0.5, XB + 4.0, c=tuple(c), n=24)); posts.append(post)
        side = 1.0 if th == post_ang[0] else -1.0; cb = S1pt(th + side * (6.0 + tb / 2) / r_st * 57.2958, r_st); bump.append(cyl("x", tb / 2 + 0.01, XB, x_post, c=tuple(cb), n=16))
    parts["S1_stop_posts"] = _p(U(*posts), "PETGCF", "F", COL["accent"], fill=0.6); parts["tpu_S1_bumpers"] = _p(U(*bump), "TPU", "F", COL["tpu"], fill=0.8)
    info["S1_stop_post_angles_deg"] = [round(a, 1) for a in post_ang]; info["S1_stop_chord_angle_deg"] = round(dpsi, 2); info["S1_post_top_x"] = round(float(x_post), 1)
    cc = np.array([y1 + 56.0 * math.cos(math.radians(psi_col)), z1 + 56.0 * math.sin(math.radians(psi_col))])
    col_out = cyl("x", 5.5, XB, XO + 27.0, c=tuple(cc), n=32); col = DIFF(col_out, cyl("x", 3.5, XB + 0.5, XO + 27.5, c=tuple(cc), n=24))      # hollow: the conductor jumper runs inside
    parts["column_anchor"] = _p(col, "PETGCF", "F", COL["steel"], fill=0.6)
    anchors["S1_balancer_base"] = [float(XO + 22.0), float(cc[0]), float(cc[1])]; anchors["cable_outer_anchor"] = [float(XO + 14.5), float(cc[0]), float(cc[1])]; anchors["cable_F_clamp"] = [-94.0, float(cc[0]), float(cc[1])]
    Ro, Ri = PAD_R + 1.8, PAD_R + 0.2; x_end = XO + 31.0; xf0 = x0 - 6.0
    cup = DIFF(cyl("x", Ro, xf0, x_end, c=c1, n=96), cyl("x", Ri, xf0 - 1, x_end - wall, c=c1, n=96))
    cut = []
    for th in np.arange(th_lo - 2.5, th_hi + 2.6, 5.0):
        Mx = tr([0, y1, z1]) @ Rx(th) @ tr([0, -y1, -z1])
        sl = cyl("y", 48.5 + fair_gap, 30.0 + y1, 130.0 + y1, c=(PX, z1), n=48); sl.apply_transform(Mx); cut.append(sl)
        bar = box(PX - 6.0 - fair_gap, PX + 6.0 + fair_gap, y1 + 0.0, y1 + TS + 3.0, z1 - 26.0 - fair_gap, z1 + 26.0 + fair_gap); bar.apply_transform(Mx); cut.append(bar)
        pl_ = box(PX - 48.5 - fair_gap, PX + 48.5 + fair_gap, y1 + TS - 1.3 - fair_gap, y1 + TS + 3.7 + fair_gap, z1 - 48.5 - fair_gap, z1 + 48.5 + fair_gap); pl_.apply_transform(Mx); cut.append(pl_)
    fair = DIFF(cup, U(*cut))
    win = prism(list(bshape.buffer(0.4).exterior.coords)[:-1], "x", x0 - 0.4, x0 + beam_d + 0.4); fair = DIFF(fair, win)                    # the beam passes through the cup wall (0.4 mm gap)
    for psi in fpsi: fair = DIFF(fair, _radial_cyl(y1, z1, psi, PAD_R + 4.0, PAD_R - 1.0, 1.7, x0 - 3.0))
    # ---------------------------------------------------------------- M1: yoke, collar, S2 motor, clamp, sleeve, stops
    disc = cyl("x", 47.0, XO, XO + 12.0, c=c1, n=96); bar0 = box(PX - 6.0, PX + 6.0, y1, y1 + TS - 1.3, z1 - 26.0, z1 + 26.0)
    ylk = DIFF(U(disc, bar0), cyl("x", 10.0, XO - 1, XO + 13, c=c1, n=48)); parts["S1_yoke"] = _p(ylk, "PETGCF", "M1", COL["steel"], fill=0.6)
    coll = cyl("x", 45.8, x_post, XO, c=c1, r_in=44.6, n=96); lug = cyl("x", 6.0, XB + 0.5, x_post, c=tuple(c1 + np.array([0.0, r_st])), n=32)
    parts["S1_stop_collar"] = _p(U(coll, lug), "PETGCF", "M1", COL["accent"], fill=0.6)
    s2m = cyl("y", AKR, y1 + TS, y1 + TS + AKL, c=(PX, z1), n=96); parts["S2_AK70"] = _p(s2m, "AL", "M1", COL["motor"], mass_g=621.0, kind="bought")
    clamp = cyl("y", 48.5, y1 + TS - 1.3, y1 + TS + 34.0, c=(PX, z1), r_in=AKR + 0.1, n=96); clamp = U(clamp, DIFF(box(PX - 48.5, PX + 48.5, y1 + TS - 1.3, y1 + TS + 3.7, z1 - 48.5, z1 + 48.5), cyl("y", AKR + 0.1, y1 + TS - 2, y1 + TS + 5, c=(PX, z1), n=96)))
    parts["S2_clamp"] = _p(clamp, "PETGCF", "M1", COL["steel"], fill=0.6)
    sleeve = cyl("y", 48.5, y1 + TS + 34.3, y1 + TS + AKL, c=(PX, z1), r_in=AKR + 0.1, n=96); parts["S2_sleeve"] = _p(sleeve, "PACF", "M1", BLUE)
    r_s2 = 66.0; dps = 2 * math.degrees(math.asin((10.0 + 2 * tb) / (2 * r_s2))); ph_lo, ph_hi = P.HARD["S2"]; s2_ang = (ph_lo - dps, ph_hi + dps); sp, sb = [], []
    def S2pt(ph, r): return np.array([PX - r * math.sin(math.radians(ph)), z1 + r * math.cos(math.radians(ph))])
    for ph in s2_ang:
        c = S2pt(ph, r_s2); sp.append(cyl("y", 5.0, y1 + TS + 34.0, y1 + TS + 44.0, c=tuple(c), n=32))
        sd = 1.0 if ph == s2_ang[0] else -1.0; cb = S2pt(ph + sd * (5.0 + tb / 2) / r_s2 * 57.2958, r_s2); sb.append(cyl("y", tb / 2 + 0.01, y1 + TS + 34.0, y1 + TS + 44.0, c=tuple(cb), n=16))
    parts["S2_stop_posts"] = _p(U(*sp), "PETGCF", "M1", COL["accent"], fill=0.6); parts["tpu_S2_bumpers"] = _p(U(*sb), "TPU", "M1", COL["tpu"], fill=0.8)
    info["S2_stop_post_angles_deg"] = [round(a, 1) for a in s2_ang]; anchors["S2_balancer_yoke"] = [float(PX), float(y1 + 30.0), float(z1 + 26.0 + 6.0)]
    # ---------------------------------------------------------------- M2: adapter, S3 cradle bracket (with a cable notch), XM540, S2 lug; ARM
    ya = y1 + T_ARM
    parts["S2_adapter"] = _p(cyl("y", 40.0, y1 + TS + AKL, ya, c=(PX, z1), n=96), "PETGCF", "M2", COL["steel"], fill=0.6)
    plate = box(PX - 24.0, PX + 24.0, ya, ya + 12.0, z1 - 40.0, z1 + 66.0)
    cr_out = box(PX - 32.55, PX + 32.55, ya - 20.05, ya + 20.05, z1 + 60.0, z1 + 112.0); cr_in = box(PX - 29.55, PX + 29.55, ya - 17.05, ya + 17.05, z1 + 58.0, z1 + 113.0)
    xN, zN = PX + 22.0, z1 + 100.0
    brk = DIFF(U(plate, cr_out), cr_in, box(xN - 7.0, xN + 7.0, ya - 21.0, ya - 16.0, zN - 5.0, zN + 5.0)); parts["S3_bracket"] = _p(brk, "PETGCF", "M2", COL["steel"], fill=0.6)   # 14 x 10 cable notch in the cradle -y wall
    parts["S3_XM540"] = _p(box(PX - 29.25, PX + 29.25, ya - 16.75, ya + 16.75, z1 + 68.0, z1 + 112.0), "AL", "M2", COL["motor"], mass_g=165.0, kind="bought")
    parts["S2_stop_lug"] = _p(cyl("y", 5.0, y1 + TS + AKL - 14.0, y1 + TS + AKL, c=tuple(S2pt(0.0, r_s2)), n=32), "PETGCF", "M2", COL["accent"], fill=0.6)
    parts["arm_tube"] = _p(cyl("z", 32.0, z1 + 113.0, z1 + 190.0, c=(PX, ya), n=64), "PETG", "ARM", COL["stage"], kind="arm")
    parts["arm_elbow_ball"] = _p(L.sph(44.0, (PX, ya, z1 + 230.0)), "PETG", "ARM", COL["stage"], kind="arm")
    anchors["S2_balancer_arm"] = [float(PX), float(ya - 24.0), float(z1 + 44.0)]
    # ---------------------------------------------------------------- CABLES (rev 2b)
    OD = 8.2; rb = OD * 6.0; ringR = 54.0; x_pl = -94.0                               # F bundle plane x = -94 (7 mm behind the beam rear face -101, ahead of the S1 stop-post tops -55.8 only matters at the ring end)
    from scipy.interpolate import splprep, splev
    psig = math.radians(psi_gland); Qg = np.array([y1 + ringR * math.cos(psig), z1 + ringR * math.sin(psig)]); hQ = np.array([math.sin(psig), -math.cos(psig)])
    E0 = np.array([-119.0, -140.0]); sc_, scinfo = _s_curve(Qg, hQ, E0, [0.0, 1.0], rb + 1.0, l0=0.0)
    ring_ang = np.arange(psi_gland - 2.0, psi_col + 10.5 - 0.01, -3.0); ringp = np.array([[y1 + ringR * math.cos(math.radians(a)), z1 + ringR * math.sin(math.radians(a))] for a in ring_ang])
    pl2 = np.vstack([sc_, ringp]); Fpath = [(x_pl, p[0], p[1]) for p in pl2]
    # rail-side harness stub: E0 is the (assumed) trunk exit from the main box, 10 mm behind the rail face
    parts["cable_F"] = _p(seg_tube(Fpath, OD / 2), "SILICONE", "F", (0.85, 0.65, 0.1), kind="cable"); Fpath_all = Fpath
    # gland in the fixed fairing where the bundle crosses the cup wall (grommet closes the gap to <= 3 mm)
    cross = [p for p in Fpath if 60.0 < math.hypot(p[1] - y1, p[2] - z1) < 72.0]
    if cross: fair = DIFF(fair, tube_solid(cross, OD / 2 + 2.5))
    parts["fairing_fixed"] = _p(fair, "PACF", "F", BLUE)
    # lap skirt: static wrap of the standoff / rail-lap cavity (rear = beam front face)
    x_far = min(xr for (_, _, xr) in bosses) - 0.5
    sk = DIFF(box(x_far, x0, y0e - wall - 0.4, y1e + wall + 0.4, zb - wall - 0.4, zt + wall + 0.4), box(x_far - 1, x0 + 0.5, y0e - 0.4, y1e + 0.4, zb - 0.4, zt + 0.4))
    Ys_, Zs_, Xs_ = L.rear_surface(-170, -80, -120, -30, 4.0, 2.0); keepout = L.height_solid(Ys_, Zs_, Xs_ - 300.0, Xs_ + 5.0, "x"); sk = DIFF(sk, keepout)
    rail_d = U(rail, *[(lambda m: (m.apply_translation(t), m)[1])(rail.copy()) for t in ([-1.5, 0, 0], [1.5, 0, 0], [0, 0, 1.5], [0, 0, -1.5])]); sk = DIFF(sk, rail_d)
    parts["fairing_lap"] = _p(sk, "PACF", "F", BLUE)
    # ribbon coil behind the yoke disc + M1 + M2 (ribbon R >= 12.6)
    xcoil = XO + 17.8; Rs = np.linspace(40.0, 14.0, 60); ang = math.radians(psi_col) + np.linspace(0, -2 * math.pi * 1.5, 60)
    coil = [(xcoil, y1 + r * math.cos(a), z1 + r * math.sin(a)) for r, a in zip(Rs, ang)]; parts["cable_coil"] = _p(seg_tube(coil, 1.05, n=8), "SILICONE", "F", (0.9, 0.7, 0.15), kind="cable")
    pc = np.array(coil[-1]); yE, zE = y1 + TS - 1.5, z1 + 33.0
    m1w = [pc, np.array([xcoil, y1 + 26.0, z1 - 4.0]), np.array([xcoil, y1 + 36.0, zE]), np.array([xcoil, yE, zE])]
    M1path, rM1 = fillet_path(m1w, 12.8); parts["cable_M1"] = _p(seg_tube(M1path, OD / 2), "SILICONE", "M1", (0.85, 0.65, 0.1), kind="cable")
    ys = y1 + TS + 46.0; ra = 54.3
    wrap = [(PX - ra * math.sin(math.radians(a_)), ys, z1 + ra * math.cos(math.radians(a_))) for a_ in np.arange(100.0, 0.0, -6.0)]
    yv = y1 + TS + 20.5; m2w = [np.array(wrap[-1]), np.array([PX + 22.0, ys, z1 + ra]), np.array([PX + 22.0, yv, z1 + ra]), np.array([xN, yv, zN]), np.array([xN, ya - 17.5, zN])]
    m2f, rM2 = fillet_path(m2w, 12.8); M2path = np.vstack([np.array(wrap[:-1]), m2f]); parts["cable_M2"] = _p(seg_tube(M2path, OD / 2), "SILICONE", "M2", (0.85, 0.65, 0.1), kind="cable")
    info["paths"] = dict(cable_F=Fpath_all, cable_coil=coil, cable_M1=M1path, cable_M2=M2path, radius=dict(cable_F=OD / 2, cable_coil=1.05, cable_M1=OD / 2, cable_M2=OD / 2))
    rF = mincirc(np.array(Fpath)); info["cable"] = dict(OD_mm=OD, bundle_min_bend_required_mm=round(rb, 1), ribbon_min_bend_required_mm=12.6, F_min_bend_mm=round(float(rF), 1), ring_radius_mm=ringR, S_curve=scinfo, F_plane_x=x_pl,
        M1_min_bend_mm=round(float(mincirc(M1path)), 1), M1_fillet_R_used=round(float(rM1), 1), M2_min_bend_mm=round(float(mincirc(M2path)), 1), M2_fillet_R_used=round(float(rM2), 1), M2_wrap_radius_mm=ra, M2_wrap_arc_deg=[0.0, 100.0],
        coil_turns=1.5, coil_mean_radius_mm=27.0, coil_ribbon="8 x 18 AWG silicone wires flat, 17 x 2.1 mm (PLACEHOLDER, measure OD)", gland_psi_deg=psi_gland, column_psi_deg=psi_col, E0_assumed_trunk_exit_xyz=[x_pl, float(E0[0]), float(E0[1])],
        F_path_len_mm=round(float(np.linalg.norm(np.diff(np.array(Fpath), axis=0), axis=1).sum()), 0))
    info["rail_lap"] = dict(bolts=[(y, z) for (y, z, _) in bosses], rail_rear_face_x=[round(xr, 1) for (_, _, xr) in bosses], boss_h=boss_h, beam_depth=beam_d, beam_wall=wall)
    info["features_positions_only"] = dict(
        S1_AK70_flange=dict(where="pad rear face x %.1f, centre (y %.0f, z %.0f)" % (XB, y1, z1), pattern="PLACEHOLDER: drawing text shows 6-M3 and 8-M3 callouts, which one is the output flange and its PCD are NOT readable (cubemars 2D drawing PDF text, retrieved 2026-11-02): the earlier 6 x M4 on 60 PCD guess is probably wrong (M3)", printed=False),
        S1_yoke_flange=dict(where="yoke disc front face x %.1f, same axis" % XO, pattern="PLACEHOLDER (same AK70 output flange)", printed=False),
        S2_clamp_ears=dict(where="clamp ring (axis y through (PX %.1f, z %.0f)), ears at +-x of the ring, split plane z = z1" % (PX, z1), fasteners="2 x M4 x 16 (baseline clamp_local carried)", printed=False),
        bar_to_clamp_tabs=dict(where="yoke bar end face y = y1 + TS - 1.3", fasteners="4 x M4 x 12", printed=False),
        collar_to_disc=dict(where="stop collar rear flange on the disc rim, r 46", fasteners="4 x M3 x 8", printed=False),
        XM540=dict(horn="HN13 horn set: horn bolt M3 x 8, frame bolts M2.5 x 4 x 10 (robotis.us); horn PCD, frame holes, output axis and connector positions NOT in the sources", printed=False),
        vendor_sources="vendor_patterns.json (2026-11-02): body dims, masses, torque, backlash (AK70 0.2 deg, XM540 0.25 deg), AK70 bearing C/C0 2000/2520 N, CAN A1257WR-S-4P, UART A1257WR-S-3P, power XT30PW-M are SOURCED")
    info["anchors"] = anchors; info["build_s"] = round(time.time() - t0, 1)
    for k, v in parts.items(): v.setdefault("name", k)
    return (parts, info) if full else parts
PL_T = P.PL_T
# ------------------------------------------------------------------ posing
def pose_matrix(group, Dv, th1, phi):
    P1 = np.array([Dv["PX"], Dv["y1"], Dv["z1"]]); Mth = tr(P1) @ Rx(th1) @ tr(-P1); Mph = tr(P1) @ Ry(-phi) @ tr(-P1)
    if group in ("M2", "ARM"): return Mth @ Mph
    if group == "M1": return Mth
    return np.eye(4)
def pose(parts, th1, phi, Dv=None):
    """returns {name: posed mesh}; Dv = P.derive(y1,z1,standoff) of the build (stored in parts['_derived'] if build(full=True) info is kept)."""
    out = {}
    for k, v in parts.items():
        if k.startswith("_"): continue
        m = v["mesh"].copy(); m.apply_transform(pose_matrix(v["group"], Dv, th1, phi)); out[k] = m
    return out
def mirror_L(m):
    m = m.copy(); m.apply_transform(np.diag([1, -1, 1, 1.0])); m.apply_translation([0, -300.0, 0]); m.invert(); return m
# ------------------------------------------------------------------ checks + export
def inter_vol_(A, B): return inter_vol(A, B)
def _js(o):
    if isinstance(o, np.ndarray): return np.round(o, 2).tolist()
    if isinstance(o, (np.floating, np.integer)): return float(o)
    return str(o)
def main(**kw):
    parts, info = build(full=True, **kw); Dv = info["derived"]; chk = {}
    th, ph = P.PARK["S1p"], P.PARK["S2"]; F0, FA, s, a, d = P.shoulder_frames(Dv, th, ph); M = pose_matrix("M2", Dv, th, ph)
    chk["pose_equals_shoulder_frames"] = bool(np.allclose((M @ np.r_[np.array([Dv["PX"], Dv["y1"] + P.T_ARM, Dv["z1"]]), 1])[:3], FA[:3, 3], atol=1e-6) and np.allclose(M[:3, :3] @ [0, 0, 1], d, atol=1e-6))
    sk = {k: round(L.skin_gap(parts[k]["mesh"], 4000)[0], 1) for k in ("pad_riser", "standoffs", "fairing_fixed", "fairing_lap", "rail", "column_anchor", "S1_stop_posts")}
    sw = {}
    for th_ in np.arange(P.HARD["S1p"][0], P.HARD["S1p"][1] + 0.1, 15.0):
        Mx = pose_matrix("M1", Dv, th_, 0.0); m = parts["S2_sleeve"]["mesh"].copy(); m.apply_transform(Mx)
        sw[int(th_)] = dict(skin=round(L.skin_gap(m, 2500)[0], 1), riser=round(mindist(m, parts["pad_riser"]["mesh"], 2500), 1), posts=round(mindist(m, parts["S1_stop_posts"]["mesh"], 2500), 1), fairing=round(mindist(m, parts["fairing_fixed"]["mesh"], 2500), 1), rail=round(mindist(m, parts["rail"]["mesh"], 2500), 1))
    F = []
    def fastener(name, p0, d_, length, r, through, note):
        d_ = np.asarray(d_, float) / np.linalg.norm(d_); ax = np.cross([0, 0, 1.0], d_); sn = np.linalg.norm(ax)
        M_ = rot(math.atan2(sn, d_[2]), ax / sn) if sn > 1e-9 else (np.eye(4) if d_[2] > 0 else rot(math.pi, [1, 0, 0])); m = trimesh.creation.cylinder(r, length, sections=24); M_[:3, 3] = np.asarray(p0) + d_ * length / 2; m.apply_transform(M_)
        hits = {}
        for k, v in parts.items():
            if v["kind"] not in ("printed", "env", "bought"): continue
            if np.any(m.bounds[1] < v["mesh"].bounds[0]) or np.any(v["mesh"].bounds[1] < m.bounds[0]): continue
            h = inter_vol(m, v["mesh"])
            if h > 0.05: hits[k] = round(h, 2)
        F.append(dict(name=name, start=[round(float(x), 1) for x in p0], dir=[round(float(x), 2) for x in d_], length=length, shank_dia=2 * r, passes_through=through, shank_material_intersections_mm3=hits, clean=bool(len(hits) == 0), note=note))
    x0, XB = Dv["x0"], Dv["XB"]
    for i, ((y, z), xr, sl) in enumerate(zip(info["rail_lap"]["bolts"], info["rail_lap"]["rail_rear_face_x"], info["standoff_length_mm"])):
        bd = info["rail_lap"]["beam_depth"]; need = (bd - 2.4) + sl + 4.0; Lb = next(l for l in (12, 16, 20, 25, 30, 35, 40) if l >= need)
        fastener(f"rail_lap_M4x{Lb}_{i + 1}", [x0 + bd - 2.4, y, z], [-1, 0, 0], need, 2.0, ["pad_riser beam (rear counterbore D8.8 x 2.4, clearance 4.5 through the post)", "standoff D10 (bore 4.5)", "rail boss (M4 insert 5.6 x 4.5)"], f"ISO 7380 M4 x {Lb}, need {need:.1f} mm to 4 mm thread in the insert; standoff {sl:.1f} mm")
    for psi in (135.0, 215.0):
        u = np.array([math.cos(math.radians(psi)), math.sin(math.radians(psi))]); p0 = np.array([Dv["y1"], Dv["z1"]]) + u * (info["params"]["pad_r"] + 4.0)
        fastener(f"fairing_M3x10_psi{int(psi)}", [Dv["x0"] - 3.0, *p0], [0, -u[0], -u[1]], 9.0, 1.5, ["fairing_fixed (clearance 3.4)", "pad front boss (radial M3 insert 4.6 x 5.7)"], "ISO 7380 M3 x 10 radial")
    for k, th_ in enumerate(info["S1_stop_post_angles_deg"]):
        c = np.array([Dv["y1"] - 54.0 * math.sin(math.radians(th_)), Dv["z1"] + 54.0 * math.cos(math.radians(th_))])
        fastener(f"S1_post_M4x10_{k + 1}", [Dv["x0"], *c], [1, 0, 0], 10.0, 2.0, ["pad_riser (clearance 4.5, ISO 10642 head flush on the front)", "S1_stop_posts (M4 insert 5.6 x 4 in the post foot)"], "ISO 10642 M4 x 10 from the pad front")
    rm = parts["pad_riser"]["mesh"]; sp_ = trimesh.sample.sample_surface(rm, 40000)[0]; tk_, pts_k, nrm_k = L._tree(); dd_, ii_ = tk_.query(sp_); sdr = np.sign(np.einsum("ij,ij->i", sp_ - pts_k[ii_], nrm_k[ii_])) * dd_
    chk["riser_skin_gap_mm"] = {"over_blade_y_gt_-100": round(float(sdr[sp_[:, 1] > -100.0].min()), 1), "rail_zone_y_le_-100": round(float(sdr[sp_[:, 1] <= -100.0].min()), 1), "all": round(float(sdr.min()), 1)}
    stops = {}
    for lab, th_ in (("hard_lo", P.HARD["S1p"][0]), ("soft_lo", P.SOFT["S1p"][0]), ("park", P.PARK["S1p"]), ("soft_hi", P.SOFT["S1p"][1]), ("hard_hi", P.HARD["S1p"][1])):
        col = parts["S1_stop_collar"]["mesh"].copy(); col.apply_transform(pose_matrix("M1", Dv, th_, 0.0))
        stops[lab] = dict(th1=round(float(th_), 1), lug_to_TPU_bumpers_mm=round(exact_min(col, parts["tpu_S1_bumpers"]["mesh"]), 2), lug_to_post_mm=round(exact_min(col, parts["S1_stop_posts"]["mesh"]), 2), collar_to_pad_riser_mm=round(exact_min(col, parts["pad_riser"]["mesh"]), 2))
    chk["S1_stops_exact"] = stops
    mv = {}
    for th_ in np.arange(P.HARD["S1p"][0], P.HARD["S1p"][1] + 0.1, 10.0):
        for nm in ("S1_yoke", "S2_clamp", "S2_sleeve", "S2_AK70", "S1_stop_collar"):
            m_ = parts[nm]["mesh"].copy(); m_.apply_transform(pose_matrix("M1", Dv, th_, 0.0))
            for fx in ("pad_riser", "fairing_lap", "standoffs", "rail", "fairing_fixed", "column_anchor"):
                dk = f"{nm}|{fx}"; mv[dk] = round(min(mv.get(dk, 1e9), exact_min(m_, parts[fx]["mesh"], 4000)), 1)
    chk["moving_vs_fixed_min_mm_S1p_hard_range_S2_0"] = dict(sorted(mv.items(), key=lambda kv: kv[1])[:14])
    chk["fasteners"] = F; chk["all_fastener_shanks_clean"] = bool(all(f["clean"] for f in F))
    pairs = [("pad_riser", "fairing_fixed"), ("pad_riser", "S1_AK70"), ("pad_riser", "standoffs"), ("standoffs", "rail"), ("S1_AK70", "fairing_fixed"), ("S1_AK70", "S1_stop_collar"), ("S1_yoke", "fairing_fixed"), ("S1_yoke", "S1_AK70"), ("S1_yoke", "S1_stop_collar"), ("S1_stop_collar", "S1_stop_posts"), ("S1_stop_collar", "pad_riser"), ("S2_AK70", "S1_yoke"), ("S2_AK70", "S2_clamp"), ("S2_sleeve", "S2_AK70"), ("S2_sleeve", "S2_clamp"), ("S2_adapter", "S2_AK70"), ("S3_bracket", "S2_adapter"), ("S3_XM540", "S3_bracket"), ("column_anchor", "fairing_fixed"), ("column_anchor", "pad_riser"), ("S2_stop_posts", "S2_clamp"), ("S2_stop_posts", "S2_sleeve"), ("S2_clamp", "S1_yoke"), ("S2_clamp", "S1_stop_collar"), ("fairing_lap", "standoffs"), ("fairing_lap", "rail"), ("fairing_lap", "pad_riser"), ("fairing_lap", "fairing_fixed"), ("column_anchor", "S1_AK70"), ("column_anchor", "S1_stop_posts"), ("column_anchor", "S1_yoke")]
    cab = {}
    cands = {"cable_F": ["pad_riser", "fairing_fixed", "fairing_lap", "S1_AK70", "column_anchor", "rail", "standoffs", "S1_stop_posts", "S1_yoke"], "cable_coil": ["S1_yoke", "fairing_fixed", "column_anchor", "S2_clamp", "S1_AK70"], "cable_M1": ["S1_yoke", "S2_clamp", "S2_AK70", "S1_stop_collar", "fairing_fixed"], "cable_M2": ["S2_sleeve", "S2_clamp", "S2_stop_posts", "S2_adapter", "S3_bracket", "S2_AK70", "S3_XM540", "S2_stop_lug"]}
    for cn, lst in cands.items():
        cab[cn] = {k: round(path_clearance(info["paths"][cn], parts[k]["mesh"], info["paths"]["radius"][cn]), 1) for k in lst}
    chk["cable_clearance_mm"] = cab; chk["cable_all_clear"] = bool(all(v >= 0.0 for d_ in cab.values() for v in d_.values()))
    chk["interference_mm3_reference_pose"] = {f"{a_}|{b_}": round(inter_vol(parts[a_]["mesh"], parts[b_]["mesh"]), 1) for a_, b_ in pairs}
    rects = [(32.0, 6.0, 3.0), (3.0, 8.0, -4.0), (3.0, 8.0, -4.0)]
    # extension section for the rail-lap: plate (y extent 30, x 6) carrying the pad moment to the 4 standoffs; stress from the standoff bolt group
    Mroot = 14.5e3; arm_ = 2 * 10.0     # 14.5 N.m about an in-plane axis reacted by the 4-bolt group: bolt rows z -92/-74, columns y -132/-112
    dz = abs(info["rail_lap"]["bolts"][0][1] - info["rail_lap"]["bolts"][1][1]); dy = abs(info["rail_lap"]["bolts"][0][0] - info["rail_lap"]["bolts"][2][0])
    Fpull = Mroot / (2 * dz) / 2; chk["rail_lap_bolt_group"] = dict(M_Nmm=Mroot, row_spacing_mm=dz, col_spacing_mm=dy, tension_per_bolt_N_about_row_axis=round(Fpull, 0), note="14.5 N.m = 12 N.m shoulder reaction + 29 N weight x 85 mm; pull-out of M4 insert in a PA-CF rail boss ~1 kN typical (UNVERIFIED) -> SF %.1f; the rail top end carries the full shoulder moment: harness_loads / body_loads MUST be re-run" % (1000.0 / Fpull))
    rows = []; dens = L.DENS; dens.setdefault("SILICONE", 1.1)
    for k, v in parts.items():
        m = v["mesh"]; ext = m.bounds[1] - m.bounds[0]; wt = bool(m.is_watertight); vol = float(m.volume) / 1000.0 if wt else float("nan")
        mass = v.get("mass_g", (vol * dens[v["mat"]] * v.get("fill", 1.0)) if wt else float("nan")); rows.append(dict(name=k, grp=v["group"], mat=v["mat"], kind=v["kind"], mass_g=round(float(mass), 1), vol_cm3=round(vol, 1), bbox=[round(float(x), 1) for x in ext], watertight=wt, fits_256=bool(np.all(m.bounding_box_oriented.extents <= 256.0))))
    cnt = [r for r in rows if r["kind"] in ("printed", "bought") and r["name"] != "rail" and np.isfinite(r["mass_g"])]
    depth = float(max(v["mesh"].bounds[1][0] for v in parts.values() if v["kind"] in ("printed", "bought") and v["group"] not in ("ARM",) and v["name"] != "rail") + 108.0)
    met = dict(info=info, checks=chk, skin_gap_mm=sk, S2_sleeve_over_S1p_mm=sw, parts=rows, mass_counted_g=round(sum(r["mass_g"] for r in cnt), 1), mass_printed_g=round(sum(r["mass_g"] for r in cnt if r["kind"] == "printed"), 1), mass_bought_g=round(sum(r["mass_g"] for r in cnt if r["kind"] == "bought"), 1), depth_behind_back_mm=round(depth, 1))
    cc_ = chk["cable_clearance_mm"]; opens = []; cb = info["cable"]
    for nm, val, need in (("cable_F bundle", cb["F_min_bend_mm"], cb["bundle_min_bend_required_mm"]), ("cable_M1 ribbon", cb["M1_min_bend_mm"], 12.6), ("cable_M2 ribbon", cb["M2_min_bend_mm"], 12.6)):
        if val < need - 0.05: opens.append(f"{nm}: min bend {val} mm < {need} mm")
    for cn, d_ in cc_.items():
        for k_, v_ in d_.items():
            if v_ < 1.0: opens.append(f"{cn} vs {k_}: clearance {v_} mm (< 1 mm; terminations excluded)")
    for k_, v_ in chk["riser_skin_gap_mm"].items():
        if (k_ == "over_blade_y_gt_-100" and v_ < 20.0) or (k_ == "rail_zone_y_le_-100" and v_ < 5.0): opens.append(f"riser skin gap {k_}: {v_} mm below its rule")
    if not chk["all_fastener_shanks_clean"]: opens.append("a fastener shank intersects material: " + str([f["name"] for f in chk["fasteners"] if not f["clean"]]))
    if any(v > 0.5 for v in chk["interference_mm3_reference_pose"].values()): opens.append("interference at park: " + str({k: v for k, v in chk["interference_mm3_reference_pose"].items() if v > 0.5}))
    opens += ["AK70 / XM540 flange patterns, S2 clamp ears, bar-to-clamp M4 tabs, collar-to-disc M3 flange are placed as positions only (vendor patterns PLACEHOLDER, nothing printed)", "ribbon / bundle ORIGINS: the trunk exit E0 at the main box and the two bundle->ribbon transitions (column clamp, S3 notch) are PLACEHOLDER positions; ribbon twist length >= 4 x 17 mm is not modelled", "rail contract change: loads from the arm enter the paraspinal rail top end through 4 standoffs: harness_loads + body_loads + verify_harness MUST be re-run (not done here)", "S1 stop posts are 4.3 mm tall (below the S2 motor swing): bench-check the TPU bumper crush and the 154 N stop load (8.3 N.m at r 54)", "the S1' balancer anchor / spring must be re-fitted at this pad position by cad/j1r2_balancer.py (anchors dict carries the hooks)"]
    met["open_items"] = opens
    for k, v in parts.items():
        v["mesh"].export(os.path.join(OUT, f"{k}_R.stl")); mirror_L(v["mesh"]).export(os.path.join(OUT, f"{k}_L.stl"))
    pm = pose(parts, th, ph, Dv); trimesh.util.concatenate([m for k, m in pm.items()]).export(os.path.join(OUT, "j1r2_view_park_R.stl"))
    pass
    json.dump(met, open(os.path.join(OUT, "metrics.json"), "w"), indent=1, default=_js)
    json.dump(dict(interfaces=INTERFACES(Dv, info, chk), anchors=info["anchors"], placeholders=["AK70-10 output flange pattern (drawing text: 6-M3 / 8-M3 callouts, PCD unknown; the older 6 x M4 on 60 PCD guess is probably wrong; NOT modelled)", "AK70 connector location (inner end face assumed)", "XM540 horn / S3 flange (contract unchanged, not modelled)", "ribbon wire OD, flat-flex clockspring vendor", "main-box cable entry (assumed at y -150, z -125)", "rail boss + M4 inserts (rail contract change)"]), open(os.path.join(OUT, "interfaces_j1r2.json"), "w"), indent=1, default=float)
    open(os.path.join(OUT, "ASSEMBLY_j1r2.md"), "w").write(ASSEMBLY)
    return parts, met
def INTERFACES(Dv, info, chk):
    cb = info["cable"]; rl = info["rail_lap"]
    return [
     dict(boundary="pad_riser <-> paraspinal rail (top end)", type="hollow PA-CF box-beam riser (%.1f deep, %.1f walls, z %s, tapering to z -76 at the pad) lapped on 4 standoffs D10 to 4 bosses on the rail" % (rl["beam_depth"], rl["beam_wall"], str(info["params"]["ext_z"])), where="beam y %s; bolts (y, z) %s; standoff length %s mm" % (str(info["params"]["ext_y"]), rl["bolts"], info["standoff_length_mm"]), fasteners="4 x ISO 7380 M4 (lengths in checks.fasteners) from the rear counterbore D8.8 x 2.4 through a D12 post inside the beam, the standoff and into an M4 heat-set 5.6 x 4.5 in the rail boss", checks="shanks clean; riser skin gap over the blade %s mm (rule 20), rail zone %s mm (rule 5)" % (chk["riser_skin_gap_mm"]["over_blade_y_gt_-100"], chk["riser_skin_gap_mm"]["rail_zone_y_le_-100"])),
     dict(boundary="pad <-> beam", type="one printed body (the beam is fused to the R%.0f pad plate; sealed internal void, 5.9 mm bridging span, print with the front face on the bed)" % info["params"]["pad_r"], checks="%d shells (1 outer + internal void)" % info["pad_riser_components"]),
     dict(boundary="S1' AK70 <-> pad", type="flange on the pad rear face (x %.1f)" % Dv["XB"], fasteners="6 x M4 on 60 PCD (PLACEHOLDER, position only)", checks="motor R44.5 inside pad R%.0f" % info["params"]["pad_r"]),
     dict(boundary="fairing_fixed <-> pad_riser", type="radial M3 into 2 front-rim bosses (psi 135, 215 deg); the beam passes the cup wall through a 0.4 mm window", fasteners="2 x ISO 7380 M3 x 10, M3 heat-set 4.6 x 5.7", checks="shanks clean"),
     dict(boundary="fairing_lap <-> pad_riser / rail", type="static U-skirt (1.8 mm walls) around the standoff cavity: rear edge on the beam front face, front edge follows the rail face with 1.5 mm gap (<= 4 mm); wraps the rail lap", checks="no moving part within 40 mm"),
     dict(boundary="S1' stop posts <-> pad", type="2 low posts (top x %.1f = 3.5 mm under the S2 motor inner face), bolted from the pad FRONT" % info["S1_post_top_x"], fasteners="2 x ISO 10642 M4 x 10, M4 insert 5.6 x 4 in the post foot", checks="posts at %s deg (hard limits + chord)" % info["S1_stop_post_angles_deg"]),
     dict(boundary="stop collar <-> yoke disc", type="flange", fasteners="4 x M3 x 8 into inserts in the disc rim (position only)", checks="collar R44.6..45.8 over the motor R44.5"),
     dict(boundary="yoke <-> S1' motor output", type="flange", fasteners="6 x M4 on 60 PCD (PLACEHOLDER)", checks="D20 centre hole"),
     dict(boundary="yoke bar <-> S2 clamp collar", type="bar end butts the clamp end face", fasteners="4 x M4 x 12 (position only)", checks="0 mm3 interference"),
     dict(boundary="S2 AK70 <-> clamp", type="split ring clamp; ears placed at +-x of the ring (position only)", fasteners="2 x M4 x 16", checks="bore R44.6"),
     dict(boundary="S2 adapter <-> S3 cradle bracket <-> XM540", type="plate on the adapter face, U-cradle 0.3 mm around the XM540 with a 14 x 10 cable notch in the -y wall at (x PX+22, z z1+100)", fasteners="4 x M4 (PLACEHOLDER pattern) + M3 lid screws", checks="contract unchanged: S3 horn / upper-arm flange"),
     dict(boundary="cable F: trunk -> column", type="round bundle OD %.1f, ONE planar run in the plane x = %.0f (7 mm behind the beam): S-curve of two arcs R %.1f (%s) from the assumed trunk exit E0 (y -119, z %s) up the rail side, through a grommeted D13 gland in the cup wall at psi %.0f deg, then the ring R %.0f to the hollow column clamp at psi %.0f deg" % (cb["OD_mm"], cb["F_plane_x"], cb["S_curve"]["radius"], cb["S_curve"], cb["S_curve"]["start_xy"][1], cb["gland_psi_deg"], cb["ring_radius_mm"], cb["column_psi_deg"]), checks="min bend %.1f mm >= %.1f (6 x OD); length %.0f mm" % (cb["F_min_bend_mm"], cb["bundle_min_bend_required_mm"], cb["F_path_len_mm"])),
     dict(boundary="cable: column -> clockspring", type="bundle ends in a clamp block on the hollow column (x -94); 8 conductors run inside the D7 bore to the second clamp at x %.1f where the ribbon coil starts (bundle -> ribbon transition, PLACEHOLDER)" % info["anchors"]["cable_outer_anchor"][0], checks="coil mean R 27 mm >= 12.6"),
     dict(boundary="cable M1: coil -> S2 motor", type="ribbon, filleted polyline in the plane x = XO+17.8, final 11 mm straight lead along +y to the S2 motor inner face connector (position PLACEHOLDER)", checks="min bend %.1f mm >= 12.6" % cb["M1_min_bend_mm"]),
     dict(boundary="cable M2: S2 wrap -> S3", type="ribbon wrapped %s deg at r %.1f around the sleeve, then up the +x side of the S2 clamp and through the cradle notch to the XM540 -y face (connector position PLACEHOLDER)" % (cb["M2_wrap_arc_deg"], cb["M2_wrap_radius_mm"]), checks="min bend %.1f mm >= 12.6; wrap radius >= 49.2" % cb["M2_min_bend_mm"]),
     dict(boundary="balancer anchors", type="named points", anchors=info["anchors"]),
     dict(boundary="harness seats F1/F4", type="slim spaulder tie NOT kept in rev 2", why="the integrated fairing + rail lap carries the load; a spaulder would add a shelf over the shoulder top (against 'flush'). Seats stay as free clasp nodes."),
    ]
ASSEMBLY = """J1 rev 2b assembly (right arm; left = mirror), tools: 2.5 / 3 / 4 mm hex.
1. Press M4 heat-set inserts (5.6 x 4.5) into the 4 rail bosses, M3 inserts (4.6 x 5.7) into the 2 pad front-rim bosses, M4 inserts (5.6 x 4) into the S1 post feet. Seat the 4 standoffs on the rail bosses.
2. Offer the pad_riser (pad + hollow beam) from the +x side: 4 x ISO 7380 M4 from the rear counterbores (D8.8 x 2.4) through the beam posts and standoffs into the rail inserts (lengths per interfaces / fasteners list). Fit the lap skirt (fairing_lap) over the standoff cavity from the body side before the last bolt.
3. Bolt the S1 stop posts to the pad from the pad FRONT (ISO 10642 M4 x 10) and bond the TPU 95A bumpers.
4. Lay the cable_F bundle in the plane x = -94 (trunk exit E0 on the rail side), feed it through the grommeted D13 gland in the cup wall BEFORE the cup is fitted, clamp it on the hollow column; thread the 8 conductors inside the column to the coil clamp.
5. Fix the AK70 (S1') to the pad rear face (6 x M4, PLACEHOLDER pattern). Fit the S1_stop_collar to the yoke disc, then the S1_yoke on the AK70 output (6 x M4 PLACEHOLDER), the ribbon coil under the disc rear face.
6. Slide the fixed fairing cup on from +x over the yoke (the swept cut passes the bar), fix with 2 radial M3 x 10; close the grommet.
7. Mount the S2 AK70 in the clamp on the bar, fit the sleeve, S2 posts + bumpers, adapter, S3 bracket (cable notch toward -y), XM540; wrap the M2 ribbon around the sleeve (100 deg) before closing the cradle; route it through the notch.
8. Dry-move S1' hard stop to hard stop and S2 hard stop to hard stop by hand with motors off; check the ring, the coil and the M2 wrap.
"""
if __name__ == "__main__":
    parts, met = main()
    print(json.dumps(dict(build_s=met["info"]["build_s"], depth=met["depth_behind_back_mm"], mass=(met["mass_counted_g"], met["mass_printed_g"], met["mass_bought_g"]), skin=met["skin_gap_mm"], riser_skin=met["checks"]["riser_skin_gap_mm"], cable=met["info"]["cable"], standoff=met["info"]["standoff_length_mm"], clean=met["checks"]["all_fastener_shanks_clean"], pose=met["checks"]["pose_equals_shoulder_frames"]), default=_js))
    print(json.dumps(met["checks"]["cable_clearance_mm"])); print(json.dumps({k: v for k, v in met["checks"]["interference_mm3_reference_pose"].items() if v > 0}, indent=1)); print(json.dumps(met["S2_sleeve_over_S1p_mm"]))
    print([(f["name"], f["shank_material_intersections_mm3"]) for f in met["checks"]["fasteners"] if not f["clean"]]); print("OPEN", met["open_items"])
