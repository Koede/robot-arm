"""Faceted pauldron (rev 2026-10-13, Forest: "facet it to my shoulder better, rather than a hinge, add a complex connection").
FIXED part (group F). It replaces the look and load path of "S1 stage block bolted to a flat plate" (a hinge bracket) with a
faceted socket that wraps the S1 stage and branches into a multi-strut web that ties the robot shoulder to Forest's shoulder:

  SOCKET HOUSING: faceted shell around the S1 base, S1 clamp, AK70 S1 and the S1 stop posts (inner face G_IN off the
    closed contents, wall T_WALL). Open toward the chassis plate (x >= PLATE_REAR + 0.4) and at the top inside the socket rim:
    rim top Z_RIM (3.4 mm below the turning cowl's bottom), bore R_RIM about the S1 axis (3 mm outside the S1 lug / yoke disc
    sweep). The cowl turning above it closes the socket visually: a faceted ball-and-socket, ~150 mm wide instead of a 190 ball.
  COMPLEX CONNECTION (all faceted struts, SDF ribbons blended into the housing):
    - 3 struts down onto the chassis-plate rear face (feet with 2 x M4 each, into new tapped holes in the plate);
    - 2 wrap-arms that go round the plate's lateral and medial edges and land on the clasp's F4 (acromion) and F1 (upper
      trapezius) nodes -- both stable zones -- with bolted pads (M4 x 12 into heat-set inserts in bosses that
      organic_connector.py adds to the clasp nodes). So stage, plate and clasp become one continuous frame round the shoulder.
  Split for the 256 mm build volume: core (housing + plate struts + arm roots) and two arms, joined by half-lap splices
  (2 x M4 x 16 + nuts each) beside the plate edges.
  DELTOID SHELL: the flat lateral shelf (planar rim plus the arm_lat ribbon out to y = 98) is replaced by a curved
  shell that wraps the deltoid. The outer rim curls inboard. Socket bore, plate feet, and clasp pads still mount.
Outputs stl/shoulder/pauldron/: pauldron_R.stl (all, for sweeps/renders), pauldron_core_R.stl, pauldron_arm_lat_R.stl,
pauldron_arm_med_R.stl, pauldron.json. Right arm, robot frame, mm."""
import os, sys, json, numpy as np, trimesh
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, facet
SH = os.path.join(HERE, "..", "stl", "shoulder"); OUT = os.path.join(SH, "pauldron"); os.makedirs(OUT, exist_ok=True)

VOX = 1.0
G_IN, T_WALL, R_CLOSE, SMOOTH = 3.0, 3.0, 15.0, 2.0
PLATE_REAR = -80.0                    # chassis plate rear face (x); the plate is x -88 .. -80
Z_FLOOR = -103.0                      # housing open below this (the S1 motor hangs free) so the core can slide on along -x
Z_RIM, R_RIM, Z_BORE = -54.0, 66.0, -63.0         # 1 mm extra over the 3 mm gate: facet snapping can move cut faces by <= 1 mm
N_FACETS, N_FACETS_HOUSING, N_FACETS_SHELL, FACET_MARGIN = 360, 110, 1600, 1.0
ST_W, ST_T = 18.0, 9.0                # strut section (width x thickness), mm
SKIN_MIN, PART_MIN = 10.0, 3.0        # design clearances for the arms (skin over the shoulder top moves a little: 10 mm); exempt within 35 mm of the clasp pads

# ---- deltoid shell (replaces the flat lateral shelf) -------------------------------------------------------------
# The old lateral pauldron was a flat shelf: the socket cap cut on the plane z = Z_RIM, and a rectangular
# ribbon (arm_lat) that ran out to y = 98 to clear the shoulder. That shelf is gone. In its place is a
# swept arc whose crown sits 12 mm off the deltoid and whose rims curl inboard of the bulge, so the outer
# edge comes back onto the arm instead of sticking out. The bridge is 9 mm thick so the M4 half-lap still
# has a grip; the cap over the muscle is 5 mm. Bore, plate feet, and the F4 pad are unchanged.
# Not a rigid tuck of the flat plate, and not a scale of the whole part: the section is an arc and the
# crown path is a different curve from the old polyline.

CROWN = np.array([
    # Root stays lateral of the S1 base (y > 72) until x < -85, so the
    # assembly draft (slide-on along -x) does not shave the shell off.
    [-10.0, 76.0, -66.0],
    [-36.0, 78.0, -76.0],
    [-62.0, 76.0, -86.0],
    [-96.0, 66.0, -104.0],
    [-132.0, 60.0, -120.0],
    [-158.0, 70.0, -142.0],
    [-170.0, 80.0, -160.0],
    [-168.0, 76.0, -172.0],
], float)
ANG_POS = np.array([16, 22, 30, 42, 54, 64, 72, 74], float)
ANG_NEG = np.array([14, 18, 24, 32, 38, 44, 48, 48], float)
THICK = np.array([9.0, 9.0, 9.0, 8.0, 6.5, 5.5, 5.0, 5.0])  # 9 mm bridge = M4 half-lap grip; 5 mm cap
R_ARC = 32.0
CLEAR = 12.0   # inner face vs skin. Design rule elsewhere is 10 mm; 12 survives the carve.

def _chaikin(P, iters=3):
    P = np.asarray(P, float)
    for _ in range(iters):
        Q = [P[0]]
        for a, b in zip(P[:-1], P[1:]):
            Q.append(0.75 * a + 0.25 * b); Q.append(0.25 * a + 0.75 * b)
        Q.append(P[-1]); P = np.array(Q)
    return P

def _resample(P, n):
    d = np.linalg.norm(np.diff(P, axis=0), axis=1)
    u = np.r_[0.0, np.cumsum(d)]
    if u[-1] < 1e-6:
        return np.repeat(P[:1], n, axis=0)
    u /= u[-1]
    t = np.linspace(0, 1, n)
    return np.stack([np.interp(t, u, P[:, i]) for i in range(P.shape[1])], axis=1)

def _push(body, pts, clear):
    pts = np.asarray(pts, float).copy()
    clear = np.broadcast_to(np.asarray(clear, float), (len(pts),)).copy()
    for _ in range(8):
        cp, dist, fi = trimesh.proximity.closest_point(body, pts)
        n = body.face_normals[fi]
        sd = np.where(np.einsum("ij,ij->i", pts - cp, n) >= 0, dist, -dist)
        short = sd < clear - 0.2
        if not np.any(short):
            break
        pts[short] += n[short] * (clear[short] - sd[short])[:, None]
    return pts

def _frames(spine):
    t = np.gradient(spine, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-8)
    hint = np.array([0.0, 0.95, 0.30])
    out = np.zeros_like(spine)
    for i, ti in enumerate(t):
        o = hint - ti * (hint @ ti)
        out[i] = o / np.linalg.norm(o)
    for i in range(1, len(t)):
        o = out[i - 1] - t[i] * (out[i - 1] @ t[i])
        if np.linalg.norm(o) < 1e-6:
            continue
        o /= np.linalg.norm(o)
        if o @ out[i] < 0:
            o = -o
        out[i] = o
    side = np.cross(t, out)
    side /= np.maximum(np.linalg.norm(side, axis=1, keepdims=True), 1e-8)
    return t, out, side

def shell_surface(body, n_s=56, n_a=19):
    """Midsurface grid (n_s, n_a, 3) and per-vertex thickness. Index 0 of the arc is posterior."""
    crown = _resample(_chaikin(CROWN, 2), n_s)
    u = np.linspace(0, 1, len(CROWN))
    us = np.linspace(0, 1, n_s)
    ang_p = np.radians(np.interp(us, u, ANG_POS))
    ang_n = np.radians(np.interp(us, u, ANG_NEG))
    thick = np.interp(us, u, THICK)
    crown = _push(body, crown, CLEAR + thick / 2.0 + 0.5)  # crown is the outer-most arc point; keep inner face clear
    tang, out, side = _frames(crown)
    # arc parameter a in [-1, 1]: -1 posterior, +1 anterior, 0 = crown
    a = np.linspace(-1, 1, n_a)
    MID = np.zeros((n_s, n_a, 3))
    for i in range(n_s):
        aa = np.where(a >= 0, a * ang_p[i], a * ang_n[i])
        c = crown[i] - out[i] * R_ARC
        nrm = np.cos(aa)[:, None] * out[i] + np.sin(aa)[:, None] * side[i]
        MID[i] = c + R_ARC * nrm
    # pull any arc point that would enter the skin back out, along the body normal
    # Inner face sits CLEAR off the skin, so the midsurface is half a wall further out.
    need = (CLEAR + thick / 2.0)
    flat = _push(body, MID.reshape(-1, 3), np.repeat(need, n_a))
    MID = flat.reshape(n_s, n_a, 3)
    from scipy.ndimage import gaussian_filter
    MID = gaussian_filter(MID, sigma=(0.6, 0.45, 0), mode="nearest")
    flat = _push(body, MID.reshape(-1, 3), np.repeat(need, n_a))
    MID = flat.reshape(n_s, n_a, 3)
    return MID, thick, crown

def _grid_mesh(mid, thick):
    ns, na, _ = mid.shape
    # normals from the grid
    du = np.gradient(mid, axis=0); dv = np.gradient(mid, axis=1)
    nrm = np.cross(du, dv)
    nrm /= np.maximum(np.linalg.norm(nrm, axis=2, keepdims=True), 1e-8)
    # orient normals to the crown side (+y-ish)
    flip = nrm[:, na // 2, 1] < 0
    nrm[flip] *= -1
    th = thick[:, None, None]
    outer = mid + nrm * (th / 2.0)
    inner = mid - nrm * (th / 2.0)
    V = np.vstack([outer.reshape(-1, 3), inner.reshape(-1, 3)])
    o = lambda i, j: i * na + j
    inn = lambda i, j: ns * na + i * na + j
    def quad(a, b, c, d):
        return [[a, b, c], [a, c, d]]
    F = []
    for i in range(ns - 1):
        for j in range(na - 1):
            F += quad(o(i, j), o(i, j + 1), o(i + 1, j + 1), o(i + 1, j))
            F += quad(inn(i, j), inn(i + 1, j), inn(i + 1, j + 1), inn(i, j + 1))
    for i in range(ns - 1):
        F += quad(o(i, 0), o(i + 1, 0), inn(i + 1, 0), inn(i, 0))
        j = na - 1
        F += quad(o(i, j), inn(i, j), inn(i + 1, j), o(i + 1, j))
    for j in range(na - 1):
        F += quad(o(0, j), inn(0, j), inn(0, j + 1), o(0, j + 1))
        F += quad(o(ns - 1, j), o(ns - 1, j + 1), inn(ns - 1, j + 1), inn(ns - 1, j))
    m = trimesh.Trimesh(V, np.array(F), process=False)
    trimesh.repair.fix_normals(m, multibody=False)
    if m.volume < 0:
        m.invert()
    return m

def _tongue(body, mid, f4):
    """Curved shell from the medial rim up onto the acromion pad. Not a flat bar."""
    f4 = np.asarray(f4, float)
    # Start on the cap (forward of the housing), not on the socket root, so the
    # tongue and the acromion bolt stay on the arm piece past the half-lap splice.
    cloud = mid.reshape(-1, 3)
    cap = cloud[:, 0] < -120.0
    use = cloud[cap] if cap.any() else cloud
    a = use[np.argmin(np.linalg.norm(use - f4, axis=1))]
    # bow up off the shoulder, then land on the pad
    midp = 0.55 * a + 0.45 * f4 + np.array([0.0, 6.0, 14.0])
    ctrl = np.vstack([a, midp, f4])
    spine = _push(body, _resample(_chaikin(ctrl, 2), 18), CLEAR + 4.0)
    # keep the ends planted: first inside the shell, last on the pad
    spine[0] = a; spine[-1] = f4
    t, out, side = _frames(spine)
    n_a = 11
    ang = np.radians(np.linspace(28, 18, len(spine)))
    aa = np.linspace(-1, 1, n_a)
    MID = np.zeros((len(spine), n_a, 3))
    for i in range(len(spine)):
        th = aa * ang[i]
        nrm = np.cos(th)[:, None] * out[i] + np.sin(th)[:, None] * side[i]
        MID[i] = (spine[i] - out[i] * 22.0) + 22.0 * nrm
    # plant the last ring on the pad as a small disk so the bolt pad fuses
    last = f4 + (MID[-1] - MID[-1].mean(0)) * 0.35
    MID[-1] = last
    need = np.full(len(spine), CLEAR + 3.0)
    # don't push the planted ends off the shell / pad
    pushed = _push(body, MID[1:-1].reshape(-1, 3), np.repeat(need[1:-1], n_a))
    MID[1:-1] = pushed.reshape(len(spine) - 2, n_a, 3)
    thick = np.full(len(spine), 6.0)
    return _grid_mesh(MID, thick)

def deltoid_shell_mesh(body, f4_pad):
    """Curved shell that replaces the flat lateral shelf. Returns (mesh, half-lap waypoints)."""
    c = body.triangles_center
    sel = (c[:, 0] > -360) & (c[:, 0] < -40) & (c[:, 1] > -180) & (c[:, 1] < 160) & (c[:, 2] > -340) & (c[:, 2] < -20)
    body = body.submesh([np.where(sel)[0]], append=True)
    mid, thick, crown = shell_surface(body)
    mesh = _grid_mesh(mid, thick)
    tongue = _tongue(body, mid, f4_pad)
    try:
        fused = trimesh.boolean.union([mesh, tongue], engine="manifold")
        if fused.is_watertight and fused.volume > mesh.volume * 0.5:
            mesh = fused
    except Exception as e:
        print("union failed", e)
        mesh = trimesh.util.concatenate([mesh, tongue])
    # half-lap waypoints: inside the thick bridge, last point is the acromion pad
    idx = [int(round(f * (len(crown) - 1))) for f in (0.04, 0.18, 0.34, 0.52)]
    wpts = [crown[i].copy() for i in idx]
    if (not mesh.is_watertight) or mesh.volume <= 0:
        raise RuntimeError("deltoid shell did not fuse into a solid")
    return mesh, wpts



lo = np.array([-215.0, -125.0, -180.0]); hi = np.array([70.0, 125.0, -40.0])
n = np.ceil((hi - lo) / VOX).astype(int)
gx, gy, gz = [lo[i] + VOX * (np.arange(n[i]) + 0.5) for i in range(3)]
gx, gy, gz = gx.astype(np.float32), gy.astype(np.float32), gz.astype(np.float32)
X, Y, Z = np.meshgrid(gx, gy, gz, indexing="ij")
def win(pmin, pmax):
    """slices + local float32 coordinates of the grid window covering [pmin, pmax] (memory: never build full-grid point arrays)"""
    i0 = np.clip(np.floor((np.asarray(pmin) - lo) / VOX).astype(int), 0, n); i1 = np.clip(np.ceil((np.asarray(pmax) - lo) / VOX).astype(int) + 1, 0, n)
    sl = tuple(slice(a, b) for a, b in zip(i0, i1))
    return sl, X[sl], Y[sl], Z[sl]
def to_idx(p): return np.floor((p - lo) / VOX).astype(int)
def solid(m):
    return facet.voxelize_like(m, n, lo, VOX)

G = ak.groups()
stp = {k: trimesh.load(os.path.join(SH, "stops", k + ".stl")) for k in ("s1_post_a_R", "s1_post_b_R", "tpu_s1_a_R", "tpu_s1_b_R")}
CONT = {**G["F"], **stp}
occ = np.zeros(n, bool)
for k, m in CONT.items(): occ |= solid(m)
# ---- housing: closed contents -> offset shell
d_out = (ndimage.distance_transform_edt(~occ) * VOX).astype(np.float32)
closed = (ndimage.distance_transform_edt(d_out <= R_CLOSE) * VOX > R_CLOSE) | occ
D_raw = d_out; del d_out
D = ndimage.gaussian_filter((ndimage.distance_transform_edt(~closed) * VOX).astype(np.float32), SMOOTH / VOX)
housing = (D <= G_IN + T_WALL)

# ---- turning M1 parts: surface of revolution about z (max radius per z) -> keep-out for everything fixed
rxy = np.hypot(X, Y)
M1parts = [G["M1"][k] for k in ("yoke", "s2_clamp", "ak_s2")] + [trimesh.load(os.path.join(SH, "stops", "s1_lug_R.stl"))]
cp = os.path.join(SH, "cowl", "cowl_R.stl")
if os.path.exists(cp): M1parts.append(trimesh.load(cp))
zbins = np.arange(lo[2], hi[2] + VOX, VOX); rmax = np.full(len(zbins), -1.0)
for m in M1parts:
    p, _ = trimesh.sample.sample_surface(m, 60000); p = np.vstack([p, m.vertices])
    zi = np.clip(((p[:, 2] - lo[2]) / VOX).astype(int), 0, len(zbins) - 1); rr = np.hypot(p[:, 0], p[:, 1])
    np.maximum.at(rmax, zi, rr)
rmax = ndimage.maximum_filter1d(rmax, 5)                                 # +-2 mm in z
RM = rmax[np.clip(((Z - lo[2]) / VOX).astype(int), 0, len(zbins) - 1)]
m1_sweep_keep = (RM > 0) & (rxy < RM + PART_MIN)

# ---- struts (SDF capsule-ish ribbons) ------------------------------------------------------------------------------------
sites = json.load(open(os.path.join(SH, "clasp_sites.json")))
def node_pad(k, extra=9.5):
    s_ = sites[k]; p_, n_ = np.array(s_["p"]), np.array(s_["n"]); return p_ + n_ * (7.0 + 3.5 + 2.25) + n_ * extra, n_
F4_pad, F4_n = node_pad("F4_acromion"); F1_pad, F1_n = node_pad("F1_upper_trap")
_pf = json.load(open(os.path.join(SH, "plate.json")))["pauldron_feet"]      # set by chassis_axilla.py (holes are tapped there)
PLATE_FEET = [np.array([PLATE_REAR + 2.5, y_, z_]) for y_, z_ in _pf]
PATHS = {
    "foot_med": [np.array([-30.0, -50.0, -105.0]), PLATE_FEET[0]],
    "foot_lat": [np.array([-30.0, 50.0, -105.0]), PLATE_FEET[1]],
    "foot_mid": [np.array([0.0, 30.0, -100.0]), np.array([-20.0, 0.0, -118.0]), np.array([-45.0, 0.0, -135.0]), PLATE_FEET[2]],
    "arm_med": [np.array([-20.0, -81.0, -85.0]), np.array([-60.0, -98.0, -84.0]), np.array([-100.0, -98.0, -76.0]), np.array([-136.0, -74.0, -72.0]), F1_pad],
}
def ribbon_into(v, a, b, w, t, nref):
    e = b - a; L = np.linalg.norm(e); e = e / L
    nn = nref - e * (nref @ e); nn /= np.linalg.norm(nn); ww = np.cross(e, nn)
    pad = max(w, t); sl, x, y, z = win(np.minimum(a, b) - pad, np.maximum(a, b) + pad)
    px, py, pz = x - a[0], y - a[1], z - a[2]
    s = np.clip(px * e[0] + py * e[1] + pz * e[2], 0, L)
    dx, dy, dz = px - s * e[0], py - s * e[1], pz - s * e[2]
    v[sl] |= (np.abs(dx * ww[0] + dy * ww[1] + dz * ww[2]) <= w / 2) & (np.abs(dx * nn[0] + dy * nn[1] + dz * nn[2]) <= t / 2)
def ball_into(v, c, r):
    sl, x, y, z = win(c - r, c + r); v[sl] |= (x - c[0]) ** 2 + (y - c[1]) ** 2 + (z - c[2]) ** 2 <= r * r
def strut_vol(pts, w=ST_W, t=ST_T, up=np.array([0.0, 0.0, 1.0])):
    v = np.zeros(n, bool)
    for a, b in zip(pts[:-1], pts[1:]):
        ribbon_into(v, a, b, w, t, up); ball_into(v, b, t / 2 + 1.5)          # rounded joints at the waypoints
    return v
_wearer = trimesh.load(os.path.join(SH, "proxies", "wearer_makehuman.stl"))
_shell_m, _wpts = deltoid_shell_mesh(_wearer, F4_pad)
PATHS["arm_lat"] = [np.asarray(p, float) for p in _wpts] + [F4_pad]
del _wearer
struts = {k: strut_vol(p_) for k, p_ in PATHS.items() if k != "arm_lat"}
struts["arm_lat"] = facet.voxelize_like(_shell_m, n, lo, VOX)
struts["arm_lat"] &= ~((rxy < R_RIM) & (Z > Z_BORE))   # socket bore stays open so the cowl still mounts
print("deltoid shell voxels", int(struts["arm_lat"].sum()), "ymax", round(float(_shell_m.vertices[:, 1].max()), 1))
# end pads: plate feet 30 x 26 x 5 on the plate rear; clasp pads 24 x 24 x 5 tangent to the node
pads = np.zeros(n, bool); holes = np.zeros(n, bool)
for f in PLATE_FEET:
    pads |= (X >= PLATE_REAR + 0.4) & (X <= PLATE_REAR + 5.4) & (np.abs(Y - f[1]) <= 15) & (np.abs(Z - f[2]) <= 13)
    for dz in (-7.0, 7.0):
        holes |= (np.hypot(Y - f[1], Z - (f[2] + dz)) <= 2.25) & (X <= PLATE_REAR + 12)       # M4 clearance
for c_, n_ in ((F4_pad, F4_n), (F1_pad, F1_n)):
    u_ = np.cross(n_, [1.0, 0, 0]); u_ /= np.linalg.norm(u_); v_ = np.cross(n_, u_)
    sl, x, y, z = win(c_ - 25, c_ + 25); px, py, pz = x - c_[0], y - c_[1], z - c_[2]
    an = px * n_[0] + py * n_[1] + pz * n_[2]; au = px * u_[0] + py * u_[1] + pz * u_[2]; av = px * v_[0] + py * v_[1] + pz * v_[2]
    pads[sl] |= (np.abs(an) <= 2.5) & (np.abs(au) <= 12) & (np.abs(av) <= 12)
    holes[sl] |= (au * au + av * av <= 2.25 ** 2) & (np.abs(an) <= 20)
web = pads.copy()
for v in struts.values(): web |= v

# ---- faceted outer solid of housing + web, then the cut-outs ---------------------------------------------------------------
# Drop the lateral planar cap. The curved shell (in the web) is the surface that sits on the shoulder.
flat_shelf = housing & (Y > 52.0) & (Z > Z_RIM - 4.5) & (rxy > R_RIM + 8.0)
print("flat shelf voxels removed", int(flat_shelf.sum()))
housing = housing & ~flat_shelf
smooth_all = ndimage.binary_closing(housing | web, iterations=2)
grow = int(round(FACET_MARGIN / VOX))
# housing, struts, and the deltoid shell are faceted separately. The shell keeps enough faces
# that the arc is not collapsed back into a flat plate.
_web_slim = web.copy(); _web_slim[struts["arm_lat"]] = False
fac_h, fmesh_h = facet.faceted(ndimage.binary_dilation(ndimage.binary_closing(housing, iterations=2), iterations=grow), lo, VOX, N_FACETS_HOUSING)
fac_w, fmesh_w = facet.faceted(ndimage.binary_dilation(ndimage.binary_closing(_web_slim, iterations=1), iterations=grow), lo, VOX, N_FACETS)
fac_s, fmesh_s = facet.faceted(ndimage.binary_dilation(ndimage.binary_closing(struts["arm_lat"], iterations=1), iterations=grow), lo, VOX, N_FACETS_SHELL)
fac = fac_h | fac_w | fac_s; fmesh = trimesh.util.concatenate([fmesh_h, fmesh_w, fmesh_s])
cavity = ((D < G_IN) | (D_raw < G_IN)) & ~web | (D_raw < 2.0)               # struts may pass the stage but never closer than 2 mm
cavity |= (Z < Z_FLOOR) & ~web                                              # open bottom
_pl = trimesh.load(os.path.join(SH, "chassis_plate_R.stl")).bounds
cavity |= (X < PLATE_REAR + 0.4) & (X > _pl[0][0] - PART_MIN) & (Y > _pl[0][1] - PART_MIN) & (Y < _pl[1][1] + PART_MIN) & (Z > _pl[0][2] - PART_MIN) & (Z < _pl[1][2] + PART_MIN)   # plate + 3 mm
cavity |= (X < PLATE_REAR + 0.4) & housing & ~web                           # the housing itself stops at the plate rear face
cavity |= (Z > Z_RIM) & (rxy < 130)                                        # socket rim plane (the cowl turns above)
cavity |= (rxy < R_RIM) & (Z > Z_BORE)                                     # socket bore round the lug / yoke disc
cavity |= m1_sweep_keep
paul = fac & ~cavity & ~holes
# assembly draft: the core slides on along -x over the bare S1 base; then the AK70 drops in from above through the socket bore
# (Oe89 < bore Oe132), the S1 clamp slides up from below along the motor axis (open floor), then posts, plate, yoke. Remove every
# core voxel whose path in crosses the base (base material at larger x in the same (y, z) column), plus 2 mm
_asm = np.zeros(n, bool)
for k in ("s1_base",): _asm |= solid(G["F"][k])
_asm = ndimage.binary_dilation(_asm, iterations=2)
_sh = np.flip(np.maximum.accumulate(np.flip(_asm, 0), axis=0), 0) & ~_asm & (X > PLATE_REAR - 1.0)     # core zone only (the arms beyond the splices are fitted later)
print("core assembly-draft voxels removed:", int((paul & _sh).sum()))
paul &= ~_sh
paul &= (smooth_all | ndimage.binary_dilation(smooth_all, iterations=2))  # facets never add more than ~2 mm beyond the smooth web

# ---- faceted windows (lightening): every large flat facet of the housing gets an inset window, leaving a triangulated
#      frame of 9 mm ribs along the facet edges (a "faceted lattice"); struts, pads, the rim zone and the plate seat stay solid
from shapely.geometry import Polygon
from shapely.ops import unary_union
from matplotlib.path import Path as MPath
WIN_INSET, WIN_MIN_AREA = 6.5, 60.0
web_keep = ndimage.binary_dilation(web, iterations=4) | (Z > Z_RIM - 8.0) | (X < PLATE_REAR + 6.0)
n_win = 0; win_area = 0.0
_grouped = set(int(f) for g in fmesh_h.facets for f in g)
_groups = [np.asarray(g) for g in fmesh_h.facets] + [np.array([f]) for f in range(len(fmesh_h.faces)) if f not in _grouped]
for fg in _groups:
    A_ = fmesh_h.area_faces[fg].sum()
    if A_ < 350.0: continue
    nf = fmesh_h.face_normals[fg[0]]; o = fmesh_h.triangles[fg][:, 0].mean(0)
    u = np.cross(nf, [0, 0, 1.0]) if abs(nf[2]) < 0.9 else np.cross(nf, [1.0, 0, 0]); u /= np.linalg.norm(u); w = np.cross(nf, u)
    tris = fmesh_h.triangles[fg] - o
    poly = unary_union([Polygon(np.c_[t @ u, t @ w]) for t in tris]).buffer(0)
    ins = poly.buffer(-WIN_INSET)
    if ins.is_empty or ins.area < WIN_MIN_AREA: continue
    geoms = [ins] if ins.geom_type == "Polygon" else list(ins.geoms)
    pts3 = fmesh_h.triangles[fg].reshape(-1, 3); sl, x, y, z = win(pts3.min(0) - 8, pts3.max(0) + 8)
    px, py, pz = x - o[0], y - o[1], z - o[2]
    dn = px * nf[0] + py * nf[1] + pz * nf[2]
    slab = (dn <= 0.6) & (dn >= -(T_WALL + FACET_MARGIN + 3.0))
    if not slab.any(): continue
    q2 = np.c_[(px * u[0] + py * u[1] + pz * u[2])[slab], (px * w[0] + py * w[1] + pz * w[2])[slab]]
    inside = np.zeros(len(q2), bool)
    for g_ in geoms:
        if g_.area >= WIN_MIN_AREA: inside |= MPath(np.asarray(g_.exterior.coords)).contains_points(q2)
    cut = np.zeros(slab.shape, bool); cut[slab] = inside
    cut &= ~web_keep[sl]
    if cut.sum() > 50:
        paul[sl] &= ~cut; n_win += 1; win_area += sum(g_.area for g_ in geoms)
print("facet windows:", n_win, "total window area %.0f mm2" % win_area)
# ---- skin / part clearance carve (design rule: arms >= SKIN_MIN from skin except the last 25 mm at the clasp pads) -------
S = SH + "/"
body = trimesh.load(S + "proxies/wearer_makehuman.stl"); c = body.triangles_center
body = body.submesh([np.where((c[:, 2] > -260) & (c[:, 2] < -20) & (c[:, 0] > -300))[0]], append=True)
OBST = {"body": (body, SKIN_MIN), "clasp": (trimesh.load(S + "clasp_frame_R.stl"), PART_MIN), "wing": (trimesh.load(S + "scapular/wing_R.stl"), PART_MIN),
        "spine": (trimesh.load(S + "scapular/spine_bridge.stl"), PART_MIN)}
idx = np.argwhere(paul); Pp = lo + VOX * (idx + 0.5)
near_pad = np.zeros(len(Pp), bool)
for c_ in (F4_pad, F1_pad): near_pad |= np.linalg.norm(Pp - c_, axis=1) < 35.0     # the arms converge onto the clasp nodes here
report = {}
from scipy.spatial import cKDTree
for nm, (m, need) in OBST.items():
    # signed distance via a dense surface sample (spacing ~0.8 mm) + KD-tree; sign from the nearest sample's face normal.
    # (trimesh closest_point runs out of memory on this many query points)
    sd = np.full(len(Pp), 1e3)
    inb = np.all((Pp > m.bounds[0] - need - 5) & (Pp < m.bounds[1] + need + 5), axis=1)
    sp, fi = trimesh.sample.sample_surface_even(m, int(m.area / 0.6) + 1000)
    tree = cKDTree(sp); dd, ii = tree.query(Pp[inb], k=1)
    sg = np.einsum("ij,ij->i", Pp[inb] - sp[ii], m.face_normals[fi[ii]])
    sd[inb] = np.where(sg >= 0, dd, -dd) if nm == "body" else dd        # thin printed parts: unsigned (a sign from the nearest sample is unreliable)
    bad = (sd < need) & ~(near_pad & (nm == "clasp")) & ~(near_pad & (nm == "body") & (sd >= 2.5))
    report[nm] = dict(min_mm=round(float(sd.min()), 1), carved_vox=int(bad.sum()))
    paul[tuple(idx[bad].T)] = False
paul = ndimage.binary_opening(paul, iterations=1)
lab, nl = ndimage.label(paul, structure=np.ones((3, 3, 3))); sz = ndimage.sum(paul, lab, range(1, nl + 1)); paul = lab == 1 + int(np.argmax(sz))
print("obstacle carve:", report, "| pieces before keep-largest:", nl)

# ---- split for printing: half-lap splices on the two arms, just past the plate edges --------------------------------------
parts = {"core": paul.copy()}
SPL = {"arm_lat": (PATHS["arm_lat"][1], PATHS["arm_lat"][2]), "arm_med": (PATHS["arm_med"][1], PATHS["arm_med"][2])}
splices = {}
for k, (a, b) in SPL.items():
    t_ = (b - a) / np.linalg.norm(b - a); s0 = a + (b - a) * 0.35; m_ = np.array([0.0, 0.0, 1.0]) - t_ * t_[2]; m_ /= np.linalg.norm(m_)
    side = np.sign(s0[1])
    # everything of the core on this arm's side beyond the splice belongs to the arm: arm region = half-space along t_ past the
    # splice (restricted to this side of the body plane y = 0 and outside the housing), with a half-lap through the overlap zone
    sl, x, y, z = win(lo, hi)
    px, py, pz = x - s0[0], y - s0[1], z - s0[2]
    along = px * t_[0] + py * t_[1] + pz * t_[2]; across = px * m_[0] + py * m_[1] + pz * m_[2]
    region = (np.sign(y) == side) & (D > G_IN + T_WALL + 4.0)
    arm_side = region & ((along > 12.0) | ((np.abs(along) <= 12.0) & (across < -0.4)))
    gap = region & (np.abs(along) <= 12.0) & (np.abs(across) <= 0.4)
    armv = parts["core"] & arm_side
    parts["core"] &= ~arm_side & ~gap
    del px, py, pz, along, across, region, arm_side, gap
    for dd in (-6.0, 6.0):
        hc = s0 + t_ * dd; sl2, x2, y2, z2 = win(hc - 30, hc + 30); qx, qy, qz = x2 - hc[0], y2 - hc[1], z2 - hc[2]
        qa = qx * m_[0] + qy * m_[1] + qz * m_[2]
        hole = (qx - qa * m_[0]) ** 2 + (qy - qa * m_[1]) ** 2 + (qz - qa * m_[2]) ** 2 <= 2.25 ** 2
        armv[sl2] &= ~hole; parts["core"][sl2] &= ~hole
    parts[k] = armv
    splices[k] = dict(at=np.round(s0, 1).tolist(), axis=np.round(t_, 3).tolist(), bolt_axis=np.round(m_, 3).tolist())
for k in list(parts):
    lab, nl = ndimage.label(parts[k], structure=np.ones((3, 3, 3))); sz = ndimage.sum(parts[k], lab, range(1, nl + 1))
    parts[k] = lab == 1 + int(np.argmax(sz))

def mesh_of(vol):
    m = facet.to_mesh(vol, lo, VOX, smooth=6); m = facet.snap(m, fmesh, tol=1.0)
    try:
        d = m.simplify_quadric_decimation(face_count=min(len(m.faces), 40000))
        if d.is_watertight and d.volume > 0: m = d
    except Exception: pass
    return m
info = dict(parts={}, splices=splices, plate_feet=[f.round(1).tolist() for f in PLATE_FEET], clasp_pads=dict(F4=F4_pad.round(1).tolist(), F1=F1_pad.round(1).tolist()),
            obstacle_carve=report, params=dict(G_IN=G_IN, T_WALL=T_WALL, Z_RIM=Z_RIM, R_RIM=R_RIM, STRUT=[ST_W, ST_T], N_FACETS=N_FACETS, shell="deltoid_arc", shell_clear_mm=CLEAR, shell_arc_r_mm=R_ARC))
allm = []
for k, v in parts.items():
    m = mesh_of(v); m.export(os.path.join(OUT, f"pauldron_{k}_R.stl")); allm.append(m)
    info["parts"][k] = dict(extents=m.extents.round(1).tolist(), fits_256=bool(m.extents.max() <= 256), volume_cm3=round(m.volume / 1e3, 1),
                            mass_g_PACF=round(m.volume / 1e3 * 1.15), watertight=bool(m.is_watertight))
trimesh.util.concatenate(allm).export(os.path.join(OUT, "pauldron_R.stl"))
json.dump(info, open(os.path.join(OUT, "pauldron.json"), "w"), indent=1)
print(json.dumps(info, indent=1))
