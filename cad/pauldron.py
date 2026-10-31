"""Armoured pauldron, rev 2026-10-30 (Forest: "It shouldn't be a shelf, it should be more like a shoulder piece of a knight's
armor" ... "really make smart ergonomic choices that are also aesthetically cool").  FIXED part (group F), PA-CF annealed.

Why the old one read as a shelf: the S1 base is a 136 mm block cantilevered off the flat 6061 plate, and the rev 2026-10-13/23
pauldron was a housing round that block plus struts under it, i.e. a bracketed shelf. This revision keeps every interface (plate
feet 3 x 2 M4, clasp seats F1/F4 M4 x 12, socket rim / bore, assembly order) and re-forms the part as plate armour:

  CROWN  (pauldron_crown_R)  the "cop": a domed brim round the turning socket (rolled bezel at the bore, 7 mm quadratic fall),
         then two flared, bulged, stepped lames wrapping the S1 stage, each ending in a rolled hem. U-shaped, open toward the plate,
         so it slides on along -x over the bare S1 base exactly like the old core (the AK70 drops in through the bore afterwards).
         Two ears beside the plate side edges carry the spaulder (2 x M3 heat-set inserts each).
  KEEL   (pauldron_keel_R)   two stepped, fluted lames under the S1 motor that taper down into the plate (teardrop side profile).
         It replaces the three plate struts: a 9 x 3 flange on the plate rear face with D13 x 6 bosses on the six old foot holes,
         M4 heat-set inserts (5.6 x 4) pressed from the plate face, ISO 10642 M4 x 12 countersunk screws from the plate FRONT
         (chassis_axilla.py now makes those holes 4.5 clearance + csk D9). Its lip nests 0.6-1.4 mm inside the crown's lower hem:
         4 x M3 x 8 button heads through the hem into radial heat-set inserts. Offered up from below (+z) before the plate bolting.
  SPAULDER (pauldron_spaulder_R) three sculpted lames that run forward from the crown over the shoulder top and land on the clasp
         seats F1 (upper trapezius) and F4 (acromion): stable zones only (standing decision). Fore-aft crest, lames stepped 3 mm
         (rear lame overlaps the next, like real lames), rolled hems, outer edges rolled and curled DOWN, rear edge scalloped round
         the socket, front cut back along the wearer's own arm / neck motion envelope (no haute-piece: nothing stands up beside the
         neck). Bolted to the crown ears (2 x M3 x 10 each side) and to the two clasp seats (D24 round pads on the D24 clasp bosses,
         M4 x 12 through a counterbore; contract unchanged).

Ergonomic rules built into the generator (all numbers are verified by verify_armor.py):
  - every surface is placed from clearance fields computed from the meshes, so a re-fit to Forest's mesh is a re-run;
  - skin >= 10 mm under the spaulder, >= 3 mm to every other part, >= 3.5 mm (+1 mm sampling margin) to everything that turns
    with S1 over the HARD range (5..150), and >= 4.5 mm below the M2/M3 arm sweep over the soft box;
  - wearer range of motion: the spaulder outline is cut back (5 mm) along the envelope of Forest's arm (abduction / scaption /
    flexion to 110 deg GH = full elevation at GH:ST 1.7:1, McClure 2001) and head-neck (lateral flexion 50, rotation 90,
    flexion / extension 70), seats exempt; lateral edge stops at the acromion line, medial edge curls away from the neck;
  - mass budget: no heavier than the harness model's 182 g pauldron (top-pad margin is ~1 %).
Lames are sculpted, not articulated: nothing under them moves relative to anything else, so articulation would only add rattle
and pinch points.

Outputs stl/shoulder/pauldron/: pauldron_{crown,keel,spaulder}_R.stl (+ _L mirrors), pauldron_R.stl (all, for sweeps/renders),
pauldron.json. The rev 2026-10-23 parts (core, arm_lat, arm_med) are moved to stl/archive/pauldron_rev2026-10-23/.
Right arm, robot frame (+x away from the back, +y wearer's right, +z up), mm."""
import os, sys, json, glob, shutil, time, numpy as np, trimesh
from scipy import ndimage
from skimage import measure
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak
ST = os.path.join(HERE, "..", "stl"); SH = os.path.join(ST, "shoulder"); OUT = os.path.join(SH, "pauldron")
t0 = time.time()
def log(*a): print("[%5.0fs]" % (time.time() - t0), *a, flush=True)

VOX = 0.5                       # voxel (mm)
T = 1.65                        # keel wall (4 perimeters at 0.4 mm + smoothing allowance; the 3-6 mm flange and bosses carry the bolt loads)
T_C, T_S = 1.6, 1.6              # crown and spaulder walls (covers; 4 perimeters at 0.4 mm)
BEAD = 1.2                      # rolled-hem radius
PACF = 1.15                     # g/cm3, PA-CF (same density the 2026-10-13 pauldron used)
G_IN, M1_CLR, ARM_CLR, PART_CLR, SKIN_CLR, FIX_CLR = 3.5, 3.5 + 1.0, 3.5 + 1.0, 3.0 + 0.5, 10.0 + 0.5, 1.0   # +0.5 voxel/sampling margin on the 3 / 10 mm gates
R_BORE_MIN = 66.0               # socket bore (old R_RIM); grown locally if the S1 sweep needs it
Z_RIM = -54.0                   # cop top at the bore (old Z_RIM; cowl bottom -50, M1 bumpers -49.5)
COP_H = 7.0                     # the cop falls 7 mm from the bezel to the brim edge on a quadratic (domed) profile
BULGE = 2.0                     # each lame bulges 2 mm outward at mid-height on top of its 3 mm flare
A_R, A_F, B_W, N_SE = 76.0, 120.0, 83.0, 3.0         # crown plan superellipse: rear / front semi-axes, half width, exponent
X_CUT = -77.0                   # crown and keel stop 3 mm short of the plate rear face (x -80)
BANDS = [(-57.0, -79.0), (-77.0, -100.0)]               # crown lames; the keel continues the cascade below -100            # crown lames (z top, z hem); 2 mm riser overlap
FLARE = 3.0                     # each lame flares 3 mm outward toward its hem
KEEL_C, KEEL_PX, KEEL_PY, KEEL_OFF = 60.0, 1.2, 2.6, 1.6
KSTEP_W, KSTEP = 0.45, 3.0      # the keel is two lames: the upper one flares 3 mm by w = 0.45, the lower one restarts inside it   # keel depth below the hem; taper exponents rear (x) / width (y); lip offset
FEET = json.load(open(os.path.join(SH, "plate.json")))["pauldron_feet"]     # [(y, z)] M4 pairs at z +-7 (chassis_axilla.py: clearance + csk)
PLATE_REAR = -80.0

# ------------------------------------------------------------------------------------------------ obstacles
L = lambda p: trimesh.load(os.path.join(ST, p + ".stl"))
G = ak.groups(include_fingers=False); ak.install_balancer(G)
stops = {k: L("shoulder/stops/" + k) for k in ("s1_post_a_R", "s1_post_b_R", "tpu_s1_a_R", "tpu_s1_b_R")}
F_CONT = {**G["F"], **stops}
body = L("shoulder/proxies/wearer_makehuman")
plate = L("shoulder/chassis_plate_R")
clasp = trimesh.util.concatenate([L("shoulder/clasp_frame_R")] + [trimesh.load(p) for p in glob.glob(os.path.join(SH, "skirts", "*_R.stl"))])
wing, spine = L("shoulder/scapular/wing_R"), L("shoulder/scapular/spine_bridge")
sites = json.load(open(os.path.join(SH, "clasp_sites.json")))
def pad_of(k):
    p, n = np.array(sites[k]["p"]), np.array(sites[k]["n"]); n = n / np.linalg.norm(n)
    return p + n * (7.0 + 3.5 + 2.25 + 9.5), n            # same seat as the 2026-10-13 pauldron (contract unchanged)
PADS = {"F4": pad_of("F4_acromion"), "F1": pad_of("F1_upper_trap")}

def surf_pts(m, spacing=0.8):
    p, _ = trimesh.sample.sample_surface_even(m, int(m.area / spacing ** 2) + 200); return np.vstack([p, m.vertices])

# S1-turning parts (M1 + balancer M1), swept over the HARD range: per (z bin, azimuth bin) max radius
m1_parts = [G["M1"][k] for k in G["M1"]] + [L("shoulder/stops/s1_lug_R")]
for nm, (m, grp) in ak.balancer_parts().items():
    if grp == "M1": m1_parts.append(m)
P1 = np.vstack([surf_pts(m, 1.0) for m in m1_parts]); P1 = P1[P1[:, 2] < -20.0]
ZB0, NZB, NAB = -130.0, 260, 720                       # z bins 0.5 mm from -130, azimuth bins 0.5 deg
Rsw = np.zeros((NZB, NAB))
r1, a1, zi1 = np.hypot(P1[:, 0], P1[:, 1]), np.degrees(np.arctan2(P1[:, 1], P1[:, 0])), np.clip(((P1[:, 2] - ZB0) / 0.5).astype(int), 0, NZB - 1)
for th in np.arange(5.0, 150.01, 0.5):                 # R = Rz(-theta): azimuth decreases by theta
    ai = (((a1 - th) % 360.0) / 0.5).astype(int) % NAB
    np.maximum.at(Rsw, (zi1, ai), r1)
Rsw = ndimage.maximum_filter(Rsw, size=(2 * int(M1_CLR / 0.5) + 1, 9), mode=("nearest", "wrap"))   # +-clearance in z, +-2 deg
log("S1 sweep: %d points, max swept radius below z -40: %.1f mm" % (len(P1), Rsw[: int((-40 - ZB0) / 0.5)].max()))
def m1_keepout(X, Y, Z):
    zi = np.clip(((Z - ZB0) / 0.5).astype(int), 0, NZB - 1); ai = ((np.degrees(np.arctan2(Y, X)) % 360.0) / 0.5).astype(int) % NAB
    R = Rsw[zi, ai]; return (R > 0) & (np.hypot(X, Y) < R + M1_CLR)

# M2/M3 arm sweep over the SOFT box (+ balancer M2 parts): per (x, y) column, the lowest arm point -> ceiling for the spaulder
def arm_floor(lo2, n2, res):
    """lowest z of M2/M3 parts per 2D column over the soft box (S1 step 5, S2 step 3, S3 -70/10/90), x < -50 and z < 0 only"""
    pts = {}
    for g in ("M2", "M3"):
        for k, m in G[g].items(): pts[(g, k)] = surf_pts(m, 2.0)
    bal = {nm: (surf_pts(m, 2.0), grp) for nm, (m, grp) in ak.balancer_parts().items() if grp == "M2"}
    zf = np.full(n2, np.inf)
    Pm = P1[P1[:, 2] < -30.0]                                  # S1-turning parts too (hard range): they also pass over the rear edge
    for th in np.arange(5.0, 150.01, 1.0):
        Rz = trimesh.transformations.rotation_matrix(np.radians(-th), [0, 0, 1])[:3, :3]; p = Pm @ Rz.T; p = p[p[:, 0] < -50.0]
        ij = np.floor((p[:, :2] - lo2) / res).astype(int); ok = np.all((ij >= 0) & (ij < n2), axis=1)
        np.minimum.at(zf, (ij[ok, 0], ij[ok, 1]), p[ok, 2])
    s1s = np.arange(10.0, 140.01, 5.0); s2s = np.arange(-30.0, 88.01, 3.0); s3s = (-70.0, 10.0, 90.0)
    for s1 in s1s:
        for s2 in s2s:
            for s3 in s3s:
                q = dict(S1=s1, S2=s2, S3=s3, E=0, W1=0, W2=0, W3=0, G=50); Fr = ak.frames(q)
                chunks = []
                for (g, k), p in pts.items():
                    if g == "M2" and s3 != s3s[0]: continue        # M2 does not turn with S3
                    Tm = Fr[g]; chunks.append(p @ Tm[:3, :3].T + Tm[:3, 3])
                if s3 == s3s[0]:
                    BF = ak.balancer_frames(q)
                    for nm, (p, grp) in bal.items():
                        if nm in BF: Tm = BF[nm]; chunks.append(p @ Tm[:3, :3].T + Tm[:3, 3])
                p = np.vstack(chunks); p = p[(p[:, 0] < -50.0) & (p[:, 2] < 0.0)]
                if not len(p): continue
                ij = np.floor((p[:, :2] - lo2) / res).astype(int); ok = np.all((ij >= 0) & (ij < n2), axis=1)
                np.minimum.at(zf, (ij[ok, 0], ij[ok, 1]), p[ok, 2])
    return zf

def shift2(a, di, dj, fill):
    """a shifted by (di, dj) cells without wrap-around (np.roll would wrap the far edge in)"""
    o = np.full_like(a, fill); H, W = a.shape
    o[max(di, 0):H + min(di, 0), max(dj, 0):W + min(dj, 0)] = a[max(-di, 0):H + min(-di, 0), max(-dj, 0):W + min(-dj, 0)]
    return o
def top_field(meshes_clr, lo2, n2, res, exempt=None):
    """per 2D column: highest obstacle point grown by a spherical clearance (exact for obstacles seen from above)"""
    zt = np.full(n2, -np.inf)
    for m, clr in meshes_clr:
        p = surf_pts(m, 0.6)
        if exempt is not None: p = p[~exempt(p)]
        ij = np.floor((p[:, :2] - lo2) / res).astype(int); ok = np.all((ij >= 0) & (ij < n2), axis=1)
        h = np.full(n2, -np.inf); np.maximum.at(h, (ij[ok, 0], ij[ok, 1]), p[ok, 2])
        k = int(np.ceil(clr / res)); g = np.full(n2, -np.inf)
        for di in range(-k, k + 1):
            for dj in range(-k, k + 1):
                d = np.hypot(di, dj) * res
                if d > clr: continue
                g = np.maximum(g, shift2(h, di, dj, -np.inf) + np.sqrt(clr * clr - d * d))
        zt = np.maximum(zt, g)
    return zt

# ------------------------------------------------------------------------------------------------ helpers
def grid(lo, hi):
    lo = np.asarray(lo, float); n = np.ceil((np.asarray(hi) - lo) / VOX).astype(int)
    ax = [(lo[i] + VOX * (np.arange(n[i]) + 0.5)).astype(np.float32) for i in range(3)]
    return lo, n, ax
def voxelize(m, lo, n):
    """exact solid voxelization by ray parity along z (one ray per column)"""
    ys = lo[1] + VOX * (np.arange(n[1]) + 0.5); xs = lo[0] + VOX * (np.arange(n[0]) + 0.5)
    XX, YY = np.meshgrid(xs, ys, indexing="ij"); o = np.c_[XX.ravel(), YY.ravel(), np.full(XX.size, lo[2] - 5.0)]
    occ = np.zeros(n, bool)
    loc, ri, _ = m.ray.intersects_location(o, np.tile([0, 0, 1.0], (len(o), 1)), multiple_hits=True)
    if not len(loc): return occ
    order = np.lexsort((loc[:, 2], ri)); loc, ri = loc[order], ri[order]
    zs = lo[2] + VOX * (np.arange(n[2]) + 0.5)
    for r in np.unique(ri):
        zz = loc[ri == r, 2]; i, j = divmod(int(r), n[1])
        for a, b in zip(zz[0::2], zz[1::2]): occ[i, j, (zs >= a) & (zs <= b)] = True
    return occ
def mesh_of(solid, lo, smooth=1.2):
    s = np.pad(solid, 2); lo2 = lo - 2 * VOX
    sdf = (ndimage.distance_transform_edt(~s) - ndimage.distance_transform_edt(s)).astype(np.float32) * VOX
    if smooth: sdf = ndimage.gaussian_filter(sdf, smooth)
    v, f, _, _ = measure.marching_cubes(sdf, 0.0, spacing=(VOX,) * 3)
    m = trimesh.Trimesh(v + lo2 + VOX / 2, f[:, ::-1], process=True); m.fix_normals()
    return m
def cyl(p0, p1, r, sections=40):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float); v = p1 - p0; L_ = np.linalg.norm(v)
    c = trimesh.creation.cylinder(radius=r, height=L_, sections=sections)
    z = np.array([0, 0, 1.0]); u = v / L_; ax = np.cross(z, u); s = np.linalg.norm(ax); M = np.eye(4)
    if s > 1e-9: M = trimesh.transformations.rotation_matrix(np.arctan2(s, z @ u), ax / s)
    elif u[2] < 0: M = trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0])
    M[:3, 3] = (p0 + p1) / 2; c.apply_transform(M); return c
def boolean(a, cutters=(), adders=()):
    import manifold3d as mf
    def toM(m): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
    A = toM(a)
    for c in adders: A = A + toM(c)
    for c in cutters: A = A - toM(c)
    o = A.to_mesh(); return trimesh.Trimesh(o.vert_properties[:, :3], o.tri_verts, process=False)
def finish(m, tol=0.06):
    """manifold3d simplify (keeps the mesh a closed manifold; deviation <= tol mm)"""
    import manifold3d as mf
    m = trimesh.Trimesh(np.asarray(m.vertices, np.float32).astype(float), m.faces, process=True)   # float32 round + merge (as an STL round trip)
    M0 = mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))
    for t_ in (tol, tol / 2, tol / 4):
        o = M0.simplify(t_).to_mesh(); d = trimesh.Trimesh(o.vert_properties[:, :3], o.tri_verts, process=False)   # keep manifold3d's topology
        if d.is_watertight and d.volume > 0: m = d; break
    else: log("  simplify gave a non-watertight mesh; kept the full mesh")
    m.fix_normals(); return m

# crown plan outline: 2D signed distance of the superellipse (positive outside), 0.25 mm raster
R2 = 0.25; lo2d = np.array([-140.0, -115.0]); n2d = np.array([int(260 / R2), int(230 / R2)])
x2 = lo2d[0] + R2 * (np.arange(n2d[0]) + 0.5); y2 = lo2d[1] + R2 * (np.arange(n2d[1]) + 0.5); X2, Y2 = np.meshgrid(x2, y2, indexing="ij")
inside2 = (np.abs(X2) / np.where(X2 >= 0, A_R, A_F)) ** N_SE + (np.abs(Y2) / B_W) ** N_SE <= 1.0
D2r = ((ndimage.distance_transform_edt(~inside2) - ndimage.distance_transform_edt(inside2)) * R2).astype(np.float32)
def D2(X, Y):
    i = (X - lo2d[0]) / R2 - 0.5; j = (Y - lo2d[1]) / R2 - 0.5
    return ndimage.map_coordinates(D2r, [np.ravel(i), np.ravel(j)], order=1, mode="nearest").reshape(np.shape(X))

def flare(Z, ztop, zhem):
    t = np.clip((ztop - Z) / (ztop - zhem), 0.0, 1.0); return FLARE * t + BULGE * np.sin(np.pi * t)
def wall_k(Z, ztop, zhem):
    """horizontal wall width that gives a normal thickness of 1 on the flared / bulged lame (1 / cos of its slope)"""
    t = np.clip((ztop - Z) / (ztop - zhem), 0.0, 1.0); return np.sqrt(1.0 + ((FLARE + BULGE * np.pi * np.cos(np.pi * t)) / (ztop - zhem)) ** 2)
def rho(phi):
    """crown outline radius (superellipse) at azimuth phi (rad)"""
    c, s_ = np.abs(np.cos(phi)), np.abs(np.sin(phi)); a = np.where(np.cos(phi) >= 0, A_R, A_F)
    return 1.0 / ((c / a) ** N_SE + (s_ / B_W) ** N_SE) ** (1.0 / N_SE)

# ---- SDF toolkit (negative inside). Shells are built as true distance fields (unions / intersections of analytic primitives),
#      so surfaces are clean and wall thickness is exact; voxel booleans are used only to carve hidden keep-out faces.
U, I = np.minimum, np.maximum
def mask_sdf(mask):
    return ((ndimage.distance_transform_edt(~mask) - ndimage.distance_transform_edt(mask)) * VOX).astype(np.float32)
def keep_component(sdf):
    s_ = sdf < 0; lab, nl = ndimage.label(s_, structure=np.ones((3, 3, 3)))
    if nl > 1:
        sz = ndimage.sum(s_, lab, range(1, nl + 1)); k = 1 + int(np.argmax(sz)); sdf = np.where(s_ & (lab != k), np.float32(VOX), sdf)
        log("  dropped %d loose pieces (%d voxels)" % (nl - 1, int(sz.sum() - sz.max())))
    return sdf
def mesh_sdf(sdf, lo, smooth=0.5):
    s_ = np.pad(sdf.astype(np.float32), 2, constant_values=10.0)
    if smooth: s_ = ndimage.gaussian_filter(s_, smooth)
    v, f, _, _ = measure.marching_cubes(s_, 0.0, spacing=(VOX,) * 3)
    m = trimesh.Trimesh(v + lo - 2 * VOX + VOX / 2, f[:, ::-1], process=True); m.fix_normals()
    return finish(m, 0.04)                                  # simplify the clean marching-cubes manifold before any boolean
def box_sdf(X, Y, Z, lo3, hi3):
    return I(I(I(lo3[0] - X, X - hi3[0]), I(lo3[1] - Y, Y - hi3[1])), I(lo3[2] - Z, Z - hi3[2]))

# ================================================================================================ CROWN
log("crown")
lo, n, (gx, gy, gz) = grid([-100.0, -100.0, -120.0], [96.0, 100.0, -48.0])
Xc, Yc = np.meshgrid(gx, gy, indexing="ij"); d2c = D2(Xc, Yc)[:, :, None]; rc = np.hypot(Xc, Yc)[:, :, None].astype(np.float32)
X, Y, Z = Xc[:, :, None], Yc[:, :, None], gz[None, None, :]
tc = np.clip((rc - R_BORE_MIN) / np.maximum(rho(np.arctan2(Yc, Xc))[:, :, None] - R_BORE_MIN, 1.0), 0.0, 1.2).astype(np.float32)
ztop = (Z_RIM - COP_H * tc ** 2).astype(np.float32)
cr = np.full(n, 1e3, np.float32)
for (zt, zh) in BANDS:
    fl = flare(Z, zt, zh); wk = T_C * wall_k(Z, zt, zh)
    cr = U(cr, I(I(fl - d2c, d2c - fl - wk), I(Z - np.minimum(zt, ztop + 0.6), zh - Z)))
    cr = U(cr, np.hypot(d2c - (FLARE + T_C - 0.2), Z - (zh + BEAD - 0.3)) - BEAD)                 # rolled hem
for (zt_, zh_) in BANDS[:-1]:
    cr = U(cr, I(I(zh_ - Z, Z - (zh_ + T_C)), I(-d2c, d2c - (FLARE + T_C))))                      # riser under each lame
copk = T_C * np.sqrt(1 + (2 * COP_H * tc / 12.0) ** 2)
cr = U(cr, I(I(R_BORE_MIN - rc, d2c - T_C), I(Z - ztop, (ztop - copk) - Z)))                       # domed cop
cr = U(cr, np.hypot(rc - (R_BORE_MIN + 1.5), Z - (Z_RIM - 1.5)) - 1.5)                             # rolled bezel at the bore
cr = I(cr, U(R_BORE_MIN - rc, Z - (Z_RIM - 9.0)))                                                   # socket bore stays open
cr = I(cr, X_CUT - X)                                                                               # 3 mm short of the plate
del ztop, tc, copk
EAR_Y, EAR_X, EAR_Z = (81.5, 93.0), (-95.0, X_CUT + 1.0), (-64.0, -57.0)
EAR_YIN = {1: plate.bounds[1][1] + 3.5, -1: -plate.bounds[0][1] + 3.5}          # 3.5 mm off the plate side edges
for s_ in (1, -1):                                                                 # ears run 5 mm back over the leg so they are part of it
    cr = U(cr, I(box_sdf(X, s_ * Y, Z, (EAR_X[0], EAR_YIN[s_], EAR_Z[0]), (X_CUT + 1.0, EAR_Y[1], EAR_Z[1])), -0.5 - d2c))
    cr = U(cr, I(box_sdf(X, s_ * Y, Z, (X_CUT - 0.5, 77.0, EAR_Z[0]), (X_CUT + 5.0, EAR_Y[1], EAR_Z[1])), -0.5 - d2c))   # root over the leg (behind the plate)
# keep-outs: stage contents + G_IN, the S1 sweep (hard range), the plate + 3 mm, and the assembly draft over the bare S1 base
occ = np.zeros(n, bool)
for m in F_CONT.values(): occ |= voxelize(m, lo, n)
cr = I(cr, (G_IN - ndimage.distance_transform_edt(~occ) * VOX).astype(np.float32)); del occ
K = m1_keepout(np.broadcast_to(X, n), np.broadcast_to(Y, n), np.broadcast_to(Z, n)); cr = I(cr, -mask_sdf(K)); del K
pb = plate.bounds; cr = I(cr, -box_sdf(X, Y, Z, pb[0] - PART_CLR, pb[1] + PART_CLR))
base = ndimage.binary_dilation(voxelize(G["F"]["s1_base"], lo, n), iterations=2)
shadow = np.flip(np.maximum.accumulate(np.flip(base, 0), axis=0), 0) & ~base
log("crown draft voxels removed:", int(((cr < 0) & shadow).sum())); cr = I(cr, -mask_sdf(shadow)); del base, shadow
cr = keep_component(cr); crown_m = mesh_sdf(cr, lo); del cr
# fastener features (exact): ears 2 x M3 heat-set (4.6 x 5.7) from the top; hem 4 x M3 clearance 3.4 for the keel screws
JOIN_PHI = (35.0, -35.0, 100.0, -100.0); Z_JOIN = -96.0
def hem_point(phi, off):
    """point at azimuth phi on the crown outline offset by off (mm), z Z_JOIN"""
    a = np.radians(phi); rr = np.linspace(40, 140, 2001); xs, ys = rr * np.cos(a), rr * np.sin(a)
    d = D2(xs, ys) - off; k = np.argmax(d >= 0); return np.array([xs[k], ys[k], Z_JOIN]), np.array([np.cos(a), np.sin(a), 0.0])
cut = []
EAR_HOLES = [(xh, s * 87.25) for s in (1, -1) for xh in (-81.5, -90.5)]
for xh, yh in EAR_HOLES: cut.append(cyl([xh, yh, EAR_Z[1] - 5.7], [xh, yh, EAR_Z[1] + 1], 2.3))
for ph in JOIN_PHI:
    p, u = hem_point(ph, float(flare(np.array(Z_JOIN), *BANDS[1])))
    cut.append(cyl(p - u * 1.0, p + u * 8.0, 1.7))
crown_m = finish(boolean(crown_m, cutters=cut))
log("crown", crown_m.is_watertight, round(crown_m.volume / 1e3, 1), "cm3")

# ================================================================================================ KEEL
log("keel")
lo, n, (gx, gy, gz) = grid([-82.0, -100.0, -172.0], [96.0, 100.0, -84.0])
Xk, Yk = np.meshgrid(gx, gy, indexing="ij"); X, Y, Z = Xk[:, :, None], Yk[:, :, None], gz[None, None, :]
w = np.clip((BANDS[1][1] - Z) / KEEL_C, 0.0, 1.0)
sig = np.maximum((1.0 - w ** KEEL_PX) ** (1.0 / KEEL_PX), 0.02); sgy = np.maximum((1.0 - w ** KEEL_PY) ** (1.0 / KEEL_PY), 0.02)
ZLIP = BANDS[1][1] + 6.0
V = np.zeros(n, bool)
for k in range(n[2]):                                    # slice by slice: crown outline (offset off_k) scaled about (-77, 0)
    zk = float(gz[k]); s_, sy_ = float(sig[0, 0, k]), float(sgy[0, 0, k]); wk = float(w[0, 0, k])
    if zk < BANDS[1][1] - KEEL_C: continue
    off_k = KEEL_OFF + (KSTEP * wk / KSTEP_W if wk < KSTEP_W else 0.0)
    V[:, :, k] = D2(X_CUT + (Xk - X_CUT) / s_, Yk / sy_) <= off_k
g3 = ndimage.gaussian_filter(mask_sdf(V), 1.0)          # 3-D signed distance of the keel volume (smoothed one voxel)
phi0 = np.degrees(np.arctan2(Y / sgy, (X_CUT + (X - X_CUT) / sig)))
rib_h = np.zeros(np.broadcast_shapes(phi0.shape, n), np.float32)
for pc, h, hw in ((0.0, 1.8, 3.0), (45.0, 1.2, 2.4), (-45.0, 1.2, 2.4)):
    rib_h = np.maximum(rib_h, h * np.clip(1.0 - np.abs(phi0 - pc) / hw, 0.0, 1.0) ** 0.5)
rib_h = np.where((Z < BANDS[1][1] - 3.0) & (Z > BANDS[1][1] - KEEL_C + 6.0), rib_h, 0.0).astype(np.float32)
ke = I(-g3 - T, g3 - rib_h)                               # shell inside the volume + flutes / crest outside it
ke = I(ke, I(X_CUT - X, Z - ZLIP))
del g3, phi0, rib_h, w, sig, sgy
# plate flange (3 thick, 9 wide) + D13 x 6 insert bosses at the 6 plate holes (ISO 10642 M4 x 12 from the plate front)
ix0 = int(round((X_CUT - lo[0]) / VOX - 0.5)); sec_ = V[ix0]
flange2 = sec_ & (ndimage.distance_transform_edt(sec_) * VOX <= 9.0) & (gz[None, :] <= BANDS[1][1] - 2.0)   # below the crown hem only
pads2 = np.zeros_like(sec_); holes_ft = []
for (yf, zf) in FEET:
    for dz in (-7.0, 7.0):
        holes_ft.append((yf, zf + dz)); pads2 |= np.hypot(Yk[0][:, None] - yf, gz[None, :] - (zf + dz)) <= 6.5
    pads2 |= (np.abs(Yk[0][:, None] - yf) <= 4.0) & (np.abs(gz[None, :] - zf) <= 7.0)
fl3 = (flange2[None, :, :] & (X >= PLATE_REAR) & (X <= PLATE_REAR + 3.0)) | (pads2[None, :, :] & (X >= PLATE_REAR) & (X <= PLATE_REAR + 6.0))
ke = U(ke, mask_sdf(fl3)); del fl3, V
for ph in JOIN_PHI:                                      # radial insert bosses inside the lip for the crown screws
    p, u = hem_point(ph, KEEL_OFF)
    q = np.stack(np.broadcast_arrays(X - p[0], Y - p[1], Z - p[2]), -1).astype(np.float32); a_ = q @ u.astype(np.float32)
    rr_ = np.linalg.norm(q - a_[..., None] * u.astype(np.float32), axis=-1); ke = U(ke, I(rr_ - 4.25, I(a_, -7.5 - a_))); del q, a_, rr_
occ = np.zeros(n, bool)
for m in F_CONT.values(): occ |= voxelize(m, lo, n)
ke = I(ke, (G_IN - ndimage.distance_transform_edt(~occ) * VOX).astype(np.float32)); del occ
ke = keep_component(ke); keel_m = mesh_sdf(ke, lo); del ke
cut = []
for (yt, zt) in holes_ft:                               # M4 heat-set insert 5.6 x 4 (+0.5) from the plate face
    cut.append(cyl([PLATE_REAR - 1.0, yt, zt], [PLATE_REAR + 4.5, yt, zt], 2.8))
for ph in JOIN_PHI:
    p, u = hem_point(ph, KEEL_OFF); cut.append(cyl(p - u * 5.7, p + u * 1.0, 2.3))
# S1 bundle port (PLACEHOLDER until the AK70-10 cable exit is measured): medial side, 22 x 12 mm slot along y
port = trimesh.creation.capsule(height=10.0, radius=6.0, count=[16, 16]); port.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [1, 0, 0]))
port.apply_scale([1.0, 6.0, 1.0]); port.apply_translation([-20.0, -70.0, -128.0]); cut.append(port)
keel_m = finish(boolean(keel_m, cutters=cut))
log("keel", keel_m.is_watertight, round(keel_m.volume / 1e3, 1), "cm3")

# ================================================================================================ SPAULDER
log("spaulder")
X_R = -85.0                                                     # rear edge (over the plate top; r >= 84 clears the S1 sweep)
lo, n, (gx, gy, gz) = grid([-218.0, -102.0, -108.0], [-76.0, 102.0, -38.0])
lo2 = lo[:2]; n2 = tuple(n[:2]); Xs, Ys = np.meshgrid(gx, gy, indexing="ij")
poly = np.array([[X_R, -86.0], [X_R, 86.0], [-110.0, 70.0], [-160.0, 48.0], [-196.0, 38.0], [-210.0, 22.0], [-210.0, -24.0],
                 [-200.0, -58.0], [-184.0, -74.0], [-150.0, -80.0], [-108.0, -84.0]])
from matplotlib.path import Path as MPath
plan = MPath(poly).contains_points(np.c_[Xs.ravel(), Ys.ravel()]).reshape(n2)
pd_in = ndimage.distance_transform_edt(plan) * VOX
plan = ndimage.distance_transform_edt(~(pd_in > 9.0)) * VOX <= 9.0          # round every corner (r 9)
plan &= Xs <= X_R
plan &= np.hypot(Xs, Ys) >= 88.5                                           # rear edge scalloped concentric with the socket (cowl rim r 82.8 sweeps here)
pad_c = np.array([c for c, _ in PADS.values()])
from scipy.spatial import cKDTree
# wearer range of motion: cut the edges along the envelope of Forest's own arm (abduction / scaption / flexion 0..GH_SHARE, rigid
# about the glenohumeral centre) and head-neck (right lateral flexion 0..50, right rotation 0..90, flexion / extension 0..70, blended
# over the cervical span) with ROM_CLR; only the D24 seat pads are exempt (they sit on the clasp bosses, which touch the skin by design)
ROM_CLR = 5.0
# the spaulder seats on the acromion (F4) and upper trapezius (F1), which ride with the scapula; relative to the scapula the humerus
# turns only the glenohumeral share of elevation. McClure et al. 2001 (bone pins): GH:ST = 1.7:1, so full elevation (~175 deg,
# DESIGN_STATE sources) is ~110 deg at the GH joint. A rigid GH rotation beyond that drives the deltoid skin through the acromion.
GH_SHARE = 110.0
lmk = json.load(open(os.path.join(SH, "proxies", "wearer_landmarks.json")))
GH, NECK = np.array(lmk["gh_joint_rel_P"], float), np.array(lmk["neck_joint_rel_P"], float)
bs_p, _ = trimesh.sample.sample_surface_even(body, int(body.area / 4.0))
axh = np.array([0.0, np.sin(np.radians(10)), -np.cos(np.radians(10))]); t_ = (bs_p - GH) @ axh; rad_ = bs_p - GH - t_[:, None] * axh
armp = bs_p[(t_ > 25) & (t_ < 170) & (np.linalg.norm(rad_, axis=1) < 62) & (rad_[:, 1] > -40)]
hsel = (bs_p[:, 2] > NECK[2]) & (np.abs(bs_p[:, 1] - NECK[1]) < 115) & (bs_p[:, 0] < -60); headp = bs_p[hsel]
hw_ = np.clip((headp[:, 2] - NECK[2]) / 50.0, 0, 1)
def rodr(P_, axis, ang_rad, piv):
    axis = np.asarray(axis, float) / np.linalg.norm(axis); Q_ = P_ - piv; c_ = np.cos(ang_rad)[:, None] if np.ndim(ang_rad) else np.cos(ang_rad)
    s__ = np.sin(ang_rad)[:, None] if np.ndim(ang_rad) else np.sin(ang_rad)
    return Q_ * c_ + np.cross(axis[None, :], Q_) * s__ + axis[None, :] * (Q_ @ axis)[:, None] * (1 - c_) + piv
romP = []
for tgt in ([0, 1.0, 0], [-np.sin(np.radians(30)), np.cos(np.radians(30)), 0], [-1.0, 0, 0]):
    axr = np.cross([0, 0, -1.0], tgt)
    for a_ in np.arange(0.0, GH_SHARE + 0.01, 2.0): romP.append(rodr(armp, axr, np.radians(a_), GH))
for axr, amax in (([-1.0, 0, 0], 50.0), ([0, 0, -1.0], 90.0), ([0, -1.0, 0], 70.0), ([0, 1.0, 0], 70.0)):   # lateral flexion 50 = old pauldron's first contact (normal 42 +- 8, Apti 2023)
    for a_ in np.arange(0.0, amax + 0.01, 2.0): romP.append(rodr(headp, axr, np.radians(a_) * hw_, NECK))
romP = np.vstack(romP); romP = romP[np.all((romP > lo - 10) & (romP < lo + VOX * n + 10), axis=1)]
romP = romP[np.min(np.linalg.norm(romP[:, None, :] - pad_c[None], axis=2), axis=1) > 13.0 + ROM_CLR]   # only the D24 pad itself is exempt
# plan trim: pull the edge back wherever the wearer-ROM envelope reaches the shell's height band (design surface, -4..+12 mm), so the
# trimmed edges keep their rolled bead instead of being carved raw; the 3-D carve below remains as a safety net
_u0 = np.clip((X_R - Xs) / (X_R + 212.0), 0, 1); _yr = -10.0 - 8.0 * _u0
_z0 = -48.0 - 30.0 * _u0 ** 2 - (0.0012 + 0.0045 * _u0 ** 0.8) * (Ys - _yr) ** 2
romT = cKDTree(romP); cols_ = np.argwhere(plan); forb = np.zeros(n2, bool)
for dz_ in np.arange(-4.0, 12.01, 1.0):
    q_ = np.c_[lo2 + VOX * (cols_ + 0.5), _z0[plan] + dz_]
    d_, _ = romT.query(q_, distance_upper_bound=ROM_CLR + 2.0, workers=-1); forb[tuple(cols_[d_ < ROM_CLR + 1.5].T)] = True
seat2 = np.min([np.hypot(Xs - c_[0], Ys - c_[1]) for c_ in pad_c], axis=0) < 13.0
for c_ in pad_c:                                           # protected corridor seat -> rear edge (a seat can never be cut loose)
    a_, b_ = np.array(c_[:2]), np.array([X_R - 6.0, c_[1] * 0.4])
    tt = np.clip(((Xs - a_[0]) * (b_ - a_)[0] + (Ys - a_[1]) * (b_ - a_)[1]) / np.sum((b_ - a_) ** 2), 0, 1)
    seat2 |= np.hypot(Xs - (a_[0] + tt * (b_ - a_)[0]), Ys - (a_[1] + tt * (b_ - a_)[1])) < 7.0
forb = ndimage.binary_dilation(forb, iterations=int(2.0 / VOX)) & ~seat2
plan &= ~forb
plan = ndimage.binary_opening(plan, structure=np.ones((3, 3)), iterations=int(4.0 / VOX)) | (plan & seat2)
plan = ndimage.binary_closing(plan, structure=np.ones((3, 3)), iterations=int(3.0 / VOX)) & ~forb
lab2, nl2 = ndimage.label(plan); plan = lab2 == 1 + int(np.argmax(ndimage.sum(plan, lab2, range(1, nl2 + 1))))
log("  plan trimmed by the wearer-ROM envelope: %.0f cm2 removed" % (forb.sum() * VOX * VOX / 100))
psdf = ndimage.gaussian_filter(((ndimage.distance_transform_edt(~plan) - ndimage.distance_transform_edt(plan)) * VOX).astype(np.float32), 1.0)
psdf = np.maximum(psdf, Xs - X_R)
din = np.maximum(-psdf, 0.0)
u = np.clip((X_R - Xs) / (X_R + 212.0), 0, 1)
y_r = -10.0 - 8.0 * u
kcam = 0.0012 + 0.0045 * u ** 0.8                          # cross camber grows from the plate seam (rear corners sit on the ears) to the front
zm = -48.0 - 30.0 * u ** 2 - kcam * (Ys - y_r) ** 2 - 0.06 * np.abs(Ys - y_r) * u ** 0.5      # fore-aft dome, cross camber, crest
LAME_X = [X_R, -124.0, -164.0, -220.0]; STEP = 3.0
lame_id = np.digitize(-Xs, [-LAME_X[1], -LAME_X[2]])           # 0 rear, 1 mid, 2 front
fr = np.zeros(n2, np.float32)
for k in range(3):
    m_ = lame_id == k; fr[m_] = ((LAME_X[k] - Xs[m_]) / (LAME_X[k] - LAME_X[k + 1]))
zm = zm + STEP * fr
sm = lambda t: np.clip(t, 0, 1) ** 2 * (3 - 2 * np.clip(t, 0, 1))
curl = 8.0 * np.clip(1.0 - din / 11.0, 0, 1) ** 2 * (Xs < X_R - 12.0)       # outer edges roll down (not the rear seam)
curl = curl + 9.0 * np.clip(1.0 - din / 22.0, 0, 1) ** 1.6 * sm((Ys - 5.0) / 25.0) * sm((-135.0 - Xs) / 35.0)   # lateral wing over the acromion
zm = zm - curl
near_pad = lambda p: np.min([np.linalg.norm(p[:, :2] - c[:2], axis=1) for c, _ in PADS.values()], axis=0) < 12.0
body_low = body.submesh([np.where(body.triangles_center[:, 2] < -40.0)[0]], append=True)
floor_ = top_field([(body_low, SKIN_CLR)], lo2, n2, VOX)
floor_ = np.maximum(floor_, top_field([(clasp, PART_CLR)], lo2, n2, VOX, exempt=near_pad))
floor_ = np.maximum(floor_, top_field([(wing, PART_CLR), (plate, PART_CLR), (spine, PART_CLR), (crown_m, FIX_CLR)], lo2, n2, VOX,
                                      exempt=lambda p: (np.abs(p[:, 1]) > EAR_Y[0] - 1.0) & (p[:, 0] < X_CUT + 2.0) & (p[:, 2] > EAR_Z[0] - 1.0)))
ceil_ = arm_floor(lo2, n2, VOX)
k_ = int(np.ceil(ARM_CLR / VOX)); cg = np.full(n2, np.inf)
for di in range(-k_, k_ + 1):
    for dj in range(-k_, k_ + 1):
        d = np.hypot(di, dj) * VOX
        if d <= ARM_CLR: cg = np.minimum(cg, shift2(ceil_, di, dj, np.inf) - np.sqrt(ARM_CLR ** 2 - d * d))
ceil_ = cg
edge_band = plan & (din <= 2.0) & (Xs < X_R - 4.0)
zm2 = zm.copy(); half = np.full(n2, T_S / 2)
for it in range(3):                                         # the underside is zm - T/2 * sec (+1.2 at the rolled edge): iterate on the slope
    need = floor_ + np.minimum(half, 0.9 * T_S) + 0.3
    lift = np.where(plan, np.maximum(need - zm, 0.0), 0.0)
    lift = ndimage.gaussian_filter(ndimage.maximum_filter(lift, size=int(8 / VOX)), 4.0 / VOX)
    zm2 = np.where(plan, np.maximum(zm + lift, need), zm); zm2 = np.where(np.isfinite(zm2), zm2, zm)
    gzx, gzy = np.gradient(zm2, VOX); sec = np.sqrt(1 + gzx ** 2 + gzy ** 2); half = T_S / 2 * sec
zm2 = np.where(plan, np.maximum(zm2, floor_ + half + 0.3), zm2); zm2 = np.where(np.isfinite(zm2), zm2, zm)
gzx, gzy = np.gradient(zm2, VOX); sec = np.sqrt(1 + gzx ** 2 + gzy ** 2); half = T_S / 2 * sec
# true 3-D clearance pass: the heightfield floor cannot see an obstacle BESIDE a curled-down edge (the F4 clasp node under the
# lateral wing). Sample the underside, measure real distances, lift the short columns, repeat.
from scipy.spatial import cKDTree
cl_p = surf_pts(clasp, 0.7); cl_p = cl_p[np.min(np.linalg.norm(cl_p[:, None, :] - pad_c[None], axis=2), axis=1) >= 20.0]
OB3 = [(cKDTree(cl_p), PART_CLR), (cKDTree(surf_pts(body_low, 0.8)), SKIN_CLR), (cKDTree(surf_pts(wing, 0.8)), PART_CLR), (cKDTree(surf_pts(plate, 0.8)), PART_CLR)]
cols = np.argwhere(plan); cxy = lo2 + VOX * (cols + 0.5); lift_tot = np.zeros(n2)
OB3P = [surf_pts(m_, 0.8) for m_ in (body_low, wing, plate)]; OB3P.insert(0, cl_p)
for it in range(4):
    under = np.c_[cxy, zm2[plan] - half[plan]]
    near_seat = np.min(np.linalg.norm(under[:, None, :] - pad_c[None], axis=2), axis=1) < 20.0
    deficit = np.zeros(len(under))
    for (tr, need3), P_ in zip(OB3, OB3P):
        d, ii = tr.query(under, distance_upper_bound=need3 + 1.0, workers=-1); ok_ = np.isfinite(d)
        below = np.zeros(len(under), bool); below[ok_] = P_[ii[ok_], 2] < under[ok_, 2] - 0.5        # lifting only helps if the obstacle is below
        deficit = np.maximum(deficit, np.where(below, need3 + 0.3 - np.minimum(d, need3 + 1.0), 0.0))
    deficit[near_seat] = 0.0
    log("  3-D clearance pass %d: worst deficit %.2f mm over %d columns" % (it + 1, deficit.max(), int((deficit > 0).sum())))
    if deficit.max() <= 0.05: break
    lift3 = np.zeros(n2); lift3[plan] = deficit * 1.3
    lift3 = ndimage.gaussian_filter(ndimage.maximum_filter(lift3, size=int(6 / VOX)), 2.5 / VOX)
    lift3 = np.minimum(lift3, np.maximum(4.0 - lift_tot, 0.0)); lift_tot += lift3          # corrective lift capped at 4 mm in total
    zm2 = np.where(plan, zm2 + lift3, zm2)
    gzx, gzy = np.gradient(zm2, VOX); sec = np.sqrt(1 + gzx ** 2 + gzy ** 2); half = T_S / 2 * sec
zm2 = np.where(plan & np.isfinite(ceil_), np.minimum(zm2, ceil_ - half - 1.9), zm2)   # never into the arm sweep: the carve trims what then sits too low
gzx, gzy = np.gradient(zm2, VOX); sec = np.sqrt(1 + gzx ** 2 + gzy ** 2); half = T_S / 2 * sec
bad = plan & (zm2 + half + 1.8 > ceil_ + 0.05)
log("spaulder: lift max %.1f mm, cells over the arm ceiling: %d (%.0f mm2)" % (float(lift[plan].max()), int(bad.sum()), bad.sum() * VOX * VOX))
if bad.any():
    bi = np.argwhere(bad); xy = lo2 + VOX * (bi + 0.5); dz = (zm2 + half + 1.8 - ceil_)[bad]
    log("  ceiling bite: x %.0f..%.0f y %.0f..%.0f, worst %.1f mm" % (xy[:, 0].min(), xy[:, 0].max(), xy[:, 1].min(), xy[:, 1].max(), dz.max()))
Z = gz[None, None, :]; X3, Y3 = Xs[:, :, None], Ys[:, :, None]
Zm, Hh, Sc, Ps = zm2[:, :, None].astype(np.float32), half[:, :, None].astype(np.float32), sec[:, :, None].astype(np.float32), psdf[:, :, None]
sp = I((np.abs(Z - Zm) - Hh) / Sc, Ps)                                            # the lame shell
for k in (1, 2):                                                                  # risers + rolled hems where a lame overlaps the next
    xb = LAME_X[k]
    sp = U(sp, I(I(np.abs(X3 - (xb + T_S / 2)) - T_S / 2, I(Z - (Zm + Hh), (Zm - Hh - STEP) - Z)), Ps))
    ix = np.clip(int((xb + 0.5 - lo[0]) / VOX), 0, n[0] - 1); zc_row = (zm2[ix, :] - 0.6)[None, :, None]
    sp = U(sp, I(I(np.hypot(X3 - (xb - 0.6), Z - zc_row) - 1.5, np.abs(X3 - xb) - 3.0), Ps))
edge = I(I(-Ps - 2.0, Ps), I(I((Zm - Hh) - Z, Z - (Zm + Hh + 1.4)), X3 - (X_R - 4.0)))   # outer rolled bead (2 mm band, rolled outward / up)
sp = U(sp, edge)
for s_ in (1, -1):                                                                # corner posts onto the crown ears
    xc_ = sum(EAR_X) / 2 - 1.0
    sp = U(sp, I(I(I(np.abs(X3 - xc_) - 8.0, np.abs(Y3 - s_ * 87.25) - 5.5), EAR_Z[1] - Z), Z - (Zm + Hh)))
for nm, (c, nrm) in PADS.items():                                                 # clasp seats (unchanged contract) + bosses
    uu = np.cross(nrm, [1.0, 0, 0]); uu /= np.linalg.norm(uu); vv = np.cross(nrm, uu)
    q = np.stack(np.broadcast_arrays(X3 - c[0], Y3 - c[1], Z - c[2]), -1).astype(np.float32)
    an, au, av = q @ nrm.astype(np.float32), q @ uu.astype(np.float32), q @ vv.astype(np.float32); del q
    sp = U(sp, I(np.abs(an) - 2.5, np.hypot(au, av) - 12.0))                    # round D24 seat pad on the D24 clasp boss (was 24 x 24 square)
    sp = U(sp, I(I(np.hypot(au, av) - 7.0, -an), Z - (Zm + Hh)))
    del an, au, av
for s_ in (1, -1):                                                                # keep off the ears below their tops
    sp = I(sp, -box_sdf(X3, s_ * Y3, Z, (EAR_X[0] - 1, EAR_Y[0] - 3, -200.0), (0.0, 200.0, EAR_Z[1])))
# side clearance: carve the shell back from any obstacle closer than the rule (an edge beside the neck / clasp node side), seats excluded
def surf_dist(P_):
    mk = np.zeros(n, bool); ij = np.floor((P_ - lo) / VOX).astype(int); ok_ = np.all((ij >= 0) & (ij < n), axis=1); mk[tuple(ij[ok_].T)] = True
    return (ndimage.distance_transform_edt(~mk) * VOX).astype(np.float32)
for P_, need3 in ((cl_p, PART_CLR), (OB3P[1], SKIN_CLR), (OB3P[2], PART_CLR), (OB3P[3], PART_CLR)):
    dd = surf_dist(P_)
    sp = I(sp, (need3 + 0.4) - dd); del dd
dd = surf_dist(romP); sp = I(sp, (ROM_CLR + 0.5) - dd); del dd
log("  wearer-ROM carve: %d envelope points" % len(romP))
# remove slivers left by the carves (anything thinner than ~1.2 mm), seats and posts untouched by construction
sol = sp < 0; opened = ndimage.binary_opening(sol, structure=ndimage.generate_binary_structure(3, 1), iterations=1)
sp = np.where(sol & ~opened, np.float32(VOX), sp)
log("  side-clearance carve done")
sp = keep_component(sp); spa_m = mesh_sdf(sp, lo); del sp
cut = []
for nm, (c, nrm) in PADS.items():
    cut.append(cyl(c - nrm * 4.0, c + nrm * 3.0, 2.25))                            # M4 clearance through the pad
    cut.append(cyl(c + nrm * 2.5, c + nrm * 40.0, 4.25))                           # counterbore for the M4 x 12 head + key
for xh, yh in EAR_HOLES:
    cut.append(cyl([xh, yh, EAR_Z[1] - 1.0], [xh, yh, -30.0], 1.7))
    cut.append(cyl([xh, yh, EAR_Z[1] + 4.0], [xh, yh, -30.0], 3.4))                # M3 x 10 button head counterbore (4 mm clamp)
spa_m = finish(boolean(spa_m, cutters=cut))
for nm_, (c_, n_) in PADS.items():                       # both bolted seats must still be part of the spaulder
    dseat = float(np.min(np.linalg.norm(spa_m.vertices - c_, axis=1)))
    assert dseat < 15.0, "seat %s detached from the spaulder (%.1f mm)" % (nm_, dseat)
log("spaulder", spa_m.is_watertight, round(spa_m.volume / 1e3, 1), "cm3")

# ================================================================================================ export
os.makedirs(OUT, exist_ok=True)
arch = os.path.join(ST, "archive", "pauldron_rev2026-10-23"); os.makedirs(arch, exist_ok=True)
for f in glob.glob(os.path.join(OUT, "pauldron_*_R.stl")) + glob.glob(os.path.join(OUT, "pauldron_*_L.stl")):
    if not any(k in os.path.basename(f) for k in ("crown", "keel", "spaulder")): shutil.move(f, os.path.join(arch, os.path.basename(f)))
parts = {"crown": crown_m, "keel": keel_m, "spaulder": spa_m}
info = dict(rev="2026-10-30 armour", parts={}, params=dict(VOX=VOX, T=T, density_g_cm3=PACF, A_R=A_R, A_F=A_F, B_W=B_W, N_SE=N_SE, Z_RIM=Z_RIM,
            R_BORE_MIN=R_BORE_MIN, BANDS=BANDS, FLARE=FLARE, KEEL=[KEEL_C, KEEL_PX, KEEL_PY, KEEL_OFF], LAME_X=LAME_X, STEP=STEP,
            clearances=dict(contents=G_IN, s1_sweep=M1_CLR, arm=ARM_CLR, part=PART_CLR, skin=SKIN_CLR, fixed=FIX_CLR)),
            pads={k: dict(centre=np.round(c, 2).tolist(), normal=np.round(nn, 3).tolist()) for k, (c, nn) in PADS.items()},
            plate_feet=FEET, foot_inserts=holes_ft, ear_inserts=EAR_HOLES, hem_joints_phi=JOIN_PHI,
            spaulder_arm_ceiling_violation_mm2=float(bad.sum() * VOX * VOX))
def post_simplify(m, tol=0.04):
    """STL round trip, then manifold3d simplify; kept only if still closed with the same volume (+-0.2 %)"""
    import manifold3d as mf, tempfile
    p_ = tempfile.mktemp(suffix=".stl"); m.export(p_); r = trimesh.load(p_); os.remove(p_)
    o = mf.Manifold(mf.Mesh(vert_properties=np.asarray(r.vertices, np.float32), tri_verts=np.asarray(r.faces, np.uint32))).simplify(tol).to_mesh()
    d = trimesh.Trimesh(o.vert_properties[:, :3], o.tri_verts, process=False)
    if r.volume < 0: r.invert()
    if d.volume < 0: d.invert()                              # outward normals (a closed manifold can come back inside-out)
    if d.is_watertight and abs(d.volume - r.volume) < 0.002 * abs(r.volume): return d
    log("  post-simplify rejected"); return r
parts = {k: post_simplify(m) for k, m in parts.items()}
for k, m in parts.items():
    m.export(os.path.join(OUT, f"pauldron_{k}_R.stl"))
    mL = m.copy(); mL.apply_transform(np.diag([1, -1, 1, 1.0]))
    if mL.volume < 0: mL.invert()
    mL.export(os.path.join(OUT, f"pauldron_{k}_L.stl"))
    assert m.volume > 0, k
    info["parts"][k] = dict(extents=m.extents.round(1).tolist(), fits_256=bool(m.extents.max() <= 256), volume_cm3=round(m.volume / 1e3, 1),
                            mass_g_PACF=round(m.volume / 1e3 * PACF, 1), watertight=bool(m.is_watertight), faces=len(m.faces))
info["total_mass_g"] = round(sum(v["mass_g_PACF"] for v in info["parts"].values()), 1)
trimesh.util.concatenate(list(parts.values())).export(os.path.join(OUT, "pauldron_R.stl"))
# render-only proxies of the visible fastener heads (ISO 7380 M3 button heads D5.7 x 1.65): they read as the rivets of the lames
heads = []
def head_at(p, nrm, r=2.85, h=1.65):
    sph = trimesh.creation.icosphere(subdivisions=2, radius=1.0); sph.apply_scale([r, r, h])
    sph = sph.slice_plane([0, 0, 0], [0, 0, 1], cap=True); z = np.array([0, 0, 1.0]); nrm = np.asarray(nrm, float) / np.linalg.norm(nrm)
    ax = np.cross(z, nrm); s_ = np.linalg.norm(ax); M = np.eye(4)
    if s_ > 1e-9: M = trimesh.transformations.rotation_matrix(np.arctan2(s_, z @ nrm), ax / s_)
    M[:3, 3] = p; sph.apply_transform(M); return sph
for ph in JOIN_PHI:
    p, u_ = hem_point(ph, float(flare(np.array(Z_JOIN), *BANDS[1])) + T_C); heads.append(head_at(p, u_))
for xh, yh in EAR_HOLES: heads.append(head_at([xh, yh, EAR_Z[1] + 4.0], [0, 0, 1.0]))
trimesh.util.concatenate(heads).export(os.path.join(ST, "shoulder", "proxies", "pauldron_rivets_R.stl"))
json.dump(info, open(os.path.join(OUT, "pauldron.json"), "w"), indent=1)
log(json.dumps(info["parts"]), "total", info["total_mass_g"], "g")
