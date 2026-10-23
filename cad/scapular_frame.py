"""Scapular frame (RIGHT side, robot frame, mm): the fixed 'scapula' of the third shoulder.

Design logic (see SPEC_third_shoulder.md):
  * The wearer's scapula slides 5-10 cm under the skin at full arm elevation (McClure 2001 upward rotation 50 deg),
    so nothing rigid may press on the blade. The wing only CONTACTS the paraspinal strip beside the spine (~0 mm skin
    motion), and BRIDGES over the blade with a clearance gap to reach the aluminium chassis plate.
  * Left and right wings are joined across the spine by a separate spine bridge (clear of the spinous processes),
    turning two shoulder clasps into one backpack-like frame.

Geometry is a heightfield shell offset from the back skin (ray cast along -x), voxelised and meshed with marching cubes.
Outputs: stl/shoulder/scapular/{wing_R,wing_L,spine_bridge}.stl, scapular_frame.json"""
import numpy as np, trimesh, json, os, sys
from scipy import ndimage
from skimage import measure
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "..", "stl", "shoulder", "scapular"); os.makedirs(OUT, exist_ok=True)
LM = json.load(open(os.path.join(HERE, "..", "stl", "shoulder", "proxies", "wearer_landmarks.json")))
SP = LM["spine_y_rel_P"]                       # spine plane y (-150)
H = 1.25                                       # grid (mm)

# ---------------- zones (y relative to the spine, z relative to P) ----------------
# rev 2026-10-12c (I2): rail lengthened to z -400 and widened to y 8..46 so the triangular rail pressure stays <= 14.7 kPa
# (harness_loads.py); gap 7 = 5 mm foam stack + 1.5 base layer + 0.5 (gate: rail foam stack >= 5 mm)
# rev 2026-10-14d (rail pressure study, rail_eval.py / rail_sweep*.py / body_loads.py RAIL_CFG): medial edge inset 6 -> 12 mm off the spinous line,
# lower end -400 -> -440 (area 141 -> 137 cm2 with the inset; lever arm longer, peaks down ~17 %), and a MEDIAL RELIEF: the wing inner face is recessed up to
# +4 mm at the medial edge (foam 6 -> 10 mm, ramp over 22 mm) and the foam there is a SOFT strip (E x 0.15, e.g. 30 kPa PU memory foam vs 200 kPa EVA, same ramp).
# Outer face is unchanged (wall 7 -> 4 mm at the edge, >= 5.7 mm for y >= 27 where the M4 inserts are).
RAIL = dict(y=(12, 50), z=(-440, -80), gap=7.5, t=7.0, relief=dict(w=22.0, extra=4.0, E_f=0.15))          # contact zone; 7 mm wall for M4 inserts
BRIDGE = dict(y=(38, 100), z=(-168, -100), gap=14.0, t=5.0)     # rev 2026-10-12c: top edge -84 -> -100 (it lapped onto the trapezius ridge)       # over the superior/medial blade, no contact
TONGUE = dict(y=(72, 104), z=(-150, -76), gap=12.0, t_max=12.0) # under the chassis plate medial edge; outer face = plate front (x=-88)
PLATE_FRONT = -88.2          # 0.2 mm shy of the plate front face: designed seat, bolted (no interference)
SPINE = dict(y=(-26, 26), z=(-240, -112), gap=14.0, t=5.0, overlap=(12, 26))   # separate part, symmetric about the spine
FILLET = 4.0

m = trimesh.load(os.path.join(HERE, "..", "stl", "shoulder", "proxies", "wearer_makehuman.stl"))
c = m.triangles_center
keep = (c[:, 1] > SP - 60) & (c[:, 1] < SP + 140) & (c[:, 2] > -480) & (c[:, 2] < -40) & (c[:, 0] > -260)
T = m.triangles[keep]

def back_x(Y, Z):
    """x of the back skin under each (y,z) via vectorised Moller-Trumbore along -x (first hit = most +x)."""
    d = np.array([-1.0, 0, 0]); v0 = T[:, 0]; e1 = T[:, 1] - v0; e2 = T[:, 2] - v0
    p = np.cross(d, e2); det = np.einsum("ij,ij->i", e1, p); ok = np.abs(det) > 1e-12; inv = np.where(ok, 1 / np.where(ok, det, 1), 0)
    out = np.full(Y.size, np.nan); O = np.c_[np.full(Y.size, 200.0), Y.ravel(), Z.ravel()]
    for i0 in range(0, len(O), 300):     # small batches: the ray-cast arrays are batch x triangles x 3
        o = O[i0:i0 + 300][:, None, :]; s = o - v0[None]
        u = np.einsum("nij,ij->ni", s, p) * inv; q = np.cross(s, e1[None]); v = (q @ d) * inv; t = np.einsum("ij,nij->ni", e2, q) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6); t = np.where(hit, t, np.inf); tm = t.min(1)
        out[i0:i0 + 300] = np.where(np.isfinite(tm), 200.0 - tm, np.nan)
    return out.reshape(Y.shape)

ys = np.arange(SP - 40, SP + 115, H); zs = np.arange(-460, -60, H)
Yg, Zg = np.meshgrid(ys, zs, indexing="ij")
XS = back_x(Yg, Zg)
XS = ndimage.generic_filter(np.nan_to_num(XS, nan=np.nanmin(XS)), np.median, size=3)   # de-speckle the low-poly mesh
dy = Yg - SP
# rev 2026-10-12c (I2): gaps are specified NORMAL to the skin. The heightfield offsets along x, so on sloped skin (top of the
# shoulder, trapezius ridge) the x-offset is divided by the x-component of the skin normal (capped at 1/0.35).
gyy, gzz = np.gradient(XS, H, H); NX = 1.0 / np.sqrt(1.0 + gyy ** 2 + gzz ** 2); XF = 1.0 / np.maximum(NX, 0.35)

def zone(Z_, **k): return (dy >= k["y"][0]) & (dy <= k["y"][1]) & (Zg >= k["z"][0]) & (Zg <= k["z"][1])
def rounded(mask, r=FILLET):
    """rounded-corner 2D outline: close+open with a disc"""
    n = int(round(r / H)); st = np.hypot(*np.mgrid[-n:n + 1, -n:n + 1]) <= n
    return ndimage.binary_opening(ndimage.binary_closing(mask, st), st)

# inner (skin-side) and outer x per (y,z) for the wing
mR, mB, mT = zone(None, **RAIL), zone(None, **BRIDGE), zone(None, **TONGUE)
inner = np.full(XS.shape, np.nan); outer = np.full(XS.shape, np.nan)
# bridge first, rail overrides (contact), tongue overrides at the plate
inner[mB] = (XS + BRIDGE["gap"] * XF)[mB]; outer[mB] = inner[mB] + BRIDGE["t"]
inner[mR] = (XS + RAIL["gap"] * XF)[mR]; outer[mR] = inner[mR] + RAIL["t"]       # outer face is set by the unrelieved gap (pod/cradle seats do not move)
_rl = RAIL["relief"]; RG = RAIL["gap"] + _rl["extra"] * (1.0 - np.clip((dy - RAIL["y"][0]) / _rl["w"], 0.0, 1.0))      # relieved gap (mm, normal to skin)
ti = np.maximum(XS + TONGUE["gap"], PLATE_FRONT - TONGUE["t_max"])
inner[mT] = ti[mT]; outer[mT] = PLATE_FRONT
# smooth the gap transition between rail and bridge so the inner face is a ramp, not a step
wing = rounded(mR | mB | mT)
gi = ndimage.gaussian_filter(np.nan_to_num(inner, nan=np.nanmean(inner)), 2.0); go = ndimage.gaussian_filter(np.nan_to_num(outer, nan=np.nanmean(outer)), 2.0)
inner = np.where(mR, XS + RG * XF, gi)
# rev 2026-10-12c (I2): the smoothed ramp may never dip below each zone's required gap (it used to sag to 2.8 mm over the blade)
req = np.where(mR, RG, np.where(mB & ~mT, BRIDGE["gap"], 0.0))
inner = np.where(mT, inner, np.maximum(inner, XS + req * XF))
outer = np.where(mT, PLATE_FRONT, np.maximum(go, inner + 4.0))
outer = np.where(mT, PLATE_FRONT, outer)

CLASP = trimesh.load(os.path.join(HERE, "..", "stl", "shoulder", "clasp_frame_R.stl"))
def solid(mask, xin, xout, name, cut=None, cut_clr=1.5):
    xs = np.arange(np.nanmin(xin[mask]) - 3, np.nanmax(xout[mask]) + 3, H)
    occ = (xs[:, None, None] >= xin[None]) & (xs[:, None, None] <= xout[None]) & mask[None]
    if cut is not None:      # rev 2026-10-12c: remove anything within cut_clr of another part (wing vs clasp interference)
        vg = cut.voxelized(pitch=H).fill(); cp_ = vg.points                         # filled voxel centres of the other part
        cp_ = cp_[np.all((cp_ > np.array([xs[0], ys[0], zs[0]]) - 5) & (cp_ < np.array([xs[-1], ys[-1], zs[-1]]) + 5), axis=1)]
        if len(cp_):
            ii = np.round((cp_ - np.array([xs[0], ys[0], zs[0]])) / H).astype(int)
            ok = np.all((ii >= 0) & (ii < np.array(occ.shape)), axis=1); blk = np.zeros_like(occ); blk[tuple(ii[ok].T)] = True
            blk = ndimage.binary_dilation(blk, iterations=int(np.ceil(cut_clr / H)) + 1)
            occ &= ~blk
    occ = np.pad(occ, 2)
    v, f, _, _ = measure.marching_cubes(occ.astype(np.float32), 0.5, spacing=(H,) * 3)
    v = v - 2 * H + np.array([xs[0], ys[0], zs[0]])
    mm = trimesh.Trimesh(v, f)
    if mm.volume < 0: mm.invert()
    trimesh.smoothing.filter_taubin(mm, iterations=8)
    try: mm = mm.simplify_quadric_decimation(face_count=40000)
    except Exception: pass
    mm.export(os.path.join(OUT, name + ".stl")); return mm

wR = solid(wing, inner, outer, "wing_R", cut=CLASP)
# rev 2026-10-12c: smoothing pushed the tongue face 0.4 mm into the plate; clip the wing flat at the seat plane (x <= -88.2)
_hs = trimesh.creation.box(bounds=[[-400.0, -400.0, -600.0], [PLATE_FRONT, 400.0, 200.0]])
wR = trimesh.boolean.intersection([wR, _hs], engine="manifold"); wR.export(os.path.join(OUT, "wing_R.stl"))
# rev 2026-10-14b: sEMG lead port. The three under-rail pairs (trap_mid, rhomboid, trap_low; emg_atlas.json under_rail) sit in the compression base layer
# UNDER the continuous 5 mm foam; their flat leads run in the base layer to the AFE pod and pass through the rail wall in ONE slot (no foam removed,
# so the rail pressure of harness_loads.py is untouched). Slot: 2.5 (y) x 14 (z) through the 7 mm wall, centred on the AFE pod (y SP+27, z -320).
LEAD = dict(y=SP + 27.0, z=-320.0, wy=2.5, hz=14.0)
_slot = trimesh.creation.box(bounds=[[-200.0, LEAD["y"] - LEAD["wy"] / 2, LEAD["z"] - LEAD["hz"] / 2], [PLATE_FRONT, LEAD["y"] + LEAD["wy"] / 2, LEAD["z"] + LEAD["hz"] / 2]])
_vb = wR.volume; wR = trimesh.boolean.difference([wR, _slot], engine="manifold"); wR.export(os.path.join(OUT, "wing_R.stl"))
print("lead slot removed", round(_vb - wR.volume), "mm3 from the wing")
# rev 2026-10-12c (I2): the lengthened rail makes the wing 325 mm tall (> 256 build volume): print it as two pieces split at
# z = Z_SPLIT, joined by a 50 x 34 x 4 mm PA-CF splice plate on the rail's outer face (4 x M4 into heat-set inserts, 2 each side)
Z_SPLIT = -250.0
_up = trimesh.creation.box(bounds=[[-400.0, -400.0, Z_SPLIT], [400.0, 400.0, 200.0]]); _dn = trimesh.creation.box(bounds=[[-400.0, -400.0, -600.0], [400.0, 400.0, Z_SPLIT]])
w_up = trimesh.boolean.intersection([wR, _up], engine="manifold"); w_dn = trimesh.boolean.intersection([wR, _dn], engine="manifold")
w_up.export(os.path.join(OUT, "wing_R_upper.stl")); w_dn.export(os.path.join(OUT, "wing_R_lower.stl"))
_yc = SP + 0.5 * (RAIL["y"][0] + RAIL["y"][1])
_loc, _, _ = wR.ray.intersects_location([[200.0, _yc, Z_SPLIT]], [[-1.0, 0, 0]]); _xo = float(_loc[:, 0].max())
splice = trimesh.creation.box(bounds=[[_xo, _yc - 17.0, Z_SPLIT - 25.0], [_xo + 4.0, _yc + 17.0, Z_SPLIT + 25.0]])
splice.export(os.path.join(OUT, "wing_splice_R.stl"))
# spine bridge (symmetric about the spine; built on the right-side heightfield and mirrored for the left half)
sp_mask = rounded((dy >= 0) & (dy <= SPINE["y"][1]) & (Zg >= SPINE["z"][0]) & (Zg <= SPINE["z"][1]))
ov = (dy >= SPINE["overlap"][0]) & (dy <= SPINE["overlap"][1]) & mR
LAP = 1.5                                     # rev 2026-10-12c: lap offset 0.3 -> 1.5 mm (smoothing used to make the parts overlap)
bi = np.where(ov, outer + LAP, XS + SPINE["gap"]); bi = np.where(ov, bi, np.maximum(bi, ndimage.gaussian_filter(np.where(ov, outer + LAP, XS + SPINE["gap"]), 2)))
bo = bi + SPINE["t"]
half = solid(sp_mask, bi, bo, "_spine_half", cut=wR, cut_clr=0.6)      # rev 2026-10-12c: lap seat carved from the wing itself
mir = half.copy(); mir.apply_transform(np.array([[1, 0, 0, 0], [0, -1, 0, 2 * SP], [0, 0, 1, 0], [0, 0, 0, 1.0]])); mir.fix_normals()
print("half", round(half.volume), half.is_watertight, "mir", round(mir.volume), mir.is_watertight)
try:
    import manifold3d as mf
    def M(t): return mf.Manifold(mf.Mesh(vert_properties=np.asarray(t.vertices, np.float32), tri_verts=np.asarray(t.faces, np.uint32)))
    u = (M(half) + M(mir)).to_mesh(); spine = trimesh.Trimesh(u.vert_properties[:, :3], u.tri_verts)
except Exception as e:
    print("manifold union failed", e); spine = trimesh.util.concatenate([half, mir])
spine.export(os.path.join(OUT, "spine_bridge.stl")); os.remove(os.path.join(OUT, "_spine_half.stl"))
wL = wR.copy(); wL.apply_transform(np.diag([1, -1, 1, 1.0])); wL.invert(); wL.export(os.path.join(OUT, "wing_L.stl"))   # left wing in the LEFT arm's own frame

# report
gapR = (inner - XS)[mR]; gapB = (inner - XS)[mB & ~mR & ~mT]
rho = 1.15e-3   # CF-PA (nylon-CF) g/mm3 (assumed material; see spec)
info = dict(wing_volume_cm3=round(wR.volume / 1e3, 1), wing_mass_g_CFPA=round(wR.volume * rho), spine_volume_cm3=round(spine.volume / 1e3, 1),
            spine_mass_g=round(spine.volume * rho), watertight=[bool(wR.is_watertight), bool(spine.is_watertight)],
            rail_gap_mm=[round(float(gapR.min()), 1), round(float(gapR.max()), 1)], bridge_gap_mm=[round(float(np.nanmin(gapB)), 1), round(float(np.nanmax(gapB)), 1)],
            wing_bounds=wR.bounds.round(1).tolist(), spine_bounds=spine.bounds.round(1).tolist(),
            lead_slot=dict(LEAD, removed_mm3=round(_vb - wR.volume)), zones=dict(RAIL=RAIL, BRIDGE=BRIDGE, TONGUE=TONGUE, SPINE=SPINE), spine_y=SP)
json.dump(info, open(os.path.join(OUT, "scapular_frame.json"), "w"), indent=1)
print(json.dumps({k: v for k, v in info.items() if k != "zones"}, indent=1))
