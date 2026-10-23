"""Compact ball-and-socket shoulder housing (rev 2026-10-13, Forest: "make the shoulders not giant spheres").
Replaces the 190 mm ball (cad/ball_shell.py, now retired) with two conformal parts:

  SOCKET COWL (M1, turns with S1): a smooth shell that shrink-wraps the S2 actuator, its clamp, the yoke saddle and the S2 stop
    pads. Built as an offset shell around the CLOSED contents (morphological closing radius R_CLOSE removes crevices, so the
    outside reads as one rounded pebble, not a wrapped bracket). Inner face G_IN off the contents, wall T_WALL. Its +y face is an
    end wall with a bore (R_BORE) where the S2 output / adapter / humeral-head dome sit. The arm's swept volume (all M2 + M3
    parts at S3 extremes, S2 from the TPU-compression limit -40 to the yoke backstop 95) grown by LIP_GAP is cut out as the
    slot (gap 3.2 mm < 4 mm: no fingertip entry, ISO 13857). Bottom cut at z Z_BOT (above the yoke disc top), 4 tabs onto
    the yoke disc. Split at x = 0 into two halves that slide on along +-x (assembly), joined by two seam laps.
  HUMERAL HEAD DOME (M2, turns with S2): spherical cap about P, R 44.4..46.8, y >= Y_DOME, over the S2 adapter and the
    bracket root. Rotationally symmetric about the S2 axis, so it sweeps only itself: constant clearance to the cowl bore
    and to the yoke (>= 3.2 mm). The bracket passes through a y-extruded cut-out, so the dome slides on along -y.

Outputs stl/shoulder/cowl/: cowl_R.stl (both halves, for sweeps/renders), cowl_R_a_posX.stl, cowl_R_b_negX.stl, head_dome_R.stl,
cowl.json. Right arm, robot frame, mm."""
import os, sys, json, numpy as np, trimesh
from scipy import ndimage
from skimage import measure
from trimesh.transformations import rotation_matrix as rot
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, rom_limits as rl, facet
OUT = os.path.join(HERE, "..", "stl", "shoulder", "cowl"); os.makedirs(OUT, exist_ok=True)
SH = os.path.join(HERE, "..", "stl", "shoulder")

VOX = 0.8
G_IN, T_WALL = 4.0, 2.4            # inner face 4 mm off the (closed) contents, 2.4 mm PETG wall
R_CLOSE = 20.0                     # closing radius: crevices narrower than ~40 mm are bridged
SMOOTH = 3.0                       # gaussian sigma (mm) on the distance field -> rounded outer surface
LIP_GAP = 3.5                      # slot clearance to the swept arm: 3.5 designed so >= 3.0 survives facet snapping; < 4 mm (no fingertip entry)
R_BORE, Y_END = 50.0, 0.0          # end-wall bore radius about the S2 axis; end wall occupies y Y_END..Y_END+T_WALL
Z_BOT = -49.6                      # cowl bottom (yoke disc top is z -50)
DOME_RI, DOME_RO_VTX, Y_DOME = 43.8, 46.4, 3.4          # vertices on 46.4: >= 3.6 mm to the yoke (r 50) after facet snapping
DOME_RO = DOME_RO_VTX
S2LO, S2HI = rl.HARD["S2"][0] - 5.0, 95.0
N_FACETS, FACET_MARGIN = 260, 0.9   # faceted outer surface (Forest: "facet it to my shoulder"); grown 1.6 mm so chords never thin the wall

G = ak.groups(); G["M1"].pop("cowl", None); G["M2"].pop("head", None)
stops = {k: trimesh.load(os.path.join(SH, "stops", k + ".stl")) for k in ("s2_pad_pos_R", "s2_pad_neg_R", "tpu_s2_pos_R", "tpu_s2_neg_R")}
CONT = {**{k: G["M1"][k] for k in ("yoke", "s2_clamp", "ak_s2")}, **stops}

lo = np.array([-100.0, -80.0, -64.0]); hi = np.array([100.0, 64.0, 84.0])
n = np.ceil((hi - lo) / VOX).astype(int)
gx, gy, gz = [lo[i] + VOX * (np.arange(n[i]) + 0.5) for i in range(3)]
X, Y, Z = np.meshgrid(gx, gy, gz, indexing="ij")
def to_idx(p): return np.floor((p - lo) / VOX).astype(int)

def solid(m):
    """solid occupancy of a closed mesh on the grid (voxelize at grid pitch, fill, shift into the grid)"""
    v = m.voxelized(pitch=VOX).fill(); pts = v.points
    i = to_idx(pts); ok = np.all((i >= 0) & (i < n), 1); o = np.zeros(n, bool); o[tuple(i[ok].T)] = True
    return ndimage.binary_dilation(o, iterations=1) if len(pts) < 50 else o
occ = np.zeros(n, bool)
for k, m in CONT.items(): occ |= solid(m)
occ_cav = occ.copy()                                    # all contents (incl. yoke at y > 0): the cavity must clear these
occ_c = occ & (Y <= Y_END)                              # the cowl wraps the motor side only
# ---- closing (via distance transforms) -> rounded pebble
d_out = ndimage.distance_transform_edt(~occ_c) * VOX
dil = d_out <= R_CLOSE
d_in = ndimage.distance_transform_edt(dil) * VOX
closed = d_in > R_CLOSE
closed |= occ_c
D = ndimage.distance_transform_edt(~closed) * VOX     # distance from the closed contents
Ds = ndimage.gaussian_filter(D, SMOOTH / VOX)
D_all = ndimage.distance_transform_edt(~occ_cav) * VOX # distance from every M1 part (for the cavity)
ry = np.hypot(X, Z)                                     # radius about the S2 axis (y)
solid_big = (Ds <= G_IN + T_WALL + FACET_MARGIN) & (Y <= Y_END + T_WALL + 3.0) & (Z >= Z_BOT - 3.0)
solid_out, facet_mesh = facet.faceted(solid_big, lo, VOX, N_FACETS)
solid_out &= (Y <= Y_END + T_WALL) & ((Ds <= G_IN + T_WALL) | solid_out)
Do = ndimage.distance_transform_edt(~solid_out) * VOX     # distance outside the faceted outer surface
Di = ndimage.distance_transform_edt(solid_out) * VOX      # depth inside it
cavity = ((Ds < G_IN) & (Y < Y_END)) | (D_all < G_IN) | ((ry < R_BORE) & (Y > -6.0))
cowl = solid_out & ~cavity & (Z >= Z_BOT)

# ---- arm slot: sweep of every M2 part (+ M3 at S3 extremes) about the S2 axis, grown by LIP_GAP
sw = np.zeros(n, bool)
M2 = dict(G["M2"]); M3 = G["M3"]
for t2 in np.arange(S2LO, S2HI + 0.1, 1.0):
    T2 = rot(np.radians(-t2), [0, 1, 0])
    for s3 in ((0.0,) if abs(t2 - round(t2 / 15) * 15) > 0.5 else (-75.0, 0.0, 95.0)):
        T3 = T2 @ rot(np.radians(s3), [0, 0, 1])
        for grp, TT in ((M2, T2), (M3, T3)):
            for k, m in grp.items():
                if m.bounds[0][2] > 150: continue                       # far up the arm: never near the cowl
                mm = m.copy(); mm.apply_transform(TT)
                if np.any(mm.bounds[0] > hi) or np.any(mm.bounds[1] < lo): continue
                p = mm.sample(int(mm.area / (VOX * VOX * 0.6)) + 200)
                i = to_idx(p); ok = np.all((i >= 0) & (i < n), 1); sw[tuple(i[ok].T)] = True
sw = ndimage.binary_closing(sw, iterations=2)
d_sw = ndimage.distance_transform_edt(~sw) * VOX
slot = d_sw < LIP_GAP + VOX * 0.5
cowl &= ~slot

# ---- assembly draft (rev 2026-10-13): each half slides on along x (+x half moving -x, -x half moving +x) with the arm at S2 0.
#      Remove every half voxel whose path in crosses part material: for the +x half, voxels with part material at larger x in
#      the same (y, z) column; for the -x half, at smaller x. Parts = M1 contents + M2/M3 at S2 0 (they stay in place).
_occ_asm = occ_cav.copy()
for grp, TT in ((G["M2"], np.eye(4)), (G["M3"], np.eye(4))):
    for k, m in grp.items():
        if m.bounds[0][2] < hi[2] and m.bounds[1][2] > lo[2]: _occ_asm |= solid(m)
_occ_asm = ndimage.binary_dilation(_occ_asm, iterations=int(round(1.6 / VOX)))
_sh_pos = np.flip(np.maximum.accumulate(np.flip(_occ_asm, 0), axis=0), 0)          # material at larger x
_sh_neg = np.maximum.accumulate(_occ_asm, axis=0)                                    # material at smaller x
_draft = ((X > 0) & _sh_pos & ~_occ_asm) | ((X < 0) & _sh_neg & ~_occ_asm)
print("assembly-draft voxels removed:", int((cowl & _draft).sum()))
cowl &= ~_draft
# ---- tabs onto the yoke disc top (z -49.8 .. -46.8), M3 x 8 down into disc inserts; azimuths where the disc top is free
tabs = []
for az in np.arange(0, 360, 5.0):
    c, s = np.cos(np.radians(az)), np.sin(np.radians(az))
    if s > -0.1: continue                                             # y < 0 side only (the dome side has no cowl)
    u = X * c + Y * s; w = -X * s + Y * c
    box = (u >= 52.0) & (u <= 70.0) & (np.abs(w) <= 5.0) & (Z >= -49.8) & (Z <= -46.8)
    if (box & (D_all < 1.0)).any() or (box & slot).any(): continue
    tabs.append(az)
print("free tab azimuths:", tabs)
pick = []
for half in (lambda a: np.cos(np.radians(a)) > 0.05, lambda a: np.cos(np.radians(a)) < -0.05):   # two per x-half, as far apart as possible
    cand = [a for a in tabs if half(a)]
    if len(cand) >= 2:
        best = max(((a, b) for a in cand for b in cand if a < b), key=lambda ab: abs(((ab[1] - ab[0] + 180) % 360) - 180))
        pick += list(best) if abs(((best[1] - best[0] + 180) % 360) - 180) >= 20.0 else [best[0]]
    elif cand: pick.append(cand[0])
holes = np.zeros(n, bool)
for az in pick:
    c, s = np.cos(np.radians(az)), np.sin(np.radians(az))
    u = X * c + Y * s; w = -X * s + Y * c
    cowl |= (u >= 52.0) & (np.abs(w) <= 5.0) & (Z >= -49.8) & (Z <= -46.8) & (solid_out | (Ds <= G_IN + 0.5))
    cx, cy = 56.0 * c, 56.0 * s
    holes |= (np.hypot(X - cx, Y - cy) <= 1.7) & (Z >= -50.5) & (Z <= -45.0)             # M3 clearance
# ---- split at x = 0 (1.6 mm seam = 2 voxels) + two lap joints on the seam (top and back faces)
SEAM = 0.8
cowl &= np.abs(X) >= SEAM
outer_shell = (Di > 0) & (Di <= VOX * 1.5) & cowl
laps = []
for (py, pz) in ((-25.0, None), (None, -10.0)):
    # find the seam-plane point of the outer surface: top (max z at x~0, y=py) or back (min y at x~0, z=pz)
    col = (np.abs(X) < SEAM + 2 * VOX) & outer_shell
    if pz is None: sel = col & (np.abs(Y - py) < 1.0); idx = np.argwhere(sel);
    else: sel = col & (np.abs(Z - pz) < 1.0); idx = np.argwhere(sel)
    if len(idx) == 0: continue
    pts = np.c_[gx[idx[:, 0]], gy[idx[:, 1]], gz[idx[:, 2]]]
    p0 = pts[np.argmax(pts[:, 2])] if pz is None else pts[np.argmin(pts[:, 1])]
    nrm = np.array([0.0, 0.0, 1.0]) if pz is None else np.array([0.0, -1.0, 0.0])
    near = (np.abs(X) <= 9.0) & (np.linalg.norm(np.stack([X - p0[0], Y - p0[1], Z - p0[2]], -1) * (1 - np.abs(nrm)), axis=-1) <= 9.0)
    lap = near & (Do > 2 * VOX) & (Do <= 2 * VOX + 2.0) & ~slot
    lap_join = near & (Di <= 1.0) & (Do <= 2 * VOX + 0.1) & (X >= SEAM + VOX) & ~slot
    cowl |= lap | lap_join                                             # outer lap on the +x half over the -x half
    bc = p0 - nrm * (T_WALL + 3.0) + np.array([-5.0, 0, 0])            # inner boss on the -x half
    dd = np.stack([X - bc[0], Y - bc[1], Z - bc[2]], -1); rad = dd @ nrm; perp = np.linalg.norm(dd - rad[..., None] * nrm, axis=-1)
    cowl |= (perp <= 4.5) & (Ds >= G_IN - 1.5) & (Ds <= G_IN + T_WALL) & (X <= -SEAM) & (D_all >= 1.5) & ~slot
    hc = p0 + np.array([-5.0, 0, 0]); dd = np.stack([X - hc[0], Y - hc[1], Z - hc[2]], -1); rad = dd @ nrm; perp = np.linalg.norm(dd - rad[..., None] * nrm, axis=-1)
    holes |= (perp <= 1.7) & (Ds >= G_IN - 2.0) & (Do <= 2 * VOX + 2.5)
    laps.append(dict(at=np.round(p0, 1).tolist(), normal=nrm.tolist()))
cowl &= ~holes
cowl = ndimage.binary_opening(cowl, iterations=1)
lab, nl = ndimage.label(cowl, structure=np.ones((3, 3, 3))); sz = ndimage.sum(cowl, lab, range(1, nl + 1))
keep = 1 + np.argsort(sz)[::-1][:2]; cowl = np.isin(lab, keep)

def thin_frac(vol, t_min=1.6):
    """fraction of voxels in walls thinner than t_min (local thickness ~ 2 x max inscribed distance in a 2-voxel neighbourhood)"""
    e = ndimage.distance_transform_edt(vol) * VOX; loc = ndimage.maximum_filter(e, size=5) * 2.0
    return float(((loc < t_min) & vol).sum() / max(1, vol.sum())), float(np.percentile(loc[vol], 5))
def mesh_of(vol, ref=None):
    v, f, _, _ = measure.marching_cubes(np.pad(vol, 1).astype(np.float32), 0.5, spacing=(VOX,) * 3)
    m = trimesh.Trimesh(v + lo - VOX * 0.5, f)
    if m.volume < 0: m.invert()
    trimesh.smoothing.filter_taubin(m, iterations=8)
    if ref is not None:                                         # crisp facets, but never move a vertex on a clearance cut
        iv = np.clip(np.floor((m.vertices - lo) / VOX).astype(int), 0, n - 1)
        m = facet.snap(m, ref, tol=1.0, protect=(d_sw[tuple(iv.T)] < LIP_GAP + 1.6) | (D_all[tuple(iv.T)] < G_IN + 0.8))
    try:
        md = m.simplify_quadric_decimation(face_count=min(len(m.faces), 40000))
        if md.is_watertight and md.volume > 0: m = md
    except Exception: pass
    return m
cm = mesh_of(cowl, facet_mesh); cm.export(os.path.join(OUT, "cowl_R.stl"))
halves = {}
for nm, sel in (("a_posX", lambda c: c[0] > 0), ("b_negX", lambda c: c[0] < 0)):
    pcs = [p for p in cm.split(only_watertight=False) if sel(p.centroid) and abs(p.volume) > 1e3]
    h = trimesh.util.concatenate(pcs); h.export(os.path.join(OUT, f"cowl_R_{nm}.stl")); halves[nm] = h

# ---- humeral head dome (M2): sphere cap, bracket cut-out extruded along +y so the dome slides on along -y
r = np.sqrt(X * X + Y * Y + Z * Z)
# faceted humeral head: geodesic polyhedron (icosphere, 2 subdivisions = 320 facets) with its vertices on R 47.0, so every
# facet lies inside the 47.0 sphere (>= 3.0 mm to the yoke, which starts at r 50) and the inner sphere is 44.2.
ico = trimesh.creation.icosphere(subdivisions=2, radius=DOME_RO_VTX)
dome = facet.voxelize_like(ico, n, lo, VOX) & (r >= DOME_RI) & (r <= DOME_RO_VTX) & (Y >= Y_DOME)
br_only = ndimage.binary_dilation(solid(G["M2"]["s3_bracket"]), iterations=1)
br = br_only | ndimage.binary_dilation(solid(G["M2"]["s2_adapter"]), iterations=1)
shadow = np.flip(np.maximum.accumulate(np.flip(br, 1), axis=1), 1)     # voxels with bracket material at larger y (the dome slides on along -y)
dome &= ~shadow
# 4 radial ribs to the adapter rim (r 40) at y 5..14, azimuths free of the bracket, M3 radial into adapter-rim inserts
ribs_at = []
sh_b = np.flip(np.maximum.accumulate(np.flip(br_only, 1), axis=1), 1)     # bracket-only shadow (ribs may touch the adapter rim)
for az in (45.0, 135.0, 225.0, 315.0, 0.0, 90.0, 180.0, 270.0):
    c, s = np.cos(np.radians(az)), np.sin(np.radians(az))           # azimuth in the x-z plane
    u = X * c + Z * s; w = -X * s + Z * c
    rib = (u >= 40.2) & (u <= DOME_RI + 0.5) & (np.abs(w) <= 4.0) & (Y >= 5.0) & (Y <= 14.0)
    if (rib & sh_b).any() or len(ribs_at) >= 4: continue
    dome |= rib; ribs_at.append(az)
    hole = (np.abs(w) <= 1.7) & (np.abs(Y - 9.5) <= 1.7) & (u >= 39.0)
    dome &= ~hole
dome = ndimage.binary_opening(dome, iterations=1)
lab, nl = ndimage.label(dome, structure=np.ones((3, 3, 3))); sz = ndimage.sum(dome, lab, range(1, nl + 1)); dome = lab == 1 + int(np.argmax(sz))
dm = mesh_of(dome, ico); dm.export(os.path.join(OUT, "head_dome_R.stl"))

tf_c, p5_c = thin_frac(cowl)
info = dict(cowl_thin_wall_frac=round(tf_c, 4), cowl_wall_p5_mm=round(p5_c, 2), cowl_bbox=cm.bounds.round(1).tolist(), cowl_extents=cm.extents.round(1).tolist(), cowl_volume_cm3=round(cm.volume / 1e3, 1),
            cowl_mass_g_PETG=round(cm.volume / 1e3 * 1.27), cowl_watertight=bool(cm.is_watertight),
            halves={k: dict(volume_cm3=round(v.volume / 1e3, 1), watertight=bool(v.is_watertight), extents=v.extents.round(1).tolist()) for k, v in halves.items()},
            dome_bbox=dm.bounds.round(1).tolist(), dome_volume_cm3=round(dm.volume / 1e3, 1), dome_mass_g_PETG=round(dm.volume / 1e3 * 1.27),
            dome_watertight=bool(dm.is_watertight), tabs_deg=pick, laps=laps, dome_ribs_deg=ribs_at,
            params=dict(G_IN=G_IN, T_WALL=T_WALL, R_CLOSE=R_CLOSE, LIP_GAP=LIP_GAP, R_BORE=R_BORE, Z_BOT=Z_BOT, DOME=[DOME_RI, DOME_RO, Y_DOME]),
            old_ball=dict(diameter_mm=188.0, mass_g=146))
json.dump(info, open(os.path.join(OUT, "cowl.json"), "w"), indent=1)
print(json.dumps(info, indent=1))
