"""Shoulder 'ball': spherical shell centred on P (concurrent S1/S2/S3 axes), rotating with S1 (M1 frame).
Arm exits through a slot = sweep of all S2-moving parts (S2 -S2LO..S2HI) grown by CLR. SDF voxels + marching cubes.
Rev 2026-10-12c (I1): slot follows the new S2 hard range (-35..92, from rom_limits) -> smaller slot; TPU 85A slot lip on the
outer surface closes the arm/slot gap to 1 mm (no finger entry, ISO/TS 15066 pinch fix); 4 attachment ribs to the yoke disc.
Rev 2026-10-13: split into two halves at y = 0 with two lap joints (assembly), see below."""
import numpy as np, trimesh, os, sys, json
from scipy import ndimage
from skimage import measure
from trimesh.transformations import rotation_matrix as rot
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import sweep_shoulder as ss, rom_limits as rl
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "shoulder")
R_IN, T = float(sys.argv[1]) if len(sys.argv) > 1 else 92.0, float(sys.argv[2]) if len(sys.argv) > 2 else 3.5
R_OUT = R_IN + T; ZCUT = -57.5; CLR = 4.0; VOX = 0.9
S2LO, S2HI = rl.HARD["S2"][0] - 5.0, 95.0     # slot covers TPU compression beyond the hard stops (-35 -> about -40; +92 -> yoke backstop +94)
LIP_T, LIP_GAP, LIP_W = 1.6, 3.2, 5.0     # 3.2 mm: passes the 3 mm part gate and stays a < 4 mm slot (ISO 13857 fingertip)     # TPU lip thickness (on the outer surface), gap left to the swept arm, bond width beyond the slot edge
F, M1, M2 = ss.load_all()
lo = np.array([-R_OUT - 2] * 3); n = int(np.ceil((2 * R_OUT + 4) / VOX)); org = lo
idx = lambda p: np.floor((p - org) / VOX).astype(int)
occ = np.zeros((n, n, n), bool)
for t2 in np.arange(S2LO, S2HI + 0.1, 1.5):
    T2 = rot(np.radians(-t2), [0, 1, 0])
    for k, m in M2.items():
        mm = m.copy(); mm.apply_transform(T2)
        pts = mm.sample(int(mm.area / (VOX * VOX * 0.5)) + 200)
        i = idx(pts); ok = ((i >= 0) & (i < n)).all(1); occ[tuple(i[ok].T)] = True
        # also fill interior for solid check (surface is enough after dilation by CLR)
slot = ndimage.binary_dilation(occ, structure=ndimage.generate_binary_structure(3, 1), iterations=int(round(CLR / VOX)))
# slot must be a sweep volume, not only surfaces: fill it between surfaces along the arm plane through closing
slot = ndimage.binary_closing(slot, iterations=3)
keep_free = ndimage.binary_erosion(slot, iterations=int(round((CLR - LIP_GAP) / VOX)))     # = swept arm + LIP_GAP, interior filled
g = (np.arange(n) * VOX + org[0] + VOX / 2)
X, Y, Z = np.meshgrid(g, g, g, indexing="ij")
r = np.sqrt(X * X + Y * Y + Z * Z)
shell = (r >= R_IN) & (r <= R_OUT) & (Z >= ZCUT)
lip = (r >= R_OUT - 0.3) & (r <= R_OUT + LIP_T) & (Z >= ZCUT + 2.0) & ndimage.binary_dilation(slot, iterations=int(round(LIP_W / VOX))) & ~keep_free
# solid check for the lip: surface sampling does not fill the arm parts' interiors, so test lip voxels against each part's convex
# hull (inflated by LIP_GAP) at 1 deg steps; hull >= part, so this only ever removes more lip
li = np.argwhere(lip); LP = np.stack([X[lip], Y[lip], Z[lip]], 1); kill = np.zeros(len(LP), bool)
hulls = {k: m_.convex_hull for k, m_ in M2.items()}
for t2 in np.arange(S2LO, S2HI + 0.1, 1.0):
    Rt = rot(np.radians(t2), [0, 1, 0])[:3, :3]            # inverse of Ry(-t2): bring lip points into the S2 frame
    Q = LP @ Rt.T
    for k, h_ in hulls.items():
        nrm = h_.face_normals; off = np.einsum("ij,ij->i", nrm, h_.triangles[:, 0])
        kill |= np.all(Q @ nrm.T - off[None] <= LIP_GAP, axis=1)
lip[tuple(li[kill].T)] = False
shell &= ~slot
# lightening windows: round holes on a spherical lattice (angular pitch 13 deg, hole radius 4.3 deg), keep a solid 10 deg band at the base rim and 6 deg around the slot
th = np.degrees(np.arccos(np.clip(Z / np.maximum(r, 1e-6), -1, 1))); ph = np.degrees(np.arctan2(Y, X))
PITCH, HR = 13.0, 4.3
tc = np.round(th / PITCH) * PITCH; dth = th - tc
pc = np.round(ph * np.sin(np.radians(np.clip(tc, 1, 179))) / PITCH) * PITCH / np.sin(np.radians(np.clip(tc, 1, 179))); dph = (ph - pc) * np.sin(np.radians(np.clip(tc, 1, 179)))
hole = (dth ** 2 + dph ** 2) < HR ** 2
near_slot = ndimage.binary_dilation(slot, iterations=int(round(6.0 / VOX))) if False else ndimage.binary_dilation(slot, iterations=int(round(7.0 / VOX)))
hole &= (Z > ZCUT + 12.0) & ~near_slot & (tc > 8)
shell &= ~hole
shell &= ~(np.abs(Y) > 200)
# attachment ribs to the yoke adapter disc (r 50, z -56..-49.9): 4 ribs r 65..R_IN, z -54..-50.1, 6 wide, clear of the S1 lug (yoke-frame 257.5 deg)
RIB_ANG = (45.0, 135.0, 215.0, 315.0)
for a_ in RIB_ANG:
    ca, sa = np.cos(np.radians(a_)), np.sin(np.radians(a_)); u_ = X * ca + Y * sa; w_ = -X * sa + Y * ca
    shell |= (u_ >= 65.0) & (r <= R_IN + 0.5) & (np.abs(w_) <= 3.0) & (Z >= -54.0) & (Z <= -50.1) & ~slot      # r >= 65: 3 mm outside the stop posts (r <= 62)
    if a_ in (45.0, 135.0):   # no yoke box above these two: a bridge above the post tops (z -50.1..-46.8) to a tab on the disc top, M3 x 8 into an insert in the disc
        shell |= (u_ >= 43.0) & (u_ <= 68.0) & (np.abs(w_) <= 4.0) & (Z >= -49.8) & (Z <= -46.8)
# ribs at 215 / 315 deg sit under the yoke box floor (z -50.1, 0.1 mm shim gap) and take an M3 x 10 up into an insert in the box floor
# ---- rev 2026-10-13: SPLIT at y = 0 into two halves (assembly gate). A one-piece ball cannot be fitted over the finished
# stage: lowered from +z it hits the S2 clamp at 70 mm, from +-x the yoke (fcl test). Split through P perpendicular to the
# S2 axis, each half slides on along y over the stage (open face = y 0 disc, R 94 > every stage corner, max r 82.5), and the
# arm (y -20..26) crosses the seam inside the slot. Seam gap 1 voxel (0.9 mm). Two lap joints on the back seam arc
# (polar 70 / 110 deg from +z toward +x, beyond the S2 sweep which ends near 60 deg): an outer lap plate on the +y half over
# the -y half, M3 x 10 through the lap and wall into a heat-set insert in an inner boss on the -y half.
SEAM = 0.9          # half-gap: 2 voxels -> 1.8 mm seam, so marching cubes cannot bridge the halves diagonally
shell &= np.abs(Y) >= SEAM
lip &= np.abs(Y) >= SEAM
LAPS = (70.0, 110.0)
holes = np.zeros_like(shell)
for th_ in LAPS:
    c_ = np.array([np.sin(np.radians(th_)), 0.0, np.cos(np.radians(th_))])          # unit vector of the joint (y = 0 plane, +x side)
    ang = np.degrees(np.arccos(np.clip((X * c_[0] + Z * c_[2]) / np.maximum(np.sqrt(X * X + Z * Z), 1e-6), -1, 1)))
    near = (ang <= 7.0)                                                                   # +-7 deg along the seam (~ +-11.5 mm)
    lap = near & (r >= R_OUT + 2 * VOX) & (r <= R_OUT + 2 * VOX + 2.0) & (np.abs(Y) <= 8.0)      # outer plate, two voxels (1.8 mm, shimmed with 1.5 mm foam tape) off the -y wall
    lap |= near & (r >= R_OUT - 0.2) & (r < R_OUT + 2 * VOX + 0.1) & (Y >= SEAM + VOX) & (Y <= 8.0) # fused to the +y wall only
    shell |= lap
    sc = np.array([c_[0] * (R_IN - 3.0), -5.0, c_[2] * (R_IN - 3.0)])                   # boss centre (inner side, -y half)
    d_ = np.stack([X - sc[0], Y - sc[1], Z - sc[2]], -1); rad = d_ @ c_; perp = np.linalg.norm(d_ - rad[..., None] * c_, axis=-1)
    shell |= (perp <= 5.0) & (r >= R_IN - 6.5) & (r <= R_IN + 0.5) & (Y <= -SEAM)
    holes |= (perp <= 1.7) & (r >= R_IN - 1.0)                                           # M3 clearance through wall + lap
    holes |= (perp <= 2.2) & (r >= R_IN - 6.0) & (r < R_IN + 0.2)                       # insert pilot (OD 4.6 - 0.2)
shell &= ~holes
shell = ndimage.binary_opening(shell, iterations=1)
lab, nl = ndimage.label(shell, structure=np.ones((3, 3, 3))); sizes = ndimage.sum(shell, lab, range(1, nl + 1)); keep = 1 + np.argsort(sizes)[::-1][:2]
shell = np.isin(lab, keep)
print("ball pieces kept:", len(keep), "voxels", sorted(sizes)[::-1][:3])
v, f, _, _ = measure.marching_cubes(np.pad(shell, 1).astype(np.float32), 0.5, spacing=(VOX,) * 3)
v += org + VOX / 2 - VOX
m = trimesh.Trimesh(v, f)
if m.volume < 0: m.invert()
trimesh.smoothing.filter_taubin(m, iterations=6)
os.makedirs(os.path.join(OUT, "ball"), exist_ok=True)
try:
    md = m.simplify_quadric_decimation(face_count=120000)
    if md.is_watertight: m = md
    else: print("decimated mesh not watertight -> keeping full-resolution mesh")
except Exception as e:
    print("decimate failed", e)
m.export(os.path.join(OUT, "ball", "ball_shell_R.stl"))        # both halves in one file (for sweeps / renders)
for nm, sel in (("a_posY", lambda c: c[1] > 0), ("b_negY", lambda c: c[1] < 0)):
    pcs = [p_ for p_ in m.split(only_watertight=False) if sel(p_.centroid) and abs(p_.volume) > 1e3]
    if pcs:
        h = trimesh.util.concatenate(pcs); h.export(os.path.join(OUT, "ball", f"ball_shell_R_{nm}.stl"))
        print("half", nm, "volume cm3", round(h.volume / 1e3, 1), "watertight", h.is_watertight, "bbox", h.extents.round(1).tolist())
lip = lip.copy()
lab2, nl2 = ndimage.label(lip)
if nl2:
    sz2 = ndimage.sum(lip, lab2, range(1, nl2 + 1)); lip = np.isin(lab2, 1 + np.where(sz2 >= 40)[0]); lab2, nl2 = ndimage.label(lip)
if nl2:
    vv, ff, _, _ = measure.marching_cubes(np.pad(lip, 1).astype(np.float32), 0.5, spacing=(VOX,) * 3); vv += org + VOX / 2 - VOX
    ml = trimesh.Trimesh(vv, ff)
    if ml.volume < 0: ml.invert()
    trimesh.smoothing.filter_taubin(ml, iterations=4)
    ml.export(os.path.join(OUT, "ball", "ball_lip_R.stl"))
    print("lip (TPU 85A): pieces", nl2, "volume cm3", round(ml.volume / 1e3, 2), "mass g (TPU 1.21)", round(ml.volume / 1e3 * 1.21, 1))
print("R_in", R_IN, "faces", len(m.faces), "volume cm3", round(m.volume / 1e3, 1), "mass g (PETG 1.27)", round(m.volume / 1e3 * 1.27), "watertight", m.is_watertight)
print("bounds", m.bounds.round(1).tolist())
