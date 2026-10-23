"""Organic, low-profile connector: grows the clasp's ribbons / columns / bridge flanges as ONE smooth-blended signed-distance
solid (marching cubes), then fuses it with the pad plates (CadQuery facets) and re-cuts the small functional holes.
Everything is in the robot frame (right shoulder, mm).  Run after:  clasp_shoulder.py --facets_only   (writes clasp_paths.json etc.)"""
import sys, os, json, time, numpy as np, trimesh
from skimage import measure
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bodyprobe as bp
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "shoulder")
VOX = float(os.environ.get("VOX", 0.7))
G, PLATE_T, PLATE_X = 7.0, 3.5, -88.0      # G must match clasp_shoulder.py (rev 2026-10-12c: 5 -> 7)
paths = json.load(open(os.path.join(OUT, "clasp_paths.json")))
sites = json.load(open(os.path.join(OUT, "clasp_sites.json")))
feet = json.load(open(os.path.join(OUT, "clasp_feet.json")))
cc = json.load(open(os.path.join(OUT, "clasp_cradle.json")))
facets = trimesh.load(os.path.join(OUT, "clasp_facets_R.stl"))
body = bp.load()

# ribbon sections (width x thickness at start -> end), mm. Low profile: 5.5-6.5 mm thick (blocky v1 was 9-10 mm)
SEC = {"top": ((13.0, 5.5), (13.0, 5.5)), "arc": ((16.0, 6.5), (16.0, 6.5)), "bridge": ((14.0, 6.0), (24.0, 6.0))}
def sec_for(name):
    return SEC["arc"] if name.startswith("arc") else SEC["bridge"] if name.startswith("bridge") else SEC["top"]
K_BLEND = 6.0

# ---------------------------------------------------------------- SDF grid
allp = np.vstack([np.array(v) for v in paths.values()] + [facets.bounds])
lo = allp.min(0) - 22; hi = allp.max(0) + 22
lo[0] = min(lo[0], facets.bounds[0][0] - 3); hi[0] = max(hi[0], PLATE_X + 2)
shape = np.ceil((hi - lo) / VOX).astype(int) + 1
print("grid", shape, "= %.1f M voxels" % (np.prod(shape) / 1e6))
grid = np.full(shape, 60.0, np.float32)

def smin(a, b, k):
    h = np.maximum(k - np.abs(a - b), 0) / k
    return np.minimum(a, b) - h * h * k * 0.25

def window(pmin, pmax):
    i0 = np.maximum(np.floor((pmin - lo) / VOX).astype(int), 0); i1 = np.minimum(np.ceil((pmax - lo) / VOX).astype(int) + 1, shape)
    sl = tuple(slice(a, b) for a, b in zip(i0, i1))
    ax = [lo[d] + VOX * np.arange(i0[d], i1[d]) for d in range(3)]
    P = np.stack(np.meshgrid(*ax, indexing="ij"), -1).reshape(-1, 3)
    return sl, P, tuple(int(i1[d] - i0[d]) for d in range(3))

def add_ribbon(a, b, wa, wb, ta, tb, nref, k=K_BLEND):
    a = np.asarray(a, float); b = np.asarray(b, float); L = np.linalg.norm(b - a)
    if L < 1e-3: return
    e = (b - a) / L; n = nref - e * (nref @ e)
    if np.linalg.norm(n) < 1e-6: n = np.array([0, 0, 1.0]) - e * e[2]
    n /= np.linalg.norm(n); w = np.cross(e, n)
    pad = max(wa, wb, ta, tb) / 2 + k + 3
    sl, P, shp = window(np.minimum(a, b) - pad, np.maximum(a, b) + pad)
    d = P - a; s = d @ e; wc = d @ w; tc = d @ n
    u = np.clip(s / L, 0, 1); wr = (wa + (wb - wa) * u) / 2; tr = (ta + (tb - ta) * u) / 2
    q = np.sqrt((wc / wr) ** 2 + (tc / tr) ** 2)
    g = np.sqrt((wc / wr ** 2) ** 2 + (tc / tr ** 2) ** 2) / np.maximum(q, 1e-6)
    g = np.maximum(g, 1.0 / np.maximum(wr, tr))
    dcr = (q - 1) / g
    ds = np.abs(s - np.clip(s, 0, L))
    sdf = np.where((dcr > 0) | (ds > 0), np.sqrt(np.maximum(dcr, 0) ** 2 + ds ** 2), dcr)
    grid[sl] = smin(grid[sl], sdf.reshape(shp).astype(np.float32), k)

def add_rbox(c, half, r, k=8.0):
    c = np.asarray(c, float); half = np.asarray(half, float)
    sl, P, shp = window(c - half - k - 3, c + half + k + 3)
    q = np.abs(P - c) - half + r
    sdf = np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(1), 0) - r
    grid[sl] = smin(grid[sl], sdf.reshape(shp).astype(np.float32), k)

t0 = time.time()
# 1) ribbons along every routed path (waypoints included)
for name, pts in paths.items():
    (wa0, ta0), (wb0, tb0) = sec_for(name)
    P = [np.array(p, float) for p in pts]
    tot = sum(np.linalg.norm(q - p) for p, q in zip(P[:-1], P[1:])); run = 0.0
    for p, q in zip(P[:-1], P[1:]):
        L = np.linalg.norm(q - p); f0, f1 = run / tot, (run + L) / tot; run += L
        wa, wb = wa0 + (wb0 - wa0) * f0, wa0 + (wb0 - wa0) * f1
        ta, tb = ta0 + (tb0 - ta0) * f0, ta0 + (tb0 - ta0) * f1
        mid = (p + q) / 2
        cp, dd, fi = __import__("trimesh").proximity.closest_point(body, mid[None])
        nref = body.face_normals[fi[0]]
        if name.startswith("bridge"): nref = np.array([0.0, 0.0, 1.0]) if abs(q[2] - p[2]) < 0.5 * L else nref
        add_ribbon(p, q, wa, wb, ta, tb, nref)
# 2) columns that lift every pad plate to its ribbons (round, tapering 9 mm radius -> blends into the ribbon)
LIFT_NODE = {"F": 5.5 / 2 - 0.5, "R": 6.5 / 2 - 0.5}
for k, s in sites.items():
    p = np.array(s["p"]); n = np.array(s["n"])
    lift = 6.5 / 2 - 0.5 if k.startswith("R") else 5.5 / 2 - 0.5
    a = p + n * (G + PLATE_T - 1.2); b = p + n * (G + PLATE_T + lift)
    add_ribbon(a, b, 17.0, 14.0, 17.0, 11.0, np.cross(n, [0, 0, 1.0]) if abs(n[2]) < 0.9 else np.array([1.0, 0, 0]), k=5.0)
# 2b) rev 2026-10-13: bosses on the F1 / F4 nodes for the pauldron wrap-arms (pad seats 0.2 mm above the boss top, M4 x 12 into a
#     5.6 x 4 heat-set insert). Node centre = site + n (G + PLATE_T + LIFT); boss top = node + 6.8 n (pad centre node + 9.5 n, 5 thick)
LINK_NODES = ("F1_upper_trap", "F4_acromion"); BOSS_TOP = 6.8
for k in LINK_NODES:
    s_ = sites[k]; p_ = np.array(s_["p"]); n_ = np.array(s_["n"]); nd = p_ + n_ * (G + 3.5 + 5.5 / 2 - 0.5)
    add_ribbon(nd - n_ * 2.0, nd + n_ * BOSS_TOP, 24.0, 24.0, 24.0, 24.0, np.cross(n_, [1.0, 0, 0]) / np.linalg.norm(np.cross(n_, [1.0, 0, 0])), k=4.0)
# 3) bridge flange pads (4.5 mm), blended
for k, f in feet.items():
    add_rbox([PLATE_X - 2.25, f["y"], f["z"]], [2.25, 15.0, 21.0], 2.0, k=7.0)
# 4) keep-out: carve any material closer than SKIN_MARGIN to the skin (the armpit corridor is exempt: the base mesh has no real axilla there)
SKIN_MARGIN = 2.5
idx = np.argwhere(grid < 1.0)
Pn = lo + idx * VOX
# rev 2026-10-12c (I2): only the cradle face itself may touch (was a whole armpit corridor, which let the arc lie on the fold)
# rev 2026-10-13: cradle box tested in the cradle's tilted frame (rotation about x by -TILT through A)
Ac = np.array(cc["A"]); _t = np.radians(cc.get("TILT", 0.0)); _d = Pn - Ac
_ly = _d[:, 1] * np.cos(_t) - _d[:, 2] * np.sin(_t); _lz = _d[:, 1] * np.sin(_t) + _d[:, 2] * np.cos(_t)
cx = (np.abs(_d[:, 0]) < cc["CL"] / 2 + 1) & (np.abs(_lz) < cc["CH"] / 2 + 1) & (np.abs(_ly) < cc["CT"] / 2 + 4)
Pn, idx = Pn[~cx], idx[~cx]
lo_b, hi_b = Pn.min(0) - 20, Pn.max(0) + 20
keepf = np.all((body.triangles_center > lo_b) & (body.triangles_center < hi_b), axis=1)
bcrop = body.submesh([np.where(keepf)[0]], append=True)
carved = 0
for i0 in range(0, len(Pn), 200000):
    P_ = Pn[i0:i0 + 200000]
    cp_, d_, fi_ = trimesh.proximity.closest_point(bcrop, P_)
    sg_ = np.einsum("ij,ij->i", P_ - cp_, bcrop.face_normals[fi_]); sd_ = np.where(sg_ >= 0, d_, -d_)
    hit = sd_ < SKIN_MARGIN
    if hit.any():
        ii = idx[i0:i0 + 200000][hit]; grid[ii[:, 0], ii[:, 1], ii[:, 2]] = np.maximum(grid[ii[:, 0], ii[:, 1], ii[:, 2]], (SKIN_MARGIN - sd_[hit]).astype(np.float32) + 0.2); carved += int(hit.sum())
print("keep-out carved %d voxels" % carved)
print("sdf built %.1f s" % (time.time() - t0))

# ---------------------------------------------------------------- mesh it
pad = 2
g2 = np.pad(grid, pad, constant_values=60.0)
# rev 2026-10-13: marching cubes can leave a non-manifold edge on an ambiguous cell; retry at a slightly shifted iso level
for lev in (0.0, 0.03, -0.03, 0.07):
    v, f, nrm, _ = measure.marching_cubes(g2, level=lev, spacing=(VOX,) * 3, allow_degenerate=False)
    v = v + lo - pad * VOX
    skel = trimesh.Trimesh(v, f, process=True)
    skel.merge_vertices(); trimesh.smoothing.filter_taubin(skel, lamb=0.5, nu=-0.53, iterations=12)
    if skel.is_watertight: break
print("skeleton", len(skel.faces), "faces, watertight", skel.is_watertight, "vol %.1f cm3" % (skel.volume / 1e3))

# ---------------------------------------------------------------- fuse with the pad plates, then re-cut functional holes
body_m = trimesh.boolean.union([skel, facets], engine="manifold")

def frame(origin, ax_u, ax_v, ax_n):
    T = np.eye(4); T[:3, 0] = ax_u; T[:3, 1] = ax_v; T[:3, 2] = ax_n; T[:3, 3] = origin; return T
cuts = []
for k, s in sites.items():
    p = np.array(s["p"]); n = np.array(s["n"]); u = np.array(s["u"]); v_ = np.cross(n, u); o_in = p + n * G
    offs = [] if s["electrodes"] == 0 else ([0.0] if s["electrodes"] == 1 else [-10.0, 10.0])
    for c in offs:   # wire holes d 2.6 through plate + blend
        cyl = trimesh.creation.cylinder(radius=1.3, height=22.0, transform=frame(o_in + u * c + n * 7.0, u, v_, n)); cuts.append(cyl)
    if k == "F3_pec" and s.get("slot_edge") == "+u":   # rev 2026-10-14: 35 x 3.2 slot for the 32 mm front strap, long axis along v, 4.5 mm in from the +u end (strap arrives from +u)
        cuts.append(trimesh.creation.box(extents=(s["slot_t"], s["slot_l"], 22.0), transform=frame(o_in + u * (s["plate"][0] / 2 - 4.5) + n * 8.0, u, v_, n)))
    elif k in ("F3_pec", "F5_mid_trap"):   # 22 x 3 webbing slot, 4.5 mm in from the long edge
        cuts.append(trimesh.creation.box(extents=(22.0, 3.0, 22.0), transform=frame(o_in + v_ * (s["plate"][1] / 2 - 4.5) + n * 8.0, u, v_, n)))
for k in LINK_NODES:   # M4 heat-set insert pilot (5.6 - 0.2 = 5.4 mm) 6 mm deep from the boss top
    s_ = sites[k]; p_ = np.array(s_["p"]); n_ = np.array(s_["n"]); nd = p_ + n_ * (G + 3.5 + 5.5 / 2 - 0.5); u_ = np.cross(n_, [1.0, 0, 0]); u_ /= np.linalg.norm(u_)
    cuts.append(trimesh.creation.cylinder(radius=2.7, height=12.0, transform=frame(nd + n_ * BOSS_TOP, u_, np.cross(n_, u_), n_)))
fs = np.array(cc["front_slot"])
_tc = np.radians(-cc.get("TILT", 0.0))   # rev 2026-10-14: 3.2 x 35 slot for the 32 mm strap, tilted with the cradle plate
cuts.append(trimesh.creation.box(extents=(cc.get("slot_t", 3.2), 12.0, cc.get("slot_l", 22.0)), center=None,
            transform=trimesh.transformations.rotation_matrix(_tc, [1, 0, 0], fs) @ trimesh.transformations.translation_matrix([fs[0], fs[1] - 1.5, fs[2]])))
for k, f_ in feet.items():   # 2 vertical stadium slots 4.5 x 10 at +-10 (M4)
    for dz in f_["bolts_dz"]:
        T = trimesh.transformations.translation_matrix([PLATE_X - 2.25, f_["y"], f_["z"] + dz])
        cuts.append(trimesh.creation.box(extents=(12.0, 4.5, 10.0), transform=T))
        for sz in (-5.0, 5.0):
            Tc = trimesh.transformations.translation_matrix([PLATE_X - 2.25, f_["y"], f_["z"] + dz + sz]) @ trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])
            cuts.append(trimesh.creation.cylinder(radius=2.25, height=12.0, transform=Tc))
cuts.append(trimesh.creation.box(extents=(60.0, 400.0, 400.0), transform=trimesh.transformations.translation_matrix([PLATE_X - 0.2 + 30.0, -100.0, -150.0])))   # nothing may poke into the plate: feet stop 0.2 mm shy (bolted seat)
cut_u = trimesh.boolean.union(cuts, engine="manifold")
final = trimesh.boolean.difference([body_m, cut_u], engine="manifold")
final.merge_vertices()
parts = final.split(only_watertight=False)
print("bodies before drop:", sorted([(round(abs(p_.volume) / 1e3, 2), p_.bounds.round(0).tolist()) for p_ in parts], key=lambda t: -t[0])[:4])
final = max(parts, key=lambda p: abs(p.volume))        # drop degenerate zero-volume slivers left by the boolean
final.remove_unreferenced_vertices()
print("final: watertight", final.is_watertight, "vol %.1f cm3" % (final.volume / 1e3), "faces", len(final.faces), "bbox", final.bounds.round(0).tolist(), "bodies", len(final.split(only_watertight=False)))
final.export(os.path.join(OUT, "clasp_frame_R.stl"))
Lm = final.copy(); Lm.apply_transform(np.diag([1, -1, 1, 1.0]))
if Lm.volume < 0: Lm.invert()
Lm.export(os.path.join(OUT, "clasp_frame_L.stl"))
print("done %.0f s" % (time.time() - t0))
