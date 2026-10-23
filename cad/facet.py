"""Faceting helper (rev 2026-10-13, Forest: "facet it to my shoulder"). Turns a smooth voxel solid into a low-poly faceted solid
in the same voxel grid, so the existing voxel pipelines (cavities, slots, holes, splits) keep working unchanged.
  faceted(solid, lo, vox, n_faces): marching cubes -> watertight mesh -> quadric decimation to ~n_faces flat facets -> voxelize back.
The caller grows the smooth solid by a margin first (decimation chords cut up to ~1-2 mm inside a convex surface) and then
subtracts the cavities, so walls never get thinner than designed."""
import numpy as np, trimesh
from scipy import ndimage
from skimage import measure

def to_mesh(vol, lo, vox, smooth=6):
    v, f, _, _ = measure.marching_cubes(np.pad(vol, 1).astype(np.float32), 0.5, spacing=(vox,) * 3)
    m = trimesh.Trimesh(v + np.asarray(lo) - vox * 0.5, f)
    if m.volume < 0: m.invert()
    if smooth: trimesh.smoothing.filter_taubin(m, iterations=smooth)
    return m

def decimate(m, n_faces):
    for nf in (n_faces, int(n_faces * 1.3), int(n_faces * 1.7), int(n_faces * 2.5)):
        try:
            d = m.simplify_quadric_decimation(face_count=nf)
            if d.is_watertight and d.volume > 0: return d
        except Exception:
            pass
    return m

def voxelize_like(m, shape, lo, vox):
    """exact solid occupancy of a closed mesh on the grid (cell centres lo + vox*(i+0.5)): one ray per (x, z) column along +y,
    parity fill between sorted hits (no voxelizer aliasing, so planar facets stay planar)"""
    lo = np.asarray(lo, float); shape = tuple(shape)
    gx = lo[0] + vox * (np.arange(shape[0]) + 0.5); gy = lo[1] + vox * (np.arange(shape[1]) + 0.5); gz = lo[2] + vox * (np.arange(shape[2]) + 0.5)
    b0, b1 = m.bounds
    ix = np.where((gx >= b0[0]) & (gx <= b1[0]))[0]; iz = np.where((gz >= b0[2]) & (gz <= b1[2]))[0]
    out = np.zeros(shape, bool)
    if len(ix) == 0 or len(iz) == 0: return out
    IX, IZ = np.meshgrid(ix, iz, indexing="ij"); IX, IZ = IX.ravel(), IZ.ravel()
    org = np.c_[gx[IX] + 1e-4, np.full(len(IX), b0[1] - 5.0), gz[IZ] + 1e-4]; dirs = np.tile([0.0, 1.0, 0.0], (len(IX), 1))
    try:
        from trimesh.ray.ray_triangle import RayMeshIntersector
        loc, ri, _ = RayMeshIntersector(m).intersects_location(org, dirs, multiple_hits=True)
    except Exception:
        loc, ri, _ = m.ray.intersects_location(org, dirs, multiple_hits=True)
    order = np.lexsort((loc[:, 1], ri)); loc, ri = loc[order], ri[order]
    starts = np.searchsorted(ri, np.arange(len(IX))); ends = np.searchsorted(ri, np.arange(len(IX)), side="right")
    for k in range(len(IX)):
        ys = np.unique(np.round(loc[starts[k]:ends[k], 1], 6))
        for a_, b_ in zip(ys[0::2], ys[1::2]):
            j0 = int(np.ceil((a_ - lo[1]) / vox - 0.5)); j1 = int(np.floor((b_ - lo[1]) / vox - 0.5))
            if j1 >= j0: out[IX[k], max(j0, 0):min(j1 + 1, shape[1]), IZ[k]] = True
    return out

def faceted(solid, lo, vox, n_faces):
    m = decimate(to_mesh(solid, lo, vox), n_faces)
    return voxelize_like(m, solid.shape, lo, vox), m

def snap(m, ref, tol=1.0, protect=None):
    """move vertices of m that lie within tol of the faceted reference surface onto it, so voxel-stepped faces become crisp planes.
    protect: optional boolean mask of vertices that must not move (e.g. edges of clearance cuts)"""
    cp, d, _ = trimesh.proximity.closest_point(ref, m.vertices)
    sel = d < tol
    if protect is not None: sel &= ~protect
    v = m.vertices.copy(); v[sel] = cp[sel]
    out = trimesh.Trimesh(v, m.faces, process=False)
    return out if out.is_watertight else m


# ======================================================================================================================
# rev 2026-10-14 (strap/facet sprint): CONTOURED, EDGE-ROLLED load-pad plates.
# A flat plate G off a curved skin leaves a foam wedge (F1 3.2..11.7 mm, F2 4.3..20 mm); the thin cells are the stiff ones
# (k = E a / t) and carry the cell peaks.  A contoured plate follows the skin at a constant offset (t ~ G everywhere), its
# rim curls AWAY from the skin (cosine roll, ROLL_H over ROLL_W) so the edge foam is thicker/softer, and the rim is tapered.
# An optional TPU 85A skirt (SKIRT_W wide, SKIRT_T thick) is bonded to the skin face of the rim and rides on the foam.
# Local frame of a pad: a along u, b along v, z along the outward normal n, origin at the skin point p (z = 0 = tangent plane).
# ======================================================================================================================
from scipy.ndimage import gaussian_filter, distance_transform_edt
from scipy.interpolate import RegularGridInterpolator

def rise_profile(d, H, W):
    """cosine roll: H at the rim (d = 0), 0 and zero slope at d >= W (d = distance inward from the outer rim)"""
    d = np.asarray(d, float); x = np.clip(d / W, 0.0, 1.0)
    return H * 0.5 * (1.0 + np.cos(np.pi * x))

def rim_taper(d, dT=1.4, w=2.5):
    """thickness removed near the rim (rounded edge): dT at d = 0, 0 at d >= w"""
    x = np.clip(np.asarray(d, float) / w, 0.0, 1.0); return dT * (1.0 - x) ** 2

def outline_poly(w, h, fillet, cu=0.0, cv=0.0):
    from shapely.geometry import box
    f = min(fillet, min(w, h) / 2 - 1.0)
    return box(cu - w / 2 + f, cv - h / 2 + f, cu + w / 2 - f, cv + h / 2 - f).buffer(f, resolution=24)

def skin_height_grid(body, p, n, u, v, a0, a1, b0, b1, step=1.0):
    """skin height z (mm along n above the skin point p) on a regular (a, b) grid: the outermost surface point along -n.
    Triangles near the patch are rasterised onto the grid (barycentric test in the (a, b) projection, max z wins)."""
    aa = np.arange(a0, a1 + 1e-6, step); bb = np.arange(b0, b1 + 1e-6, step)
    Z = np.full((len(aa), len(bb)), -np.inf)
    R = np.stack([u, v, n]); ctr = p + 0.5 * ((a0 + a1) * u + (b0 + b1) * v); rad = 0.5 * np.hypot(a1 - a0, b1 - b0) + 10.0
    tc = body.triangles_center; sel = np.where(np.linalg.norm(tc - ctr, axis=1) < rad + 40.0)[0]
    T = (body.triangles[sel] - p[None, None, :]) @ R.T            # (k, 3, 3) -> (a, b, z) coordinates
    for t in T:
        amin, amax = t[:, 0].min(), t[:, 0].max(); bmin, bmax = t[:, 1].min(), t[:, 1].max()
        i0, i1 = np.searchsorted(aa, amin - 1e-9), np.searchsorted(aa, amax + 1e-9); j0, j1 = np.searchsorted(bb, bmin - 1e-9), np.searchsorted(bb, bmax + 1e-9)
        if i1 <= i0 or j1 <= j0: continue
        A_, B_ = np.meshgrid(aa[i0:i1], bb[j0:j1], indexing="ij")
        d = (t[1, 1] - t[2, 1]) * (t[0, 0] - t[2, 0]) + (t[2, 0] - t[1, 0]) * (t[0, 1] - t[2, 1])
        if abs(d) < 1e-9: continue
        l1 = ((t[1, 1] - t[2, 1]) * (A_ - t[2, 0]) + (t[2, 0] - t[1, 0]) * (B_ - t[2, 1])) / d
        l2 = ((t[2, 1] - t[0, 1]) * (A_ - t[2, 0]) + (t[0, 0] - t[2, 0]) * (B_ - t[2, 1])) / d
        l3 = 1.0 - l1 - l2
        m = (l1 >= -1e-9) & (l2 >= -1e-9) & (l3 >= -1e-9)
        if not m.any(): continue
        z = l1 * t[0, 2] + l2 * t[1, 2] + l3 * t[2, 2]
        sub = Z[i0:i1, j0:j1]; sub[m] = np.maximum(sub[m], z[m])
    z = np.where(np.isfinite(Z), Z, np.nan)
    if np.isnan(z).any():
        idx = distance_transform_edt(np.isnan(z), return_distances=False, return_indices=True); z = z[tuple(idx)]
    return aa, bb, z

def _tri_prism(P, T, fz, spacing=1.6, edge_step=1.0):
    """watertight solid: Delaunay triangulation of polygon P (shapely, may have holes), extruded T, vertices warped z -> z + fz(a, b, d_edge)
    (fz returns bottom offset and thickness reduction per vertex)"""
    from scipy.spatial import Delaunay
    import shapely
    pts = []
    rings = [P.exterior] + list(P.interiors)
    for r in rings:
        L = r.length; m = max(8, int(np.ceil(L / edge_step)))
        for s in np.linspace(0, L, m, endpoint=False):
            q = r.interpolate(s); pts.append((q.x, q.y))
    b0, b1 = np.array(P.bounds[:2]), np.array(P.bounds[2:])
    gx = np.arange(b0[0], b1[0] + spacing, spacing); gy = np.arange(b0[1], b1[1] + spacing, spacing)
    G_ = np.array([(x + (0.5 * spacing if (j % 2) else 0.0), y) for j, y in enumerate(gy) for x in gx])
    keep = shapely.contains(P, shapely.points(G_)) & (shapely.distance(shapely.points(G_), P.boundary) > 0.55 * spacing)
    pts = np.vstack([np.array(pts), G_[keep]])
    tri = Delaunay(pts)
    cen = pts[tri.simplices].mean(1); inside = shapely.contains(P, shapely.points(cen))
    F = tri.simplices[inside]
    A2 = pts[F]; sgn = (A2[:, 1, 0] - A2[:, 0, 0]) * (A2[:, 2, 1] - A2[:, 0, 1]) - (A2[:, 1, 1] - A2[:, 0, 1]) * (A2[:, 2, 0] - A2[:, 0, 0])
    F = np.where((sgn > 0)[:, None], F, F[:, [0, 2, 1]])                  # ccw
    N = len(pts)
    # boundary edges (used once)
    E = np.sort(np.c_[F[:, [0, 1]], F[:, [1, 2]], F[:, [2, 0]]].reshape(-1, 2), axis=1)
    key = E[:, 0] * N + E[:, 1]; uq, cnt = np.unique(key, return_counts=True)
    bset = set(uq[cnt == 1].tolist())
    bedges = [(int(a), int(b)) for tri_ in F for a, b in ((tri_[0], tri_[1]), (tri_[1], tri_[2]), (tri_[2], tri_[0])) if (min(a, b) * N + max(a, b)) in bset]
    d_edge = shapely.distance(shapely.points(pts), P.exterior)
    zb, th = fz(pts[:, 0], pts[:, 1], d_edge)
    Vb = np.c_[pts, zb]; Vt = np.c_[pts, zb + T - th]
    V = np.vstack([Vb, Vt])
    faces = [F[:, [0, 2, 1]], F + N]
    for a, b in bedges:
        faces.append(np.array([[a, b, b + N], [a, b + N, a + N]]))
    m = trimesh.Trimesh(V, np.vstack(faces), process=False)
    m.merge_vertices(); m.remove_unreferenced_vertices()
    if m.volume < 0: m.invert()
    if not m.is_watertight:        # pinched vertex / sliver triangle at a concave notch: drop the offending triangles, refill holes
        for _ in range(4):
            e = np.sort(m.edges, axis=1); u_, inv, cnt = np.unique(e, axis=0, return_inverse=True, return_counts=True)
            badE = u_[cnt != 2]
            if len(badE) == 0: break
            badv = set(badE.ravel().tolist()); keep = np.array([not (set(fc).issubset(badv)) for fc in m.faces])
            m = trimesh.Trimesh(m.vertices, m.faces[keep], process=False); m.remove_unreferenced_vertices()
        trimesh.repair.fill_holes(m); trimesh.repair.fix_normals(m)
        if m.volume < 0: m.invert()
    return m

def contoured_pad(body, p, n, u, v, w, h, fillet, G, T, cu=0.0, cv=0.0, reliefs=(), halos=(), roll_H=3.0, roll_W=8.0,
                  sigma=3.0, bosses=(), boss_d=17.0, boss_h=5.0, pocket_d=12.4, pocket_depth=4.0, wire_d=2.6,
                  skirt_w=0.0, skirt_t=1.5, cut_boxes=(), avoid=(), avoid_margin=6.0, foam_relief=()):
    """Build one contoured, edge-rolled load-pad plate in the pad's local frame (a, b, z).
    reliefs: [(a, b, r)] circular windows through plate AND foam; halos: [(a, b, r)] discs kept solid inside a relief (not used for the AC joint any more).
    foam_relief: [(a, b, r)] discs where the load FOAM is die-cut away (plate stays continuous); the bearing mask / area exclude them (AC-joint relief).
    Returns dict(mesh=local-frame trimesh, skirt=local-frame trimesh|None, grid=dict for the load model, stats)."""
    from shapely.geometry import Point
    P0 = outline_poly(w, h, fillet, cu, cv)
    R = None
    for (ra, rb, rr) in reliefs:
        c = Point(ra, rb).buffer(rr, resolution=32)
        for (ha, hb, hr) in halos: c = c.difference(Point(ha, hb).buffer(hr, resolution=32))
        R = c if R is None else R.union(c)
    P = P0.difference(R) if R is not None else P0
    if P.geom_type != "Polygon": P = max(P.geoms, key=lambda g: g.area)
    M = skirt_w + 4.0
    a0, a1, b0, b1 = P0.bounds[0] - M, P0.bounds[2] + M, P0.bounds[1] - M, P0.bounds[3] + M
    aa, bb, zs = skin_height_grid(body, p, n, u, v, a0, a1, b0, b1, 1.0)
    zf = gaussian_filter(zs, sigma, mode="nearest")
    ia, ib = np.abs(aa).argmin(), np.abs(bb).argmin()
    z_in = G + (zf - zf[ia, ib])                                      # plate inner face (datum G at the pad centre), follows the skin
    if avoid:                                  # iteratively trim the outline where the plate (incl. its own rim roll) would come within avoid_margin of an already-built plate
        from shapely import contains_xy as _cx, distance as _dist, points as _pts
        from shapely.geometry import box as _box
        from shapely.ops import unary_union
        AA0, BB0 = np.meshgrid(aa, bb, indexing="ij")
        for _it in range(6):
            ins0 = _cx(P, AA0, BB0)
            d0 = _dist(_pts(np.c_[AA0.ravel(), BB0.ravel()]), P.exterior).reshape(AA0.shape)
            zt = z_in + rise_profile(d0, roll_H, roll_W)
            sel = np.where(ins0.ravel())[0]; bad = np.zeros(AA0.size, bool)
            for zoff in (0.0, T):
                X = p[None] + AA0.ravel()[sel, None] * u[None] + BB0.ravel()[sel, None] * v[None] + (zt.ravel()[sel] + zoff)[:, None] * n[None]
                dm = np.min([trimesh.proximity.closest_point(am, X)[1] for am in avoid], axis=0)
                bad[sel[dm < avoid_margin]] = True
            if not bad.any(): break
            ia_, ib_ = np.where(bad.reshape(AA0.shape))
            cut = unary_union([_box(aa[i] - 0.75, bb[j] - 0.75, aa[i] + 0.75, bb[j] + 0.75) for i, j in zip(ia_, ib_)])
            P_new = P.difference(cut).buffer(-1.5, resolution=8).buffer(1.5, resolution=8)
            if P_new.geom_type != "Polygon": P_new = max(P_new.geoms, key=lambda g: g.area)
            P = P_new
    from shapely.geometry import Polygon as _Poly
    P0 = _Poly(P.exterior)                      # outer outline after trimming (roll, skirt and the AC-fill ablation follow it)
    f_in = RegularGridInterpolator((aa, bb), z_in, bounds_error=False, fill_value=None)
    f_sk = RegularGridInterpolator((aa, bb), zs, bounds_error=False, fill_value=None)
    def fz(a, b, d):
        return f_in(np.c_[a, b]) + rise_profile(d, roll_H, roll_W), rim_taper(d)
    plate = _tri_prism(P, T, fz)
    # bosses / pockets / wire holes (electrode geometry identical to the flat facet: tip at z = G - boss_h above the skin point)
    parts = [plate]; cuts = []
    for (ba, bb_) in bosses:
        zc = float(f_in([[ba, bb_]])[0])
        tip = G - boss_h
        cyl = trimesh.creation.cylinder(radius=boss_d / 2, height=(zc + 1.5) - tip, sections=48,
                                        transform=trimesh.transformations.translation_matrix([ba, bb_, tip + ((zc + 1.5) - tip) / 2]))
        parts.append(cyl)
        cuts.append(trimesh.creation.cylinder(radius=pocket_d / 2, height=pocket_depth, sections=40,
                                              transform=trimesh.transformations.translation_matrix([ba, bb_, tip + pocket_depth / 2])))
        cuts.append(trimesh.creation.cylinder(radius=wire_d / 2, height=40.0, sections=24,
                                              transform=trimesh.transformations.translation_matrix([ba, bb_, tip + pocket_depth + 20.0 - 0.01])))
    m = trimesh.boolean.union(parts, engine="manifold") if len(parts) > 1 else plate
    if cuts: m = trimesh.boolean.difference([m, trimesh.boolean.union(cuts, engine="manifold")], engine="manifold")
    for (c_, e_) in cut_boxes:                     # extra through-cuts in the pad frame (strap slot): (centre a, b, z), (extent a, b, z)
        m = trimesh.boolean.difference([m, trimesh.creation.box(extents=e_, transform=trimesh.transformations.translation_matrix(c_))], engine="manifold")
    # foam thickness field for the load model (t = inner face incl. roll - skin), plus the bearing mask
    from shapely import contains_xy, distance, points
    AA, BB = np.meshgrid(aa, bb, indexing="ij")
    inside = contains_xy(P, AA, BB)
    inside_full = inside.copy()
    for (ra, rb, rr) in foam_relief: inside &= (np.hypot(AA - ra, BB - rb) > rr)
    dd = distance(points(np.c_[AA.ravel(), BB.ravel()]), P0.exterior).reshape(AA.shape)
    t_f = z_in + rise_profile(dd, roll_H, roll_W) - zs
    out = dict(mesh=m, plate_only=plate, P=P, P0=P0, grid=dict(a=aa, b=bb, t=t_f, inside=inside, zin=z_in, skin=zs, rise=rise_profile(dd, roll_H, roll_W), inside0=inside_full))
    st = t_f[inside]
    out["stats"] = dict(foam_t_min=float(st.min()), foam_t_max=float(st.max()), foam_t_mean=float(st.mean()), area_mm2=float(P.area) - float((inside_full & ~inside).sum()), plate_area_mm2=float(P.area),
                        plate_vol_mm3=float(m.volume), z_in_range=[float(z_in[inside].min()), float(z_in[inside].max())])
    # TPU skirt: ring from 2 mm under the rim to skirt_w outside; skin-side face rides on the foam (1.5 mm of the foam height is the skirt)
    out["skirt"] = None
    if skirt_w > 0:
        ring = P0.buffer(skirt_w, resolution=16).difference(P0.buffer(-2.0, resolution=16))
        if ring.geom_type != "Polygon": ring = max(ring.geoms, key=lambda g: g.area)
        def fzs(a, b, d_):
            q = np.c_[a, b]; dist_in = distance(points(q), P0.exterior); ins = contains_xy(P0, a, b)
            dsig = np.where(ins, dist_in, -dist_in)
            base = f_in(q) + rise_profile(np.maximum(dsig, 0.0), roll_H, roll_W)       # skin face of the plate at the rim (flat continuation outside)
            return base - skirt_t, np.zeros(len(a))
        out["skirt"] = _tri_prism(ring, skirt_t, fzs)
        # skirt cells for the load model: ring band outside the rim, cell area = band area / count; foam gap under the skirt
        band = P0.buffer(skirt_w, resolution=16).difference(P0)
        ins_b = contains_xy(band, AA, BB)
        tsk = (z_in + roll_H - skirt_t) - zs
        out["grid"]["skirt_mask"] = ins_b; out["grid"]["skirt_t"] = tsk
        out["stats"]["skirt_area_mm2"] = float(band.area); out["stats"]["skirt_foam_t_min"] = float(tsk[ins_b].min())
    return out
