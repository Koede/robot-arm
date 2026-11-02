"""Shared concept-study library (Third Shoulder, SHOULDER mount alternatives, 2026-11-01).  Robot frame, right arm, mm.
+x rearward, +y wearer right, +z up, P = (0,0,0).  Body at -x (back surface x ~ -108).  Everything here is a PROXY for the
generated parts (the verified STLs live in the GitHub repo, which was not attachable this session): dimensions come from the
specs (S1 base block, AK70, plate outline seeds, spaulder envelope), not from the verified meshes."""
import numpy as np, trimesh, os, json, shapely.geometry as sg
from scipy.spatial import cKDTree
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter
from trimesh.transformations import rotation_matrix as rot, translation_matrix as tr
ROOT = os.environ.get("J1R2_ROOT", os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # work root holding stl/proxies (wearer), stl/j1r2, render/ (see docs/ENV_SETUP.md)
DENS = dict(PACF=1.15, PETGCF=1.30, PETG=1.27, TPU=1.21, AL=2.70, STEEL=7.85, TI=4.43)   # g/cm3 (PA-CF 1.15 per SPEC_shoulder_housing)
COL = dict(body=(0.86, 0.74, 0.66), stage=(0.55, 0.57, 0.60), motor=(0.20, 0.21, 0.23), al=(0.78, 0.80, 0.83), pacf=(0.16, 0.18, 0.24),
           accent=(0.95, 0.45, 0.10), teal=(0.10, 0.62, 0.62), tpu=(0.85, 0.20, 0.20), steel=(0.45, 0.47, 0.5), lime=(0.55, 0.80, 0.15))
_B = {}
def body():
    if "m" not in _B: _B["m"] = trimesh.load(os.path.join(ROOT, "stl/proxies/wearer_makehuman.stl"))
    return _B["m"]
LM = json.load(open(os.path.join(ROOT, "stl/proxies/wearer_landmarks.json")))
# ------------------------------------------------------------------ primitives
def box(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0], transform=tr([(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]))
def cyl(axis, r, a0, a1, c=(0.0, 0.0), n=72, r_in=None):
    """cylinder / tube about axis ('x','y','z') from a0 to a1; c = centre in the two other coords (x:(y,z) y:(x,z) z:(x,y))."""
    h = a1 - a0
    m = trimesh.creation.annulus(r_in, r, h, sections=n) if r_in else trimesh.creation.cylinder(r, h, sections=n)
    m.apply_translation([0, 0, (a0 + a1) / 2 - 0.0])            # along z first
    if axis == "z": m.apply_translation([c[0], c[1], 0])
    elif axis == "x":  m.apply_transform(rot(np.pi / 2, [0, 1, 0])); m.apply_translation([0, c[0], c[1]])   # z->x
    elif axis == "y":  m.apply_transform(rot(-np.pi / 2, [1, 0, 0])); m.apply_translation([c[0], 0, c[1]])  # z->y
    return m
def sph(r, c=(0, 0, 0), n=3): m = trimesh.creation.icosphere(n, r); m.apply_translation(c); return m
def U(*ms): return trimesh.boolean.union([m for m in ms], engine="manifold")
def D(a, *bs):
    for b in bs: a = trimesh.boolean.difference([a, b], engine="manifold")
    return a
def I(a, b): return trimesh.boolean.intersection([a, b], engine="manifold")
def prism(poly, axis, a0, a1):
    """extrude a 2D polygon [(u,v)..] along axis: x:(y,z)  y:(x,z)  z:(x,y)  from a0 to a1."""
    p = sg.Polygon(poly).buffer(0)
    m = trimesh.creation.extrude_polygon(p, a1 - a0)            # along z, from 0..h
    if axis == "z": m.apply_translation([0, 0, a0])
    elif axis == "x": m.apply_transform(np.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1.0]])); m.apply_translation([a0, 0, 0])   # (u,v,w)->(w,u,v)
    elif axis == "y": m.apply_transform(np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]])); m.apply_translation([0, a1, 0])   # (u,v,w)->(u,-w,v)
    return m
def hull(*ms): return trimesh.convex.convex_hull(np.vstack([m.vertices for m in ms]))
def solid_fix(m):
    m.merge_vertices(); m.update_faces(m.nondegenerate_faces()); m.remove_unreferenced_vertices(); trimesh.repair.fix_normals(m)
    if m.is_watertight and m.volume < 0: m.invert()
    return m
# ------------------------------------------------------------------ body-conforming surfaces
def _vgrid(axis_dir, u0, u1, v0, v1, step):
    """true first-hit height field by ray casting.  'x': rear surface x(y,z) (rays from x=+80 toward -x); 'z': top surface z(x,y) (rays from z=-45 downward, below the head/neck)."""
    uu = np.arange(u0, u1 + 1e-6, step); vv = np.arange(v0, v1 + 1e-6, step); Ug, Vg = np.meshgrid(uu, vv, indexing="ij"); q = np.c_[Ug.ravel(), Vg.ravel()]
    if axis_dir == "x": o = np.c_[np.full(len(q), 80.0), q]; d = np.tile([-1.0, 0, 0], (len(q), 1)); col = 0
    else: o = np.c_[q, np.full(len(q), -45.0)]; d = np.tile([0, 0, -1.0], (len(q), 1)); col = 2
    H = np.full(len(q), np.nan); loc, ir, _ = body().ray.intersects_location(o, d, multiple_hits=False); H[ir] = loc[:, col]
    H = H.reshape(Ug.shape)
    if np.isnan(H).any():
        ok = ~np.isnan(H); H = griddata(np.c_[Ug[ok], Vg[ok]], H[ok], (Ug, Vg), method="nearest")
    return uu, vv, Ug, Vg, H
def rear_surface(y0, y1, z0, z1, step=4.0, smooth=1.5):
    """x of the back surface over (y,z); returns grids (Y,Z,X)."""
    uu, vv, Y, Z, X = _vgrid("x", y0, y1, z0, z1, step); return Y, Z, gaussian_filter(X, smooth / step * 2, mode="nearest")
def top_surface(x0, x1, y0, y1, step=4.0, smooth=1.5):
    uu, vv, X, Y, H = _vgrid("z", x0, x1, y0, y1, step); return X, Y, gaussian_filter(H, smooth / step * 2, mode="nearest")
def height_solid(Ug, Vg, H0, H1, axis):
    """closed solid between two height fields. axis 'x': (u,v)=(y,z), h along x ; axis 'z': (u,v)=(x,y), h along z."""
    nu, nv = Ug.shape
    def P(H):
        return np.c_[H.ravel(), Ug.ravel(), Vg.ravel()] if axis == "x" else np.c_[Ug.ravel(), Vg.ravel(), H.ravel()]
    Vt = np.vstack([P(H0), P(H1)]); n = nu * nv; idx = lambda i, j: i * nv + j; F = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b, c, d = idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)
            F += [[a, b, c], [a, c, d], [n + a, n + c, n + b], [n + a, n + d, n + c]]
    for i in range(nu - 1):                                   # sides v = 0 and v = end
        for j in (0, nv - 1):
            a, b = idx(i, j), idx(i + 1, j); F += [[a, n + b, b], [a, n + a, n + b]] if j == 0 else [[a, b, n + b], [a, n + b, n + a]]
    for j in range(nv - 1):
        for i in (0, nu - 1):
            a, b = idx(i, j), idx(i, j + 1); F += [[a, b, n + b], [a, n + b, n + a]] if i == 0 else [[a, n + b, b], [a, n + a, n + b]]
    m = trimesh.Trimesh(Vt, F, process=True); trimesh.repair.fix_normals(m)
    if m.volume < 0: m.invert()
    return m
def draped_back_plate(y0, y1, z0, z1, gap, thick, step=4.0, outline=None, smooth=3.0):
    """printable plate riding `gap` mm off the back surface (rear surface smoothed `smooth` mm), `thick` thick. optional outline polygon in (y,z)."""
    Y, Z, X = rear_surface(y0, y1, z0, z1, step, smooth)
    m = height_solid(Y, Z, X + gap, X + gap + thick, "x")
    if outline is not None: m = I(m, prism(outline, "x", -400, 100))
    return m
# ------------------------------------------------------------------ clearance / metrics
_K = {}
def _tree():
    if "t" not in _K:
        b = body(); lo = np.array([-330, -330, -520.]); hi = np.array([160, 250, 330.])
        keep = np.all((b.triangles_center > lo) & (b.triangles_center < hi), axis=1); sub = b.submesh([np.where(keep)[0]], append=True)
        pts, fi = trimesh.sample.sample_surface_even(sub, 450000)[:2] if False else trimesh.sample.sample_surface(sub, 450000)
        _K["t"] = (cKDTree(pts), pts, sub.face_normals[fi])
    return _K["t"]
def skin_gap(m, n=6000):
    """(min signed distance mm to the wearer, mean of the lowest 1 %).  negative = inside the body.  Sample-based (~0.8 mm resolution)."""
    t, pts, nrm = _tree(); s = trimesh.sample.sample_surface(m, n)[0]; d, i = t.query(s)
    sd = np.sign(np.einsum("ij,ij->i", s - pts[i], nrm[i])) * d; return float(sd.min()), float(np.sort(sd)[:max(1, n // 100)].mean())
def part_info(name, spec):
    m = spec["mesh"]; bb = m.bounds; ext = bb[1] - bb[0]; vol = m.volume / 1000.0 if m.is_watertight else float("nan")
    g = spec.get("mass_g"); g = g if g is not None else vol * DENS[spec.get("mat", "PACF")] * spec.get("fill", 1.0)
    d = dict(name=name, mat=spec.get("mat"), vol_cm3=round(vol, 2), mass_g=round(g, 1), bbox_mm=[round(float(x), 1) for x in ext], watertight=bool(m.is_watertight),
             fits_256=bool(np.all(m.bounding_box_oriented.extents <= 256.0)), group=spec.get("group", "mount"), contact=spec.get("contact", False))
    if spec.get("group", "mount") != "stage":
        sd = skin_gap(m, 3000); d["skin_min_mm"] = round(sd[0], 1)
        if spec.get("group") in ("joint", "armour", "bought", "mount"): d["protrusion_mm"] = round(protrusion(m), 1)
    return d
# ------------------------------------------------------------------ stage proxy (identical for every concept; render only, not in mount mass)
PARK = dict(S1=45.0, S2=25.0, E=45.0)
def stage_parts(S1=PARK["S1"], S2=PARK["S2"], E=PARK["E"], arm=True):
    """grey proxies of the (unchanged) S1/S2/S3 stage: AK70 S1 + yoke + AK70 S2 + cowl + dome + S3 + upper arm + ball elbow + forearm stub.
    S1 about z (R=Rz(-S1)), S2 about y (Ry(-S2)).  Returns {name: (mesh, colour, group)}"""
    out = {}
    out["ak70_S1"] = (cyl("z", 44.5, -106.75, -56.5), COL["motor"], "fixedstage")
    M1 = []
    M1.append(cyl("z", 50.0, -56.5, -49.9)); M1.append(box(-60.6, 60.6, -46, -4, -50, 0))
    M1.append(cyl("y", 44.5, -50.25, 0))                                                # AK70 S2
    cowl = D(cyl("y", 62.0, -58, 12.0), cyl("y", 59.0, -56, 14.0), box(-9, 9, -70, 30, 20, 80))
    M1c = U(*M1)
    M2 = [cyl("y", 40, 0.5, 17.05), box(-32.55, 32.55, -20.05, 20.05, 68, 112), cyl("z", 32.0, 112, 230 - 38)]
    dome = I(sph(46.8), box(-60, 60, 0, 60, -60, 60))
    ball = sph(44.0, (0, 0, 230)); arm_ = [m for m in M2] + [dome]
    if arm:
        fore = cyl("z", 28, 230 + 30, 230 + 180, c=(0, 0)); fore.apply_translation([0, 0, 0])
        wrist = sph(24, (0, 0, 410)); hand = box(-30, 30, -30, 30, 425, 520)
    T_S2 = rot(-np.radians(S2), [0, 1, 0]); T_S1 = rot(-np.radians(S1), [0, 0, 1])
    def mv1(m): m = m.copy(); m.apply_transform(T_S1); return m
    def mv2(m): m = m.copy(); m.apply_transform(T_S1 @ T_S2); return m
    out["yoke_S2"] = (mv1(M1c), COL["stage"], "movingstage"); out["cowl"] = (mv1(cowl), COL["stage"], "movingstage")
    out["S3_bracket"] = (mv2(U(*M2)), COL["motor"], "movingstage"); out["dome"] = (mv2(dome), COL["stage"], "movingstage")
    if arm:
        T_E = tr([0, 0, 230]) @ rot(np.radians(-E), [0, 1, 0]) @ tr([0, 0, -230])    # E>0 swings the forearm toward -x
        f = U(fore, wrist, hand); f.apply_transform(T_E); b2 = ball.copy()
        out["ball_elbow"] = (mv2(b2), COL["stage"], "movingstage"); out["forearm_hand"] = (mv2(f), COL["stage"], "movingstage")
    return out
# ------------------------------------------------------------------ baseline interface constants (specs)
S1_BASE_BLOCK = dict(x=(-80.0, 0.0), y=(-68.0, 68.0), z=(-100.0, -62.0), bolts=[(y, z) for y in (-62.0, 62.0) for z in (-94.0, -68.0)])  # 4 x M5 into the plate rear face
PLATE_X = (-88.0, -80.0)          # chassis plate x range (front face seats the clasp feet, rear face seats the S1 base)
TONGUE = (-78.0, -46.0)           # wing tongue seat y range (spine plane y = -150, y_rel 72..104), z -150..-76
SPINE_Y = -150.0
# ------------------------------------------------------------------ export + metrics
def export_concept(cid, title, parts, notes=None, extra=None):
    """parts: {name: dict(mesh, mat, group('mount'|'armour'|'stage-iface'), colour, contact(bool), mass_g(optional))}.  Writes stl/concepts/<cid>/*.stl, a combined posed view STL, metrics.json."""
    d = os.path.join(ROOT, "stl", "concepts", cid); os.makedirs(d, exist_ok=True); rows = []
    for k, s in parts.items():
        s["mesh"].export(os.path.join(d, f"{cid}_{k}_R.stl")); rows.append(part_info(k, s))
    mount_g = round(sum(r["mass_g"] for r in rows if r["group"] in ("mount", "armour", "joint", "bought")), 1)
    cols = {k: [float(x) for x in s["colour"]] for k, s in parts.items() if s.get("colour") is not None}; json.dump(cols, open(os.path.join(d, "colours.json"), "w"))
    meta = dict(id=cid, title=title, parts=rows, mount_mass_g=mount_g, notes=notes or {}, extra=extra or {})
    json.dump(meta, open(os.path.join(d, "metrics.json"), "w"), indent=1)
    return meta

# ------------------------------------------------------------------ shared environment proxies (UNCHANGED contract in every concept)
def seats():
    """assumed clasp seat centres F1 (upper trapezius) and F4 (acromion) on the skin top surface + 5 mm pad stack; positions are ASSUMED
    (clasp_sites.json is not in the project knowledge) - flagged in every concept that uses them.  Returns dict name->(point, normal)."""
    out = {}
    for nm, (x, y) in dict(F1=(-150.0, -62.0), F4=(-178.0, 14.0)).items():
        X, Y, H = top_surface(x - 8, x + 8, y - 8, y + 8, 4.0, 1.0); z = float(H.mean())
        out[nm] = (np.array([x, y, z]), np.array([0.0, 0.0, 1.0]))
    return out
def env_parts():
    """wing + rail + tongue proxies (generic draped plates, NOT the verified wing): rail on the paraspinal strip (y -138..-100, foam gap 7),
    wing bridging over the moving blade to the plate tongue (y -100..-46).  Returns {name: dict(mesh, mat, group='env', colour)}"""
    rail = draped_back_plate(-138, -100, -420, -60, 10.0, 3.5, 4.0, smooth=6.0)
    Y, Z, Xs = rear_surface(-106, -44, -165, -60, 4.0, 6.0)
    s_ = np.clip((Y + 104.0) / 26.0, 0, 1); s_ = s_ * s_ * (3 - 2 * s_)              # 0 at the rail edge -> 1 at the tongue (y >= -78)
    xb = (Xs + 10.0) * (1 - s_) + (-91.5) * s_; xb = np.maximum(xb, np.where(Y > -96, Xs + 20.0, -1e9))   # >= 20 mm over the moving blade
    xb = gaussian_filter(xb, 1.5, mode="nearest"); wing = height_solid(Y, Z, xb, xb + 3.5, "x")
    return dict(rail=dict(mesh=rail, mat="PACF", group="env", colour=COL["pacf"]), wing=dict(mesh=wing, mat="PACF", group="env", colour=COL["pacf"]))
def plate_outline():
    """baseline chassis plate outline (y,z): hull of the S1-block footprint, keel feet, tongue seat (specs: 158 x 104, z top -54)."""
    from scipy.spatial import ConvexHull
    pts = []
    circ = lambda cy, cz, r: [(cy + r * np.cos(a), cz + r * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 24, endpoint=False)]
    for cy in (-68.0, 68.0):
        for cz in (-100.0, -62.0): pts += circ(cy, cz, 8.0)
    for cy, cz in ((-50.0, -109.0), (50.0, -109.0)): pts += circ(cy, cz, 10.0)
    for ty in (-78.0, -46.0):
        for tz in (-76.0, -150.0): pts += circ(ty, tz, 4.0)
    pts = np.array(pts); h = pts[ConvexHull(pts).vertices]; return [tuple(p) for p in h]

def protrusion(m, n=0):
    """depth of the part behind the back: max x + 108 (mm); 108 = the most rearward body surface (x = -108). Baseline: cowl 170, crown 181, S1 block 108."""
    return float(m.bounds[1][0] + 108.0)
def union_protrusion(meshes): return max(protrusion(m) for m in meshes)

# ------------------------------------------------------------------ joint-study helpers (kinematic equivalence with the baseline shoulder)
def baseline_dir(S1, S2):
    """arm direction of the baseline shoulder, R = Rz(-S1) Ry(-S2), arm along +z at rest: d = (-sin S2 cos S1, sin S2 sin S1, cos S2)."""
    s1, s2 = np.radians(S1), np.radians(S2); return np.array([-np.sin(s2) * np.cos(s1), np.sin(s2) * np.sin(s1), np.cos(s2)])
def baseline_dir_set(step=5.0):
    """unit arm directions reachable by the baseline soft box S1 10..140, S2 -30..88 (S3 does not change the direction)."""
    return np.array([baseline_dir(a, b) for a in np.arange(10, 140.1, step) for b in np.arange(-30, 88.1, step)])
def coverage(new_dirs, tol_deg=3.0):
    """fraction of the baseline direction set within tol_deg of any direction the new joint reaches (new_dirs: (N,3) unit vectors in the SAME robot frame, arm direction)."""
    B = baseline_dir_set(); t = cKDTree(np.asarray(new_dirs)); d, _ = t.query(B); return float(np.mean(d <= 2 * np.sin(np.radians(tol_deg) / 2)))
def rot_dir(R, v=(0, 0, 1.0)): return np.asarray(R) @ np.asarray(v)
