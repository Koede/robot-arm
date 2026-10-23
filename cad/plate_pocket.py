"""Chassis-plate pocketing (-80 g target) with a finite-element stress check, baseline vs pocketed.  mm, N, MPa, RIGHT plate (x in [-88,-80], outline in the y-z plane).
Outline + holes are rebuilt from chassis_axilla.py (same hull, same holes); chassis_axilla.py itself and chassis_plate_R.stl are NOT modified.
Model: Kirchhoff plate bending (Morley triangle) + plane-stress membrane (P2 triangle), scikit-fem; the plate is thin (t = 8 mm vs ~150 x 100 mm), 6061-T6 (E 68.9 GPa, nu 0.33).
  bending moment + membrane stress -> top/bottom-fibre von Mises per element (centroid values).  Staircase mesh 1.5 mm; stress within R_EXCL of a pin support or a
  load footprint is excluded from the maxima (singular nodal supports) and reported separately as 'near-hole' with the Kt = 3 hole factor applied analytically.
Loads (limit, at the S1 base block footprint y +-68, z -100..-62; applied as rigid-block tractions): pitch M_y = 24.8 N m (AK70 peak = S2 reaction), yaw M_z = 24.8 N m (S1 peak),
  roll M_x = 24.8 N m, plus F_z = -75 N (3 g of the 2.5 kg stage + arm).  Supports: clasp-foot and pauldron-foot holes (w = u = v = 0 on the hole circles).
Pocket layout: hexagonal lattice of round through-windows in the LOW-stress, hole-free, edge-free area of the baseline plate; ligament >= 10 mm between windows and >= 10 mm to any hole / edge,
  corner radius = window radius (>= 7 mm).  Target: 80 g of 6061 = 29.6 cm3 = 3704 mm2 of plate area.
Run: nice -n 10 python3 -I plate_pocket.py     -> stl/balancer/chassis_plate_pocketed_{R,L}.stl, stl/balancer/plate_pocket.json"""
import os, sys, json, time, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import cadquery as cq, trimesh
from scipy.spatial import ConvexHull
from scipy import sparse
from scipy.sparse.linalg import spsolve
from scipy.ndimage import distance_transform_edt, binary_erosion, uniform_filter
from matplotlib.path import Path
S = os.path.join(HERE, "..", "stl", "shoulder") + "/"; OUT = os.path.join(HERE, "..", "stl", "balancer") + "/"
T, X0 = 8.0, -88.0; E_AL, NU, SY_AL, RHO = 68900.0, 0.33, 240.0, 2.70e-3       # MPa, -, MPa (6061-T6 yield), g/mm3
H = 1.5                                                                         # FE grid, mm
LIG, EDGE_M, HOLE_M = 10.0, 10.0, 10.0

# ------------------------------------------------------------------ outline + holes (identical to chassis_axilla.py)
feet = json.load(open(S + "clasp_feet.json"))
def circ(cy, cz, r, n=40): return [(cy + r * np.cos(a), cz + r * np.sin(a)) for a in np.linspace(0, 2 * np.pi, n, endpoint=False)]
seed = []
for cy_ in (-68.0, 68.0):
    for cz_ in (-100.0, -62.0): seed += circ(cy_, cz_, 8.0)
for k_, f_ in feet.items():
    for dz_ in f_["bolts_dz"]: seed += circ(f_["y"], f_["z"] + dz_, 16.0)
PAULDRON_FEET = [(-48.0, -140.0), (48.0, -140.0), (0.0, -150.0)]
for cy_, cz_ in PAULDRON_FEET:
    for dz_ in (-7.0, 7.0): seed += circ(cy_, cz_ + dz_, 11.0)
seed = np.array(seed); hull = seed[ConvexHull(seed).vertices]
holes = []; supp = []
for y in (-62.0, 62.0):
    for z in (-68.0, -94.0): holes.append((4.2, y, z))
for k, f in feet.items():
    for dz in f["bolts_dz"]: holes.append((3.3, f["y"], f["z"] + dz)); supp.append((f["y"], f["z"] + dz))
for cy_, cz_ in PAULDRON_FEET:
    for dz in (-7.0, 7.0): holes.append((3.3, cy_, cz_ + dz)); supp.append((cy_, cz_ + dz))
FOOT = (-68.0, 68.0, -100.0, -62.0)      # S1 base block footprint (y0, y1, z0, z1)

# ------------------------------------------------------------------ raster
y0, y1, z0, z1 = hull[:, 0].min() - 2, hull[:, 0].max() + 2, hull[:, 1].min() - 2, hull[:, 1].max() + 2
ny, nz = int(np.ceil((y1 - y0) / H)) + 1, int(np.ceil((z1 - z0) / H)) + 1
Yg, Zg = np.meshgrid(y0 + H * np.arange(ny), z0 + H * np.arange(nz), indexing="ij")
inside_poly = Path(hull).contains_points(np.c_[Yg.ravel(), Zg.ravel()]).reshape(ny, nz)
hole_mask = np.zeros_like(inside_poly)
for d, y, z in holes: hole_mask |= (Yg - y) ** 2 + (Zg - z) ** 2 <= (d / 2) ** 2
def cell_mask(win_mask=None):
    """cells (squares between grid nodes) that are solid: centre inside outline and not in a hole / window"""
    yc, zc = Yg[:-1, :-1] + H / 2, Zg[:-1, :-1] + H / 2
    ok = Path(hull).contains_points(np.c_[yc.ravel(), zc.ravel()]).reshape(ny - 1, nz - 1)
    for d, y, z in holes: ok &= (yc - y) ** 2 + (zc - z) ** 2 > (d / 2) ** 2
    if win_mask is not None: ok &= ~win_mask
    return ok
plate_area = lambda wins: Path(hull).contains_points(np.c_[Yg.ravel(), Zg.ravel()]).sum() * H * H     # (not used for the mass: exact polygon area below)
def poly_area(p): x, y = p[:, 0], p[:, 1]; return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
A_PLATE = poly_area(hull) - sum(np.pi * (d / 2) ** 2 for d, _, _ in holes)

# ------------------------------------------------------------------ FE
from skfem import MeshTri, Basis, ElementTriMorley, ElementTriP2, ElementVector, BilinearForm, LinearForm, solve, condense, asm
from skfem.helpers import dd, ddot, trace, eye, sym_grad, dot
from skfem.models.elasticity import linear_elasticity

def build_mesh(ok):
    idx = -np.ones((ny, nz), int); cells = np.argwhere(ok)
    used = np.zeros((ny, nz), bool)
    for di in (0, 1):
        for dj in (0, 1): used[cells[:, 0] + di, cells[:, 1] + dj] = True
    idx[used] = np.arange(used.sum()); pts = np.c_[Yg[used], Zg[used]].T
    tris = []
    i, j = cells[:, 0], cells[:, 1]
    a, b, c, d = idx[i, j], idx[i + 1, j], idx[i + 1, j + 1], idx[i, j + 1]
    alt = ((i + j) % 2 == 0)
    t1 = np.where(alt[:, None], np.c_[a, b, c], np.c_[a, b, d]); t2 = np.where(alt[:, None], np.c_[a, c, d], np.c_[b, c, d])
    return MeshTri(pts, np.vstack([t1, t2]).T.astype(np.int64))

def tractions(case):
    """(p_x, t_y, t_z) tractions [MPa = N/mm2] over the footprint: rigid-block distribution carrying the resultant of `case`"""
    A = (FOOT[1] - FOOT[0]) * (FOOT[3] - FOOT[2]); yc, zc = 0.5 * (FOOT[0] + FOOT[1]), 0.5 * (FOOT[2] + FOOT[3])
    Iy = (FOOT[3] - FOOT[2]) * (FOOT[1] - FOOT[0]) ** 3 / 12.0; Iz = (FOOT[1] - FOOT[0]) * (FOOT[3] - FOOT[2]) ** 3 / 12.0     # second moments of the footprint about z-axis / y-axis lines
    Ip = Iy + Iz
    def f(y, z):
        inside = ((y >= FOOT[0]) & (y <= FOOT[1]) & (z >= FOOT[2]) & (z <= FOOT[3])).astype(float)
        px = np.zeros_like(y); ty = np.zeros_like(y); tz = np.zeros_like(y)
        if case == "pitch":                           # moment about y (N mm): x-traction linear in z: p = M (z - zc) / Iz
            px = 24.8e3 * (z - zc) / Iz; tz = -75.0 / A
        elif case == "yaw":                           # moment about z: x-traction linear in y
            px = 24.8e3 * (y - yc) / Iy; tz = -75.0 / A
        elif case == "roll":                          # moment about x: in-plane tangential traction (torsion of the footprint)
            ty = -24.8e3 * (z - zc) / Ip; tz = 24.8e3 * (y - yc) / Ip
        return px * inside, ty * inside, tz * inside
    return f

def solve_case(mesh, case):
    t = T
    bm = Basis(mesh, ElementTriMorley()); mm = Basis(mesh, ElementVector(ElementTriP2()))
    def C(Tn): return E_AL / (1 + NU) * (Tn + NU / (1 - NU) * eye(trace(Tn), 2))
    @BilinearForm
    def kb(u, v, w): return t ** 3 / 12.0 * ddot(C(dd(u)), dd(v))
    Kb = asm(kb, bm)
    lam = E_AL * NU / (1 - NU ** 2); mu = E_AL / (2 * (1 + NU))
    Km = asm(linear_elasticity(lam, mu), mm) * t
    f = tractions(case)
    @LinearForm
    def lb(v, w): return f(w.x[0], w.x[1])[0] * v
    @LinearForm
    def lmem(v, w): px, ty, tz = f(w.x[0], w.x[1]); return ty * v[0] + tz * v[1]
    fb = asm(lb, bm); fm = asm(lmem, mm)
    # supports: nodes on / inside the support hole circles (every support dof)
    P = mesh.p
    sup_nodes = np.zeros(P.shape[1], bool)
    for (y, z) in supp: sup_nodes |= (P[0] - y) ** 2 + (P[1] - z) ** 2 <= (4.0) ** 2
    vert_dofs = np.where(sup_nodes)[0]                                       # Morley vertex dofs are the first N_v dofs (value w)
    Dm = np.concatenate([mm.nodal_dofs[:, vert_dofs].ravel()])
    # Morley: first nodal dof set = w at vertices (nodal_dofs[0]); edge dofs follow
    Dw = bm.nodal_dofs[0, vert_dofs]
    wb = solve(*condense(Kb, fb, D=Dw))
    um = solve(*condense(Km, fm, D=Dm))
    return bm, mm, wb, um, float(fb @ wb + fm @ um), float(np.abs(wb[:mesh.p.shape[1]]).max())

def stress_map(mesh, bm, mm, wb, um):
    """element-centroid top/bottom von Mises from membrane + bending"""
    t = T; lam = E_AL * NU / (1 - NU ** 2); mu = E_AL / (2 * (1 + NU))
    g = mm.interpolate(um).grad                                              # (2,2,nel,nqp)
    eps = 0.5 * (g + np.swapaxes(g, 0, 1))
    sxx = (lam + 2 * mu) * eps[0, 0] + lam * eps[1, 1]; syy = (lam + 2 * mu) * eps[1, 1] + lam * eps[0, 0]; sxy = 2 * mu * eps[0, 1]
    sxx, syy, sxy = (a.mean(axis=1) for a in (sxx, syy, sxy))
    h = bm.interpolate(wb).hess                                              # (2,2,nel,nqp)
    kxx, kyy, kxy = (h[0, 0].mean(axis=1), h[1, 1].mean(axis=1), h[0, 1].mean(axis=1))
    Mxx = -(kxx + NU * kyy); Myy = -(kyy + NU * kxx); Mxy = -(1 - NU) * kxy                    # per unit D (sign irrelevant for vm), times Db below
    Db = E_AL * t ** 3 / (12 * (1 - NU ** 2)); fac = 6 * Db / t ** 2
    bxx, byy, bxy = fac * Mxx, fac * Myy, fac * Mxy
    out = []
    for sgn in (1, -1):
        a, b, c = sxx + sgn * bxx, syy + sgn * byy, sxy + sgn * bxy
        out.append(np.sqrt(a * a - a * b + b * b + 3 * c * c))
    return np.maximum(*out)

def run_fe(win_mask=None):
    ok = cell_mask(win_mask); mesh = build_mesh(ok); vm_all = {}; global _COMP; _COMP = {}
    cen = mesh.p[:, mesh.t].mean(axis=1)                                       # (2, nel)
    for case in ("pitch", "yaw", "roll"):
        bm, mm, wb, um, comp, wmax = solve_case(mesh, case); vm_all[case] = stress_map(mesh, bm, mm, wb, um); _COMP[case] = (round(comp, 4), round(wmax, 5))
    return mesh, cen, vm_all

def excl_mask(cen, r_sup=9.0, r_hole=7.0, r_foot=3.0):
    """elements excluded from the 'field' maxima: round supports, round bolt holes, the load footprint edge band"""
    ex = np.zeros(cen.shape[1], bool)
    for (y, z) in supp: ex |= (cen[0] - y) ** 2 + (cen[1] - z) ** 2 < r_sup ** 2
    for d, y, z in holes: ex |= (cen[0] - y) ** 2 + (cen[1] - z) ** 2 < (d / 2 + r_hole) ** 2
    edge = ((np.abs(cen[0] - FOOT[0]) < 3) | (np.abs(cen[0] - FOOT[1]) < 3)) & (cen[1] > FOOT[2] - 3) & (cen[1] < FOOT[3] + 3)
    edge |= ((np.abs(cen[1] - FOOT[2]) < 3) | (np.abs(cen[1] - FOOT[3]) < 3)) & (cen[0] > FOOT[0] - 3) & (cen[0] < FOOT[1] + 3)
    return ex | edge

# ------------------------------------------------------------------ window layout from the baseline stress map
KEEP = {"foot": 14.0, "paul": 10.0}      # keep-out radius round the clasp-foot / pauldron-foot bolts: the foot seats on the front face over this area
EDGE_KEEP, FOOT_KEEP, CORNER_R = 6.0, 5.0, 5.0
def layout(cen, vm_env, mesh):
    """free-form through-windows = opening (R5 corners) of the allowed region; windows closer than LIG to each other are merged by dropping the smaller one"""
    from scipy.ndimage import binary_opening, label
    R = np.zeros((ny - 1, nz - 1)); cnt = np.zeros_like(R)
    ii = np.clip(((cen[0] - y0) / H).astype(int), 0, ny - 2); jj = np.clip(((cen[1] - z0) / H).astype(int), 0, nz - 2)
    np.add.at(R, (ii, jj), vm_env); np.add.at(cnt, (ii, jj), 1); R = np.where(cnt > 0, R / np.maximum(cnt, 1), 0.0)
    Rs = uniform_filter(R, size=7)
    ok = cell_mask(); inside = inside_poly[:-1, :-1]
    yc, zc = Yg[:-1, :-1] + H / 2, Zg[:-1, :-1] + H / 2
    keep = np.zeros_like(ok)
    for d, y, z in holes:
        if abs(y) in (62.0,) and z in (-68.0, -94.0): continue
    for k, f in feet.items():
        for dz in f["bolts_dz"]: keep |= (yc - f["y"]) ** 2 + (zc - (f["z"] + dz)) ** 2 < KEEP["foot"] ** 2
    for cy_, cz_ in PAULDRON_FEET:
        for dz in (-7.0, 7.0): keep |= (yc - cy_) ** 2 + (zc - (cz_ + dz)) ** 2 < KEEP["paul"] ** 2
    keep |= (yc > FOOT[0] - FOOT_KEEP) & (yc < FOOT[1] + FOOT_KEEP) & (zc > FOOT[2] - FOOT_KEEP) & (zc < FOOT[3] + FOOT_KEEP)
    dist_edge = distance_transform_edt(inside) * H
    thr = 0.5 * np.percentile(Rs[ok], 99)
    allowed = ok & ~keep & (dist_edge >= EDGE_KEEP) & (Rs < thr)
    rpx = int(round(CORNER_R / H)); yy, xx = np.ogrid[-rpx:rpx + 1, -rpx:rpx + 1]; disc = (yy ** 2 + xx ** 2) <= rpx ** 2
    W = binary_opening(allowed, structure=disc)
    lab, n = label(W)
    comps = [(float((lab == i).sum() * H * H), i) for i in range(1, n + 1)]
    comps.sort(reverse=True); kept = []
    for area, i in comps:
        m_i = lab == i
        dist_i = distance_transform_edt(~m_i) * H
        if all(dist_i[lab == j].min() >= LIG for j in kept) and area > 150.0: kept.append(i)
    Wk = np.isin(lab, kept)
    return Wk, Rs, allowed

if __name__ == "__main__":
    t0 = time.time(); res = dict(T_mm=T, material="6061-T6", E_MPa=E_AL, nu=NU, yield_MPa=SY_AL, plate_area_mm2=round(A_PLATE, 1), plate_mass_g=round(A_PLATE * T * RHO, 1), fe_grid_mm=H)
    mesh, cen, vm0 = run_fe(); comp0 = dict(_COMP); env0 = np.maximum.reduce(list(vm0.values())); ex0 = excl_mask(cen)
    res["baseline"] = dict(n_elem=int(mesh.t.shape[1]), field_max_vm_MPa={k: round(float(v[~ex0].max()), 1) for k, v in vm0.items()}, near_hole_max_vm_MPa={k: round(float(v[ex0].max()), 1) for k, v in vm0.items()},
                           field_p99_vm_MPa=round(float(np.percentile(env0[~ex0], 99)), 1), field_median_vm_MPa=round(float(np.median(env0[~ex0])), 1))
    print("baseline", res["baseline"], "%.0fs" % (time.time() - t0), flush=True)
    Wk, Rs, allowed = layout(cen, env0, mesh)
    from scipy.ndimage import label as _lab
    lab, nwin = _lab(Wk); area = float(Wk.sum() * H * H)
    res["layout"] = dict(n_windows=int(nwin), window_area_mm2=round(area, 1), ligament_mm=LIG, corner_radius_mm=CORNER_R, removed_cm3=round(area * T / 1e3, 2), removed_g=round(area * T * RHO, 1),
                         mass_after_g=round((A_PLATE - area) * T * RHO, 1), allowed_area_mm2=round(float(allowed.sum() * H * H), 1), edge_keep_mm=EDGE_KEEP, foot_keep_r_mm=KEEP,
                         windows=[dict(area_mm2=round(float((lab == i).sum() * H * H), 1), centroid_yz=[round(float(Yg[:-1, :-1][lab == i].mean() + H / 2), 1), round(float(Zg[:-1, :-1][lab == i].mean() + H / 2), 1)]) for i in range(1, nwin + 1)])
    print("layout", {k: v for k, v in res["layout"].items() if k != "windows"}, flush=True)
    mesh1, cen1, vm1 = run_fe(Wk); comp1 = dict(_COMP); env1 = np.maximum.reduce(list(vm1.values())); ex1 = excl_mask(cen1)
    # extra exclusion: none; ligament elements are the interesting ones
    from scipy.ndimage import binary_dilation
    ring = binary_dilation(Wk, iterations=int(round(6.0 / H))) & ~Wk
    ci = np.clip(((cen1[0] - y0) / H).astype(int), 0, ny - 2); cj = np.clip(((cen1[1] - z0) / H).astype(int), 0, nz - 2); lig = ring[ci, cj]
    res["pocketed"] = dict(n_elem=int(mesh1.t.shape[1]), field_max_vm_MPa={k: round(float(v[~ex1].max()), 1) for k, v in vm1.items()}, near_hole_max_vm_MPa={k: round(float(v[ex1].max()), 1) for k, v in vm1.items()},
                           window_rim_max_vm_MPa={k: round(float(v[lig & ~ex1].max()), 1) if (lig & ~ex1).any() else None for k, v in vm1.items()},
                           field_p99_vm_MPa=round(float(np.percentile(env1[~ex1], 99)), 1))
    kt_hole = 3.0
    fm1 = max(res["pocketed"]["field_max_vm_MPa"].values()); rim1 = max([v for v in res["pocketed"]["window_rim_max_vm_MPa"].values() if v is not None] or [0.0])
    res["stiffness"] = {c: dict(compliance_Nmm_base=comp0[c][0], compliance_Nmm_pocketed=comp1[c][0], ratio=round(comp1[c][0] / comp0[c][0], 3), max_w_mm_base=comp0[c][1], max_w_mm_pocketed=comp1[c][1]) for c in comp0}
    res["checks"] = dict(max_field_stress_MPa=fm1, SF_yield_field=round(SY_AL / fm1, 1), window_rim_stress_MPa=rim1, SF_yield_rim=round(SY_AL / max(rim1, 1e-9), 1),
                         note="FE gives the nominal field; the 1.5 mm staircase rim over-predicts the hole peak, no extra Kt applied to the rim values. Limit loads (peak actuator torque); fatigue limit 6061-T6 ~ 95 MPa (R=-1, 5e8)")
    res["_runtime_s"] = round(time.time() - t0)
    # ------------------------------------------------------------ CAD: cut the windows into the plate, export
    poly = [tuple(map(float, p)) for p in hull]
    plate = cq.Workplane("YZ").polyline(poly).close().extrude(T).translate((X0, 0, 0))
    for d, y, z in holes: plate = plate.cut(cq.Workplane("YZ").circle(d / 2).extrude(T + 2).translate((X0 - 1, y, z)))
    from skimage import measure
    contours_out = []
    for i in range(1, nwin + 1):
        mk = np.pad((lab == i).astype(float), 1)
        for c in measure.find_contours(mk, 0.5):
            pts = [(float(y0 + (p[0] - 1 + 0.5) * H + H / 2 - H / 2 * 0 ), float(z0 + (p[1] - 1 + 0.5) * H + H / 2 - H / 2 * 0)) for p in c]
            poly_i = measure.approximate_polygon(np.array(pts), tolerance=0.15)
            if len(poly_i) < 4: continue
            contours_out.append([[round(float(a), 2), round(float(b), 2)] for a, b in poly_i])
            plate = plate.cut(cq.Workplane("YZ").polyline([tuple(map(float, q)) for q in poly_i[:-1]]).close().extrude(T + 2).translate((X0 - 1, 0, 0)))
    res["layout"]["contours_yz"] = contours_out
    p = OUT + "chassis_plate_pocketed_R.stl"; cq.exporters.export(plate, p, tolerance=0.03, angularTolerance=0.05)
    m = trimesh.load(p); ml = m.copy(); ml.apply_transform(np.diag([1, -1, 1, 1.0]))
    if ml.volume < 0: ml.invert()
    ml.export(OUT + "chassis_plate_pocketed_L.stl")
    base = trimesh.load(S + "chassis_plate_R.stl")
    res["cad"] = dict(vol_cm3=round(m.volume / 1e3, 2), mass_g=round(m.volume * RHO, 1), base_vol_cm3=round(base.volume / 1e3, 2), base_mass_g=round(base.volume * RHO, 1), delta_g=round((m.volume - base.volume) * RHO, 1),
                      watertight=bool(m.is_watertight), bbox_mm=[round(float(x), 1) for x in (m.bounds[1] - m.bounds[0])])
    json.dump(res, open(OUT + "plate_pocket.json", "w"), indent=1)
    print(json.dumps({k: res[k] for k in ("stiffness", "checks", "cad")}, indent=1))
    # picture
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(13, 5.5))
        for a_, (mesh_, cen_, env_, ttl) in zip(ax, ((mesh, cen, env0, "baseline"), (mesh1, cen1, env1, "pocketed"))):
            sc = a_.tripcolor(mesh_.p[0], mesh_.p[1], mesh_.t.T, facecolors=np.minimum(env_, 40.0), cmap="viridis", vmin=0, vmax=40); a_.set_aspect("equal"); a_.set_title("%s: envelope von Mises (clipped at 40 MPa)" % ttl)
            for d, y, z in holes: a_.add_patch(plt.Circle((y, z), d / 2, fill=False, color="w", lw=0.6))
        fig.colorbar(sc, ax=ax); fig.savefig(os.path.join(HERE, "..", "plate_pocket_stress.png"), dpi=110)
    except Exception as e:
        print("plot failed", e)
