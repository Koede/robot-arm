"""J1 rev 2 collision-sweep engine (project method, specs/SPEC_collision_method.md).  Robot frame, mm, deg.

    from j1r2_sweep import Scene, run_sweep, local_opt_hints, swept_volume, cut_by_sweep, make_wearer_parts

Scene  : fixed {name: Trimesh}, wearer parts (skin 2 mm, head 60 mm), moving GROUPS {name: dict(mesh, joints=[upstream chain, ordered])},
         joints [ordered names], pose_fn(group, q:dict deg) -> 4x4 world transform of the group mesh (mesh given in its REFERENCE frame),
         park {joint: deg}, soft {joint:(lo,hi)}, hard {joint:(lo,hi)}, exclude set of "group|object" keys.
Method : pair tables over ONLY the joints between the two bodies (upstream joints cancel; bodies vs fixed/wearer use all upstream joints),
         FCL BVH mesh distance (exact mesh-mesh) with a certified bounding-sphere lower bound to skip far poses, union contact mask on the full
         joint grid, flood fill from the park pose (reachable free region), frontier = contact cells touching it (deep ones dropped),
         sub-grid first-contact angles by linear zero crossing of d-MM on every free->contact edge, certified sphere tracing
         (dq=(d-MM)/sum r_j|dq_j|) as the fast path, validated against the per-pair grid reference.
Gates  : MM part-part 3, skin 2, head/neck 60 mm; d < 1 mm = FAIL (collision), 1..MM = BELOW_GATE."""
import os, sys, json, time, itertools, math
import numpy as np, trimesh, fcl
from scipy import ndimage as ndi
from concurrent.futures import ProcessPoolExecutor
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clib as L
OUT = os.path.join(L.ROOT, "stl", "j1r2_sweep"); os.makedirs(OUT, exist_ok=True)
MM_DEFAULT = dict(part=3.0, skin=2.0, head=60.0)
FAR = 150.0                                              # distances above this are only bounded (lower bound used), never queried
# ------------------------------------------------------------------------------------------------ scene
class Scene:
    def __init__(s, fixed, groups, joints, pose_fn, park, soft, hard=None, wearer=None, exclude=(), MM=None, name="scene"):
        s.fixed = dict(fixed); s.groups = groups; s.joints = list(joints); s.pose_fn = pose_fn; s.park = dict(park); s.soft = dict(soft)
        s.hard = dict(hard) if hard else {j: (a - 5, b + 5) for j, (a, b) in soft.items()}; s.wearer = wearer or {}; s.exclude = set(exclude)
        s.MM = dict(MM_DEFAULT); s.MM.update(MM or {}); s.name = name
    def objects(s):
        """fixed object table: name -> (mesh, kind)"""
        o = {k: (m, "part") for k, m in s.fixed.items()}
        for k, m in s.wearer.items(): o["wearer_" + k] = (m, "skin" if k == "skin" else "head")
        return o
def make_wearer_parts(center=(-110.0, -40.0, -90.0), radius=520.0, target_faces=14000):
    """crop clib.body() to the region the arm can reach and split off the head/neck keep-out (faces above z -70 near the spine plane y -150, x < -90)."""
    import fast_simplification as fs
    b = L.body(); C = np.asarray(center); keep = np.linalg.norm(b.triangles_center - C, axis=1) < radius
    sub = b.submesh([np.where(keep)[0]], append=True)
    v, f = (np.asarray(sub.vertices, np.float64), np.asarray(sub.faces, np.int32)) if len(sub.faces) <= target_faces else fs.simplify(np.asarray(sub.vertices, np.float64), np.asarray(sub.faces, np.int32), target_count=target_faces)
    skin = trimesh.Trimesh(v, f, process=False)
    tc = skin.triangles_center; hs = (tc[:, 2] > -70) & (np.abs(tc[:, 1] + 150) < 120) & (tc[:, 0] < -90)
    head = skin.submesh([np.where(hs)[0]], append=True)
    return dict(skin=skin, head=head)
# ------------------------------------------------------------------------------------------------ fcl helpers
def _bvh(m):
    v = np.asarray(m.vertices, np.float64); f = np.asarray(m.faces, np.int32); b = fcl.BVHModel(); b.beginModel(len(v), len(f)); b.addSubModel(v, f); b.endModel(); return b
class Fcl:
    def __init__(s, scene):
        s.sc = scene; s.model = {}; s.obj = {}
        for k, (m, kind) in scene.objects().items():
            s.model[k] = _bvh(m); o = fcl.CollisionObject(s.model[k], fcl.Transform()); s.obj[k] = o
        s.gmodel = {g: _bvh(d["mesh"]) for g, d in scene.groups.items()}; s.gobj = {g: fcl.CollisionObject(s.gmodel[g], fcl.Transform()) for g in scene.groups}
        s.req = fcl.DistanceRequest(enable_nearest_points=False); s.req_np = fcl.DistanceRequest(enable_nearest_points=True)
        s.kd = {}; s.sph = {}
        from scipy.spatial import cKDTree
        for k, (m, kind) in scene.objects().items():
            n = int(min(250000, max(20000, m.area / 4.0))); p = trimesh.sample.sample_surface(m, n)[0]; h = math.sqrt(m.area / n) * 1.5 + 0.5
            s.kd[k] = (cKDTree(p), h)
        for g, d in scene.groups.items():
            c, r = trimesh.nsphere.minimum_nsphere(d["mesh"]) if False else _min_sphere(d["mesh"]); s.sph[g] = (c, r)
    def set_group(s, g, T):
        o = s.gobj[g]; o.setTransform(fcl.Transform(T[:3, :3], T[:3, 3])); return o
    def dist_obj(s, g, T, k):
        """distance group g (pose T) to fixed object k; certified lower bound shortcut beyond FAR."""
        c, r = s.sph[g]; cw = T[:3, :3] @ c + T[:3, 3]; kd, h = s.kd[k]; lb = kd.query(cw)[0] - r - h
        if lb > FAR: return float(lb)
        o = s.set_group(g, T); res = fcl.DistanceResult(); d = fcl.distance(o, s.obj[k], s.req, res); return float(d)
    def dist_gg(s, ga, Ta, gb, Tb):
        ca, ra = s.sph[ga]; cb, rb = s.sph[gb]; pa = Ta[:3, :3] @ ca + Ta[:3, 3]; pb = Tb[:3, :3] @ cb + Tb[:3, 3]
        lb = np.linalg.norm(pa - pb) - ra - rb
        if lb > FAR: return float(lb)
        oa = s.set_group(ga, Ta); ob = s.set_group(gb, Tb); res = fcl.DistanceResult(); return float(fcl.distance(oa, ob, s.req, res))
    def nearest_obj(s, g, T, k):
        o = s.set_group(g, T); res = fcl.DistanceResult(); d = fcl.distance(o, s.obj[k], s.req_np, res); return float(d), np.array(res.nearest_points[0]), np.array(res.nearest_points[1])
    def nearest_gg(s, ga, Ta, gb, Tb):
        oa = s.set_group(ga, Ta); ob = s.set_group(gb, Tb); res = fcl.DistanceResult(); d = fcl.distance(oa, ob, s.req_np, res); return float(d), np.array(res.nearest_points[0]), np.array(res.nearest_points[1])
def _min_sphere(m):
    """small bounding sphere (centre of the vertex bbox refined by Ritter) -- conservative, not minimal."""
    v = np.asarray(m.vertices); c = (v.min(0) + v.max(0)) / 2; r = np.linalg.norm(v - c, axis=1).max()
    for _ in range(3):
        d = np.linalg.norm(v - c, axis=1); i = d.argmax()
        if d[i] > r * 1.0001: pass
        c = c + (v[i] - c) * 0.1 / (1 + 0.0); r = np.linalg.norm(v - c, axis=1).max()
    c0 = (v.min(0) + v.max(0)) / 2; r0 = np.linalg.norm(v - c0, axis=1).max()
    return (c0, r0) if r0 <= r else (c, r)
# ------------------------------------------------------------------------------------------------ pairs and grids
class Pair:
    def __init__(p, key, g, obj, kind, dims, MM, other=None):
        p.key, p.g, p.obj, p.kind, p.dims, p.MM, p.other = key, g, obj, kind, dims, MM, other
def build_pairs(sc):
    P = []; objs = sc.objects()
    for g, d in sc.groups.items():
        for k, (m, kind) in objs.items():
            P.append(Pair(f"{g}|{k}", g, k, "obj", [j for j in sc.joints if j in d["joints"]], sc.MM[kind]))
    gs = list(sc.groups)
    for a in gs:
        for b in gs:
            ja, jb = set(sc.groups[a]["joints"]), set(sc.groups[b]["joints"])
            if a != b and ja < jb:                                    # a is an ancestor chain of b: relative pose depends on the joints between them
                P.append(Pair(f"{a}|{b}", b, a, "grp", [j for j in sc.joints if j in jb - ja], sc.MM["part"], other=a))
    return P
def grid_values(sc, opts):
    rng = sc.hard if opts.get("range", "soft") == "hard" else sc.soft; steps = opts.get("steps", {}); vals = {}
    for j in sc.joints:
        lo, hi = rng[j]; st = steps.get(j, 5.0)
        v = np.array(sorted(set(np.round(st, 4)))) if isinstance(st, (list, tuple, np.ndarray)) else np.append(np.arange(lo, hi - 1e-9, st), hi)
        v = np.array(sorted(set(np.round(np.append(v, sc.park[j]), 4)))); vals[j] = v[(v >= lo - 1e-9) & (v <= hi + 1e-9)]
    return vals
_G = {}                                                               # fork-shared state for workers
def _qd(sc, joints, vals_idx, vals):
    q = dict(sc.park)
    for j, i in zip(joints, vals_idx): q[j] = float(vals[j][i])
    return q
def _eval_pair_chunk(args):
    pi, flat = args; sc, F, pairs, vals = _G["sc"], _G["F"], _G["pairs"], _G["vals"]; p = pairs[pi]; shp = tuple(len(vals[j]) for j in p.dims)
    out = np.empty(len(flat), np.float32)
    for n, f in enumerate(flat):
        idx = np.unravel_index(f, shp) if shp else (); q = _qd(sc, p.dims, idx, vals)
        if p.kind == "obj": out[n] = F.dist_obj(p.g, sc.pose_fn(p.g, q), p.obj)
        else: out[n] = F.dist_gg(p.other, sc.pose_fn(p.other, q), p.g, sc.pose_fn(p.g, q))
    return pi, flat, out
def tabulate(sc, pairs, vals, procs=2, chunk=400):
    _G.update(sc=sc, F=Fcl(sc), pairs=pairs, vals=vals); tabs = {}; tasks = []
    for pi, p in enumerate(pairs):
        shp = tuple(len(vals[j]) for j in p.dims); n = int(np.prod(shp)) if shp else 1; tabs[pi] = np.empty(n, np.float32)
        for s in range(0, n, chunk): tasks.append((pi, np.arange(s, min(n, s + chunk))))
    if procs > 1:
        import multiprocessing as mp
        with ProcessPoolExecutor(procs, mp_context=mp.get_context("fork")) as ex:
            for pi, flat, out in ex.map(_eval_pair_chunk, tasks, chunksize=2): tabs[pi][flat] = out
    else:
        for t in tasks: pi, flat, out = _eval_pair_chunk(t); tabs[pi][flat] = out
    return {pi: tabs[pi].reshape(tuple(len(vals[j]) for j in pairs[pi].dims)) for pi in tabs}
# ------------------------------------------------------------------------------------------------ analysis on the full grid
def _bcast(tab, dims, joints):
    shape = [tab.shape[dims.index(j)] if j in dims else 1 for j in joints]; return tab.reshape(shape)
def analyse(sc, pairs, tabs, vals, designed):
    J = sc.joints; GS = tuple(len(vals[j]) for j in J); contact = np.zeros(GS, bool); contact_p = {}
    for pi, p in enumerate(pairs):
        if designed[pi]: continue
        c = _bcast(tabs[pi] < p.MM, p.dims, J); contact_p[pi] = np.broadcast_to(c, GS); contact |= contact_p[pi]
    pk = tuple(int(np.argmin(np.abs(vals[j] - sc.park[j]))) for j in J)
    free = ~contact; lab, _ = ndi.label(free, structure=ndi.generate_binary_structure(len(J), 1))
    reach = (lab == lab[pk]) if free[pk] else np.zeros(GS, bool)
    nb = ndi.binary_dilation(reach, structure=ndi.generate_binary_structure(len(J), 1)); frontier = contact & nb; deep = contact & ~nb
    res = dict(grid_shape=GS, park_free=bool(free[pk]), n_cells=int(np.prod(GS)), n_contact=int(contact.sum()), n_reach_free=int(reach.sum()), n_frontier=int(frontier.sum()), n_deep=int(deep.sum()))
    rows = {}
    for pi, p in enumerate(pairs):
        t = tabs[pi]; ax = [J.index(j) for j in p.dims]; miss = tuple(i for i in range(len(J)) if i not in ax)
        r = dict(pair=p.key, MM=p.MM, dims=p.dims, designed=bool(designed[pi]), d_min_table=float(t.min()), park_d=float(t[tuple(int(np.argmin(np.abs(vals[j] - sc.park[j]))) for j in p.dims)]) if p.dims else float(t))
        if designed[pi]: r["status"] = "EXCLUDED(designed contact at park)"; rows[p.key] = r; continue
        reach_p = np.moveaxis(reach.any(axis=miss) if miss else reach, range(len(ax)), np.argsort(ax)) if False else (reach.any(axis=miss) if miss else reach)
        fr_p = (frontier & contact_p[pi]); fr_pp = fr_p.any(axis=miss) if miss else fr_p
        dp_ = (deep & contact_p[pi]); dp_p = dp_.any(axis=miss) if miss else dp_
        con_p = t < p.MM
        r["n_contact_cells"] = int(con_p.sum()); r["n_frontier_cells"] = int(fr_pp.sum()); r["n_deep_cells"] = int((dp_p & ~fr_pp).sum())
        r["min_clear_reach_free"] = float(t[reach_p].min()) if reach_p.any() else None
        if r["n_frontier_cells"]:
            tt = np.where(fr_pp, t, np.inf); i = np.unravel_index(np.argmin(tt), t.shape); r["d_frontier_min"] = float(tt[i]); r["worst_contact_pose"] = {j: float(vals[j][i[k]]) for k, j in enumerate(p.dims)}
        elif not con_p.any() and reach_p.any():
            tt = np.where(reach_p, t, np.inf); i = np.unravel_index(np.argmin(tt), t.shape); r["worst_pose"] = {j: float(vals[j][i[k]]) for k, j in enumerate(p.dims)}
        dmin = r.get("d_frontier_min", None)
        r["status"] = ("FAIL(<1mm)" if dmin is not None and dmin < 1.0 else "BELOW_GATE" if dmin is not None else "PASS" if not con_p.any() else "PASS(contact only in unreachable region, dropped)")
        rows[p.key] = r
    return res, rows, reach, frontier, contact_p
def first_contact_points(sc, pairs, tabs, vals, reach, contact_p, designed):
    """sub-grid first-contact angles: linear zero crossing of d-MM on every free(reachable) -> contact edge, per pair."""
    J = sc.joints; out = {}
    for pi, p in enumerate(pairs):
        if designed[pi]: continue
        t = np.broadcast_to(_bcast(tabs[pi], p.dims, J), reach.shape); pts = []
        for ax, j in enumerate(J):
            v = vals[j]
            for sgn in (1, -1):
                a = reach; b = np.roll(contact_p[pi] & True, -sgn, axis=ax); da = t; db = np.roll(t, -sgn, axis=ax)
                sl = [slice(None)] * len(J)
                edge = a & b & (da >= p.MM)
                if sgn == 1: sl[ax] = slice(0, -1)
                else: sl[ax] = slice(1, None)
                m = np.zeros_like(edge); m[tuple(sl)] = True; edge &= m
                for idx in np.argwhere(edge):
                    i = tuple(idx); k = i[ax]; k2 = k + sgn; f = (da[i] - p.MM) / max(1e-9, (da[i] - db[i])); q = [float(vals[jj][ii]) for jj, ii in zip(J, i)]
                    q[ax] = float(v[k] + f * (v[k2] - v[k])); pts.append(q + [float(da[i]), float(db[i])])
        out[p.key] = np.array(pts) if pts else np.zeros((0, len(J) + 2))
    return out
# ------------------------------------------------------------------------------------------------ certified sphere tracing
def lipschitz_radii(sc, pairs, nsamp=120, pad=1.15, seed=0):
    """r_j for a group = max over sampled poses of the max vertex speed per rad when joint j moves (finite difference of pose_fn); padded."""
    rng = np.random.default_rng(seed); R = {}
    for g, d in sc.groups.items():
        V = np.asarray(d["mesh"].vertices)[::max(1, len(d["mesh"].vertices) // 800)]; V4 = np.c_[V, np.ones(len(V))]; r = {j: 0.0 for j in d["joints"]}
        for _ in range(nsamp):
            q = {j: float(rng.uniform(*sc.soft[j])) for j in sc.joints}; T0 = sc.pose_fn(g, q); X0 = (T0 @ V4.T).T[:, :3]
            for j in d["joints"]:
                q2 = dict(q); q2[j] += 0.25; X1 = (sc.pose_fn(g, q2) @ V4.T).T[:, :3]; r[j] = max(r[j], np.linalg.norm(X1 - X0, axis=1).max() / np.radians(0.25))
        R[g] = {j: v * pad for j, v in r.items()}
    return R
def _dist_pair(sc, F, p, q):
    if p.kind == "obj": return F.dist_obj(p.g, sc.pose_fn(p.g, q), p.obj)
    return F.dist_gg(p.other, sc.pose_fn(p.other, q), p.g, sc.pose_fn(p.g, q))
def sphere_trace(sc, F, p, radii, rng_kind="soft", nrays=None, tol=0.05, seed=1):
    """star-shaped first contact from the park pose along rays in normalised joint space; returns hits (n,dims) in deg and d at each hit, and ray count."""
    rng = sc.hard if rng_kind == "hard" else sc.soft; D = len(p.dims); rs = np.random.default_rng(seed)
    if D == 0: return np.zeros((0, 0)), 0
    if D == 1: U = np.array([[1.0], [-1.0]])
    elif D == 2: a = np.radians(np.arange(0, 360, 1.5)); U = np.c_[np.cos(a), np.sin(a)]
    else:
        n = nrays or (2500 if D == 3 else 6000); U = rs.normal(size=(n, D)); U /= np.linalg.norm(U, axis=1)[:, None]
    half = np.array([(rng[j][1] - rng[j][0]) / 2 for j in p.dims]); park = np.array([sc.park[j] for j in p.dims]); lo = np.array([rng[j][0] for j in p.dims]); hi = np.array([rng[j][1] for j in p.dims])
    r_ = np.array([radii[p.g if p.kind == "obj" else p.g][j] for j in p.dims])
    hits = []
    for u in U:
        du = u * half                                                       # deg per unit t
        with np.errstate(divide="ignore"):
            tmax = np.min(np.where(du > 0, (hi - park) / du, np.where(du < 0, (lo - park) / du, np.inf)))
        L_rate = float(np.sum(r_ * np.abs(du)) * np.pi / 180.0); t = 0.0
        for _ in range(80):
            q = dict(sc.park); qq = park + t * du
            for j, v in zip(p.dims, qq): q[j] = float(v)
            d = _dist_pair(sc, F, p, q)
            if d <= p.MM + tol: hits.append(qq.copy()); break
            t += max((d - p.MM) / L_rate, 1e-4)
            if t >= tmax: break
    return (np.array(hits) if hits else np.zeros((0, D))), len(U)
def grid_pair_frontier(sc, p, tab, vals):
    """per-pair reference: flood fill on the pair's own table, frontier cells (for validation against sphere tracing)."""
    con = tab < p.MM; pk = tuple(int(np.argmin(np.abs(vals[j] - sc.park[j]))) for j in p.dims); st = ndi.generate_binary_structure(len(p.dims), 1)
    lab, _ = ndi.label(~con, structure=st); reach = (lab == lab[pk]) if (~con)[pk] else np.zeros_like(con); fr = con & ndi.binary_dilation(reach, structure=st)
    return reach, fr
def validate_trace(sc, F, p, tab, vals, radii, rng_kind="soft"):
    reach, fr = grid_pair_frontier(sc, p, tab, vals); hits, nr = sphere_trace(sc, F, p, radii, rng_kind)
    cells = np.argwhere(fr); step = np.array([np.median(np.diff(vals[j])) if len(vals[j]) > 1 else 1.0 for j in p.dims])
    pts = np.array([[vals[j][i] for j, i in zip(p.dims, c)] for c in cells]) if len(cells) else np.zeros((0, len(p.dims)))
    out = dict(pair=p.key, rays=nr, hits=len(hits), grid_frontier_cells=int(len(cells)))
    if len(hits) and len(pts):
        from scipy.spatial import cKDTree
        d1 = cKDTree(pts / step).query(hits / step)[0]; out["hit_to_gridfrontier_cells_max"] = float(d1.max()); out["hit_to_gridfrontier_cells_mean"] = float(d1.mean())
        # star-visible grid frontier cells: straight segment from park crosses no contact cell
        con = tab < p.MM; vis = []; park = np.array([sc.park[j] for j in p.dims])
        for c, pt in zip(cells, pts):
            ok = True
            for s in np.linspace(0, 0.97, 40):
                qq = park + s * (pt - park); ci = tuple(int(np.argmin(np.abs(vals[j] - v))) for j, v in zip(p.dims, qq))
                if con[ci]: ok = False; break
            vis.append(ok)
        vis = np.array(vis); out["star_visible_frontier_cells"] = int(vis.sum())
        if vis.any():
            d2 = cKDTree(hits / step).query(pts[vis] / step)[0]; out["visible_cell_to_hit_cells_max"] = float(d2.max()); out["visible_cell_covered_within_1p5_cells"] = float((d2 <= 1.5).mean())
        out["hits_within_1p5_cells_of_grid_frontier"] = float((d1 <= 1.5).mean())
    return out
# ------------------------------------------------------------------------------------------------ driver
def run_sweep(sc, opts=None):
    """opts: steps {joint: step_deg | [values]}, range 'soft'|'hard', procs, validate (list of pair keys or True), label."""
    opts = opts or {}; t0 = time.time(); vals = grid_values(sc, opts); pairs = build_pairs(sc); F = Fcl(sc); procs = opts.get("procs", 2)
    # designed contact at park
    designed = []
    for p in pairs:
        if p.key in sc.exclude: designed.append(True); continue
        d = _dist_pair(sc, F, p, sc.park); is_wearer = p.kind == "obj" and p.obj.startswith("wearer_")
        designed.append(bool(d < p.MM) and not is_wearer)          # designed contact = closer than MM at park (adjacent links, bearings, stops); never for the wearer (that is a real failure)
    # fixed pairs already in contact at park (not excluded unless asked): they simply make the park cell non-free -> reported
    tabs = tabulate(sc, pairs, vals, procs); t1 = time.time()
    res, rows, reach, frontier, contact_p = analyse(sc, pairs, tabs, vals, designed)
    fcp = first_contact_points(sc, pairs, tabs, vals, reach, contact_p, designed)
    for pi, p in enumerate(pairs):
        pts = fcp.get(p.key)
        if pts is not None and len(pts):
            J = sc.joints; k = int(np.argmin(pts[:, len(J)] - p.MM)); rows[p.key]["first_contact"] = {j: [round(float(pts[:, i].min()), 2), round(float(pts[:, i].max()), 2)] for i, j in enumerate(J) if j in p.dims or True}
            rows[p.key]["first_contact_example"] = {j: round(float(pts[k, i]), 2) for i, j in enumerate(J)}; rows[p.key]["n_first_contact_points"] = int(len(pts))
    out = dict(scene=sc.name, label=opts.get("label", ""), range=opts.get("range", "soft"), park=sc.park, MM=sc.MM, grid={j: [float(vals[j][0]), float(vals[j][-1]), int(len(vals[j]))] for j in sc.joints}, summary=res, pairs=rows,
               timing=dict(tabulate_s=round(t1 - t0, 1), total_s=round(time.time() - t0, 1)))
    if opts.get("validate"):
        rad = lipschitz_radii(sc, pairs); keys = opts["validate"] if opts["validate"] is not True else [p.key for p, d in zip(pairs, designed) if not d and len(p.dims) <= 3]
        tv = time.time(); out["validation"] = [validate_trace(sc, F, p, tabs[pi], vals, rad, opts.get("range", "soft")) for pi, p in enumerate(pairs) if p.key in keys]; out["timing"]["validate_s"] = round(time.time() - tv, 1)
    tag = opts.get("label", sc.name).replace(" ", "_"); json.dump(out, open(os.path.join(OUT, f"sweep_{tag}.json"), "w"), indent=1, default=float)
    np.savez_compressed(os.path.join(OUT, f"tables_{tag}.npz"), **{p.key.replace("|", "__"): tabs[pi] for pi, p in enumerate(pairs)}, **{"fcp__" + k.replace("|", "__"): v for k, v in fcp.items()})
    out["_internal"] = dict(pairs=pairs, tabs=tabs, vals=vals, reach=reach, frontier=frontier, fcp=fcp, designed=designed, F=F); return out
def table(out):
    s = out["summary"]; lines = [f"{out['scene']} [{out['label']}] range={out['range']} grid={out['grid']}", f"cells {s['n_cells']} contact {s['n_contact']} reach-free {s['n_reach_free']} frontier {s['n_frontier']} deep(dropped) {s['n_deep']}  time {out['timing']['total_s']} s", f"{'pair':44s} {'MM':>4s} {'park_d':>8s} {'min_free':>8s} {'d_front':>8s} {'nF':>5s} {'nD':>5s}  status"]
    for k, r in out["pairs"].items():
        f = lambda x: "-" if x is None else f"{x:8.1f}"
        lines.append(f"{k:44s} {r['MM']:4.0f} {r['park_d']:8.1f} {f(r.get('min_clear_reach_free'))} {f(r.get('d_frontier_min'))} {r.get('n_frontier_cells', 0):5d} {r.get('n_deep_cells', 0):5d}  {r['status']}")
    return "\n".join(lines)
# ------------------------------------------------------------------------------------------------ hints and swept volume
def local_opt_hints(out, near_factor=1.6, patch_r=18.0):
    """for each failing / near pair: worst pose, closest points (world), the gap to open, the fixed-surface patch (vertex ids within patch_r of the contact) to shave, move direction."""
    I = out["_internal"]; sc = I["F"].sc; F = I["F"]; hints = []; pairs = I["pairs"]
    for pi, p in enumerate(pairs):
        r = out["pairs"][p.key]
        if r.get("designed"): continue
        d_worst = r.get("d_frontier_min", r.get("min_clear_reach_free")); pose = r.get("worst_contact_pose") or r.get("worst_pose")
        if d_worst is None or pose is None or d_worst >= p.MM * near_factor: continue
        q = dict(sc.park); q.update(pose); T = sc.pose_fn(p.g, q)
        if p.kind == "obj": d, a, b = F.nearest_obj(p.g, T, p.obj); mesh = sc.objects()[p.obj][0]
        else: d, a, b = F.nearest_gg(p.other, sc.pose_fn(p.other, q), p.g, T); mesh = sc.groups[p.other]["mesh"]
        v = (a - b); nv = np.linalg.norm(v); dirv = v / nv if nv > 1e-6 else np.array([0, 0, 1.0])
        # direction from the contact point on the OBSTACLE towards the moving group (shave the obstacle along -dirv_to_group, or move the group along it)
        to_group = (a - b) if p.kind == "obj" else (b - a)
        vids = np.where(np.linalg.norm(np.asarray(mesh.vertices) - (b if p.kind == "obj" else a), axis=1) < patch_r)[0]
        hints.append(dict(pair=p.key, status=r["status"], d_mm=round(float(d), 2), need_mm=round(float(p.MM - d), 2), pose=pose, group_point=np.round(a if p.kind == "obj" else b, 2).tolist(), obstacle_point=np.round(b if p.kind == "obj" else a, 2).tolist(),
                          move_obstacle_away_from_group_dir=np.round(-to_group / max(1e-6, np.linalg.norm(to_group)), 3).tolist(), obstacle=p.obj, patch_vertex_ids=vids.tolist()[:400], patch_n=int(len(vids))))
    hints.sort(key=lambda h: h["d_mm"]); return hints
def swept_volume(sc, group, ranges, step=5.0, pitch=2.0, pad=0.0, base=None, nsamp=60000):
    """union of the group's poses over joint ranges {joint:(lo,hi)} (others at park or `base`), as a mesh grown by `pad`.  voxel occupancy from dense surface samples per pose."""
    from skimage import measure
    q0 = dict(base or sc.park); js = list(ranges); V = [np.arange(ranges[j][0], ranges[j][1] + 1e-6, step) for j in js]
    pts = trimesh.sample.sample_surface(sc.groups[group]["mesh"], nsamp)[0]; P4 = np.c_[pts, np.ones(len(pts))]; occ = set()
    for combo in itertools.product(*V):
        q = dict(q0); q.update(dict(zip(js, combo))); X = (sc.pose_fn(group, q) @ P4.T).T[:, :3]; occ.update(map(tuple, np.floor(X / pitch).astype(int)))
    A = np.array(list(occ)); lo = A.min(0) - 3; hi = A.max(0) + 4; G = np.zeros(tuple(hi - lo), bool); G[tuple((A - lo).T)] = True
    G = ndi.binary_closing(G, iterations=2); G = ndi.binary_fill_holes(G)
    if pad > 0: G = ndi.binary_dilation(G, structure=ndi.generate_binary_structure(3, 3), iterations=max(1, int(round(pad / pitch))))
    vv, ff, _, _ = measure.marching_cubes(np.pad(G, 1).astype(float), 0.5, spacing=(pitch,) * 3)
    m = trimesh.Trimesh(vv + (lo - 1) * pitch, ff); trimesh.repair.fix_normals(m); return m
def cut_by_sweep(fixed_mesh, sweep_mesh): return L.D(fixed_mesh, sweep_mesh)

def scene_from_kin(shoulder, fixed, wearer=None, kin=None, name="J1r2", exclude=(), MM=None, joints_upto="W3"):
    """whole-arm scene using cad/j1r2_kin.Kin: shoulder = {'M1': mesh, 'M2': mesh} (HOME-pose meshes of the yoke group and the S2-group), distal groups from Kin.proxies()
    (M3 upper arm+elbow ball, ME, MW1, MW2, MW3).  pose_fn(g,q) = Kin.group_transforms(q)[g].  joints chain S1p,S2,S3,E,W1,W2,W3."""
    import j1r2_kin as K, j1r2_params as P
    kin = kin or K.Kin(); order = ["M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"]; chain = ["S1p", "S2", "S3", "E", "W1", "W2", "W3"]
    px = kin.proxies(); meshes = dict(M1=shoulder["M1"], M2=shoulder["M2"])
    for g in order[2:]:
        if g in px: meshes[g] = L.U(*px[g].values()) if len(px[g]) > 1 else list(px[g].values())[0]
    groups = {g: dict(mesh=meshes[g], joints=chain[:i + 1]) for i, g in enumerate(order) if g in meshes}
    stop = chain.index(joints_upto) + 1; groups = {g: d for g, d in groups.items() if len(d["joints"]) <= stop}
    soft = {j: P.SOFT[j] for j in chain[:stop]}; park = {j: P.PARK[j] for j in chain[:stop]}
    cache = {}
    def pose_fn(g, q):
        key = tuple(round(q[j], 6) for j in chain[:stop])
        if cache.get("k") != key: cache["k"] = key; cache["v"] = kin.group_transforms({**{j: 0.0 for j in chain}, **q})
        return cache["v"][g]
    return Scene(fixed, groups, chain[:stop], pose_fn, park, soft, wearer=wearer, name=name, exclude=exclude, MM=MM)
# ------------------------------------------------------------------------------------------------ self-test scene on the J1 parts (kinematics from j1r2_params)
def test_scene(D=None):
    import j1r2_params as P
    D = D or P.derive(); Rx = lambda a: L.rot(np.radians(a), [1, 0, 0]); T = L.tr
    F00, FA00, s0, a0, d0 = P.shoulder_frames(D, 0.0, 0.0); P1 = np.array([D["PX"], D["y1"], D["z1"]]); TS, TA = P.TS, P.T_ARM; AKR, AKL = P.AKR, P.AKL
    place = lambda m, M: (lambda c: (c.apply_transform(M), c)[1])(m.copy())
    x0, XB, XO = D["x0"], D["XB"], D["XO"]; y1, z1 = D["y1"], D["z1"]
    fixed = dict(pad=L.cyl("x", 62.0, x0, x0 + P.PL_T, c=(y1, z1)), S1_AK70=L.cyl("x", AKR, XB, XO, c=(y1, z1)), S1_ring=L.I(L.cyl("x", 66.0, x0, XO + 14.0, c=(y1, z1), r_in=64.2), L.box(x0 - 1, XO + 15, y1 - 70, y1 + 70, z1 - 70, z1 + 70)))
    for k in ("rail", "wing"):
        pass
    env = L.env_parts(); fixed["wing"] = env["wing"]["mesh"]; fixed["rail"] = env["rail"]["mesh"]
    disc = L.cyl("x", 47.0, XO, XO + 12.0, c=(y1, z1)); bar = place(L.box(-6, 6, 0.0, TS + 3.0, -26, 26), F00)
    m1 = L.U(disc, bar, place(L.cyl("y", AKR, TS, TS + AKL, c=(0, 0)), F00), place(L.cyl("y", AKR + 4.0, TS + 4.0, TS + 34.0, c=(0, 0), r_in=AKR + 0.1), F00), place(L.cyl("y", AKR + 2.6, TS - 1.0, TS + AKL, c=(0, 0), r_in=AKR + 0.8), F00))
    ad = place(L.cyl("y", 40.0, TS + AKL, TA, c=(0, 0)), FA00); brk = place(L.box(-24, 24, TS + AKL, TA + 12.0, 6.0, 70.0), FA00); xm = place(L.box(-29.25, 29.25, TA - 16.75, TA + 16.75, 68.0, 112.0), FA00)
    m2 = L.U(ad, brk, xm)
    m3 = L.U(place(L.cyl("z", 32.0, 112.0, 190.0, c=(0, TA)), FA00), place(L.sph(44.0, (0, TA, 230.0)), FA00))
    groups = dict(M1=dict(mesh=m1, joints=["S1p"]), M2=dict(mesh=m2, joints=["S1p", "S2"]), M3=dict(mesh=m3, joints=["S1p", "S2", "S3"]))
    FA0inv = np.linalg.inv(FA00); F0inv = np.linalg.inv(F00)
    def pose_fn(g, q):
        F0, FA, s, a, d = P.shoulder_frames(D, q["S1p"], q["S2"]); T1 = F0 @ F0inv
        if g == "M1": return T1
        T2 = FA @ FA0inv
        if g == "M2": return T2
        ref = np.array([D["PX"], D["y1"] + TA, D["z1"]]); Rz = L.rot(np.radians(q["S3"]), [0, 0, 1], point=ref); return T2 @ Rz
    sc = Scene(fixed, groups, ["S1p", "S2", "S3"], pose_fn, dict(S1p=P.PARK["S1p"], S2=P.PARK["S2"], S3=0.0), P.SOFT and {k: P.SOFT[k] for k in ("S1p", "S2", "S3")}, wearer=make_wearer_parts(), name="J1_selftest")
    return sc
def full_tier_test(label="selftest_fulltier"):
    """whole arm (kin proxies) vs the test fixed set + wearer on the project's tier grid (S1' 10, S2 10, S3 {-70,0,90}, E 20, W1/W2/W3 3 values)."""
    sc0 = test_scene(); sc = scene_from_kin(dict(M1=sc0.groups["M1"]["mesh"], M2=sc0.groups["M2"]["mesh"]), sc0.fixed, sc0.wearer, name="J1r2_fulltier")
    opts = dict(steps={"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}, procs=2, label=label)
    out = run_sweep(sc, opts); print(table(out)); print("timing", out["timing"]); return out
if __name__ == "__main__":
    import j1r2_params as P
    if len(sys.argv) > 1 and sys.argv[1] == "full": full_tier_test(); sys.exit(0)
    t0 = time.time(); sc = test_scene(); print("scene built", round(time.time() - t0, 1), "s;", {k: len(v.faces) for k, v in sc.wearer.items()})
    opts = dict(steps={"S1p": 2.5, "S2": 2.5, "S3": [-70.0, 0.0, 90.0]}, procs=2, label="selftest_soft", validate=True)
    out = run_sweep(sc, opts); print(table(out)); print(json.dumps(out.get("validation"), indent=1)[:3500]); print("timing", out["timing"])
