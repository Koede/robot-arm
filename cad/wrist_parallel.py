"""Parallel wrist (spherical five-bar) proxy geometry, right arm, MW1/forearm frame (= robot frame at q = 0), mm.

Reads stl/wrist_parallel/kin.json (or an alternative params dict). All joint axes meet at the wrist centre C = (0, D_OFF, Z_W3).
Links are chains of boxes along great-circle arcs on spheres of radius R_W + k*LAYER (k = 0..3) so that adjacent links are stacked
radially and a pivot pin (M2.5) goes through both:
    layer 0  crank 1   (J0 -> J1), servo A on the J0 axis outside the sphere
    layer 1  floater 1 (J1 -> J2)
    layer 2  floater 2 (J2 -> J3) + carrier strut to the hand tray
    layer 3  crank 2   (J3 -> J4), servo B on the J4 axis outside the sphere
Servos are STS3215 proxies (ST_W x ST_L x ST_H from distal_arm.py, shaft SHAFT_OFF from the body centre, shaft along ST_H) fixed to the
forearm distal tube by a thin strut (bracket proxy). Hand = claw tray + G servo (+ fingers) rigid with floater 2.
Writes meshes in the reference pose (serial target W2 = W3 = 0) to stl/wrist_parallel/ and wrist_parallel_layout.json.
Usage: python3 wrist_parallel.py
"""
import os, sys, json, numpy as np, trimesh
from scipy.optimize import least_squares
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import wrist_parallel_kin as wk
ROOT = os.path.join(HERE, "..", "stl"); OUT = os.path.join(ROOT, "wrist_parallel")
LAY = json.load(open(os.path.join(ROOT, "distal", "distal.json")))["_layout"]
D_OFF, Z_W3, Z_W = LAY["D_OFF"], LAY["Z_W3"], LAY["Z_W"]
C = np.array([0.0, D_OFF, Z_W3])                       # wrist centre (robot frame = MW1 frame at q = 0)
ST_L, ST_W, ST_H, SHAFT_OFF, HORN_D, HORN_P = 45.2, 24.7, 35.0, 11.0, 20.0, 4.0   # distal_arm.py
R_W = 70.0; LAYER = 8.0; W_LINK = 8.0; T_LINK = 6.0; D_BOSS = 10.0; D_PIN = 2.5
KEYS = ("az0", "pol0", "az4", "pol4", "a1", "a2", "b1", "b2")

def load_params(path=None):
    d = json.load(open(path or os.path.join(OUT, "kin.json")))["params"]
    return np.array([d[k] for k in KEYS], float)

def unit(v): v = np.asarray(v, float); return v / np.linalg.norm(v, axis=-1, keepdims=True)

# ------------------------------------------------------------------------------------------------ five-bar kinematics (unit vectors)
def points(params, th1, th2):
    """both assembly branches: returns dict J1, J2, J3, h each (N, 2, 3), ok (N,), J0, J4 (3,). th in deg (arrays)."""
    az0, pol0, az4, pol4, a1, a2, b1, b2 = params
    J0, J4 = wk.ground_pts((az0, pol0, az4, pol4))
    th1 = np.atleast_1d(np.radians(th1)); th2 = np.atleast_1d(np.radians(th2))
    J1 = wk.on_sphere_point(J0, np.radians(a1), th1); J3 = wk.on_sphere_point(J4, np.radians(a2), th2)
    d = np.cross(J1, J3); dd = np.einsum("ij,ij->i", d, d); ok = dd > 1e-10
    c1, c2 = np.cos(np.radians(b1)), np.cos(np.radians(b2))
    X0 = (c1 * np.cross(J3, d) + c2 * np.cross(d, J1)) / np.where(ok, dd, 1.0)[:, None]
    s2 = 1 - np.einsum("ij,ij->i", X0, X0); ok &= s2 >= 0
    s = np.sqrt(np.clip(s2, 0, None)); dh = unit(d)
    J2 = np.stack([X0 + sg * s[:, None] * dh for sg in (1.0, -1.0)], 1)
    h = unit(np.cross(J2, J3[:, None, :]))
    n = len(th1)
    return dict(J1=np.repeat(J1[:, None, :], 2, 1), J2=J2, J3=np.repeat(J3[:, None, :], 2, 1), h=h, ok=ok, J0=J0, J4=J4)

def one(params, th1, th2, sg):
    P = points(params, [th1], [th2]); return {k: (v[0, sg] if k in ("J1", "J2", "J3", "h") else v) for k, v in P.items()}

def sigma_min(params, th1, th2, sg, eps=1e-3):
    h0 = one(params, th1, th2, sg)["h"]
    a = (one(params, th1 + eps, th2, sg)["h"] - h0) / np.radians(eps); b = (one(params, th1, th2 + eps, sg)["h"] - h0) / np.radians(eps)
    return float(np.sqrt(max(np.linalg.eigvalsh(np.array([[a @ a, a @ b], [a @ b, b @ b]]))[0], 0)))

def serial_frame(w2, w3):
    """claw frame of the serial wrist in the MW1 frame (3x3, columns = x, y, z of the hand), rotation about the wrist axes only."""
    from trimesh.transformations import rotation_matrix as rot
    return (rot(np.radians(-w2), [0, 1, 0]) @ rot(np.radians(w3), [1, 0, 0]))[:3, :3]

def carrier_frame(J2, J3):
    h = unit(np.cross(J2, J3)); return np.stack([J2, np.cross(h, J2), h], 1)

# ------------------------------------------------------------------------------------------------ inverse (claw axis -> cranks), branch tracking
_GRID = {}
def _grid(params, n=181):
    key = (tuple(np.round(params, 6)), n)
    if key not in _GRID:
        t = np.linspace(0, 360, n, endpoint=False); T1, T2 = np.meshgrid(t, t, indexing="ij"); T1 = T1.ravel(); T2 = T2.ravel()
        P = points(params, T1, T2); _GRID[key] = (T1, T2, P["h"], P["ok"])
    return _GRID[key]

def wrap(a): return (a + 180.0) % 360.0 - 180.0

def candidates(params, axis, tol_deg=2.0, cluster=12.0):
    """all (th1, th2, sg, err_deg, sigma) solutions of the claw axis, refined, one per cluster."""
    T1, T2, H, ok = _grid(params); out = []
    for sg in (0, 1):
        ang = np.degrees(np.arccos(np.clip(H[:, sg] @ axis, -1, 1))); idx = np.where(ok & (ang < tol_deg))[0]
        taken = []
        for i in idx[np.argsort(ang[idx])]:
            if any(abs(wrap(T1[i] - T1[j])) < cluster and abs(wrap(T2[i] - T2[j])) < cluster for j in taken): continue
            taken.append(i)
            f = lambda x: one(params, x[0], x[1], sg)["h"] - axis if one(params, x[0], x[1], sg)["ok"] else np.ones(3)
            r = least_squares(f, [T1[i], T2[i]], xtol=1e-10, ftol=1e-12)
            err = float(np.degrees(np.arccos(np.clip(one(params, *r.x, sg)["h"] @ axis, -1, 1))))
            if err < 1.5: out.append(dict(th1=float(r.x[0]), th2=float(r.x[1]), sg=sg, err=err, sigma=sigma_min(params, r.x[0], r.x[1], sg)))
    return out

def _pos(params, c):
    Q = one(params, c["th1"], c["th2"], c["sg"]); return np.concatenate([Q["J1"], Q["J2"], Q["J3"]])

def _pdist(a, b):
    """largest angular distance (deg) between corresponding pivot points J1, J2, J3."""
    return max(float(np.degrees(np.arccos(np.clip(a[3 * i:3 * i + 3] @ b[3 * i:3 * i + 3], -1, 1)))) for i in range(3))

def _bfs(params, cand, pos, w2s, w3s, start, first, s_min):
    sol = {start: dict(first, jump=0.0)}; queue = [start]
    while queue:
        cur = queue.pop(0)
        for da in (-1, 0, 1):
            for db in (-1, 0, 1):
                ia, ib = list(w2s).index(cur[0]) + da, list(w3s).index(cur[1]) + db
                if (da == db == 0) or not (0 <= ia < len(w2s) and 0 <= ib < len(w3s)): continue
                k = (w2s[ia], w3s[ib])
                if k in sol: continue
                ref = sol[cur]; rp = pos[(cur, id(ref))] if (cur, id(ref)) in pos else _pos(params, ref)
                cs = cand[k]
                if cs:
                    sc = [(_pdist(rp, _pos(params, c)) + (15.0 if c["sigma"] < s_min else 0.0), c) for c in cs]
                    d, c = min(sc, key=lambda t: t[0]); c = dict(c, jump=float(_pdist(rp, _pos(params, c))))
                else: c = dict(ref, err=999.0, sigma=0.0, jump=0.0)
                sol[k] = c; queue.append(k)
    return sol

def solve_grid(params, w2s, w3s, s_min=wk.S_MIN):
    """branch-continuous solution for every serial target: try each of the solutions at (0,0) as the start, BFS over 8-neighbours choosing the
    candidate with the smallest pivot-point jump, keep the family with the fewest failures / smallest max jump / best conditioning."""
    cand = {(a, b): candidates(params, wk.serial_axis(a, b)) for a in w2s for b in w3s}
    start = (0, 0) if (0, 0) in cand else (w2s[len(w2s) // 2], w3s[len(w3s) // 2])
    best = None
    for c0 in cand[start]:
        sol = _bfs(params, cand, {}, w2s, w3s, start, c0, s_min)
        key = (sum(v["err"] > 1.5 for v in sol.values()), sum(v["sigma"] < s_min for v in sol.values()), max(v["jump"] for v in sol.values()))
        if best is None or key < best[0]: best = (key, sol)
    return best[1]

# ------------------------------------------------------------------------------------------------ geometry helpers
def frame_from(z, ref=None):
    z = unit(z); a = np.array([0.0, 0, 1]) if abs(z[2]) < 0.9 else np.array([1.0, 0, 0])
    if ref is not None: a = ref - (ref @ z) * z if np.linalg.norm(ref - (ref @ z) * z) > 1e-6 else a
    x = unit(np.cross(np.cross(z, a), z)) if ref is None else unit(a); y = np.cross(z, x)
    return np.stack([x, y, z], 1)

def box_between(p, q, w, t, up):
    """box from p to q (length |q-p|), width w along `up`-perpendicular side, thickness t along `up`."""
    v = q - p; L = float(np.linalg.norm(v)); ex = v / L; ez = unit(up - (up @ ex) * ex); ey = np.cross(ez, ex)
    m = trimesh.creation.box(extents=[L + 0.0, w, t]); T = np.eye(4); T[:3, :3] = np.stack([ex, ey, ez], 1); T[:3, 3] = (p + q) / 2
    m.apply_transform(T); return m

def cyl(p, axis, r, length):
    m = trimesh.creation.cylinder(radius=r, height=length, sections=24); ez = unit(axis); T = np.eye(4); T[:3, :3] = frame_from(ez); T[:3, 3] = p
    m.apply_transform(T); return m

def arc_chain(ua, ub, r, steps_deg=7.0, ext_deg=0.0):
    """great-circle chain of boxes from unit ua to unit ub at radius r about C (+ bosses d10 at both ends, radial). Returns list of meshes."""
    ang = float(np.arccos(np.clip(ua @ ub, -1, 1))); n = max(2, int(np.ceil(np.degrees(ang) / steps_deg)))
    ax = unit(np.cross(ua, ub)); out = []
    from trimesh.transformations import rotation_matrix as rot
    pts = [ (rot(ang * i / n, ax)[:3, :3] @ ua) for i in range(n + 1)]
    for i in range(n):
        p, q = C + r * pts[i], C + r * pts[i + 1]; mid = unit(pts[i] + pts[i + 1])
        out.append(box_between(p, q, W_LINK, T_LINK, mid))
    for u in (ua, ub): out.append(cyl(C + r * u, u, D_BOSS / 2, T_LINK))
    return out

def pin(u, r_in, r_out):
    return cyl(C + (r_in + r_out) / 2 * u, u, D_PIN / 2, r_out - r_in)

def servo_mesh(u, r_face, psi_deg):
    """STS3215 proxy with shaft on unit u (outwards from C), horn face at radius r_face; long axis direction rotated psi about u."""
    from trimesh.transformations import rotation_matrix as rot
    F = frame_from(u); xl = rot(np.radians(psi_deg), u)[:3, :3] @ F[:, 0]; zl = unit(u); yl = np.cross(zl, xl)
    body = trimesh.creation.box(extents=[ST_W, ST_L, ST_H])
    T = np.eye(4); T[:3, :3] = np.stack([xl, yl, zl], 1)
    T[:3, 3] = C + zl * (r_face + ST_H / 2) - yl * SHAFT_OFF                    # shaft SHAFT_OFF from the centre toward +y_l
    body.apply_transform(T); return body, np.stack([xl, yl, zl], 1)

def strut(points_list, w=10.0, t=5.0):
    out = []
    for p, q in zip(points_list[:-1], points_list[1:]):
        up = np.cross(q - p, [0.0, 0, 1]); up = up if np.linalg.norm(up) > 1e-6 else np.array([1.0, 0, 0])
        out.append(box_between(p, q, w, t, up))
    return trimesh.util.concatenate(out)

# ------------------------------------------------------------------------------------------------ the mechanism at a given crank pose
def layers(): return [R_W + k * LAYER for k in range(4)]

def build_ref(params, th1, th2, sg, psiA=0.0, psiB=0.0, hand=True):
    """reference-pose bodies (world frame). Returns dict name -> mesh, dict of unit-vector points and servo frames."""
    P = one(params, th1, th2, sg); J0, J1, J2, J3, J4, h = P["J0"], P["J1"], P["J2"], P["J3"], P["J4"], P["h"]
    r0, r1, r2, r3 = layers(); B = {}
    # crank 1 (layer 0): arc J0 -> J1, pin J0 stub to the horn, pin J1 (through layers 0..1)
    B["crank1"] = trimesh.util.concatenate(arc_chain(J0, J1, r0) + [pin(J1, r0 - T_LINK / 2 - 1.5, r1 + T_LINK / 2 + 1.5), cyl(C + (r0 + T_LINK / 2 + HORN_P / 2) * J0, J0, HORN_D / 2, HORN_P)])
    B["float1"] = trimesh.util.concatenate(arc_chain(J1, J2, r1) + [pin(J2, r1 - T_LINK / 2 - 1.5, r2 + T_LINK / 2 + 1.5)])
    # floater 2 (layer 2) + carrier strut radial inward from its mid-point to the hand tray
    mid = unit(J2 + J3); f2 = arc_chain(J2, J3, r2) + [pin(J3, r2 - T_LINK / 2 - 1.5, r3 + T_LINK / 2 + 1.5)]
    f2.append(box_between(C + r2 * mid, C + 30.0 * mid - 12.0 * h, 14.0, T_LINK, h))     # strut: mid-point -> tray flange (sits ~12 mm below the equator)
    B["float2"] = trimesh.util.concatenate(f2)
    B["crank2"] = trimesh.util.concatenate(arc_chain(J3, J4, r3) + [cyl(C + (r3 + T_LINK / 2 + HORN_P / 2) * J4, J4, HORN_D / 2, HORN_P)])
    sA, FA = servo_mesh(J0, r0 + T_LINK / 2 + HORN_P + 0.5, psiA); sB, FB = servo_mesh(J4, r3 + T_LINK / 2 + HORN_P + 0.5, psiB)
    B["servoA"], B["servoB"] = sA, sB
    # brackets: servo body centre -> waypoint outside the link spheres -> rim of the forearm distal tube (z = Z_W - 23.5)
    z_rim = Z_W - 23.5; brs = {}
    for nm, (u, S) in dict(brA=(J0, sA), brB=(J4, sB)).items():
        c = S.centroid; d = unit(0.45 * u + 0.9 * np.array([0.0, 0, -1])); Wp = C + 120.0 * d
        hd = np.array([Wp[0], Wp[1] - D_OFF, 0.0]); hd = unit(hd) if np.linalg.norm(hd) > 1e-6 else np.array([1.0, 0, 0])
        rim = np.array([0.0, D_OFF, z_rim]) + 29.0 * hd
        B[nm] = strut([c, Wp, rim])
    if hand:
        B["tray"] = hand_meshes(with_fingers=FINGERS)
    return B, dict(J0=J0, J1=J1, J2=J2, J3=J3, J4=J4, h=h, servoA_frame=FA, servoB_frame=FB)

_HAND = {}; FINGERS = True
def hand_meshes(with_fingers=True):
    """hand body (claw tray + G servo proxy + claw parts at G = 50 %) in the hand frame = W3 frame at q = 0 (world frame at the reference pose)."""
    if "m" not in _HAND:
        ms = [trimesh.load(os.path.join(ROOT, "distal", "claw", "hand_tray.stl")), trimesh.load(os.path.join(ROOT, "distal", "claw", "proxies", "sts_g.stl"))]
        if with_fingers:
            try:
                import arm_kin as ak
                for k, v in ak.claw_parts(50.0).items():
                    if k not in ("hand_tray", "sts_g"): ms.append(v)
            except Exception as e: print("claw fingers skipped:", e)
        _HAND["m"] = trimesh.util.concatenate(ms)
    return _HAND["m"].copy()

# ------------------------------------------------------------------------------------------------ main: build the reference pose and write the STLs + layout
def main(params=None, tag="", psi=(0.0, 0.0)):
    params = load_params() if params is None else np.asarray(params, float)
    sol = solve_grid(params, [-10, 0, 10], [-10, 0, 10]); s = sol[(0, 0)]
    B, pts = build_ref(params, s["th1"], s["th2"], s["sg"], *psi)
    os.makedirs(OUT, exist_ok=True)
    for k, m in B.items():
        if k != "tray": m.export(os.path.join(OUT, f"wp_{k}{tag}.stl"))
    lay = dict(C=C.tolist(), R_W=R_W, layer_radii=layers(), link_w=W_LINK, link_t=T_LINK, boss_d=D_BOSS, pin_d=D_PIN, params=dict(zip(KEYS, params.tolist())),
               ref_pose=dict(W2=0, W3=0, **s), points={k: (v.tolist() if hasattr(v, "tolist") else v) for k, v in pts.items()},
               servo_proxy=dict(ST_W=ST_W, ST_L=ST_L, ST_H=ST_H, SHAFT_OFF=SHAFT_OFF), bodies=sorted(B),
               adjacency=[["servoA", "crank1"], ["crank1", "float1"], ["float1", "float2"], ["float2", "crank2"], ["crank2", "servoB"], ["float2", "tray"]],
               note="proxy geometry for collision / layout study only; tray body = hand_tray + sts_g + claw parts (G 50%), rigid with floater 2")
    json.dump(lay, open(os.path.join(OUT, f"wrist_parallel_layout{tag}.json"), "w"), indent=1)
    print("reference solution", s); print("bodies:", {k: (len(m.faces)) for k, m in B.items()})

def wp_load(a): return load_params(a[a.index("--params") + 1]) if "--params" in a else None
if __name__ == "__main__":
    a = sys.argv; main(wp_load(a), a[a.index("--tag") + 1] if "--tag" in a else "", (180.0, 180.0) if "--psi180" in a else (0.0, 0.0))
