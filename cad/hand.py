"""HAND rev B CAD: organic SDF parts (marching cubes) + exact booleans (manifold) for pins, grooves, pockets.
Part-local frame for digits: X = a (along the phalanx from its proximal joint), Y = t (width), Z = b (palmar).  Palm/base in the hand frame (x, u, z).
Outputs stl/hand/<part>.stl (+ proxies/), hand_parts.json.  PLACEHOLDERS: micro-servo envelopes (MG90S class), horn/crank, rod ends, M2 cover screws."""
import numpy as np, trimesh, json, os, sys
from skimage import measure
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K
OUT = K.OUT; os.makedirs(OUT + "/proxies", exist_ok=True)
FB = json.load(open(OUT + "/fourbars.json"))
FBT = K.fourbar_fit(K.TLEN[1], K.K_TIP, 75.0); json.dump(FBT, open(OUT + "/fourbar_thumb.json", "w"))
def smin(a, b, k):
    h = np.clip(0.5 + 0.5*(b - a)/k, 0, 1); return b*(1 - h) + a*h - k*h*(1 - h)
def mc(sdf_fn, lo, hi, vox):
    lo, hi = np.array(lo, float) + np.array([0.0137, 0.0291, 0.0173])*vox, np.array(hi, float); n = np.ceil((hi - lo)/vox).astype(int) + 1
    g = [np.linspace(lo[i], lo[i] + vox*(n[i] - 1), n[i]) for i in range(3)]
    X, Y, Z = np.meshgrid(*g, indexing='ij'); F = sdf_fn(X, Y, Z)
    F[0, :, :] = F[-1, :, :] = F[:, 0, :] = F[:, -1, :] = F[:, :, 0] = F[:, :, -1] = 1.0
    v, f, _, _ = measure.marching_cubes(F, 1e-4, spacing=(vox, vox, vox)); m = trimesh.Trimesh(v + lo, f[:, ::-1], process=False)
    m.merge_vertices(); trimesh.repair.fix_normals(m)
    if not m.is_volume:
        import manifold3d as M3
        mm = M3.Manifold(M3.Mesh(np.asarray(m.vertices, np.float32), np.asarray(m.faces, np.uint32))); o = mm.to_mesh(); m = trimesh.Trimesh(o.vert_properties[:, :3], o.tri_verts)
    if m.volume < 0: m.invert()
    return m
def B(a, b, op):
    return getattr(a, op)(b, engine='manifold') if op != 'union' else trimesh.boolean.union([a, b], engine='manifold')
def cyl(r, p0, p1, sec=28):
    p0, p1 = np.array(p0, float), np.array(p1, float); return trimesh.creation.cylinder(radius=r, segment=[p0, p1], sections=sec)
def boxm(lo, hi):
    lo, hi = np.array(lo, float), np.array(hi, float); b = trimesh.creation.box(extents=hi - lo); b.apply_translation((lo + hi)/2); return b
# ------------------------------------------------------------------ finger geometry
def taper(nm):
    L = K.LEN[nm]; Lt = sum(L); (w0, w1), (h0, h1) = K.WID[nm], K.THK[nm]
    return (lambda f: w0 + (w1 - w0)*f), (lambda f: h0 + (h1 - h0)*f), Lt
def ell_capsule(A, T, Bv, a0, a1, w_f, h_f):
    s = np.clip((A - a0)/(a1 - a0), 0, 1); w, h = w_f(s), h_f(s); ac = a0 + s*(a1 - a0)
    rad = np.sqrt((T/w)**2 + (Bv/h)**2); m = np.minimum(w, h)
    return np.sqrt(((A - ac))**2 + (np.maximum(rad - 1, -1e9)*m)**2) * np.where(rad > 1, 1, 1) * np.sign(np.maximum(rad - 1, 0) + np.abs(A - ac) + 1e-12) if False else np.where((A >= a0) & (A <= a1), (rad - 1)*m, np.sqrt((A - ac)**2 + np.maximum((rad - 1)*m, 0)**2) - 0.0)
def phalanx(nm, k):
    """k = 0 prox, 1 mid, 2 dist."""
    L = K.LEN[nm]; wf, hf, Lt = taper(nm); start = sum(L[:k]); Lk = L[k]
    W = lambda s: wf((start + s*Lk)/Lt); Hh = lambda s: hf((start + s*Lk)/Lt)
    rb = 0.9*min(W(0), Hh(0))                                           # joint ball radius at the proximal end
    tip = (k == 2)
    def sdf(A, T, Bv):
        a1 = Lk - (K.TIP_R - 0.3) if tip else Lk
        body = ell_capsule(A, T, Bv, 0.0, a1, W if not tip else (lambda s: W(s*a1/Lk)), Hh if not tip else (lambda s: Hh(s*a1/Lk)))
        if tip:
            sph = np.sqrt((A - a1)**2 + T**2 + (Bv - 0.8)**2) - (K.TIP_R - 1.2)        # fingertip core (cap adds 1.2)
            body = smin(body, sph, 3.0)
        ball = np.sqrt(A**2 + T**2 + Bv**2) - rb
        return smin(body, ball, 2.5)
    wmax = max(W(0), K.TIP_R) + 1.5; hmax = max(Hh(0), K.TIP_R) + 1.5
    m = mc(sdf, (-rb - 1.5, -wmax, -hmax), (Lk + 2.0, wmax, hmax), 0.32)
    # cup for the next ball (prox, mid)
    if not tip:
        nwf = W(1.0); nhf = Hh(1.0); rbn = 0.9*min(nwf, nhf)
        m = B(m, trimesh.creation.icosphere(subdivisions=4, radius=rbn + 0.4).apply_translation([Lk, 0, 0]), 'difference')
        m = B(m, cyl(0.95, [Lk, -12, 0], [Lk, 12, 0]), 'difference')          # next joint pin: press in the cheeks
    m = B(m, cyl(1.05, [0, -12, 0], [0, 12, 0]), 'difference')                  # own joint pin: running in the ball
    # dorsal groove for the coupling bars (open on the dorsal side b<0), floor at b = +3.9
    g_lo = -rb - 2 if k > 0 else -rb - 2; g_hi = (Lk + 1.0) if not tip else 7.0
    m = B(m, boxm([g_lo, -3.1, -20], [g_hi, 3.1, 3.9]), 'difference')
    fb = FB[nm]
    # bar pins: G of the PIP bar sits on the carrier; H of the PIP bar on the mid phalanx; G of the DIP bar on the prox (origin at PIP); H of the DIP bar on dist
    pins = []
    if k == 1: pins.append(fb['pip']['H'])
    if k == 0: pins.append([Lk + fb['dip']['G'][0], fb['dip']['G'][1]])
    if k == 2: pins.append(fb['dip']['H'])
    for (pa, pb) in pins: m = B(m, cyl(0.95, [pa, -12, pb], [pa, 12, pb]), 'difference')
    if k == 0:   # palmar flexor lever lug (rod end pin along t) below the ball
        lug = B(trimesh.creation.box(extents=[5.0, 5.0, 5.0]).apply_translation([-1.0, 0, rb + 1.6]), cyl(0.95, [-1.0, -4, rb + 2.2], [-1.0, 4, rb + 2.2]), 'difference')
        m = B(m, lug, 'union')
    return m, rb
def tip_cap(nm):
    L = K.LEN[nm]; wf, hf, Lt = taper(nm); start = L[0] + L[1]; Lk = L[2]; a1 = Lk - (K.TIP_R - 0.3)
    W = lambda s: wf((start + s*a1)/Lt); Hh = lambda s: hf((start + s*a1)/Lt)
    def core(A, T, Bv):
        body = ell_capsule(A, T, Bv, 0.0, a1, W, Hh); sph = np.sqrt((A - a1)**2 + T**2 + (Bv - 0.8)**2) - (K.TIP_R - 1.2)
        return smin(body, sph, 3.0)
    sdf = lambda A, T, Bv: np.maximum.reduce([core(A, T, Bv) - 1.2, -core(A, T, Bv) - 0.02, (a1 - 7.0) - A])
    return mc(sdf, (a1 - 8, -K.TIP_R - 2, -K.TIP_R - 2), (Lk + 2, K.TIP_R + 2, K.TIP_R + 2.5), 0.22)
def bar(lb, t0, t1, r=1.7):
    m = trimesh.creation.capsule(height=lb, radius=r, count=[20, 20]); m.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [0, 1, 0])); m.apply_translation([lb/2, 0, 0])
    m = B(m, boxm([-5, t0, -5], [lb + 5, t1, 5]), 'intersection')
    for x in (0, lb): m = B(m, cyl(1.05, [x, -5, 0], [x, 5, 0]), 'difference')
    return m
def carrier(nm, rb_prox):
    """MCP carrier: cheeks around the proximal ball + abduction pivot boss (axis x); sits in the palm knuckle pocket.  Local finger frame at the MCP."""
    w = rb_prox + 1.4; h = K.THK[nm][0]
    def sdf(A, T, Bv):
        q = np.stack([np.abs(A + 2.5) - 5.0, np.abs(T) - (w - 1.0), np.abs(Bv) - h*0.85]); out = np.linalg.norm(np.maximum(q, 0), axis=0) + np.minimum(q.max(0), 0) - 1.0
        return out
    m = mc(sdf, (-12, -w - 3, -h - 3), (5, w + 3, h + 3), 0.3)
    m = B(m, trimesh.creation.icosphere(subdivisions=4, radius=rb_prox + 0.4), 'difference')
    m = B(m, boxm([-3, -3.1, -20], [8, 3.1, 3.9]), 'difference')                       # bar groove continues
    m = B(m, boxm([-1, -w - 4, -0.5], [10, w + 4, 20]), 'difference')                  # flex sweep opening on the palmar side
    m = B(m, cyl(0.95, [0, -12, 0], [0, 12, 0]), 'difference')                        # MCP pin (press)
    G = FB[nm]['pip']['G']; m = B(m, cyl(0.95, [G[0], -12, G[1]], [G[0], 12, G[1]]), 'difference')
    m = B(m, cyl(1.05, [-6.0, 0, -20], [-6.0, 0, 20]), 'difference')                  # abduction pivot pin along b (PLACEHOLDER: kinematics assume the MCP; offset 6 mm adds < 0.6 mm tip error at 10 deg)
    return m
# ------------------------------------------------------------------ palm (hand frame x, u, z)
SV = dict(L=22.8, W=12.2, H=22.7)                                                  # PLACEHOLDER micro-servo
SERVO_U = dict(index=19.0, middle=5.5, ring=-8.0, pinky=-21.5)
Z_SV0 = 518.2; Z_SHAFT = Z_SV0 + SV['L'] - 6.0; X_SV0 = -10.5
THEN = [dict(u0=26.0, z0=494.0), dict(u0=26.0, z0=517.6)]                         # two thenar pockets (thumb swing / lift); thumb flex servo in the carpal block
CARP = dict(u0=-22.0, z0=494.0)                                                    # abduction servo + thumb flex share the carpal row
def palm_sdf(X, U, Z):
    def rbox(c, hs, r):
        q = np.stack([np.abs(X - c[0]) - hs[0] + r, np.abs(U - c[1]) - hs[1] + r, np.abs(Z - c[2]) - hs[2] + r]); return np.linalg.norm(np.maximum(q, 0), axis=0) + np.minimum(q.max(0), 0) - r
    def ell(c, r):
        p = np.stack([(X - c[0])/r[0], (U - c[1])/r[1], (Z - c[2])/r[2]]); k0 = np.linalg.norm(p, axis=0); return (k0 - 1)*min(r)
    d = rbox((1.0, -4.0, 521.5), (13.2, 34.5, 29.0), 9.0)
    d = smin(d, ell((5.0, 27.0, 512.0), (12.5, 13.0, 21.0)), 5.0)                  # thenar
    d = smin(d, ell((2.0, -33.0, 515.0), (11.0, 7.0, 22.0)), 5.0)                   # hypothenar
    d = smin(d, ell((0.0, 0.0, 495.0), (17.5, 27.0, 8.0)), 6.0)                     # carpal / wrist blend
    for nm in K.LEN: d = smin(d, ell((-6.5, K.U[nm], K.ZM[nm] - 1.0), (6.5, 8.0, 7.5)), 3.0)    # dorsal knuckle bumps (hood over the carriers)
    return d
def palm():
    m = mc(palm_sdf, (-20, -50, 484), (24, 46, 556), 0.5)
    # wrist interface (contract: W3 ears, floor plate HB0 491.6 .. FT 493.6)
    for sx in (-1, 1):
        x0, x1 = sorted([sx*23.3, sx*26.8]); ear = cyl(16.0, [x0, 0, 455.0], [x1, 0, 455.0], 48)
        ear = B(ear, boxm([x0, -14, 455.0], [x1, 14, 494.6]), 'union'); m = B(m, ear, 'union')
    m = B(m, boxm([-26.8, -16, 491.6], [26.8, 16, 493.6]), 'union')
    m = B(m, boxm([-17.5 - 3.2, -12.3 - 3.2, 440.0], [17.5 + 3.2, 12.3 + 3.2, 491.6]), 'difference')     # W3 servo body relief between the ears
    # MCP pockets + flex sweep, carrier clearance (index/ring abduct +-25 deg)
    for nm in K.LEN:
        u, z = K.U[nm], K.ZM[nm]; w = K.WID[nm][0]; ab = 4.5 if nm in ('index', 'ring') else 0.8
        hw = w + ab + 1.0
        m = B(m, boxm([-7.6, u - hw, z - 11.0], [9.0, u + hw, 600]), 'difference')          # carrier + ball pocket (dorsal hood x < -7.6 stays)
        m = B(m, boxm([-2.5, u - 3.6, z - 15.0], [16.0, u + 3.6, 600]), 'difference')         # flexor lever + rod slot
        m = B(m, boxm([-7.6, u - hw, z - 7.0], [30.0, u + hw, 600]), 'difference')           # proximal phalanx flex sweep above the palmar pad
    # servo pockets (+0.3)
    def pocket(lo, hi): return boxm(np.array(lo) - 0.3, np.array(hi) + 0.3)
    for nm, us in SERVO_U.items():
        m = B(m, pocket([X_SV0, us - SV['W']/2, Z_SV0], [X_SV0 + SV['H'], us + SV['W']/2, Z_SV0 + SV['L']]), 'difference')
        m = B(m, boxm([X_SV0 + SV['H'], us - 7, Z_SHAFT - 10], [X_SV0 + SV['H'] + 3.5, us + 7, 552]), 'difference')   # crank + rod channel under the cover
    for th in THEN: m = B(m, pocket([X_SV0, th['u0'], th['z0']], [X_SV0 + SV['H'], th['u0'] + SV['W'], th['z0'] + SV['L']]), 'difference')
    m = B(m, pocket([X_SV0, CARP['u0'], CARP['z0']], [X_SV0 + SV['H'], CARP['u0'] + 2*SV['W'] + 1, CARP['z0'] + SV['L']]), 'difference')   # 2 servos side by side
    # thumb CMC socket
    m = B(m, trimesh.creation.icosphere(subdivisions=4, radius=8.6).apply_translation(K.T0), 'difference')
    return m
def split_palm(m, xs=9.5):
    back = B(m, boxm([-60, -80, 400], [xs, 80, 620]), 'intersection'); cover = B(m, boxm([xs, -80, 400], [60, 80, 620]), 'intersection')
    return back, cover
# ------------------------------------------------------------------ thumb (local digit frame as fingers)
TW = (8.2, 6.4); TH = (7.4, 5.6)
def thumb_part(k):
    L = K.TLEN; Lt = sum(L); start = sum(L[:k]); Lk = L[k]; wf = lambda f: TW[0] + (TW[1] - TW[0])*f; hf = lambda f: TH[0] + (TH[1] - TH[0])*f
    W = lambda s: wf((start + s*Lk)/Lt); Hh = lambda s: hf((start + s*Lk)/Lt); rb = 0.9*min(W(0), Hh(0)); tip = (k == 2)
    def sdf(A, T, Bv):
        a1 = Lk - (K.TIP_R - 0.3) if tip else Lk
        body = ell_capsule(A, T, Bv, 0.0, a1, (lambda s: W(s*a1/Lk)), (lambda s: Hh(s*a1/Lk)))
        if tip: body = smin(body, np.sqrt((A - a1)**2 + T**2 + (Bv - 0.8)**2) - (K.TIP_R - 1.2), 3.0)
        if k == 0: body = smin(body, np.sqrt(((A - Lk*0.45)/1.0)**2 + (T/1.15)**2 + ((Bv - 1.0)/1.1)**2) - 8.0, 6.0)   # metacarpal belly
        return smin(body, np.sqrt(A**2 + T**2 + Bv**2) - rb, 2.5)
    wm = max(W(0), K.TIP_R, 9.5) + 2; m = mc(sdf, (-rb - 1.5, -wm, -wm), (Lk + 2, wm, wm), 0.35)
    if not tip:
        rbn = 0.9*min(W(1), Hh(1)); m = B(m, trimesh.creation.icosphere(subdivisions=4, radius=rbn + 0.4).apply_translation([Lk, 0, 0]), 'difference')
        m = B(m, cyl(0.95, [Lk, -14, 0], [Lk, 14, 0]), 'difference')
    m = B(m, cyl(1.05, [0, -14, 0], [0, 14, 0]), 'difference')
    if k >= 1:
        m = B(m, boxm([-rb - 2, -3.1, -20], [(Lk + 1) if k == 1 else 7.0, 3.1, 3.9]), 'difference')
        pa, pb = FBT['H'] if k == 2 else FBT['H']; 
        if k == 2: m = B(m, cyl(0.95, [pa, -12, pb], [pa, 12, pb]), 'difference')
    if k == 0:
        m = B(m, boxm([Lk - 9, -3.1, -20], [Lk + 3, 3.1, 3.9]), 'difference'); G = FBT['G']; m = B(m, cyl(0.95, [Lk + G[0], -12, G[1]], [Lk + G[0], 12, G[1]]), 'difference')
    return m, rb
def thumb_cap():
    L = K.TLEN; Lt = sum(L); start = L[0] + L[1]; Lk = L[2]; a1 = Lk - (K.TIP_R - 0.3)
    W = lambda s: TW[0] + (TW[1] - TW[0])*((start + s*a1)/Lt); Hh = lambda s: TH[0] + (TH[1] - TH[0])*((start + s*a1)/Lt)
    core = lambda A, T, Bv: smin(ell_capsule(A, T, Bv, 0.0, a1, W, Hh), np.sqrt((A - a1)**2 + T**2 + (Bv - 0.8)**2) - (K.TIP_R - 1.2), 3.0)
    return mc(lambda A, T, Bv: np.maximum.reduce([core(A, T, Bv) - 1.2, -core(A, T, Bv) - 0.02, (a1 - 7.5) - A]), (a1 - 9, -K.TIP_R - 2.5, -K.TIP_R - 2.5), (Lk + 2, K.TIP_R + 2.5, K.TIP_R + 3), 0.22)
# ------------------------------------------------------------------ build
if __name__ == "__main__":
    import time; t0 = time.time(); parts = {}; info = {}
    for nm in K.LEN:
        for k, pn in enumerate(('prox', 'mid', 'dist')):
            m, rb = phalanx(nm, k); parts[f"{nm}_{pn}"] = m; info[f"{nm}_{pn}_ball_r"] = round(rb, 2)
        parts[f"{nm}_cap"] = tip_cap(nm)
        parts[f"{nm}_bar_pip"] = bar(FB[nm]['pip']['lb'], -2.8, -0.8); parts[f"{nm}_bar_dip"] = bar(FB[nm]['dip']['lb'], 0.8, 2.8)
        parts[f"{nm}_carrier"] = carrier(nm, info[f"{nm}_prox_ball_r"]); print(nm, round(time.time() - t0), 's', flush=True)
    for k, pn in enumerate(('meta', 'prox', 'dist')): parts[f"thumb_{pn}"], _ = thumb_part(k)
    parts["thumb_cap"] = thumb_cap(); parts["thumb_bar"] = bar(FBT['lb'], -1.0, 1.0)
    pm = palm(); parts["palm_back"], parts["palm_cover"] = split_palm(pm); print('palm', round(time.time() - t0), 's', flush=True)
    dens = lambda n: 1.21 if n.endswith('cap') else (1.30 if n.startswith('palm') else 1.20)
    meta = {}
    for n, m in parts.items():
        m.export(f"{OUT}/{n}.stl"); meta[n] = dict(vol_cm3=round(m.volume/1e3, 3), g_solid=round(m.volume/1e3*dens(n), 2), watertight=bool(m.is_watertight), bbox=np.round(m.extents, 1).tolist())
    # proxies (not printed)
    for nm, us in SERVO_U.items(): boxm([X_SV0, us - SV['W']/2, Z_SV0], [X_SV0 + SV['H'], us + SV['W']/2, Z_SV0 + SV['L']]).export(f"{OUT}/proxies/servo_{nm}.stl")
    json.dump(dict(parts=meta, info=info, SERVO_U=SERVO_U, Z_SV0=Z_SV0, Z_SHAFT=Z_SHAFT, X_SV0=X_SV0), open(OUT + "/hand_parts.json", "w"), indent=1)
    bad = [n for n, v in meta.items() if not v['watertight']]; print(len(parts), 'parts; not watertight:', bad, round(time.time() - t0), 's')
