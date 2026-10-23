"""Shared tools for the in-arm wiring analysis (right arm, robot frame, mm).

* group_sdf(g)       signed distance field (mm, negative inside) of every solid that rides on kinematic group g (arm_kin groups incl. balancer, cowl, stops),
                     sampled on a voxel grid in the group's OWN frame, cached to stl/electronics/_sdf/<g>.npz.  Resolution PITCH mm (error <= ~PITCH/2).
* clearance(P, q)    min over all groups of (sdf - 0) for robot-frame points P at pose q -> signed distance to the nearest solid surface (negative = inside).
* bezier_len(...)    cubic Bezier cable model between two anchors with end tangents and a FIXED cable length L (handle length solved) -> polyline + min bend radius.
* cable_sweep(...)   cable between an anchor in group gi (local frame) and one in group gj over a list of poses: clearance, min bend radius, required length.
The cable shape model is a kinematic envelope (smooth curve of fixed length), NOT a mechanics simulation; it is used to decide where loops go and how much
slack is needed.  Bench test T-W1 (to be written) confirms the loops on the printed arm.
"""
import os, sys, json, numpy as np, trimesh
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak
SDF_DIR = os.path.join(HERE, "..", "stl", "electronics", "_sdf"); os.makedirs(SDF_DIR, exist_ok=True)
PITCH = 1.0
GROUPS = ["F", "M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"]

def _all_groups(g_pct=50.0):
    G = ak.groups(True, g_pct)
    if ak.has_balancer(): ak.install_balancer(G)
    return G

def _balancer_local():
    """balancer bodies expressed in the frame of the arm_kin group that carries them (M1 / M2); swivel parts (turn with S1 and the tube axis) are placed at S2 = 0 in M1."""
    out = {}
    if not ak.has_balancer(): return out
    import balancer_calc as bc
    for nm, (m, grp) in ak.balancer_parts().items():
        tgt = "M1" if grp in ("M1", "SWIVEL") else "M2"
        out.setdefault(tgt, []).append(m)
    return out

def build_group_meshes():
    G = _all_groups(); bal = _balancer_local(); res = {}
    for g in GROUPS:
        ms = [m for m in G.get(g, {}).values()]
        ms += bal.get(g, [])
        res[g] = ms
    # fixed-side shell parts that arm_kin does not list in F: chassis plate, pauldron core + wrap-arms, scapular wing (the wearer is not included)
    R = os.path.join(HERE, "..", "stl", "shoulder")
    for p in ("chassis_plate_R.stl", "pauldron/pauldron_core_R.stl", "pauldron/pauldron_arm_lat_R.stl", "pauldron/pauldron_arm_med_R.stl", "scapular/wing_R.stl"):
        f = os.path.join(R, p)
        if os.path.exists(f): res["F"].append(trimesh.load(f))
    return res


def _raster(m, occ, lo, shape):
    """z-ray scanline fill of a watertight mesh into occ (voxel centres at lo + i*PITCH)"""
    b0, b1 = m.bounds; ix = np.arange(max(int(np.floor((b0[0] - lo[0]) / PITCH)), 0), min(int(np.ceil((b1[0] - lo[0]) / PITCH)) + 1, shape[0]))
    iy = np.arange(max(int(np.floor((b0[1] - lo[1]) / PITCH)), 0), min(int(np.ceil((b1[1] - lo[1]) / PITCH)) + 1, shape[1]))
    XX, YY = np.meshgrid(ix, iy, indexing="ij"); X = lo[0] + XX.ravel() * PITCH; Y = lo[1] + YY.ravel() * PITCH
    org = np.c_[X + 1e-4, Y + 1e-4, np.full(X.size, b0[2] - 2.0)]; dirs = np.tile([0, 0, 1.0], (X.size, 1))
    for c0 in range(0, len(org), 20000):
        o = org[c0:c0 + 20000]; d = dirs[c0:c0 + 20000]
        loc, ir, _ = m.ray.intersects_location(o, d, multiple_hits=True)
        if len(ir) == 0: continue
        order = np.lexsort((loc[:, 2], ir)); ir = ir[order]; z = loc[order, 2]
        for r in np.unique(ir):
            zz = z[ir == r]
            if len(zz) % 2: zz = zz[:-1]
            gx = XX.ravel()[c0 + r]; gy = YY.ravel()[c0 + r]
            for k in range(0, len(zz), 2):
                k0 = int(np.ceil((zz[k] - lo[2]) / PITCH)); k1 = int(np.floor((zz[k + 1] - lo[2]) / PITCH))
                if k1 >= k0: occ[gx, gy, max(k0, 0):min(k1 + 1, shape[2])] = True

def group_sdf(g, rebuild=False):
    f = os.path.join(SDF_DIR, g + ".npz")
    if os.path.exists(f) and not rebuild:
        d = np.load(f); return d["sdf"], d["origin"], float(d["pitch"])
    ms = build_group_meshes()[g]
    if not ms: raise ValueError("empty group " + g)
    allv = np.vstack([m.vertices for m in ms]); lo = allv.min(0) - 12.0; hi = allv.max(0) + 12.0
    shape = np.ceil((hi - lo) / PITCH).astype(int) + 1; occ = np.zeros(shape, bool)
    for m in ms: _raster(m, occ, lo, shape)
    out = ndimage.distance_transform_edt(~occ) * PITCH; inn = ndimage.distance_transform_edt(occ) * PITCH
    sdf = (out - inn).astype(np.float32)           # + outside, - inside (voxel-centre distances: surface at ~0 +- PITCH/2)
    np.savez_compressed(f, sdf=sdf, origin=lo, pitch=PITCH); return sdf, lo, PITCH

_SDF = {}
def sdf_of(g):
    if g not in _SDF: _SDF[g] = group_sdf(g)
    return _SDF[g]

def sample_sdf(g, Ploc):
    sdf, lo, p = sdf_of(g); idx = ((np.asarray(Ploc) - lo) / p).T
    return ndimage.map_coordinates(sdf, idx, order=1, mode="nearest", cval=50.0)

def to_local(T, P):
    Ti = np.linalg.inv(T); P = np.asarray(P, float); return (np.c_[P, np.ones(len(P))] @ Ti.T)[:, :3]

def clearance(P, q, groups=GROUPS, ignore=()):
    """signed distance (mm) from robot-frame points P (N,3) to the nearest solid at pose q; per-group values returned as dict"""
    F = ak.frames(q); per = {}
    for g in groups:
        if g in ignore: continue
        try: per[g] = sample_sdf(g, to_local(F[g], P))
        except ValueError: pass
    stack = np.vstack(list(per.values())); return stack.min(0), per

# ---------------------------------------------------------------------------------------------- cable curve model
def _bez(P0, P1, P2, P3, n):
    t = np.linspace(0, 1, n)[:, None]
    return (1 - t) ** 3 * P0 + 3 * (1 - t) ** 2 * t * P1 + 3 * (1 - t) * t ** 2 * P2 + t ** 3 * P3

def _len(pts): return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())

def min_radius(pts):
    d1 = np.gradient(pts, axis=0); d2 = np.gradient(d1, axis=0)
    cr = np.linalg.norm(np.cross(d1, d2), axis=1); sp = np.linalg.norm(d1, axis=1)
    k = cr / np.maximum(sp ** 3, 1e-9); return float(1.0 / max(k[2:-2].max(), 1e-9))

def bezier_len(A, tA, B, tB, L, n=120):
    """cubic Bezier from A (leaving along unit tA) to B (arriving along -tB, i.e. tB points OUT of B back into the cable run) with arclength L.
    Returns (pts, ok, chord). ok False when L is shorter than the shortest such curve."""
    A, B = np.asarray(A, float), np.asarray(B, float); tA = np.asarray(tA, float) / np.linalg.norm(tA); tB = np.asarray(tB, float) / np.linalg.norm(tB)
    chord = float(np.linalg.norm(B - A)); lo, hi = 0.0, max(L, 1.0) * 2
    f = lambda h: _len(_bez(A, A + tA * h, B + tB * h, B, n))
    if f(lo) > L: return _bez(A, A, B, B, n), False, chord
    for _ in range(60):
        mid = (lo + hi) / 2
        if f(mid) < L: lo = mid
        else: hi = mid
    h = (lo + hi) / 2; return _bez(A, A + tA * h, B + tB * h, B, n), True, chord

def cable_sweep(gA, A, tA, gB, B, tB, L, poses, r, bend_min, ignore=()):
    """A, tA given in the local frame of group gA; B, tB in gB.  poses: list of q dicts.  Returns per-pose records and a summary."""
    recs = []
    for q in poses:
        F = ak.frames(q); Pa = (F[gA] @ np.r_[A, 1])[:3]; Pb = (F[gB] @ np.r_[B, 1])[:3]
        ta = F[gA][:3, :3] @ np.asarray(tA, float); tb = F[gB][:3, :3] @ np.asarray(tB, float)
        pts, ok, chord = bezier_len(Pa, ta, Pb, tb, L)
        c, per = clearance(pts, q, ignore=ignore)
        # ends sit in grommets: exclude the first and last 6 mm of arclength from the clearance test
        s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]; mask = (s > 6.0) & (s < s[-1] - 6.0)
        recs.append(dict(q={k: q[k] for k in ("S1", "S2", "S3", "E", "W1", "W2", "W3")}, ok=bool(ok), chord=round(chord, 1), minrad=round(min_radius(pts), 1),
                         clr=round(float(c[mask].min()) - r, 2), where=max(per, key=lambda g: -per[g][mask].min()) if len(per) else ""))
    return recs

def poses_grid(joint, rng, step, base=None, extra=None):
    base = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50) if base is None else dict(base)
    out = []
    for v in np.arange(rng[0], rng[1] + 1e-9, step):
        q = dict(base); q[joint] = float(v); out.append(q)
    return out

if __name__ == "__main__":
    import time
    for g in GROUPS:
        t = time.time(); sdf, lo, p = group_sdf(g, rebuild=("--rebuild" in sys.argv)); print(g, sdf.shape, "lo", lo.round(1), "%.1fs" % (time.time() - t))

# ---------------------------------------------------------------------------------------------- discrete elastic rod relaxation (cable between two clamped anchors)
from scipy.optimize import minimize
_GRAD = {}
def _sdf_and_grad(g):
    if g not in _GRAD:
        sdf, lo, p = sdf_of(g); gx, gy, gz = np.gradient(sdf, p); _GRAD[g] = (gx, gy, gz)
    return _GRAD[g]

def sdf_world(P, F, groups=GROUPS):
    """min signed distance over groups and its gradient (world frame) for points P (N,3); F = ak.frames(q)"""
    best = np.full(len(P), 1e9); gbest = np.zeros_like(P)
    for g in groups:
        try: sdf, lo, p = sdf_of(g)
        except ValueError: continue
        Ti = np.linalg.inv(F[g]); Pl = (np.c_[P, np.ones(len(P))] @ Ti.T)[:, :3]; idx = ((Pl - lo) / p).T
        v = ndimage.map_coordinates(sdf, idx, order=1, mode="nearest", cval=50.0)
        gx, gy, gz = _sdf_and_grad(g); gl = np.stack([ndimage.map_coordinates(a, idx, order=1, mode="nearest") for a in (gx, gy, gz)], 1)
        gw = gl @ Ti[:3, :3]                                   # d/dx_world = R * d/dx_local  (gl row-vector @ R^-1 ... for rotation: gw = R gl  -> rows: gl @ R^T)
        gw = gl @ F[g][:3, :3].T
        m = v < best; best = np.where(m, v, best); gbest = np.where(m[:, None], gw, gbest)
    return best, gbest

def rod_relax(Pa, ta, Pb, tb, Lc, F, r, cmin, x0=None, N=26, kb=1.0, ks=60.0, ko=40.0, iters=250):
    """relax a cable of length Lc, clamped at Pa (leaving along ta) and Pb (leaving along tb, pointing back into the run), radius r, wanted clearance cmin.
    Returns nodes (N+1,3), info dict (min clearance to solids minus r, min bend radius, max stretch error)."""
    ds = Lc / N; ta = np.asarray(ta, float) / np.linalg.norm(ta); tb = np.asarray(tb, float) / np.linalg.norm(tb)
    if x0 is None:
        x0, _, _ = bezier_len(Pa, ta, Pb, tb, Lc, n=N + 1)
    x0 = np.array(x0, float); x0[0] = Pa; x0[1] = Pa + ta * ds; x0[-1] = Pb; x0[-2] = Pb + tb * ds
    nfree = N - 3; idx_free = np.arange(2, N - 1)
    def full(z):
        x = x0.copy(); x[idx_free] = z.reshape(-1, 3); return x
    def fun(z):
        x = full(z); e = 0.0; g = np.zeros_like(x)
        seg = np.diff(x, axis=0); ln = np.linalg.norm(seg, axis=1); u = seg / np.maximum(ln, 1e-9)[:, None]; err = ln - ds
        e += ks * np.sum(err ** 2) / ds; f = 2 * ks * err[:, None] * u / ds; g[:-1] -= f; g[1:] += f
        b = x[:-2] - 2 * x[1:-1] + x[2:]; e += kb * np.sum(b ** 2) / ds ** 2 * ds; gb = 2 * kb * b / ds
        g[:-2] += gb; g[1:-1] -= 2 * gb; g[2:] += gb
        mid = 0.5 * (x[:-1] + x[1:]); pts = np.vstack([x, mid]); v, gv = sdf_world(pts, F)
        viol = np.maximum(0.0, r + cmin - v); e += ko * np.sum(viol ** 2)
        gp = -2 * ko * viol[:, None] * gv; g += gp[:len(x)]; g[:-1] += 0.5 * gp[len(x):]; g[1:] += 0.5 * gp[len(x):]
        return e, g[idx_free].ravel()
    res = minimize(fun, x0[idx_free].ravel(), jac=True, method="L-BFGS-B", options=dict(maxiter=iters))
    x = full(res.x); seg = np.diff(x, axis=0); ln = np.linalg.norm(seg, axis=1)
    v, _ = sdf_world(np.vstack([x, 0.5 * (x[:-1] + x[1:])]), F); s = np.r_[0, np.cumsum(ln)]
    m = np.r_[(s > 6.0) & (s < s[-1] - 6.0), ((s[:-1] + s[1:]) / 2 > 6.0) & ((s[:-1] + s[1:]) / 2 < s[-1] - 6.0)]
    if not m.any(): m[:] = True
    return x, dict(clr=float(v[m].min()) - r, minrad=min_radius(np.vstack([x])), stretch=float(np.abs(ln - ds).max() / ds), energy=float(res.fun))
