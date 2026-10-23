"""In-link cable corridors: shortest free path (A* on the signed-distance field of the link's own solids, in that link's frame) between the anchors chosen by the crossing search.
cost per voxel = 1 + 6 exp(-(sdf - r)/3);  voxels with sdf < r + CLR are impassable.  Path smoothed (moving average over ~6 mm) before measuring length and the minimum bend radius (circle through 3 points 8 mm apart).
Provides corridor(group, P, Q, od) -> dict(ok, length_mm, min_clear_mm, min_radius_mm, path) and the chain selection used by arm_wiring_verify.py.
The hub board of H1 (M3) is added to the M3 obstacle field as axis-aligned boxes.
Usage: python3 arm_route.py     (self-test on M3: hub J1 -> E anchor)"""
import os, sys, json, math, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L
from skimage.graph import route_through_array
from scipy.ndimage import uniform_filter1d
OUT = os.path.join(HERE, "..", "stl", "electronics")
CLR = 0.8

def box_sdf(P, lo, hi):
    c = (np.asarray(lo) + np.asarray(hi)) / 2; h = (np.asarray(hi) - np.asarray(lo)) / 2; q = np.abs(P - c) - h
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)

def hub_boxes():
    """hub board + parts as boxes in the M3 frame (board normal y); inputs on the -y face, outputs on the +y face (arm_hub.py)"""
    import arm_hub as H
    d = json.load(open(os.path.join(OUT, "arm_hub.json"))); yb, zb = d["y_board"], d["z_board"]; bx, bz, bt = H.BOARD; boxes = [((-bx / 2, yb - bt / 2, zb - bz / 2), (bx / 2, yb + bt / 2, zb + bz / 2))]
    for n, (px, pz) in d["placement_xz"].items():
        sx, sz, h = H.PARTS[n][:3]; f = d["face"][n]; y0, y1 = (yb + bt / 2, yb + bt / 2 + h) if f > 0 else (yb - bt / 2 - h, yb - bt / 2)
        boxes.append(((px - sx / 2, y0, pz - sz / 2), (px + sx / 2, y1, pz + sz / 2)))
    for hx, hz in H.HOLES: boxes.append(((hx - 3.5, yb - bt / 2 - 8.4, zb + hz - 3.5), (hx + 3.5, yb - bt / 2, zb + hz + 3.5)))      # mounting posts (printed on ua_med)
    return boxes, d

def hub_points(d):
    """cable start points (M3 frame) 3 mm outside each connector: J1 trunk-in leaves down (-z) from the lower edge on the -y side; J3/J4 leave +y"""
    import arm_hub as H
    yb, zb = d["y_board"], d["z_board"]
    bx, bz, bt = H.BOARD; pl = d["placement_xz"]; pts = {}
    pts["J1"] = np.array([pl["J1_trunk_in_12V"][0], yb - bt / 2 - 2.0, zb - bz / 2 - 3.0])
    for n, k in (("J3_elbow_XM", "J3"), ("J4_STS_out", "J4")): pts[k] = np.array([pl[n][0], yb + bt / 2 + H.PARTS[n][2] + 3.0, pl[n][1]])
    return pts

def field(group, extra_boxes=None, box=None, margin=2.0):
    """cropped SDF (world-of-group frame) -> (sdf array, origin lo, pitch)"""
    sdf, lo, p = L.sdf_of(group)
    if extra_boxes:
        ii = np.indices(sdf.shape).reshape(3, -1).T; P = lo + ii * p; v = np.full(len(P), 1e9)
        for a, b in extra_boxes: v = np.minimum(v, box_sdf(P, a, b))
        sdf = np.minimum(sdf, v.reshape(sdf.shape))
    return sdf, lo, p

def corridor(group, P, Q, od, extra_boxes=None, pad=35.0, fld=None, kmin=4.0):
    sdf, lo, p = fld if fld is not None else field(group, extra_boxes)
    r = od / 2.0; P = np.asarray(P, float); Q = np.asarray(Q, float)
    a = np.minimum(P, Q) - pad; b = np.maximum(P, Q) + pad
    i0 = np.maximum(((a - lo) / p).astype(int), 0); i1 = np.minimum(((b - lo) / p).astype(int) + 1, np.array(sdf.shape))
    sub = sdf[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]]
    cost = 1.0 + 6.0 * np.exp(-(sub - r) / 3.0); cost[sub < r + CLR] = 1e6
    s = tuple(np.clip(np.round((P - lo) / p).astype(int) - i0, 0, np.array(sub.shape) - 1)); t = tuple(np.clip(np.round((Q - lo) / p).astype(int) - i0, 0, np.array(sub.shape) - 1))
    if cost[s] >= 1e6 or cost[t] >= 1e6: return dict(ok=False, why="endpoint inside an obstacle (sdf %.2f / %.2f, need %.2f)" % (sub[s], sub[t], r + CLR))
    path, c = route_through_array(cost, s, t, fully_connected=True, geometric=True)
    if c >= 1e5: return dict(ok=False, why="no free path")
    pts = lo + (np.array(path) + i0) * p; k = max(int(round(6.0 / p)), 3)
    sm = np.vstack([uniform_filter1d(pts[:, d], k, mode="nearest") for d in range(3)]).T; sm[0], sm[-1] = pts[0], pts[-1]
    seg = np.linalg.norm(np.diff(sm, axis=0), axis=1); length = float(seg.sum())
    # clearance of the smoothed path
    clr = float(L.sample_sdf(group, sm).min()) if extra_boxes is None else float(min(L.sample_sdf(group, sm).min(), min(box_sdf(sm, a_, b_).min() for a_, b_ in extra_boxes)))
    # bend radius from 3 points ~8 mm apart
    cum = np.r_[0, np.cumsum(seg)]; step = 8.0; rad = 1e9
    for u in np.arange(step, cum[-1] - step, 2.0):
        i, j, k2 = (np.searchsorted(cum, [u - step, u, u + step]).clip(0, len(sm) - 1)); A, B, C = sm[i], sm[j], sm[k2]
        ab, bc, ca = np.linalg.norm(B - A), np.linalg.norm(C - B), np.linalg.norm(A - C); cr = np.linalg.norm(np.cross(B - A, C - A))
        if cr > 1e-6: rad = min(rad, ab * bc * ca / (2 * cr))
    ok = (clr - r) >= CLR - 0.05 and rad >= kmin * od
    return dict(ok=bool(ok), length_mm=round(length, 1), min_clear_mm=round(clr - r, 2), min_radius_mm=round(float(min(rad, 999)), 1), path=sm.round(1).tolist(), od=od)


# ----------------------------------------------------------------------------------------------- cutting windows
def obox_sdf(P, c, R, half):
    """signed distance of points P to an oriented box (centre c, columns of R = box axes, half sizes)"""
    q = (np.asarray(P) - c) @ R; d = np.abs(q) - np.asarray(half)
    return np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(d.max(axis=1), 0)

def carve(sdf, lo, p, windows):
    """subtract oriented boxes from a solid SDF: sdf := max(sdf, -box_sdf) (inside the box the distance to the box boundary is the free clearance)"""
    if not windows: return sdf
    out = sdf.copy(); ii = np.indices(sdf.shape).reshape(3, -1).T; P = lo + ii * p
    for w in windows:
        c = np.array(w["center"], float); R = np.array(w["R"], float); half = np.array(w["half"], float)
        # only voxels near the box
        near = np.linalg.norm(P - c, axis=1) < np.linalg.norm(half) + 3.0
        idx = np.where(near)[0]; v = obox_sdf(P[idx], c, R, half)
        flat = out.reshape(-1); flat[idx] = np.maximum(flat[idx], -v)
    return out

def corridor_cut(group, P, Q, od, fld, wall_penalty=25.0, pad=40.0):
    """like corridor() but walls can be crossed at a penalty: returns the path and the wall crossings (window candidates): list of dict(center, n, thickness)"""
    sdf, lo, p = fld; r = od / 2.0; P = np.asarray(P, float); Q = np.asarray(Q, float)
    a = np.minimum(P, Q) - pad; b = np.maximum(P, Q) + pad
    i0 = np.maximum(((a - lo) / p).astype(int), 0); i1 = np.minimum(((b - lo) / p).astype(int) + 1, np.array(sdf.shape)); sub = sdf[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]]
    cost = 1.0 + 6.0 * np.exp(-np.clip(sub - r, 0, 50) / 3.0); blocked = sub < r + CLR; cost[blocked] = wall_penalty
    s = tuple(np.clip(np.round((P - lo) / p).astype(int) - i0, 0, np.array(sub.shape) - 1)); t = tuple(np.clip(np.round((Q - lo) / p).astype(int) - i0, 0, np.array(sub.shape) - 1))
    path, c = route_through_array(cost, s, t, fully_connected=True, geometric=True); path = np.array(path); inb = blocked[path[:, 0], path[:, 1], path[:, 2]]
    pts = lo + (path + i0) * p; cross = []; k = 0
    while k < len(pts):
        if inb[k]:
            j = k
            while j + 1 < len(pts) and inb[j + 1]: j += 1
            cross.append(dict(center=pts[(k + j) // 2].round(1).tolist(), thickness_mm=round(float(np.linalg.norm(pts[j] - pts[k])) + p, 1), a=pts[k].round(1).tolist(), b=pts[j].round(1).tolist())); k = j + 1
        else: k += 1
    return dict(path=pts.round(1).tolist(), crossings=cross, cost=float(c))

# ----------------------------------------------------------------------------------------------- windows from the chosen chain
def window_from_crossing(cr, width=10.0, height=12.0, extra_depth=6.0):
    a = np.array(cr["a"]); b = np.array(cr["b"]); n = b - a; n[2] = 0
    n = n / np.linalg.norm(n) if np.linalg.norm(n) > 1e-6 else np.array([1.0, 0, 0])
    u = np.cross([0, 0, 1.0], n); u /= np.linalg.norm(u); depth = cr["thickness_mm"] + extra_depth
    return dict(center=cr["center"], R=np.stack([n, u, [0, 0, 1.0]], 1).tolist(), half=[depth / 2, width / 2, height / 2], n=n.round(3).tolist(), u=u.round(3).tolist(), width=width, height=height, depth=round(depth, 1))

def windows_from_chain(chain):
    wins = []
    for pathd in chain["links"]["M3"]:
        for cr in pathd["crossings"]:
            if float(L.sample_sdf("M3", np.array([cr["center"]]))[0]) < 1.0: wins.append(window_from_crossing(cr))
    return wins

if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "windows":
    ch = json.load(open(os.path.join(OUT, "arm_chain.json"))); w = windows_from_chain(ch); json.dump(w, open(os.path.join(OUT, "windows.json"), "w"), indent=1); print(len(w), "windows", [x["center"] for x in w])

# ----------------------------------------------------------------------------------------------- swept-keep-out aware corridors (rev 2026-10-17 b)
def field_K(g, lo, hi, FR, pitch=2.0, boxes=None):
    """combined obstacle field in g's frame on a regular grid: min( own solids, swept keep-out of every other group over the pose sample FR [, extra boxes] )"""
    import arm_keepout as K
    ax = [np.arange(lo[i], hi[i] + 1e-9, pitch) for i in range(3)]; X, Y, Zg = np.meshgrid(*ax, indexing="ij"); P = np.c_[X.ravel(), Y.ravel(), Zg.ravel()]
    v = np.minimum(L.sample_sdf(g, P), K.keepout(g, P, FR))
    if boxes:
        for a, b in boxes: v = np.minimum(v, box_sdf(P, a, b))
    return v.reshape(X.shape).astype(np.float32), np.array(lo, float), pitch

def _resample(pts, step=1.0):
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1); s = np.r_[0, np.cumsum(seg)]; n = max(int(s[-1] / step), 2); u = np.linspace(0, s[-1], n)
    return np.vstack([np.interp(u, s, pts[:, d]) for d in range(3)]).T

def _min_radius(pts, step=8.0):
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1); cum = np.r_[0, np.cumsum(seg)]; rad = 1e9
    for u in np.arange(step, cum[-1] - step, 1.5):
        i, j, k2 = np.searchsorted(cum, [u - step, u, u + step]).clip(0, len(pts) - 1); A, B, C = pts[i], pts[j], pts[k2]
        ab, bc, ca = np.linalg.norm(B - A), np.linalg.norm(C - B), np.linalg.norm(A - C); cr = np.linalg.norm(np.cross(B - A, C - A))
        if cr > 1e-6: rad = min(rad, ab * bc * ca / (2 * cr))
    return rad

def corridor_k(fld, P, Q, od, kmin=3.0, clr_block=1.5, clr_final=1.0, pad=30.0):
    """A* on the combined field, then smoothing with a growing window until the bend radius >= kmin x od while the clearance stays >= clr_final"""
    from scipy.ndimage import map_coordinates
    v, lo, p = fld; r = od / 2.0; P = np.asarray(P, float); Q = np.asarray(Q, float)
    a = np.minimum(P, Q) - pad; b = np.maximum(P, Q) + pad
    i0 = np.maximum(((a - lo) / p).astype(int), 0); i1 = np.minimum(((b - lo) / p).astype(int) + 1, np.array(v.shape)); sub = v[i0[0]:i1[0], i0[1]:i1[1], i0[2]:i1[2]]
    cost = 1.0 + 6.0 * np.exp(-np.clip(sub - r, 0, 50) / 3.0); cost[sub < r + clr_block] = 1e6
    s = tuple(np.clip(np.round((P - lo) / p).astype(int) - i0, 0, np.array(sub.shape) - 1)); t = tuple(np.clip(np.round((Q - lo) / p).astype(int) - i0, 0, np.array(sub.shape) - 1))
    for end in (s, t):                                                              # endpoints sit inside the clip halo: free them locally (3 voxels)
        sl = tuple(slice(max(e - 2, 0), e + 3) for e in end); cost[sl] = np.minimum(cost[sl], 20.0)
    path, c = route_through_array(cost, s, t, fully_connected=True, geometric=True)
    if c >= 1e5: return dict(ok=False, why="no free path")
    pts = lo + (np.array(path) + i0) * p; pts[0], pts[-1] = P, Q; base = _resample(pts, 1.0); best = None
    def clear(x): return float(map_coordinates(v, ((x - lo) / p).T, order=1, mode="nearest").min()) - r
    for w in (6, 10, 14, 20, 28, 38, 50):
        k = max(int(w), 3); sm = np.vstack([uniform_filter1d(base[:, d], k, mode="nearest") for d in range(3)]).T; sm[0], sm[-1] = P, Q
        cl = clear(sm[3:-3] if len(sm) > 8 else sm); rad = _min_radius(sm); L_ = float(np.linalg.norm(np.diff(sm, axis=0), axis=1).sum())
        cand = dict(window=w, clear=cl, rad=rad, length=L_, pts=sm)
        if cl >= clr_final and (best is None or rad > best["rad"]): best = cand
        if best is not None and best["rad"] >= kmin * od: break
    if best is None: return dict(ok=False, why="smoothing violates clearance")
    return dict(ok=bool(best["rad"] >= kmin * od * 0.95 and best["clear"] >= clr_final), length_mm=round(best["length"], 1), min_clear_mm=round(best["clear"], 2), min_radius_mm=round(min(best["rad"], 999), 1), window=best["window"], od=od, path=best["pts"].round(1).tolist(), cost=float(c))
