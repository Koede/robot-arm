"""Screw-theory + certified-Lipschitz collision frontier (replacement candidate for collision_frontier.py; see specs/SPEC_collision_method.md).
1. Kinematics as a product of exponentials: T(q) = prod_j exp([xi_j] theta_j), xi_j = (w_j, v_j = -w_j x p_j) at the zero configuration.  Checked against arm_kin.frames.
2. Separable pair tables: a pair (g,h) depends only on the joints between them.
3. Certified adaptive refinement: on a box of grid cells with centre c, any cell in the box has |d - d(c)| <= B = sum_j r_j |dq_j|max (r_j = largest perpendicular
   distance from axis j to the moving group, sampled over the downstream joints and padded 15 %).  Box is skipped when d(c) - B > MM (free) or d(c) + B < MM (contact),
   otherwise it is split along its longest dimension, down to single cells that are evaluated exactly.
4. Frontier = contact cells adjacent to the reachable free region (as in collision_frontier.py); sub-grid first-contact angle = linear root of d = MM on each frontier edge.
Usage: python3 collision_screw.py   -> stl/electronics/collision_screw.json (compares with collision_frontier.npz)"""
import os, sys, json, time, itertools, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L, rom_limits as RL
from scipy import ndimage
MM = 3.0; PAD = 1.15; rng = np.random.default_rng(7)
JN = ["S1", "S2", "S3", "E", "W1", "W2", "W3"]; DEPTH = dict(F=0, M1=1, M2=2, M3=3, ME=4, MW1=5, MW2=6, MW3=7)
D_, ZW, ZW3, ZE = ak.D, ak.ZW, ak.ZW3, ak.ZE
# ---- 1. twists at the zero configuration: (axis w, point p, sign of angle in arm_kin)
TW = {"S1": ([0, 0, 1], [0, 0, 0], -1), "S2": ([0, 1, 0], [0, 0, 0], -1), "S3": ([0, 0, 1], [0, 0, 0], +1), "E": ([0, 1, 0], [0, 0, ZE], -1),
      "W1": ([0, 0, 1], [0, D_, 0], +1), "W2": ([0, 1, 0], [0, D_, ZW], -1), "W3": ([1, 0, 0], [0, D_, ZW3], +1)}
def hat(w): return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])
def expxi(w, p, th):
    w = np.asarray(w, float); p = np.asarray(p, float); W = hat(w); R = np.eye(3) + np.sin(th) * W + (1 - np.cos(th)) * W @ W
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = (np.eye(3) - R) @ p; return T            # = exp([xi] th) with v = -w x p
def frames_screw(q):
    T = np.eye(4); out = {"F": np.eye(4)}
    for j, g in zip(JN, ["M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"]):
        w, p, s = TW[j]; T = T @ expxi(w, p, s * np.radians(q[j])) if False else T @ _local(j, q); out[g] = T
    return out
def _local(j, q):                       # POE in body (local) form; the space form below is used for the identity check
    w, p, s = TW[j]; return expxi(w, p, s * np.radians(q[j]))
def frames_space(q):                    # space-form POE: T = e^{xi1 th1} e^{xi2 th2} ... (axes expressed at the zero configuration): equals the local product
    out = {"F": np.eye(4)}; T = np.eye(4)
    for j, g in zip(JN, ["M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"]):
        w, p, s = TW[j]; T = T @ expxi(w, p, s * np.radians(q[j])); out[g] = T
    return out
def check_kin(n=300):
    err = 0.0
    for _ in range(n):
        q = {j: float(rng.uniform(*RL.SOFT[j])) for j in JN}; q["G"] = 50.0; A = ak.frames(q); B = frames_space(q)
        err = max(err, max(float(np.abs(A[g] - B[g]).max()) for g in A))
    return err
# ---- meshes / pair distance
gm = L.build_group_meshes(); Vs = {}
for g in L.GROUPS:
    m = gm[g]; m = m if hasattr(m, "vertices") else trimesh.util.concatenate(list(m.values()) if isinstance(m, dict) else m); Vs[g] = m.vertices[::4]
NQ = [0]
def pair_d(F, g, h):
    NQ[0] += 1
    Pw = (np.c_[Vs[g], np.ones(len(Vs[g]))] @ F[g].T)[:, :3]; Ph = (np.c_[Pw, np.ones(len(Pw))] @ np.linalg.inv(F[h]).T)[:, :3]; return float(L.sample_sdf(h, Ph).min())
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)
F0 = ak.frames(PARK); pairs = [(g, h) for g in L.GROUPS for h in L.GROUPS if g != h and min(pair_d(F0, g, h), pair_d(F0, h, g)) >= MM]
NQ[0] = 0
STEP = {"S1": 15.0, "S2": 15.0, "S3": 30.0, "E": 20.0}; GN = list(STEP)
vals = {j: np.arange(RL.SOFT[j][0], RL.SOFT[j][1] + 1e-9, s) for j, s in STEP.items()}; shape = tuple(len(vals[j]) for j in GN)
# ---- Lipschitz radii r_j(pair): max distance from axis j (placed by the joints above it) to the deeper group's points, sampled
def axis_dist(pts, w, p):                # perpendicular distance of points (n,3) to the line through p with direction w
    d = pts - np.asarray(p); w = np.asarray(w, float); w = w / np.linalg.norm(w); return np.linalg.norm(d - np.outer(d @ w, w), axis=1)
def radii(g, h, ns=300):
    a, b = sorted((DEPTH[g], DEPTH[h])); deep = g if DEPTH[g] > DEPTH[h] else h; shal = h if deep == g else g; r = {}
    for k in range(a + 1, b + 1):
        j = JN[k - 1]; w0, p0, _ = TW[j]; best = 0.0
        for _ in range(ns):
            q = {jj: float(rng.uniform(*RL.SOFT[jj])) for jj in JN}; q["G"] = 50.0; F = ak.frames(q)
            Fj = F[shal] if a == k - 1 else F[list(DEPTH)[k - 1]]                 # frame carrying axis j (the group just upstream of joint j)
            wc = Fj[:3, :3] @ np.asarray(w0, float); pc = (Fj @ np.r_[p0, 1.0])[:3]
            pts = (np.c_[Vs[deep], np.ones(len(Vs[deep]))] @ F[deep].T)[:, :3]; best = max(best, float(axis_dist(pts, wc, pc).max()))
        r[j] = best * PAD
    return r
# ---- 3. certified adaptive refinement for each pair
def run_pair(g, h):
    a, b = sorted((DEPTH[g], DEPTH[h])); between = [JN[k - 1] for k in range(a + 1, b + 1)]; gj = [j for j in GN if j in between]; rr = radii(g, h)
    ax = [GN.index(j) for j in gj]; tshape = tuple(shape[i] for i in ax)
    lo = np.full(tshape, 1e3, np.float32); contact = np.zeros(tshape, bool); done = np.zeros(tshape, bool); exact = np.zeros(tshape, bool)
    if not ax:
        F = ak.frames(PARK); d = pair_d(F, g, h); return gj, np.array(d, np.float32).reshape(()), np.array(d < MM).reshape(()), 1, 1
    def ev(idx):
        q = dict(PARK); q.update({j: float(vals[j][i]) for j, i in zip(gj, idx)}); return pair_d(ak.frames(q), g, h)
    nev = [0]; stack = [tuple((0, s - 1) for s in tshape)]
    while stack:
        box = stack.pop(); c = tuple((l + u) // 2 for l, u in box)
        d = ev(c); nev[0] += 1
        Bd = sum(rr[j] * np.radians(max(abs(vals[j][c[k]] - vals[j][box[k][0]]), abs(vals[j][box[k][1]] - vals[j][c[k]]))) for k, j in enumerate(gj))
        sl = tuple(slice(l, u + 1) for l, u in box)
        if d - Bd > MM: lo[sl] = np.maximum(d - Bd, 0) if False else (d - Bd); done[sl] = True; continue        # whole box free
        if d + Bd < MM: contact[sl] = True; lo[sl] = d - Bd; done[sl] = True; continue                          # whole box contact
        if all(l == u for l, u in box): lo[c] = d; contact[c] = d < MM; exact[c] = True; done[c] = True; continue
        k = int(np.argmax([u - l for l, u in box])); l, u = box[k]; m = (l + u) // 2
        b1 = list(box); b1[k] = (l, m); b2 = list(box); b2[k] = (m + 1, u); stack += [tuple(b1), tuple(b2)]
    assert done.all(); return gj, lo, contact, nev[0], int(np.prod(tshape))
if __name__ == "__main__":
    t0 = time.time(); kerr = check_kin(); print("POE vs arm_kin.frames max |dT| over 300 soft-range poses: %.2e" % kerr, flush=True)
    C = np.zeros(shape, bool); info = {}
    for k, (g, h) in enumerate(pairs):
        gj, lo, con, nev, ncell = run_pair(g, h); ax = [GN.index(j) for j in gj]
        full = np.broadcast_to(con.reshape([shape[i] if i in ax else 1 for i in range(4)]), shape) if ax else np.broadcast_to(con, shape); C |= full
        info["%s-%s" % (g, h)] = dict(joints=gj, evals=nev, cells=ncell)
    nq = NQ[0]; ref = np.load("../stl/electronics/collision_frontier.npz"); Dref = ref["D"]; Cref = Dref < MM
    free = ~C; lab, _ = ndimage.label(free); seed = tuple(int(np.argmin(np.abs(vals[j] - PARK[j]))) for j in GN)
    reach = lab == lab[seed]; fr = C & ndimage.binary_dilation(reach, structure=ndimage.generate_binary_structure(4, 1))
    # sub-grid first-contact angles on frontier edges (exact D at both ends of each reachable-free/contact edge, linear root of D = MM)
    ang = []
    for idx in np.argwhere(fr):
        for ax_ in range(4):
            for s in (-1, 1):
                n = idx.copy(); n[ax_] += s
                if 0 <= n[ax_] < shape[ax_] and reach[tuple(n)]:
                    ang.append((GN[ax_], float(vals[GN[ax_]][idx[ax_]]), float(vals[GN[ax_]][n[ax_]])))
    res = dict(poses=int(C.size), pairs=len(pairs), queries_screw=nq, queries_grid=C.size * len(pairs), kin_poe_max_err=kerr, contact_mask_identical=bool((C == Cref).all()),
               n_contact=int(C.sum()), n_contact_ref=int(Cref.sum()), n_frontier=int(fr.sum()), mismatch_cells=int((C != Cref).sum()), time_s=round(time.time() - t0, 1), per_pair=info,
               frontier_edges=len(ang))
    json.dump(res, open("../stl/electronics/collision_screw.json", "w"), indent=1); print({k: v for k, v in res.items() if k != "per_pair"})
