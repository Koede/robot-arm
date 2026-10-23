"""Swept keep-out of the other links, seen from one link's own frame.
K_g(P) = min over sampled arm poses q (every joint inside its HARD range) and over every group h != g of the signed distance from the point P (given in g's frame) to the solids of h at pose q.
A cable anchor / clip / in-link cable route in group g must have K_g >= its radius + margin, otherwise some other link sweeps into it at some pose.
The pose sample is: 14 single-joint limits (others at the park pose), 4 soft-limit corner poses, plus Latin-hypercube random poses over the hard ranges (default 130 in total ~ 150 poses).
Not a proof (sparse sampling in 7 joints); arm_wiring_verify.py re-checks the final routes with an independent 400-pose sample.
Usage as a library: poses(n, seed) -> list of frames dicts ; keepout(g, P, FR) ; keepout_grid(g, lo, hi, pitch, FR)."""
import os, sys, numpy as np
from scipy import ndimage
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_kin as ak, rom_limits as RL
J = ["S1", "S2", "S3", "E", "W1", "W2", "W3"]
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)

def pose_list(n=130, seed=0):
    rng = np.random.default_rng(seed); qs = []
    for j in J:
        for v in RL.HARD[j]:
            q = dict(PARK); q[j] = float(v); qs.append(q)
    for sgn in (0, 1):
        q = {j: float(RL.SOFT[j][sgn]) for j in J}; q["G"] = 50.0; qs.append(q)
        q = {j: float(RL.SOFT[j][1 - sgn]) if j in ("S2", "E", "W2") else float(RL.SOFT[j][sgn]) for j in J}; q["G"] = 50.0; qs.append(q)
    m = len(J); u = (np.argsort(rng.random((n, m)), axis=0) + rng.random((n, m))) / n            # Latin hypercube
    for row in u:
        q = {j: float(RL.HARD[j][0] + row[i] * (RL.HARD[j][1] - RL.HARD[j][0])) for i, j in enumerate(J)}; q["G"] = 50.0; qs.append(q)
    return qs

def frames_list(qs): return [ak.frames(q) for q in qs]

def keepout(g, P, FR, chunk=40000, others=None):
    """min signed distance (mm) from points P (in g's frame) to every OTHER group over the poses FR; also returns the index of the nearest group"""
    P = np.asarray(P, float); best = np.full(len(P), 1e9)
    others = [h for h in L.GROUPS if h != g] if others is None else others
    for F in FR:
        Pw = (np.c_[P, np.ones(len(P))] @ F[g].T)[:, :3]
        for h in others:
            Ph = L.to_local(F[h], Pw) if len(P) < 5 else (np.c_[Pw, np.ones(len(Pw))] @ np.linalg.inv(F[h]).T)[:, :3]
            v = L.sample_sdf(h, Ph); best = np.minimum(best, v)
    return best

def keepout_grid(g, lo, hi, pitch, FR):
    """K_g on a regular grid covering the box lo..hi (in g's frame) at the given pitch -> (K array, origin lo, pitch)"""
    ax = [np.arange(lo[i], hi[i] + 1e-9, pitch) for i in range(3)]; X, Y, Zg = np.meshgrid(*ax, indexing="ij"); P = np.c_[X.ravel(), Y.ravel(), Zg.ravel()]
    return keepout(g, P, FR).reshape(X.shape).astype(np.float32), np.array(lo, float), pitch

def legal_poses(FR, mm=3.0):
    """operating-envelope filter for the swept-clearance check: a pose is legal when no pair of groups that are NOT in designed contact at the park pose (pairs closer than mm there are excluded:
    adjacent links, bearings, stops) is closer than mm anywhere.  Poses where the arm hits the wing / boxes / its own shoulder stage are outside the envelope the firmware tiers and the
    keep-out tables allow.  Returns (list of frames, count)."""
    import trimesh
    gm = L.build_group_meshes(); Vs = {}
    for g in L.GROUPS:
        m = gm[g]; m = m if hasattr(m, "vertices") else trimesh.util.concatenate(list(m.values()) if isinstance(m, dict) else m); Vs[g] = m.vertices[::4]
    def pair_d(F, g, h):
        Pw = (np.c_[Vs[g], np.ones(len(Vs[g]))] @ F[g].T)[:, :3]; Ph = (np.c_[Pw, np.ones(len(Pw))] @ np.linalg.inv(F[h]).T)[:, :3]; return float(L.sample_sdf(h, Ph).min())
    F0 = ak.frames(dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)); pairs = []
    for g in L.GROUPS:
        for h in L.GROUPS:
            if g == h: continue
            if min(pair_d(F0, g, h), pair_d(F0, h, g)) >= mm: pairs.append((g, h))
    keep = []
    for F in FR:
        if all(pair_d(F, g, h) >= mm for g, h in pairs): keep.append(F)
    return keep, len(keep)


if __name__ == "__main__":
    import time
    FR = frames_list(pose_list()); print(len(FR), "poses")
    t = time.time(); P = np.array([[16.0, 66.8, -93.7], [0.0, 0.0, 130.0], [53.6, -17.6, 26.0]]); print(keepout("M1", P, FR), "%.1fs" % (time.time() - t))
