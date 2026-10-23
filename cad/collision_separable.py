"""Separable collision tables: exact reduction of collision_frontier.py. A group pair (g,h) in a serial chain depends only on the joints BETWEEN them (depth(g)+1..depth(h)); every other joint cancels.
Each pair is tabulated over those joints only (grid joints S1,S2,S3,E; wrist neutral as in the reference), then broadcast to the full grid. Must reproduce collision_frontier.npz exactly.
Usage: python3 collision_separable.py [step_scale]   -> stl/electronics/collision_separable.json"""
import os, sys, json, time, itertools, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L, rom_limits as RL
from scipy import ndimage
MM = 3.0; sc = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
STEP = {"S1": 15.0 * sc, "S2": 15.0 * sc, "S3": 30.0 * sc, "E": 20.0 * sc}; names = list(STEP)
vals = {j: np.arange(RL.SOFT[j][0], RL.SOFT[j][1] + 1e-9, s) for j, s in STEP.items()}; shape = tuple(len(vals[j]) for j in names)
JOINT_OF_DEPTH = {1: "S1", 2: "S2", 3: "S3", 4: "E", 5: "W1", 6: "W2", 7: "W3"}; DEPTH = dict(F=0, M1=1, M2=2, M3=3, ME=4, MW1=5, MW2=6, MW3=7)
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)
gm = L.build_group_meshes(); Vs = {}
for g in L.GROUPS:
    m = gm[g]; m = m if hasattr(m, "vertices") else trimesh.util.concatenate(list(m.values()) if isinstance(m, dict) else m); Vs[g] = m.vertices[::4]
def pair_d(F, g, h):
    Pw = (np.c_[Vs[g], np.ones(len(Vs[g]))] @ F[g].T)[:, :3]; Ph = (np.c_[Pw, np.ones(len(Pw))] @ np.linalg.inv(F[h]).T)[:, :3]; return float(L.sample_sdf(h, Ph).min())
F0 = ak.frames(PARK); pairs = [(g, h) for g in L.GROUPS for h in L.GROUPS if g != h and min(pair_d(F0, g, h), pair_d(F0, h, g)) >= MM]
t0 = time.time(); Dfull = np.full(shape, 1e3, np.float32); PRfull = np.full(shape, -1, np.int16); nq = 0; info = {}
for k, (g, h) in enumerate(pairs):
    a, b = sorted((DEPTH[g], DEPTH[h])); between = [JOINT_OF_DEPTH[d] for d in range(a + 1, b + 1)]; gj = [j for j in names if j in between]      # grid joints that matter for this pair
    axes = [names.index(j) for j in gj]; tshape = tuple(shape[i] for i in axes); T = np.zeros(tshape, np.float32) if axes else np.zeros((), np.float32)
    for idx in (itertools.product(*[range(s) for s in tshape]) if axes else [()]):
        q = dict(PARK); q.update({j: float(vals[j][i]) for j, i in zip(gj, idx)}); T[idx if axes else ()] = pair_d(ak.frames(q), g, h); nq += 1
    # broadcast to the full grid
    full = np.broadcast_to(T.reshape([shape[i] if i in axes else 1 for i in range(len(shape))]) if axes else T, shape)
    m = full < Dfull; Dfull = np.where(m, full, Dfull); PRfull = np.where(m, k, PRfull)
    info["%s-%s" % (g, h)] = dict(joints=gj, table_size=int(np.prod(tshape)) if axes else 1)
nq_grid = int(np.prod(shape)) * len(pairs)
ref = np.load("../stl/electronics/collision_frontier.npz"); Dref = ref["D"]
cm_new = Dfull < MM; cm_ref = Dref < MM
ok_mask = bool((cm_new == cm_ref).all()); pos = (Dref >= 0) & (Dfull >= 0); dmax = float(np.abs(Dfull[pos] - Dref[pos]).max())
free = ~cm_new; lab, _ = ndimage.label(free); seed = tuple(int(i) for i in ref["frontier"].shape and [np.argmin(np.abs(vals[j] - PARK[j])) for j in names]); reach = lab == lab[seed]
nb = ndimage.binary_dilation(reach, structure=ndimage.generate_binary_structure(len(shape), 1)); frontier = cm_new & nb
res = dict(grid_poses=int(np.prod(shape)), ordered_pairs=len(pairs), queries_grid=nq_grid, queries_separable=nq, reduction=round(nq_grid / nq, 1), contact_mask_identical=ok_mask, max_abs_D_diff_mm=round(dmax, 4),
           n_contact=int(cm_new.sum()), n_frontier=int(frontier.sum()), time_s=round(time.time() - t0, 1), pairs=info)
json.dump(res, open("../stl/electronics/collision_separable.json", "w"), indent=1); print({k: v for k, v in res.items() if k != "pairs"})
