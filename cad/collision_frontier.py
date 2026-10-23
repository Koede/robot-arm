"""Collision frontier: keep only the collisions that matter = the FIRST-CONTACT surface in joint space.
Pose grid over S1 x S2 x S3 x E (soft range, wrist neutral). A pose is 'contact' when the closest non-adjacent group pair is < MM (3 mm gate).
Free poses are labelled into connected regions (axis-adjacent grid steps); the region containing the park pose is the REACHABLE set (the arm can only get anywhere else by crossing a contact pose).
  frontier = contact poses with a reachable free neighbour  -> the interaction points, the only collisions reported / used for keep-out
  deep     = contact poses with no reachable free neighbour -> arm cannot arrive there without first crossing the frontier: dropped
Pair exclusion: group pairs closer than MM at the park pose (adjacent links, bearings, stops) are designed contact and excluded (same rule as arm_keepout.legal_poses).
Usage: python3 collision_frontier.py [step_scale]   -> stl/electronics/collision_frontier.json, ../collision_frontier.png"""
import os, sys, json, time, itertools, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L, rom_limits as RL
from scipy import ndimage
MM = 3.0; sc = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
AX = {"S1": 15.0 * sc, "S2": 15.0 * sc, "S3": 30.0 * sc, "E": 20.0 * sc}
vals = {j: np.arange(RL.SOFT[j][0], RL.SOFT[j][1] + 1e-9, s) for j, s in AX.items()}; names = list(vals)
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)
gm = L.build_group_meshes(); Vs = {}
for g in L.GROUPS:
    m = gm[g]; m = m if hasattr(m, "vertices") else trimesh.util.concatenate(list(m.values()) if isinstance(m, dict) else m); Vs[g] = m.vertices[::4]
def pair_d(F, g, h):
    Pw = (np.c_[Vs[g], np.ones(len(Vs[g]))] @ F[g].T)[:, :3]; Ph = (np.c_[Pw, np.ones(len(Pw))] @ np.linalg.inv(F[h]).T)[:, :3]; return float(L.sample_sdf(h, Ph).min())
F0 = ak.frames(PARK); pairs = [(g, h) for g in L.GROUPS for h in L.GROUPS if g != h and min(pair_d(F0, g, h), pair_d(F0, h, g)) >= MM]
print("grid", {j: len(v) for j, v in vals.items()}, "poses", int(np.prod([len(v) for v in vals.values()])), "| non-adjacent ordered pairs:", len(pairs), flush=True)
shape = tuple(len(vals[j]) for j in names); D = np.full(shape, 1e3, np.float32); PR = np.full(shape, -1, np.int16); t0 = time.time(); n = 0
for idx in itertools.product(*[range(s) for s in shape]):
    q = dict(PARK); q.update({j: float(vals[j][i]) for j, i in zip(names, idx)}); F = ak.frames(q); best = 1e3; bp = -1
    for k, (g, h) in enumerate(pairs):
        d = pair_d(F, g, h)
        if d < best: best, bp = d, k
        if best < 0: break
    D[idx] = best; PR[idx] = bp; n += 1
    if n % 200 == 0: print(" %d poses %.0fs" % (n, time.time() - t0), flush=True)
contact = D < MM; free = ~contact
lab, nlab = ndimage.label(free)                                                  # axis-adjacent connectivity (default structure)
seed = tuple(int(np.argmin(np.abs(vals[j] - PARK[j]))) for j in names)
if contact[seed]:                                                                # park cell is a contact cell: take the nearest free cell
    fr = np.argwhere(free); seed = tuple(fr[np.argmin(np.abs(fr - np.array(seed)).sum(1))])
reach = lab == lab[seed]
nb = ndimage.binary_dilation(reach, structure=ndimage.generate_binary_structure(len(shape), 1))
frontier = contact & nb; deep = contact & ~nb
out = dict(grid={j: vals[j].tolist() for j in names}, n_poses=int(D.size), n_contact=int(contact.sum()), n_free=int(free.sum()), n_free_reachable=int(reach.sum()), n_free_unreachable=int((free & ~reach).sum()),
           n_frontier=int(frontier.sum()), n_deep_dropped=int(deep.sum()), seed=[int(i) for i in seed])
pc = {}
for k, (g, h) in enumerate(pairs):
    a = int(((PR == k) & contact).sum()); b = int(((PR == k) & frontier).sum())
    if a: pc["%s-%s" % (g, h)] = dict(all_contact=a, frontier=b)
out["by_pair"] = pc
fp = []
for idx in np.argwhere(frontier):
    t = tuple(idx); g, h = pairs[PR[t]]; fp.append(dict(q={j: float(vals[j][i]) for j, i in zip(names, t)}, d_mm=round(float(D[t]), 2), pair="%s-%s" % (g, h)))
out["frontier_poses"] = fp
json.dump(out, open("../stl/electronics/collision_frontier.json", "w"), indent=1)
print({k: v for k, v in out.items() if k.startswith("n_")}); print("by pair:", pc)
np.savez("../stl/electronics/collision_frontier.npz", D=D, frontier=frontier, reach=reach, deep=deep, **{j: vals[j] for j in names})
