"""Arm vs wearer + harness, first-contact frontier (same definition as collision_frontier.py).
Fixed bodies: wearer skin (gate 2 mm; head/neck region gate 60 mm) and harness (clasp frame, chassis plate, wing, spine bridge, pauldron; gate 3 mm).
Each fixed body is voxelised (3 mm) into a signed-distance field; moving-group vertices are looked up (trilinear). Resolution error <= ~1.5 mm, so frontier cells are
re-checked with exact point-to-mesh distances (trimesh proximity) before being reported.
Grid: S1 x S2 x S3 x E (soft range) x W2 {-70,0,80}, W1 = W3 = 0, G 50.  Usage: python3 collision_body.py  -> stl/electronics/collision_body.json (+ .npz)"""
import os, sys, json, time, itertools, numpy as np, trimesh
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L, rom_limits as RL
from scipy import ndimage
from scipy.ndimage import map_coordinates
S = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "shoulder") + "/"
PITCH = 3.0; LO = np.array([-560.0, -520.0, -820.0]); HI = np.array([300.0, 420.0, 700.0]); GATE = dict(skin=2.0, head=60.0, harness=3.0)
lm = json.load(open(S + "proxies/wearer_landmarks.json")); NECKZ = lm["neck_joint_rel_P"][2]
def sdf_field(mesh):
    """signed distance (mm, + outside) on the box LO..HI"""
    shape = np.ceil((HI - LO) / PITCH).astype(int) + 1
    ax = [LO[i] + PITCH * np.arange(shape[i]) for i in range(3)]
    vox = trimesh.voxel.creation.voxelize(mesh, PITCH).fill() if False else None
    # occupancy by ray parity is slow; use the mesh's own voxelisation on the same lattice
    vg = mesh.voxelized(PITCH).fill(); idx = np.round((vg.points - LO) / PITCH).astype(int)
    ok = np.all((idx >= 0) & (idx < shape), axis=1); occ = np.zeros(shape, bool); occ[tuple(idx[ok].T)] = True
    out = ndimage.distance_transform_edt(~occ, sampling=PITCH); inn = ndimage.distance_transform_edt(occ, sampling=PITCH)
    return (out - inn).astype(np.float32)
def look(F, P):
    c = (P - LO) / PITCH; return map_coordinates(F, c.T, order=1, mode="nearest")
wear = trimesh.load(S + "proxies/wearer_makehuman.stl"); head_mask = wear.triangles_center[:, 2] > NECKZ
head = wear.submesh([np.where(head_mask)[0]], append=True); torso = wear.submesh([np.where(~head_mask)[0]], append=True)
HAR = {k: trimesh.load(S + p) for k, p in dict(clasp="clasp_frame_R.stl", plate="chassis_plate_R.stl", wing="scapular/wing_R.stl", spine="scapular/spine_bridge.stl", pauldron="pauldron/pauldron_R.stl").items()}
t0 = time.time()
FIELDS = {"skin": (sdf_field(wear), GATE["skin"]), "harness": (sdf_field(trimesh.util.concatenate(list(HAR.values()))), GATE["harness"])}
HEADF = (sdf_field(head), GATE["head"]); print("fields %.0fs" % (time.time() - t0), flush=True)
gm = L.build_group_meshes(); Vs = {}
for g in L.GROUPS[1:]:
    m = gm[g]; m = m if hasattr(m, "vertices") else trimesh.util.concatenate(list(m.values()) if isinstance(m, dict) else m); Vs[g] = np.c_[m.vertices[::6], np.ones(len(m.vertices[::6]))]
BAL = os.environ.get("BAL", "0") == "1"; BP = {}
if BAL:
    for nm, (m, grp) in ak.balancer_parts().items(): BP[nm] = np.c_[m.vertices[::4], np.ones(len(m.vertices[::4]))]
    print("balancer parts:", sorted(BP), flush=True)
PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)
def dists(q):
    F = ak.frames(q); out = {}
    for g, V in Vs.items():
        P = (V @ F[g].T)[:, :3]
        out[g] = dict(skin=look(FIELDS["skin"][0], P).min() - 0.0, harness=look(FIELDS["harness"][0], P).min(), head=look(HEADF[0], P).min())
    if BAL:
        BF = ak.balancer_frames(q)
        for nm, V in BP.items():
            P = (V @ BF[nm].T)[:, :3]; out["B:" + nm] = dict(skin=look(FIELDS["skin"][0], P).min(), harness=look(FIELDS["harness"][0], P).min(), head=look(HEADF[0], P).min())
    return out
d0 = dists(PARK); excl = {(g, k) for g in d0 for k in d0[g] if d0[g][k] < {"skin": 2.0, "harness": 3.0, "head": 60.0}[k]}
print("designed-contact exclusions at park:", sorted(excl), flush=True)
AX = {"S1": 15.0, "S2": 15.0, "S3": 30.0, "E": 20.0, "W2": None}
vals = {j: np.arange(RL.SOFT[j][0], RL.SOFT[j][1] + 1e-9, s) for j, s in AX.items() if s}; vals["W2"] = np.array([-70.0, 0.0, 80.0]); names = list(vals)
shape = tuple(len(vals[j]) for j in names); margin = np.full(shape, 1e3, np.float32); who = np.empty(shape, object); n = 0
for idx in itertools.product(*[range(s) for s in shape]):
    q = dict(PARK); q.update({j: float(vals[j][i]) for j, i in zip(names, idx)}); d = dists(q); best = 1e3; bw = None
    for g in d:
        for k in ("skin", "harness", "head"):
            if (g, k) in excl: continue
            m_ = d[g][k] - {"skin": 2.0, "harness": 3.0, "head": 60.0}[k]
            if m_ < best: best, bw = m_, (g, k)
    margin[idx] = best; who[idx] = bw; n += 1
    if n % 500 == 0: print(" %d/%d %.0fs" % (n, margin.size, time.time() - t0), flush=True)
contact = margin < 0; free = ~contact; lab, _ = ndimage.label(free)
seed = tuple(int(np.argmin(np.abs(vals[j] - PARK.get(j, 0.0)))) for j in names)
if contact[seed]: fr_ = np.argwhere(free); seed = tuple(fr_[np.argmin(np.abs(fr_ - np.array(seed)).sum(1))])
reach = lab == lab[seed]; nb = ndimage.binary_dilation(reach, structure=ndimage.generate_binary_structure(len(shape), 1)); frontier = contact & nb; deep = contact & ~nb
from collections import Counter
cnt = Counter(who[tuple(i)] for i in np.argwhere(frontier)); cntd = Counter(who[tuple(i)] for i in np.argwhere(deep))
res = dict(grid={j: vals[j].tolist() for j in names}, n_poses=int(margin.size), n_contact=int(contact.sum()), n_free=int(free.sum()), n_reachable_free=int(reach.sum()), n_frontier=int(frontier.sum()), n_deep_dropped=int(deep.sum()),
           worst_margin_mm=float(margin.min()), frontier_by_pair={"%s|%s" % k: v for k, v in cnt.items()}, deep_by_pair={"%s|%s" % k: v for k, v in cntd.items()},
           exclusions=sorted("%s|%s" % e for e in excl), park_distance_mm={g: {k: round(float(v), 1) for k, v in d0[g].items()} for g in d0}, time_s=round(time.time() - t0, 1))
# first-contact examples: earliest-contact frontier poses with joint values
fl = []
for i in np.argwhere(frontier):
    fl.append(dict(q={j: float(vals[j][a]) for j, a in zip(names, i)}, pair="%s|%s" % who[tuple(i)], margin_mm=round(float(margin[tuple(i)]), 2)))
fl.sort(key=lambda r: r["margin_mm"]); res["frontier_worst20"] = fl[:20]
json.dump(res, open("../stl/electronics/collision_body%s.json" % ("_bal" if BAL else ""), "w"), indent=1); np.savez("../stl/electronics/collision_body%s.npz" % ("_bal" if BAL else ""), margin=margin, frontier=frontier, deep=deep)
print({k: v for k, v in res.items() if k not in ("frontier_worst20", "grid", "park_distance_mm")})
