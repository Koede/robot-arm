"""Which pair bounds each side of the MJ11 passive-stop box (5 deg shoulder-level grid, S3 {-70,0,90}) + the shoulder-level first-contact keep-out list.  -> stl/j1r2_sweep/mj11_shoulder5_report.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, j1r2_sweep as S, j1r2_opt as O, mj11_sweep as W, mj_concepts3 as C3
sc, _ = W.scene(C3.mj11_geo(None)); vals = S.grid_values(sc, dict(steps={"S1p": 5.0, "S2": 5.0, "S3": [-70.0, 0.0, 90.0]})); pairs = S.build_pairs(sc); J = sc.joints
js = json.load(open(os.path.join(S.OUT, "sweep_mj11_sh5.json"))); z = np.load(os.path.join(S.OUT, "tables_mj11_sh5.npz")); S1, S2 = vals["S1p"], vals["S2"]
tabs = {pi: z[p.key.replace("|", "__")] for pi, p in enumerate(pairs)}; des = [bool(js["pairs"][p.key]["designed"]) for p in pairs]
bad = {}; 
for pi, p in enumerate(pairs):
    if des[pi]: continue
    full = np.broadcast_to(S._bcast(tabs[pi], p.dims, J), tuple(len(vals[j]) for j in J)); bad[p.key] = (full < p.MM).any(axis=2)
ok = ~np.any(list(bad.values()), axis=0); box = O.best_box(ok[:, :, None], dict(S1p=S1, S2=S2)); r = box["ranges"]
i0, i1 = [int(np.argmin(abs(S1 - v))) for v in r["S1p"]]; j0, j1 = [int(np.argmin(abs(S2 - v))) for v in r["S2"]]
ring = {"S1p lo (hard stop -5 beyond)": (max(i0 - 1, 0), slice(max(j0 - 1, 0), min(j1 + 2, len(S2)))), "S1p hi": (min(i1 + 1, len(S1) - 1), slice(max(j0 - 1, 0), min(j1 + 2, len(S2))))}
out = dict(box=box, grid_S1p=S1.tolist(), grid_S2=S2.tolist(), bounding={})
for nm, (i, jj) in ring.items(): out["bounding"][nm] = {k: int(b[i, jj].sum()) for k, b in bad.items() if b[i, jj].any()}
for nm, (jx, ii) in {"S2 lo": (max(j0 - 1, 0), slice(max(i0 - 1, 0), min(i1 + 2, len(S1)))), "S2 hi": (min(j1 + 1, len(S2) - 1), slice(max(i0 - 1, 0), min(i1 + 2, len(S1))))}.items():
    out["bounding"][nm] = {k: int(b[ii, jx].sum()) for k, b in bad.items() if b[ii, jx].any()}
out["bounding_by_S1p_row_at_S2_hi"] = {}
# first-contact points per shoulder pair (sub-grid)
out["first_contact"] = {k: dict(status=v["status"], d_frontier_min=v.get("d_frontier_min"), example=v.get("first_contact_example"), ranges=v.get("first_contact"), nF=v.get("n_frontier_cells")) for k, v in js["pairs"].items() if v.get("n_frontier_cells")}
out["contact_edge_S2_at_each_S1p"] = {}
for k, b in bad.items():
    if not b.any(): continue
    edge = {}
    for a, s1 in enumerate(S1):
        col = np.where(b[a])[0]; free_below = [S2[j] for j in range(len(S2)) if not b[a, j]]
        if len(col): edge[float(s1)] = [float(S2[col.min()]), float(S2[col.max()])]
    out["contact_edge_S2_at_each_S1p"][k] = edge
json.dump(out, open(os.path.join(S.OUT, "mj11_shoulder5_report.json"), "w"), indent=1, default=float); print(json.dumps({k: out[k] for k in ("box", "bounding", "first_contact")}, indent=1, default=float))
# the ring (hard-stop expansion) is free by construction; the pair(s) that END the free region are the first contact beyond it
def beyond(nm, i=None, j=None, steps=2):
    res = {}
    for k, b in bad.items():
        if i is not None:
            ii = i + steps; sub = b[ii, max(j0 - 1, 0):min(j1 + 2, len(S2))] if 0 <= ii < len(S1) else np.zeros(1, bool)
        else:
            jj = j + steps; sub = b[max(i0 - 1, 0):min(i1 + 2, len(S1)), jj] if 0 <= jj < len(S2) else np.zeros(1, bool)
        if sub.any(): res[k] = int(sub.sum())
    return res
out["bounding_beyond_hard_stop"] = {"S1p hi (cell " + str(S1[min(i1 + 2, len(S1) - 1)]) + ")": beyond("", i=i1, steps=2), "S2 hi (cell " + str(S2[min(j1 + 2, len(S2) - 1)]) + ")": beyond("", j=j1, steps=2), "S2 lo (cell " + str(S2[max(j0 - 2, 0)]) + ")": beyond("", j=j0, steps=-2), "S1p lo": ("grid edge -95 = soft min" if i0 == 0 else beyond("", i=i0, steps=-2))}
json.dump(out, open(os.path.join(S.OUT, "mj11_shoulder5_report.json"), "w"), indent=1, default=float); print(json.dumps(out["bounding_beyond_hard_stop"], indent=1)); print("i0,i1,j0,j1", i0, i1, j0, j1, S1[i1 + 1:i1 + 4], S2[j1 + 1:j1 + 4], S2[max(j0 - 3, 0):j0])
