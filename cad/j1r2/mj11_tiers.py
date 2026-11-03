"""Post-process a whole-arm sweep (tables_<label>.npz + sweep_<label>.json): frontier cells per moving group, shoulder-level keep-out contacts, passive-stop box, and the elbow/wrist tier tables (FULL / RESTRICT / NEUTRAL).
usage: J1R2_ROOT=<work root> python3 mj11_tiers.py [label_full] [label_restricted]  -> stl/j1r2_sweep/mj11_tiers.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, scipy.ndimage as ndi
import clib as L, j1r2_params as P, j1r2_opt as O, mj_frame as M
OUT = os.path.join(M.REPO, "stl", "j1r2_sweep"); J = ["S1p", "S2", "S3", "E", "W1", "W2", "W3"]
def load(label):
    d = json.load(open(os.path.join(OUT, f"sweep_{label}.json"))); z = np.load(os.path.join(OUT, f"tables_{label}.npz")); g = d["grid"]
    vals = {j: np.linspace(g[j][0], g[j][1], int(g[j][2])) if j in ("S1p", "S2", "E") else None for j in J}
    return d, z, g
def grid_vals(label, steps_json):
    d, z, g = load(label); return d, z, g
def values(g, steps):
    """replicates j1r2_sweep.grid_values (soft range, park value added)"""
    v = {}
    for j in J:
        lo, hi = g[j][0], g[j][1]; st = steps[j]
        a = np.array(sorted(set(np.round(st, 4)))) if isinstance(st, list) else np.append(np.arange(lo, hi - 1e-9, st), hi)
        a = np.array(sorted(set(np.round(np.append(a, P.PARK[j]), 4)))); v[j] = a[(a >= lo - 1e-9) & (a <= hi + 1e-9)]
        assert len(v[j]) == g[j][2], (j, len(v[j]), g[j])
    return v
def analyse(label, steps):
    d, z, g = load(label); vals = values(g, steps); GS = tuple(len(vals[j]) for j in J); contact, cp, dist, mm = np.zeros(GS, bool), {}, {}, {}
    for k, r in d["pairs"].items():
        if r["designed"]: continue
        t = z[k.replace("|", "__")]; dims = r["dims"]; sh = [t.shape[dims.index(j)] if j in dims else 1 for j in J]; t = np.broadcast_to(t.reshape(sh), GS)
        dist[k] = t; mm[k] = r["MM"]; cp[k] = t < r["MM"]; contact |= cp[k]
    pk = tuple(int(np.argmin(np.abs(vals[j] - P.PARK[j]))) for j in J); free = ~contact; lab, _ = ndi.label(free, structure=ndi.generate_binary_structure(7, 1)); reach = (lab == lab[pk]) if free[pk] else np.zeros(GS, bool)
    nb = ndi.binary_dilation(reach, structure=ndi.generate_binary_structure(7, 1)); front = contact & nb
    return dict(d=d, vals=vals, GS=GS, contact=contact, cp=cp, dist=dist, mm=mm, reach=reach, front=front)
def groups_of(k): a, b = k.split("|"); return a, b
def report(A):
    vals, front, cp, J_ = A["vals"], A["front"], A["cp"], J; rep = {}
    # frontier cells per moving group (union over its pairs; a group-group pair counts for the downstream group)
    order = ["M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"]; per = {}
    for g in order:
        m = np.zeros(A["GS"], bool); pairs = []
        for k, c in cp.items():
            a, b = groups_of(k); mover = a if not (b in order) else max(a, b, key=order.index)
            if mover == g: m |= c; pairs.append(k)
        per[g] = dict(frontier_cells=int((front & m).sum()), contact_cells=int(m.sum()), n_pairs_with_frontier=sum(1 for k in pairs if (front & cp[k]).any()))
    rep["frontier_per_group"] = per
    # shoulder-level contacts (groups M1..M3: their own pairs, S1p x S2 x S3 only matter)
    sh = {}
    for k, c in cp.items():
        a, b = groups_of(k)
        if a in ("M1", "M2", "M3") and b not in ("ME", "MW1", "MW2", "MW3"):
            f = front & c
            if f.any():
                idx = np.argwhere(f); ext = {j: [float(vals[j][idx[:, i]].min()), float(vals[j][idx[:, i]].max())] for i, j in enumerate(J_[:3])}
                # worst pose, free-side clearance, and the reach-free minimum distance
                rr = A["dist"][k][A["reach"]]; sh[k] = dict(MM=A["mm"][k], frontier_cells=int(f.sum()), S1p_S2_S3_extent=ext, min_dist_reach_free=float(rr.min()) if rr.size else None, status=A["d"]["pairs"][k]["status"])
    rep["shoulder_level_contacts"] = sh
    return rep
SHOULDER_PAIRS = lambda k: groups_of(k)[0] in ("M1", "M2", "M3") and groups_of(k)[1] not in ("ME", "MW1", "MW2", "MW3")
def shoulder_ok(A):
    """allowed map over (S1p,S2,S3) from shoulder-level pairs only, taking the worst over the distal joints for nothing (those pairs do not depend on them)"""
    ok = np.ones(A["GS"][:3], bool)
    for k, c in A["cp"].items():
        if SHOULDER_PAIRS(k): ok &= ~c.reshape(A["GS"][:3] + (-1,)).any(axis=3)
    return ok
def tier_tables(A, sel):
    """E_max per (S1p,S2) for the tier whose joints are restricted to the index sets sel = {S3:[i..], W1:[..], W2:[..], W3:[..]}; ok if every pair >= MM on all lower E too (E scanned from 0 up)."""
    vals = A["vals"]; ok = ~A["contact"]; S3, W1, W2, W3 = (sel[j] for j in ("S3", "W1", "W2", "W3")); sub = ok[:, :, S3][:, :, :, :, W1][:, :, :, :, :, W2][:, :, :, :, :, :, W3]
    allE = sub.all(axis=(2, 4, 5, 6))                                        # (S1p, S2, E)
    n1, n2, ne = allE.shape; out = -np.ones((n1, n2)); Ev = vals["E"]
    for i in range(n1):
        for k in range(n2):
            last = -1.0
            for e in range(ne):
                if allE[i, k, e]: last = Ev[e]
                else: break
            out[i, k] = last
    return out
if __name__ == "__main__":
    lf = sys.argv[1] if len(sys.argv) > 1 else "mj11_whole_arm"; lr = sys.argv[2] if len(sys.argv) > 2 else "mj11_whole_arm_restricted"
    import mj11_whole_arm as WA
    A = analyse(lf, WA.STEPS); R = report(A); vals = A["vals"]
    ok3 = shoulder_ok(A); R["passive_box_whole_arm_tables_shoulder_pairs_S3_all"] = O.best_box(ok3, vals)
    ok_full7 = ~A["contact"]; R["passive_box_all_pairs"] = O.best_box(ok_full7, vals)
    R["grid"] = {j: vals[j].tolist() for j in J}; R["reach_free_cells"] = int(A["reach"].sum())
    idx = lambda j, vs: [int(np.argmin(abs(vals[j] - v))) for v in vs]
    FULL = tier_tables(A, dict(S3=list(range(3)), W1=list(range(3)), W2=list(range(3)), W3=list(range(3))))
    # RESTRICT from the full-grid sweep: only S3=0, W2=0, W3=0 samples are inside the RESTRICT box here (+-30/+-15 not sampled): kept as a cross-check against the dedicated restricted sweep
    RES_coarse = tier_tables(A, dict(S3=idx("S3", [0]), W1=list(range(3)), W2=idx("W2", [0]), W3=idx("W3", [0])))
    R["EL_F"] = FULL.tolist(); R["EL_R_coarse_from_full_grid"] = RES_coarse.tolist(); R["S1p"] = vals["S1p"].tolist(); R["S2"] = vals["S2"].tolist()
    # NEUTRAL: E=0 (E<=5), S3=0, W2=W3=0 (|.|<=10), W1 any (0 sampled): needs d>=MM on every pair; report cells where it fails
    neu = ~A["contact"][:, :, idx("S3", [0])[0], idx("E", [0])[0], 1, idx("W2", [0])[0], idx("W3", [0])[0]]; R["NEUTRAL_ok_S1p_x_S2"] = neu.astype(int).tolist()
    # straight-arm clearance (E=0,S3=0,W=0): min over pairs of (d - MM) -> margin; and raw d to wearer parts
    sa = (0, 0, idx("S3", [0])[0], idx("E", [0])[0], 1, idx("W2", [0])[0], idx("W3", [0])[0]); marg = np.full(A["GS"][:2], np.inf); head = np.full(A["GS"][:2], np.inf)
    for k, t in A["dist"].items():
        v = t[:, :, sa[2], sa[3], sa[4], sa[5], sa[6]]; marg = np.minimum(marg, v - A["mm"][k])
        if k.endswith("wearer_head"): head = np.minimum(head, v)
    R["straight_arm_min_margin_mm"] = np.round(marg, 1).tolist(); R["straight_arm_head_mm"] = np.round(head, 1).tolist()
    if os.path.exists(os.path.join(OUT, f"tables_{lr}.npz")):
        B = analyse(lr, WA.STEPS_R); RES = tier_tables(B, dict(S3=list(range(3)), W1=list(range(3)), W2=list(range(3)), W3=list(range(3)))); R["EL_R"] = RES.tolist(); R["restricted_sweep"] = dict(steps=WA.STEPS_R, reach_free=int(B["reach"].sum()), frontier=int(B["front"].sum()))
        R["frontier_per_group_restricted"] = report(B)["frontier_per_group"]
    json.dump(R, open(os.path.join(OUT, "mj11_tiers.json"), "w"), indent=1, default=float)
    print(json.dumps({k: R[k] for k in ("frontier_per_group", "passive_box_whole_arm_tables_shoulder_pairs_S3_all", "passive_box_all_pairs", "reach_free_cells")}, indent=1, default=float))
    print(json.dumps(R["shoulder_level_contacts"], indent=1, default=float))
    S1, S2 = vals["S1p"], vals["S2"]; fmt = lambda T: "S1p\\S2 " + " ".join("%5.0f" % s for s in S2) + "\n" + "\n".join("%6.0f " % a + " ".join("%5d" % v for v in row) for a, row in zip(S1, T))
    print("EL_F\n" + fmt(FULL)); print("EL_R(coarse)\n" + fmt(RES_coarse))
    if "EL_R" in R: print("EL_R\n" + fmt(np.array(R["EL_R"])))
