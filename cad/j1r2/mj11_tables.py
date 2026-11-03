"""Post-process the MJ11 whole-arm sweep (mj11_whole_arm.py): first-contact frontier per moving group, shoulder-level keep-outs, and the elbow/wrist tier tables
(FULL / RESTRICT / NEUTRAL) on the (S1', S2) grid.  Reads stl/j1r2_sweep/sweep_<label>.json + tables_<label>.npz (never the scene).
Tier rule (grid-resolved): E_max(tier; S1', S2) = largest sampled E such that every sampled E' <= E is contact-free (all non-designed pairs, project gates) for EVERY
wrist/S3 combination of the tier; -1 = not even E = 0 is free in that tier (use the NEUTRAL tier).  FULL: S3 {-70,0,90} x W1 {-80,0,80} x W2 {-70,0,80} x W3 {-20,0,30}.
RESTRICT (|S3|<=30, |W2|<=30, |W3|<=15): S3 0, W2 0, W3 0, W1 {-80,0,80}.  NEUTRAL (E<=5, |S3|,|W2|,|W3|<=10): S3 W1 W2 W3 = 0 at E = 0; reported as free / not free per cell.
usage: J1R2_ROOT=<work root> python3 mj11_tables.py [label]"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy import ndimage as ndi
import clib as L, j1r2_params as P
OUT = os.path.join(L.ROOT, "stl", "j1r2_sweep"); JOINTS = ["S1p", "S2", "S3", "E", "W1", "W2", "W3"]
STEPS = {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}
def grid_vals():                                    # identical to j1r2_sweep.grid_values (soft range)
    vals = {}
    for j in JOINTS:
        lo, hi = P.SOFT[j]; st = STEPS[j]
        v = np.array(sorted(set(np.round(st, 4)))) if isinstance(st, list) else np.append(np.arange(lo, hi - 1e-9, st), hi)
        v = np.array(sorted(set(np.round(np.append(v, P.PARK[j]), 4)))); vals[j] = v[(v >= lo - 1e-9) & (v <= hi + 1e-9)]
    return vals
def bcast(t, dims):
    return t.reshape([t.shape[dims.index(j)] if j in dims else 1 for j in JOINTS])
def main(label):
    js = json.load(open(os.path.join(OUT, f"sweep_{label}.json"))); z = np.load(os.path.join(OUT, f"tables_{label}.npz")); vals = grid_vals()
    GS = tuple(len(vals[j]) for j in JOINTS); assert [int(js["grid"][j][2]) for j in JOINTS] == list(GS), (js["grid"], GS)
    contact = np.zeros(GS, bool); cp = {}; grp = {}
    for k, r in js["pairs"].items():
        if r["designed"]: continue
        t = z[k.replace("|", "__")]; c = np.broadcast_to(bcast(t < r["MM"], r["dims"]), GS); cp[k] = c; contact |= c
        g = k.split("|")[0]; grp[g] = grp.get(g, np.zeros(GS, bool)) | c
    pk = tuple(int(np.argmin(abs(vals[j] - P.PARK[j]))) for j in JOINTS); free = ~contact
    lab, _ = ndi.label(free, structure=ndi.generate_binary_structure(7, 1)); reach = (lab == lab[pk]) if free[pk] else np.zeros(GS, bool)
    nb = ndi.binary_dilation(reach, structure=ndi.generate_binary_structure(7, 1)); frontier = contact & nb
    assert int(frontier.sum()) == js["summary"]["n_frontier"], (int(frontier.sum()), js["summary"]["n_frontier"])
    rep = dict(label=label, grid={j: vals[j].tolist() for j in JOINTS}, n_cells=int(np.prod(GS)), n_contact=int(contact.sum()), n_reach_free=int(reach.sum()), n_frontier=int(frontier.sum()), n_deep=int((contact & ~nb).sum()),
               frontier_by_group={g: int((frontier & m).sum()) for g, m in grp.items()}, frontier_by_pair={}, shoulder_level=[])
    for k, r in js["pairs"].items():
        if r["designed"]: continue
        n = int((frontier & cp[k]).sum())
        if n == 0: continue
        e = dict(pair=k, MM=r["MM"], n_frontier_cells=n, d_frontier_min=r.get("d_frontier_min"), worst_contact_pose=r.get("worst_contact_pose"), first_contact_range=r.get("first_contact"))
        rep["frontier_by_pair"][k] = e
        dep = set(r["dims"])
        if dep <= {"S1p", "S2", "S3"}: rep["shoulder_level"].append(e)          # depends on shoulder joints only -> a limit / keep-out table entry
    # ---- tier tables on the (S1', S2) grid
    iE = JOINTS.index("E"); s1, s2 = vals["S1p"], vals["S2"]; Ev = vals["E"]
    def sel(**kw):
        c = contact
        for j, v in kw.items(): c = np.take(c, [int(np.argmin(abs(vals[j] - x))) for x in v], axis=JOINTS.index(j))
        return c
    def emax(c):                                     # c: (S1p,S2,S3',E,W1',W2',W3') contact for the tier's combos
        bad = c.any(axis=(2, 4, 5, 6)); out = np.full(bad.shape[:2], -1.0)                       # (S1p,S2,E)
        for a in range(bad.shape[0]):
            for b in range(bad.shape[1]):
                ok = np.where(bad[a, b])[0]; n = ok[0] if len(ok) else len(Ev)               # first contact index along E
                out[a, b] = Ev[n - 1] if n > 0 else -1.0
        return out
    EF = emax(contact); ER = emax(sel(S3=[0.0], W2=[0.0], W3=[0.0]))
    EN = contact[:, :, int(np.argmin(abs(vals["S3"]))), 0, int(np.argmin(abs(vals["W1"]))), int(np.argmin(abs(vals["W2"]))), int(np.argmin(abs(vals["W3"])))]
    key = lambda a, b: f"{a:.0f},{b:.0f}"; E_SOFT = P.SOFT["E"][1]
    tabF = {key(a, b): float(EF[i, j]) for i, a in enumerate(s1) for j, b in enumerate(s2)}; tabR = {key(a, b): float(ER[i, j]) for i, a in enumerate(s1) for j, b in enumerate(s2)}
    assert len(tabF) == len(s1) * len(s2), "table keys collide"
    json.dump(tabF, open(os.path.join(OUT, "mj11_elbow_limits_full.json"), "w"), indent=0); json.dump(tabR, open(os.path.join(OUT, "mj11_elbow_limits_restricted.json"), "w"), indent=0)
    json.dump(dict(S1=[float(x) for x in s1], S2=[float(x) for x in s2], E=[float(x) for x in Ev], S1_name="S1p"), open(os.path.join(OUT, "mj11_sweep_axes.json"), "w"), indent=0)
    reach_ij = reach.any(axis=(2, 3, 4, 5, 6))
    rep["tiers"] = dict(axes=dict(S1p=s1.tolist(), S2=s2.tolist()), EL_F=EF.tolist(), EL_R=ER.tolist(), NEUTRAL_contact=EN.tolist(), n_cells=int(EF.size),
                        full_unrestricted=int((EF >= E_SOFT).sum()), full_forbidden=int((EF < 0).sum()), restr_unrestricted=int((ER >= E_SOFT).sum()), restr_forbidden=int((ER < 0).sum()),
                        neutral_not_free_cells=[[float(s1[i]), float(s2[j])] for i, j in zip(*np.where(EN))], park_cell_EF=float(EF[int(np.argmin(abs(s1 - P.PARK['S1p']))), int(np.argmin(abs(s2 - P.PARK['S2'])))]))
    json.dump(rep, open(os.path.join(OUT, "mj11_whole_arm_summary.json"), "w"), indent=1, default=float); return rep, EF, ER, EN, s1, s2
if __name__ == "__main__":
    rep, EF, ER, EN, s1, s2 = main(sys.argv[1] if len(sys.argv) > 1 else "mj11_whole_arm")
    print({k: v for k, v in rep.items() if k not in ("grid", "frontier_by_pair", "shoulder_level", "tiers")}); print("tiers", {k: v for k, v in rep["tiers"].items() if not isinstance(v, list) or k == "neutral_not_free_cells"})
    print("S2 ->", s2.astype(int).tolist())
    for nm, T in (("EL_F", EF), ("EL_R", ER)):
        print(nm)
        for i, a in enumerate(s1): print(f"{a:7.1f}", " ".join(f"{int(v):4d}" for v in T[i]))
