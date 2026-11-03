"""Post-process stl/j1r2_sweep/tables_mj11_whole.npz (mj11_whole_arm.py): per-group first-contact frontier, shoulder-level keep-outs, passive-stop box, and the elbow/wrist tier tables
(FULL / RESTRICT / NEUTRAL) on the (S1', S2) grid.  -> stl/j1r2_sweep/mj11_whole_report.json, elbow_tiers_mj11.json, firmware_preview/elbow_limits_mj11.h
Tier definitions as control.h: FULL any S3/wrist; RESTRICT |S3|<=30,|W2|<=30,|W3|<=15 (W1 free); NEUTRAL E<=5,|S3|<=10,|W2|,|W3|<=10 (W1 free).  EL = largest E (2.5 deg floor) such that every
pose with E' <= E (contiguous from E=0) and every wrist combo in the tier is contact-free (all non-designed pairs, gates 3/2/60 mm); linear zero-crossing of (d-MM) between grid E values."""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, scipy.ndimage as ndi
import j1r2_sweep as S, j1r2_opt as O, mj11_sweep as W, mj_concepts3 as C3, mj11_whole_arm as A
OUT = S.OUT; os.makedirs(os.path.join(OUT, "firmware_preview"), exist_ok=True)
sc, info = W.scene(C3.mj11_geo(None), "MJ11_whole", "W3"); vals = S.grid_values(sc, dict(steps=A.STEPS)); pairs = S.build_pairs(sc); J = sc.joints
js = json.load(open(os.path.join(OUT, "sweep_mj11_whole.json"))); z = np.load(os.path.join(OUT, "tables_mj11_whole.npz"))
tabs = {pi: z[p.key.replace("|", "__")] for pi, p in enumerate(pairs)}; designed = [bool(js["pairs"][p.key]["designed"]) for p in pairs]
GS = tuple(len(vals[j]) for j in J); res, rows, reach, frontier, contact_p = S.analyse(sc, pairs, tabs, vals, designed)
assert res["n_frontier"] == js["summary"]["n_frontier"], "post-process does not reproduce the sweep frontier"
rep = dict(grid={j: [float(v) for v in vals[j]] for j in J}, summary=res)
# ---- frontier cells per moving group (a cell counts for a group if any non-designed pair of that group is in contact there)
grp = {}
for pi, p in enumerate(pairs):
    if designed[pi]: continue
    g = p.g; grp.setdefault(g, np.zeros(GS, bool)); grp[g] |= contact_p[pi]
rep["frontier_cells_by_group"] = {g: int((m & frontier).sum()) for g, m in grp.items()}
rep["deep_cells_by_group"] = {g: int((m & ~frontier & (~reach)).sum()) for g, m in grp.items()}
# ---- frontier cells per pair (also by obstacle class)
rep["frontier_by_pair"] = {p.key: dict(nF=rows[p.key].get("n_frontier_cells", 0), d_frontier_min=rows[p.key].get("d_frontier_min"), status=rows[p.key]["status"], worst=rows[p.key].get("worst_contact_pose")) for p in pairs if not rows[p.key]["designed"] and rows[p.key].get("n_frontier_cells", 0)}
cls = {}
for p in pairs:
    if rows[p.key]["designed"]: continue
    c = ("head" if p.obj == "wearer_head" else "skin" if p.obj == "wearer_skin" else "group" if p.kind == "grp" else "MJ11 fixed" if p.obj.startswith("mj11_") else "wing" if p.obj == "wing" else "other fixed (S1 stage/rail/bumpers)")
    cls.setdefault(p.g, {}); cls[p.g][c] = cls[p.g].get(c, 0) + rows[p.key].get("n_frontier_cells", 0)
rep["frontier_pair_cells_by_group_and_class"] = cls
# ---- shoulder-level (M1, M2, M3 = S1p,S2,S3 only) frontier contacts and passive box
sh = [pi for pi, p in enumerate(pairs) if p.g in ("M1", "M2", "M3") and not designed[pi]]; i3 = J.index("S3"); S3v = vals["S3"]
ok = np.ones((len(vals["S1p"]), len(vals["S2"]), len(S3v)), bool); worst = {}
for pi in sh:
    p = pairs[pi]; t = tabs[pi]; full = S._bcast(t, p.dims, J[:3]) if p.dims else t.reshape(1, 1, 1); full = np.broadcast_to(full, ok.shape); ok &= ~(full < p.MM)
sh_con = ~ok; pk = (int(np.argmin(abs(vals["S1p"] - sc.park["S1p"]))), int(np.argmin(abs(vals["S2"] - sc.park["S2"]))))
def box_for(okk, s3idx):
    o = okk[:, :, s3idx] if np.isscalar(s3idx) else okk.all(axis=2)
    b = O.best_box(o[:, :, None], dict(S1p=vals["S1p"], S2=vals["S2"])); return b
lab, _ = ndi.label(ok[:, :, list(S3v).index(0.0)]); r0 = lab == lab[pk] if ok[pk[0], pk[1], list(S3v).index(0.0)] else np.zeros(ok.shape[:2], bool)
th_b, ph_b = O._preimage(); S1, S2 = vals["S1p"], vals["S2"]
ti = np.clip(np.round(np.interp(th_b, S1, np.arange(len(S1)))).astype(int), 0, len(S1) - 1); pj = np.clip(np.round(np.interp(ph_b, S2, np.arange(len(S2)))).astype(int), 0, len(S2) - 1)
inr = (th_b >= S1[0] - 5) & (th_b <= S1[-1] + 5) & (ph_b >= S2[0] - 5) & (ph_b <= S2[-1] + 5)
rep["shoulder_level"] = dict(grid_step_deg=10, reach_free_cov_S3_0=round(float(np.mean(inr & r0[ti, pj])), 3), box_S3_0=box_for(ok, list(S3v).index(0.0)), box_all_S3={f"S3={s}": box_for(ok, i) for i, s in enumerate(S3v)}, box_S3_all_three=box_for(ok, None))
fc = {}
for pi in sh:
    p = pairs[pi]; r = rows[p.key]
    if r.get("n_frontier_cells"): fc[p.key] = dict(status=r["status"], MM=p.MM, d_frontier_min=r.get("d_frontier_min"), worst_pose=r.get("worst_contact_pose"), first_contact=js["pairs"][p.key].get("first_contact"), first_contact_example=js["pairs"][p.key].get("first_contact_example"), n_frontier=r["n_frontier_cells"])
rep["shoulder_level"]["first_contacts"] = fc
# frontier cells (S1p,S2,S3) of every shoulder pair, explicit list
cells = {}
for pi in sh:
    p = pairs[pi]; t = tabs[pi]; dims = p.dims; shp = tuple(len(vals[j]) for j in dims)
    c = (t < p.MM); fr = frontier & contact_p[pi]; miss = tuple(i for i in range(len(J)) if J[i] not in dims); frp = fr.any(axis=miss) if miss else fr
    L_ = [{j: float(vals[j][i]) for j, i in zip(dims, idx)} | dict(d=round(float(t[tuple(idx)]), 1)) for idx in np.argwhere(frp)]
    if L_: cells[p.key] = L_
rep["shoulder_level"]["frontier_cells"] = cells
# ---- tier tables on (S1p, S2)
margin = np.full(GS, np.inf, np.float32)
for pi, p in enumerate(pairs):
    if designed[pi]: continue
    margin = np.minimum(margin, np.broadcast_to(S._bcast(tabs[pi] - p.MM, p.dims, J), GS))
Ev = vals["E"]; iE = J.index("E")
def idx(j, cond): return [i for i, v in enumerate(vals[j]) if cond(v)]
tier_sel = dict(FULL=dict(S3=idx("S3", lambda v: True), W1=idx("W1", lambda v: True), W2=idx("W2", lambda v: True), W3=idx("W3", lambda v: True)),
                RESTRICT=dict(S3=idx("S3", lambda v: abs(v) <= 30), W1=idx("W1", lambda v: True), W2=idx("W2", lambda v: abs(v) <= 30), W3=idx("W3", lambda v: abs(v) <= 15)),
                NEUTRAL=dict(S3=idx("S3", lambda v: abs(v) <= 10), W1=idx("W1", lambda v: True), W2=idx("W2", lambda v: abs(v) <= 10), W3=idx("W3", lambda v: abs(v) <= 10)))
def emax_curve(g):                      # g: margin along E (len nE) -> largest safe E (contiguous from E[0]); -1 if E[0] itself contacts
    if g[0] < 0: return -1.0
    k = np.argmax(g < 0) if (g < 0).any() else None
    if k is None: return float(Ev[-1])
    e = Ev[k - 1] + (Ev[k] - Ev[k - 1]) * g[k - 1] / (g[k - 1] - g[k]); return float(np.floor(e / 2.5) * 2.5)
tiers = {}
for name, sel in tier_sel.items():
    sub = margin[:, :, sel["S3"], :, :, :, :][:, :, :, :, sel["W1"], :, :][:, :, :, :, :, sel["W2"], :][:, :, :, :, :, :, sel["W3"]]     # axes S1p,S2,S3,E,W1,W2,W3
    g = sub.min(axis=(2, 4, 5, 6))                                                                                      # -> (S1p, S2, E)
    tiers[name] = np.array([[emax_curve(g[a, b]) for b in range(GS[1])] for a in range(GS[0])])
# NEUTRAL tier is E<=5: it is 'allowed' at E=0 if margin(E=0) >= 0 for the tier's wrist combos
ne = tier_sel["NEUTRAL"]; sub0 = margin[:, :, ne["S3"], 0, :, :, :][:, :, :, ne["W1"], :, :][:, :, :, :, ne["W2"], :][:, :, :, :, :, ne["W3"]]; neutral_ok = sub0.min(axis=(2, 3, 4, 5)) >= 0
shr = r0                                                    # shoulder-level reach-free at S3 = 0
rep["tiers"] = dict(axes=dict(S1p=[float(v) for v in vals["S1p"]], S2=[float(v) for v in vals["S2"]]),
    EL_F=tiers["FULL"].tolist(), EL_R=tiers["RESTRICT"].tolist(), EL_N_E0_ok=neutral_ok.tolist(), shoulder_reach_free_S3_0=shr.tolist(),
    stats={n: dict(cells=int(t.size), unrestricted_145=int((t >= 145).sum()), minus1=int((t < 0).sum()), mean_E_where_allowed=round(float(t[t >= 0].mean()), 1)) for n, t in tiers.items()},
    neutral_fail_cells_in_reach_free=int((~neutral_ok & shr).sum()), neutral_fail_cells_total=int((~neutral_ok).sum()))
json.dump(rep, open(os.path.join(OUT, "mj11_whole_report.json"), "w"), indent=1, default=float)
json.dump(dict(tier_axes=rep["tiers"]["axes"], EL_F=rep["tiers"]["EL_F"], EL_R=rep["tiers"]["EL_R"], neutral_E0_ok=rep["tiers"]["EL_N_E0_ok"], shoulder_reach_free_S3_0=rep["tiers"]["shoulder_reach_free_S3_0"], gates=sc.MM, note="proposal; not wired into firmware; see specs/SPEC_j1r2_mj11_whole_arm.md"), open(os.path.join(OUT, "elbow_tiers_mj11.json"), "w"), indent=1)
s1, s2 = vals["S1p"], vals["S2"]
with open(os.path.join(OUT, "firmware_preview", "elbow_limits_mj11.h"), "w") as f:
    f.write("// PREVIEW (not wired): generated by cad/j1r2/mj11_whole_post.py from stl/j1r2_sweep/tables_mj11_whole.npz -- do not edit.\n// Axes are S1' (not S1) and S2; park S1' -18.2 / S2 17.4. Gates: 3 mm parts, 2 mm skin, 60 mm head/neck (old tables used 15 mm to body/harness). 10 deg grid; look-up = MIN of the 4 surrounding points (control.h tab4).\n// EL_F: any S3/wrist. EL_R: |S3|<=30, |W2|<=30, |W3|<=15. -1 = not allowed in that tier (use NEUTRAL).\n#pragma once\n")
    f.write(f"const int EL_N1 = {len(s1)}, EL_N2 = {len(s2)};\nconst float EL_S1[EL_N1] = {{{','.join(f'{v:.2f}f' for v in s1)}}};\nconst float EL_S2[EL_N2] = {{{','.join(f'{v:.2f}f' for v in s2)}}};\n")
    for nm, t in (("EL_F", tiers["FULL"]), ("EL_R", tiers["RESTRICT"])): f.write(f"const float {nm}[EL_N1][EL_N2] = {{\n" + ",\n".join("{" + ",".join(f"{v:.1f}f" for v in row) + "}" for row in t) + "};\n")
print(json.dumps({k: rep[k] for k in ("frontier_cells_by_group", "frontier_pair_cells_by_group_and_class")}, indent=1))
print(json.dumps({k: v for k, v in rep["shoulder_level"].items() if k != "frontier_cells"}, indent=1, default=float)); print(json.dumps(rep["tiers"]["stats"]), rep["tiers"]["neutral_fail_cells_in_reach_free"], rep["tiers"]["neutral_fail_cells_total"])
