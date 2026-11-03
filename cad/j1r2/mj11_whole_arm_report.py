"""Post-process the MJ11 whole-arm sweep tables (mj11_whole_arm.py): first-contact frontier cells per moving group / partner, shoulder-level keep-outs, passive-stop box,
elbow/wrist tier tables (FULL / RESTRICT / NEUTRAL) and the straight-arm keep-out map in the firmware-generator format.
usage: J1R2_ROOT=<work root> python3 mj11_whole_arm_report.py  -> stl/j1r2_sweep/mj11_whole_arm_report.json, mj11_elbow_limits*.json, mj11_keepout_map.npz, firmware_proposed/*.h"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, scipy.ndimage as ndi
import j1r2_params as P, j1r2_opt as O, mj11_whole_arm as W
OUT = os.path.join(os.environ.get("J1R2_ROOT", "."), "stl", "j1r2_sweep")
CHAIN = ["S1p", "S2", "S3", "E", "W1", "W2", "W3"]
def load(label, steps):
    o = json.load(open(os.path.join(OUT, f"sweep_{label}.json"))); z = np.load(os.path.join(OUT, f"tables_{label}.npz")); vals = {}
    for j in CHAIN:
        lo, hi, n = o["grid"][j]; st = steps[j]; slo, shi = P.SOFT[j]          # same construction as j1r2_sweep.grid_values (park value is inserted)
        v = np.array(sorted(set(np.round(st, 4)))) if isinstance(st, (list, tuple)) else np.append(np.arange(slo, shi - 1e-9, st), shi)
        v = np.array(sorted(set(np.round(np.append(v, P.PARK[j]), 4)))); vals[j] = v[(v >= slo - 1e-9) & (v <= shi + 1e-9)]
        assert len(vals[j]) == n and abs(vals[j][0] - lo) < 1e-6 and abs(vals[j][-1] - hi) < 1e-6, (j, vals[j], o["grid"][j])
    pairs = []
    for k, r in o["pairs"].items():
        g = k.split("|")[0] if r["dims"] == [] else None
        pairs.append(dict(key=k, dims=r["dims"], MM=r["MM"], designed=r["designed"], tab=np.asarray(z[k.replace("|", "__")])))
    for p in pairs:
        a, b = p["key"].split("|"); p["g"] = b if a in ("M1", "M2", "M3", "ME", "MW1", "MW2", "MW3") and b in ("M1", "M2", "M3", "ME", "MW1", "MW2", "MW3") else a
        p["partner"] = b if p["g"] == a else a
        p["cat"] = "wearer_head" if p["partner"] == "wearer_head" else "wearer_skin" if p["partner"] == "wearer_skin" else "group" if p["partner"].startswith("M") and p["partner"][1:2] in "123EW" and not p["partner"].startswith("mj") else "fixed"
    return o, z, vals, pairs
def bc(p, J):
    t = p["tab"]; return t.reshape([t.shape[p["dims"].index(j)] if j in p["dims"] else 1 for j in J])
def analyse(vals, pairs, park):
    J = CHAIN; GS = tuple(len(vals[j]) for j in J); contact = np.zeros(GS, bool)
    for p in pairs:
        p["con"] = np.zeros(GS, bool) if p["designed"] else np.broadcast_to(bc(p, J) < p["MM"], GS)
        contact |= p["con"]
    pk = tuple(int(np.argmin(np.abs(vals[j] - park[j]))) for j in J); st = ndi.generate_binary_structure(7, 1); free = ~contact; lab, _ = ndi.label(free, structure=st)
    reach = (lab == lab[pk]) if free[pk] else np.zeros(GS, bool); frontier = contact & ndi.binary_dilation(reach, structure=st)
    return contact, reach, frontier
def fmt(x): return float(np.round(x, 2))
def main():
    O_, z, vals, pairs = load("mj11_whole_arm", W.STEPS); J = CHAIN
    contact, reach, frontier = analyse(vals, pairs, P.PARK); S = O_["summary"]
    assert int(contact.sum()) == S["n_contact"] and int(frontier.sum()) == S["n_frontier"] and int(reach.sum()) == S["n_reach_free"], "re-analysis does not reproduce run_sweep summary"
    R = dict(label="mj11_whole_arm", clasp_psi=120.0, pad=[-60.0, -190.0, 12.0], grid={j: [fmt(v) for v in vals[j]] for j in J}, summary=dict(S), MM=O_["MM"])
    # ---- frontier cells per moving group (union over its pairs) and per partner category
    groups = ["M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"]; pg = {}
    for g in groups:
        ps = [p for p in pairs if p["g"] == g and not p["designed"]]; u = np.zeros_like(contact)
        for p in ps: u |= p["con"]
        fr = u & frontier; d = dict(contact_cells=int(u.sum()), frontier_cells=int(fr.sum()), deep_cells=int((u & ~frontier).sum()), by_partner={})
        for cat in ("fixed", "wearer_skin", "wearer_head", "group"):
            uc = np.zeros_like(contact)
            for p in ps:
                if p["cat"] == cat: uc |= p["con"]
            d["by_partner"][cat] = int((uc & frontier).sum())
        pg[g] = d
    R["per_group"] = pg
    # ---- per pair frontier cells + first contact (from run_sweep rows)
    rows = []
    for p in pairs:
        if p["designed"]: continue
        r = O_["pairs"][p["key"]]
        if r.get("n_frontier_cells"):
            fp = z["fcp__" + p["key"].replace("|", "__")] if ("fcp__" + p["key"].replace("|", "__")) in z.files else np.zeros((0, 9))
            row = dict(pair=p["key"], MM=p["MM"], frontier_cells_projected=r["n_frontier_cells"], d_frontier_min=r.get("d_frontier_min"), status=r["status"], park_d=round(r["park_d"], 1), n_first_contact_points=int(len(fp)))
            if len(fp): row["first_contact_ranges"] = {j: [fmt(fp[:, i].min()), fmt(fp[:, i].max())] for i, j in enumerate(J)}; k = int(np.argmin(fp[:, 7] - p["MM"])); row["example"] = {j: fmt(fp[k, i]) for i, j in enumerate(J)}
            rows.append(row)
        elif r["status"].startswith("BELOW") : rows.append(dict(pair=p["key"], status=r["status"]))
    R["pairs_with_frontier"] = rows
    # ---- shoulder-level keep-outs: pairs whose moving group is M1/M2/M3 (dims within S1p,S2,S3)
    sh = [r for r in rows if r["pair"].split("|")[0] in ("M1", "M2", "M3") and (r["pair"].split("|")[1] in ("M1", "M2", "M3", "wearer_skin", "wearer_head", "mj11_clasp_bracket", "wing") or True) and next(p for p in pairs if p["key"] == r["pair"])["g"] in ("M1", "M2", "M3")]
    R["shoulder_level_contacts"] = sh
    # ---- passive-stop box: shoulder groups only, all S3 values, 10 deg grid (hard-stop expansion = 1 cell = 10 deg: conservative)
    J3 = ["S1p", "S2", "S3"]; ok3 = np.ones(tuple(len(vals[j]) for j in J3), bool); wst = {}
    for p in pairs:
        if p["designed"] or p["g"] not in ("M1", "M2", "M3") or any(d not in J3 for d in p["dims"]): continue
        t = p["tab"].reshape([p["tab"].shape[p["dims"].index(j)] if j in p["dims"] else 1 for j in J3]); ok3 &= ~np.broadcast_to(t < p["MM"], ok3.shape); wst[p["key"]] = float(t.min())
    sv = {j: vals[j] for j in J3}; box = O.best_box(ok3, sv); R["passive_box_10deg_grid_allS3"] = box
    R["shoulder_ok_map_S3all"] = dict(S1p=[fmt(v) for v in vals["S1p"]], S2=[fmt(v) for v in vals["S2"]], ok=ok3.all(axis=2).astype(int).tolist())
    # ---- tier tables
    def tier(vals_, pairs_, subset=None):
        Jx = CHAIN; GS = tuple(len(vals_[j]) for j in Jx); bad = np.zeros(GS, bool)
        for p in pairs_:
            if not p["designed"]: bad |= np.broadcast_to(bc(p, Jx) < p["MM"], GS)
        tab = np.full((GS[0], GS[1]), -1.0); Ev = vals_["E"]
        for a in range(GS[0]):
            for b in range(GS[1]):
                okE = -1.0
                for ie, e in enumerate(Ev):
                    if bad[a, b, :, ie].any(): break
                    okE = float(e)
                tab[a, b] = okE
        return tab, bad
    EF, badF = tier(vals, pairs); R["tier_FULL"] = dict(S1p=[fmt(v) for v in vals["S1p"]], S2=[fmt(v) for v in vals["S2"]], E_grid=[fmt(v) for v in vals["E"]], Emax=EF.tolist())
    O2, z2, vals2, pairs2 = load("mj11_whole_arm_restricted", W.STEPS_RESTRICT) if os.path.exists(os.path.join(OUT, "sweep_mj11_whole_arm_restricted.json")) else (None,) * 4
    if O2 is not None:
        # restricted grid has the same S1p/S2/E axes: asserts below
        for j in ("S1p", "S2", "E"): assert np.allclose(vals[j], vals2[j])
        ER, badR = tier(vals2, pairs2); R["tier_RESTRICT"] = dict(Emax=ER.tolist(), summary=O2["summary"])
        # RESTRICT box (S3 +-30, W2 +-30, W3 +-15) lies inside the FULL ranges, but the two grids sample different corners. A pose in the restricted box that fails at lower E than the FULL grid
        # found is a real FULL-range pose too -> EL_F = min(EL_F_raw, EL_R). EL_R is never raised.
        R["tier_RESTRICT"]["Emax"] = ER.tolist(); R["tier_FULL_raw"] = R["tier_FULL"]["Emax"]; EFm = np.minimum(EF, ER); R["tier_FULL"]["Emax"] = EFm.tolist(); R["tier_FULL"]["cells_lowered_by_restricted_samples"] = int((EFm < EF).sum()); EF = EFm
    # ---- NEUTRAL: E=0 (<=5), S3=0, W1/W2/W3=0 slice of the full tables; fine straight-arm sweep
    i0 = lambda j, v=0.0: int(np.argmin(abs(vals[j] - v)))
    neut = badF[:, :, i0("S3"), 0, i0("W1"), i0("W2"), i0("W3")]; R["neutral_fail_cells_10deg"] = int(neut.sum()); R["neutral_fail_list"] = [[fmt(vals["S1p"][a]), fmt(vals["S2"][b])] for a, b in np.argwhere(neut)]
    SA = {}
    if os.path.exists(os.path.join(OUT, "sweep_mj11_straight_arm.json")):
        O3, z3, vals3, pairs3 = load("mj11_straight_arm", W.STEPS_STRAIGHT); n1, n2 = len(vals3["S1p"]), len(vals3["S2"]); Jx = CHAIN; GS = tuple(len(vals3[j]) for j in Jx)
        dskin = np.full(GS, np.inf); dhead = np.full(GS, np.inf); dpart = np.full(GS, np.inf); badS = np.zeros(GS, bool); worst = {}
        for p in pairs3:
            if p["designed"]: continue
            t = np.broadcast_to(bc(p, Jx), GS); badS |= t < p["MM"]
            if p["partner"] == "wearer_skin": dskin = np.minimum(dskin, t)
            elif p["partner"] == "wearer_head": dhead = np.minimum(dhead, t)
            else: dpart = np.minimum(dpart, t)
        ie0 = int(np.argmin(abs(vals3["E"]))); sq = lambda a: a[:, :, 0, ie0, 0, 0, 0]
        badS = badS
        contactS, reachS, frontS = analyse(vals3, pairs3, P.PARK); rs = reachS[:, :, 0, ie0, 0, 0, 0]
        margin = np.minimum.reduce([sq(dskin) - O3["MM"]["skin"], sq(dhead) - O3["MM"]["head"], sq(dpart) - O3["MM"]["part"]])
        C = np.minimum(sq(dskin), sq(dhead)); SA = dict(S1p=vals3["S1p"], S2=vals3["S2"], C=C, margin=margin, reach=rs, d_skin=sq(dskin), d_head=sq(dhead), d_parts=sq(dpart), bad=sq(badS))
        np.savez_compressed(os.path.join(OUT, "mj11_keepout_map.npz"), S1=vals3["S1p"], S2=vals3["S2"], C=C, d_skin=SA["d_skin"], d_head=SA["d_head"], d_parts=SA["d_parts"], bad=SA["bad"], margin_gate=SA["margin"], reach_free_E0=SA["reach"])
        R["straight_arm"] = dict(grid=[n1, n2], cells_failing_gate=int(SA["bad"].sum()), min_skin_mm=fmt(SA["d_skin"].min()), min_head_mm=fmt(SA["d_head"].min()), min_parts_mm=fmt(SA["d_parts"].min()), summary=O3["summary"],
                                 neutral_fail_cells_2p5deg=int(SA["bad"].sum()), E_grid=[float(v) for v in vals3["E"]], reach_free_E0_cells=int(rs.sum()), reach_free_E0_S1p=[fmt(vals3["S1p"][np.where(rs.any(1))[0]].min()), fmt(vals3["S1p"][np.where(rs.any(1))[0]].max())] if rs.any() else None, reach_free_E0_S2=[fmt(vals3["S2"][np.where(rs.any(0))[0]].min()), fmt(vals3["S2"][np.where(rs.any(0))[0]].max())] if rs.any() else None, frontier_cells_E_0_45=int(frontS.sum()), neutral_fail_ranges=None if not SA["bad"].any() else dict(S1p=[fmt(vals3["S1p"][np.where(SA["bad"].any(1))[0]].min()), fmt(vals3["S1p"][np.where(SA["bad"].any(1))[0]].max())], S2=[fmt(vals3["S2"][np.where(SA["bad"].any(0))[0]].min()), fmt(vals3["S2"][np.where(SA["bad"].any(0))[0]].max())]))
    json.dump(R, open(os.path.join(OUT, "mj11_whole_arm_report.json"), "w"), indent=1, default=float)
    # gen_tables-format elbow json (key "s1,s2" -> E) for FULL / RESTRICT on the new (S1p,S2) grid; sweep_full.json-like header
    def dump(tab, name):
        d = {f"{a:.0f},{b:.0f}": float(tab[i, k]) for i, a in enumerate(vals["S1p"]) for k, b in enumerate(vals["S2"])}; json.dump(d, open(os.path.join(OUT, name), "w"), indent=1)
    dump(EF, "mj11_elbow_limits.json")
    if O2 is not None: dump(ER, "mj11_elbow_limits_restricted.json")
    json.dump(dict(S1=[float(v) for v in vals["S1p"]], S2=[float(v) for v in vals["S2"]], E_tested=[float(v) for v in vals["E"]], note="S1 axis = S1' (MJ11 blade-seat J1); gates part 3 / skin 2 / head 60 mm; first-contact E scan from E=0; -1 = even E=0 violates a gate at that shoulder pose"), open(os.path.join(OUT, "mj11_sweep_full_axes.json"), "w"), indent=1)
    print(json.dumps({k: R[k] for k in ("summary", "per_group", "passive_box_10deg_grid_allS3")}, indent=1, default=float)); print(len(sh), "shoulder-level pairs with frontier")
    return R
if __name__ == "__main__": main()
