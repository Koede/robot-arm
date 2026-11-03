"""Report for the MJ11 whole-arm sweep: per-moving-group first-contact frontier cells (union over that group's non-designed pairs), per-pair status, passive-stop-box check of the tier tables.
usage: python3 mj11_whole_report.py [sweep dir] -> mj11_whole_report.json"""
import os, sys, json, numpy as np, scipy.ndimage as ndi
HERE = os.path.dirname(os.path.abspath(__file__)); D = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "..", "stl", "j1r2_sweep"))
sys.argv = [sys.argv[0], D]; import importlib.util
spec = importlib.util.spec_from_file_location("t", os.path.join(HERE, "mj11_tiers.py")); T = importlib.util.module_from_spec(spec); import io, contextlib
with contextlib.redirect_stdout(io.StringIO()): spec.loader.exec_module(T)
J, GS, vals, sw, z = T.J, T.GS, T.vals, T.sw, T.z
contact = {}; bad_all = np.zeros(GS, bool)
for k, r in sw["pairs"].items():
    if r["designed"]: continue
    c = np.broadcast_to(T.bcast(z[k.replace("|", "__")], r["dims"]) < r["MM"], GS); contact[k] = c; bad_all |= c
pk = tuple(int(np.argmin(abs(vals[j] - T.PARK[j]))) for j in J); lab, _ = ndi.label(~bad_all, structure=ndi.generate_binary_structure(len(J), 1)); reach = lab == lab[pk]
nb = ndi.binary_dilation(reach, structure=ndi.generate_binary_structure(len(J), 1)); front = bad_all & nb
out = dict(total=dict(cells=int(np.prod(GS)), contact=int(bad_all.sum()), reach_free=int(reach.sum()), frontier=int(front.sum()), deep_dropped=int((bad_all & ~nb).sum())), groups={}, pairs={})
for g in ("M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"):
    # frontier cells attributed to the moving group = contact cells (pairs where g is the downstream group) adjacent to the reachable free region
    cg = np.zeros(GS, bool)
    for k, c in contact.items():
        a, b = k.split("|")
        if (a == g and not b.startswith("M")) or (b == g and b.startswith("M")) or False: cg |= c
    out["groups"][g] = dict(frontier_cells=int((cg & front).sum()), contact_cells=int(cg.sum()))
for k, r in sw["pairs"].items(): out["pairs"][k] = {x: r.get(x) for x in ("MM", "park_d", "status", "n_frontier_cells", "n_deep_cells", "d_frontier_min", "min_clear_reach_free") if x in r}
# passive box (S1p -95..5, S2 -15..17.4) + 5 deg hard-stop expansion: tier tables over the covered grid cells
S1, S2 = vals["S1p"], vals["S2"]; i = (S1 >= -100) & (S1 <= 10); j_ = (S2 >= -20) & (S2 <= 22.4)
tt = json.load(open(os.path.join(D, "mj11_tiers.json")))
for conv in ("fw15", "sweep"):
    out["box_check_" + conv] = {t: dict(min=float(np.array(tt[conv][t])[np.ix_(i, j_)].min()), max=float(np.array(tt[conv][t])[np.ix_(i, j_)].max()), n_closed=int((np.array(tt[conv][t])[np.ix_(i, j_)] < 0).sum()), n=int(i.sum() * j_.sum())) for t in ("FULL", "RESTRICT", "NEUTRAL")}
    out["park_cell_" + conv] = {t: tt[conv][t][int(np.argmin(abs(S1 + 18.2)))][int(np.argmin(abs(S2 - 17.4)))] for t in ("FULL", "RESTRICT", "NEUTRAL")}
json.dump(out, open(os.path.join(D, "mj11_whole_report.json"), "w"), indent=1, default=float); print(json.dumps({k: v for k, v in out.items() if k != "pairs"}, indent=1))
