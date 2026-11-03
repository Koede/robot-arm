"""MJ11 balancer alternatives (2026-11-03): (1) cord-clear pad-spring design search (cord vs the S1' group over the box), (2) S1'-only torsion cassette residuals incl. boots,
(3) free annulus around the S1' motor for a coaxial clock/negator spring. usage: J1R2_ROOT=/home/claude/ts python3 mj11_balancer_alt.py -> stl/j1r2_bal/mj11_balancer_alt.json"""
import os, sys, json, math
os.environ["SKIP_PACK"] = "1"
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "mj11_balancer_refit.py")).read()
g = {"__name__": "alt", "__file__": os.path.join(os.path.dirname(os.path.abspath(__file__)), "mj11_balancer_refit.py")}
exec(compile(src[:src.index("# --- 1. invariance check")], "refit_head", "exec"), g)       # constants, gravity pair arrays, spring_torque / residual
import numpy as np
from scipy.spatial import cKDTree
import mj_eval as E, j1r2_params as P
pairs, FA, P1, DER, residual = g["pairs"], g["FA"], g["P1"], g["DER"], g["residual"]; REPO, OUT = g["REPO"], g["OUT"]
TH = np.arange(-100.0, 5.1, 2.5); trees = {}
for th in TH:
    Mx = E.G.pose_matrix("M1", E.DER, th, 0.0); trees[float(th)] = cKDTree(E.MOV1 @ Mx[:3, :3].T + Mx[:3, 3])
PHs = (-20.0, -10.0, 0.0, 10.0, 17.4, 22.4); FAc = {(float(th), ph): P.shoulder_frames(DER, th, ph)[1] for th in TH for ph in PHs}
def cord_clear(Aoff, Bl):
    A = P1 + Aoff; best = 1e9; hits = 0
    for th in TH:
        for ph in PHs:
            Bw = (FAc[(float(th), ph)] @ np.r_[Bl, 1.0])[:3]; t = np.linspace(0, 1, 50)[:, None]; seg = (A[None] * (1 - t) + Bw[None] * t)[4:]
            dm = float(trees[float(th)].query(seg)[0].min()) - 1.25; best = min(best, dm); hits += dm < 3.0
    return best, hits
rng = np.random.default_rng(7); x0 = np.r_[g["A_OFF0"], g["B0"], g["K0"], g["L00"]]; pool = []
def ok(x):
    Aoff, Bl, k, l0 = x[:3], x[3:6], x[6], x[7]; r1, r2, d = residual(Aoff, Bl, k, l0); rr = math.hypot(x[1], x[2]); ang = math.degrees(math.atan2(x[2], x[1])) % 360; rb = math.hypot(x[3], x[4])
    good = (r1.max() <= 3.4 and r2.max() <= 3.4 and 70 <= rr <= 110 and 60 <= ang <= 250 and d.max() <= 200 and k * (d.max() - l0) <= 130 and l0 <= d.min() - 2 and 36 <= rb <= 52 and 100 <= x[5] <= 160 and (d.max() - d.min()) <= 140)
    return good, float(r1.max()), float(r2.max())
cur = x0.copy(); cands = []
for it in range(4):
    sig = np.array([8, 12, 12, 6, 6, 8, 0.08, 8.0]) * (0.7 ** it)
    for _ in range(4000):
        x = cur + rng.normal(0, 1, 8) * sig; good, a, b = ok(x)
        if good: cands.append((x, a, b))
    if cands:
        sel = sorted(cands, key=lambda c: c[1])[:150]; best = (None, -1e9, None)
        for x, a, b in sel:
            cc, h = cord_clear(x[:3], x[3:6])
            if cc > best[1]: best = (x, cc, (a, b, h))
        cur = best[0]; print("round", it, "cands", len(cands), "best cord clearance", round(best[1], 1), "S1p/S2/hits", [round(v, 2) for v in best[2][:2]], best[2][2], flush=True)
x = cur; cc, h = cord_clear(x[:3], x[3:6]); r1, r2, d = residual(x[:3], x[3:6], x[6], x[7])
res = dict(cord_clear_design=dict(A_off=x[:3].round(2).tolist(), B_loc=x[3:6].round(2).tolist(), k_cord=round(float(x[6]), 4), L0=round(float(x[7]), 2), S1p=round(float(r1.max()), 2), S2=round(float(r2.max()), 2),
           SF_S1p=round(8.3 / float(r1.max()), 2), SF_S2=round(8.3 / float(r2.max()), 2), cord_len=[round(float(d.min()), 1), round(float(d.max()), 1)], F_cord_max_N=round(float(x[6] * (d.max() - x[7])), 1), cord_min_clear_to_S1p_group_mm=round(cc, 1), poses_below_3mm=int(h), poses=len(TH) * len(PHs)))
res["rev2_cord_for_reference"] = dict(zip(("min_clear_mm", "poses_below_3mm"), cord_clear(g["A_OFF0"], g["B0"])))
# annulus around the S1' axis: parts of BASE that live within r 40..70 of the axis, with x range
rows = []
for k, p in E.BASE.items():
    if k.startswith("_") or "mesh" not in p: continue
    v = p["mesh"].vertices; r = np.hypot(v[:, 1] - DER["y1"], v[:, 2] - DER["z1"]); m = r < 75
    if m.sum() > 5: rows.append(dict(part=k, group=p.get("group"), x_range=[round(float(v[m, 0].min()), 1), round(float(v[m, 0].max()), 1)], r_range=[round(float(r[m].min()), 1), round(float(r[m].max()), 1)]))
res["parts_near_S1p_axis"] = rows; res["geometry"] = dict(x0=DER["x0"], XB=DER["XB"], XO=DER["XO"], PX=DER["PX"])
json.dump(res, open(os.path.join(OUT, "mj11_balancer_alt.json"), "w"), indent=1, default=float); print(json.dumps(res, default=float)[:3000])
