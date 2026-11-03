"""MJ11 balancer: re-optimise the psi-sector solutions with the cord clearance to the S1' group (yoke / AK70 / collar, rotating with th1) and to the arm (beyond 18 mm of B) as a penalty (gate 3 mm + 1 mm margin).  2026-11-03, CONCEPT."""
import os, sys, json, math, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import minimize, differential_evolution
import mj11_balancer as MB, mj_eval as E, j1r2_params as P, j1r2_geom as G
t0 = time.time(); pr = MB.make_problem(MB.SOFT); cells = pr["idx"]
rng = np.random.default_rng(0); sub = lambda X, n: X[rng.choice(len(X), min(n, len(X)), replace=False)]
M1p, M2p = sub(E.MOV1, 1500), sub(E.MOV2, 1500)
T1, T2 = {}, {}
for n, (i, j) in enumerate(cells):
    th, ph = MB.S1V[i], MB.S2V[j]
    if th not in T1: Mx = G.pose_matrix("M1", E.DER, th, 0.0); T1[th] = cKDTree(M1p @ Mx[:3, :3].T + Mx[:3, 3])
    Mm = G.pose_matrix("M2", E.DER, th, ph); T2[(i, j)] = (cKDTree(M2p @ Mm[:3, :3].T + Mm[:3, 3]), Mm)
SEL = [n for n in range(len(cells)) if (cells[n][0] % 2 == 0)]
tt = np.linspace(0, 1, 14)[1:-1, None]
def clear(x):
    A = MB.P1 + np.array(x[0:3]); Bl = np.array(x[3:6]); d1 = d2 = 1e9
    for n in SEL:
        i, j = cells[n]; FA = pr["FA"][n]; Bw = FA[:3, :3] @ Bl + FA[:3, 3]; seg = A + (Bw - A) * tt; seg1 = seg[np.linalg.norm(seg - A, axis=1) > 14.0]
        if len(seg1): d1 = min(d1, float(T1[MB.S1V[i]].query(seg1)[0].min()))
        far = seg[np.linalg.norm(seg - Bw, axis=1) > 18.0]
        if len(far): d2 = min(d2, float(T2[(i, j)][0].query(far)[0].min()))
    return d1, d2
def cost2(x, psi, fcap):
    c = MB.cost(x, pr, psi, fcap, (70.0, 100.0)); d1, d2 = clear(x); return c + 3.0 * max(0.0, 4.0 - d1) + 3.0 * max(0.0, 4.0 - d2)
if __name__ == "__main__":
    base = json.load(open(os.path.join(MB.OUTD, "balancer_pack.json"))); out = []
    for psi, fcap in ((100, 130.0), (110, 130.0), (120, 130.0), (130, 130.0)):
        x0 = np.array([r for r in base if r["psi"] == psi and r["fcap"] == fcap][0]["x"]); best = None
        for k in range(3):
            xs = x0 * (1 + 0.01 * k * rng.standard_normal(len(x0))) if k else x0
            r = minimize(cost2, xs, args=(psi, fcap), method="Nelder-Mead", options=dict(maxiter=500, xatol=1e-3, fatol=1e-4, adaptive=True))
            if best is None or r.fun < best.fun: best = r
        r1, r2, d, Fc = MB.spring_resid(best.x, pr); d1, d2 = clear(best.x)
        row = dict(psi=psi, fcap=fcap, x=[float(v) for v in best.x], S1p_res=float(r1.max()), S2_res=float(r2.max()), SF=float(MB.RATED / max(r1.max(), r2.max())), F_cord_max=float(Fc.max()), cord_min=float(d.min()), cord_max=float(d.max()), cord_vs_S1p_group_mm=round(d1, 1), cord_vs_arm_mm=round(d2, 1))
        row["spring"] = MB.spring_design(best.x[6], d.min(), d.max(), Fc.max(), L0=best.x[7]); out.append(row); print(round(time.time() - t0), json.dumps({k: v for k, v in row.items() if k != "x"}), flush=True)
    json.dump(out, open(os.path.join(MB.OUTD, "balancer_cordfix.json"), "w"), indent=1, default=float)
