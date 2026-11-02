import sys, os, json, time; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, j1r2_bal_model as M
exec(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "j1r2_bal_search.py")).read().split("def resid_pad")[0].split("t0 = time.time()")[0])
from scipy.optimize import differential_evolution
def spring2(A, Bl, k, L0):
    B = np.einsum("nij,j->ni", FAs[:, :3, :3], Bl) + FAs[:, :3, 3]; v = A[None] - B; d = np.linalg.norm(v, axis=1); F = (k * (d - L0))[:, None] * v / d[:, None]
    r = (B - P1) * 1e-3; mom = np.cross(r, F); return mom[:, 0], np.einsum("ni,ni->n", mom, S), d
def f(x, ret=False):
    A = P1 + np.array(x[0:3]); Bl = np.array(x[3:6]); k, L0 = x[6], x[7]; m1, m2, d = spring2(A, Bl, k, L0)
    r1 = np.abs(G1 + m1); r2 = np.abs(G2 + m2); pen = 0.0
    pen += max(0, np.hypot(x[1], x[2]) - 62.0) * 0.2                    # anchor must sit on the pad rim (r <= 62 about the S1' axis)
    pen += max(0, d.max() - 150.0) * 0.05 + max(0, (d.max() - d.min()) - 90.0) * 0.05      # cord length / stroke
    pen += max(0, L0 - d.min() + 2.0) * 0.5                              # spring must stay in tension (compression) over the whole range
    if ret: return r1, r2, d, pen
    return max(r1.max(), r2.max()) + pen
bnds = [(-45, 10), (-62, 62), (-62, 62), (-40, 40), (-30, 30), (20, 120), (0.05, 3.0), (0, 80)]
best = None
for seed in (1, 2, 3):
    r = differential_evolution(f, bnds, seed=seed, maxiter=150, popsize=24, tol=1e-6, polish=True)
    if best is None or r.fun < best.fun: best = r
    print(seed, r.fun, flush=True)
x = best.x; r1, r2, d, pen = f(x, True)
res = dict(x=[float(v) for v in x], S1p_residual=float(r1.max()), S2_residual=float(r2.max()), cord_len_min=float(d.min()), cord_len_max=float(d.max()), penalty=float(pen),
           A_off_mm=[float(v) for v in x[:3]], B_local_mm=[float(v) for v in x[3:6]], k_N_per_mm=float(x[6]), L0_mm=float(x[7]))
print(json.dumps(res, indent=1)); json.dump(res, open("/home/claude/ts/stl/j1r2_bal/search2.json", "w"), indent=1)
