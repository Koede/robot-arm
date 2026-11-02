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
    rr = np.hypot(x[1], x[2]); ang = np.degrees(np.arctan2(x[2], x[1])) % 360; pen += max(0, 105.0 - ang) * 1.0 + max(0, ang - 230.0) * 1.0; pen += max(0, rr - 110.0) * 0.5 + max(0, 70.0 - rr) * 0.5                    # anchor must sit on the pad rim (r <= 62 about the S1' axis)
    pen += max(0, d.max() - 200.0) * 0.05 + max(0, (d.max() - d.min()) - 140.0) * 0.05      # cord length / stroke
    pen += max(0, L0 - d.min() + 2.0) * 0.5
    Fm = x[6] * (d.max() - L0); pen += max(0, Fm - 260.0) * 0.02                              # spring must stay in tension (compression) over the whole range
    if ret: return r1, r2, d, pen
    return max(r1.max(), r2.max()) + pen

from scipy.optimize import minimize
Aoff=np.array([-10.018,-18.148*1.086,67.623*1.086]); print("A radius", np.hypot(Aoff[1],Aoff[2]))
def f2(y, ret=False):
    x=np.r_[Aoff,y]; return f(x, ret)
y0=np.array([-12.368,-29.999,114.636,0.8819,61.897])
best=None
for sc in (1.0,1.02,0.98):
    r=minimize(f2, y0*np.array([1,1,sc,1,1]), method="Nelder-Mead", options=dict(maxiter=3000,xatol=1e-3,fatol=1e-5))
    if best is None or r.fun<best.fun: best=r
x=np.r_[Aoff,best.x]; r1,r2,d,pen=f(x,True); print("cost",best.fun,"pen",pen,"S1'",r1.max(),"S2",r2.max(),"cord",d.min(),d.max(),"y",np.round(best.x,3), "F",best.x[3]*(d.max()-best.x[4]))
