import sys, os, json, time; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, j1r2_bal_model as M, j1r2_params as P
from scipy.optimize import differential_evolution, minimize
g = 9.81; poses = M.grid_poses(); N = len(poses)
FAs = np.zeros((N, 4, 4)); S = np.zeros((N, 3)); G1 = np.zeros(N); G2 = np.zeros(N)
for i, p in enumerate(poses):
    F0, FA, s, a, d = P.shoulder_frames(M.D0, p[0], p[1]); FAs[i] = FA; S[i] = s
    t1, t2 = M.grav_moments(*p); G1[i] = t1 * 1e-3; G2[i] = t2 * 1e-3            # N m
P1 = M.P1
def spring(A, Bl, k):
    B = np.einsum("nij,j->ni", FAs[:, :3, :3], Bl) + FAs[:, :3, 3]; F = k * (A[None] - B); r = (B - P1) * 1e-3
    mom = np.cross(r, F); return mom[:, 0], np.einsum("ni,ni->n", mom, S)             # N m
def resid_pad(x):                                   # single pad-anchored spring (D2)
    A = P1 + np.array(x[0:3]); Bl = np.array(x[3:6]); k = x[6]; m1, m2 = spring(A, Bl, k); return np.abs(G1 + m1), np.abs(G2 + m2), A, Bl, k
def cost(x): r1, r2, *_ = resid_pad(x); return max(r1.max(), r2.max())
t0 = time.time(); out = {}
print("unbalanced", abs(G1).max(), abs(G2).max())
# D1: decoupled.  S2 yoke-anchored K2 sin(phi) (moment on the arm only about s, nothing on S1');  S1' pad-yoke K1 sin(th+d0)
th = np.array([p[0] for p in poses]); ph = np.array([p[1] for p in poses])
best2 = min(((np.abs(G2 + K * np.sin(np.radians(ph))).max()), K) for K in np.arange(0, 9, .05)) if False else min(((np.abs(G2 - K * np.sin(np.radians(ph))).max()), K) for K in np.arange(-9, 9, .05)) 
best1 = min(((np.abs(G1 - K * np.sin(np.radians(th + d0))).max()), K, d0) for K in np.arange(0, 9, .1) for d0 in np.arange(-180, 180, 5))
print("D1 S2 yoke-anchored", best2, " S1' phase spring", best1)
out["D1"] = dict(S2_residual=float(best2[0]), K2=float(best2[1]), S1p_residual=float(best1[0]), K1=float(best1[1]), phase_deg=float(best1[2]))
# D2: single pad-anchored spring: A offset from P1 (mm) world, B local (mm), k N/mm.  bounds: A within 130 mm, B within the arm 20..300 along z, |x,y|<80
bnds = [(-90, 40), (-100, 100), (20, 130), (-60, 60), (-60, 60), (20, 300), (0.02, 1.5)]
r = differential_evolution(cost, bnds, seed=1, maxiter=120, popsize=24, tol=1e-6, polish=True, updating="immediate", workers=1)
x = r.x; r1, r2, A, Bl, k = resid_pad(x)
print("D2 single pad-anchored: cost", r.fun, "S1'", r1.max(), "S2", r2.max(), "x", np.round(x, 3), time.time() - t0)
out["D2"] = dict(x=[float(v) for v in x], S1p_residual=float(r1.max()), S2_residual=float(r2.max()), A_off_mm=[float(v) for v in x[:3]], B_local_mm=[float(v) for v in x[3:6]], k_N_per_mm=float(x[6]))
# D3: pad-anchored spring for BOTH + trim on S1' (phase spring) : start from D2
def cost3(y):
    x = y[:7]; K1, d0 = y[7], y[8]; r1, r2, *_ = resid_pad(x); m1, m2 = spring(P1 + np.array(x[:3]), np.array(x[3:6]), x[6])
    t1 = np.abs(G1 + m1 - K1 * np.sin(np.radians(th + d0))); return max(t1.max(), r2.max())
r3 = minimize(cost3, np.r_[x, 0.5, 0.0], method="Nelder-Mead", options=dict(maxiter=4000, xatol=1e-3, fatol=1e-4))
print("D3 pad spring + S1' trim:", r3.fun, np.round(r3.x, 3))
out["D3"] = dict(y=[float(v) for v in r3.x], resid=float(r3.fun))
# worst poses of D2
i1 = int(np.argmax(r1)); i2 = int(np.argmax(r2)); out["D2"]["worst1"] = [float(v) for v in poses[i1]]; out["D2"]["worst2"] = [float(v) for v in poses[i2]]
json.dump(out, open("/home/claude/ts/stl/j1r2_bal/search.json", "w"), indent=1)
