import sys, os, json, time; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, j1r2_bal2 as B2, clib as L
from scipy.optimize import differential_evolution
D = B2.D; P1 = B2.P1; TH, PH = B2.TH, B2.PH
Z = np.load(os.path.join(L.ROOT, "stl", "j1r2_bal", "grav_grid_y10.npz")); u1, u2 = Z["u1"], Z["u2"]
F0, FA, S = B2.frames(TH, PH); F0inv = np.linalg.inv(F0); FAinv = np.linalg.inv(FA)
IT = np.where((TH >= -75) & (TH <= 25))[0]; IP = np.where((PH >= -16) & (PH <= 56))[0]; CL_IDX = np.ix_(IT, IP)   # clearance gate region = the usable passive box (+ margin); beyond it software keep-out applies
tt = np.linspace(0, 1, 26)[None, None, :, None]; EXTRA_G = 52.0
def unpack(x):
    x = x[:8]; ax, ay, az, rho, beta, bd, k, L0 = x; A = P1 + np.array([ax, ay, az]); Bl = np.array([rho * np.cos(np.radians(beta)), rho * np.sin(np.radians(beta)), bd]); return A, Bl, k, L0
def evaluate(x, detail=False):
    A, Bl, k, L0 = unpack(x); s1, s2, T, n = B2.spring_moments(A, Bl, k, L0, FA, S, EXTRA_G)
    K1, dl = (x[8], x[9]) if len(x) > 8 else (0.0, 0.0); s1 = s1 + K1 * np.sin(np.radians(TH + dl))[:, None]
    r1 = u1 + s1[:, :, None, None, None]; r2 = u2 + s2[:, :, None, None, None]; R = max(abs(r1).max(), abs(r2).max())
    Bw = FA[..., :3, :3] @ Bl + FA[..., :3, 3]; pts = A + (Bw - A)[:, :, None, :] * tt                # (nth,nph,26,3)
    cl = B2.clearance(pts[CL_IDX], F0inv[CL_IDX], FAinv[CL_IDX]); dmin = min(v.min() for v in cl.values())
    rr = np.hypot(x[1], x[2]); pen = 0.0
    pen += 2.0 * max(0.0, 4.0 - dmin) + 0.05 * max(0.0, T.max() - 110.0)
    pen += 0.2 * max(0.0, 72.0 - rr) + 0.2 * max(0.0, rr - 105.0) + 0.1 * max(0.0, (P1[2] + x[2]) - 70.0)
    pen += 0.3 * max(0.0, 12.0 - (n.min() - L0)) * 0 + 0.001 * max(0.0, n.max() - 260.0)
    if detail: return dict(R=float(R), dmin=float(dmin), Tmax=float(T.max()), nmin=float(n.min()), nmax=float(n.max()), cl={k_: round(float(v.min()), 1) for k_, v in cl.items()}, rr=float(rr), SF=float(8.3 / R))
    return R + pen
bounds = [(16, 60), (-110, 110), (-110, 110), (44, 52), (0, 360), (114, 125), (0.2, 2.0), (0, 150), (0.0, 8.0), (-180.0, 180.0)]
if __name__ == "__main__":
    best = []
    for seed in range(int(sys.argv[1]) if len(sys.argv) > 1 else 4):
        t0 = time.time(); r = differential_evolution(evaluate, bounds, seed=seed, popsize=12, maxiter=140, tol=1e-6, mutation=(0.5, 1.0), recombination=0.8, polish=False)
        d = evaluate(r.x, True); best.append(dict(seed=seed, f=float(r.fun), x=[float(v) for v in r.x], **d)); print(seed, round(time.time() - t0), "s", round(r.fun, 3), json.dumps(d)[:400], flush=True)
    json.dump(best, open(os.path.join(L.ROOT, "stl", "j1r2_bal", "de_pad_y10_box.json"), "w"), indent=1)
