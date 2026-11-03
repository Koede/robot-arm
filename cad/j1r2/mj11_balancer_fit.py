"""MJ11 balancer refit (2026-11-03): re-optimises the pad-anchored zero-length-type spring (J1 rev 2 design: cord A on the pad -> B on the upper arm, F = k (|AB| - L0)) for the REAL MJ11 workspace instead of the J1 grid,
and fits the S1'-only alternatives (coaxial flat spiral spring) for comparison.  The A / B / k / L0 values are relative to P1 (translation invariant), so the J1 rev 2 solution carries over to pad (-60, -190); what changes is the workspace (smaller box,
S2 <= 17.4) and the packaging.  usage: J1R2_ROOT=<root> python3 mj11_balancer_fit.py BOX|REACH|FULL [light]  -> stl/j1r2_mounts/MJ11/bal_fit_<WS>_<mode>.json"""
import os, sys, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import j1r2_params as P, j1r2_bal_model as BM, mj11_torque as T
from scipy.optimize import differential_evolution, minimize
A_OFF0 = np.array([-10.018, -19.708, 73.434]); B_LOC0 = np.array([-15.629, -42.256, 118.385]); K0, L00 = 0.7520, 54.587
class Fit:
    def __init__(s, ws):
        cells, _ = T.ws_cells(); cl = cells[ws]; s.cl = cl; FA, S = T.frames(cl); idx, G1, G2, Ei = [], [], [], []
        for pay in (0.0, 0.5):
            for (E, S3) in T.ECONF:
                g1, g2 = T.grav(FA, S, T.local_masses(E, S3, pay)); G1.append(g1); G2.append(g2); idx.append(np.arange(len(cl)))
        s.G1 = np.concatenate(G1); s.G2 = np.concatenate(G2); ix = np.concatenate(idx); s.FA = FA[ix]; s.S = S[ix]; s.P1 = BM.P1
    def spring(s, A, Bl, k, L0):
        B = np.einsum("nij,j->ni", s.FA[:, :3, :3], Bl) + s.FA[:, :3, 3]; v = A[None] - B; d = np.linalg.norm(v, axis=1); F = (k * (d - L0))[:, None] * v / d[:, None]; r = (B - s.P1) * 1e-3
        m = np.cross(r, F); return m[:, 0], np.einsum("ni,ni->n", m, s.S), d, F
    def cost(s, x, mode="best", ret=False, sector=(105.0, 230.0)):
        Ao = np.array(x[0:3]); Bl = np.array(x[3:6]); k, L0 = x[6], x[7]; m1, m2, d, F = s.spring(s.P1 + Ao, Bl, k, L0); r1 = np.abs(s.G1 + m1); r2 = np.abs(s.G2 + m2); pen = 0.0
        rr = np.hypot(Ao[1], Ao[2]); ang = np.degrees(np.arctan2(Ao[2], Ao[1])) % 360.0; pen += max(0, sector[0] - ang) + max(0, ang - sector[1]) + max(0, rr - 110.0) * 0.5 + max(0, 70.0 - rr) * 0.5
        rb = np.hypot(Bl[0], Bl[1] + 0.0); pen += max(0, rb - 50.0) * 0.3 + max(0, 38.0 - rb) * 0.3
        pen += max(0, d.max() - 200.0) * 0.05 + max(0, (d.max() - d.min()) - 140.0) * 0.05 + max(0, L0 - d.min() + 2.0) * 0.5
        Fm = float(np.linalg.norm(F, axis=1).max()); pen += max(0, Fm - 260.0) * 0.02
        res = max(r1.max(), r2.max())
        if ret: return dict(r1=float(r1.max()), r2=float(r2.max()), d_min=float(d.min()), d_max=float(d.max()), F_cord_max=Fm, pen=float(pen))
        if mode == "light": return Fm * 0.02 + 10.0 * max(0.0, res - 3.5) + pen
        return res + pen
BND = [(-40, 10), (-110, 110), (-110, 110), (-40, 40), (-70, 70), (60, 190), (0.2, 3.0), (20, 120)]
if __name__ == "__main__":
    ws = sys.argv[1]; mode = sys.argv[2] if len(sys.argv) > 2 else "best"; t0 = time.time(); F = Fit(ws); x0 = np.r_[A_OFF0, B_LOC0, K0, L00]
    base = F.cost(x0, ret=True); print(ws, mode, "J1 rev 2 design on this workspace:", base, flush=True)
    rng = np.random.default_rng(1); pop = 24; init = np.array([[rng.uniform(a, b) for a, b in BND] for _ in range(pop * 8)]); init[0] = x0
    for i in range(1, 12): init[i] = x0 * (1 + 0.04 * rng.standard_normal(8))
    r = differential_evolution(lambda x: F.cost(x, mode), BND, init=init, maxiter=400, tol=1e-7, seed=2, polish=False, mutation=(0.4, 1.0), recombination=0.8, workers=1)
    r2 = minimize(lambda x: F.cost(x, mode), r.x, method="Nelder-Mead", options=dict(maxiter=4000, xatol=1e-4, fatol=1e-7)); x = r2.x if r2.fun < r.fun else r.x
    fin = F.cost(x, ret=True); out = dict(ws=ws, mode=mode, x=dict(A_off=x[0:3].tolist(), B_loc=x[3:6].tolist(), k=float(x[6]), L0=float(x[7])), final=fin, J1rev2_on_ws=base, secs=round(time.time() - t0, 1))
    print(json.dumps(out, indent=1)); json.dump(out, open(os.path.join(T.OUTD, f"bal_fit_{ws}_{mode}.json"), "w"), indent=1)
