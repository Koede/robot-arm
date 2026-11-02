"""J1 rev 2 balancer, rev 2 (2026-11-02): re-fit at the chosen pad (y1,z1)=(+10,-30) with a REMOTE spring (cord over a world-fixed sheave A, spring tube on the paraspinal rail).
Analytic model: torque of a cord from arm-lug B to world-fixed sheave A: moment about P1 = (B-P1) x T u(A-B) (T = k max(|AB|-L0,0)).  Clearance of the cord line A-B to analytic stage primitives."""
import sys, os, json, time; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, j1r2_params as P, j1r2_bal_model as M
from scipy.optimize import differential_evolution
g = 9.81; RATED = 8.3
PAD = (10.0, -30.0, 22.0)
D = P.derive(*PAD); M.D0 = D; M.P1 = np.array([D["PX"], D["y1"], D["z1"]]); P1 = M.P1
XB, XO, PX, x0 = D["XB"], D["XO"], D["PX"], D["x0"]; y1, z1 = D["y1"], D["z1"]
TS, TA, AKR, AKL = P.TS, P.T_ARM, P.AKR, P.AKL
TH = np.arange(-95, 35.1, 10.0); PH = np.append(np.arange(-30, 88.1, 14.0), 88.0)
poses = [(t, p, E, S3, pay) for t in TH for p in PH for E in (0, 45, 90, 145) for S3 in (-70, 0, 90) for pay in (0.0, 0.5)]
def grav_grid(TH=TH, PH=PH):
    u1 = np.zeros((len(TH), len(PH), 4, 3, 2)); u2 = np.zeros_like(u1)
    for i, t in enumerate(TH):
        for j, p in enumerate(PH):
            for a, E in enumerate((0, 45, 90, 145)):
                for b, S3 in enumerate((-70, 0, 90)):
                    for c, pay in enumerate((0.0, 0.5)):
                        t1, t2 = M.grav_moments(t, p, E, S3, pay); u1[i, j, a, b, c] = t1 * 1e-3; u2[i, j, a, b, c] = t2 * 1e-3
    return u1, u2
def frames(TH, PH):
    FA = np.zeros((len(TH), len(PH), 4, 4)); F0 = np.zeros_like(FA); S = np.zeros((len(TH), len(PH), 3))
    for i, t in enumerate(TH):
        for j, p in enumerate(PH):
            f0, fa, s, a, d = P.shoulder_frames(D, t, p); F0[i, j] = f0; FA[i, j] = fa; S[i, j] = s
    return F0, FA, S
def spring_moments(A, Bloc, k, L0, FA, S, extra_g=0.0):
    """moments (N m) on the arm about S1' (x) and S2 (s) from the cord A-B (+ weight of extra_g grams at B)."""
    B = FA[..., :3, :3] @ Bloc + FA[..., :3, 3]; v = A - B; n = np.linalg.norm(v, axis=-1); T = k * np.maximum(n - L0, 0.0); F = (T / np.maximum(n, 1e-9))[..., None] * v
    r = (B - P1) * 1e-3; mom = np.cross(r, F)
    if extra_g: mom = mom + np.cross(r, np.array([0, 0, -extra_g * 1e-3 * g]))
    return mom[..., 0], np.einsum("...i,...i->...", mom, S), T, n
# ---------------------------------------------------------------- analytic clearance of points to the stage primitives
def sd_box(p, c, h):
    q = np.abs(p - c) - h; return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)
def sd_cyl(p, axis, c, R, a0, a1):
    ax = np.eye(3)[axis]; rel = p - c; za = rel[..., axis]; rad = np.linalg.norm(rel - za[..., None] * ax, axis=-1)
    dr = rad - R; dz = np.abs(za - 0.5 * (a0 + a1)) - 0.5 * (a1 - a0); return np.linalg.norm(np.maximum(np.stack([dr, dz], -1), 0), axis=-1) + np.minimum(np.maximum(dr, dz), 0)
CUP_X1 = XO + 14.0              # fairing cup top lowered to just above the yoke disc (XO+12): requested change to j1r2_geom (the S2 bar/clamp still sweep through slots)
def clearance(pts, F0inv, FAinv):
    """pts (..., 3) world points of the cord line; returns min distance (mm) to M1 / M2 / fixed primitives over all points, per leading index."""
    ph = lambda Minv: pts @ Minv[..., :3, :3].swapaxes(-1, -2) + Minv[..., :3, 3][..., None, :]
    p0, pa = ph(F0inv), ph(FAinv); out = {}
    out["bar"] = sd_box(p0, np.array([0, 0.5 * (TS - 1.3), 0.0]), np.array([6.0, 0.5 * (TS - 1.3), 26.0]))
    out["S2motor"] = sd_cyl(p0, 1, np.zeros(3), AKR + 4.0, TS - 1.3, TS + AKL)
    out["adapter"] = sd_cyl(pa, 1, np.zeros(3), 40.0, -8.0, 0.0)
    out["bracket"] = np.minimum(sd_box(pa, np.array([0, 6.0, 13.0]), np.array([24.0, 6.0, 53.0])), sd_box(pa, np.array([0, 0, 86.0]), np.array([32.55, 20.05, 26.0])))
    out["collar"] = sd_cyl(pa, 2, np.zeros(3), 38.0, 113.0, 126.0)
    out["tube"] = sd_cyl(pa, 2, np.zeros(3), 32.0, 113.0, 190.0)
    out["ball"] = np.linalg.norm(pa - np.array([0, 0, 230.0]), axis=-1) - 44.0
    c1 = np.array([0.0, y1, z1])
    out["disc"] = sd_cyl(pts, 0, c1, 47.0, XO, XO + 12.0)
    out["S1motor"] = sd_cyl(pts, 0, c1, AKR, XB, XO)
    out["cup"] = sd_cyl(pts, 0, c1, 66.0, x0 - 6.0, CUP_X1)
    return {k: v.min(-1) for k, v in out.items()}
if __name__ == "__main__":
    t0 = time.time(); u1, u2 = grav_grid(); print("gravity grid", u1.shape, round(time.time() - t0, 1), "s; unbalanced max", abs(u1).max().round(2), abs(u2).max().round(2))
    np.savez(os.path.join(L.ROOT, "stl", "j1r2_bal", "grav_grid_y10.npz"), u1=u1, u2=u2)
