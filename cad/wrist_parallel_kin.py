"""Parallel wrist feasibility (spherical five-bar), right arm, forearm frame, deg.

Question: can two motors fixed to the forearm (the MW1 frame) drive the claw axis through the same range that the serial
W2 (about y) + W3 (about x) wrist reaches, with no branch loss and no singularity inside the operating range?

Mechanism (all joint axes meet at the wrist centre, so every point is a unit vector):
  ground   J0 - J4          (forearm, fixed)
  crank 1  J0 -> J1         arc a1, driven at J0 by motor 1 (angle th1 about J0)
  float 1  J1 -> J2         arc b1
  float 2  J2 -> J3         arc b2   (the claw is rigidly fixed to this link)
  crank 2  J3 -> J4         arc a2, driven at J4 by motor 2 (angle th2 about J4)
Claw axis = unit(J2 x J3): the normal of the floater-2 great circle. Its roll about the claw axis is not controlled here
(W1 is the forearm roll); this script checks direction only.

Serial reference: claw axis in the MW1 frame at (W2, W3) from arm_kin.frames, same convention as the rest of the repo.

Outputs stl/wrist_parallel/kin.json (new folder; no existing file is touched).
Usage: python3 wrist_parallel_kin.py [--quick]
"""
import os, sys, json, itertools, numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import differential_evolution
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, rom_limits as rl
OUT = os.path.join(HERE, "..", "stl", "wrist_parallel"); os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------- serial reference (direction set)
def serial_axis(w2, w3):
    q = dict(S1=0, S2=0, S3=0, E=0, W1=0, W2=w2, W3=w3, G=50)
    T = ak.frames(q)
    M = np.linalg.inv(T["MW1"]) @ T["MW3"]
    return (M[:3, :3] @ np.array([0, 0, 1.0]))

def targets(step=5.0, margin=0.0):
    lo2, hi2 = rl.SOFT["W2"]; lo3, hi3 = rl.SOFT["W3"]
    pts = []
    for w2 in np.arange(lo2 + margin, hi2 - margin + 1e-9, step):
        for w3 in np.arange(lo3 + margin, hi3 - margin + 1e-9, step):
            pts.append(serial_axis(w2, w3))
    return np.array(pts), [(w2, w3) for w2 in np.arange(lo2, hi2 + 1e-9, step) for w3 in np.arange(lo3, hi3 + 1e-9, step)]

# ---------------------------------------------------------------- spherical five-bar
def unit(v): return v / np.linalg.norm(v, axis=-1, keepdims=True)
def on_sphere_point(J, ang, t):
    """point at arc distance ang from unit J, azimuth t (array) about J."""
    J = unit(np.asarray(J, float)); a = np.cross(J, [0.3, 0.5, 0.8]); a = unit(a); b = np.cross(J, a)
    return np.cos(ang) * J + np.sin(ang) * (np.cos(t)[..., None] * a + np.sin(t)[..., None] * b)

def ground_pts(p):
    """p = (az0, pol0, az4, pol4): ground points J0, J4 as unit vectors (spherical coordinates, deg)."""
    def sph(az, pol):
        az, pol = np.radians(az), np.radians(pol)
        return np.array([np.sin(pol) * np.cos(az), np.sin(pol) * np.sin(az), np.cos(pol)])
    return sph(p[0], p[1]), sph(p[2], p[3])

def assemble(params, th1, th2):
    """params = (az0, pol0, az4, pol4, a1, a2, b1, b2) in deg. Returns claw-axis unit vectors (N, 3, both branches) and validity.
    th1, th2: 1-D arrays of crank angles (deg), taken on a grid by the caller."""
    az0, pol0, az4, pol4, a1, a2, b1, b2 = params
    J0, J4 = ground_pts((az0, pol0, az4, pol4))
    th1 = np.radians(th1); th2 = np.radians(th2)
    J1 = on_sphere_point(J0, np.radians(a1), th1)
    J3 = on_sphere_point(J4, np.radians(a2), th2)
    n1, n2 = J1, J3
    d = np.cross(n1, n2); dd = np.einsum("ij,ij->i", d, d)
    ok = dd > 1e-10
    c1, c2 = np.cos(np.radians(b1)), np.cos(np.radians(b2))
    X0 = (c1 * np.cross(n2, d) + c2 * np.cross(d, n1)) / np.where(ok, dd, 1.0)[:, None]
    s2 = 1 - np.einsum("ij,ij->i", X0, X0)
    ok &= s2 >= 0
    s = np.sqrt(np.clip(s2, 0, None))
    dh = unit(d)
    H = []
    for sg in (1.0, -1.0):
        J2 = X0 + sg * s[:, None] * dh
        h = unit(np.cross(J2, n2))                                    # claw axis = normal of the floater-2 circle
        H.append(h)
    return np.stack(H, axis=1), ok, J0, J4

S_MIN = 0.15   # smallest singular value of d(axis)/d(crank) (rad of axis per rad of crank) for a configuration to count

def gain_min(params, T1, T2, h=1e-4):
    """smallest singular value of the 3x2 map (th1, th2) -> claw axis, per branch. Returns (sigma_min, H, ok) with H (N, 2, 3)."""
    H0, ok, _, _ = assemble(params, T1, T2)
    H1, _, _, _ = assemble(params, T1 + np.degrees(h), T2)
    H2, _, _, _ = assemble(params, T1, T2 + np.degrees(h))
    J1 = (H1 - H0) / h; J2 = (H2 - H0) / h
    a = np.einsum("nbi,nbi->nb", J1, J1); b = np.einsum("nbi,nbi->nb", J1, J2); c = np.einsum("nbi,nbi->nb", J2, J2)
    lo = ((a + c) - np.sqrt((a - c) ** 2 + 4 * b * b)) / 2
    return np.sqrt(np.clip(lo, 0, None)), H0, ok

def coverage(params, tgt, grid=121):
    """for each serial target, the smallest angle (deg) to a well-conditioned achieved claw axis (both branches)."""
    t = np.linspace(0, 360, grid, endpoint=False)
    T1, T2 = np.meshgrid(t, t, indexing="ij"); T1 = T1.ravel(); T2 = T2.ravel()
    sig, H, ok = gain_min(params, T1, T2)
    good = ok[:, None] & (sig >= S_MIN)                    # (N, 2)
    pts = H[good]                                          # (M, 3)
    frac_ok = float(good.mean())
    if len(pts) < 10: return np.full(len(tgt), 180.0), frac_ok, 180.0
    dist, _ = cKDTree(pts).query(tgt)
    ang = np.degrees(2 * np.arcsin(np.clip(dist / 2, 0, 1)))
    return ang, frac_ok, float(np.max(ang))

if __name__ == "__main__":
    quick = "--quick" in sys.argv
    TGT, LABELS = targets(step=10.0 if quick else 5.0)
    print(f"serial targets: {len(TGT)} (W2 soft {rl.SOFT['W2']}, W3 soft {rl.SOFT['W3']})")
    bounds = [(0, 360), (10, 170), (0, 360), (10, 170), (20, 90), (20, 90), (30, 120), (30, 120)]

    def cost(p):
        ang, frac_ok, worst = coverage(p, TGT, grid=61 if quick else 91)
        return float(np.mean(np.clip(ang - 3.0, 0, None)) + 2.0 * np.max(np.clip(ang - 3.0, 0, None)) + 20.0 * (1 - frac_ok))

    res = differential_evolution(cost, bounds, seed=2, maxiter=40 if quick else 120, popsize=12, tol=1e-4, polish=False, workers=1)
    p = res.x
    ang, frac_ok, worst = coverage(p, TGT, grid=181)
    out = dict(params=dict(az0=p[0], pol0=p[1], az4=p[2], pol4=p[3], a1=p[4], a2=p[5], b1=p[6], b2=p[7]),
               cost=float(res.fun), n_targets=int(len(TGT)), worst_angle_deg=worst, mean_angle_deg=float(ang.mean()),
               frac_targets_within_3deg=float((ang <= 3.0).mean()), frac_configs_well_conditioned=float(frac_ok), s_min=S_MIN,
               serial_range=dict(W2=rl.SOFT["W2"], W3=rl.SOFT["W3"]),
               note="direction only; roll about the claw axis is not checked; both assembly branches counted")
    json.dump(out, open(os.path.join(OUT, "kin.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))
