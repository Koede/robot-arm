"""Claw hand kinematic synthesis, C3 (three-fold) punch-ring claw. Session 2026-10-13g. Replaces the planar rev 13d/e synthesis.
Planar RADIAL-plane finger (all three fingers identical, 120 deg apart about the hand axis). Finger frame (a, b): origin at the proximal pivot B,
b = +z (hand axis direction), a = radially INWARD (toward the hand axis). theta1 = proximal angle from +b toward +a (inward = positive); theta2 = distal
angle relative to the proximal phalanx.
Distal coupling = four-bar B-J-R-Q (ground pin Q on the clevis bracket, coupling link lc, tab R on the distal phalanx).
Drive = punch ring (nut) on the hand axis: ring lug at radius RL, height zn; drive link LD to point P on the proximal phalanx
(P = dp along the phalanx + np outward-normal).  zn(theta1) is closed-form.  Outputs claw_synth.json.  Pure numpy/scipy."""
import numpy as np, json, os, math
from scipy.optimize import differential_evolution
# layout / linkage (from the numeric search c3_opt, rounded)
RB, ZB = 30.5, 519.4             # proximal pivot radius from the hand axis, height (hand frame z)
L1, L2 = 38.5, 14.5              # B->J, J->tip
TH1_O, TH1_C = -8.0, 15.5        # proximal angle open / closed, soft (deg)
TH2_O, TH2_C = -4.0, 48.7        # distal angle open / closed (deg)  (target law, four-bar fitted below)
DP, NP, LD, RL = 6.6, 7.5, 30.0, 13.0     # drive-link point on phalanx (along, outward-normal), link length, ring lug radius

def unit(th): return np.array([np.sin(th), np.cos(th)])          # (a, b) direction at angle th from +b toward +a
def fourbar(th1, P):
    qa, qb, lc, ry, rn = P
    J = L1 * unit(th1); Q = np.array([qa, qb]); d = J - Q
    rho = math.hypot(ry, rn); psi = math.atan2(rn, ry)
    k = (lc ** 2 - d @ d - rho ** 2) / 2.0; dn = math.hypot(*d)
    if dn < 1e-9: return None
    cv = k / (rho * dn)
    if abs(cv) > 1: return None
    base = math.atan2(d[0], d[1])
    return [base + math.acos(cv) - psi - th1, base - math.acos(cv) - psi - th1]
def pick(th1, P, prev):
    s = fourbar(th1, P)
    if s is None: return None
    s = [((x + math.pi) % (2 * math.pi)) - math.pi for x in s]
    return min(s, key=lambda x: abs(x - prev))
def curve(P, n=41):
    t1 = np.radians(np.linspace(TH1_O, TH1_C, n)); out = []; prev = math.radians(TH2_O)
    for t in t1:
        v = pick(t, P, prev)
        if v is None: return None
        out.append(v); prev = v
    return t1, np.array(out)
def cost(x):
    P = tuple(x); r = curve(P)
    if r is None: return 1e3
    t1, t2 = r; qa, qb, lc, ry, rn = P
    tgt = np.radians(TH2_O + (TH2_C - TH2_O) * (np.degrees(t1) - TH1_O) / (TH1_C - TH1_O))
    err = np.degrees(np.sqrt(np.mean((t2 - tgt) ** 2))); pen = 0.0
    if np.any(np.diff(t2) <= 0): pen += 50
    for th1, th2 in zip(t1, t2):
        J = L1 * unit(th1); phi = th1 + th2
        R = J + ry * unit(phi) + rn * unit(phi + np.pi / 2); Q = np.array([qa, qb]); seg = R - Q
        tt = np.clip(-Q @ seg / (seg @ seg), 0, 1); dmin = np.linalg.norm(Q + tt * seg)
        if dmin < 9.0: pen += (9.0 - dmin) * 3                       # keep the link off the pivot hub
        lev = R - J; ang = math.degrees(math.acos(np.clip(abs(lev @ seg) / (np.linalg.norm(lev) * np.linalg.norm(seg) + 1e-9), 0, 1)))
        if ang < 25: pen += (25 - ang) * 0.5                          # transmission angle
    if math.hypot(qa, qb) < 16: pen += (16 - math.hypot(qa, qb)) * 2
    if qa > -4: pen += (qa + 4) * 2                                   # Q on the outer side of the pivot
    return err + pen
def nut_z(th1_deg):
    """ring lug height zn for proximal angle theta1 (deg), hand frame; closed form from |P - lug| = LD."""
    th = math.radians(th1_deg); B = np.array([RB, ZB])
    e1 = np.array([-math.sin(th), math.cos(th)]); n1 = np.array([math.cos(th), math.sin(th)])        # (r, z); r outward
    P = B + DP * e1 + NP * n1; dr = RL - P[0]
    return float(P[1] + math.sqrt(LD ** 2 - dr ** 2)), P.tolist()
if __name__ == "__main__":
    bounds = [(-40, -5), (-25, 50), (8, 80), (-12, 32), (-28, 28)]        # qa, qb, lc, ry, rn
    best = None
    for seed in range(1):
        r = differential_evolution(cost, bounds, seed=seed, maxiter=150, popsize=20, tol=1e-9, polish=True)
        print(seed, round(r.fun, 3), np.round(r.x, 2))
        if best is None or r.fun < best.fun: best = r
    P = tuple(best.x); t1, t2 = curve(P)
    print("best", np.round(best.x, 3), "cost", round(best.fun, 3))
    rows = []
    for a, b in zip(np.degrees(t1)[::5], np.degrees(t2)[::5]):
        zn, Pp = nut_z(a); rows.append((a, b, zn)); print(f"th1 {a:6.1f}  th2 {b:6.1f}  nut_z {zn:6.2f}")
    json.dump(dict(RB=RB, ZB=ZB, L1=L1, L2=L2, TH1_O=TH1_O, TH1_C=TH1_C, TH2_O=TH2_O, TH2_C=TH2_C, DP=DP, NP=NP, LD=LD, RL=RL,
                   qa=P[0], qb=P[1], lc=P[2], ry=P[3], rn=P[4], cost=float(best.fun)),
              open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "claw_synth.json"), "w"), indent=1)
