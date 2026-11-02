"""J1 rev 2 gravity model + balancer family search (analysis helper for j1r2_balancer.py).  N, m, deg.  Right arm, robot frame."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, itertools
import j1r2_params as P
g = 9.81
D0 = P.derive()
# --- mass model beyond S2 (local arm frame x=w, y=s, z=d, origin on the arm axis where it crosses the S2 axis).  Sum 1489 g, CG(E=0, S3=0) ~ 247 mm (project: 1489 g, CG 249)
SEG = [  # (name, mass g, local position at E=0, rotates_with_E)
    ("S3 stage", 213.0, (0, 0, 90.0), False), ("upper arm", 377.0, (0, 0, 170.0), False), ("elbow", 330.0, (0, 0, 230.0), False),
    ("forearm+wrist", 380.0, (0, 57.0, 330.0), True), ("hand", 189.0, (0, 57.0, 480.0), True)]
TIP = (0, 57.0, 230.0 + 345.0)       # claw tip (payload point), rotates with E
ELB = np.array([0, 0, 230.0])
def rotE(v, E):                       # about local y through the elbow, E>0 swings toward -x_loc
    c, s = np.cos(np.radians(E)), np.sin(np.radians(E)); d = np.asarray(v, float) - ELB
    return ELB + np.array([d[0] * c - d[2] * s, d[1], d[0] * s + d[2] * c]) * np.array([1, 1, 1]) if False else ELB + np.array([d[0] * c + d[2] * (-s), d[1], d[0] * s + d[2] * c])
def rotS3(v, S3):
    c, s = np.cos(np.radians(S3)), np.sin(np.radians(S3)); v = np.asarray(v, float); return np.array([v[0] * c - v[1] * s, v[0] * s + v[1] * c, v[2]])
def arm_points(E, S3, pay_kg):
    pts = []
    for n, m, p, re in SEG:
        q = rotE(p, E) if re else np.asarray(p, float); pts.append((m * 1e-3, rotS3(q, S3)))
    if pay_kg: pts.append((pay_kg, rotS3(rotE(TIP, E), S3)))
    return pts
def cg_arm(E=0, S3=0, pay=0.0):
    pts = arm_points(E, S3, pay); M = sum(m for m, _ in pts); return M, sum(m * p for m, p in pts) / M
def world(th, ph, loc):
    """local arm-frame point (mm) -> world, for the J1 frames at given D."""
    F0, FA, s, a, d = P.shoulder_frames(D0, th, ph); return (FA @ np.r_[loc, 1.0])[:3]
P1 = np.array([D0["PX"], D0["y1"], D0["z1"]])
M1_MASSES = [(0.621, P.TS + P.AKL / 2), (0.150, 0.03)]           # (kg, distance along s from the S1' axis point)
def grav_moments(th, ph, E, S3, pay, extra_arm=(), extra_m1=()):
    """moments of gravity (N m) about the S1' axis (x) and the S2 axis (s) as seen by each joint: returns (tau1, tau2).  extra_*: [(kg, local mm xyz)] / [(kg, s-distance mm, off-axis mm (x,z world-ish) )]"""
    F0, FA, s, a, d = P.shoulder_frames(D0, th, ph); W = np.array([0, 0, -g])
    t1 = 0.0; t2 = 0.0
    for m, loc in list(arm_points(E, S3, pay)) + [(m, np.asarray(l, float)) for m, l in extra_arm]:
        p = (FA @ np.r_[loc, 1.0])[:3]; r = p - P1; mom = np.cross(r, m * W); t1 += mom[0]; t2 += mom @ s
    for m, ds in M1_MASSES + [(m, ds) for m, ds in extra_m1]:
        p = P1 + ds * 1e0 * s; r = p - P1; mom = np.cross(r, m * W); t1 += mom[0]       # M1 masses do not load S2
    return t1, t2
def spring_moments(th, ph, A_w, B_loc, k_N_per_mm):
    """zero-length spring from world point A (fixed on the pad) to B (arm local mm): moments on the arm about x-axis(S1') and s (S2)."""
    F0, FA, s, a, d = P.shoulder_frames(D0, th, ph); B = (FA @ np.r_[B_loc, 1.0])[:3]; F = k_N_per_mm * (np.asarray(A_w) - B)  # N (mm * N/mm)
    r = (B - P1) * 1e-3; mom = np.cross(r, F); return mom[0], mom @ s
def grid_poses(coarse=False):
    TH = np.arange(-95, 35.1, 10 if coarse else 5.0); PH = np.arange(-30, 88.1, 10 if coarse else 6.0)
    return [(t, p, E, S3, pay) for t in TH for p in PH for E in (0, 45, 90, 145) for S3 in (-70, 0, 90) for pay in (0.0, 0.5)]
