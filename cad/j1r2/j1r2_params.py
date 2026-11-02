"""J1 rev 2 (integration): single source of truth for the pancake / side-hung shoulder. Robot frame, mm, deg; right arm (left = mirror in y about y = -150).
S1' : AK70 flat on a back-plane pad, axis +x through (y1, z1).  th1 rotates about +x: arm rest direction a(th1) = (0, -sin th1, cos th1).
S2  : AK70 whose axis s(th1) = (0, cos th1, sin th1) lies in the back plane (turns with S1').  d = a cos(phi) - xhat sin(phi).
S3  : XM540 along the arm axis d.   E / W1 / W2 / W3 / G : distal chain exactly as in cad/arm_kin.py (E axis = s x d ... see dist_frames()).
Baseline park S1 45 / S2 25 <-> th1 = -18.2, phi = 17.4 (same arm direction).  Pole (arm along +-x) is 10.2 deg outside the baseline direction set."""
import numpy as np, sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clib as L
AKR, AKL = 44.5, 50.25
SOFT = dict(S1p=(-95.0, 35.0), S2=(-30.0, 88.0), S3=(-70.0, 90.0), E=(0.0, 145.0), W1=(-80.0, 80.0), W2=(-70.0, 80.0), W3=(-20.0, 30.0))
STOP_MARGIN = 5.0
HARD = {k: (a - STOP_MARGIN, b + STOP_MARGIN) for k, (a, b) in SOFT.items()}
PARK = dict(S1p=-18.2, S2=17.4, S3=0.0, E=45.0, W1=0.0, W2=0.0, W3=0.0)
TS = AKR + 3.0; T_ARM = TS + AKL + 8.0           # S2 motor starts 47.5 along s; arm axis crosses the S2 axis T_ARM = 105.75 along s
PL_T = 6.0                                       # base pad thickness
def derive(y1=-70.0, z1=-35.0, standoff=22.0, extra_sweep=2.3):
    """pad front face x0 etc. from the wearer's back surface (ray cast)."""
    Y, Z, X = L.rear_surface(y1 - 160, y1 + 160, z1 - 160, z1 + 160, 6.0, 3.0)
    fp = (np.hypot(Y - y1, Z - z1) < 60); sw = (np.hypot(Y - y1, Z - z1) < TS + AKL + 8 + 45)
    xs_fp, xs_sw = float(X[fp].max()), float(X[sw].max())
    x0 = max(xs_fp + standoff, xs_sw + extra_sweep); XB = x0 + PL_T; XO = XB + AKL; PX = XO + 6.0
    return dict(y1=y1, z1=z1, x0=x0, XB=XB, XO=XO, PX=PX, xs_fp=xs_fp, xs_sw=xs_sw, standoff=standoff)
def dirJ1(th1, phi):
    t, p = np.radians(th1), np.radians(phi); return np.array([-np.sin(p), -np.sin(t) * np.cos(p), np.cos(t) * np.cos(p)])
def s_of(th): return np.array([0, np.cos(np.radians(th)), np.sin(np.radians(th))])
def a_of(th): return dirJ1(th, 0)
def frame(o, ex, ey, ez):
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = ex, ey, ez, o; return M
def shoulder_frames(D, th1, phi):
    """D = derive(); returns F0 (yoke frame at phi=0: local x=+x, y=s, z=a), FA (arm frame: local x=w=s x d, y=s, z=d, origin on the S2 axis T_ARM along s), s, a, d."""
    P1 = np.array([D["PX"], D["y1"], D["z1"]]); s, a = s_of(th1), a_of(th1); ex0 = np.cross(s, a)
    F0 = frame(P1, ex0, s, a); d = dirJ1(th1, phi); w = np.cross(s, d); FA = frame(P1 + T_ARM * s, w, s, d)
    return F0, FA, s, a, d
