"""HAND rev B (2026-10-30b): human-layout micro-servo hand.  Kinematics + chuck (bunched-fingertip pinch) solver.
Hand frame = W3 frame: x palm normal (+x palmar), u = y - D (+u thumb side), z absolute (distal).  Palm centred on the wrist (x 0).
Fingers: MCP flexion theta (servo, crank + rod pulling a palmar lever = flexor), PIP = K_PIP*MCP, DIP = K_DIP*PIP (ground-linked four-bars hidden in a dorsal groove).
Index/ring: shared abduction servo (mirrored) about the palm normal x through the MCP (+psi = adduct toward the middle finger).
Thumb: 2-axis CMC (swing about x, lift about u) + coupled MCP/IP flexion; fixed built-in pronation RHO_T."""
import numpy as np, json, os
from scipy.optimize import least_squares, differential_evolution
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "hand"); os.makedirs(OUT, exist_ok=True)
D = 57.0; SC = 0.80
LEN = {k: tuple(round(SC*v, 1) for v in vv) for k, vv in dict(index=(40, 23, 18), middle=(44, 27, 19), ring=(41, 26, 18), pinky=(33, 19, 17)).items()}
TLEN = tuple(round(SC*v, 1) for v in (48, 33, 27))
U = dict(index=20.0, middle=3.0, ring=-14.0, pinky=-30.0)              # MCP u (pitch 17 / 16; middle near the forearm axis like the 3rd metacarpal)
ZM = dict(index=549.0, middle=552.0, ring=549.0, pinky=541.0)          # human knuckle arch
XM = 0.0                                                               # finger planes centred on the palm (palm x -13 .. +15)
U_MID = 3.0                                                            # abduction mirror plane
K_PIP, K_DIP = 1.10, 0.70
TIP_R = 6.3                                                            # fingertip sphere radius incl. 1.2 mm TPU
WID = dict(index=(7.6, 6.2), middle=(7.8, 6.3), ring=(7.4, 6.0), pinky=(6.6, 5.4))   # half-width at MCP, at tip (t)
THK = dict(index=(6.6, 5.4), middle=(6.8, 5.5), ring=(6.5, 5.3), pinky=(5.8, 4.8))   # half-thickness (b)
T0 = np.array([6.0, 27.0, 494.0])                                      # thumb CMC
ALPHA_T = 32.0; K_TIP = 0.75
def rod(v, k, a):
    k = k/np.linalg.norm(k); a = np.radians(a); return v*np.cos(a) + np.cross(k, v)*np.sin(a) + k*np.dot(k, v)*(1-np.cos(a))
def finger_frame(nm, psi):
    s = 0.0 if nm in ('middle', 'pinky') else (-1.0 if nm == 'index' else 1.0)    # index adducts toward -u, ring toward +u
    ex, ez = np.array([1.0, 0, 0]), np.array([0, 0, 1.0])
    ez2 = rod(ez, ex, -s*psi*(-1)) if s else ez                                    # rotate about x (palm normal)
    # rotation about +x by angle a moves +z toward -u for a>0; index needs -u -> a = +psi; ring needs +u -> a = -psi
    a = {'index': psi, 'ring': -psi}.get(nm, 0.0); ez2 = rod(ez, ex, a)
    return ex, ez2
def chain(nm, th1, psi=0.0):
    L1, L2, L3 = LEN[nm]; ec, ez = finger_frame(nm, psi)
    t1 = np.radians(th1); t2 = K_PIP*t1; t3 = K_DIP*t2
    d = lambda S: ec*np.sin(S) + ez*np.cos(S); n = lambda S: ec*np.cos(S) - ez*np.sin(S)
    M = np.array([XM, U[nm], ZM[nm]]); S1, S2, S3 = t1, t1+t2, t1+t2+t3
    P1 = M + L1*d(S1); P2 = P1 + L2*d(S2); P3 = P2 + L3*d(S3)
    tipc = P3 - (TIP_R - 0.3)*d(S3) + 0.8*n(S3)                                     # fingertip sphere centre (pad bulges palmar)
    return dict(M=M, P1=P1, P2=P2, P3=P3, tipc=tipc, n=n(S3), d=d(S3), S=(S1, S2, S3), ec=ec, ez=ez, dfun=d, nfun=n)
RHO_T = 70.0                                                           # built-in thumb pronation (design constant, set by solve_design)
def thumb(sw, lift, th1, rho=None, L=TLEN):
    rho = RHO_T if rho is None else rho
    m0 = np.array([0.25, np.sin(np.radians(ALPHA_T)), np.cos(np.radians(ALPHA_T))]); m0 /= np.linalg.norm(m0)
    b0 = np.array([1.0, 0, 0]); b0 = b0 - m0*np.dot(b0, m0); b0 /= np.linalg.norm(b0)
    R = lambda v: rod(rod(v, np.array([1.0, 0, 0]), sw), np.array([0, 1.0, 0]), lift)
    m = R(m0); b = rod(R(b0), m, rho); f = np.cross(m, b); f /= np.linalg.norm(f)
    s2 = rod(m, f, th1); s3 = rod(m, f, th1*(1+K_TIP)); b2 = rod(b, f, th1); b3 = rod(b, f, th1*(1+K_TIP))
    P1 = T0 + L[0]*m; P2 = P1 + L[1]*s2; P3 = P2 + L[2]*s3
    tipc = P3 - (TIP_R - 0.3)*s3 + 0.8*b3
    return dict(P0=T0, P1=P1, P2=P2, P3=P3, tipc=tipc, n=b3, d=s3, frames=[(T0, m, b), (P1, s2, b2), (P2, s3, b3)])
def segs_finger(c): return [(c['M'], c['P1']), (c['P1'], c['P2']), (c['P2'], c['P3'])]
def segs_thumb(c): return [(c['P0'], c['P1']), (c['P1'], c['P2']), (c['P2'], c['P3'])]
def seg_seg(p1, q1, p2, q2):
    d1, d2, r = q1-p1, q2-p2, p1-p2; a, e, f = d1@d1, d2@d2, d2@r; c = d1@r; b = d1@d2; den = a*e - b*b
    s = np.clip((b*f - c*e)/den, 0, 1) if den > 1e-9 else 0.0; t = (b*s + f)/e; t = np.clip(t, 0, 1); s = np.clip((b*t - c)/a, 0, 1)
    return np.linalg.norm((p1 + d1*s) - (p2 + d2*t))
def fourbar_fit(L1, k, th_max=100.0, n=14):
    th = np.radians(np.linspace(0, th_max, n)); phi = k*th
    def res(p):
        gx, gy, hx, hy, lb = p; out = []
        for t, f in zip(th, phi):
            O2 = L1*np.array([np.cos(t), np.sin(t)]); c, s = np.cos(t+f), np.sin(t+f)
            H = O2 + np.array([c*hx - s*hy, s*hx + c*hy]); out.append(np.hypot(*(H - [gx, gy])) - lb)
        out.append(10*max(0, 3.6 - np.hypot(gx, gy))); out.append(10*max(0, 3.6 - np.hypot(hx, hy)))     # pin spacing from the joint pin
        return out
    best = None; rng = np.random.default_rng(0)
    for _ in range(300):
        p0 = [rng.uniform(-6, 2), rng.uniform(-5, -1.5), rng.uniform(-6, 2), rng.uniform(-1.0, 1.6), rng.uniform(8, 40)]
        r = least_squares(res, p0, bounds=([-7, -5.2, -7, -1.0, 6], [3, -1.0, 3, 1.6, 60]))     # keep G dorsal, H palmar-ish but inside a +-5 groove band
        if best is None or r.cost < best.cost: best = r
    p = best.x; return dict(G=p[:2].tolist(), H=p[2:4].tolist(), lb=float(p[4]), max_err_mm=float(np.max(np.abs(res(p)[:-2]))), pin_gap=[float(np.hypot(*p[:2])), float(np.hypot(*p[2:4]))])
if __name__ == "__main__":
    FB = {nm: dict(pip=fourbar_fit(LEN[nm][0], K_PIP), dip=fourbar_fit(LEN[nm][1], K_DIP, 110)) for nm in LEN}
    for nm in FB: print(nm, {k: (round(v['lb'], 2), round(v['max_err_mm'], 3), np.round(v['G'], 2).tolist(), np.round(v['H'], 2).tolist()) for k, v in FB[nm].items()})
    json.dump(FB, open(os.path.join(OUT, "fourbars.json"), "w"), indent=1)
