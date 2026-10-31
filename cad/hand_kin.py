"""HAND (human-layout, 5-digit, micro-servo) kinematics + chuck-mode solver.  Right arm, W3/hand frame, mm, deg.
Frame: x = palm normal (+x palm side), u = y - D (+u = thumb/radial side), z = absolute robot z.  Hand axis (W1 axis) = (x 0, u 0).
Finger i: MCP at (XM, U_i, ZM_i); flexion plane yawed by psi_i about a z-parallel hinge through the MCP; curl direction e_c = (cos psi, -sign(U_i) sin psi).
Phalanx angles: theta1 (MCP, servo driven) -> theta2 = FP(theta1) -> theta3 = FD(theta2) via ground-linked four-bars (fitted below).
Chuck axis A = hand axis (x 0, u 0), parallel to z.  Pad centre = distal phalanx 0.6 L3 + palmar offset."""
import numpy as np, json, os
from scipy.optimize import least_squares, differential_evolution
HERE = os.path.dirname(os.path.abspath(__file__)); D = 57.0
DZ = 24.0                                                    # hand lifted rigidly by the plane-to-plane wrist coupling (wrist_rotary.py, mating plane z 491.6 + DZ)
SC = 0.80                                                    # scale of the 50th-percentile male hand (hand length 189 -> ~151 + wrist), servo bodies are NOT scaled
# phalanx lengths mm (prox, mid, dist), 50th pct male x SC.  Thumb: metacarpal, prox, dist
LEN = dict(index=(40, 22, 17), middle=(44, 26, 18), ring=(41, 24, 17), pinky=(32, 18, 16))
LEN = {k: tuple(round(SC * v, 1) for v in vv) for k, vv in LEN.items()}
TLEN = tuple(round(0.92 * v, 1) for v in (46, 32, 26))
PITCH = 23.0
U = dict(index=PITCH, middle=0.0, ring=-PITCH, pinky=-PITCH - 19.5)      # MCP u positions
ZM = dict(index=498.0 + DZ, middle=558.0 + DZ, ring=535.0 + DZ, pinky=514.0 + DZ)               # knuckle line (human arch: middle most distal)
XMD = dict(index=-20.0, middle=-27.0, ring=-20.0, pinky=-20.0)             # per-finger plane depth (middle finger sits deeper so the yawed index/ring pass in front of it)
XM = -20.0                                                                  # finger-plane centre x (axis A at x 0)
PADT = 5.0 + 1.5                                                           # phalanx half thickness 5 + TPU pad 1.5 -> palmar surface offset from axis of phalanx
PAD_S = 0.62                                                               # pad centre along distal phalanx (fraction of L3 from the DIP)
K_PIP, K_DIP = 1.10, 0.70
SIDE_MARGIN = -1.0                                                         # mm: a finger's pad may sit at most 1 mm past the axis plane (keeps index/ring on their own side)
TH_MAX = 26.0                                                              # chuck mode keeps cumulative flexion <= 2.87*26 = 75 deg (pinch cone, not a fist)                                                  # human synergy: PIP = 1.10 MCP, DIP = 0.70 PIP  (Cobos 2008 style: fist MCP 90, PIP ~100, DIP ~70)

TAU = 45.0                                                                 # knuckle yaw hinge tilted 35 deg from z toward +x (spreads an extended finger AND tilts the curl plane)
def rod(v, k, a):
    k = k / np.linalg.norm(k); a = np.radians(a); return v*np.cos(a) + np.cross(k, v)*np.sin(a) + k*np.dot(k, v)*(1-np.cos(a))
def rot2(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s], [s, c]])

def fit_fourbar(L1, k, th_max=100.0, seed=0, n=14):
    """ground-linked four-bar: ground pt G (frame of the lower link), pt H on the upper link (frame rotating by th+phi about the joint at (L1,0)); bar length lb.
    Want phi = k*th for th in [0, th_max].  Frame: lower link axis along +a, flexion rotates the link toward +b (palm side)."""
    th = np.radians(np.linspace(0, th_max, n)); phi = k * th
    def res(p):
        gx, gy, hx, hy, lb = p
        O2 = L1 * np.stack([np.cos(th), np.sin(th)], 1)
        H = O2 + np.einsum('nij,j->ni', np.stack([rot2(t + f) for t, f in zip(th, phi)]), np.array([hx, hy]))
        return np.hypot(H[:, 0] - gx, H[:, 1] - gy) - lb
    best = None; rng = np.random.default_rng(seed)
    for _ in range(400):
        p0 = [rng.uniform(-9, 9), rng.uniform(-9, -2), rng.uniform(-9, 9), rng.uniform(2, 9), rng.uniform(10, 40)]
        try: r = least_squares(res, p0, bounds=([-10, -10, -10, 2, 6], [10, -2, 10, 10, 60]))
        except Exception: continue
        if best is None or r.cost < best.cost: best = r
    p = best.x; rr = res(p)
    return dict(G=[float(p[0]), float(p[1])], H=[float(p[2]), float(p[3])], lb=float(p[4]), max_err_mm=float(np.abs(rr).max()), th_max=th_max, k=k)

def chain(name, th1_deg, psi_deg, fb=None):
    """points of finger `name` (MCP, PIP, DIP, tip, pad centre, pad normal) in (x,u,z)."""
    L1, L2, L3 = LEN[name]; s = np.sign(U[name]) if U[name] != 0 else 1.0
    h = np.array([np.sin(np.radians(TAU)), 0.0, np.cos(np.radians(TAU))]); ec = rod(np.array([1.0, 0, 0]), h, s * psi_deg); ez = rod(np.array([0, 0, 1.0]), h, s * psi_deg)   # psi>0 = toward the hand axis (index/ring mirror), psi<0 = spread outward
    t1 = np.radians(th1_deg); t2 = K_PIP * t1; t3 = K_DIP * t2
    d = lambda S: ec * np.sin(S) + ez * np.cos(S); nrm = lambda S: ec * np.cos(S) - ez * np.sin(S)
    M = np.array([XMD[name], U[name], ZM[name]]); P1 = M + L1 * d(t1); S2 = t1 + t2; P2 = P1 + L2 * d(S2); S3 = S2 + t3; P3 = P2 + L3 * d(S3)
    pad = P2 + PAD_S * L3 * d(S3) + PADT * nrm(S3)
    return dict(M=M, P1=P1, P2=P2, P3=P3, pad=pad, n=nrm(S3), d=d(S3), S=(t1, S2, S3))

def solve_chuck(rb, rod_dir=(0, 0, 1)):
    """Find (th_idx, th_mid, th_ring, psi) so each finger's pad surface touches a rod of radius rb on the hand axis.  returns dict"""
    names = ['index', 'middle', 'ring']
    def res(p):
        thi, thm, thr, psi = p; r = []
        for nm, th in zip(names, (thi, thm, thr)):
            c = chain(nm, th, 0.0 if nm == 'middle' else psi)
            h = np.array([c['pad'][0], c['pad'][1]]); nh = np.array([c['n'][0], c['n'][1]])
            r.append(np.linalg.norm(h) - rb)                                    # pad surface touches the rod
            # pad normal should point at the axis: component along -h/|h|
            r.append(1.0 * (1.0 - float(np.dot(nh, -h / (np.linalg.norm(h) + 1e-9)) / (np.linalg.norm(nh) + 1e-9))))
            sgn = np.sign(U[nm]) if U[nm] != 0 else 0.0
            r.append(2.0 * max(0.0, -sgn * c['pad'][1] + SIDE_MARGIN) if sgn != 0 else 0.0)        # each finger stays on its own side of the rod
        return r
    best = None
    for t0 in (4, 8, 12, 16):
        for ps in (10, 25, 40):
            r = least_squares(res, [t0, t0, t0, ps], bounds=([0, 0, 0, 0], [TH_MAX, TH_MAX, TH_MAX, 60]))
            if best is None or r.cost < best.cost: best = r
    return best

if __name__ == "__main__":
    print("LEN", LEN, "THUMB", TLEN)
    FB = {}
    for nm in LEN:
        L1, L2, L3 = LEN[nm]
        a = fit_fourbar(L1, K_PIP, 100.0); b = fit_fourbar(L2, K_DIP, 100.0 * K_PIP)
        FB[nm] = dict(pip=a, dip=b); print(nm, "PIP bar", {k: (np.round(v, 2).tolist() if isinstance(v, list) else round(v, 3)) for k, v in a.items()}, "\n     DIP bar", {k: (np.round(v, 2).tolist() if isinstance(v, list) else round(v, 3)) for k, v in b.items()})
    json.dump(FB, open(os.path.join(HERE, "..", "stl", "hand", "fourbars.json"), "w"), indent=1)
    for rb in (2.0, 3.0, 4.0, 5.0, 6.0, 8.0):
        r = solve_chuck(rb); thi, thm, thr, psi = r.x; print(f"rb {rb}: th_idx {thi:.1f} th_mid {thm:.1f} th_ring {thr:.1f} psi {psi:.1f}  cost {r.cost:.4f}")


# ---------------------------------------------------------------- THUMB
T0 = np.array([-8.0, 41.0, 500.0 + DZ])        # CMC centre (x,u,z): radial side of the palm, below the index knuckle
K_TIP = 0.75                               # thumb IP = K_TIP * MCP (coupled bar)
def thumb_chain(axis_az, axis_el, alpha, omega, th1, L=TLEN, tilt_b=0.0):
    ax = np.array([np.cos(np.radians(axis_el))*np.cos(np.radians(axis_az)), np.cos(np.radians(axis_el))*np.sin(np.radians(axis_az)), np.sin(np.radians(axis_el))])
    m0 = np.array([0.0, np.sin(np.radians(alpha)), np.cos(np.radians(alpha))]); b0 = np.array([1.0, 0.0, 0.0])
    f0 = np.cross(m0, b0); f0 /= np.linalg.norm(f0)
    m = rod(m0, ax, omega); f = rod(f0, ax, omega); b = rod(b0, ax, omega)
    th2 = K_TIP * th1
    s1 = m; s2 = rod(m, f, th1); s3 = rod(m, f, th1 + th2)
    n3 = rod(np.cross(f, s3) * -1.0 if False else rod(b, f, 0.0), f, th1 + th2)   # palmar normal rotates with the distal segment
    P1 = T0 + L[0]*s1; P2 = P1 + L[1]*s2; P3 = P2 + L[2]*s3
    pad = P2 + 0.62*L[2]*s3 + 6.5*n3
    return dict(P0=T0, P1=P1, P2=P2, P3=P3, pad=pad, n=n3, axis=ax, f=f, s=(s1, s2, s3))
def solve_thumb(rb, zc=578.0, seed=1):
    tgt = np.array([rb, 0.0, zc])
    def cost(p):
        c = thumb_chain(*p); e = c['pad'] - tgt; en = 1.0 + c['n'][0]          # want n = -x
        return float(e[0]**2 + e[1]**2 + 0.15*e[2]**2 + 400.0*en**2)
    b = [(-180, 180), (-90, 90), (10, 40), (0, 170), (0, 70)]
    r = differential_evolution(cost, b, seed=seed, tol=1e-9, maxiter=400, popsize=30, polish=True)
    return r

def solve_thumb_target(tgt, ntgt, seed=1):
    def cost(p):
        c = thumb_chain(*p); e = c['pad'] - tgt; return float(np.sum(e**2) + 200.0*np.sum((c['n'] - ntgt)**2))
    b = [(-180, 180), (-90, 90), (10, 40), (0, 170), (0, 70)]
    return differential_evolution(cost, b, seed=seed, tol=1e-9, maxiter=300, popsize=25, polish=True)

# ---------------------------------------------------------------- THUMB v2: 2-axis CMC (swing about x (palm normal), lift about u) + coupled MCP/IP flexion
ALPHA_T = 25.0                                                             # rest metacarpal tilt outward (deg)
def thumb2(sw, lift, th1, L=TLEN):
    """sw: swing about the z-parallel CMC axis (+ = toward the palm centre, -u); lift: rotation about the u-parallel axis (+ = tip toward +x, palm normal)."""
    m0 = np.array([0.0, np.sin(np.radians(ALPHA_T)), np.cos(np.radians(ALPHA_T))]); b0 = np.array([1.0, 0, 0])
    f0 = np.cross(m0, b0); f0 /= np.linalg.norm(f0)
    R = lambda v: rod(rod(v, np.array([1.0, 0, 0]), sw), np.array([0, 1.0, 0]), lift)   # lift sign so +lift raises the tip toward +x
    m, f, b = R(m0), R(f0), R(b0)
    s2 = rod(m, f, th1); s3 = rod(m, f, th1 + K_TIP * th1); n3 = rod(b, f, th1 + K_TIP * th1)
    P1 = T0 + L[0]*m; P2 = P1 + L[1]*s2; P3 = P2 + L[2]*s3; pad = P2 + 0.62*L[2]*s3 + 6.5*n3
    return dict(P1=P1, P2=P2, P3=P3, pad=pad, n=n3, dirs=(m, s2, s3))
def solve_thumb2(tgt, ntgt, wn=200.0, seed=2):
    def cost(p):
        c = thumb2(*p); return float(np.sum((c['pad'] - tgt)**2) + wn*np.sum((c['n'] - ntgt)**2))
    return differential_evolution(cost, [(-10, 100), (-10, 100), (0, 75)], seed=seed, tol=1e-10, maxiter=300, popsize=30, polish=True)

def thumb3(sw, lift, roll, th1, L=TLEN):
    """thumb2 + roll about the metacarpal long axis (pronation).  roll is coupled to swing in hardware (roll = KAPPA*swing, fitted below)."""
    m0 = np.array([0.0, np.sin(np.radians(ALPHA_T)), np.cos(np.radians(ALPHA_T))]); b0 = np.array([1.0, 0, 0]); f0 = np.cross(m0, b0); f0 /= np.linalg.norm(f0)
    R = lambda v: rod(rod(v, np.array([1.0, 0, 0]), sw), np.array([0, 1.0, 0]), lift)
    m = R(m0); b = rod(R(b0), m, roll); f = np.cross(m, b); f /= np.linalg.norm(f)
    s2 = rod(m, f, th1); s3 = rod(m, f, th1 + K_TIP*th1); n3 = rod(b, f, th1 + K_TIP*th1)
    P1 = T0 + L[0]*m; P2 = P1 + L[1]*s2; P3 = P2 + L[2]*s3; pad = P2 + 0.62*L[2]*s3 + 6.5*n3
    return dict(P1=P1, P2=P2, P3=P3, pad=pad, n=n3, dirs=(m, s2, s3))
def solve_thumb3(tgt, ntgt, wn=200.0, seed=2, bounds=((-10, 100), (-10, 100), (-20, 200), (0, 75))):
    cost = lambda p: float(np.sum((thumb3(*p)['pad'] - tgt)**2) + wn*np.sum((thumb3(*p)['n'] - ntgt)**2))
    return differential_evolution(cost, list(bounds), seed=seed, tol=1e-10, maxiter=300, popsize=30, polish=True)

RHO_T = 190.0                                                              # thumb roll, built into the CMC housing (fixed)
def thumb_frames(sw, lift, th1, L=TLEN):
    """segment frames of the thumb: list of (origin, a, b) for metacarpal, proximal, distal; b = palmar normal"""
    m0 = np.array([0.0, np.sin(np.radians(ALPHA_T)), np.cos(np.radians(ALPHA_T))]); b0 = np.array([1.0, 0, 0])
    R = lambda v: rod(rod(v, np.array([1.0, 0, 0]), sw), np.array([0, 1.0, 0]), lift)
    m = R(m0); b = rod(R(b0), m, RHO_T); f = np.cross(m, b); f /= np.linalg.norm(f)
    th2 = K_TIP * th1; s2 = rod(m, f, th1); s3 = rod(m, f, th1 + th2); b2 = rod(b, f, th1); b3 = rod(b, f, th1 + th2)
    P1 = T0 + L[0]*m; P2 = P1 + L[1]*s2
    return [(T0, m, b), (P1, s2, b2), (P2, s3, b3)]
