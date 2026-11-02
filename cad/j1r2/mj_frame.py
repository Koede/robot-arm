"""J1 mount study (2026-11-02): linear 3-D frame model of harness + mount, shared by every mount concept.
Robot frame, mm, N, MPa (N/mm2). Right arm at the pad (y1, z1) = (+10, -30); the left side is the mirror in y about the spine plane y = -150.

Model (same for every concept; only the mount members between the pad and the harness change):
  * rail: the real wing rail section (scapular_frame.json: 38 x 7 mm PA-CF, z -440..-80, 7.5 mm foam gap), Euler-Bernoulli beams every 20 mm,
    each node on a foam spring normal to the back (E_f 0.15 MPa / 7.5 mm = 0.020 N/mm3 x tributary area) + shear springs (1/3);
  * spine bridge: 3 PA-CF straps (60 x 5) joining left and right rails at z -130 / -180 / -230 (no skin contact, gap 14);
  * clasp: hub at the centroid of its 4 feet; arms (PA-CF 20 x 6) to the pads F1, F4 (top, n +z), F2, F3 (front), axilla cradle; each pad is a
    foam spring along its normal (area from clasp_sites.json, k = 0.15 MPa / foam t); the clasp only carries arm load if the concept seats its feet;
  * contacts are linear about the strap preload (bilateral). A negative pressure increment larger than the preload means lift-off (flagged).
Loads: gravity of the arm + payload + the fixed module at the pad, from the project mass model (j1r2_kin.gravity_torques masses);
       LC_park (both arms at park, 0.5 kg in the right hand), LC_reach (right arm horizontal lateral, phi 0, th1 -90, 0.5 kg; left at park).
Outputs: pad rotation -> hand-tip deflection from mount compliance (arm rigid), pressures per contact zone, interface bolt-group tension."""
import numpy as np, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import j1r2_params as P
G_ = 9.81
MAT = dict(PACF=(6000.0, 2200.0), PETGCF=(4000.0, 1500.0), AL=(69000.0, 26000.0), CF=(110000.0, 5000.0), STEEL=(200000.0, 79000.0), RIGID=(1e7, 4e6))
# ------------------------------------------------------------------ sections (A, Iy, Iz, J) ; local y / z of the section given by 'up'
def rect(b, h):
    """solid rectangle, b along local y, h along local z."""
    a, c = max(b, h), min(b, h); J = a * c ** 3 * (1 / 3 - 0.21 * c / a * (1 - c ** 4 / (12 * a ** 4)))
    return dict(A=b * h, Iy=b * h ** 3 / 12.0, Iz=h * b ** 3 / 12.0, J=J, cy=b / 2, cz=h / 2)
def box(b, h, t):
    """closed thin-wall box b (local y) x h (local z), wall t (Bredt torsion)."""
    bi, hi = b - 2 * t, h - 2 * t; A = b * h - bi * hi; Am = (b - t) * (h - t)
    return dict(A=A, Iy=(b * h ** 3 - bi * hi ** 3) / 12.0, Iz=(h * b ** 3 - hi * bi ** 3) / 12.0, J=4 * Am ** 2 * t / (2 * (b - t) + 2 * (h - t)), cy=b / 2, cz=h / 2)
def tube(D, t):
    d = D - 2 * t; I = np.pi * (D ** 4 - d ** 4) / 64.0; return dict(A=np.pi * (D ** 2 - d ** 2) / 4.0, Iy=I, Iz=I, J=2 * I, cy=D / 2, cz=D / 2)
def bolt_group(pts_uv, d_shank=4.0, n_scale=1.0):
    """equivalent section of a bolted joint (steel shanks at offsets (u, v) in the joint plane): A = n A_b, I = sum A_b d^2."""
    Ab = np.pi * d_shank ** 2 / 4.0 * n_scale; pts = np.asarray(pts_uv, float); c = pts.mean(0); d = pts - c
    Iy = float(Ab * (d[:, 1] ** 2).sum()) + 1.0; Iz = float(Ab * (d[:, 0] ** 2).sum()) + 1.0
    return dict(A=Ab * len(pts), Iy=Iy, Iz=Iz, J=Iy + Iz, bolts=d.tolist(), Ab=Ab)
# ------------------------------------------------------------------ frame solver
class Frame:
    def __init__(s):
        s.nodes, s.idx, s.beams, s.springs, s.loads = [], {}, [], [], {}
    def node(s, name, xyz):
        if name in s.idx: return s.idx[name]
        s.idx[name] = len(s.nodes); s.nodes.append(np.asarray(xyz, float)); return s.idx[name]
    def beam(s, a, b, sec, mat="PACF", up=(1, 0, 0), tag=""):
        E, G = MAT[mat]; s.beams.append(dict(a=s.idx[a], b=s.idx[b], E=E, G=G, up=np.asarray(up, float), tag=tag, **sec))
    def rigid(s, a, b, tag="rigid"): s.beam(a, b, dict(A=1e5, Iy=1e8, Iz=1e8, J=1e8), "RIGID", tag=tag)
    def spring(s, n, direction, k, area=None, label="", kind="foam"):
        d = np.asarray(direction, float); d /= np.linalg.norm(d); s.springs.append(dict(n=s.idx[n], d=d, k=k, area=area, label=label, kind=kind))
    def force(s, n, F, M=(0, 0, 0)):
        i = s.idx[n]; f = s.loads.setdefault(i, np.zeros(6)); f[:3] += F; f[3:] += M
    @staticmethod
    def _T(xa, xb, up):
        ex = xb - xa; L = np.linalg.norm(ex); ex = ex / L; u = up - (up @ ex) * ex
        if np.linalg.norm(u) < 1e-6: u = np.array([0, 0, 1.0]) - ex[2] * ex
        if np.linalg.norm(u) < 1e-6: u = np.array([0, 1.0, 0]) - ex[1] * ex
        ez = u / np.linalg.norm(u); ey = np.cross(ez, ex); R = np.vstack([ex, ey, ez]); return R, L
    @staticmethod
    def _kloc(E, G, A, Iy, Iz, J, L):
        k = np.zeros((12, 12)); a = E * A / L; t = G * J / L
        k[0, 0] = k[6, 6] = a; k[0, 6] = k[6, 0] = -a; k[3, 3] = k[9, 9] = t; k[3, 9] = k[9, 3] = -t
        def bend(i_v, i_r, I, sgn):
            c1, c2, c3, c4 = 12 * E * I / L ** 3, 6 * E * I / L ** 2, 4 * E * I / L, 2 * E * I / L
            v1, r1, v2, r2 = i_v, i_r, i_v + 6, i_r + 6
            k[v1, v1] += c1; k[v2, v2] += c1; k[v1, v2] -= c1; k[v2, v1] -= c1
            k[v1, r1] += sgn * c2; k[r1, v1] += sgn * c2; k[v1, r2] += sgn * c2; k[r2, v1] += sgn * c2
            k[v2, r1] -= sgn * c2; k[r1, v2] -= sgn * c2; k[v2, r2] -= sgn * c2; k[r2, v2] -= sgn * c2
            k[r1, r1] += c3; k[r2, r2] += c3; k[r1, r2] += c4; k[r2, r1] += c4
        bend(1, 5, Iz, +1.0); bend(2, 4, Iy, -1.0); return k
    def solve(s):
        n = len(s.nodes); K = np.zeros((6 * n, 6 * n)); F = np.zeros(6 * n)
        for b in s.beams:
            R, L = s._T(s.nodes[b["a"]], s.nodes[b["b"]], b["up"]); T = np.kron(np.eye(4), R)
            kg = T.T @ s._kloc(b["E"], b["G"], b["A"], b["Iy"], b["Iz"], b["J"], L) @ T; ia, ib = 6 * b["a"], 6 * b["b"]
            ix = np.r_[ia:ia + 6, ib:ib + 6]; K[np.ix_(ix, ix)] += kg; b["_RL"] = (R, L, kg)
        for sp in s.springs:
            i = 6 * sp["n"]; K[i:i + 3, i:i + 3] += sp["k"] * np.outer(sp["d"], sp["d"])
        K[np.arange(3, 6 * n, 6), np.arange(3, 6 * n, 6)] += 1.0; K[np.arange(4, 6 * n, 6), np.arange(4, 6 * n, 6)] += 1.0; K[np.arange(5, 6 * n, 6), np.arange(5, 6 * n, 6)] += 1.0
        for i, f in s.loads.items(): F[6 * i:6 * i + 6] += f
        u = np.linalg.solve(K, F); s.u = u.reshape(n, 6); return s.u
    def spring_forces(s):
        """compression positive for foam (body pushes the harness along d) -> force = -k (u . d)."""
        out = []
        for sp in s.springs:
            f = -sp["k"] * float(s.u[sp["n"], :3] @ sp["d"]); out.append(dict(label=sp["label"], kind=sp["kind"], F=f, p_kPa=(f / sp["area"] * 1e3) if sp["area"] else None))
        return out
    def end_forces(s, tag):
        """local end forces of the beams with this tag at end a: N, Vy, Vz, T, My, Mz."""
        out = []
        for b in s.beams:
            if b["tag"] != tag: continue
            R, L, kg = b["_RL"]; ue = np.r_[s.u[b["a"]], s.u[b["b"]]]; fg = kg @ ue; T = np.kron(np.eye(4), R); out.append(T @ fg)
        return out
# ------------------------------------------------------------------ body geometry helpers (rear surface)
import clib as L_
_RS = {}
def back_x(y, z):
    """x of the back surface at (y, z): one cached ray-cast grid (y -330..70, z -470..70, 4 mm; misses filled with the nearest hit), bilinear."""
    if "f" not in _RS:
        from scipy.interpolate import RegularGridInterpolator
        Y, Z, X = L_.rear_surface(-330.0, 70.0, -470.0, 70.0, 4.0, 1.0); _RS["f"] = RegularGridInterpolator((Y[:, 0], Z[0, :]), X, bounds_error=False, fill_value=None)
    return float(_RS["f"]([[y, z]])[0])
DER = P.derive(10.0, -30.0, 22.0); PX, Y1, Z1, X0, XB = DER["PX"], DER["y1"], DER["z1"], DER["x0"], DER["XB"]
SPINE_Y = -150.0
def mirror(p): p = np.asarray(p, float).copy(); p[1] = 2 * SPINE_Y - p[1]; return p
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
SITES = json.load(open(os.path.join(REPO, "stl", "shoulder", "clasp_sites.json")))
FEET = json.load(open(os.path.join(REPO, "stl", "shoulder", "clasp_feet.json")))
CRADLE = json.load(open(os.path.join(REPO, "stl", "shoulder", "clasp_cradle.json")))
E_FOAM = 0.15                        # MPa (scapular_frame.json relief E_f; combined foam + tissue, assumption)
RAIL = dict(y=-119.0, b=38.0, t=7.0, gap=7.5, z=(-440.0, -80.0), dz=20.0)
# ------------------------------------------------------------------ mass model -> wrench at the pad
def arm_masses(th1, phi, payload=0.5):
    """[(kg, xyz)] of everything the pad carries (j1r2_kin.gravity_torques masses + the fixed module at the pad)."""
    F0, FA, s, a, d = P.shoulder_frames(DER, th1, phi); P1 = np.array([PX, Y1, Z1]); base = P1 + P.T_ARM * s
    return [(1.489, base + 249.0 * d), (payload, base + 575.0 * d), (0.621, P1 + (P.TS + P.AKL / 2) * s), (0.150, P1 + 30.0 * s),
            (0.621, np.array([(XB + DER["XO"]) / 2, Y1, Z1])), (0.36, np.array([XB + 20, Y1, Z1]))]     # S1' AK70 + pad/fairing/stops/fixed balancer side ~0.36 kg
def hand_tip(th1, phi):
    F0, FA, s, a, d = P.shoulder_frames(DER, th1, phi); return np.array([PX, Y1, Z1]) + P.T_ARM * s + 575.0 * d
LCS = dict(park=dict(right=(P.PARK["S1p"], P.PARK["S2"], 0.5), left=(P.PARK["S1p"], P.PARK["S2"], 0.0)),
           reach=dict(right=(-90.0, 0.0, 0.5), left=(P.PARK["S1p"], P.PARK["S2"], 0.0)))
# ------------------------------------------------------------------ the common harness
FOOT_X = dict(F5_mid_trap=-123.0, F1_upper_trap=-126.0, F4_acromion=-126.0, R1_scapula=-102.0)   # re-seated bracket mid-planes (mj_concepts)
def harness(fr, side="R", clasp=True):
    """adds rail + foam + clasp (pads, arms, hub) for one side; returns dict of named nodes. side 'L' mirrors y."""
    mp = (lambda p: np.asarray(p, float)) if side == "R" else mirror; nm = lambda s_: f"{side}_{s_}"; out = {}
    E, b, t, gap = E_FOAM, RAIL["b"], RAIL["t"], RAIL["gap"]; zs = np.arange(RAIL["z"][0], RAIL["z"][1] + 0.1, RAIL["dz"]); prev = None
    for z in zs:
        x = back_x(RAIL["y"], z) + gap + t / 2; p = mp([x, RAIL["y"], z]); fr.node(nm(f"rail{int(z)}"), p)
        k = E / gap * b * RAIL["dz"]; fr.spring(nm(f"rail{int(z)}"), [1, 0, 0], k, area=b * RAIL["dz"], label=nm(f"rail{int(z)}"), kind="rail")
        fr.spring(nm(f"rail{int(z)}"), [0, 1, 0], k / 3.0, kind="shear"); fr.spring(nm(f"rail{int(z)}"), [0, 0, 1], k / 3.0, kind="shear")
        if prev: fr.beam(prev, nm(f"rail{int(z)}"), rect(b, t), "PACF", up=(1, 0, 0), tag="rail")
        prev = nm(f"rail{int(z)}")
    out["rail_nodes"] = [nm(f"rail{int(z)}") for z in zs]
    if clasp:
        hub = mp(np.array([FOOT_X["F1_upper_trap"], FEET["F1_upper_trap"]["y"], FEET["F1_upper_trap"]["z"]])); fr.node(nm("clasp_hub"), hub); out["clasp_hub"] = nm("clasp_hub")
        pads = dict(F1=("F1_upper_trap", "top"), F4=("F4_acromion", "top"), F2=("F2_ant_delt", "front"), F3=("F3_pec", "front"))
        for key, (sk, kind) in pads.items():
            st = SITES[sk]; p = mp(st["p"]); n = np.asarray(st["n"], float); n = n / np.linalg.norm(n)
            if side == "L": n = n * np.array([1, -1, 1])
            A = st["area_mm2"]; tf = st["contour"]["stats"]["foam_t_mean"]; fr.node(nm(key), p)
            fr.spring(nm(key), n, E / tf * A, area=A, label=nm(key), kind=kind)
            tg = [n[1], -n[0], 0] if abs(n[2]) < 0.9 else [1, 0, 0]
            fr.spring(nm(key), tg, E / tf * A / 3, kind="shear"); fr.spring(nm(key), np.cross(n, tg), E / tf * A / 3, kind="shear")
            fr.beam(nm("clasp_hub"), nm(key), rect(20.0, 6.0), "PACF", up=(0, 0, 1), tag="clasp_arm")
        ax = mp(CRADLE["A"]); fr.node(nm("axilla"), ax); fr.spring(nm("axilla"), [0.3, 0, 1.0], E / 8.0 * 48 * 58, area=48 * 58, label=nm("axilla"), kind="axilla")
        fr.beam(nm("F3"), nm("axilla"), rect(20.0, 6.0), "PACF", up=(1, 0, 0), tag="clasp_arm"); fr.beam(nm("F2"), nm("axilla"), rect(20.0, 6.0), "PACF", up=(1, 0, 0), tag="clasp_arm")
        out["feet"] = {k: nm("foot_" + k) for k in FEET}
        for k, f in FEET.items():
            if k in ("F1_upper_trap", "R1_scapula"): continue                   # the hub IS the F1 foot; the R1 foot is dropped (lifted standoff node)
            fr.node(nm("foot_" + k), mp([FOOT_X[k], f["y"], f["z"]]))
            if k in ("F5_mid_trap", "F4_acromion"): fr.beam(nm("clasp_hub"), nm("foot_" + k), rect(30.0, 6.0), "PACF", up=(1, 0, 0), tag="clasp_seat")
    return out
def spine_bridge(fr):
    for z in (-140.0, -180.0, -220.0):
        a, b = f"R_rail{int(z)}", f"L_rail{int(z)}"; zz = z
        mid = np.array([back_x(SPINE_Y, zz) + 14.0 + 2.5, SPINE_Y, zz]); fr.node(f"sb{int(z)}", mid)
        fr.beam(a, f"sb{int(z)}", rect(60.0, 5.0), "PACF", up=(1, 0, 0), tag="spine_bridge"); fr.beam(f"sb{int(z)}", b, rect(60.0, 5.0), "PACF", up=(1, 0, 0), tag="spine_bridge")
def apply_arm(fr, pad_node, side, th1, phi, payload, extra_g=0.0, extra_xyz=None):
    """gravity of the arm/module (+ the concept's own structure mass, lumped) as a wrench at the pad node."""
    ms = arm_masses(th1, phi, payload)
    if extra_g: ms.append((extra_g / 1000.0, np.asarray(extra_xyz if extra_xyz is not None else [XB, Y1, Z1], float)))
    pn = fr.nodes[fr.idx[pad_node]]; F = np.zeros(3); M = np.zeros(3)
    for m, r in ms:
        r = r if side == "R" else mirror(r); f = np.array([0, 0, -m * G_]); F += f; M += np.cross(r - pn, f)
    fr.force(pad_node, F, M); return F, M
