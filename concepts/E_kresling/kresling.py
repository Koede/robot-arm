"""CONCEPT E: Kresling (origami) TPU bellows between plane A and plane B. One actuator (R servo) twists plane B; the bellows shortens as it twists (b bars fixed:
b^2 = H^2 + 2 R^2 (1 - cos phi)).  Wire tube OD12/ID9 on the axis (fixed to A, plane B slides on a bushing).  Constants from cad/wrist_rotary.py.  Truss/hinge stiffness is an ESTIMATE (low confidence)."""
import sys, os, json, math, itertools
sys.path.insert(0, "/home/claude/robot-arm/cad")
import numpy as np, cadquery as cq, trimesh
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
OUT = os.path.dirname(os.path.abspath(__file__))
E_TPU = 26.0            # MPa, TPU 95A (range 20-40)
H0, PHI0 = 20.0, 30.0   # neutral height mm, neutral twist deg; ROM wanted +-30 deg about PHI0
HMIN = 6.0
def geom(n, R, H0=H0, phi0=PHI0):
    b2 = H0 ** 2 + 2 * R ** 2 * (1 - math.cos(math.radians(phi0))); return math.sqrt(b2)
def H_of_phi(b, R, phi): return math.sqrt(max(b * b - 2 * R * R * (1 - math.cos(math.radians(phi))), 0.0))
def rom(n, R):
    b = geom(n, R); up = math.degrees(math.acos(max(-1, 1 - (b * b - HMIN ** 2) / (2 * R * R)))); dn = PHI0
    return min(up - PHI0, dn), b
def verts(n, R, H, phi, tx=0, ty=0, thx=0, thy=0):
    be = 2 * math.pi / n; P = np.array([[R * math.cos(k * be), R * math.sin(k * be), 0.0] for k in range(n)])
    Q0 = np.array([[R * math.cos(k * be + math.radians(phi)), R * math.sin(k * be + math.radians(phi)), 0.0] for k in range(n)])
    cx, sx, cy, sy = math.cos(thx), math.sin(thx), math.cos(thy), math.sin(thy)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]]); Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Q = (Ry @ Rx @ Q0.T).T + np.array([tx, ty, H]); return P, Q
def bars(n, R, q):
    H, phi, tx, ty, thx, thy = q; P, Q = verts(n, R, H, phi, tx, ty, thx, thy)
    b = np.linalg.norm(P - Q, axis=1); c = np.linalg.norm(np.roll(P, -1, axis=0) - Q, axis=1); return b, c
def stiffness(n, R, t, kap_b=0.30, kap_c=0.03):
    """truss model: b bars (EA = E t kap_b a), c bars (valley/fold zone, EA = E t kap_c a); pose q=(H, phi_deg, tx, ty, thx, thy); returns condensed stiffnesses."""
    a = 2 * R * math.sin(math.pi / n); q0 = np.array([H0, PHI0, 0, 0, 0, 0.0]); b0, c0 = bars(n, R, q0)
    kb = E_TPU * t * kap_b * a / b0; kc = E_TPU * t * kap_c * a / c0
    def U(q): b, c = bars(n, R, q); return 0.5 * np.sum(kb * (b - b0) ** 2) + 0.5 * np.sum(kc * (c - c0) ** 2)
    h = np.array([1e-3, 1e-3, 1e-3, 1e-3, 1e-5, 1e-5]); K = np.zeros((6, 6))
    for i, j in itertools.product(range(6), range(6)):
        ei = np.zeros(6); ej = np.zeros(6); ei[i] = h[i]; ej[j] = h[j]
        K[i, j] = (U(q0 + ei + ej) - U(q0 + ei - ej) - U(q0 - ei + ej) + U(q0 - ei - ej)) / (4 * h[i] * h[j])
    # units: H,tx,ty mm; phi deg; th rad.  convert phi to rad
    S = np.diag([1, 180 / math.pi, 1, 1, 1, 1]); K = S @ K @ S; K += 1e-9 * np.eye(6)
    Kfree = [1 / np.linalg.inv(K)[i, i] for i in range(6)]
    return dict(axial_free_N_per_mm=Kfree[0], torsion_free_Nmm_per_rad=Kfree[1], bend_free_Nmm_per_rad=0.5 * (Kfree[4] + Kfree[5]),
                axial_fixed=K[0, 0], torsion_fixed_Nmm_per_rad=K[1, 1], bend_fixed_Nmm_per_rad=0.5 * (K[4, 4] + K[5, 5]), coupling_H_phi=K[0, 1] / math.sqrt(K[0, 0] * K[1, 1]))
res = dict(neutral=dict(H0=H0, phi0_deg=PHI0), sweep=[])
for n, R, t in itertools.product((6, 7, 8), (20, 22, 25), (0.8, 1.0, 1.2)):
    rm, b = rom(n, R); st = stiffness(n, R, t)
    dphidH = -H0 / (R * R * math.sin(math.radians(PHI0))) * 180 / math.pi        # deg per mm
    res["sweep"].append(dict(n=n, R=R, t=t, b=round(b, 2), rom_pm_deg=round(rm, 1), twist_deg_per_mm=round(dphidH, 2), bend_Nm_per_deg=round(st["bend_free_Nmm_per_rad"] / 1e3 * math.pi / 180, 5),
                             tors_Nm_per_deg=round(st["torsion_free_Nmm_per_rad"] / 1e3 * math.pi / 180, 5), axial_N_per_mm=round(st["axial_free_N_per_mm"], 3)))
# chosen design: n 6, R 20, t 1.0 (largest valid ROM; R<=22 required for +-30 deg with Hmin 6)
n, R, t = 6, 20.0, 1.0; rm, b = rom(n, R); st = stiffness(n, R, t); a = 2 * R * math.sin(math.pi / n)
Hs = {phi: H_of_phi(b, R, phi) for phi in (0, 30, 60)}
dphidH = -H0 / (R * R * math.sin(math.radians(PHI0)))
w_h = 1.5 * t                                                            # living-hinge width at creases
dpsi = abs(math.acos(Hs[60] / b) - math.acos(Hs[0] / b))                 # change of b-bar tilt over the full +-30 deg ROM
eps = t * dpsi / (2 * w_h)
M = 0.5 * 9.81 * 0.2 * 2.0 / 1.0                                          # N.m, 0.5 kg at 200 mm, 2 g
kb_Nm_rad = st["bend_free_Nmm_per_rad"] / 1e3
sag_deg = math.degrees(M / kb_Nm_rad)
Pcr = math.pi ** 2 * (st["bend_free_Nmm_per_rad"] * H0) / (4 * (2 * H0) ** 2) if False else math.pi ** 2 * st["bend_free_Nmm_per_rad"] / (4 * H0)   # EI = k_b*H0 ; cantilever column length H0
F_ax = (0.5 + 0.5) * 9.81 * 2.0                                          # hand ~0.5 kg + payload, 2 g
vol = None
res["chosen"] = dict(n=n, R=R, t=t, H0=H0, phi0_deg=PHI0, b_mm=round(b, 2), a_mm=round(a, 2), H_at_phi0_30_60=[round(Hs[0], 2), round(Hs[30], 2), round(Hs[60], 2)],
    axial_travel_for_pm30_mm=[round(Hs[60] - H0, 2), round(Hs[0] - H0, 2)], twist_deg_per_mm_at_neutral=round(math.degrees(dphidH), 2),
    stiffness={k: round(float(v), 4) for k, v in st.items()}, torsion_Nm_per_deg=round(st["torsion_free_Nmm_per_rad"] / 1e3 * math.pi / 180, 5), bending_Nm_per_deg=round(kb_Nm_rad * math.pi / 180, 5), axial_N_per_mm=round(st["axial_free_N_per_mm"], 3),
    hinge_strain_pct_over_ROM=round(eps * 100, 1), payload_moment_Nm_2g=round(M, 2), sag_under_payload_deg=round(sag_deg, 1), axial_load_2g_N=round(F_ax, 1), Euler_Pcr_N=round(Pcr, 2), buckles=bool(F_ax > Pcr),
    stiffness_uncertainty="kap_c (fold-zone width) x3 either way; TPU E 20-40 MPa", )
# CAD
def tri_solid(p0, p1, p2, th):
    f = cq.Face.makeFromWires(cq.Wire.makePolygon([cq.Vector(*p0), cq.Vector(*p1), cq.Vector(*p2), cq.Vector(*p0)]))
    nrm = np.cross(np.array(p1) - p0, np.array(p2) - p0); nrm /= np.linalg.norm(nrm); return cq.Solid.extrudeLinear(f, cq.Vector(*(nrm * th)))
def build(n, R, t, H, phi):
    P, Q = verts(n, R, H, phi); sols = []
    for k in range(n):
        k1 = (k + 1) % n
        sols.append(tri_solid(P[k], P[k1], Q[k], t)); sols.append(tri_solid(Q[k], Q[k1], P[k1], t))
    bell = cq.Workplane("XY").newObject([cq.Compound.makeCompound(sols)])
    plA = cq.Workplane("XY").circle(R + 3).extrude(-2.0).cut(cq.Workplane("XY").circle(6.0).extrude(-3.0).translate((0, 0, 0.5)))
    plB = cq.Workplane("XY").circle(R + 3).extrude(2.0).translate((0, 0, H)).rotate((0, 0, 0), (0, 0, 1), 0).cut(cq.Workplane("XY").circle(7.5).extrude(4).translate((0, 0, H - 1)))
    tube = cq.Workplane("XY").circle(6.0).circle(4.5).extrude(H + 8.0).translate((0, 0, -2.0))
    return bell, plA, plB, tube
bell, plA, plB, tube = build(n, R, t, H0, PHI0)
comp = [("bellows", bell, 1.21), ("plane_A", plA, 1.30), ("plane_B", plB, 1.30)]
meshes = {}
for nm, w, d in comp + [("tube", tube, 8.5)]:
    p = os.path.join(OUT, nm + ".stl"); cq.exporters.export(w, p, tolerance=0.02, angularTolerance=0.05); meshes[nm] = trimesh.load(p)
bm = meshes["bellows"]; res["chosen"]["bellows_mass_g"] = round(bm.volume / 1e3 * 1.21, 1) if bm.volume > 0 else None
res["chosen"]["bellows_watertight"] = bool(bm.is_watertight); res["chosen"]["bellows_volume_cm3"] = round(abs(bm.volume) / 1e3, 2)
tm = trimesh.util.concatenate(list(meshes.values())); tm.export(os.path.join(OUT, "kresling_neutral.stl"))
# printability: panel tilt from vertical (b-bar) and overhang of panels when printed axis-up
tilt = math.degrees(math.acos(H0 / b)); res["chosen"]["printability"] = dict(print_axis="z up, TPU 95A, 0.2 mm layers, 3 perimeters at 0.4 mm nozzle for 1.0 mm wall", panel_overhang_from_vertical_deg=round(math.degrees(math.atan2(math.sqrt(max(b*b-H0*H0,0)), H0)), 1),
                         note="roof triangles (upper panels) lean outward; panels are 1.0 mm = 2.5 perimeters, creases are the same wall (thinning to 0.6 mm at the fold needs 0.2 mm hinge slots), no supports if lean < 45 deg")
# plots
fig, ax = plt.subplots(1, 3, figsize=(16, 5.2), dpi=110)
for nm, m in meshes.items():
    s = m.section(plane_origin=[0, 0, 0], plane_normal=[0, 1, 0])
    if s is None: continue
    for e in s.discrete: ax[0].fill(e[:, 0], e[:, 2], color={"bellows": "tab:orange", "plane_A": "tab:blue", "plane_B": "tab:green", "tube": "goldenrod"}[nm], alpha=0.85, lw=0.4, ec="k")
    for e in s.discrete: ax[0].plot(e[:, 0], e[:, 2], "k", lw=0.5)
ax[0].set_aspect("equal"); ax[0].set_title("Kresling bellows, section y=0, neutral (H 20, twist 30 deg)"); ax[0].set_xlabel("x"); ax[0].set_ylabel("z")
phis = np.linspace(0, 60, 61); ax[1].plot(phis, [H_of_phi(b, R, p) for p in phis]); ax[1].axvspan(0, 60, alpha=0.08); ax[1].set_xlabel("twist phi (deg)"); ax[1].set_ylabel("height H (mm)"); ax[1].set_title("Twist-height coupling (b fixed): %.1f deg/mm at neutral" % abs(math.degrees(dphidH))); ax[1].grid(alpha=0.3)
P, Q = verts(n, R, H0, PHI0)
for k in range(n):
    k1 = (k + 1) % n; ax[2].plot([P[k][0], P[k1][0]], [P[k][1], P[k1][1]], "b-"); ax[2].plot([Q[k][0], Q[k1][0]], [Q[k][1], Q[k1][1]], "g-"); ax[2].plot([P[k][0], Q[k][0]], [P[k][1], Q[k][1]], "r--", lw=0.8)
ax[2].set_aspect("equal"); ax[2].set_title("Top view: A (blue) B (green) twisted by phi0=30 deg; creases dashed"); fig.tight_layout(); fig.savefig(os.path.join(OUT, "section.png")); plt.close(fig)
json.dump(res, open(os.path.join(OUT, "concept.json"), "w"), indent=1)
print(json.dumps(res["chosen"], indent=1)); print([ (r["n"], r["R"], r["t"], r["rom_pm_deg"], r["bend_Nm_per_deg"]) for r in res["sweep"][:9]])
