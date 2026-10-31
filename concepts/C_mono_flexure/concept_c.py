"""CONCEPT C: monolithic printed flexure on the W3 axis (x through y 57, z 455), yokeless, driven by the W3 servo through a crank r15 + push-rod.  Analytic comparison + cadquery model."""
import numpy as np, json, math, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "cad"))
HERE = os.path.dirname(os.path.abspath(__file__)); D, ZW3 = 57.0, 455.0
TH = math.radians(30); MAT = dict(PA12=dict(E=1700., G=620., eps_static=0.030, eps_end=0.008), PA_CF=dict(E=4000., G=1500., eps_static=0.010, eps_end=0.003))   # MPa; eps_end = assumed 1e6-cycle strain amplitude
K_FAT = 8.0                                                                                # assumed strain-life slope N = 1e6 (eps_end/eps)^k
F_PAY, LEVER, M_HAND = 0.5 * 9.81 * 2.0, 0.200, 0.5                                          # 0.5 kg payload at 2 g, 200 mm from the W3 axis; hand 0.5 kg
M_TIP = F_PAY * LEVER; F_AX = (0.5 + M_HAND) * 9.81 * 2.0
res = {}
def notch(kind, R, t, b, mat, ratio=1.0):
    E, G = MAT[mat]["E"], MAT[mat]["G"]
    a = R if kind == "circular" else 2.0 * R; c = R                                         # ellipse: semi-axis along the beam a = 2c
    x = np.linspace(-a, a, 4001); h = t + 2 * c * (1 - np.sqrt(np.clip(1 - (x / a) ** 2, 0, 1)))
    C = 12.0 / (E * b) * np.trapezoid(1.0 / h ** 3, x); k = 1.0 / C                              # N.mm/rad (Euler-Bernoulli; shear neglected)
    Kt = 1 + 2.0 * math.sqrt(t / (2 * c)) if kind == "circular" else 1 + 1.0 * math.sqrt(t / (2 * c))    # assumed stress-concentration fits
    eps = Kt * 6 * k * TH / (E * b * t ** 2)
    leff = 2 * a * 0.6                                                                       # effective bending length for shift / torsion estimates
    ex, ey = leff * math.sin(TH) / TH, leff * (1 - math.cos(TH)) / TH                         # constant-curvature end point
    ix, iy = leff / 2 + leff / 2 * math.cos(TH), leff / 2 * math.sin(TH)                      # ideal pin at mid-length
    shift = math.hypot(ex - ix, ey - iy)
    k_tors = G * b * t ** 3 / (3 * leff)                                                     # torsion about the beam axis (the hand roll axis z)
    th_max = TH * (MAT[mat]["eps_static"] / 2) / eps                                         # angle reaching eps_static/2 (SF 2)
    return dict(k_Nm_per_deg=round(k / 1e3 * math.pi / 180, 5), eps_pct_at_30deg=round(eps * 100, 2), theta_max_SF2_deg=round(math.degrees(th_max), 1), parasitic_shift_mm=round(shift, 2),
                roll_stiff_Nm_per_deg=round(k_tors / 1e3 * math.pi / 180, 4), roll_deflection_deg_at_0p35Nm=round(0.35e3 / k_tors * 180 / math.pi, 1))
def cross(L, t, b, mat, cross_at=0.5):
    E, G = MAT[mat]["E"], MAT[mat]["G"]; I = b * t ** 3 / 12; k = 2 * E * I / L                 # two leaves, N.mm/rad
    eps = t * TH / (2 * L)
    # PRBM crossed four-bar (leaf pins 0.075 L inside each end, link length 0.85 L): centre shift at 30 deg
    from scipy.optimize import fsolve
    l = 0.85 * L; a = l / (2 * math.sqrt(2)); P = [np.array([-a, -a]), np.array([a, -a])]; Q = [np.array([a, a]), np.array([-a, a])]
    Rm = np.array([[math.cos(TH), -math.sin(TH)], [math.sin(TH), math.cos(TH)]])
    f = lambda u: [np.linalg.norm(Rm @ Q[i] + u - P[i]) - l for i in range(2)]
    u = fsolve(f, [0.0, 0.0]); shift = float(np.linalg.norm(u))
    Pcr = 4 * math.pi ** 2 * E * I / L ** 2; Pleaf = F_AX / (2 * math.cos(math.radians(45)))   # per-leaf compression (fixed-fixed buckling)
    Fm = M_TIP * 1e3 / b / 2                                                                  # edge force per leaf from a tip moment about y (couple over the leaf width)
    dl = Fm * L / (E * t * b); defl = dl / b * 200.0                                          # tip deflection (mm) from leaf axial strain
    sig_cr = 4 * math.pi ** 2 * E / (12 * 0.91) * (t / L) ** 2; Fcr_edge = sig_cr * b * t     # plate local buckling, k=4
    k_shear = G * t * b / L; k_roll = 2 * k_shear * (L / 4) ** 2                              # roll (z) stiffness N.mm/rad
    return dict(k_Nm_per_deg=round(k / 1e3 * math.pi / 180, 5), eps_pct_at_30deg=round(eps * 100, 2), theta_max_SF2_deg=round(math.degrees(2 * L * MAT[mat]["eps_static"] / 2 / t), 1),
                parasitic_shift_mm=round(shift, 2), buckling_SF=round(Pcr / Pleaf, 1), payload_tip_defl_mm=round(defl, 2), plate_buckling_SF=round(Fcr_edge / Fm, 2),
                roll_stiff_Nm_per_deg=round(k_roll / 1e3 * math.pi / 180, 3), roll_deflection_deg_at_0p35Nm=round(0.35e3 / k_roll * 180 / math.pi, 2))
geo = {}
for mat in MAT:
    geo[mat] = dict(circular_notch=notch("circular", 8.0, 1.0, 20.0, mat), elliptical_notch=notch("elliptical", 8.0, 1.0, 20.0, mat), cross_strip=cross(24.0, 0.7 if mat == "PA12" else 0.45, 20.0, mat))
    for g, v in geo[mat].items():
        eps = v["eps_pct_at_30deg"] / 100; v["fatigue_cycles_1e6_at_30deg"] = float(f"{1e6 * (MAT[mat]['eps_end'] / eps) ** K_FAT:.2g}")
        v["strain_vs_static_limit"] = f"{eps*100:.2f} % vs {MAT[mat]['eps_static']*100:.0f} %"
res["geometries"] = geo
res["loads"] = dict(M_tip_Nm_2g=round(M_TIP, 2), axial_N_2g=round(F_AX, 1), crank_r_mm=15, rod_lug_radius_mm=39, mech_adv=round(39 / 15, 2), servo_deg_for_30deg_hand=round(30 * 39 / 15, 0),
                    w3_torque_available_Nm=round(2.94 * 39 / 15, 1))
res["assumptions"] = "eps_end (1e6 cycles) PA12 0.8 %, PA-CF 0.3 %, slope k=8: ASSUMED; Kt fits assumed; notch strain via Euler-Bernoulli integral; cross-strip PRBM shift; MJF/SLS PA12 (0.45-0.7 mm strips); FDM PA-CF cannot print 0.45 mm strips reliably."
res["correction_for_parent_v2"] = dict(note="wrist_rotary.py flexure leaves: the in-plane dimension perpendicular to x (12 mm) is the beam length L, the x extent (8 mm) is the width b (I had them swapped in verify_wrist_rotary.py).",
    corrected=dict(k_Nm_per_deg=round(2 * 190e3 * (8 * 0.1 ** 3 / 12) / 12 / 1e3 * math.pi / 180, 5), strain_pct_35deg=round(0.1 * math.radians(35) / (2 * 12) * 100, 3), buckling_per_leaf_N=round(4 * math.pi ** 2 * 190e3 * (8 * 0.1 ** 3 / 12) / 12 ** 2, 1)))
json.dump(res, open(os.path.join(HERE, "concept.json"), "w"), indent=1)
for mat in geo:
    for g, v in geo[mat].items(): print(mat, g, v)
print(res["loads"]); print(res["correction_for_parent_v2"]["corrected"])
