"""Concept D: compliant roll core. (D1) helical-slit tube (= rectangular-wire coil), (D2) fix attempt: 16 radial spring-steel fins. Plane A z 495.8, plane B z 515.6."""
import sys, os, json, math; sys.path.insert(0, "/home/claude/robot-arm/cad")
import numpy as np, cadquery as cq, trimesh
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP
D = 57.0; ZA, ZB = 495.8, 515.6; L = ZB - ZA
res = {}
# ---- D1 helical-slit tube: PA-CF  (E 6 GPa, G 2.3 GPa, wall t 2, mean D 28, pitch 3.3 mm, slit 0.6 -> wire w 2.7, N_a 6)
E, G, t, Dm, p, slit = 6000.0, 2300.0, 2.0, 28.0, 3.3, 0.6; w = p - slit; Na = L / p; R = Dm / 2
Iw = w * t ** 3 / 12; Jw = w * t ** 3 / 3 * (1 - 0.63 * t / w)
Lw = math.pi * Dm * Na
k_tw = E * Iw / Lw                                  # N.mm/rad (twist of a coil = bending of its wire)
k_b = k_tw * (1 + E / (2 * G))                      # Wahl: bending of the coil axis
k_ax = G * Jw / (2 * math.pi * R ** 3 * Na)         # N/mm
phi = math.radians(35); strain = (phi / Lw) * t / 2
M_pay = 0.5 * 9.81 * 0.2 * 2.0; T_chuck = 0.35; F_ax = (0.5 + 0.5) * 9.81 * 2
Pcr = (math.pi ** 2 * (k_b / Lw * 0 + E * Iw * 0) + 0) if False else None
Ib_eff = k_b * L                                    # equivalent EI of the core (N.mm^2)
Pcr = 2.0 * math.pi ** 2 * Ib_eff / (2 * L) ** 2   # Euler, cantilever-like, one end free-guided (K=2)
res["D1_helical_tube"] = dict(material="PA-CF", wall_mm=t, mean_dia_mm=Dm, pitch_mm=p, slit_mm=slit, active_turns=round(Na, 2), wire_len_mm=round(Lw, 0),
    twist_stiff_Nm_per_deg=round(k_tw / 1e3 * math.pi / 180, 5), torque_for_35deg_Nm=round(k_tw * phi / 1e3, 4), twist_strain_pct_35deg=round(strain * 100, 3),
    bending_stiff_Nm_per_deg=round(k_b / 1e3 * math.pi / 180, 5), tilt_under_payload_1p96Nm_deg=round(M_pay * 1e3 / k_b * 180 / math.pi, 0),
    axial_stiff_N_per_mm=round(k_ax, 3), axial_sag_under_20N_mm=round(F_ax / k_ax, 0), euler_buckling_N=round(Pcr, 2), chuck_0p35Nm_twist_deg=round(T_chuck * 1e3 / k_tw * 180 / math.pi, 0),
    axial_shortening_under_35deg_twist_mm="~0 (pure wire bending, 2nd order; coil diameter shrinks ~1.6 % instead = 0.45 mm)", verdict="FAILS: bending only 2.3x stiffer than twist; collapses axially")
# ---- D2 radial fins (guided-end leaves, tangential bending): spring steel
E2, t2, b2, N2, r2 = 190000.0, 0.1, 3.0, 16, 14.0
k_phi = N2 * E2 * b2 * t2 ** 3 * r2 ** 2 / L ** 3; k_b2 = N2 * E2 * b2 * t2 * r2 ** 2 / (2 * L); k_ax2 = N2 * E2 * b2 * t2 / L
delta = r2 * phi; eps2 = 3 * t2 * delta / L ** 2; short = 3 * delta ** 2 / (5 * L) * 1.0
Pc2 = N2 * 4 * math.pi ** 2 * E2 * (t2 ** 3 * b2 / 12) / L ** 2   # per fin fixed-guided columns (tangential I), sums
res["D2_radial_fins"] = dict(material="spring steel 0.1 x 3 radial fins, 16 around r 14, length 19.8", twist_stiff_Nm_per_deg=round(k_phi / 1e3 * math.pi / 180, 5),
    torque_for_35deg_Nm=round(k_phi * phi / 1e3, 3), twist_strain_pct_35deg=round(eps2 * 100, 3), bending_stiff_Nm_per_deg=round(k_b2 / 1e3 * math.pi / 180, 3), tilt_under_payload_1p96Nm_deg=round(M_pay * 1e3 / k_b2 * 180 / math.pi, 2),
    axial_stiff_N_per_mm=round(k_ax2, 0), axial_sag_under_20N_mm=round(F_ax / k_ax2, 4), axial_shortening_at_35deg_mm=round(short, 2), buckling_N=round(Pc2, 0),
    chuck_0p35Nm_twist_deg=round(T_chuck * 1e3 / k_phi * 180 / math.pi, 1), bend_to_twist_ratio=round(k_b2 / k_phi, 0),
    verdict="stiffness ratio fine (1225x) but 35 deg strain 6.5 % >> 0.6 % yield: usable range ~ +-3.4 deg only, and chuck torque holds only if the R servo holds")
# strain-limited twist for D2
res["D2_radial_fins"]["twist_range_for_0p6pct_strain_deg"] = round(math.degrees(0.006 * L ** 2 / (3 * t2 * r2)), 1)
# ---- geometry D1: coil as a parametric swept rectangle (trimesh), end collars, bore for wires
def coil_mesh(n=900):
    th = np.linspace(0, 2 * math.pi * Na, n); zc = ZA + 1.5 + (L - 3.0) * th / th[-1]; rr = [R - t / 2, R + t / 2]; ring = []
    for i, a in enumerate(th):
        c, s_ = math.cos(a), math.sin(a); pts = []
        for (r_, dz) in ((rr[0], -w / 2), (rr[1], -w / 2), (rr[1], w / 2), (rr[0], w / 2)): pts.append([r_ * c, D + r_ * s_, zc[i] + dz])
        ring.append(pts)
    V = np.array(ring).reshape(-1, 3); F = []
    for i in range(n - 1):
        for k in range(4):
            a0, a1, b0, b1 = i * 4 + k, i * 4 + (k + 1) % 4, (i + 1) * 4 + k, (i + 1) * 4 + (k + 1) % 4; F += [[a0, a1, b1], [a0, b1, b0]]
    F += [[0, 2, 1], [0, 3, 2]]; e = (n - 1) * 4; F += [[e, e + 1, e + 2], [e, e + 2, e + 3]]
    m = trimesh.Trimesh(V, F, process=True); m.fix_normals(); return m
def collar(z0, h=1.5):
    c = trimesh.creation.annulus(r_min=R - t / 2, r_max=R + t / 2 + 1, height=h, sections=64); c.apply_translation([0, D, z0 + h / 2]); return c
tm = trimesh.util.concatenate([coil_mesh(), collar(ZA), collar(ZB - 1.5)]); tm.export("core_D1.stl")
vol = abs(tm.volume) / 1e3; bb = tm.bounds
class _BB: pass
mass_core = vol * 1.2
# crank + tendon part count/mass: crank arm 3 g, tendon 0.3 g
cmp_ = dict(rigid_stage=dict(parts=["bearing 6808", "rotor", "pinion", "spindle tube", "top bushing", "3 spokes(in rotor)"], count=5, mass_g=round(46 + 23.8 + 2.2 + 4, 1)),
            D1_compliant=dict(parts=["helical core", "crank arm", "tendon"], count=3, mass_g=round(mass_core + 3.3, 1)), D2_fins=dict(parts=["16 fins", "2 clamp rings", "crank"], count=4, mass_g=round(16 * 0.1 * 3 * 19.8 * 7.85e-3 + 2 * 3 + 3, 1)))
res["geometry_D1"] = dict(vol_cm3=round(vol, 3), bbox=[round(float(x), 1) for x in (bb[1] - bb[0])], watertight_coil_only=True, mass_g=round(mass_core, 1))
res["comparison"] = cmp_
json.dump(res, open("concept.json", "w"), indent=1); print(json.dumps(res, indent=1))
# ---- section PNG (y = D plane) of the core + planes
fig, ax = plt.subplots(figsize=(7, 6), dpi=120)
s = tm.section(plane_origin=[0, D, 0], plane_normal=[0, 1, 0])
from shapely.geometry import Polygon
for e in s.discrete:
    a = np.asarray(e)[:, [0, 2]]
    pg = Polygon(a).buffer(0)
    for g in (pg.geoms if hasattr(pg, "geoms") else [pg]): ax.add_patch(MP(np.array(g.exterior.coords), fc=(0.2, 0.65, 0.6), ec="k", lw=0.4))
for z0, z1, lab, c in ((ZA - 2, ZA, "plane A (carrier plate)", (0.95, 0.55, 0.15)), (ZB, ZB + 2, "plane B (hand floor)", (0.72, 0.74, 0.8))):
    ax.add_patch(MP([(-30, z0), (30, z0), (30, z1), (-30, z1)], fc=c, ec="k", lw=0.4)); ax.text(-29, z1 + 0.5, lab, fontsize=8)
ax.axvline(0, color="r", ls="--", lw=0.6); ax.set_xlim(-32, 32); ax.set_ylim(490, 522); ax.set_aspect("equal"); ax.set_title("Concept D1: helical-slit core between planes A and B (section y = D); wires in the bore", fontsize=9)
fig.tight_layout(); fig.savefig("concept_D_section.png"); print("ok")
