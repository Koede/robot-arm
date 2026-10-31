"""Concept A: tendon-driven slotted TPU bend core (hinge-web stack) between plane A (z 495.8) and plane B (z 515.6). Bend axis = x (W3). Webs on +-x, tendons on +-y.
Model: 5... discs joined by N_G gaps; each gap keeps two webs (|y|<=W/2, annulus) = short beams (len G) bending about x. TPU 95A E 26 MPa, nu 0.45, rho 1.21. Dyneema d0.8 (E_eff 30 GPa braided, MBL ~450 N: PLACEHOLDER)."""
import cadquery as cq, trimesh, numpy as np, json, math, os
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP
from shapely.geometry import Polygon
D, Z0, Z1 = 57.0, 495.8, 515.6; L = Z1 - Z0
OD, ID = 34.0, 14.0; NG, G, W = 3, 3.4, 3.0; RT, RC = 13.0, 6.0
E, NU, RHO = 26.0, 0.45, 1.21; Gs = E / (2 * (1 + NU)); DR = (OD - ID) / 2
discs = [2.8, 2.0, 2.0, 2.8]; assert abs(sum(discs) + NG * G - L) < 1e-6, (sum(discs) + NG * G, L)
body = cq.Workplane("XY").circle(OD / 2).circle(ID / 2).extrude(L)
z = discs[0]
for i in range(NG):
    ring = cq.Workplane("XY").circle(OD / 2 + 1).circle(ID / 2 - 1).extrude(G).translate((0, 0, z))
    ring = ring.cut(cq.Workplane("XY").box(OD + 4, W, G + 2, centered=(True, True, False)).translate((0, 0, z - 1)))
    body = body.cut(ring); z += G + discs[i + 1]
for sy in (-1, 1): body = body.cut(cq.Workplane("XY").circle(0.6).extrude(L + 2).translate((0, sy * RT, -1)))        # d1.2 tendon holes
for sy in (-1, 1): body = body.cut(cq.Workplane("XY").circle(1.6).extrude(1.2).translate((0, sy * RT, L - 1.2)))    # knot pockets in the plane-B disc
body = body.translate((0, D, Z0)); cq.exporters.export(body, "core.stl", tolerance=0.02, angularTolerance=0.05)
m = trimesh.load("core.stl"); sol = body.val()
# ---- analytic model
I = DR * W ** 3 / 12; k_web = E * I / G; k_gap = 2 * k_web; k_th = k_gap / NG                    # N.mm/rad
k_deg = k_th / 1e3 * math.pi / 180
th_gap = math.radians(30) / NG; eps = 0.5 * W * th_gap / G                                         # outer-fibre strain of a web at +-30 deg
M = 0.5 * 9.81 * 0.2 * 2.0 * 1e3                                                                   # N.mm, 0.5 kg at 200 mm, 2 g
dF = M / (2 * RT); F0 = dF + 5; Fmax = F0 + dF; P_comp = 2 * F0
A_web = DR * W; k_ax = 2 * E * A_web / G / NG; sh = P_comp / k_ax
Lt = 80.0; EA = 30e3 * math.pi / 4 * 0.8 ** 2; kt = EA / Lt; d_t = dF / kt; th_load = 2 * d_t / (2 * RT)
F_lat = (0.5 + 0.33 + 0.17) * 9.81 * 2.0; k_sh = 2 * Gs * A_web / G / NG; lat = F_lat / k_sh
xc = (OD / 2 + ID / 2) / 2; k_tors = 2 * (Gs * A_web / G) * xc ** 2 / NG                           # N.mm/rad about z
vol = sol.Volume() / 1e3; mass = vol * RHO
res = dict(concept="A slotted TPU 95A bend core + 2 antagonistic Dyneema tendons", OD=OD, bore_ID=ID, length=L, gaps=NG, gap_height=G, web_w=W, web_radial=DR, disc_thk=discs, tendon_radius=RT, capstan_r=RC,
  bend_stiffness_Nm_per_deg=round(k_deg, 5), rom_deg=30.0, web_strain_pct_at_30deg=round(eps * 100, 2), tendon_preload_N=round(F0, 1), tendon_peak_N=round(Fmax, 1), tendon_dF_N=round(dF, 1),
  core_axial_load_N=round(P_comp, 1), core_shortening_mm=round(sh, 2), tip_deflection_deg_2g_payload_held=round(math.degrees(th_load), 2), tip_deflection_mm_at_200=round(200 * th_load, 1), lateral_sag_mm=round(lat, 2),
  capstan_torque_Nm=round(Fmax * RC / 1e3, 2), capstan_turn_deg_for_30=round(math.degrees(RT * math.radians(30) / RC), 0), servo_margin_vs_2p94=round(2.94 / (Fmax * RC / 1e3), 2),
  chuck_torsional_stiffness_Nm_per_deg=round(k_tors / 1e3 * math.pi / 180, 3), mass_g=round(mass, 1), mass_with_tendons_capstan_g=round(mass + 3.0, 1), wire_bore_mm=ID,
  watertight=bool(m.is_watertight), valid=bool(sol.isValid()), volume_cm3=round(vol, 3),
  print="axis horizontal (lying), gaps vertical, no supports except the 14 mm bore arch (teardrop it); webs 3 mm wide TPU: bench-test fatigue at 7.7% strain",
  caveats=["R rigid stage (z 496-503) occupies the core volume in wrist_rotary v2: it must move above plane B or the core must replace the rotor neck", "tendon E_eff, MBL and TPU E are assumptions", "core buckling stabilised by tendon pre-tension (tendon stiffness about x 1.1 N.m/deg)", "servo/capstan backlash not included"])
json.dump(res, open("concept.json", "w"), indent=1); print(json.dumps(res, indent=1))
# ---- sections
fig, axs = plt.subplots(1, 2, figsize=(12, 6), dpi=120)
for ax, (nm, org, nrm, ix) in zip(axs, (("xz (y = D)", [0, D, 0], [0, 1, 0], 0), ("yz (x = 0)", [0, 0, 0], [1, 0, 0], 1))):
    s = m.section(plane_origin=org, plane_normal=nrm)
    for e in s.discrete:
        e = np.asarray(e); pts = np.column_stack([e[:, ix], e[:, 2]]); p = Polygon(pts).buffer(0)
        for g in (p.geoms if hasattr(p, "geoms") else [p]):
            if not g.is_empty: ax.add_patch(MP(np.array(g.exterior.coords), closed=True, fc=(0.2, 0.65, 0.6), ec="k", lw=0.5))
    c = 0 if ix == 0 else D
    ax.axhline(Z0, color="orange", lw=1); ax.axhline(Z1, color="g", lw=1, ls="-."); ax.text(c - 21, Z0 - 1.5, "plane A z 495.8", color="orange", fontsize=8); ax.text(c - 21, Z1 + 0.8, "plane B (mating) z 515.6", color="g", fontsize=8)
    if ix == 1:
        for sy in (-1, 1): ax.plot([D + sy * RT] * 2, [Z0 - 14, Z1], color="b", lw=1.2); ax.text(D + sy * RT + 0.6, Z0 - 13, "tendon", color="b", fontsize=8)
        ax.text(D - 17, Z0 - 10, "to W3 capstan r6", fontsize=8, color="b")
    ax.set_xlim(c - 24, c + 24); ax.set_ylim(Z0 - 16, Z1 + 6); ax.set_aspect("equal"); ax.set_title(f"Concept A slotted TPU core, section {nm}", fontsize=9); ax.grid(alpha=0.2)
fig.tight_layout(); fig.savefig("section.png"); print("saved")
