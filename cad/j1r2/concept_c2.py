"""C2 Triangulated spaulder truss: PA-CF boxed hoop (plate top edge -> F1 / F4) carries part of the S1/S2 reaction; plate shrinks to a doubler. PROXY, static only."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, shapely.geometry as sg
from shapely.ops import unary_union
from scipy.ndimage import maximum_filter
from clib import *
import concept_ref as R

def plate_doubler_outline():
    from scipy.spatial import ConvexHull
    pts = []; circ = lambda cy, cz, r: [(cy + r * np.cos(a), cz + r * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 24, endpoint=False)]
    for (y, z) in S1_BASE_BLOCK["bolts"]: pts += circ(y, z, 9.0)               # M5 pattern, 9 mm margin
    for cy in (-50.0, 50.0): pts += circ(cy, -109.0, 10.0)                       # keel feet (unchanged)
    pts += [(-82, -54), (-46, -54), (-82, -112), (-46, -112), (72, -54), (-82, -54)]   # tongue seat truncated to z -76..-112 (contract change), top edge z -54
    pts = np.array(pts); h = pts[ConvexHull(pts).vertices]; return [tuple(p) for p in h]

def build():
    P = {}; s = seats(); F1, F4 = s["F1"][0], s["F4"][0]
    # ---- plate doubler (6061, 8 mm)
    plate = prism(plate_doubler_outline(), "x", PLATE_X[0], PLATE_X[1])
    for (y, z) in S1_BASE_BLOCK["bolts"]: plate = D(plate, cyl("x", 2.1, -90, -78, c=(y, z)))
    for ty in (-40.0, 40.0): plate = D(plate, cyl("z", 1.25, -64.0, -53.0, c=(-84.0, ty)))     # existing 2 x M3 tapped top-edge holes (contract kept)
    P["plate_doubler_6061"] = dict(mesh=plate, mat="AL", group="mount", colour=COL["al"])
    # ---- reuse REF base block, crown, keel
    ref = R.build()
    for k in ("s1_base_block", "armour_crown", "armour_keel"): P[k] = ref[k]
    # ---- structural hoop: height-field slab (10 mm, boxed 2 mm walls -> fill 0.55) on the obstacle floor, cut to a plan frame
    X, Y, H = top_surface(-205, -72, -85, 62, 3.0, 3.0)
    ramp = -54.0 - 0.62 * (-84.0 - X)
    floor = np.maximum(maximum_filter(H, size=5, mode="nearest") + 11.0, ramp); floor = gaussian_filter(floor, 2.0, mode="nearest")
    slab = height_solid(X, Y, floor, floor + 10.0, "z")
    A_med, A_lat = (-84.0, -48.0), (-84.0, 48.0); f1, f4 = tuple(F1[:2]), tuple(F4[:2])
    lines = [sg.LineString([A_med, A_lat]), sg.LineString([A_med, f1]), sg.LineString([A_lat, f4]), sg.LineString([f1, f4])]
    frame = unary_union([l.buffer(11.0, cap_style=1) for l in lines] + [sg.Point(f1).buffer(14.0), sg.Point(f4).buffer(14.0)])
    frame = frame.intersection(sg.box(-205, -72, -85, 62))
    hoop = I(slab, prism(list(frame.exterior.coords), "z", -300, 100))
    clip = U(box(-95, -88.3, -52, 52, -67, -54), box(-79.7, -73, -52, 52, -67, -54))   # U-clip wraps the plate top edge (fore-aft capture)
    hoop = U(hoop, clip)
    for nm, F in (("F1", F1), ("F4", F4)):                                              # seat pads D24 x 5 on the clasp bosses
        zb = F[2] + 4.0; zt = float(floor[np.argmin(abs(X[:, 0] - F[0])), np.argmin(abs(Y[0] - F[1]))]) + 1.0
        P["seat_pad_" + nm] = dict(mesh=cyl("z", 12.0, zb, zt, c=(F[0], F[1])), mat="PACF", fill=0.9, group="mount", colour=COL["accent"], contact=True)
    P["hoop_truss"] = dict(mesh=hoop, mat="PACF", fill=0.55, group="mount", colour=COL["teal"])
    return P, hoop, F1, F4

if __name__ == "__main__":
    P, hoop, F1, F4 = build()
    REF_MASS = 565.6
    # ---- static clearances: hoop away from the seats (>= 10), near seats (>= 2.5), neck column
    t, pts, nrm = L._tree(); smp = trimesh.sample.sample_surface(hoop, 20000)[0]; d, i = t.query(smp); sd = np.sign(np.einsum("ij,ij->i", smp - pts[i], nrm[i])) * d
    near = (np.linalg.norm(smp[:, :2] - F1[:2], axis=1) < 35) | (np.linalg.norm(smp[:, :2] - F4[:2], axis=1) < 35)
    clip_zone = smp[:, 0] > -100
    gap_far = float(sd[~near & ~clip_zone].min()); gap_near = float(sd[near].min()); gap_clip = float(sd[clip_zone].min())
    V = body().vertices; neck = V[(V[:, 1] < -105) & (V[:, 2] > -100) & (V[:, 0] < -120)]
    from scipy.spatial import cKDTree
    dn = float(cKDTree(neck).query(smp)[0].min())
    # ---- statics (planar, sagittal; S2 cap reaction + weight)
    m_stage = 3.0; Fw = m_stage * 9.81; lev = 0.080; Mw = Fw * lev; My = 8.3 + Mw      # N, m, N.m
    couple_ref = My / 0.026                                                           # REF top bolt pair tension (z spacing 26 mm)
    E, b, h, t_w = 5000.0, 22.0, 10.0, 2.0                                              # MPa, mm
    Ix = (b * h**3 - (b - 2 * t_w) * (h - 2 * t_w)**3) / 12.0; Ae = b * h - (b - 2 * t_w) * (h - 2 * t_w)
    Lleg = float(np.hypot(F1[0] + 84, F1[1] + 48)); Lleg2 = float(np.hypot(F4[0] + 84, F4[1] - 48))
    Fd = 50.0; M_leg = Fd * Lleg; sig = M_leg * (h / 2) / Ix; defl = Fd * Lleg**3 / (3 * E * Ix)
    A_top = 56.8 / 14.4e-3                                                              # mm2: 5.79 kg*9.81 / 14.4 kPa (derived, N_top = W model)
    dF_margin = (14.7 - 14.28) * 1e-3 * A_top                                          # N of extra compression on the top pad before the gate fails
    lever_seat = 0.078; s_max = dF_margin * lever_seat / My                            # max moment fraction through seats in compression
    k_hoop = 2 * E * Ae / ((Lleg + Lleg2) / 2)
    calcs = [
      dict(name="Reaction moment at plate", formula="My = 8.3 + m g l", inputs="S2 cap 8.3 N.m; m_stage+arm 3.0 kg (assumed); l 0.080 m", result=f"Fw {Fw:.1f} N, Mw {Mw:.2f} N.m, My {My:.2f} N.m"),
      dict(name="REF top bolt pair", formula="T = My / dz", inputs="dz 26 mm (M5 pattern z -68/-94)", result=f"{couple_ref:.0f} N total on the top pair (all through the wing tongue)"),
      dict(name="Hoop leg section", formula="I = (b h^3 - b' h'^3)/12, A = b h - b' h'", inputs="b 22, h 10, wall 2 mm", result=f"I {Ix:.0f} mm4, A {Ae:.0f} mm2"),
      dict(name="Leg bending, vertical 50 N at the seat end", formula="sigma = F L (h/2)/I ; delta = F L^3/(3 E I)", inputs=f"F 50 N, L {Lleg:.0f} mm, E 5000 MPa (annealed PA-CF XY, assumed)", result=f"sigma {sig:.1f} MPa (PA-CF ~80 MPa XY, interlayer lower -> SF ~{80/sig:.1f} in-plane), delta {defl:.2f} mm"),
      dict(name="Leg axial stiffness", formula="k = 2 E A / L", inputs="two legs", result=f"{k_hoop:.0f} N/mm: far stiffer than the soft-tissue seat (tens of N/mm), so the seat/clasp compliance sets the share"),
      dict(name="Top-pad headroom", formula="dF = (14.7-14.28 kPa) * A_top ; A_top = W/p", inputs="W 56.8 N, p 14.4 kPa -> A_top 3944 mm2 (derived)", result=f"A_top {A_top:.0f} mm2; dF {dF_margin:.1f} N extra compression allowed (0.25 kPa per N)"),
      dict(name="Load shared to the seats", formula="s = dF * l_seat / My", inputs="l_seat 78 mm (mean plate-edge-to-seat x)", result=f"compression share <= {100*s_max:.1f} % of My before the top-pad gate fails; tension (lift) share is limited by the strap preload, not computed"),
    ]
    extra = dict(
      summary="PA-CF boxed hoop (22 x 10 mm legs, 2 mm walls) from the plate top edge to F1/F4 closes a plate-F1-F4 triangle; plate shrinks to a 153 x 65 doubler.",
      mass_delta_note="same proxy basis as REF 565.6 g",
      assumptions=["F1 (-150,-62) and F4 (-178,14) are ASSUMED seat centres (clasp_sites.json not in the project)", "wing/clasp/rail are generic proxies", "stage+arm 3.0 kg at 80 mm; E 5 GPa; fill 0.55 for a boxed 2 mm wall section; A_top derived from the N_top = W model", "REF proxy armour is ~50 g heavier than spec (161.6 vs 110.6 g); crown/keel reused unchanged"],
      contract_changes=["plate outline 158 x 100 -> 153 x 65: wing tongue seat truncated from z -76..-150 to z -76..-112 (wing lap/bolts must move up 38 mm; re-run scapular_frame)", "spaulder rear feet (2 x M3 x 10 vertical into the plate top edge) kept as the hoop's roof screws, plus a U-clip capture", "hoop becomes load-bearing on clasp seats F1/F4: seat M4 x 12 now carry structural load, pad/boss and insert pull-out need a re-check"],
      decision_conflicts=["Top pad is the binding harness gate (14.28 vs 14.7 kPa, 2.9 %% margin): 1 N of extra seat compression = +0.25 kPa, so the headroom is only ~%.1f N (%.1f %% of My). The hoop can carry the load only as tension/lift or a tiny compressive share; it is not a general load path into F1/F4." % (dF_margin, 100 * s_max),
                         "Nothing rigid on moving skin: respected (seats and top only; static gap away from seats %.1f mm, near seats %.1f mm)" % (gap_far, gap_near),
                         "Flush request (2026-10-30): hoop top sits ~21 mm over skin (11 gap + 10 section), about 8 mm higher than the 1.8 mm REF spaulder"],
      calcs=calcs,
      static_checks=dict(skin_gap_far_from_seats_mm=round(gap_far, 1), skin_gap_within_35mm_of_seats_mm=round(gap_near, 1), gap_clip_zone_mm=round(gap_clip, 1), min_dist_to_neck_column_mm=round(dn, 1),
                         plate_envelope=[153, 65, 8], hoop_bbox=[round(float(x), 1) for x in hoop.bounds[1] - hoop.bounds[0]]),
      printability="hoop ~%d x %d x %d mm fits the 256 cube; print flat (z up) so leg bending stress lies in the layer plane; supports under the clip slot and the sloped underside (est. 10-15 cm2, not computed); 4 perimeters, 55 %% effective fill" % tuple(hoop.bounds[1] - hoop.bounds[0]),
      pros=["Triangulates the plate: lower tongue/wing bending demand in the sagittal plane (wing share not quantified)", "Plate 278.8 g -> doubler; one structural PA-CF member replaces the shell spaulder", "Gravity moment tends to put the legs in tension, which unloads the top pad slightly"],
      cons=["Can't take compressive load without breaking the 14.7 kPa top-pad gate", "Higher profile than the flush REF spaulder", "Truncated tongue forces a wing change", "Open frame leaves the shoulder top uncovered unless a skin is added"],
      risks=["Clasp seat bosses/inserts not rated for structural loads", "PA-CF interlayer strength and creep unmeasured", "Hoop stiffness could act as a hard point on trapezius under strap tension", "Neck lateral-flexion clearance only a static distance, not swept"],
      gates_to_rerun=["verify_harness / harness_loads (top pad)", "body_loads (F1/F4 cell peaks)", "verify_armor (skin 10 mm, ROM abduction/scaption/neck lateral flexion 50 deg)", "scapular_frame (tongue seat)", "chassis_axilla (plate outline)", "print_table"],
    )
    meta = export_concept("C2", "Triangulated spaulder truss", P, notes=dict(ref_mount_mass_g=REF_MASS), extra=extra)
    extra["mass_delta_vs_REF_g"] = round(meta["mount_mass_g"] - REF_MASS, 1)
    import json; json.dump(dict(meta, extra=extra), open(os.path.join(ROOT, "stl/concepts/C2/metrics.json"), "w"), indent=1, default=float)
    for r in meta["parts"]: print(r["name"], r["mass_g"], r["watertight"], r.get("skin_min_mm"))
    print("mount", meta["mount_mass_g"], "delta", extra["mass_delta_vs_REF_g"], extra["static_checks"]); [print(c["name"], "|", c["result"]) for c in calcs]
