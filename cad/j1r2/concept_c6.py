"""C6 Ventilated lattice cage: baseline plate + S1 base block unchanged, armour rebuilt as ribs/hoops + solid liner band at the S2 pinch zone."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, shapely.geometry as sg, shapely.ops as so
from clib import *
from scipy.ndimage import maximum_filter
from scipy.spatial import cKDTree
import concept_ref
def slabs(angles, half_w, z0, z1, L_=200.0):
    ms = []
    for a in angles:
        m = box(0, L_, -half_w, half_w, z0, z1); m.apply_transform(rot(np.radians(a), [0, 0, 1])); ms.append(m)
    return U(*ms)
def open_stats(shell_pts, mask):
    ins = mask.contains(shell_pts); cl = shell_pts[ins]; op = shell_pts[~ins]
    if len(op) == 0 or len(cl) == 0: return 0.0, 0.0
    d = cKDTree(cl).query(op)[0]; return float(len(op) / len(shell_pts)), float(2 * d.max())
def build():
    P = {}; REF = concept_ref.build(); info = {}
    P["plate_6061"] = REF["plate_6061"]; P["s1_base_block"] = REF["s1_base_block"]
    # ---- crown: ribs (3.6 mm deep, +1.8 outward) + hoops, solid liner band z -66..-50 (S2 pinch zone / yoke disc rim at z -56..-50)
    crown_o = hull(cyl("z", 73.5, -99, -50), box(-84, -10, -71, 71, -99, -50)); crown_i = hull(cyl("z", 71.7, -101, -52), box(-82.2, -10, -69.2, 69.2, -101, -52))
    crown_o2 = hull(cyl("z", 75.3, -99, -50), box(-85.8, -10, -72.8, 72.8, -99, -50))
    base = D(crown_o, crown_i, cyl("z", 66.5, -53, -48.0))                # baseline crown shell
    thick = D(crown_o2, crown_i, cyl("z", 66.5, -53, -48.0))
    ang = [18 + 36 * k for k in range(10)]
    rib_mask = U(slabs(ang, 2.75, -100, -50), box(-200, 200, -200, 200, -99.5, -95.5), box(-200, 200, -200, 200, -81, -77))   # 10 ribs 5.5 wide, hems z -99.5..-95.5, mid hoop z -81..-77
    liner = I(base, box(-200, 200, -200, 200, -66, -49))
    lat = I(thick, rib_mask); crown = U(lat, liner)
    P["cage_crown"] = dict(mesh=crown, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    # ---- keel: 8 radial ribs + 2 ring hoops on the dish bottom, solid lip band, solid plate feet
    keel_o = hull(cyl("z", 72.6, -115, -97), box(-84, -10, -70, 70, -115, -97)); keel_i = hull(cyl("z", 70.8, -113.2, -95), box(-82.2, -10, -68.2, 68.2, -113.2, -95))
    kshell = D(keel_o, keel_i)
    rings = U(cyl("z", 20, -116, -96, r_in=12), cyl("z", 47, -116, -96, r_in=41))
    kmask = U(slabs([22.5 + 45 * k for k in range(8)], 2.75, -116, -96), rings, cyl("z", 200, -116, -111, r_in=56), box(-200, 200, -200, 200, -103, -97))
    keel = U(I(kshell, kmask), box(-88, -80, -52, 52, -113, -105))
    P["cage_keel"] = dict(mesh=keel, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    # ---- spaulder: flowing ribs on the same obstacle floor (skin +12.5 max-filtered), 3 mm deep
    X, Y, H = top_surface(-208, -84, -58, 88, 4.0, 4.0)
    ramp = -54.0 - 0.62 * (-84.0 - X); floor = np.maximum(maximum_filter(H, size=4, mode="nearest") + 12.5, ramp); floor = gaussian_filter(floor, 2.0, mode="nearest")
    slab = height_solid(X, Y, floor, floor + 2.2, "z")
    cx, cy, rx, ry = -146.0, 15.0, 62.0, 70.0
    oval = sg.Polygon([(cx + rx * np.cos(a), cy + ry * np.sin(a)) for a in np.linspace(0, 2 * np.pi, 80, endpoint=False)])
    lines = []
    for k in range(-2, 3):                                               # fan of curved ribs, rear (plate edge) -> front lobes
        t = np.linspace(0, 1, 30); y0 = cy + 24 * k; y1 = cy + 28 * k; x = -86 + (-122) * t
        y = y0 + (y1 - y0) * t + 9 * k * np.sin(np.pi * t) * (1 if k else 0)
        lines.append(sg.LineString(np.c_[x, y]).buffer(2.5, cap_style=2))
    rim = oval.difference(oval.buffer(-3.5)); bands = [sg.box(-125, -119, -80, 110), sg.box(-165, -159, -80, 110)]
    pads = [sg.Point(-150, -62).buffer(12), sg.Point(-178, 14).buffer(12), sg.Point(-90, -40).buffer(7), sg.Point(-90, 40).buffer(7)]
    msk = so.unary_union(lines + [rim] + bands + pads).intersection(oval)
    geoms = list(msk.geoms) if hasattr(msk, "geoms") else [msk]
    def ex(g):
        m = trimesh.creation.extrude_polygon(g.simplify(0.3).buffer(0), 400); m.apply_translation([0, 0, -300]); solid_fix(m); return m
    ms_ = [ex(g) for g in geoms if g.geom_type == "Polygon" and g.area > 20]; print("mask pieces", len(ms_), [m.is_volume for m in ms_])
    mm = ms_[0] if len(ms_) == 1 else U(*[m for m in ms_ if m.is_volume])
    sp = I(slab, mm)
    P["cage_spaulder"] = dict(mesh=sp, mat="PACF", group="armour", colour=COL["pacf"], contact=False)
    # ---- opening statistics (sample outer wall points, nearest-rib distance = largest inscribed circle radius)
    rng = np.random.default_rng(1)
    def wall_pts(outer, zr, n=40000):
        s = trimesh.sample.sample_surface(outer, 3 * n)[0]; nrm = outer.face_normals[trimesh.sample.sample_surface(outer, 1)[1]] if False else None
        s = s[(s[:, 2] > zr[0]) & (s[:, 2] < zr[1])]; r = np.hypot(s[:, 0], s[:, 1]); s = s[(s[:, 0] < -5) | (r > 60)]; return s[:n]
    cp = wall_pts(crown_o2, (-98.5, -66.5)); f_c, d_c = open_stats(cp, rib_mask)
    kp = trimesh.sample.sample_surface(keel_o, 40000)[0]; kp = kp[kp[:, 2] < -103.5]; f_k, d_k = open_stats(kp, kmask)
    sp_area = float(msk.area); ov = float(oval.area); f_s = 1 - sp_area / ov
    # spaulder largest opening: inscribed circle in oval minus mask
    pts = np.array([(x, y) for x in np.arange(cx - rx, cx + rx, 1.5) for y in np.arange(cy - ry, cy + ry, 1.5) if oval.contains(sg.Point(x, y)) and not msk.contains(sg.Point(x, y))])
    bd = so.unary_union([msk.boundary]); d_s = max(bd.distance(sg.Point(p)) for p in pts[::3]) * 2
    info = dict(crown_open_frac=round(f_c, 3), crown_max_opening_mm=round(d_c, 1), keel_open_frac=round(f_k, 3), keel_max_opening_mm=round(d_k, 1),
                spaulder_open_frac=round(f_s, 3), spaulder_max_opening_mm=round(d_s, 1))
    return P, info
if __name__ == "__main__":
    P, info = build(); print(info)
    arm = {k: v for k, v in P.items() if v["group"] == "armour"}
    # --- calcs
    ref_arm = 161.6; ref_mount = 565.6
    m_arm = sum(part_info(k, v)["mass_g"] for k, v in arm.items())
    A_side = np.pi * 0.089 * 0.05025; A_bot = np.pi * 0.0445 ** 2; A = A_side + A_bot
    f_open = (info["crown_open_frac"] * 0.6 + info["keel_open_frac"] * 0.4)        # area-weighted guess: crown wall ~60 % of the enclosure around the motor
    U_closed, U_open, dT = 3.0, 7.5, 30.0
    Uef = U_closed + f_open * (U_open - U_closed)
    b, t_r, Lr, F, nrib = 5.5, 3.6, 49.0, 100.0, 3
    S = b * t_r ** 2 / 6; Mb = F / nrib * Lr; sig = Mb / S
    calcs = [dict(name="open-area fraction", formula="A_open / A_wall (sampled outer-wall points not inside the rib/hoop mask)", inputs="10 crown ribs 5.5 mm, 2 hoops 4 mm; 8 keel ribs + 2 rings",
                  result=f"crown (z -98.5..-66.5) {info['crown_open_frac']:.0%}, keel {info['keel_open_frac']:.0%}, spaulder {info['spaulder_open_frac']:.0%}"),
             dict(name="AK70 S1 housing cooling (ASSUMED h)", formula="Q = U_eff * A * dT ; U_eff = U_closed + f_open (U_open - U_closed)", inputs=f"A = {A*1e4:.0f} cm2 (side {A_side*1e4:.0f} + bottom {A_bot*1e4:.0f}), dT {dT} K, U_closed 3 W/m2K (enclosed, ASSUMED), U_open 7.5 W/m2K (free convection + radiation, ASSUMED 5-10), f_open {f_open:.2f}",
                  result=f"closed {3.0*A*dT:.1f} W -> cage {Uef*A*dT:.1f} W (+{(Uef/3.0-1):.0%}); order-of-magnitude only"),
             dict(name="rib bending, 100 N press at the rear rim (worst case, ribs alone)", formula="sigma = (F/n * L) / (b t^2/6)", inputs=f"F {F} N, n {nrib} ribs share, L {Lr} mm cantilever from the lower hoop, rib b {b} x t {t_r} mm (3.6 deep: 1.8 wall + 1.8 outward); spaulder ribs 2.2 deep",
                  result=f"M {Mb:.0f} N.mm, S {S:.1f} mm3, sigma {sig:.0f} MPa vs ~70 MPa PA-CF XY allowable (unverified): FAILS as isolated cantilevers; hoops/liner band (closed ring, hoop action) must carry it - needs FEA. Axial (down) press: {F/nrib/(b*t_r):.1f} MPa, OK")]
    extra = dict(
        summary="Armour redone as an open rib cage (10 crown ribs + 2 hoops, 8 keel ribs + 2 rings, 7-rib spaulder fan with rim, 2 cross lames, 4 solid pads) with a solid 16 mm liner band at the top of the crown over the S2 yoke-disc / brim pinch zone.",
        open_areas="OPEN (outer lattice): crown wall z -98.5..-66.5, keel dish, spaulder between ribs. CLOSED: crown liner band z -66..-50 incl. top brim (yoke disc rim z -56..-49.9), keel lip band z -103..-97, flange feet, spaulder rim/pads/lames.",
        mass=dict(armour_proxy_g=round(m_arm, 1), ref_armour_proxy_g=ref_arm, armour_delta_pct=round(100 * (m_arm / ref_arm - 1), 1), mount_proxy_g=round(m_arm + 278.8 + 125.2, 1), ref_mount_proxy_g=ref_mount,
                  delta_vs_ref_g=round(m_arm + 278.8 + 125.2 - ref_mount, 1), note="same proxy basis; spec armour 110.6 g -> scaled estimate %.0f g" % (110.6 * m_arm / ref_arm)),
        openings=info,
        opening_check="Largest inscribed through-opening (diameter, nearest rib sample distance x2): crown %.1f, keel %.1f, spaulder %.1f mm. All exceed 4 mm, so the 4 mm fingertip rule is met ONLY by geometry, not openings: behind the open crown/keel walls is static hardware (AK70 S1 housing, fixed base block, clamp) with no relative motion (S1 rotor/yoke disc sits in the closed band). Spaulder is >= 12 mm from the stage. Burn/touch hazard at the AK70 housing is not removed." % (info["crown_max_opening_mm"], info["keel_max_opening_mm"], info["spaulder_max_opening_mm"]),
        contract_changes=[], decision_conflicts=["Crown ribs protrude 1.8 mm outward vs the baseline wall (envelope +3.6 mm in x/y; skin/clasp clearances only gained). 4 mm fingertip rule not met by openings (justified by static hardware) - needs Forest/safety sign-off."],
        calcs=calcs,
        printability="Crown: print -z up as baseline (ribs vertical, hoops 4 mm bridge <= 25 mm between ribs OK). Keel: dish bottom plate becomes radial ribs + 2 rings, bridging <= 30 mm between ribs, print +x up per baseline. Spaulder: ribs 3 mm deep, print +y up. All fit 256 cube; rib width 5.5 mm = 13 perimeters of 0.4 (use 3 perimeters + solid), wall >= 1.4 mm gate: thinnest section 1.8 mm.",
        pros=["~%d %% less armour mass on the proxy basis" % round(-100 * (m_arm / ref_arm - 1)), "open path for AK70 S1 housing heat; more air to the shoulder-top skin", "keeps every interface, fastener and assembly step of rev b", "ribs follow the same floors so flush profile is kept"],
        cons=["rib-only bending failure case needs hoops/FEA", "exposes the hot/rotating-adjacent hardware to touch and debris", "dust/hair/cable snag path; loses the knight-armour closed read", "4 mm rule not met by openings"],
        risks=["thin-rib print quality in PA-CF (layer adhesion across 5.5 mm ribs)", "liner band transitions are stress risers", "heat gain estimate rests on assumed h values"],
        gates_to_rerun=["verify_armor (static/interference/wall p5/print/assembly)", "S1 and arm soft sweeps vs crown and spaulder", "harness_loads (mass)", "finger-access probe (ISO 13857) at crown liner edge", "thermal on the AK70 S1 housing (bench)"],
        assumptions=["armour proxies from concept_ref (spec envelopes, not verified meshes)", "h values and AK70 dissipation assumed", "rib allowable ~70 MPa PA-CF XY (unverified)", "seat F1/F4 positions assumed (clib.seats)"])
    meta = export_concept("C6", "Ventilated lattice cage", P, notes=dict(basis="proxy"), extra=extra)
    for r in meta["parts"]: print(r)
    print("mount mass", meta["mount_mass_g"], extra["mass"], extra["opening_check"]); print(calcs[1]["result"], calcs[2]["result"])
