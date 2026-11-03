"""J1 pancake holder, round 3 (2026-11-02, SHOULDER / WING, concept): MJ11 "blade-seated pancake".
Forest: "make the shoulder pancake mount/holder be flat facing against the shoulder blade so the large flat bit is up against the back".
He chose (AskUserQuestion): rides on the blade (OVERRIDE of "nothing rigid on skin that moves with the scapula" for this concept only), work read-only (no push).
Findings behind the geometry (mj11_scan.py): at the J1 rev 2 pad (+10, -30) the pad face hovers 50..67 mm (mean 62) off the skin because the S2 sweep (radius 154) crosses the blade bulge,
so a flat pad can only lie on the back if the S1' axis is moved onto the blade itself.  MJ11 sits the pad at (y1, z1) = (-60, -190) (J1_PAD env, standoff 12 = liner 8 + 4 clearance over the ridge):
  * pad disc R66 x 6 (unchanged part), its FLAT front face x0 parallel to the S1' plane;
  * seat dish: a conformal PA-CF shell whose outer face is the flat plane x0 and whose inner face follows the skin + 8 mm liner (variable thickness, height field, prints flat face down, no supports);
  * liner: 8 mm closed-cell foam with a PTFE-fabric slip face (the skin travels 56-96 mm under it) and 3 radial lead channels for the electrode flex leads;
  * the pad's medial 24 mm overlaps the paraspinal rail: 4 x M4 straight into the rail through 2 spacer pairs (no riser, no knee, no truss).
Proxy geometry (CAD-as-code primitives).  PLACEHOLDER: AK70 flange pattern on the pad, rail boss positions (new M4 inserts), foam and slip-fabric grades."""
import numpy as np, trimesh, os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clib as L, mj_frame as M, mj_concepts as C
from clib import U, D as DIFF, box as bx, rot
from mj_concepts import P_, pad_disc, pad_pt, link_bolts, rail_node_name, rail_x, clasp_bracket_geo, clasp_bracket_frame, boxbeam, BLUE, PAD_R
Y1, Z1, X0, XB = M.Y1, M.Z1, M.X0, M.XB
LINER_T = 8.0                      # foam liner thickness (compliance over the moving blade)
PLATE_MIN = 3.0                    # minimum dish thickness at the ridge
RAIL_EDGE_Y = -96.0                # dish and liner stop 4 mm lateral of the rail edge (rail y -138..-100)
PAD_TRIM_Y = -121.0                # pad disc trimmed flat on its medial side: clears the real spine-bridge ears by >= 3 mm
BOLT_Z = (-160.0, -220.0)          # rail node heights of the two bolt pairs (rail nodes are every 20 mm)
BOLT_Y = (-113.0, -105.0)          # PLACEHOLDER positions of the new M4 bosses on the rail rear face
CLASP_PSI = float(os.environ.get("MJ11_CLASP_PSI", "120"))     # clasp bracket anchor on the pad rim (deg from +y toward +z)
LEAD_PSI = (225.0, 270.0, 315.0)   # lead channels through the liner (flex electrode leads, SPEC_flex_electrode_leads)
FOAM_DENS = 0.12                   # g/cm3 closed-cell EVA / PE foam (assumption)
_G = {}
def _grid(step=3.0):
    if "g" in _G: return _G["g"]
    R = PAD_R + 4.0; Y, Z, X = L.rear_surface(Y1 - R, Y1 + R, Z1 - R, Z1 + R, step, 1.0); _G["g"] = (Y, Z, X); return _G["g"]
def _domain(thick=60.0):
    """disc R66 about the S1' axis, cut at the rail edge (the pad overlaps the rail medially; the rail sits between the skin and the pad plane there)."""
    d = L.cyl("x", PAD_R, X0 - 80.0, X0 + 2.0, c=(Y1, Z1), n=96); return L.I(d, bx(X0 - 90.0, X0 + 10.0, RAIL_EDGE_Y, Y1 + 200.0, Z1 - 200.0, Z1 + 200.0))
def seat_surfaces():
    Y, Z, X = _grid(); skin = X.copy(); inner = skin + LINER_T; plane = np.full_like(skin, X0)
    assert float((plane - inner).min()) >= PLATE_MIN - 1e-6, "dish thinner than PLATE_MIN: raise the standoff"
    return Y, Z, skin, inner, plane
def dish_mesh():
    Y, Z, skin, inner, plane = seat_surfaces(); return L.I(L.height_solid(Y, Z, inner, plane, "x"), _domain())
def liner_mesh():
    Y, Z, skin, inner, plane = seat_surfaces(); ln = L.I(L.height_solid(Y, Z, skin - 0.0, inner, "x"), _domain())
    cuts = []
    for psi in LEAD_PSI:
        a = math.radians(psi); p0 = np.array([X0 - 40.0, Y1 + 18.0 * math.cos(a), Z1 + 18.0 * math.sin(a)]); p1 = np.array([X0 - 40.0, Y1 + 80.0 * math.cos(a), Z1 + 80.0 * math.sin(a)])
        cuts.append(boxbeam(p0, p1, 8.0, 60.0, None, up=(1, 0, 0), ext=0.0))
    return DIFF(ln, *cuts)
def rail_bolt_pts():
    """(x_pad_face, y, z) of the 4 M4 bolts on the pad plane and the rail rear face x at the same place."""
    return [(X0, y, z, rail_x(-119.0, z)) for z in BOLT_Z for y in BOLT_Y]
def mj11_geo(base):
    out = {}; padm = L.I(pad_disc(), bx(X0 - 10.0, XB + 10.0, PAD_TRIM_Y, Y1 + 100.0, Z1 - 100.0, Z1 + 100.0))
    out["pad"] = P_("pad", padm, "PACF", BLUE, note="R66 x 6 PA-CF pad disc trimmed flat at y %.0f (spine-bridge ears), S1' AK70 on its rear face (flange PLACEHOLDER); its flat front face lies on the seat dish and, medially, on the rail spacers" % PAD_TRIM_Y)
    dish = dish_mesh(); out["seat_dish"] = P_("seat_dish", dish, "PACF", BLUE, fill=0.30, note="conformal PA-CF shell: outer face the flat plane x0, inner face the skin + %.0f mm liner; variable thickness %.1f..%.1f mm (drawn solid, mass at the hollow fill 0.30: 2.4 mm walls + ribs); prints flat face down, no supports" % (LINER_T, PLATE_MIN, float((X0 - seat_surfaces()[3]).max())))
    ln = liner_mesh(); out["liner"] = P_("liner", ln, "TPU", (0.85, 0.85, 0.88), kind="bought", mass_g=round(abs(ln.volume) / 1000.0 * FOAM_DENS, 1), note="8 mm closed-cell foam (0.12 g/cm3 assumed) + PTFE-fabric slip face toward the plate side of the base layer; 3 radial lead channels 8 mm wide (psi 225/270/315)")
    sp = U(*[L.cyl("x", 5.0, rail_x(-119.0, z) - 0.2, X0 + 0.2, c=(y, z), n=24) for (_, y, z, _) in [(0, y, z, 0) for z in BOLT_Z for y in BOLT_Y]])
    out["rail_spacers"] = P_("rail_spacers", sp, "PETGCF", C.COL["steel"], note="4 spacers D10 from the rail rear face to the pad plane (%.1f..%.1f mm), M4 x 20 SHCS into new M4 heat-set inserts (5.6 OD x 4) in the rail (PLACEHOLDER bosses)" % tuple(sorted((X0 - rail_x(-119.0, z)) for z in BOLT_Z)))
    out["liner"]["skin_contact"] = True
    fb, fc = fairing_geo(); out["fairing_body"] = P_("fairing_body", fb, "PACF", (0.93, 0.55, 0.12), note="1.8 mm PA-CF shell around the pad rim (R66 trimmed y %.0f, flush with the dish plane x0) and the S1' AK70 / stop posts: front ring + the wall sectors the S1' sweep (%.0f..%.0f + 2.5) leaves, 3 pad tabs (psi 225/270/315, M3 x 8, pad inserts PLACEHOLDER), 3 cover lugs, cable gland D%.1f at psi %.0f; prints x0 face down, no supports" % (PAD_TRIM_Y, FAIR_TH[0], FAIR_TH[1], GLAND_D, GLAND_PSI))
    out["fairing_cover"] = P_("fairing_cover", fc, "PACF", (0.93, 0.55, 0.12), note="1.8 mm rear cover disc R68 with the swept S1'/S2 sleeve slot and a 4 mm spigot into the body; 3 x M3 x 8 into the body lugs; prints flat")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(pad_pt(CLASP_PSI, PAD_R - 8, x=XB - 3.0)), "PACF", BLUE, note="clasp feet (re-seated) to the pad rim at psi %.0f" % CLASP_PSI)
    return out
# ================================================================== fixed fairing for MJ11 (2026-11-03): 1.8 mm PA-CF, S1' AK70 + pad rear / sides, NO riser, NO lap
FAIR_WALL = 1.8                    # PA-CF shell wall
FAIR_PADGAP = 0.2                  # print clearance between the pad rim and the shell inner face
FAIR_GAP = 3.2                     # swept clearance cut around every moving profile (>= 3 mm gate + sweep faceting)
FAIR_TH = tuple(float(v) for v in os.environ.get("MJ11_FAIR_TH", "-100,40").split(","))      # S1' range the slot is swept over = J1 rev 2 hard range -100..40 (covers the MJ11 passive box -95..5 / -80..5 plus the 5 deg stops); -100,10 costs reach-free 0.786 -> 0.706 and +8 g; 2.5 deg stations
FAIR_REAR = 31.0                   # rear cover outer face at XO + 31 (same as J1 rev 2: ribbon coil at XO + 17.8 keeps >= 7 mm)
FAIR_TAB_PSI = (225.0, 270.0, 315.0)   # 3 pad tabs (M3 into the pad rear face, PLACEHOLDER inserts) on the lower rim, clear of the stop-collar lug sweep
FAIR_LUG_PSI = (85.0, 125.0, 165.0)    # 3 internal lugs on the wall sector for the rear cover (M3)
GLAND_PSI, GLAND_D = 190.0, 13.2       # cable gland through the medial flat (bundle OD 8.2 + 2 x 2.5 grommet), axis along y; open, harness side not routed yet (PLACEHOLDER)
def _fair_cut(th_lo, th_hi):
    """union of the M1 profiles that cross the cup wall (S2 clamp cylinder, its square flange, yoke bar), swept over S1' with FAIR_GAP all round (as j1r2_geom 'fairing_fixed')."""
    import j1r2_params as P, j1r2_geom as G
    Dv = M.DER; PX, y1, z1 = Dv["PX"], Dv["y1"], Dv["z1"]; TS, AKR, g = P.TS, P.AKR, FAIR_GAP; cut = []
    for th in np.arange(th_lo - 2.5, th_hi + 2.6, 2.5):
        Mx = G.pose_matrix("M1", Dv, th, 0.0)
        sl = L.cyl("y", 48.5 + g, 30.0 + y1, 130.0 + y1, c=(PX, z1), n=48); sl.apply_transform(Mx); cut.append(sl)
        bar = bx(PX - 6.0 - g, PX + 6.0 + g, y1 + 0.0, y1 + TS + 3.0, z1 - 26.0 - g, z1 + 26.0 + g); bar.apply_transform(Mx); cut.append(bar)
        pl_ = bx(PX - 48.5 - g, PX + 48.5 + g, y1 + TS - 1.3 - g, y1 + TS + 3.7 + g, z1 - 48.5 - g, z1 + 48.5 + g); pl_.apply_transform(Mx); cut.append(pl_)
    return L.U(*cut)
FAIR_INFO = {}
def _clean(m, vmin=50.0):
    """drop boolean debris (components under 50 mm3, e.g. 1 mm3 slivers where a swept slot edge grazes a lug)."""
    parts = [p for p in m.split(only_watertight=False) if abs(p.volume) > vmin]; return parts[0] if len(parts) == 1 else trimesh.util.concatenate(parts)
def sg_Polygon(p):
    import shapely.geometry as sg; return sg.Polygon(p)
def fairing_geo(split=True):
    """returns (body, cover) meshes.  body: front ring (flush with the dish plane x0, wraps the pad rim R66 trimmed at y -121) + the wall sectors the S1' sweep leaves + 3 pad tabs +
    3 cover lugs + gland; prints x0 face down, no supports.  cover: flat rear disc (prints flat), swept slot cut through it, spigot into the body."""
    Dv = M.DER; x0, XB, XO, PX = Dv["x0"], Dv["XB"], Dv["XO"], Dv["PX"]; y1, z1 = Y1, Z1; Ro = PAD_R + FAIR_PADGAP + FAIR_WALL; Ri = PAD_R + FAIR_PADGAP; w = FAIR_WALL
    x_end = XO + FAIR_REAR; x_c = x_end - w                                   # cover plate x_c .. x_end
    half = lambda a0, a1, y_min: bx(a0, a1, y_min, y1 + 300.0, z1 - 300.0, z1 + 300.0)
    outer = L.I(L.cyl("x", Ro, x0, x_end, c=(y1, z1), n=128), half(x0 - 1, x_end + 1, PAD_TRIM_Y))                       # medial flat: outer face = the pad's trim line (the bridge ears clear it by 3.3)
    void = L.I(L.cyl("x", Ri, x0 - 1.0, x_end - w, c=(y1, z1), n=128), half(x0 - 2, x_end, PAD_TRIM_Y + w))
    shell = DIFF(outer, void)
    padg = L.I(L.cyl("x", PAD_R + FAIR_PADGAP, x0 - 1.0, XB + FAIR_PADGAP, c=(y1, z1), n=128), half(x0 - 2, XB + 1, PAD_TRIM_Y - FAIR_PADGAP))
    shell = DIFF(shell, padg)                                               # nothing on the pad itself: the medial wall starts behind the pad rear face
    # pad tabs: inward ledge on the pad rear face at the lower rim (3 x M3 x 8 into pad rear-face inserts, PLACEHOLDER)
    tabs = []
    for psi in FAIR_TAB_PSI:
        a = math.radians(psi); u = np.array([math.cos(a), math.sin(a)]); c = np.array([y1, z1]) + u * (Ri - 4.0)
        t = L.cyl("x", 7.0, XB + FAIR_PADGAP, XB + FAIR_PADGAP + w, c=tuple(c), n=40); t = L.U(t, L.cyl("x", 7.0 + 0.0, XB + FAIR_PADGAP, XB + FAIR_PADGAP + w, c=tuple(np.array([y1, z1]) + u * (Ri - 1.0)), n=40))
        t = DIFF(t, L.cyl("x", 1.7, XB - 1, XB + 4, c=tuple(np.array([y1, z1]) + u * (Ri - 6.0)), n=24))                    # M3 clearance hole 3.4
        tabs.append(t)
    shell = L.U(shell, L.I(L.U(*tabs), outer))                              # tabs are clipped to the outer envelope (not proud of the shell)
    # rear-cover lugs (inner, on the wall at the cover plane), M3 tapped into the lug
    lugs = []
    for psi in FAIR_LUG_PSI:
        a = math.radians(psi); u = np.array([math.cos(a), math.sin(a)]); c = np.array([y1, z1]) + u * (Ri - 4.5)
        cone = trimesh.creation.cone(5.5, 5.5, sections=32, transform=None); cone.apply_transform(rot(math.pi, [1, 0, 0])); cone.apply_translation([0, 0, 5.5])      # apex down: tip at z 0, base r5.5 at z 5.5 (45 deg, self-supporting)
        cone.apply_transform(rot(math.pi / 2, [0, 1, 0])); cone.apply_translation([x_c - 8.5 - 0.0, c[0], c[1]])                                                       # axis along +x, tip at x_c-8.5, base at x_c-3
        lg = L.U(cone, L.cyl("x", 5.5, x_c - 3.0, x_c, c=tuple(c), n=32)); lg = DIFF(lg, L.cyl("x", 1.25, x_c - 7.5, x_c + 0.5, c=tuple(c), n=16)); lugs.append(lg)   # M3 pilot 2.5 (blind 7.5; insert PLACEHOLDER)
    shell = L.U(shell, L.I(L.U(*lugs), outer))                              # lugs stay inside the outer envelope (never proud of the medial flat)
    # medial flat wall: its lower edge (where it leaves the pad rear face) is a 50 deg gable, apex at the flat's centre, so it prints from the ring ends upward without support
    a_f = math.sqrt(Ro ** 2 - (y1 - PAD_TRIM_Y) ** 2); xb0 = XB + FAIR_PADGAP; k = 1.2
    gpoly = [(xb0 - 2.0, z1 - a_f - 1.0), (xb0 - 2.0, z1 + a_f + 1.0), (xb0, z1 + a_f + 1.0), (xb0, z1 + a_f), (xb0 + k * a_f, z1), (xb0, z1 - a_f), (xb0, z1 - a_f - 1.0)]
    gab = L.prism(gpoly, "y", PAD_TRIM_Y - 6.0, PAD_TRIM_Y + w + 0.0); shell = DIFF(shell, gab)
    FAIR_INFO["gable"] = dict(half_chord_mm=round(a_f, 1), apex_x=round(xb0 + k * a_f, 1), apex_above_pad_rear_mm=round(k * a_f + FAIR_PADGAP, 1))
    cut = _fair_cut(*FAIR_TH)
    body = DIFF(L.I(shell, bx(x0 - 1.0, x_c, y1 - 300, y1 + 300, z1 - 300, z1 + 300)), cut)
    body = _clean(body)
    # clasp-bracket window: the bracket leaves the pad rim through the ring (psi ~ CLASP_PSI), cut a house-shaped (45 deg gable, prints without support) window >= 3.2 mm clear of it
    br = clasp_bracket_geo(pad_pt(CLASP_PSI, PAD_R - 8, x=XB - 3.0)); bp, _ = trimesh.sample.sample_surface_even(br, 40000); rr = np.hypot(bp[:, 1] - y1, bp[:, 2] - z1)
    sel = bp[(rr > Ri - 6.0) & (rr < Ro + 6.0) & (bp[:, 0] > x0 - 3.0) & (bp[:, 0] < XB + 4.0)]
    if len(sel):
        ps = np.degrees(np.arctan2(sel[:, 2] - z1, sel[:, 1] - y1)); pc = float(np.median(ps)); ac = math.radians(pc); ur = np.array([math.cos(ac), math.sin(ac)]); ut = np.array([-ur[1], ur[0]])
        tt = (sel[:, 1] - y1) * ut[0] + (sel[:, 2] - z1) * ut[1]; t0_, t1_ = float(tt.min()) - 3.4, float(tt.max()) + 3.4; xr0, xr1 = x0 - 1.0, float(sel[:, 0].max()) + 3.4; hh = (t1_ - t0_) / 2.0; tm = (t0_ + t1_) / 2.0
        poly = [(t0_, xr0), (t1_, xr0), (t1_, xr1), (tm, xr1 + hh), (t0_, xr1)]
        # prism in (tangential, x) extruded along the radial direction from r = Ri - 6 to Ro + 8
        wp = trimesh.creation.extrude_polygon(sg_Polygon(poly), 14.0 + (Ro - Ri) + 8.0)         # along local z = radial
        Tm = np.eye(4); Tm[:3, 0] = [0, ut[0], ut[1]]; Tm[:3, 1] = [1, 0, 0]; Tm[:3, 2] = [0, ur[0], ur[1]]; Tm[:3, 3] = [0, y1 + ur[0] * (Ri - 6.0), z1 + ur[1] * (Ri - 6.0)]
        wp.apply_transform(Tm)       # local x (tangential) -> (0,ut), local y (poly second coordinate = x) -> world x, local z -> radial
        body = DIFF(body, wp); FAIR_INFO["bracket_window"] = dict(psi=round(pc, 1), width_mm=round(t1_ - t0_, 1), top_x=round(xr1 + hh, 1))
    # relief notches at the 4 rail-spacer ends (spacers D10 touch the pad plane; keep >= 1.5 mm from the shell)
    for z in BOLT_Z:
        for y in BOLT_Y: body = DIFF(body, L.cyl("x", 5.0 + 1.5, x0 - 2.0, x0 + 2.0, c=(y, z), n=32))
    body = _clean(body)
    # gland: round hole through the medial flat, axis along y
    a = math.radians(GLAND_PSI); zg = z1 + PAD_R * math.sin(a); xg = XB + 40.0
    gl = L.cyl("y", GLAND_D / 2, PAD_TRIM_Y - 6.0, PAD_TRIM_Y + 8.0, c=(xg, zg), n=48); body = DIFF(body, gl)
    # cover: flat disc, spigot of 4 mm inside the wall, same swept slot, M3 clearance holes at the lugs
    plate = L.cyl("x", Ro, x_c, x_end, c=(y1, z1), n=128); plate = L.I(plate, half(x_c - 1, x_end + 1, PAD_TRIM_Y))
    spig = L.cyl("x", Ri - 0.3, x_c - 4.0, x_c, c=(y1, z1), n=128, r_in=Ri - 0.3 - w); spig = L.I(spig, half(x_c - 5, x_c + 1, PAD_TRIM_Y + w + 0.3))
    cover = L.U(plate, spig); cover = _clean(DIFF(cover, cut))
    for psi in FAIR_LUG_PSI:
        a2 = math.radians(psi); u = np.array([math.cos(a2), math.sin(a2)]); c = np.array([y1, z1]) + u * (Ri - 4.5); cover = DIFF(cover, L.cyl("x", 1.7, x_c - 1, x_end + 1, c=tuple(c), n=24))
    return body, cover
def fairing_clearance(body, cover, base, lim_th=(-100.0, 10.0), lim_ph=(-25.0, 40.0), step_th=2.5, step_ph=5.0, extra=None):
    """min distance (mm) of the fairing to every moving group over the hard ranges: M1 over S1', M2 (S2 adapter, S3 cradle, XM540, S2 stop lug, arm) over S1' x S2.  Samples 1.0 / 2.0 mm."""
    import j1r2_geom as G, trimesh as T_
    from scipy.spatial import cKDTree
    def samp(m, sp):
        p, _ = T_.sample.sample_surface_even(m, int(m.area / sp ** 2) + 300); return np.vstack([p, m.vertices])
    Dv = M.DER; F = cKDTree(np.vstack([samp(body, 1.0), samp(cover, 1.0)]))
    M1N = ("S1_yoke", "S1_stop_collar", "S2_AK70", "S2_clamp", "S2_sleeve", "S2_stop_posts", "tpu_S2_bumpers"); M2N = ("S2_adapter", "S3_bracket", "S3_XM540", "S2_stop_lug", "arm_tube", "arm_elbow_ball")
    out = {}
    for nm, names, grp in (("M1", M1N, "M1"), ("M2", M2N, "M2")):
        pts = {k: samp(base[k]["mesh"], 2.0) for k in names}; best = (1e9, None, None)
        for th in np.arange(lim_th[0], lim_th[1] + 0.1, step_th):
            for ph in ([0.0] if grp == "M1" else np.arange(lim_ph[0], lim_ph[1] + 0.1, step_ph)):
                Mx = G.pose_matrix(grp, Dv, th, ph)
                for k, p in pts.items():
                    d = float(F.query(p @ Mx[:3, :3].T + Mx[:3, 3], distance_upper_bound=30.0)[0].min())
                    if d < best[0]: best = (d, k, (float(th), float(ph)))
        out[nm] = dict(min_mm=round(min(best[0], 30.0), 2), part=best[1], at=best[2])
    return out
def fairing_extras(body, cover):
    """clearances of the fairing to the wearer, the rail, the spine bridge, the dish/liner lead channels; mass; printability."""
    import clib as L_
    from scipy.spatial import cKDTree
    import trimesh as T_
    out = {}
    for nm, m in (("body", body), ("cover", cover)):
        s_, _ = T_.sample.sample_surface_even(m, int(m.area / 1.5 ** 2) + 300); out[nm] = dict(skin_gap_mm=round(float(L_.skin_gap(m, 4000)[0]), 1), watertight=bool(m.is_watertight), volume_cm3=round(abs(m.volume) / 1000, 2),
                                                                                                mass_g=round(abs(m.volume) / 1000 * L_.DENS["PACF"], 1), bbox=np.round(m.extents, 1).tolist(), fits_256=bool(np.all(m.bounding_box_oriented.extents <= 256)), components=len(m.split(only_watertight=False)))
    return out
def mj11_frame(fr, side):
    mp = (lambda p: np.asarray(p, float)) if side == "R" else M.mirror; pad = C.pad_nodes(fr, side); Y, Z, skin, inner, plane = seat_surfaces()
    # --- bolted joints pad -> rail (2 pairs, 60 mm apart in z): rigid pad body, bolt-group beams, spacer stiffness folded into the beam length
    for z in BOLT_Z:
        nb = f"{side}_pb{int(z)}"; fr.node(nb, mp(np.array([X0, np.mean(BOLT_Y), z]))); fr.rigid(pad, nb, tag="pad")
        link_bolts(fr, nb, rail_node_name(side, z), [(-4, 0), (4, 0)], tag="iface")
    # --- liner: normal foam springs over the dish footprint (shear free: slip face), a 16 mm lattice of rigid links to the pad body
    pts = []; Dm = _domain(); step = 16.0
    for y in np.arange(Y1 - PAD_R + 8, Y1 + PAD_R - 7, step):
        for z in np.arange(Z1 - PAD_R + 8, Z1 + PAD_R - 7, step):
            if math.hypot(y - Y1, z - Z1) <= PAD_R - 4 and y >= RAIL_EDGE_Y + 2: pts.append((y, z))
    k_a = M.E_FOAM / LINER_T; A_i = step * step
    for i, (y, z) in enumerate(pts):
        xs = float(M.back_x(y, z))
        n = f"{side}_ln{i}"; fr.node(n, mp(np.array([xs + LINER_T / 2, y, z]))); fr.rigid(pad, n, tag="pad"); fr.spring(n, [1.0, 0, 0], k_a * A_i, area=A_i, label=f"{side}_liner{i}", kind="liner")
    an = f"{side}_clasp_anchor"; fr.node(an, mp(pad_pt(CLASP_PSI, PAD_R - 8, x=XB - 3.0))); fr.rigid(pad, an, tag="pad"); clasp_bracket_frame(fr, side, an); return pad
C.CONCEPTS.update(MJ11=dict(rule=dict(blade=5.0), title="Blade-seated pancake: flat pad on a conformal foam dish, bolted to the rail", geo=mj11_geo, frame=mj11_frame, balancer="none", clasp="feet to the pad's top rim"))
