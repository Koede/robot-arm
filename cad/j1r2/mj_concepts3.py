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
from clib import U, D as DIFF, box as bx
from mj_concepts import feet_xyz, P_, pad_disc, pad_pt, link_bolts, rail_node_name, rail_x, clasp_bracket_geo, clasp_bracket_frame, boxbeam, BLUE, PAD_R
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

# ------------------------------------------------------------------ fixed fairing for MJ11 (2026-11-03): 1.8 mm PA-CF shell over the S1' AK70 + pad rear/sides (no riser, no lap)
import shapely.geometry as sg, j1r2_params as PP, j1r2_geom as GG
FAIR_T = 1.8                        # shell wall
FAIR_GAP = 3.2                      # swept-clearance margin to the moving S1' group (gate 3.0)
FAIR_S1 = (-100.0, 10.0)            # S1' range the slot is swept over: hard stops of both the task box (-80..5 +-5) and the MJ11 sweep box (-95..5 +-5)
FAIR_X_END = M.DER["XO"] + 31.0     # rear face of the cap (as J1 rev 2: yoke + coil + 10 mm)
FAIR_LIP = 0.2                      # radial clearance to the pad rim
GLAND = dict(x=X0 + 19.0, dz=-12.0, d=13.2)      # cable gland hole through the flat medial wall (bundle OD 8.2 + grommet), PLACEHOLDER route
FAIR_SCREW_PSI = (45.0, 270.0, 340.0)   # 3 x radial M3 x 8 thread-forming into the pad rim, at x0 + 3 (psi from +y toward +z); keeps off the medial setback (144..216) and the clasp-bracket window (103..137)
FAIR_SETBACK_Y = -100.0             # setback region y < this (also lifts the lip 5.2 mm off the two rail-spacer ends)
FAIR_BRIDGE_X = -92.5               # medial setback: the flat wall (y < -100) starts here, not at x0, so it clears the spine-bridge ears (y <= -124.3, x <= -95.5) by >= 3 mm
FAIR_WIN_M = 3.5                    # window margin around the clasp bracket arm
def _poly(c, r, ymin=None):
    d = sg.Point(c).buffer(r, 64)
    if ymin is not None: d = d.intersection(sg.box(ymin, c[1] - 2 * r, c[0] + 2 * r, c[1] + 2 * r))
    return list(d.exterior.coords)[:-1]
def fairing_mesh(bracket=None):
    c = (Y1, Z1); Ro = PAD_R + FAIR_LIP + FAIR_T; Ri = PAD_R + FAIR_LIP; xe = FAIR_X_END
    out = L.prism(_poly(c, Ro, PAD_TRIM_Y - FAIR_LIP - FAIR_T), "x", X0, xe)
    inn = L.prism(_poly(c, Ri, PAD_TRIM_Y - FAIR_LIP), "x", X0 - 1.0, xe - FAIR_T)
    fair = DIFF(out, inn); cut = []; TS, AKR = PP.TS, PP.AKR
    for th in np.arange(FAIR_S1[0] - 2.5, FAIR_S1[1] + 2.6, 5.0):
        Mx = L.tr([0, Y1, Z1]) @ GG.Rx(th) @ L.tr([0, -Y1, -Z1]); g = FAIR_GAP
        sl = L.cyl("y", 48.5 + g, 30.0 + Y1, 130.0 + Y1, c=(M.PX, Z1), n=48); sl.apply_transform(Mx); cut.append(sl)
        bar = bx(M.PX - 6.0 - g, M.PX + 6.0 + g, Y1 + 0.0, Y1 + TS + 3.0, Z1 - 26.0 - g, Z1 + 26.0 + g); bar.apply_transform(Mx); cut.append(bar)
        pl_ = bx(M.PX - 48.5 - g, M.PX + 48.5 + g, Y1 + TS - 1.3 - g, Y1 + TS + 3.7 + g, Z1 - 48.5 - g, Z1 + 48.5 + g); pl_.apply_transform(Mx); cut.append(pl_)
    fair = DIFF(fair, L.U(*cut))
    fair = DIFF(fair, bx(X0 - 2.0, FAIR_BRIDGE_X, PAD_TRIM_Y - 10.0, FAIR_SETBACK_Y, Z1 - 100.0, Z1 + 100.0))                                  # medial setback (spine-bridge ears)
    if bracket is not None:
        hub, anc = bracket; _G["win"] = boxbeam(hub, anc, 24.0 + 2 * FAIR_WIN_M, 6.0 + 2 * FAIR_WIN_M, None, up=(1, 0, 0), ext=6.0 + FAIR_WIN_M); fair = DIFF(fair, _G["win"])          # window for the clasp bracket arm
    gx, gz = GLAND["x"], Z1 + GLAND["dz"]; fair = DIFF(fair, L.cyl("y", GLAND["d"] / 2, PAD_TRIM_Y - 12.0, PAD_TRIM_Y + 6.0, c=(gx, gz), n=40))       # cable gland through the flat medial wall
    for psi in FAIR_SCREW_PSI: fair = DIFF(fair, GG._radial_cyl(Y1, Z1, psi, PAD_R + FAIR_LIP + FAIR_T + 1.0, PAD_R - 1.0, 1.7, X0 + 3.0))                       # M3 clearance holes in the lip
    return fair
FAIR_XS = M.PX - 48.5 - FAIR_GAP    # print split plane = start of the S2 slot (x -86.95): ring (lip band, all round) | hood (rear shell + cap)
FAIR_TONGUE = 4.0                   # hood tongue length forward into the ring (inner half of the wall), bonded
FAIR_TG = 0.15                      # glue gap of the lap
def fairing_split(F):
    """cut the fairing at x = FAIR_XS into ring (x < XS: lip band all round) and hood (x > XS: rear shell + cap) so that neither needs supports (the lip arc over the S2 window cannot be printed cap-down;
    mouth-down the cap roof cannot).  Lap joint: the hood carries a 4 mm tongue (inner half of the wall) forward into a matching rebate of the ring, 0.15 mm glue gap, bonded (2-part epoxy / MMA)."""
    XS, tl = FAIR_XS, FAIR_TONGUE; c = (Y1, Z1); Ri = PAD_R + FAIR_LIP
    hood = L.I(F, bx(XS, 0.0, -400, 400, -600, 0.0)); ring = L.I(F, bx(-200, XS, -400, 400, -600, 0.0))
    sec = F.section(plane_origin=[XS + 3.0, 0, 0], plane_normal=[1, 0, 0]); fp = None
    for d in sec.discrete:
        pl = sg.Polygon(np.asarray(d)[:, 1:3]).buffer(0)
        if pl.area > 1.0: fp = pl if fp is None else fp.union(pl)
    half = sg.Polygon(_poly(c, Ri + FAIR_T / 2 - FAIR_TG, PAD_TRIM_Y - FAIR_LIP)).difference(sg.Polygon(_poly(c, Ri, PAD_TRIM_Y - FAIR_LIP)))
    tong = half.intersection(fp.buffer(0.3)); tg = []
    for pl in ([tong] if tong.geom_type == "Polygon" else list(tong.geoms)):
        if pl.area > 2.0: tg.append(L.prism(list(pl.exterior.coords)[:-1], "x", XS - tl, XS + 0.3))
    tongue = L.U(*tg)
    if "win" in _G: tongue = DIFF(tongue, _G["win"])                      # the tongue must not enter the clasp-bracket window either
    hood = L.U(hood, tongue)
    # rebate in the ring: the tongue + glue gap, inner half only; the ring keeps its outer half and the rear edge on the bed
    hf = []
    for pl in ([tong] if tong.geom_type == "Polygon" else list(tong.geoms)):
        if pl.area > 2.0: hf.append(L.prism(list(pl.buffer(FAIR_TG, join_style=2).exterior.coords)[:-1], "x", XS - tl - FAIR_TG, XS + 0.5))
    ring = DIFF(ring, L.I(L.U(*hf), L.prism(_poly(c, Ri + FAIR_T / 2 + 0.0, PAD_TRIM_Y - FAIR_LIP), "x", XS - tl - 1.0, XS + 1.0)))
    return ring, hood
def mj11_geo(base):
    out = {}; padm = L.I(pad_disc(), bx(X0 - 10.0, XB + 10.0, PAD_TRIM_Y, Y1 + 100.0, Z1 - 100.0, Z1 + 100.0))
    out["pad"] = P_("pad", padm, "PACF", BLUE, note="R66 x 6 PA-CF pad disc trimmed flat at y %.0f (spine-bridge ears), S1' AK70 on its rear face (flange PLACEHOLDER); its flat front face lies on the seat dish and, medially, on the rail spacers" % PAD_TRIM_Y)
    dish = dish_mesh(); out["seat_dish"] = P_("seat_dish", dish, "PACF", BLUE, fill=0.30, note="conformal PA-CF shell: outer face the flat plane x0, inner face the skin + %.0f mm liner; variable thickness %.1f..%.1f mm (drawn solid, mass at the hollow fill 0.30: 2.4 mm walls + ribs); prints flat face down, no supports" % (LINER_T, PLATE_MIN, float((X0 - seat_surfaces()[3]).max())))
    ln = liner_mesh(); out["liner"] = P_("liner", ln, "TPU", (0.85, 0.85, 0.88), kind="bought", mass_g=round(abs(ln.volume) / 1000.0 * FOAM_DENS, 1), note="8 mm closed-cell foam (0.12 g/cm3 assumed) + PTFE-fabric slip face toward the plate side of the base layer; 3 radial lead channels 8 mm wide (psi 225/270/315)")
    sp = U(*[L.cyl("x", 5.0, rail_x(-119.0, z) - 0.2, X0 + 0.2, c=(y, z), n=24) for (_, y, z, _) in [(0, y, z, 0) for z in BOLT_Z for y in BOLT_Y]])
    out["rail_spacers"] = P_("rail_spacers", sp, "PETGCF", C.COL["steel"], note="4 spacers D10 from the rail rear face to the pad plane (%.1f..%.1f mm), M4 x 20 SHCS into new M4 heat-set inserts (5.6 OD x 4) in the rail (PLACEHOLDER bosses)" % tuple(sorted((X0 - rail_x(-119.0, z)) for z in BOLT_Z)))
    out["liner"]["skin_contact"] = True
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(pad_pt(CLASP_PSI, PAD_R - 8, x=XB - 3.0)), "PACF", BLUE, note="clasp feet (re-seated) to the pad rim at psi %.0f" % CLASP_PSI)
    fm = fairing_mesh((feet_xyz()["F1_upper_trap"], pad_pt(CLASP_PSI, PAD_R - 8, x=XB - 3.0)))
    ring, hood = fairing_split(fm)
    out["fairing_ring"] = P_("fairing_ring", ring, "PACF", (0.80, 0.55, 0.20), note="fixed fairing, front lip ring (x %.1f..%.1f): R66.2 lip around the pad rim, flush with the dish plane x0, trimmed flat at y %.1f with a medial setback to x %.1f (spine-bridge ears), clasp-bracket window, 3 x M3 radial into the pad rim; prints standing on its rear edge, no supports" % (X0, FAIR_XS, PAD_TRIM_Y - FAIR_LIP - FAIR_T, FAIR_BRIDGE_X))
    out["fairing_hood"] = P_("fairing_hood", hood, "PACF", (0.86, 0.62, 0.26), note="fixed fairing, rear hood (x %.1f..%.1f): 1.8 mm shell over the S1' AK70 medial/top side + cap; S2 slot swept over S1' %.0f..%.0f with %.1f mm margin; cable gland D%.1f; 4 mm inner-half tongue bonded into the ring; prints cap down, no supports" % (FAIR_XS, FAIR_X_END, FAIR_S1[0], FAIR_S1[1], FAIR_GAP, GLAND["d"]))
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
