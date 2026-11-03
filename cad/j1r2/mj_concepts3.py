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
import clib as L, mj_frame as M, mj_concepts as C, j1r2_params as P, shapely.geometry as sg
from clib import prism, cyl, tr
from j1r2_geom import Rx as G_Rx
from clib import U, D as DIFF, box as bx
from mj_concepts import P_, pad_disc, pad_pt, link_bolts, rail_node_name, rail_x, clasp_bracket_geo, clasp_bracket_frame, boxbeam, node_ball, BLUE, PAD_R
Y1, Z1, X0, XB = M.Y1, M.Z1, M.X0, M.XB
LINER_T = 8.0                      # foam liner thickness (compliance over the moving blade)
PLATE_MIN = 3.0                    # minimum dish thickness at the ridge
RAIL_EDGE_Y = -96.0                # dish and liner stop 4 mm lateral of the rail edge (rail y -138..-100)
PAD_TRIM_Y = -121.0                # pad disc trimmed flat on its medial side: clears the real spine-bridge ears by >= 3 mm
BOLT_Z = (-160.0, -220.0)          # rail node heights of the two bolt pairs (rail nodes are every 20 mm)
BOLT_Y = (-113.0, -105.0)          # PLACEHOLDER positions of the new M4 bosses on the rail rear face
CLASP_PSI = float(os.environ.get("MJ11_CLASP_PSI", "119"))     # clasp bracket anchor on the pad rim (deg from +y toward +z)
# clasp bracket hub -> pad-rim bar (2026-11-03 search, mj11_bracket_search.py): the 2026-11-02 bar (24 x 6, anchor r 58, ext 6) climbed to x -89.3 at its tip and sat 2.0 mm from the S1' stop-ring front face (x -87.3, r 44.6..60)
CLASP_W = float(os.environ.get("MJ11_CLASP_W", "24"))          # hub -> anchor bar width (mm, in the pad plane)
CLASP_T = float(os.environ.get("MJ11_CLASP_T", "6"))           # hub -> anchor bar thickness (mm, along x)
CLASP_RA = float(os.environ.get("MJ11_CLASP_RA", str(PAD_R)))   # anchor radius on the pad (mm from the S1' axis)
CLASP_EXT = float(os.environ.get("MJ11_CLASP_EXT", "6"))       # bar overhang past the anchor (mm)
CLASP_VIA = None if os.environ.get("MJ11_CLASP_VIA", "") == "" else np.array([float(v) for v in os.environ["MJ11_CLASP_VIA"].split(",")])   # optional via point x,y,z
LEAD_PSI = tuple(float(v) for v in os.environ.get("MJ11_LEAD_PSI", "225,270,315").split(","))   # lead channels through the liner (default 225/270/315; MJ11_LEAD_PSI e.g. "315" = motion-aligned variant, SPEC_j1r2_mj11_leads_shear s3)
LEAD_W = float(os.environ.get("MJ11_LEAD_W", "8"))         # channel width (default 8 mm)
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
        cuts.append(boxbeam(p0, p1, LEAD_W, 60.0, None, up=(1, 0, 0), ext=0.0))
    return DIFF(ln, *cuts)
def rail_bolt_pts():
    """(x_pad_face, y, z) of the 4 M4 bolts on the pad plane and the rail rear face x at the same place."""
    return [(X0, y, z, rail_x(-119.0, z)) for z in BOLT_Z for y in BOLT_Y]
def mj11_anchor(psi=None, ra=None):
    return pad_pt(CLASP_PSI if psi is None else psi, CLASP_RA if ra is None else ra, x=XB - 3.0)
def mj11_bracket(psi=None, w=None, t=None, ra=None, ext=None, via=None):
    """feet bars F5 / F4 -> hub (16 x 6, unchanged) and hub -> (via) -> pad-rim anchor bar (w x t); the anchor end overhangs by ext."""
    w = CLASP_W if w is None else w; t = CLASP_T if t is None else t; ext = CLASP_EXT if ext is None else ext; via = CLASP_VIA if via is None else via
    f = C.feet_xyz(); h = f["F1_upper_trap"]; bars = [boxbeam(f[k], h, 16.0, 6.0, None, up=(1, 0, 0), ext=4.0) for k in ("F5_mid_trap", "F4_acromion")]
    pts = [h] + ([np.asarray(via, float)] if via is not None else []) + [mj11_anchor(psi, ra)]
    segs = list(zip(pts[:-1], pts[1:]))
    for i, (a, b) in enumerate(segs): bars.append(boxbeam(a, b, w, t, None, up=(1, 0, 0), ext=(ext if i == len(segs) - 1 else 6.0)))
    return U(*bars, *[node_ball(f[k], 5.0) for k in ("F5_mid_trap", "F4_acromion", "F1_upper_trap")])

# ------------------------------------------------------------------ fixed fairing (2026-11-03): 1.8 mm PA-CF cap over the S1' AK70 + pad rear, no riser / no lap
FAIR_WALL = 1.8
FAIR_GAP = 3.2                     # swept-cut clearance to the moving S1'/S2 stage (gate 3.0 mm)
FAIR_TH = (-102.5, 12.5)           # S1' passive box -95..5 (the MJ11 box before the fairing; the task text quoted -80..5, the wider one is kept so the workspace is not cut) + the 5 deg hard-stop expansion, + 2.5 for the 5 deg cut step
FAIR_TABS = (170.0, 198.0, 218.0)     # 3 inward tabs at the open rim, M3 x 5 ISO 7380 into short M3 heat-set inserts (4.6 OD x 3 L, PLACEHOLDER) in the pad rear face
GLAND_PSI, GLAND_X, GLAND_R = 190.0, 12.0, 6.6      # cable gland: radial bore through the medial wall at XB+12 (the F cable run is not redone for MJ11: open item), R = bundle 8.2/2 + 2.5
def _outline_poly(r=PAD_R, trim=PAD_TRIM_Y, n=96):
    c = sg.Point(Y1, Z1).buffer(r, resolution=n // 4); return c.intersection(sg.box(trim, Z1 - 200.0, Y1 + 200.0, Z1 + 200.0))
def _poly_xy(p): return list(p.exterior.coords)[:-1]
def _dilate(m, d):
    """cheap outward offset (about d mm) of a solid: union of its copies shifted d along the 26 neighbour directions (axes, edges, corners)."""
    cp = [m]
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dz in (-1, 0, 1):
                v = np.array([dx, dy, dz], float)
                if not v.any(): continue
                c = m.copy(); c.apply_translation(v / np.linalg.norm(v) * d); cp.append(c)
    return U(*cp)
def fairing_mesh(bracket=None):
    """Cap on the pad rear face: outer outline = the pad outline (R66 trimmed at y -121, flush with pad and dish edge), 1.8 mm walls from x = XB to XO + 31, closed end plate; the arm/S2 side is
    open where the S1' yoke bar, S2 clamp flange and S2 sleeve sweep (cuts as in j1r2_geom.build, gap 3.2) so what is left is a C-shaped shell over the medial/upper sector of the motor.
    Also: no ring below the cut zone (psi 224..61), notch for the clasp bracket, gland bore, 3 tabs with M3 holes."""
    XO, PX = M.DER["XO"], M.DER["PX"]; x_end = XO + 31.0; out_p = _outline_poly(); in_p = out_p.buffer(-FAIR_WALL, join_style=2)
    cap = DIFF(prism(_poly_xy(out_p), "x", XB, x_end), prism(_poly_xy(in_p), "x", XB - 1.0, x_end - FAIR_WALL))
    th_lo, th_hi = FAIR_TH; cut = []; g = FAIR_GAP; TS = P.TS
    for th in np.arange(th_lo, th_hi + 0.1, 5.0):
        Mx = tr([0, Y1, Z1]) @ G_Rx(th) @ tr([0, -Y1, -Z1])
        for m in (cyl("y", 48.5 + g, 30.0 + Y1, 130.0 + Y1, c=(PX, Z1), n=48), L.box(PX - 6.0 - g, PX + 6.0 + g, Y1, Y1 + TS + 3.0, Z1 - 26.0 - g, Z1 + 26.0 + g),
                  L.box(PX - 48.5 - g, PX + 48.5 + g, Y1 + TS - 1.3 - g, Y1 + TS + 3.7 + g, Z1 - 48.5 - g, Z1 + 48.5 + g)):
            m.apply_transform(Mx); cut.append(m)
    fair = DIFF(cap, U(*cut))
    # the thin ring XB..XB+4.5 that the clamp sweep (starts at XB+4.5) leaves in the cut sector: remove the sector psi 224..(360+61) so no floating strips remain
    wedge = [(Y1, Z1)] + [(Y1 + 140.0 * math.cos(math.radians(a)), Z1 + 140.0 * math.sin(math.radians(a))) for a in np.arange(224.0, 421.1, 4.0)]
    fair = DIFF(fair, prism(wedge, "x", XB - 1.0, x_end - FAIR_WALL))
    if bracket is not None: fair = DIFF(fair, _dilate(L.I(bracket, L.box(XB - 12.0, XB + 60.0, Y1 - 120.0, Y1 + 120.0, Z1 - 120.0, Z1 + 120.0)), 4.6))     # clasp feet bars pass the rim at psi 120
    a = math.radians(GLAND_PSI); u = np.array([math.cos(a), math.sin(a)]); xg = XB + GLAND_X
    gl = trimesh.creation.cylinder(radius=GLAND_R, height=40.0, sections=32); gl.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], [0, u[0], u[1]])); gl.apply_translation([xg, Y1 + u[0] * 62.0, Z1 + u[1] * 62.0])
    fair = DIFF(fair, gl)
    for psi in FAIR_TABS:
        a = math.radians(psi); c = np.array([Y1 + 58.0 * math.cos(a), Z1 + 58.0 * math.sin(a)]); u = np.array([math.cos(a), math.sin(a)]); v = np.array([-u[1], u[0]])
        tab = prism([tuple(c - 6.0 * u - 6.0 * v), tuple(c + 9.0 * u - 6.0 * v), tuple(c + 9.0 * u + 6.0 * v), tuple(c - 6.0 * u + 6.0 * v)], "x", XB, XB + 2.4)
        fair = DIFF(U(fair, L.I(tab, prism(_poly_xy(out_p.buffer(-0.5)), "x", XB - 1.0, XB + 3.0))), cyl("x", 1.7, XB - 1.0, XB + 3.5, c=tuple(c), n=24))
    return fair
def fair_insert_pts(): return [(Y1 + 58.0 * math.cos(math.radians(p)), Z1 + 58.0 * math.sin(math.radians(p))) for p in FAIR_TABS]
def mj11_geo(base):
    out = {}; padm = L.I(pad_disc(), bx(X0 - 10.0, XB + 10.0, PAD_TRIM_Y, Y1 + 100.0, Z1 - 100.0, Z1 + 100.0))
    for (yy, zz) in fair_insert_pts(): padm = DIFF(padm, cyl("x", 2.0, XB - 3.5, XB + 0.5, c=(yy, zz), n=24))      # 3 short M3 heat-set inserts for the fairing tabs (4.6 OD x 3 L, hole D4.0 x 3.5, PLACEHOLDER)
    out["pad"] = P_("pad", padm, "PACF", BLUE, note="R66 x 6 PA-CF pad disc trimmed flat at y %.0f (spine-bridge ears), S1' AK70 on its rear face (flange PLACEHOLDER); its flat front face lies on the seat dish and, medially, on the rail spacers" % PAD_TRIM_Y)
    dish = dish_mesh(); out["seat_dish"] = P_("seat_dish", dish, "PACF", BLUE, fill=0.30, note="conformal PA-CF shell: outer face the flat plane x0, inner face the skin + %.0f mm liner; variable thickness %.1f..%.1f mm (drawn solid, mass at the hollow fill 0.30: 2.4 mm walls + ribs); prints flat face down, no supports" % (LINER_T, PLATE_MIN, float((X0 - seat_surfaces()[3]).max())))
    ln = liner_mesh(); out["liner"] = P_("liner", ln, "TPU", (0.85, 0.85, 0.88), kind="bought", mass_g=round(abs(ln.volume) / 1000.0 * FOAM_DENS, 1), note="8 mm closed-cell foam (0.12 g/cm3 assumed) + PTFE-fabric slip face toward the plate side of the base layer; 3 radial lead channels 8 mm wide (psi 225/270/315)")
    sp = U(*[L.cyl("x", 5.0, rail_x(-119.0, z) - 0.2, X0 + 0.2, c=(y, z), n=24) for (_, y, z, _) in [(0, y, z, 0) for z in BOLT_Z for y in BOLT_Y]])
    out["rail_spacers"] = P_("rail_spacers", sp, "PETGCF", C.COL["steel"], note="4 spacers D10 from the rail rear face to the pad plane (%.1f..%.1f mm), M4 x 20 SHCS into new M4 heat-set inserts (5.6 OD x 4) in the rail (PLACEHOLDER bosses)" % tuple(sorted((X0 - rail_x(-119.0, z)) for z in BOLT_Z)))
    out["liner"]["skin_contact"] = True
    out["clasp_bracket"] = P_("clasp_bracket", mj11_bracket(), "PACF", BLUE, note="clasp feet (re-seated) to the pad rim at psi %.0f, anchor r %.0f, hub bar %.0f x %.0f, overhang %.0f%s" % (CLASP_PSI, CLASP_RA, CLASP_W, CLASP_T, CLASP_EXT, "" if CLASP_VIA is None else ", via %s" % np.round(CLASP_VIA, 1).tolist()))
    out["fairing"] = P_("fairing", fairing_mesh(out["clasp_bracket"]["mesh"]), "PACF", (0.45, 0.50, 0.60), note="fixed fairing: 1.8 mm PA-CF C-shell on the pad rear face (outline = pad R66 trimmed y %.0f, flush with pad and dish), x XB..XO+31, closed end plate, swept slot for the S1' yoke bar / S2 clamp (gap %.1f), gland bore R%.1f at psi %.0f, 3 tabs M3 x 5 into the pad (inserts PLACEHOLDER); prints end plate down, no supports" % (PAD_TRIM_Y, FAIR_GAP, GLAND_R, GLAND_PSI))
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
    an = f"{side}_clasp_anchor"; fr.node(an, mp(mj11_anchor())); fr.rigid(pad, an, tag="pad"); hub = f"{side}_clasp_hub"; sec = M.rect(CLASP_W, CLASP_T)
    if CLASP_VIA is None: fr.beam(hub, an, sec, "PACF", up=(1, 0, 0), tag="clasp_seat")
    else: v = f"{side}_clasp_via"; fr.node(v, mp(CLASP_VIA)); fr.beam(hub, v, sec, "PACF", up=(1, 0, 0), tag="clasp_seat"); fr.beam(v, an, sec, "PACF", up=(1, 0, 0), tag="clasp_seat")
    return pad
C.CONCEPTS.update(MJ11=dict(rule=dict(blade=5.0), title="Blade-seated pancake: flat pad on a conformal foam dish, bolted to the rail", geo=mj11_geo, frame=mj11_frame, balancer="none", clasp="feet to the pad's top rim"))
