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
LEAD_W = 8.0                       # trench width (two 3 mm flex tails + 2 mm)
RIM_R, RIM_PSI = 56.0, (315.0, 218.0)   # rev 2026-11-03: rim collector trench r 56 from the psi-315 channel round to the rail-side exit (psi 218 meets the y = -96 cut at z -224); all lead trenches end in ONE fixed exit at the rail edge
CH_R0 = 18.0
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
    cuts = [boxbeam(np.array([X0 - 40.0, a[0], a[1]]), np.array([X0 - 40.0, b[0], b[1]]), LEAD_W, 60.0, None, up=(1, 0, 0), ext=0.0) for a, b in trench_segments()]
    return DIFF(ln, *cuts)
def trench_segments():
    """2-D (y, z) centre-line segments of the lead trenches: 3 radial channels (r 18..80) + the rim collector arc (r 56, psi 315 -> 218, 8 deg steps)."""
    seg = []
    for psi in LEAD_PSI:
        a = math.radians(psi); seg.append(((Y1 + CH_R0 * math.cos(a), Z1 + CH_R0 * math.sin(a)), (Y1 + 80.0 * math.cos(a), Z1 + 80.0 * math.sin(a))))
    ps = np.linspace(RIM_PSI[0], RIM_PSI[1], int(abs(RIM_PSI[0] - RIM_PSI[1]) / 8.0) + 2)
    for p0, p1 in zip(ps[:-1], ps[1:]):
        a0, a1 = math.radians(p0), math.radians(p1); seg.append(((Y1 + RIM_R * math.cos(a0), Z1 + RIM_R * math.sin(a0)), (Y1 + RIM_R * math.cos(a1), Z1 + RIM_R * math.sin(a1))))
    return seg
def in_trench(y, z, buf=0.0):
    """True if (y, z) lies within a lead trench (+ buf mm)."""
    p = np.array([y, z]); 
    for a, b in trench_segments():
        a, b = np.array(a), np.array(b); t = np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0, 1)
        if np.linalg.norm(p - (a + t * (b - a))) <= LEAD_W / 2 + buf: return True
    return False
def rail_bolt_pts():
    """(x_pad_face, y, z) of the 4 M4 bolts on the pad plane and the rail rear face x at the same place."""
    return [(X0, y, z, rail_x(-119.0, z)) for z in BOLT_Z for y in BOLT_Y]
def mj11_geo(base):
    out = {}; padm = L.I(pad_disc(), bx(X0 - 10.0, XB + 10.0, PAD_TRIM_Y, Y1 + 100.0, Z1 - 100.0, Z1 + 100.0))
    out["pad"] = P_("pad", padm, "PACF", BLUE, note="R66 x 6 PA-CF pad disc trimmed flat at y %.0f (spine-bridge ears), S1' AK70 on its rear face (flange PLACEHOLDER); its flat front face lies on the seat dish and, medially, on the rail spacers" % PAD_TRIM_Y)
    dish = dish_mesh(); out["seat_dish"] = P_("seat_dish", dish, "PACF", BLUE, fill=0.30, note="conformal PA-CF shell: outer face the flat plane x0, inner face the skin + %.0f mm liner; variable thickness %.1f..%.1f mm (drawn solid, mass at the hollow fill 0.30: 2.4 mm walls + ribs); prints flat face down, no supports" % (LINER_T, PLATE_MIN, float((X0 - seat_surfaces()[3]).max())))
    ln = liner_mesh(); out["liner"] = P_("liner", ln, "TPU", (0.85, 0.85, 0.88), kind="bought", mass_g=round(abs(ln.volume) / 1000.0 * FOAM_DENS, 1), note="8 mm closed-cell foam (0.12 g/cm3 assumed) + PTFE-fabric slip face toward the plate side of the base layer; 3 radial lead channels 8 mm wide (psi 225/270/315) + rim collector trench r 56 to one fixed exit at the rail edge (rev 2026-11-03)")
    sp = U(*[L.cyl("x", 5.0, rail_x(-119.0, z) - 0.2, X0 + 0.2, c=(y, z), n=24) for (_, y, z, _) in [(0, y, z, 0) for z in BOLT_Z for y in BOLT_Y]])
    out["rail_spacers"] = P_("rail_spacers", sp, "PETGCF", C.COL["steel"], note="4 spacers D10 from the rail rear face to the pad plane (%.1f..%.1f mm), M4 x 16 (4.8 mm spacers) / M4 x 20 (8.3 mm) SHCS into new M4 heat-set inserts (5.6 OD x 4) in the rail (PLACEHOLDER bosses)" % tuple(sorted((X0 - rail_x(-119.0, z)) for z in BOLT_Z)))
    out["liner"]["skin_contact"] = True
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(pad_pt(CLASP_PSI, PAD_R - 8, x=XB - 3.0)), "PACF", BLUE, note="clasp feet (re-seated) to the pad rim at psi %.0f" % CLASP_PSI)
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
            if math.hypot(y - Y1, z - Z1) <= PAD_R - 4 and y >= RAIL_EDGE_Y + 2 and not in_trench(y, z, 5.0): pts.append((y, z))      # rev 2026-11-03: lattice cells over the lead trenches carry no liner load
    k_a = M.E_FOAM / LINER_T; A_i = step * step
    for i, (y, z) in enumerate(pts):
        xs = float(M.back_x(y, z))
        n = f"{side}_ln{i}"; fr.node(n, mp(np.array([xs + LINER_T / 2, y, z]))); fr.rigid(pad, n, tag="pad"); fr.spring(n, [1.0, 0, 0], k_a * A_i, area=A_i, label=f"{side}_liner{i}", kind="liner")
    an = f"{side}_clasp_anchor"; fr.node(an, mp(pad_pt(CLASP_PSI, PAD_R - 8, x=XB - 3.0))); fr.rigid(pad, an, tag="pad"); clasp_bracket_frame(fr, side, an); return pad
C.CONCEPTS.update(MJ11=dict(rule=dict(blade=5.0), title="Blade-seated pancake: flat pad on a conformal foam dish, bolted to the rail", geo=mj11_geo, frame=mj11_frame, balancer="none", clasp="feet to the pad's top rim"))
