"""J1 mount alternatives (2026-11-02, SHOULDER / WING, concept study): HOW the J1 rev 2 pancake shoulder attaches to Forest's back.
Same pad (y1, z1) = (+10, -30), same moving stage, same harness (mj_frame.harness); each concept supplies
  geo(base)  -> {name: dict(mesh, mat, colour, kind)}  new fixed parts (right side, robot frame) that replace REF's pad_riser + standoffs + fairing_lap
  frame(fr, side) -> adds the members between the pad node and the harness nodes for one side (mirrored for L)
  balancer   -> where the S1'/S2 balancer spring tube lives (the anchor A stays a sheave on the pad, so the arm torque balance is unchanged)
  clasp      -> how the clasp frame (4 feet, orphaned since the chassis plate was retired) is seated.
Proxy geometry (CAD-as-code primitives); fastener patterns of bought parts are PLACEHOLDER."""
import numpy as np, trimesh, math, os, sys
from scipy.ndimage import gaussian_filter
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clib as L, mj_frame as M
from clib import cyl, box as bx, U, D as DIFF, prism, COL
DER = M.DER; PX, Y1, Z1, X0, XB, XO = DER["PX"], DER["y1"], DER["z1"], DER["x0"], DER["XB"], DER["XO"]
PAD_R = 66.0
A_SHEAVE = np.array([PX - 10.018, Y1 - 19.708, Z1 + 73.434])     # balancer anchor A (j1r2_balancer A_OFF), world-fixed sheave on the pad
BLUE, DARK, CFC, ALC, ORANGE = (0.20, 0.34, 0.52), (0.13, 0.14, 0.17), (0.10, 0.10, 0.11), (0.78, 0.80, 0.83), (0.95, 0.45, 0.10)
# ------------------------------------------------------------------ oriented solids
def _frame(p0, p1, up=(1, 0, 0)):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float); ex = p1 - p0; Lh = np.linalg.norm(ex); ex /= Lh; u = np.asarray(up, float) - (np.asarray(up, float) @ ex) * ex
    if np.linalg.norm(u) < 1e-6: u = np.array([0, 0, 1.0]) - ex[2] * ex
    ez = u / np.linalg.norm(u); ey = np.cross(ez, ex); T = np.eye(4); T[:3, 0], T[:3, 1], T[:3, 2], T[:3, 3] = ex, ey, ez, (p0 + p1) / 2; return T, Lh
def boxbeam(p0, p1, b, h, t=None, up=(1, 0, 0), ext=0.0):
    """box section b (local y) x h (local z, along 'up'), hollow if t; axis p0->p1 (extended ext at both ends)."""
    T, Lh = _frame(p0, p1, up); o = trimesh.creation.box([Lh + 2 * ext, b, h])
    if t: o = DIFF(o, trimesh.creation.box([Lh + 2 * ext - 2 * t, b - 2 * t, h - 2 * t]))
    o.apply_transform(T); return o
def tubebeam(p0, p1, D, t=None, ext=0.0, n=32):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float); v = (p1 - p0) / np.linalg.norm(p1 - p0); a, b = p0 - ext * v, p1 + ext * v
    o = trimesh.creation.cylinder(D / 2, segment=[a, b], sections=n)
    if t: o = DIFF(o, trimesh.creation.cylinder(D / 2 - t, segment=[a - v, b + v], sections=n))
    return o
def node_ball(p, r): return L.sph(r, p, 2)
def P_(name, mesh, mat, colour, kind="printed", fill=1.0, mass_g=None, note=""):
    d = dict(mesh=mesh, mat=mat, colour=colour, kind=kind, fill=fill, note=note)
    if mass_g is not None: d["mass_g"] = mass_g
    return d
def pad_disc():
    """the pad plate itself (common): R66 x 6 PA-CF disc on the S1' axis (S1' AK70 on its rear face, flange PLACEHOLDER)."""
    return cyl("x", PAD_R, X0, XB, c=(Y1, Z1), n=96)
def pad_pt(psi_deg, r=PAD_R - 8.0, x=None):
    """point on the pad (psi from +y toward +z)."""
    a = math.radians(psi_deg); return np.array([XB - 3.0 if x is None else x, Y1 + r * math.cos(a), Z1 + r * math.sin(a)])
def rail_pt(z, dx=0.0, y=M.RAIL["y"]):
    """point on the rail REAR face (x = back + gap + t) at height z, shifted dx rearward."""
    return np.array([M.back_x(y, z) + M.RAIL["gap"] + M.RAIL["t"] + dx, y, z])
def rail_x(y, z): return M.back_x(y, z) + M.RAIL["gap"] + M.RAIL["t"]
# ------------------------------------------------------------------ frame helpers
def pad_nodes(fr, side):
    """pad centre (load node) + rigid rim nodes at psi 90/135/180/225/270/315 deg (pad disc 6 mm PA-CF with the motor flange behind: treated rigid)."""
    mp = (lambda p: p) if side == "R" else M.mirror; c = f"{side}_pad"; fr.node(c, mp(np.array([XB, Y1, Z1])))
    for psi in (90, 135, 180, 225, 270, 315, 0, 45):
        n = f"{side}_pad{psi}"; fr.node(n, mp(pad_pt(psi))); fr.rigid(c, n, tag="pad")
    return c
def link_bolts(fr, a, b, pts_uv, up=(1, 0, 0), tag="iface", d=4.0):
    """bolted joint a-b: steel bolt-group equivalent beam (A = n A_b, I = sum A_b d^2); the joint tag is used for the bolt-tension report."""
    fr.beam(a, b, M.bolt_group(pts_uv, d), "STEEL", up=up, tag=tag)
def rail_node_name(side, z): return f"{side}_rail{int(z)}"
# clasp feet re-seat (the chassis plate front face x -88 is gone and lies inside the S2 motor + stop-post sweep): feet within 110 mm of the S1' axis move forward
# to x -126 / -123 (S2 stop posts reach x -117 at 81..92 mm from the axis), the outer foot R1 stays at x -102; F5 stops at -123 to keep 20 mm over the blade.
FOOT_X = M.FOOT_X
def feet_xyz(): return {k: np.array([FOOT_X[k], v["y"], v["z"]]) for k, v in M.FEET.items()}
def clasp_hub_xyz(): return feet_xyz()["F1_upper_trap"]
R1_RAIL = np.array([-108.0, -103.0, -140.0])        # R1 foot bar lands on the rail lateral edge (rail zone, z -140)
def clasp_bracket_geo(anchor, via=None):
    """6 mm PA-CF bars (mid-planes FOOT_X): F5 / F4 -> hub (= F1 foot), hub -> (via) -> the concept's anchor. The R1 foot is dropped (R1 is a lifted standoff node; its bar ran 5 mm over the blade)."""
    f = feet_xyz(); h = f["F1_upper_trap"]; bars = [boxbeam(f[k], h, 16.0, 6.0, None, up=(1, 0, 0), ext=4.0) for k in ("F5_mid_trap", "F4_acromion")]
    pts = [h] + ([np.asarray(via, float)] if via is not None else []) + ([np.asarray(anchor, float)] if anchor is not None else [])
    bars += [boxbeam(a, b, 24.0, 6.0, None, up=(1, 0, 0), ext=6.0) for a, b in zip(pts[:-1], pts[1:])]
    return U(*bars, *[node_ball(f[k], 5.0) for k in ("F5_mid_trap", "F4_acromion", "F1_upper_trap")])
def clasp_bracket_frame(fr, side, anchor_node, via_xyz=None, w=24.0, t=6.0):
    mp = (lambda p: p) if side == "R" else M.mirror; hub = f"{side}_clasp_hub"
    if anchor_node is None: return
    if via_xyz is not None:
        v = f"{side}_clasp_via"; fr.node(v, mp(np.asarray(via_xyz, float))); fr.beam(hub, v, M.rect(w, t), "PACF", up=(1, 0, 0), tag="clasp_seat"); fr.beam(v, anchor_node, M.rect(w, t), "PACF", up=(1, 0, 0), tag="clasp_seat")
    else: fr.beam(hub, anchor_node, M.rect(w, t), "PACF", up=(1, 0, 0), tag="clasp_seat")
KNEE_A = lambda: pad_pt(180, PAD_R - 8, x=X0 + 2)     # leaves the pad medially (outside the S2 motor / stop-post sweep)
KNEE_B = np.array([-101.0, -98.0, -96.0])             # blade corner crossed at x -101 (back bulges to -127 here; outside the S2 sweep, r 122 from the S1' axis)
CLASP_VIA = np.array([-124.0, -72.0, -78.0])          # hub -> concept anchor path, forward of the stop-post band, >= 20 mm over the blade
# ================================================================== REF: J1 rev 2 as built (rail-top lap)
TONGUE = np.array([-113.0, -62.0, -120.0])          # wing tongue seat (scapular_frame TONGUE zone, gap 12) - retired with the chassis plate
RISER_MID = np.array([-107.0, -76.0, -92.0])        # underside of the J1 rev 2 riser beam (clasp anchor in REF / MJ2 / MJ3)
def ref_geo(base):
    out = {k: P_(k, base[k]["mesh"], base[k]["mat"], BLUE if k != "standoffs" else COL["steel"], fill=base[k].get("fill", 1.0)) for k in ("pad_riser", "standoffs", "fairing_lap")}
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(RISER_MID, via=CLASP_VIA), "PACF", BLUE, note="ASSUMED minimal fix (not in the J1 rev 2 CAD): clasp feet bars to the riser underside (the wing tongue is retired and the real tongue collides with the riser)")
    return out
def _ref_riser(fr, side):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = pad_nodes(fr, side)
    lap = f"{side}_lap"; fr.node(lap, mp(np.array([X0 + 4.75, -122.0, -77.0])))
    rm = f"{side}_rmid"; fr.node(rm, mp(RISER_MID + [0, 0, 18.0]))
    fr.beam(f"{side}_pad225", rm, M.box(39.0, 9.5, 1.8), "PACF", up=(1, 0, 0), tag="riser"); fr.beam(rm, lap, M.box(39.0, 9.5, 1.8), "PACF", up=(1, 0, 0), tag="riser")   # hollow box riser 9.5 deep (x) x 39 (z), wall 1.8
    link_bolts(fr, lap, rail_node_name(side, -80), [(-10, -9), (10, -9), (-10, 9), (10, 9)], tag="iface")  # 4 x M4 through D10 standoffs into the rail top end
    return pad
def _tongue_clasp(fr, side):
    clasp_bracket_frame(fr, side, f"{side}_rmid", via_xyz=CLASP_VIA)
def ref_frame(fr, side):
    pad = _ref_riser(fr, side); _tongue_clasp(fr, side); return pad
# ================================================================== MJ1: spine spar with the balancer inside
# corridor: leave the pad MEDIALLY (psi 180, outside the S2 sweep), cross y -95 below z -88 (neck), land on the LATERAL half of the rail (y -112) at z -105;
# a straight box spar (so the 151 mm spring tube fits) on 4 standoffs to rail bosses at z -110/-170/-230/-290. Balancer at 4:1 (tube 151 mm).
SPAR = dict(y=-104.0, b=40.0, h=44.0, t=2.4, z=(-105.0, -300.0), joints=(-120.0, -180.0, -240.0, -300.0), min_so=6.0)   # y -124..-84: clear of the spine-bridge ends (rail medial strip y < -124, z -241..-112)
def _spar_line():
    zz = np.linspace(SPAR["z"][0], SPAR["z"][1], 40); need = np.array([max(rail_x(y_, z) for y_ in (-118.0, -104.0, -86.0)) + SPAR["min_so"] + SPAR["h"] / 2 for z in zz])
    ztop, zbot = SPAR["z"]; t = (zz - ztop) / (zbot - ztop)
    best = None
    for slope in np.linspace(-0.3, 0.3, 121):
        c = float(np.max(need - slope * (zz - ztop)))
        dev = float(np.sum(c + slope * (zz - ztop) - need))
        if best is None or dev < best[0]: best = (dev, c, slope)
    _, c, slope = best; return lambda z: c + slope * (z - ztop)
SPX = None
def spar_x(z):
    global SPX
    if SPX is None: SPX = _spar_line()
    return float(SPX(z))
def mj1_pts():
    return KNEE_A(), KNEE_B, np.array([spar_x(SPAR["z"][0]) - 8, SPAR["y"] + 4, SPAR["z"][0] + 6])
def mj1_geo(base):
    out = {}; ztop, zbot = SPAR["z"]; top, bot = np.array([spar_x(ztop), SPAR["y"], ztop]), np.array([spar_x(zbot), SPAR["y"], zbot])
    spar = boxbeam(top, bot, SPAR["b"], SPAR["h"], SPAR["t"], up=(1, 0, 0), ext=2.0)
    spar = U(spar, cyl("z", 20.4, zbot + 40, ztop - 4, c=(spar_x(ztop - 100), SPAR["y"])) if False else spar)
    out["spar"] = P_("spar", spar, "PACF", BLUE, note="straight closed PA-CF box 44 x 44 x 2.4, z -105..-300, on 4 standoffs to the rail lateral half")
    a, b, c = mj1_pts(); out["knee"] = P_("knee", U(pad_disc(), boxbeam(a, b, 50.0, 16.0, 2.4, up=(1, 0, 0), ext=8.0), boxbeam(b, c, 50.0, 16.0, 2.4, up=(1, 0, 0), ext=8.0), node_ball(b, 10.0)),
                                          "PACF", BLUE, note="pad + 2-segment box knee 50 x 16 x 2.4: medial out of the S2 sweep, down below the neck line, onto the spar head")
    st = []
    for z in SPAR["joints"]:
        for yy in (-118.0, -106.0): st.append(cyl("x", 5.0, rail_x(yy, z) - 0.2, spar_x(z) - SPAR["h"] / 2 + 0.3, c=(yy, z), n=24))
    out["spar_standoffs"] = P_("spar_standoffs", U(*st), "PETGCF", COL["steel"])
    tt = np.array([spar_x(ztop - 14), SPAR["y"], ztop - 14]); tb = np.array([spar_x(ztop - 14 - 165), SPAR["y"], ztop - 14 - 165])
    out["bal_liner"] = P_("bal_liner", tubebeam(tt, tb, 40.4, 0.8), "PETGCF", ORANGE, note="4:1 spring tube 151 + caps inside the spar (spring OD 39.5 -> liner 40.4)")
    sh1 = np.array([spar_x(ztop) + 6, SPAR["y"] + 12, ztop + 6]); sh2 = np.array([-112.0, -52.0, 18.0])
    out["bal_sheaves"] = P_("bal_sheaves", U(*[cyl("x", 9.0, q[0] - 3, q[0] + 3, c=(q[1], q[2])) for q in (sh1, sh2, A_SHEAVE)]), "AL", ALC, kind="bought", mass_g=27.0)
    out["bal_cord_fixed"] = P_("bal_cord_fixed", U(trimesh.creation.cylinder(1.0, segment=[sh1, sh2], sections=8), trimesh.creation.cylinder(1.0, segment=[sh2, A_SHEAVE], sections=8)), "TPU", ORANGE, kind="cord", mass_g=1.0)
    out["cord_guide"] = P_("cord_guide", U(boxbeam(sh1, sh2, 10.0, 10.0, 1.6, up=(1, 0, 0)), boxbeam(sh2, A_SHEAVE, 10.0, 10.0, 1.6, up=(1, 0, 0))), "PACF", BLUE, note="closed cord channel 10 x 10 on the knee and the pad rim")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(b + [0, 2, 4], via=CLASP_VIA), "PACF", BLUE, note="clasp feet (re-seated) to the knee corner")
    return out
def mj1_frame(fr, side):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = pad_nodes(fr, side); prev = None; ztop = SPAR["z"][0]
    for z in (ztop,) + SPAR["joints"]:
        n = f"{side}_spar{int(z)}"; fr.node(n, mp(np.array([spar_x(z), SPAR["y"], z])))
        if prev: fr.beam(prev, n, M.box(SPAR["b"], SPAR["h"], SPAR["t"]), "PACF", up=(1, 0, 0), tag="spar")
        prev = n
    for z in SPAR["joints"]: link_bolts(fr, f"{side}_spar{int(z)}", rail_node_name(side, z), [(-6, 0), (6, 0)], tag="iface")
    a, b, c = mj1_pts(); fr.node(f"{side}_ka", mp(a)); fr.node(f"{side}_kb", mp(b)); fr.rigid(f"{side}_pad", f"{side}_ka", tag="pad")
    fr.beam(f"{side}_ka", f"{side}_kb", M.box(50.0, 16.0, 2.4), "PACF", up=(1, 0, 0), tag="knee"); fr.beam(f"{side}_kb", f"{side}_spar{int(ztop)}", M.box(50.0, 16.0, 2.4), "PACF", up=(1, 0, 0), tag="knee")
    clasp_bracket_frame(fr, side, f"{side}_kb", via_xyz=CLASP_VIA); return pad
# ================================================================== MJ2: shoulder-top truss (struts to the clasp top seats F1 / F4)
def seat_top(key): st = M.SITES[key]; return np.asarray(st["p"], float) + np.array([0, 0, 1.0]) * 26.0
def mj2_pts(): return dict(a4=pad_pt(290, PAD_R - 4, x=X0 - 6), a1=pad_pt(250, PAD_R - 4, x=X0 - 6), s4=seat_top("F4_acromion"), s1=seat_top("F1_upper_trap"))
def mj2_geo(base):
    out = {k: P_(k, base[k]["mesh"], base[k]["mat"], BLUE if k != "standoffs" else COL["steel"]) for k in ("pad_riser", "standoffs", "fairing_lap")}
    q = mj2_pts()
    out["strut_F4"] = P_("strut_F4", tubebeam(q["a4"], q["s4"], 12.0, 1.0), "CF", CFC, kind="bought", note="pultruded CF tube 12 x 1 to a post on the F4 (acromion) top seat")
    out["strut_F1"] = P_("strut_F1", tubebeam(q["a1"], q["s1"], 12.0, 1.0), "CF", CFC, kind="bought", note="to the F1 (upper trapezius) top seat")
    out["lugs"] = P_("lugs", U(*[node_ball(p, 9.0) for p in q.values()], *[tubebeam(q[k], q[k] - [0, 0, 24.0], 12.0) for k in ("s4", "s1")]), "PACF", BLUE, note="printed lugs; seat posts 24 mm on the clasp seat bosses")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(pad_pt(205, PAD_R - 6, x=X0 - 2), via=CLASP_VIA), "PACF", BLUE, note="clasp feet to the pad's medial-low rim (forward of the S2 stop posts)")
    return out
def mj2_frame(fr, side):
    pad = _ref_riser(fr, side); mp = (lambda p: p) if side == "R" else M.mirror; q = mj2_pts()
    for nm, key in (("a4", "F4"), ("a1", "F1")):
        n = f"{side}_{nm}"; fr.node(n, mp(q[nm])); fr.rigid(f"{side}_pad", n, tag="pad"); fr.beam(n, f"{side}_{key}", M.tube(12.0, 1.0), "CF", up=(1, 0, 0), tag="strut")
    fr.node(f"{side}_pc", ((lambda p: p) if side == "R" else M.mirror)(pad_pt(205, PAD_R - 6, x=X0 - 2))); fr.rigid(f"{side}_pad", f"{side}_pc", tag="pad"); clasp_bracket_frame(fr, side, f"{side}_pc", via_xyz=CLASP_VIA); return pad
# ================================================================== MJ3: bilateral yoke (left and right pads tied across the upper back, below the neck line)
YOKE = dict(z=-92.0, D=22.0, t=1.5, gap=30.0)
def yoke_pts():
    a = KNEE_A(); b = KNEE_B + [0, 0, 0]; m = np.array([M.back_x(M.SPINE_Y, YOKE["z"]) + YOKE["gap"], M.SPINE_Y, YOKE["z"]]); return a, b, m
def mj3_geo(base):
    out = {k: P_(k, base[k]["mesh"], base[k]["mat"], BLUE if k != "standoffs" else COL["steel"]) for k in ("pad_riser", "standoffs", "fairing_lap")}
    a, b, m = yoke_pts(); aL, bL = M.mirror(a), M.mirror(b)
    out["yoke_tube"] = P_("yoke_tube", U(tubebeam(a, b, YOKE["D"], YOKE["t"]), tubebeam(b, m, YOKE["D"], YOKE["t"]), tubebeam(m, bL, YOKE["D"], YOKE["t"]), tubebeam(bL, aL, YOKE["D"], YOKE["t"])), "CF", CFC, kind="bought", fill=0.5,
                         note="CF tube 22 x 1.5 pad-to-pad, bent at 2 printed knees, 30 mm off T2 (mass: half per side)")
    out["yoke_nodes"] = P_("yoke_nodes", U(node_ball(a, 14.0), node_ball(b, 12.0), node_ball(m, 14.0), node_ball(aL, 14.0), node_ball(bL, 12.0)), "PACF", BLUE, fill=0.5)
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(b + [0, 2, 4], via=CLASP_VIA), "PACF", BLUE, note="clasp feet to the yoke knee")
    return out
def mj3_frame(fr, side):
    pad = _ref_riser(fr, side); mp = (lambda p: p) if side == "R" else M.mirror; a, b, m = yoke_pts()
    fr.node(f"{side}_ya", mp(a)); fr.node(f"{side}_yb", mp(b)); fr.rigid(f"{side}_pad", f"{side}_ya", tag="pad"); fr.node("yoke_mid", m); clasp_bracket_frame(fr, side, f"{side}_yb", via_xyz=CLASP_VIA)
    fr.beam(f"{side}_ya", f"{side}_yb", M.tube(YOKE["D"], YOKE["t"]), "CF", up=(1, 0, 0), tag="yoke"); fr.beam(f"{side}_yb", "yoke_mid", M.tube(YOKE["D"], YOKE["t"]), "CF", up=(1, 0, 0), tag="yoke")
    return pad
# ================================================================== MJ4: carbon K-truss (pultruded tubes + printed nodes), same corridor as the knee
def mj4_pts():
    E_ = KNEE_B + np.array([0.0, -4.0, -6.0]); return dict(tA=(pad_pt(170, PAD_R - 6, x=X0 + 2), E_), tB=(pad_pt(205, PAD_R - 6, x=X0 + 2), E_),
                tC=(E_, rail_pt(-215, 9.0, y=-112.0)), tD=(E_, rail_pt(-112, 9.0, y=-112.0)))
def mj4_geo(base):
    out = {"pad": P_("pad", pad_disc(), "PACF", BLUE)}; pts = mj4_pts(); tubes, nodes = [], []
    for k, (a, b) in pts.items(): tubes.append(tubebeam(a, b, 14.0, 1.5)); nodes += [node_ball(a, 9.0), node_ball(b, 9.0)]
    out["tubes"] = P_("tubes", U(*tubes), "CF", CFC, kind="bought", note="4 pultruded CF tubes 14 x 1.5: 2 from the pad's medial rim to a node over the blade corner, 2 from it down to rail clamps (z -112, -215)")
    rc = [bx(rail_x(-112, z) - 0.2, rail_x(-112, z) + 7.0, -123.0, -101.0, z - 18.0, z + 18.0) for z in (-215.0, -112.0)]
    out["nodes"] = P_("nodes", U(*nodes, *rc), "PACF", BLUE, note="printed tube nodes; rail clamps 22 x 36 x 7 (3 x M4 each)")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(pts["tA"][1] + [0, 2, 4], via=CLASP_VIA), "PACF", BLUE, note="clasp feet to the K node")
    return out
def mj4_frame(fr, side):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = pad_nodes(fr, side); pts = mj4_pts(); E_ = f"{side}_kE"; fr.node(E_, mp(pts["tA"][1]))
    for k in ("tA", "tB"):
        na = f"{side}_{k}a"; fr.node(na, mp(pts[k][0])); fr.rigid(f"{side}_pad", na, tag="pad"); fr.beam(na, E_, M.tube(14.0, 1.5), "CF", up=(1, 0, 0), tag="tube")
    for k, zr in (("tC", -220), ("tD", -120)):
        nb = f"{side}_{k}b"; fr.node(nb, mp(pts[k][1])); fr.beam(E_, nb, M.tube(14.0, 1.5), "CF", up=(1, 0, 0), tag="tube"); link_bolts(fr, nb, rail_node_name(side, zr), [(-5, -12), (5, 0), (-5, 12)], tag="iface")
    clasp_bracket_frame(fr, side, E_, via_xyz=CLASP_VIA); return pad
# ================================================================== MJ5: quick-release dock (dovetail + cam; the arm module comes off as one piece)
DOCK = dict(y=-112.0, z=-125.0, rec=(-205.0, -100.0))
def mj5_geo(base):
    out = {}; y0, y1_ = -123.0, -101.0; z0, z1 = DOCK["rec"]
    Y, Z, X = L.rear_surface(y0, y1_, z0, z1, 4.0, 6.0); xr = X + M.RAIL["gap"] + M.RAIL["t"]
    out["receiver"] = P_("receiver", L.height_solid(Y, Z, xr + 0.2, xr + 3.2, "x"), "AL", ALC, kind="bought", note="6061 receiver 40 x 105 x 3 on the rail lateral half, 6 x M4")
    zc = DOCK["z"]; xd = rail_x(DOCK["y"], zc) + 3.2
    out["dovetail"] = P_("dovetail", prism([(-122.0, zc - 30), (-102.0, zc - 30), (-105.0, zc + 30), (-119.0, zc + 30)], "x", xd, xd + 10.0), "AL", ALC, kind="bought", note="6061 dovetail pair 60 long + over-centre cam (PLACEHOLDER part)")
    a = KNEE_A(); b = KNEE_B; c = np.array([xd + 14.0, DOCK["y"], zc + 8])
    out["riser_dock"] = P_("riser_dock", U(pad_disc(), boxbeam(a, b, 50.0, 16.0, 2.4, up=(1, 0, 0), ext=8.0), boxbeam(b, c, 50.0, 16.0, 2.4, up=(1, 0, 0), ext=8.0), node_ball(b, 10.0)),
                            "PACF", BLUE, note="pad + knee (as MJ1) ending in the female dovetail")
    out["cam_lever"] = P_("cam_lever", bx(xd + 10.0, xd + 16.0, -100.0, -93.0, zc - 25, zc + 25), "AL", ORANGE, kind="bought", mass_g=22.0)
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(b + [0, 2, 4], via=CLASP_VIA), "PACF", BLUE, note="clasp feet to the module knee (they come off with the module)")
    return out
def mj5_frame(fr, side):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = pad_nodes(fr, side); a = KNEE_A(); b = KNEE_B
    fr.node(f"{side}_ka", mp(a)); fr.node(f"{side}_kb", mp(b)); fr.rigid(f"{side}_pad", f"{side}_ka", tag="pad"); fr.beam(f"{side}_ka", f"{side}_kb", M.box(50.0, 16.0, 2.4), "PACF", up=(1, 0, 0), tag="knee")
    dk = f"{side}_dock"; fr.node(dk, mp(np.array([rail_x(DOCK["y"], DOCK["z"]) + 12.0, DOCK["y"], DOCK["z"]]))); fr.beam(f"{side}_kb", dk, M.box(50.0, 16.0, 2.4), "PACF", up=(1, 0, 0), tag="knee")
    prev = None
    for z in (-100.0, -120.0, -140.0, -160.0, -180.0, -200.0):
        n = f"{side}_rec{int(z)}"; fr.node(n, mp(np.array([rail_x(DOCK["y"], z) + 1.7, DOCK["y"], z])))
        if prev: fr.beam(prev, n, M.rect(40.0, 3.0), "AL", up=(1, 0, 0), tag="receiver")
        if z in (-100.0, -160.0, -200.0): link_bolts(fr, n, rail_node_name(side, z), [(-6, 0), (6, 0)], tag="rec_bolts")
        prev = n
    fr.beam(dk, f"{side}_rec-120", M.rect(40.0, 26.0), "AL", up=(1, 0, 0), tag="iface"); fr.beam(dk, f"{side}_rec-140", M.rect(40.0, 26.0), "AL", up=(1, 0, 0), tag="iface")
    clasp_bracket_frame(fr, side, f"{side}_kb", via_xyz=CLASP_VIA); return pad
# ================================================================== MJ6: monocoque (the fairing IS the structure: one closed PA-CF shell pad -> rail)
SHELL = dict(t=1.8, outline=[(-123.0, -200.0), (-101.0, -200.0), (-88.0, -112.0), (24.0, -104.0), (24.0, 10.0), (-20.0, 10.0), (-50.0, -40.0), (-78.0, -78.0), (-108.0, -96.0), (-123.0, -100.0)],
             bolts=[(-117.0, z) for z in (-120.0, -160.0, -195.0)] + [(-106.0, z) for z in (-120.0, -160.0, -195.0)])
def _offset_inner(Yg, Zg, Xs, gap):
    """x of a skin that keeps >= gap of TRUE 3-D distance to the wearer (iterated along +x), smoothed."""
    inner = Xs + gap
    for _ in range(6):
        q = np.c_[inner.ravel(), Yg.ravel(), Zg.ravel()]; d, i = L._tree()[0].query(q); short = np.maximum(gap.ravel() - d, 0.0).reshape(inner.shape); inner = inner + 1.15 * short
    return gaussian_filter(inner, 1.0, mode="nearest") + 0.6
def mj6_geo(base):
    import shapely.geometry as sg
    out = {}; ys = np.arange(-142.0, 28.1, 4.0); zs = np.arange(-206.0, 26.1, 4.0); Yg, Zg = np.meshgrid(ys, zs, indexing="ij")
    Xs = np.array([[M.back_x(y, z) for z in zs] for y in ys])
    gap = np.where(Yg > -100, 21.5, M.RAIL["gap"] + M.RAIL["t"] + 0.6); inner = _offset_inner(Yg, Zg, Xs, gap)
    inner = np.where(Yg <= -100, np.maximum(inner, Xs + gap), inner)
    outer = np.maximum(np.full_like(inner, XB + 2.0), inner + 14.0)
    ol = sg.Polygon(SHELL["outline"]).buffer(10.0).buffer(-10.0); oi = ol.buffer(-SHELL["t"])
    shell = L.I(L.height_solid(Yg, Zg, inner, outer, "x"), prism(list(ol.exterior.coords)[:-1], "x", -400, 100))
    core = L.I(L.height_solid(Yg, Zg, inner + SHELL["t"], outer - SHELL["t"], "x"), prism(list(oi.exterior.coords)[:-1], "x", -400, 100))
    sh = DIFF(shell, core); sh = DIFF(sh, cyl("x", 44.6, XB - 6, XB + 40, c=(Y1, Z1), n=96)); sh = U(sh, pad_disc())
    for (y, z) in SHELL["bolts"]: sh = U(sh, cyl("x", 6.0, rail_x(y, z) + 0.6, rail_x(y, z) + 14.0, c=(y, z), n=24))
    for (y, z) in SHELL["bolts"]: sh = DIFF(sh, cyl("x", 2.25, rail_x(y, z) - 1, rail_x(y, z) + 20.0, c=(y, z), n=20))
    out["monocoque"] = P_("monocoque", sh, "PACF", BLUE, note="closed PA-CF shell (1.8 mm skins) = pad + riser + fairing; inner skin >= 20 mm (3-D) over the blade, on the rail lateral half below the neck line (6 x M4)")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(None, via=CLASP_VIA), "PACF", BLUE, note="clasp feet to the shell's lower edge (the via node is moulded into the shell)")
    out["bal_cassette"] = P_("bal_cassette", cyl("x", 30.0, -124.0, -110.0, c=(-70.0, -48.0), n=48), "AL", ORANGE, kind="bought", mass_g=260.0, note="flat spiral-spring + cam-drum cassette (NEW balancer type, PLACEHOLDER: not designed)")
    return out
def mj6_frame(fr, side):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = pad_nodes(fr, side)
    a, b = f"{side}_sh_in", f"{side}_sh_rail"; fr.node(a, mp(np.array([X0 - 10, -70.0, -60.0]))); fr.node(b, mp(np.array([rail_x(-118, -120) + 8, -118.0, -120.0])))
    fr.beam(f"{side}_pad180", a, M.box(110.0, 20.0, 1.8), "PACF", up=(1, 0, 0), tag="shell"); fr.beam(f"{side}_pad225", a, M.box(80.0, 20.0, 1.8), "PACF", up=(1, 0, 0), tag="shell")
    fr.beam(a, b, M.box(70.0, 18.0, 1.8), "PACF", up=(1, 0, 0), tag="shell"); prev = b
    for z in (-160.0, -195.0):
        n = f"{side}_sh{int(z)}"; fr.node(n, mp(np.array([rail_x(-118, z) + 8, -118.0, z]))); fr.beam(prev, n, M.box(36.0, 16.0, 1.8), "PACF", up=(1, 0, 0), tag="shell"); prev = n
    for z, nn in ((-120.0, b), (-160.0, f"{side}_sh-160"), (-200.0, f"{side}_sh-195")): link_bolts(fr, nn, rail_node_name(side, z), [(-10, 0), (10, 0)], tag="iface")
    clasp_bracket_frame(fr, side, a)
    return pad
CONCEPTS = dict(
    REF=dict(title="J1 rev 2 as built: rail-top lap", geo=ref_geo, frame=ref_frame, balancer="radial", clasp="none in the J1 rev 2 CAD; modelled with an assumed bracket to the riser underside"),
    MJ1=dict(title="Spine spar, balancer inside", geo=mj1_geo, frame=mj1_frame, balancer="spar", clasp="feet re-seated on the knee corner"),
    MJ2=dict(title="Shoulder-top truss to F1 / F4", geo=mj2_geo, frame=mj2_frame, balancer="radial", clasp="feet re-seated on the pad rim"),
    MJ3=dict(title="Bilateral yoke across T2", geo=mj3_geo, frame=mj3_frame, balancer="radial", clasp="feet to the yoke knee"),
    MJ4=dict(title="Carbon K-truss", geo=mj4_geo, frame=mj4_frame, balancer="radial", clasp="feet to the K node"),
    MJ5=dict(title="Quick-release dock", geo=mj5_geo, frame=mj5_frame, balancer="radial", clasp="feet to the module knee (come off with the module)"),
    MJ6=dict(title="Monocoque: the fairing is the structure", geo=mj6_geo, frame=mj6_frame, balancer="cassette", clasp="feet re-seated on the shell rim"))
