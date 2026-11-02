"""J1 mount alternatives, round 2 (2026-11-02, SHOULDER / WING + HARNESS, concept study): SPINE-LED mounts, NO balancer (Forest: "no balancer maybe").
Round 1 (mj_concepts.py) showed the foam / body is not the limit (body-only floor 0.7-1.4 mm hand-tip sag at reach, rail pressure <= 2.4 kPa); the structure and its bolts are
(MJ4: 322 N on a 103 mm bolt pair, CF tubes 62 MPa). So the spine is used to (1) lengthen the bolt lever, (2) tie right and left rails into one stiff ladder so the
opposite side shares the load, (3) take the mount's top end off the rail top (rail unloading).
  MJ7  Spine ladder       : PA-CF box spar on the midline (floating 24 mm over the spinous processes), z -112..-328, tied to the real spine bridge + 2 new rungs;
                            both pads' knees meet at the spar head (a T).
  MJ8  K-truss, long leg  : MJ4 geometry with the lower leg moved from rail z -215 to z -320 (lever 103 -> 208 mm). No spine member: isolates the lever effect.
  MJ10 K-truss to the spine head : MJ4's pultruded CF tubes, but the two lower legs land on the spine spar head / upper spar (a 60 mm lever on the spar) instead of on rail clamps.
  (A straight pultruded CF mast was tried as MJ9 and dropped: a straight tube cannot follow the lumbar curve and ends up 50 mm proud of the back.)
Midline members are modelled once (built in the R pass); their mass is split half per side so mount_mass_g stays "per side" like round 1.
Proxy geometry (CAD-as-code primitives); fastener patterns of bought parts are PLACEHOLDER."""
import numpy as np, trimesh, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import clib as L, mj_frame as M, mj_concepts as C
from clib import U, D as DIFF, box as bx
from mj_concepts import (boxbeam, tubebeam, node_ball, P_, pad_disc, pad_pt, rail_pt, rail_x, link_bolts, rail_node_name, clasp_bracket_geo, clasp_bracket_frame, KNEE_A, KNEE_B, CLASP_VIA,
                         PAD_R, BLUE, CFC, ALC, DARK, X0)
SY = M.SPINE_Y
# ------------------------------------------------------------------ the spine line
SP = dict(b=36.0, h=16.0, t=2.0, gap=26.0, z_top=-112.0, z_bot=-328.0, nodes=tuple(float(z) for z in range(-120, -321, -20)))
def _need(z): return max(M.back_x(y, z) for y in (-168.0, -150.0, -132.0)) + SP["gap"] + SP["h"] / 2        # spar CENTRE x so that the inner face keeps 'gap' over the groove + paraspinals
_ZZ = np.linspace(SP["z_top"], SP["z_bot"], 60); _NEED = np.array([_need(z) for z in _ZZ]) + 7.0 * np.clip((_ZZ + 170.0) / 30.0, 0.0, 1.0)     # +7 near the head: the real spine-bridge ears top out at x -101 (y -124..-140, z -112..-130) and the knee crosses over them
_PC = np.polyfit(_ZZ, _NEED, 3); _PC[-1] += float(np.max(_NEED - np.polyval(_PC, _ZZ))) + 0.5            # cubic envelope (printable gentle curve), never below need
def sx(z): return float(np.polyval(_PC, z))
RUNGS_NEW = (-260.0, -300.0)                       # new rungs below the real spine bridge (bridge straps: z -140, -180, -220; stl spans -241..-112)
RUNG = dict(b=40.0, t=4.0, off=16.0)               # strap 40 (z) x 4 (x), inner face 16 mm over the back (rail rear face is +14.5)
def sb_x(z): return M.back_x(SY, z) + RUNG["off"] + RUNG["t"] / 2
def _surf(y0, y1_, z0, z1, f, step=4.0):
    ys = np.arange(y0, y1_ + 0.1, step); zs = np.arange(z0, z1 + 0.1, step); Y, Z = np.meshgrid(ys, zs, indexing="ij"); return Y, Z, np.array([[f(y, z) for z in zs] for y in ys])
def rung_mesh(zc):
    Y, Z, X = _surf(-187.0, -113.0, zc - RUNG["b"] / 2, zc + RUNG["b"] / 2, lambda y, z: M.back_x(y, z) + RUNG["off"]); return L.height_solid(Y, Z, X, X + RUNG["t"], "x")
def spar_mesh():
    ys = np.arange(SY - SP["b"] / 2, SY + SP["b"] / 2 + 0.1, 4.0); zs = np.arange(SP["z_bot"], SP["z_top"] + 0.1, 4.0); Y, Z = np.meshgrid(ys, zs, indexing="ij")
    X = np.array([[sx(z) - SP["h"] / 2 for z in zs] for _ in ys]); return L.height_solid(Y, Z, X, X + SP["h"], "x")
# ------------------------------------------------------------------ knees: pad -> blade corner -> spar head (right and mirrored left)
HEAD_Z = -120.0
KH = 22.0                                                      # knee depth (x): 22 (16 in MJ1) - bending stiffness x 2.6
def head_xyz(): return np.array([sx(HEAD_Z), SY, HEAD_Z])
def knee_pts():
    a, kb, h = KNEE_A(), KNEE_B + np.array([5.0, 0.0, 0.0]), head_xyz()                       # KB +5 mm out: the 22 mm knee must clear the rail edge (rear face x -110) by 3
    zw = -112.0; xw = max(rail_x(y, zw) for y in np.arange(-140.0, -99.0, 4.0)) + 9.0 + KH / 2       # waypoint above the rail top strip with >= 4 mm over the rail rear face
    w = np.array([max(xw, 0.5 * (kb[0] + h[0])), -124.0, zw]); return a, kb, w, h
def knee_mesh(mirror=False):
    a, kb, w, h = knee_pts(); mp = M.mirror if mirror else (lambda p: p)
    return U(boxbeam(mp(a), mp(kb), 44.0, KH, 2.4, up=(1, 0, 0), ext=8.0), boxbeam(mp(kb), mp(w), 40.0, KH, 2.4, up=(1, 0, 0), ext=6.0), boxbeam(mp(w), mp(h), 40.0, KH, 2.4, up=(1, 0, 0), ext=6.0), node_ball(mp(kb), 10.0), node_ball(mp(w), 9.0))
def _half(mesh, mat, fill):
    return float(abs(mesh.volume) / 1000.0 * (dict(L.DENS, CF=1.55)).get(mat, 1.3) * fill) / 2.0
def _ladder_parts(spar_kind="box"):
    out = {}
    out["knees"] = P_("knees", U(knee_mesh(False), knee_mesh(True)), "PACF", BLUE, fill=0.5, note="2.4 mm PA-CF box knees 44/40 x 22, pad -> blade corner -> spar head, right + left drawn in the R file (mass: half = one side)")
    out["pad"] = P_("pad", pad_disc(), "PACF", BLUE, note="R66 x 6 PA-CF pad disc, S1' AK70 on its rear face (flange PLACEHOLDER)")
    if True:
        sp = spar_mesh(); out["spar"] = P_("spar", sp, "PACF", BLUE, fill=0.30, mass_g=_half(sp, "PACF", 0.30), note="closed PA-CF box 36 x 16 x 2.0 (drawn solid, mass at the hollow fill 0.30, half per side), z -112..-328, curved to follow the back; inner face 24 mm over the spine groove (no skin load on the spinous processes)")
    rz = RUNGS_NEW if True else ()
    rg = U(*[rung_mesh(z) for z in rz]); out["rungs"] = P_("rungs", rg, "PACF", BLUE, mass_g=_half(rg, "PACF", 1.0), note="2 new rungs 40 x 4 PA-CF at z -260 / -300, rail to rail across the spine, 2 x M4 per rail; the existing spine bridge (z -241..-112) carries the upper three")
    if True:
        pst = [L.cyl("x", 5.0, sb_x(z) + RUNG["t"] / 2 - 0.2, sx(z) - SP["h"] / 2 + 0.3, c=(SY, z), n=24) for z in (-140.0, -180.0, -220.0, -260.0, -300.0)]; ps = U(*pst)
        out["posts"] = P_("posts", ps, "PETGCF", COL_ST, mass_g=_half(ps, "PETGCF", 1.0), note="5 posts D10 from the rungs / bridge straps to the spar, 2 x M3")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(knee_pts()[1] + [0, 2, 4], via=CLASP_VIA), "PACF", BLUE, note="clasp feet (re-seated) to the knee corner")
    return out
COL_ST = tuple(C.COL["steel"])
# ------------------------------------------------------------------ frame
def _ladder_frame(fr, side, spar_kind):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = C.pad_nodes(fr, side); a, kb, w, h = knee_pts()
    if side == "R":                                                                      # midline members are built once
        zs = SP["nodes"]; prev = None
        for z in zs:
            x = sx(z); fr.node(f"sp{int(z)}", np.array([x, SY, z]))
            if prev: fr.beam(prev, f"sp{int(z)}", M.box(SP["b"], SP["h"], SP["t"]), "PACF", up=(1, 0, 0), tag="spar")
            prev = f"sp{int(z)}"
        for z in RUNGS_NEW:                                                              # new rungs: R rail - mid - L rail
            n = f"sb{int(z)}"; fr.node(n, np.array([sb_x(z), SY, z]))
            fr.beam(f"R_rail{int(z)}", n, M.rect(RUNG["b"], RUNG["t"]), "PACF", up=(1, 0, 0), tag="spine_bridge"); fr.beam(n, f"L_rail{int(z)}", M.rect(RUNG["b"], RUNG["t"]), "PACF", up=(1, 0, 0), tag="spine_bridge")
        for z in (-140.0, -180.0, -220.0, -260.0, -300.0):                              # posts / saddles rung -> spar
            fr.beam(f"sb{int(z)}", f"sp{int(z)}", M.rect(10.0, 10.0), "PETGCF", up=(1, 0, 0), tag="spar_post")
    fr.node(f"{side}_ka", mp(a)); fr.node(f"{side}_kb", mp(kb)); fr.node(f"{side}_kw", mp(w)); fr.rigid(f"{side}_pad", f"{side}_ka", tag="pad")
    for p, q, b_ in ((f"{side}_ka", f"{side}_kb", 44.0), (f"{side}_kb", f"{side}_kw", 40.0)): fr.beam(p, q, M.box(b_, KH, 2.4), "PACF", up=(1, 0, 0), tag="knee")
    fr.beam(f"{side}_kw", f"sp{int(HEAD_Z)}", M.box(40.0, KH, 2.4), "PACF", up=(1, 0, 0), tag="knee")
    clasp_bracket_frame(fr, side, f"{side}_kb", via_xyz=CLASP_VIA); return pad
def mj7_geo(base): return _ladder_parts("box")
def mj7_frame(fr, side): return _ladder_frame(fr, side, "box")
# ------------------------------------------------------------------ MJ8: K-truss, long leg
LONG_Z = -320.0
def mj8_pts():
    E_ = KNEE_B + np.array([0.0, -4.0, -6.0]); return dict(tA=(pad_pt(170, PAD_R - 6, x=X0 + 2), E_), tB=(pad_pt(205, PAD_R - 6, x=X0 + 2), E_), tC=(E_, rail_pt(LONG_Z, 9.0, y=-112.0)), tD=(E_, rail_pt(-112, 9.0, y=-112.0)))
def mj8_geo(base):
    out = {"pad": P_("pad", pad_disc(), "PACF", BLUE)}; pts = mj8_pts(); tubes, nodes = [], []
    for k, (a, b) in pts.items(): tubes.append(tubebeam(a, b, 14.0, 1.5)); nodes += [node_ball(a, 9.0), node_ball(b, 9.0)]
    out["tubes"] = P_("tubes", U(*tubes), "CF", CFC, kind="bought", note="4 pultruded CF tubes 14 x 1.5: 2 from the pad rim to the K node over the blade corner, the lower leg runs 208 mm down to a rail clamp at z -320")
    rc = [bx(rail_x(-112, z) - 0.2, rail_x(-112, z) + 7.0, -123.0, -101.0, z - 18.0, z + 18.0) for z in (LONG_Z, -112.0)]
    out["nodes"] = P_("nodes", U(*nodes, *rc), "PACF", BLUE, note="printed tube nodes; rail clamps 22 x 36 x 7 (3 x M4 each)")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(pts["tA"][1] + [0, 2, 4], via=CLASP_VIA), "PACF", BLUE, note="clasp feet to the K node")
    return out
def mj8_frame(fr, side):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = C.pad_nodes(fr, side); pts = mj8_pts(); E_ = f"{side}_kE"; fr.node(E_, mp(pts["tA"][1]))
    for k in ("tA", "tB"):
        na = f"{side}_{k}a"; fr.node(na, mp(pts[k][0])); fr.rigid(f"{side}_pad", na, tag="pad"); fr.beam(na, E_, M.tube(14.0, 1.5), "CF", up=(1, 0, 0), tag="tube")
    for k, zr in (("tC", int(LONG_Z)), ("tD", -120)):
        nb = f"{side}_{k}b"; fr.node(nb, mp(pts[k][1])); fr.beam(E_, nb, M.tube(14.0, 1.5), "CF", up=(1, 0, 0), tag="tube"); link_bolts(fr, nb, rail_node_name(side, zr), [(-5, -12), (5, 0), (-5, 12)], tag="iface")
    clasp_bracket_frame(fr, side, E_, via_xyz=CLASP_VIA); return pad
# ------------------------------------------------------------------ MJ10: K-truss whose lower legs land on the spar (head z -120, and z -180)
HEAD2_Z = -180.0
def mj10_pts(mirror=False):
    mp = M.mirror if mirror else (lambda p: p); E_ = KNEE_B + np.array([4.0, -4.0, -6.0]); h1, h2 = head_xyz(), np.array([sx(HEAD2_Z), SY, HEAD2_Z])
    d = dict(tA=(pad_pt(170, PAD_R - 6, x=X0 + 2), E_), tB=(pad_pt(205, PAD_R - 6, x=X0 + 2), E_), tC=(E_, h1 + [17.0, 0, 0]), tD=(E_, h2 + [17.0, 0, 0]))
    return {k: (mp(a), mp(b)) for k, (a, b) in d.items()}
def mj10_geo(base):
    out = {"pad": P_("pad", pad_disc(), "PACF", BLUE)}; tubes, nodes = [], []
    for mir in (False, True):
        for k, (a, b) in mj10_pts(mir).items():
            if mir and k in ("tA", "tB"): continue
            tubes.append(tubebeam(a, b, 14.0, 1.5)); nodes += [node_ball(a, 9.0), node_ball(b, 9.0)]
    tt = U(*tubes); out["tubes"] = P_("tubes", tt, "CF", CFC, kind="bought", fill=0.5, note="CF tubes 14 x 1.5: 2 from the pad rim to the K node over the blade corner, 2 from it to the spar (z -120 and z -180); right + left in the R file (mass: half = one side)")
    nn = U(*nodes, *[L.cyl("x", 11.0, sx(z) + 6.0, sx(z) + 23.0, c=(SY, z), n=32) for z in (HEAD_Z, HEAD2_Z)]); out["nodes"] = P_("nodes", nn, "PACF", BLUE, fill=0.6, note="printed tube nodes; spar-side lugs D22 (2 x M3 into the spar)")
    sp = spar_mesh(); out["spar"] = P_("spar", sp, "PACF", BLUE, fill=0.30, mass_g=_half(sp, "PACF", 0.30), note="closed PA-CF box 36 x 16 x 2.0 (drawn solid, mass at the hollow fill 0.30, half per side), z -112..-328, curved to follow the back")
    pst = [L.cyl("x", 5.0, sb_x(z) + RUNG["t"] / 2 - 0.2, sx(z) - SP["h"] / 2 + 0.3, c=(SY, z), n=24) for z in (-140.0, -180.0, -220.0, -260.0, -300.0)]; ps = U(*pst); out["posts"] = P_("posts", ps, "PETGCF", COL_ST, mass_g=_half(ps, "PETGCF", 1.0), note="5 posts D10, 2 x M3")
    rg = U(*[rung_mesh(z) for z in RUNGS_NEW]); out["rungs"] = P_("rungs", rg, "PACF", BLUE, mass_g=_half(rg, "PACF", 1.0), note="2 new rungs 40 x 4 PA-CF at z -260 / -300 (half per side)")
    out["clasp_bracket"] = P_("clasp_bracket", clasp_bracket_geo(mj10_pts()["tA"][1] + [0, 2, 4], via=CLASP_VIA), "PACF", BLUE, note="clasp feet to the K node")
    return out
def _midline(fr):
    zs = SP["nodes"]; prev = None
    for z in zs:
        fr.node(f"sp{int(z)}", np.array([sx(z), SY, z]))
        if prev: fr.beam(prev, f"sp{int(z)}", M.box(SP["b"], SP["h"], SP["t"]), "PACF", up=(1, 0, 0), tag="spar")
        prev = f"sp{int(z)}"
    for z in RUNGS_NEW:
        n = f"sb{int(z)}"; fr.node(n, np.array([sb_x(z), SY, z])); fr.beam(f"R_rail{int(z)}", n, M.rect(RUNG["b"], RUNG["t"]), "PACF", up=(1, 0, 0), tag="spine_bridge"); fr.beam(n, f"L_rail{int(z)}", M.rect(RUNG["b"], RUNG["t"]), "PACF", up=(1, 0, 0), tag="spine_bridge")
    for z in (-140.0, -180.0, -220.0, -260.0, -300.0): fr.beam(f"sb{int(z)}", f"sp{int(z)}", M.rect(10.0, 10.0), "PETGCF", up=(1, 0, 0), tag="spar_post")
def mj10_frame(fr, side):
    mp = (lambda p: p) if side == "R" else M.mirror; pad = C.pad_nodes(fr, side)
    if side == "R": _midline(fr)
    pts = mj10_pts(); E_ = f"{side}_kE"; fr.node(E_, mp(pts["tA"][1]))
    for k in ("tA", "tB"):
        na = f"{side}_{k}a"; fr.node(na, mp(pts[k][0])); fr.rigid(f"{side}_pad", na, tag="pad"); fr.beam(na, E_, M.tube(14.0, 1.5), "CF", up=(1, 0, 0), tag="tube")
    for k, z in (("tC", HEAD_Z), ("tD", HEAD2_Z)): fr.beam(E_, f"sp{int(z)}", M.tube(14.0, 1.5), "CF", up=(1, 0, 0), tag="tube")
    clasp_bracket_frame(fr, side, E_, via_xyz=CLASP_VIA); return pad
C.CONCEPTS.update(
    MJ7=dict(title="Spine ladder: midline spar + rungs", geo=mj7_geo, frame=mj7_frame, balancer="none", clasp="feet to the knee corner"),
    MJ8=dict(title="K-truss, long leg to z -320", geo=mj8_geo, frame=mj8_frame, balancer="none", clasp="feet to the K node"),
    MJ10=dict(title="K-truss to the spine head", geo=mj10_geo, frame=mj10_frame, balancer="none", clasp="feet to the K node"))
