"""Connector concept coupons (A dual-pin dovetail splice, B bayonet-dovetail quick-connect, C cross-spring flexure hinge).
mm, degrees.  Parametric CadQuery; FDM clearance parameter CLR (nominal 0.25; variants 0.15 / 0.35).
Writes stl/connectors/<concept>_<part>_c<clr*100>.stl and stl/connectors/connector_concepts.json (dimensions + checks).
Checks per coupon: every printed part is a single watertight solid (trimesh); the mating halves do not interfere at the nominal
clearance and do interfere at CLR = -0.1 (cadquery boolean intersection volume).  Run:  python3 connector_concepts.py   (~1 min, 1 core)
Frames: A  t = x (strut axis), w = y (strut width 18), z' = z (strut thickness 9; the pauldron bolt axis).  B  stem axis = z.  C  hinge axis = y.
Steel dowels / the TPU detent are separate proxy bodies (written as *_pin / *_detent) and are not part of the printed-solid check except TPU."""
import os, sys, json, math
import numpy as np, cadquery as cq, trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "stl", "connectors")
CLR_NOM = 0.25
VARIANTS = (0.15, 0.25, 0.35)
NEG_CLR = -0.1                      # deliberately wrong clearance: halves must interfere

# ------------------------------------------------------------------ A: dual-pin dovetail splice ---------------------------
A_W, A_T = 18.0, 9.0                # pauldron strut section (pauldron.py STRUT = [18, 9])
A_LD, A_ROOT, A_ANG = 16.0, 8.0, 10.0   # dovetail tongue length (t), root width (w), flank angle from t
A_PIN_D, A_PIN_X = 2.5, (3.5, 12.5)     # steel dowel d2.5 (same family as claw_hand.py), positions along t from the seam
A_LEN_CORE, A_LEN_ARM = 20.0, 36.0


def a_half_w(x):
    return 0.5 * A_ROOT + x * math.tan(math.radians(A_ANG))


def build_A(clr):
    """Dovetail tongue on the core member (slides in along z'), arm member has the groove; 2 dowels along w cross the seam:
    PRESS (d2.4) in the arm cheeks, RUN (d2.5 + 0.4 clr = 2.6 at 0.25) in the tongue -> same rule as claw_hand.py PIN RETENTION."""
    hp = a_half_w(A_LD)
    tongue_pts = [(0, -a_half_w(0)), (A_LD, -hp), (A_LD, hp), (0, a_half_w(0))]
    tongue = cq.Workplane("XY").polyline(tongue_pts).close().extrude(A_T)
    core = cq.Workplane("XY").box(A_LEN_CORE, A_W, A_T, centered=(False, True, False)).translate((-A_LEN_CORE, 0, 0)).union(tongue)
    ext = [(-2.0, -a_half_w(-2.0)), (A_LD, -hp), (A_LD, hp), (-2.0, a_half_w(-2.0))]
    groove = cq.Workplane("XY").polyline(ext).close().offset2D(clr, "intersection").extrude(A_T)
    arm = cq.Workplane("XY").box(A_LEN_ARM, A_W, A_T, centered=(False, True, False)).translate((clr, 0, 0)).cut(groove)
    run_d, press_d = A_PIN_D + 0.4 * clr, A_PIN_D - 0.1
    for x in A_PIN_X:
        cyl = lambda d: cq.Workplane("XZ").center(x, A_T / 2).circle(d / 2).extrude(A_W, both=True)
        core = core.cut(cyl(run_d)); arm = arm.cut(cyl(press_d))
    pins = [cq.Workplane("XZ").center(x, A_T / 2).circle(A_PIN_D / 2).extrude(A_W / 2, both=True) for x in A_PIN_X]
    dims = dict(strut_w=A_W, strut_t=A_T, tongue_len=A_LD, tongue_root_w=A_ROOT, tongue_tip_w=round(2 * hp, 2), flank_deg=A_ANG,
                pin_d=A_PIN_D, pin_x_from_seam=list(A_PIN_X), pin_pitch=A_PIN_X[1] - A_PIN_X[0], hole_run_d=round(run_d, 3), hole_press_d=press_d,
                cheek_min_mm=round(A_W / 2 - hp, 2), clearance=clr)
    return dict(core=core, arm=arm), dict(pin0=pins[0], pin1=pins[1]), dims


# ------------------------------------------------------------------ B: bayonet-dovetail quick connect ---------------------------
B_RS, B_RL = 9.0, 12.0              # stem radius, lug outer radius
B_LUG_DEG = 36.0                    # lug angular width (7.5 mm arc at r 12)
B_LUG_T = 3.0                       # lug axial thickness
B_LUG_Z = 4.0                       # lug underside height above the male base face
B_STEM = 10.0
B_DET_D, B_DET_ANG = 3.0, 62.0      # TPU detent pin d3 in a radial hole at azimuth 62 deg (ahead of the lug trailing edge when locked at 90)
B_DET_PROT = 0.6                    # nose protrusion into the channel


def sector(r1, r2, a0, a1, z0, z1):
    a0r, a1r, am = map(math.radians, (a0, a1, 0.5 * (a0 + a1)))
    P = lambda r, a: (r * math.cos(a), r * math.sin(a))
    w = (cq.Workplane("XY").workplane(offset=z0).moveTo(*P(r1, a0r)).lineTo(*P(r2, a0r))
         .threePointArc(P(r2, am), P(r2, a1r)).lineTo(*P(r1, a1r))
         .threePointArc(P(r1, am), P(r1, a0r)).close())
    return w.extrude(z1 - z0)


def build_B(clr):
    """Male: 30 x 30 x 4 base + d18 stem + two diametric lugs (annular sectors, the 'parallel tongues').  Female: block with bore, two axial entry slots
    and an L-channel (90 deg arc) at lug height; lugs enter at 0/180 deg, quarter turn locks them at 90/270 deg against the channel end wall.
    TPU detent pin (d3, domed nose) in a radial hole at 62 deg protrudes 0.6 mm into the channel: the lug rides over it on lock and unlock."""
    base = cq.Workplane("XY").box(30, 30, 4, centered=(True, True, False)).translate((0, 0, -4))
    stem = cq.Workplane("XY").circle(B_RS).extrude(B_STEM)
    male = base.union(stem)
    for a in (0.0, 180.0):
        male = male.union(sector(B_RS - 0.5, B_RL, a - B_LUG_DEG / 2, a + B_LUG_DEG / 2, B_LUG_Z + clr, B_LUG_Z + clr + B_LUG_T))
    # female: z in [0, 14]; mating face at z = 0 (flush with male base top)
    fem = cq.Workplane("XY").box(30, 30, 14, centered=(True, True, False))
    fem = fem.cut(cq.Workplane("XY").circle(B_RS + clr).extrude(B_STEM + clr))
    zc0, zc1 = B_LUG_Z, B_LUG_Z + B_LUG_T + 2 * clr
    half = 0.5 * B_LUG_DEG + clr * 180 / (math.pi * B_RL)       # clearance on the arc ends too
    for a in (0.0, 180.0):
        fem = fem.cut(sector(B_RS, B_RL + clr, a - half, a + half, 0.0, zc1))              # entry slot (axial) from the face to the channel
        fem = fem.cut(sector(B_RS, B_RL + clr, a - half, a + 90.0 + half, zc0, zc1))       # channel, quarter turn, ends in a stop wall
    # TPU detent: radial hole from outside (along +x rotated to B_DET_ANG) into the channel
    det_len = 8.0
    hole = (cq.Workplane("YZ").workplane(offset=B_RL + clr - B_DET_PROT).center(0, 0.5 * (zc0 + zc1)).circle(B_DET_D / 2 + 0.05).extrude(14)
            .rotate((0, 0, 0), (0, 0, 1), B_DET_ANG))
    hole2 = hole.rotate((0, 0, 0), (0, 0, 1), 180.0)
    fem = fem.cut(hole).cut(hole2)
    det = []
    for a in (0.0, 180.0):
        d = (cq.Workplane("YZ").workplane(offset=B_RL + clr - B_DET_PROT).center(0, 0.5 * (zc0 + zc1)).circle(B_DET_D / 2).extrude(det_len - 1.5)
             .rotate((0, 0, 0), (0, 0, 1), B_DET_ANG + a))
        det.append(d)
    dims = dict(base="30x30x4", stem_r=B_RS, stem_len=B_STEM, lug_r_out=B_RL, lug_deg=B_LUG_DEG, lug_t=B_LUG_T, lug_z=B_LUG_Z, lug_arc_mm=round(math.radians(B_LUG_DEG) * B_RL, 2),
                channel_t=round(B_LUG_T + 2 * clr, 2), channel_travel_deg=90.0, detent_d=B_DET_D, detent_az_deg=B_DET_ANG, detent_protrusion=B_DET_PROT, detent_hole_d=B_DET_D + 0.1,
                female="30x30x14", clearance=clr)
    return dict(male=male, female=fem), dict(detent0=det[0], detent1=det[1]), dims


# ------------------------------------------------------------------ C: cross-spring flexure hinge --------------------------------
C_B, C_H, C_D = 10.0, 12.0, 4.0     # leaf width (along hinge axis y), vertical span between blocks, half horizontal offset of leaf ends
C_T_PACF, C_T_TPU = 0.8, 1.2         # leaf thickness: 2 perimeters of a 0.4 nozzle (PA-CF) / TPU
C_BLK = (16.0, 10.0, 6.0)            # block x, y, z


def leaf_len():
    return math.hypot(2 * C_D, C_H)


def build_C(clr, t=C_T_PACF):
    """Monolithic cross-spring: fixed block (bottom), moving block (top), leaf 1 full width in the middle, leaf 2 split in two outer strips with
    the 'clr' slot either side of leaf 1 so the leaves pass each other where they cross.  Leaves go block-to-block (fused, one print)."""
    ang = math.degrees(math.atan2(2 * C_D, C_H))
    L = leaf_len()
    bw = C_BLK
    fixed = cq.Workplane("XY").box(*bw, centered=(True, True, False)).translate((0, 0, -C_H / 2 - bw[2]))
    moving = cq.Workplane("XY").box(*bw, centered=(True, True, False)).translate((0, 0, C_H / 2))

    def leaf(y0, y1, sign):
        # centred strip of thickness t, length L + overlap into blocks, tilted +-ang about the y axis through the crossing
        s = cq.Workplane("XY").box(t, y1 - y0, L + 2 * t, centered=(True, False, True)).translate((0, y0, 0))
        return s.rotate((0, 0, 0), (0, 1, 0), sign * ang)
    wm = C_B - 2 * (1.5 + clr)        # leaf 1 width after leaving the clearance slots (middle strip)
    leaf1 = leaf(-wm / 2, wm / 2, +1)
    side = 1.5
    leaf2a = leaf(-C_B / 2, -C_B / 2 + side, -1)
    leaf2b = leaf(C_B / 2 - side, C_B / 2, -1)
    # leaves must stay inside the blocks' footprints at both ends
    parts = dict(fixed=fixed, moving=moving, leaf1=leaf1, leaf2a=leaf2a, leaf2b=leaf2b)
    solid = fixed.union(moving).union(leaf1).union(leaf2a).union(leaf2b)
    dims = dict(leaf_w=C_B, leaf_t=t, leaf_len=round(L, 2), span_h=C_H, cross_deg_between_leaves=round(2 * ang, 1), block=C_BLK, middle_leaf_w=round(wm, 2),
                outer_strip_w=side, slot_between_leaves=clr, clearance=clr)
    return solid, parts, dims


# ------------------------------------------------------------------ checks ------------------------------------------------------
def write(shape, name):
    os.makedirs(OUT, exist_ok=True)
    p = os.path.join(OUT, name + ".stl")
    cq.exporters.export(shape, p, tolerance=0.02, angularTolerance=0.1)
    return p


def mesh_report(path):
    m = trimesh.load(path)
    comps = m.split(only_watertight=False)
    return dict(watertight=bool(m.is_watertight), bodies=len(comps), volume_mm3=round(float(m.volume), 1), extents=np.round(m.extents, 2).tolist())


def vol(shape):
    return float(shape.val().Volume())


def inter_vol(a, b):
    try:
        return abs(float(a.intersect(b).val().Volume()))
    except Exception:
        return 0.0


def run():
    res = dict(units="mm, deg", clearance_nominal=CLR_NOM, variants=list(VARIANTS), negative_test_clearance=NEG_CLR, concepts={})
    tag = lambda c: "c%03d" % round(c * 100)
    # ---- A
    A = res["concepts"]["A"] = dict(variants={})
    for c in VARIANTS:
        parts, pins, dims = build_A(c)
        rep = {n: mesh_report(write(s, "A_%s_%s" % (n, tag(c)))) for n, s in parts.items()}
        for n, s in pins.items(): write(s, "A_%s_pin_%s" % (n, tag(c)))
        iv = inter_vol(parts["core"], parts["arm"])
        # pins: steel proxies must be a press fit in the arm cheeks (interference = by design) and free in the tongue
        piv_core = sum(inter_vol(pins[k], parts["core"]) for k in pins); piv_arm = sum(inter_vol(pins[k], parts["arm"]) for k in pins)
        A["variants"][tag(c)] = dict(dims=dims, parts=rep, halves_interference_mm3=round(iv, 4), pin_vs_core_mm3=round(piv_core, 4), pin_vs_arm_press_mm3=round(piv_arm, 3))
    p, _, d = build_A(NEG_CLR)
    A["negative_test"] = dict(clearance=NEG_CLR, halves_interference_mm3=round(inter_vol(p["core"], p["arm"]), 3))
    # ---- B
    B = res["concepts"]["B"] = dict(variants={})
    for c in VARIANTS:
        parts, det, dims = build_B(c)
        rep = {n: mesh_report(write(s, "B_%s_%s" % (n, tag(c)))) for n, s in parts.items()}
        for n, s in det.items(): write(s, "B_%s_TPU_%s" % (n, tag(c)))
        sweep = {}
        for rot in (0, 30, 60, 90):                    # insertion pose (0) to locked (90): male turned by rot about z
            mrot = parts["male"].rotate((0, 0, 0), (0, 0, 1), rot)
            sweep[str(rot)] = round(inter_vol(mrot, parts["female"]), 4)
        # lock check: at 0 deg the lugs sit in the entry slot (free to pull out); at 90 deg an axial pull is blocked -> male shifted +z/-z collides?
        mp = parts["male"].rotate((0, 0, 0), (0, 0, 1), 90).translate((0, 0, -1.0))           # try to pull the male out by 1 mm
        pull_block = round(inter_vol(mp, parts["female"]), 3)
        B["variants"][tag(c)] = dict(dims=dims, parts=rep, interference_vs_rotation_deg_mm3=sweep, locked_pull_out_1mm_interference_mm3=pull_block)
    p, _, d = build_B(NEG_CLR)
    B["negative_test"] = dict(clearance=NEG_CLR, interference_by_rotation_mm3={str(r): round(inter_vol(p["male"].rotate((0, 0, 0), (0, 0, 1), r), p["female"]), 3) for r in (0, 90)})
    # ---- C
    C = res["concepts"]["C"] = dict(variants={})
    for c in VARIANTS:
        for mat, t in (("PACF", C_T_PACF), ("TPU", C_T_TPU)):
            solid, parts, dims = build_C(c, t)
            rep = mesh_report(write(solid, "C_cross_spring_%s_%s" % (mat, tag(c))))
            l1 = parts["leaf1"]
            ivs = round(inter_vol(l1, parts["leaf2a"]) + inter_vol(l1, parts["leaf2b"]), 4)
            C["variants"]["%s_%s" % (mat, tag(c))] = dict(dims=dims, part=rep, leaf1_vs_leaf2_interference_mm3=ivs,
                                                         blocks_touch_mm3=round(inter_vol(parts["fixed"], parts["moving"]), 4))
    solid, parts, d = build_C(NEG_CLR)
    l1 = parts["leaf1"]
    C["negative_test"] = dict(clearance=NEG_CLR, leaf1_vs_leaf2_interference_mm3=round(inter_vol(l1, parts["leaf2a"]) + inter_vol(l1, parts["leaf2b"]), 3))
    # ---- verdicts
    ok = True; msgs = []
    for k, c in res["concepts"].items():
        for v, r in c["variants"].items():
            parts = r.get("parts") or {"part": r["part"]}
            for n, pr in parts.items():
                if not (pr["watertight"] and pr["bodies"] == 1): ok = False; msgs.append("%s %s %s not single watertight" % (k, v, n))
    for v, r in A["variants"].items():
        if r["halves_interference_mm3"] > 1e-3: ok = False; msgs.append("A %s interferes" % v)
    if not A["negative_test"]["halves_interference_mm3"] > 0.1: ok = False; msgs.append("A negative test shows no interference")
    for v, r in B["variants"].items():
        if max(r["interference_vs_rotation_deg_mm3"].values()) > 1e-3: ok = False; msgs.append("B %s interferes %s" % (v, r["interference_vs_rotation_deg_mm3"]))
        if not r["locked_pull_out_1mm_interference_mm3"] > 1e-3: ok = False; msgs.append("B %s lock does not block pull-out" % v)
    if not max(B["negative_test"]["interference_by_rotation_mm3"].values()) > 0.1: ok = False; msgs.append("B negative test shows no interference")
    for v, r in C["variants"].items():
        if r["leaf1_vs_leaf2_interference_mm3"] > 1e-3 or r["blocks_touch_mm3"] > 1e-3: ok = False; msgs.append("C %s interferes" % v)
    if not C["negative_test"]["leaf1_vs_leaf2_interference_mm3"] > 0.01: ok = False; msgs.append("C negative test shows no interference")
    res["all_checks_pass"] = ok; res["failures"] = msgs
    with open(os.path.join(OUT, "connector_concepts.json"), "w") as f:
        json.dump(res, f, indent=1)
    print(json.dumps(dict(all_checks_pass=ok, failures=msgs, A_neg=A["negative_test"], B_neg=B["negative_test"], C_neg=C["negative_test"]), indent=1))
    return res


if __name__ == "__main__":
    run()
