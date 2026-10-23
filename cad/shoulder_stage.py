"""
Shoulder stage (S1 / S2 / S3) for the supernumerary arms -- CadQuery, all units mm.

Frame (RIGHT arm, right-handed): origin P = shoulder centre where S1, S2, S3 axes meet.
  +x = rearward, -x = forward, +y = lateral = wearer's RIGHT, +z = up.
  Arm hangs along -z at S1=S2=S3=0. Positive S2 swings the arm forward (toward -x) then up.
  Backplate front face is at x = -D_PLATE. Left arm = mirror in y (done at export, *_L.stl).

Layout (as EXPORTED, i.e. after the z-mirror in export()): S1 actuator sits BELOW P (output face up),
S2 body is medial (y<0) of P with axis y, S3 is a Dynamixel-class actuator ABOVE P on the S1 axis
(offset out of the S2 swing radius) and the arm rests pointing UP (+z). Positive S2 swings the arm
forward (-x): R = Ry(-theta2). Human elevation = 180 deg - theta2.
The geometry code below is written in the un-mirrored frame (S1 above P, arm down); read z as -z.

VERIFIED from vendor pages (Oct 2026):
  CubeMars AK70-10: dia 89 x 50.25 mm, 621 g, 8.3 N.m rated / 24.8 N.m peak, 24/48 V
  Dynamixel XM540-W270: 33.5 x 58.5 x 44 mm, 165 g, 10.6 N.m stall @12 V
NOT VERIFIED (datasheet drawings not retrievable as text) -> flagged *_UNVERIFIED below:
  AK70-10 output-flange bolt pattern / protrusion
  XM540 output axis direction (assumed normal to the 33.5 x 58.5 face), horn offset and diameter
"""
import cadquery as cq
import math, os, json

OUT = os.path.join(os.path.dirname(__file__), "..", "stl", "shoulder")
os.makedirs(OUT, exist_ok=True)

# ---------------------------------------------------------------- hardware envelopes
AK_R, AK_L = 44.5, 50.25            # verified
XM_X, XM_Y, XM_Z = 58.5, 33.5, 44.0  # verified dims; axis along Z (assumed) -> footprint 58.5 (x) x 33.5 (y)
AK_OUT_N_UNVERIFIED, AK_OUT_PCD_UNVERIFIED = 6, 60.0   # PLACEHOLDER output-flange pattern (M4)
XM_HORN_OFFSET_UNVERIFIED = 0.0                         # PLACEHOLDER: horn centred on body

# ---------------------------------------------------------------- print / fit rules
RB = AK_R + 0.1      # clamp bore radius: 0.1 mm radial running clearance on a 3D-printed bore
WALL = 5.0           # clamp ring wall
EAR_T = 8.0          # clamp ear thickness
EAR_OUT = 11.0       # ear projection beyond ring OD
GAP = 0.6            # split gap before tightening
CLR = 0.3            # pocket clearance round rigid parts (slip fit)

# ---------------------------------------------------------------- stage layout
D_PLATE = 80.0       # backplate face distance behind S1 axis (set by S2 body swing)
H1 = 56.0            # S1 output face height above P
Z1_0, LC1 = 62.0, 38.0     # S1 clamp z-range [62,100]
Y2_0, LC2 = -46.0, 42.0    # S2 clamp y-range [-46,-4]


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))

def cyl_z(r, z0, z1, cx=0.0, cy=0.0):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((cx, cy, z0))

def cyl_x(r, x0, x1, cy=0.0, cz=0.0):
    return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, cy, cz))

def cyl_y(r, y0, y1, cx=0.0, cz=0.0):
    # "XZ" workplane normal is -Y: extrude(+d) goes to -y, so shift to y1
    return cq.Workplane("XZ").circle(r).extrude(y1 - y0).translate((cx, y1, cz))

def clamp_local(Rb, Lc):
    """Split ring clamp in a local frame: bore axis = +z (0..Lc), split plane y=0.
    saddle half: y<=0 (belongs to the structure), strap half: y>=GAP.
    Returns (saddle_ring_with_ears, strap_solid, insert_cutters, bolt_xz)"""
    Ro = Rb + WALL
    ring = cq.Workplane("XY").circle(Ro).circle(Rb).extrude(Lc)
    sad = ring.intersect(box(-Ro - 1, Ro + 1, -Ro - 1, 0, 0, Lc))
    stp = ring.intersect(box(-Ro - 1, Ro + 1, GAP, Ro + 1, 0, Lc))
    xe0, xe1 = Rb, Ro + EAR_OUT
    for s in (1, -1):
        a, b = (xe0, xe1) if s > 0 else (-xe1, -xe0)
        sad = sad.union(box(a, b, -EAR_T, 0, 0, Lc))
        stp = stp.union(box(a, b, GAP, GAP + EAR_T, 0, Lc))
    xb = Ro + EAR_OUT / 2.0
    zs = (0.25 * Lc, 0.75 * Lc)
    ins = None
    for s in (1, -1):
        for z in zs:
            # M4 heat-set insert pocket (dia 5.6, 8 deep) in saddle ear, from split face
            c = cq.Workplane("XZ").center(s * xb, z).circle(2.8).extrude(8.0)  # y 0 -> -8
            ins = c if ins is None else ins.union(c)
            # strap ear: M4 clearance + counterbore for SHCS head
            th = cq.Workplane("XZ").center(s * xb, z).circle(2.25).extrude(-(EAR_T + 2)).translate((0, GAP - 1, 0))
            cb = cq.Workplane("XZ").center(s * xb, z).circle(4.1).extrude(-4.4).translate((0, GAP + EAR_T - 4.2, 0))
            stp = stp.cut(th).cut(cb)
    stp = stp.cut(cq.Workplane("XY").circle(Rb).extrude(Lc))  # keep bore clean through ears
    return sad, stp, ins, xb, zs

def to_s1(w):   # local (bore z, split normal y) -> global (bore z, split normal x, saddle at x<=0)
    return w.rotate((0, 0, 0), (0, 0, 1), -90).translate((0, 0, Z1_0))

def to_s2(w):   # local -> global (bore along +y, saddle at z>=0)
    return w.rotate((0, 0, 0), (1, 0, 0), -90).translate((0, Y2_0, 0))

parts = {}

# ================================================================= 1. S1 base block (fixed to backplate)
HW = 68.0           # was 75: bolts only need to clear the O99 clamp ring (7.6 mm wall), not carry load
blk = box(-D_PLATE, 0, -HW, HW, Z1_0, Z1_0 + LC1)
blk = blk.cut(cyl_z(RB, Z1_0 - 1, Z1_0 + LC1 + 1))
# lightening pockets, open at top (print with z=62 face on the bed)
blk = blk.cut(box(-D_PLATE + 4, -(RB + 4.5), -44, 44, Z1_0 + 4, Z1_0 + LC1 + 1))
# M5 through holes to backplate (4x) + counterbores on the clamp side
for y in (-62, 62):
    for z in (Z1_0 + LC1 / 2 - 13, Z1_0 + LC1 / 2 + 13):
        blk = blk.cut(cyl_x(2.75, -D_PLATE - 1, 1, y, z))
        blk = blk.cut(cyl_x(4.75, -5.5, 1, y, z))
sad1, stp1, ins1, xb1, zs1 = clamp_local(RB, LC1)
blk = blk.cut(to_s1(ins1))
parts["s1_base"] = blk
parts["s1_clamp"] = to_s1(stp1)

# ================================================================= 2. S1->S2 yoke (rotates with S1)
Ro = RB + WALL
XW = Ro + EAR_OUT              # half width 60.6
yk = box(-XW, XW, Y2_0, Y2_0 + LC2, 0, 50.0)
yk = yk.cut(cyl_y(RB, Y2_0 - 1, Y2_0 + LC2 + 1))
# open the middle slab (keep ring, ears z<8, top plate z>46, end plates)
mid = box(-XW - 1, XW + 1, Y2_0 + 5, Y2_0 + LC2 - 5, 14, 46).cut(cyl_y(Ro, Y2_0, Y2_0 + LC2))  # 6 mm web (z 8..14) under ears = no bridging
yk = yk.cut(mid)
# S1 output adapter disc (z 49.9..56) with PLACEHOLDER M4 pattern
yk = yk.union(cyl_z(50.0, 49.9, H1))
for i in range(AK_OUT_N_UNVERIFIED):
    a = 2 * math.pi * i / AK_OUT_N_UNVERIFIED
    x, y = AK_OUT_PCD_UNVERIFIED / 2 * math.cos(a), AK_OUT_PCD_UNVERIFIED / 2 * math.sin(a)
    yk = yk.cut(cyl_z(2.25, 40, H1 + 1, x, y))
    yk = yk.cut(cyl_z(4.1, 40, 50.0, x, y))        # head counterbore, tool enters from below
# S2 clamp insert pockets
sad2, stp2, ins2, xb2, zs2 = clamp_local(RB, LC2)
yk = yk.cut(to_s2(ins2))
parts["s1s2_yoke"] = yk
# rev 2026-10-12c: relief ring so the S3 lid (corner radius 75.4 about the S2 axis, y +-20.05) clears the S2 clamp ear tips by >= 3.2 mm
# over the whole S2 range: remove clamp material at r 59.8..80 about the S2 axis for |y| <= 23.25 (ear tips lose <= 1 mm; M4 bolts at r 55.1)
LID_RELIEF = cyl_y(80.0, -23.25, 23.25).cut(cyl_y(59.8, -24.0, 24.0))
parts["s2_clamp"] = to_s2(stp2).cut(LID_RELIEF)

# ================================================================= 3. S2 output -> S3 sleeve bracket (rotates with S2)
LUG_Y1 = 26.3
PL_Y0 = XM_Y / 2 + CLR            # 17.05  inner face of lateral wall
SX, SY = XM_X / 2 + CLR + 3.0, XM_Y / 2 + CLR + 3.0     # sleeve outer half sizes 32.55 / 20.05
ZB, ZT = -112.0, -68.0            # S3 body z range (top is outside S2 swing radius 59.2 + margin)
br = box(-SX, SX, PL_Y0, LUG_Y1, ZB, 30.0)                        # lateral web plate; top corners at radius 44.4 from P (< 49.9 yoke disc)
sleeve = box(-SX, SX, -SY, SY, ZB, ZT).cut(box(-SX + 3, SX - 3, -SY + 3, SY - 3, ZB - 1, ZT + 1))
br = br.union(sleeve)
# corner posts for M3 lid inserts
for sx in (1, -1):
    for sy in (1, -1):
        br = br.union(box(sx * (SX - 7) if sx > 0 else -SX, sx * SX if sx > 0 else -SX + 7,
                          sy * (SY - 7) if sy > 0 else -SY, sy * SY if sy > 0 else -SY + 7, ZB, ZT))
lipw = box(-SX, SX, -SY, SY, ZB, ZB + 3).cut(box(-(XM_X / 2 - 3), XM_X / 2 - 3, -(XM_Y / 2 - 3), XM_Y / 2 - 3, ZB - 1, ZB + 4))
br = br.union(lipw)
# cavity re-cut so posts/lip do not intrude on the body envelope above the lip
br = br.cut(box(-(XM_X / 2 + CLR), XM_X / 2 + CLR, -(XM_Y / 2 + CLR), XM_Y / 2 + CLR, ZB + 3, ZT + 0.01))
# 4x M4 plate-to-adapter screws (heads counterbored on the lateral face y=26.3), clear of the S2 flange pattern
ADP_SCREWS = [(sx * 14.1, sz * 14.1) for sx in (1, -1) for sz in (1, -1)]
for (x, z) in ADP_SCREWS:
    br = br.cut(cyl_y(2.25, PL_Y0 - 1, LUG_Y1 + 1, x, z))
    br = br.cut(cyl_y(4.1, LUG_Y1 - 4.2, LUG_Y1 + 1, x, z))

# S2 output adapter: flat disc between the S2 output flange and the plate. Prints flat, no supports.
adp = cyl_y(40.0, 0.5, PL_Y0)
for i in range(AK_OUT_N_UNVERIFIED):          # PLACEHOLDER flange pattern, M4 heads counterbored from the plate side
    a = 2 * math.pi * i / AK_OUT_N_UNVERIFIED
    x, z = AK_OUT_PCD_UNVERIFIED / 2 * math.cos(a), AK_OUT_PCD_UNVERIFIED / 2 * math.sin(a)
    adp = adp.cut(cyl_y(2.25, -1, PL_Y0 + 1, x, z))
    adp = adp.cut(cyl_y(4.1, PL_Y0 - 5.0, PL_Y0 + 1, x, z))
for (x, z) in ADP_SCREWS:                      # M4 heat-set inserts (dia 5.6 x 8) on the plate face
    adp = adp.cut(cyl_y(2.8, PL_Y0 - 8.0, PL_Y0 + 1, x, z))
parts["s2_adapter"] = adp
# M3 insert pockets in corner posts (top face)
for sx in (1, -1):
    for sy in (1, -1):
        br = br.cut(cyl_z(2.0, ZT - 6.0, ZT + 1, sx * (SX - 3.5), sy * (SY - 3.5)))
parts["s3_bracket"] = br

lid = box(-SX, SX, -SY, SY, ZT, ZT + 5.0)
for sx in (1, -1):
    for sy in (1, -1):
        lid = lid.cut(cyl_z(1.7, ZT - 1, ZT + 6, sx * (SX - 3.5), sy * (SY - 3.5)))
        lid = lid.cut(cyl_z(3.0, ZT + 2.5, ZT + 6, sx * (SX - 3.5), sy * (SY - 3.5)))
parts["s3_lid"] = lid

# ================================================================= non-printed envelopes (collision proxies)
prox = {
    "ak_s1_body": cyl_z(AK_R, H1 + 0.5, H1 + 0.5 + AK_L),
    "ak_s2_body": cyl_y(AK_R, -AK_L, 0.0),
    "xm_s3_body": box(-XM_X / 2, XM_X / 2, -XM_Y / 2, XM_Y / 2, ZB, ZT),
    "upper_arm_proxy": cyl_z(32.0, -265.0, ZB),
}

def export(w, name, sub=""):
    # Layout flip: the stage above was modelled with the S1 body above P. Final layout is its mirror in z
    # (S1 body BELOW P, arm rests pointing UP along +z), so the arm can swing forward over the shoulder.
    w = w.mirror("XY")
    p = os.path.join(OUT, sub, name + ".stl")
    os.makedirs(os.path.dirname(p), exist_ok=True)
    cq.exporters.export(w, p, tolerance=0.03, angularTolerance=0.05)
    return p

if __name__ == "__main__":
    meta = {}
    for k, v in parts.items():
        export(v, k + "_R")
        bb = v.val().BoundingBox()
        meta[k] = dict(vol_cm3=round(v.val().Volume() / 1000, 1),
                       bbox=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)],
                       valid=v.val().isValid())
    for k, v in prox.items():
        export(v, k, "proxies")
    import trimesh, numpy as np, glob
    for f in glob.glob(os.path.join(OUT, "*_R.stl")):
        m = trimesh.load(f); m.apply_transform(np.diag([1, -1, 1, 1.0])); m.export(f.replace("_R.stl", "_L.stl"))
    json.dump(meta, open(os.path.join(OUT, "meta.json"), "w"), indent=1)
    print(json.dumps(meta, indent=1))
