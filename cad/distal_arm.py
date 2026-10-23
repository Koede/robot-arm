"""Distal arm (I6): S3 stop, elbow stops, forearm with W1 roll, 2-DOF wrist (W2, W3) in a ball-styled wrist, parallel gripper (G).
Right arm, robot frame, mm. CadQuery for printed parts. Session 2026-10-12c; hand section replaced 2026-10-13d.
The hand (3-finger double-jointed claw driven by G) now lives in claw_hand.py (stl/distal/claw/); this script builds forearm root, W1, distal forearm, W2 yoke.
"""
import cadquery as cq, math, os, json
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "distal"); os.makedirs(os.path.join(OUT, "proxies"), exist_ok=True)

# ------------------------------------------------------------------ layout
Z_ELB = 230.0; ZB = Z_ELB + 62.0
Y_MOUNT = 31.7
D_OFF = 57.0
Z_W = Z_ELB + 180.0
Z_W3 = Z_W + 45.0
Z_TIP = Z_W + 165.0
ST_L, ST_W, ST_H = 45.2, 24.7, 35.0
SHAFT_OFF = 11.0
HORN_D, HORN_P = 20.0, 4.0
CLR, WALL = 0.3, 2.4

def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = min(x0, x1), max(x0, x1); y0, y1 = min(y0, y1), max(y0, y1); z0, z1 = min(z0, z1), max(z0, z1)
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cyl_z(r, z0, z1, cx=0.0, cy=0.0): return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((cx, cy, z0))
def cyl_y(r, y0, y1, cx=0.0, cz=0.0): return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((cx, y0, cz))
def cyl_x(r, x0, x1, cy=0.0, cz=0.0): return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, cy, cz))
def tube_z(ro, ri, z0, z1, cx=0.0, cy=0.0): return cyl_z(ro, z0, z1, cx, cy).cut(cyl_z(ri, z0 - 1, z1 + 1, cx, cy))
parts, prox = {}, {}

# ================================================================== forearm root (rides on the fork bridge; E frame)
R_RO, R_RI = 30.0, 27.4
ZR0, ZR1 = ZB + 2.0, ZB + 76.0
W1_Z0 = ZR0 + 6.0
W1_TOP = W1_Z0 + ST_H
GEAR_Z0, GEAR_Z1 = W1_TOP + HORN_P, W1_TOP + HORN_P + 7.0
BRG_Z0 = GEAR_Z1 + 3.0
adapter = box(-14.0, 14.0, Y_MOUNT, Y_MOUNT + 6.0, Z_ELB + 20.0, ZR0)
for xx in (-8.0, 8.0):
    for zz in (ZB - 9.0, ZB - 27.0):
        adapter = adapter.cut(cyl_y(1.7, Y_MOUNT - 1, Y_MOUNT + 7, xx, zz)).cut(cyl_y(3.1, Y_MOUNT + 6.0 - 3.2, Y_MOUNT + 7, xx, zz))
root = adapter.union(cyl_z(R_RO, ZB, ZR1, 0, D_OFF)).cut(cyl_z(R_RI, ZR0, ZR1 + 1, 0, D_OFF))
root = root.cut(cyl_z(26.0 + 0.1, BRG_Z0, BRG_Z0 + 7.0, 0, D_OFF))
for sy in (-1, 1):
    rib = box(-ST_L / 2, ST_L / 2, D_OFF + sy * (ST_W / 2 + CLR + 0.5), D_OFF + sy * (R_RI + 0.5), W1_Z0, W1_TOP - 6.0)
    root = root.union(rib.intersect(cyl_z(R_RI + 0.5, W1_Z0, W1_TOP, 0, D_OFF)))
root = root.union(cyl_z(R_RI + 0.5, ZR0, W1_Z0, 0, D_OFF))
root = root.cut(box(-ST_L / 2 - CLR, ST_L / 2 + CLR, D_OFF - ST_W / 2 - CLR, D_OFF + ST_W / 2 + CLR, W1_Z0, W1_TOP + 0.01))
root = root.cut(box(-6.0, 6.0, D_OFF - 4.0, D_OFF + 4.0, ZB - 1, W1_Z0 + 0.5))
for sx in (-1, 1):
    for zz in (ZR0 + 14.0, ZR1 - 10.0):
        root = root.union(box(-4.0, 4.0, D_OFF + sx * (R_RO - 1.0) - (0 if sx > 0 else 8.0), D_OFF + sx * (R_RO - 1.0) + (8.0 if sx > 0 else 0), zz - 5.0, zz + 5.0))
        root = root.cut(cyl_x(1.7, -6.0, 6.0, D_OFF + sx * (R_RO + 3.0), zz))
parts["forearm_root_a"] = root.intersect(box(-80, 0, -60, 120, ZB - 1, ZR1 + 1))
parts["forearm_root_b"] = root.intersect(box(0, 80, -60, 120, ZB - 1, ZR1 + 1))
prox["sts_w1"] = box(-ST_L / 2, ST_L / 2, D_OFF - ST_W / 2, D_OFF + ST_W / 2, W1_Z0, W1_TOP)
SHAFT_W1 = (SHAFT_OFF, D_OFF)
pinion = cyl_z(12.0, GEAR_Z0, GEAR_Z1, *SHAFT_W1).cut(cyl_z(3.0, GEAR_Z0 - 1, GEAR_Z1 + 1, *SHAFT_W1))
parts["w1_pinion"] = pinion
prox["bearing_6808"] = tube_z(26.0, 20.0, BRG_Z0, BRG_Z0 + 7.0, 0, D_OFF)

# ================================================================== distal forearm (W1 frame)
RING_RO, RING_RI = 25.5, 21.0
dist = tube_z(RING_RO, RING_RI, GEAR_Z0, GEAR_Z1, 0, D_OFF)
dist = dist.union(tube_z(RING_RO, 17.0, GEAR_Z1, BRG_Z0, 0, D_OFF))
dist = dist.union(tube_z(20.0, 16.0, BRG_Z0, BRG_Z0 + 7.0, 0, D_OFF))
DT0 = ZR1 + 1.5
dist = dist.union(tube_z(20.0, 16.0, BRG_Z0 + 7.0, DT0 + 2.0, 0, D_OFF))
R_DO, R_DI = 29.0, 26.6
dist = dist.union(tube_z(R_DO, R_DI, DT0, Z_W - 23.5, 0, D_OFF))
dist = dist.union(cyl_z(R_DO, DT0, DT0 + 3.0, 0, D_OFF).cut(cyl_z(16.0, DT0 - 1, DT0 + 4, 0, D_OFF)))
W2_C = (0.0, D_OFF, Z_W - SHAFT_OFF)
prox["sts_w2"] = box(-ST_W / 2, ST_W / 2, D_OFF - ST_H / 2, D_OFF + ST_H / 2, W2_C[2] - ST_L / 2, W2_C[2] + ST_L / 2)
for sx in (-1, 1):
    dist = dist.union(box(sx * (ST_W / 2 + CLR + 0.5) if sx > 0 else -(R_DI + 0.5), (R_DI + 0.5) if sx > 0 else -(ST_W / 2 + CLR + 0.5),
                          D_OFF - 12.0, D_OFF + 12.0, W2_C[2] - ST_L / 2, Z_W - 22.0).intersect(cyl_z(R_DI + 0.5, DT0, Z_W, 0, D_OFF)))
dist = dist.cut(box(-ST_W / 2 - CLR, ST_W / 2 + CLR, D_OFF - ST_H / 2 - CLR, D_OFF + ST_H / 2 + CLR, W2_C[2] - ST_L / 2 - CLR, Z_W + 40))
WB_RI, WB_RO = 33.0, 35.4
ball = cq.Workplane("XY").sphere(WB_RO).cut(cq.Workplane("XY").sphere(WB_RI)).translate((0, D_OFF, Z_W))
ball = ball.intersect(box(-60, 60, D_OFF - 60, D_OFF + 60, Z_W - 26.0, Z_W + 22.0))
ball = ball.cut(box(-60, 60, D_OFF - 29.5, D_OFF + 29.5, Z_W - 24.0, Z_W + 40.0))
dist = dist.union(ball)
parts["forearm_dist"] = dist

# ================================================================== W2 yoke
YA_IN = ST_H / 2 + HORN_P + 1.0
YA_T = 3.5
ARM_R = 20.0
yoke = None
for sy in (-1, 1):
    y0 = D_OFF + sy * YA_IN; y1 = D_OFF + sy * (YA_IN + YA_T)
    arm = cyl_y(ARM_R, min(y0, y1), max(y0, y1), 0, Z_W).union(box(-12.0, 12.0, min(y0, y1), max(y0, y1), Z_W, Z_W + 32.0))
    yoke = arm if yoke is None else yoke.union(arm)
yoke = yoke.union(box(-12.0, 12.0, D_OFF - YA_IN - YA_T, D_OFF + YA_IN + YA_T, Z_W + 28.0, Z_W + 32.0))
W3_C = (0.0, D_OFF, Z_W3 + SHAFT_OFF)
prox["sts_w3"] = box(-ST_H / 2, ST_H / 2, D_OFF - ST_W / 2, D_OFF + ST_W / 2, W3_C[2] - ST_L / 2, W3_C[2] + ST_L / 2)
cradle = box(-ST_H / 2 - CLR - 2.4, ST_H / 2 + CLR + 2.4, D_OFF - ST_W / 2 - CLR - 2.4, D_OFF + ST_W / 2 + CLR + 2.4, Z_W + 32.0, W3_C[2] - ST_L / 2 + 12.0)
cradle = cradle.cut(box(-ST_H / 2 - CLR, ST_H / 2 + CLR, D_OFF - ST_W / 2 - CLR, D_OFF + ST_W / 2 + CLR, W3_C[2] - ST_L / 2 - CLR, W3_C[2] + 1))
yoke = yoke.union(cradle)
yoke = yoke.cut(cyl_y(3.0, D_OFF - 40, D_OFF + 40, 0, Z_W))
parts["w2_yoke"] = yoke

# ================================================================== hand: see claw_hand.py (3-finger double-jointed claw). Layout keys kept for arm_kin.py.
HA_IN = ST_H / 2 + HORN_P + 1.8; HA_T = 3.5
HB0 = W3_C[2] + ST_L / 2 + 3.0
G_C = (0.0, D_OFF, HB0 + 2.0 + CLR + ST_L / 2)       # claw: G servo body centre, L along z, shaft along y (toward the wrist), PLACEHOLDER offset
PZ0 = 0.0; STROKE = 0.0

meta = {}
def export(w, name, sub=""):
    p = os.path.join(OUT, sub, name + "_R.stl"); cq.exporters.export(w, p, tolerance=0.05, angularTolerance=0.08); return p
if __name__ == "__main__":
    for k, v in parts.items():
        export(v, k); sol = v.val(); bb = sol.BoundingBox()
        meta[k] = dict(vol_cm3=round(sol.Volume() / 1e3, 2), valid=sol.isValid(), bbox=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)])
    for k, v in prox.items(): export(v, k, "proxies")
    meta["_layout"] = dict(D_OFF=D_OFF, Z_W=Z_W, Z_W3=Z_W3, Z_TIP=Z_TIP, ZB=ZB, ZR1=ZR1, GEAR=[GEAR_Z0, GEAR_Z1], BRG_Z0=BRG_Z0, W2_C=W2_C, W3_C=W3_C, G_C=G_C, PZ0=PZ0, STROKE=STROKE,
                           SHAFT_OFF_PLACEHOLDER=SHAFT_OFF, HORN_PLACEHOLDER=[HORN_D, HORN_P])
    json.dump(meta, open(os.path.join(OUT, "distal.json"), "w"), indent=1)
    for k, m in meta.items(): print(k, m)
