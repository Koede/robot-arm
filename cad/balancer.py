"""S2 gravity balancer (I1b), rev e 2026-10-15: ZERO-LENGTH spring.  Printed / machined parts, hardware list, patched yoke, TPU soft-landing bumpers.
CadQuery, mm, RIGHT arm, exported frame: origin P = shoulder centre, +x rearward, -x forward, +y lateral, +z up, arm rests pointing UP (+z); S2 swings it
forward (-x), R = Rz(-S1) Ry(-S2).  Every number comes from balancer_calc.py (single source).  Left arm = mirror in y (done at export, *_L.stl).

PARTS (frame in brackets)
  bal_anchor_R    (M1, PA12-CF printed, ONE part)  T-bar foot on the yoke's S1-output disc (rev d pad + the SAME 6 x M4 insert pattern, s1s2_yoke_bal unchanged)
                  + web + beam + guide tube along +y (axis through A = (0, ., 22)), closed lateral end with a hex-key access hole, medial flange ring with 3 x M3 inserts
  bal_tubecap_R   (M1, PA12-CF printed)  medial plug: flange + spigot (bearing No 1) + hub inside the spring bore (bearing No 2, span 23 mm), spring seat
  bal_swivel_R    (swivel, machined 7075-T6)  hollow shank (cord bore 4) in the 6800s + fork cheeks + bridge; rotates about the tube axis so the sheave plane follows B
  bal_sheave_R    (swivel, turned 6061 sheave on a GGB 0404DU bush + 4 mm pin; PLACEHOLDER shop part)
  bal_piston_R    (M1, PA12-CF)  spring cup at the lateral end of the spring, M4 insert for the cord adjuster
  tpu_bal_buffer_R (M1, TPU 85A, 6 mm)  ring against the 6 mm closed tube end: catches the piston if the cord breaks
  bal_collar_R / bal_collar_cap_R (M2, PA12-CF)  rev d clamp collar on the S3 sleeve; the boss block is replaced by a lug leg carrying the cord pin B
  bal_pin_proxy   (M2, steel ISO 7379 shoulder screw d6 x 10, M5, spacer + washer) proxy for the sweeps
  s1s2_yoke_bal_R  rev d patched yoke (6 x M4 heat-set pockets), unchanged interface
  tpu_s2_pos_bump_R / tpu_s2_neg_bump_R (TPU 95A)  crush-blade bumpers, rev f: first contact +91 / -35 (the passive stops), bottomed at ~+94 / -39
  s2_pad_pos_bal_R / s2_pad_neg_bal_R (PETG-CF)  rev f: ball_stops carriers with the face behind the bumper trimmed flat (bumper seat)
  tpu_s2_*_bump_solid_R  (proxy, not printed)  bottomed bumper as a rigid body for verify_stops.py
  spring_proxy (inside the tube, not swept: it never leaves the bore)
Run:  python3 -I balancer.py     (writes stl/balancer/*.stl and balancer_parts.json)"""
import os, sys, json, math
import numpy as np
import cadquery as cq
import trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import balancer_calc as bc
OUT = os.path.join(HERE, "..", "stl", "balancer"); os.makedirs(OUT, exist_ok=True)
STL = os.path.join(HERE, "..", "stl", "shoulder")

SP = bc.SPRING; TB = bc.tube_layout(SP)
A_X, A_Z, B_X, B_Z, Y_B, R_P, Y_C = bc.A_X, bc.A_Z, bc.B_X, bc.B_Z, bc.Y_B, bc.R_P, bc.Y_C
HOLE_COMP = 0.2            # printed through-hole diameter compensation (mm added to nominal)
CLR = 0.3

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cyl_y(r, y0, y1, cx=0.0, cz=0.0):
    return cq.Workplane("XZ").circle(r).extrude(y1 - y0).translate((cx, y1, cz))        # XZ normal is -y
def cyl_z(r, z0, z1, cx=0.0, cy=0.0):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((cx, cy, z0))
def cyl_x(r, x0, x1, cy=0.0, cz=0.0):
    return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, cy, cz))
def ring_y(ro, ri, y0, y1, cx=0.0, cz=0.0): return cyl_y(ro, y0, y1, cx, cz).cut(cyl_y(ri, y0 - 1, y1 + 1, cx, cz))

# ============================================================================================== 1. anchor + guide tube (M1)
DISC_TOP, PAD_T = -49.9, 6.0
INS_XY = [(-14.0, 36.0), (24.0, 35.0), (30.0, 12.0), (30.0, 24.0), (-30.0, 12.0), (-30.0, 24.0)]     # rev d pattern (s1s2_yoke_bal), unchanged
DOME_KEEP = 49.5           # keep-out sphere round P: humeral-head dome R 46.3 (M2) + 3.2
R_TO, R_TI = TB["OD"] / 2, TB["ID"] / 2
Y_FL0, Y_FL1, R_FL = 73.0, 79.0, 28.0          # tube medial flange ring (3 x M3 inserts), cap flange sits on its face y = 73
Y_T0 = Y_FL0                                   # tube bore opens at the flange face
Y_END, Y_IN_END = TB["y_end"], TB["inner_end_y"]
END_CHAMFER = 2.0          # rev e1 (gI): outer edge of the closed lateral tube end, two-arm clearance at S1 150
SWIVEL_KEEP_R = 19.5       # swept radius of sheave + cheeks about the A axis (16.3) + 3.2
INS_M3 = [(R_FL - 3.4, a) for a in (90.0, 210.0, 330.0)]       # (radius, angle about the tube axis from +z towards +x) of the 3 cap screws
def anchor():
    pad = box(-44.0, 44.0, 31.0, 52.0, DISC_TOP, DISC_TOP + PAD_T).union(box(24.0, 44.0, 8.0, 52.0, DISC_TOP, DISC_TOP + PAD_T)).union(box(-44.0, -24.0, 8.0, 52.0, DISC_TOP, DISC_TOP + PAD_T))
    web = box(-10.0, 10.0, 40.0, 60.0, DISC_TOP + PAD_T - 1.0, -10.0)
    beam = box(-10.0, 10.0, 40.0, 104.0, -14.0, A_Z - R_TO + 6.0)            # beam top merges into the tube's lower wall (load enters at the cap, y 70..80)
    rib = box(-2.5, 2.5, 60.0, 104.0, -14.0, A_Z)                             # vertical rib under the tube axis (stiffens beam-tube joint)
    tube = cyl_y(R_TO, Y_T0, Y_END, A_X, A_Z).union(cyl_y(R_FL, Y_FL0, Y_FL1, A_X, A_Z))
    a = pad.union(web).union(beam).union(rib).union(tube)
    a = a.cut(cyl_y(R_TI, Y_T0 - 1.0, Y_IN_END, A_X, A_Z))                    # bore, closed lateral end (wall y IN_END..END)
    a = a.cut(cyl_y(4.0 + HOLE_COMP / 2, Y_IN_END - 1.0, Y_END + 1.0, A_X, A_Z))       # hex-key access hole dia 8 for the cord adjuster
    for (r, ang) in INS_M3:                                                   # M3 heat-set inserts 4.0 OD x 5.7, axial into the flange face y = 73
        x, z = A_X + r * math.sin(math.radians(ang)), A_Z + r * math.cos(math.radians(ang)); a = a.cut(cyl_y(2.0, Y_FL0 - 1.0, Y_FL0 + 5.7, x, z))
    # rev e1 2026-10-15 (gI): 2 x 45 deg chamfer on the outer edge of the closed lateral end. With both arms at the S1 150 hard stop
    # this corner of the R tube faced its L mirror 2.86 mm apart (< 3); the chamfer gains 0.5 mm per mm on each side (gap ~4.9 mm).
    # The critical end-wall-to-tube section (inner corner -> chamfer) stays 4.95 mm >= the 2.5 mm wall, so balancer_struct is unaffected.
    ch = cyl_y(R_TO + 1.0, Y_END - END_CHAMFER, Y_END + 1.0, A_X, A_Z).cut(
        cq.Workplane("XY").add(cq.Solid.makeCone(R_TO, R_TO - END_CHAMFER - 1.0, END_CHAMFER + 1.0, cq.Vector(A_X, Y_END - END_CHAMFER, A_Z), cq.Vector(0, 1, 0))))
    a = a.cut(ch)
    a = a.cut(cq.Workplane("XY").sphere(DOME_KEEP))                           # whole keep-out sphere (the dome apex is at y 46.3 on the S2 axis)
    a = a.cut(cyl_y(SWIVEL_KEEP_R, 40.0, Y_FL0 - 0.01, A_X, A_Z))              # swivel + sheave sweep (and the cap flange y 70..73 sits in here)
    for (x, y) in INS_XY:                                                      # foot screws: ISO 7380 M4 x 10 -> inserts in the yoke disc; clearance 4.4 + counterbore 8.2 x 2.6
        a = a.cut(cyl_z((4.0 + 0.4) / 2, DISC_TOP - 1, DISC_TOP + PAD_T + 1, x, y)); a = a.cut(cyl_z(8.2 / 2, DISC_TOP + PAD_T - 2.6, DISC_TOP + PAD_T + 30, x, y))
    return a

# ============================================================================================== 2. tube cap, piston, buffer (M1)
Y_CAP0 = 70.0                       # cap flange medial face
BRG_OD, BRG_W = 19.0, 5.0          # 6800ZZ 10 x 19 x 5
Y_BRG1, Y_BRG2 = Y_CAP0, TB["hub_y1"] - 2.0 - BRG_W   # bearing No 1 y 70..75 (locating: takes the +y cord pull on a bore shoulder at 75); No 2 y 93..98 (floating), span 23
def tubecap():
    fl = cyl_y(R_FL, Y_CAP0, Y_FL0, A_X, A_Z)                                  # flange 3 thick against the tube face
    sp = cyl_y(R_TI - 0.15, Y_FL0, TB["seat_y"], A_X, A_Z)                     # spigot, slip fit (0.15 radial)
    hub = cyl_y(TB["hub_OD"] / 2, TB["seat_y"] - 0.01, TB["hub_y1"], A_X, A_Z)  # hub inside the spring bore (spring ID %.1f) carries bearing No 2
    c = fl.union(sp).union(hub)
    c = c.cut(cyl_y(BRG_OD / 2 - 0.05, Y_BRG1 - 1.0, Y_BRG1 + BRG_W, A_X, A_Z))                       # bearing seats (press, -0.05 radial; ream)
    c = c.cut(cyl_y(BRG_OD / 2 - 0.05, Y_BRG2, Y_BRG2 + BRG_W, A_X, A_Z))
    c = c.cut(cyl_y(6.0, Y_BRG1 + BRG_W - 0.01, Y_BRG2 + 0.01, A_X, A_Z)).cut(cyl_y(6.5, Y_BRG2 + BRG_W - 0.01, TB["hub_y1"] + 1.0, A_X, A_Z))   # shank relief, e-clip exit
    for (r, ang) in INS_M3:
        x, z = A_X + r * math.sin(math.radians(ang)), A_Z + r * math.cos(math.radians(ang))
        c = c.cut(cyl_y((3.0 + 0.4) / 2, Y_CAP0 - 1.0, Y_FL0 + 1.0, x, z)).cut(cyl_y(3.1, Y_CAP0 - 1.0, Y_CAP0 + 0.5, x, z))   # M3 x 8 SHCS, low head seat
    return c
def piston(y0):
    p = cyl_y(R_TI - 0.3, y0, y0 + TB["piston_t"], A_X, A_Z).union(cyl_y(SP["ID"] / 2 - 0.4, y0 - 3.0, y0, A_X, A_Z))
    return p.cut(cyl_y(2.8, y0 - 4.0, y0 + TB["piston_t"] + 1.0, A_X, A_Z))     # M4 heat-set insert 5.6 OD (cord adjuster screw)
def buffer_ring():
    y1 = Y_IN_END; return ring_y(R_TI - 0.3, 6.0, y1 - TB["buffer_t"], y1, A_X, A_Z)

# ============================================================================================== 3. swivel + sheave (local frame: e = +z, built at S2 = 0; verify/render rotate it about the A axis)
CHEEK_IN, CHEEK_T, CHEEK_R = bc.SHEAVE_W / 2 + 0.6, 2.5, 9.5
Y_BR0, Y_BR1 = Y_C + bc.SHEAVE_R + 0.75, 68.75     # fork bridge (y 66.0 .. 68.75)
def swivel():
    cz = A_Z + R_P
    cheeks = None
    for s in (-1, 1):
        x0 = s * CHEEK_IN; x1 = s * (CHEEK_IN + CHEEK_T)
        ch = cyl_x(CHEEK_R, min(x0, x1), max(x0, x1), Y_C, cz).union(box(min(x0, x1), max(x0, x1), Y_C, Y_BR1, A_Z - 3.0, cz + CHEEK_R - 2.0))
        cheeks = ch if cheeks is None else cheeks.union(ch)
    bridge = box(-(CHEEK_IN + CHEEK_T), CHEEK_IN + CHEEK_T, Y_BR0, Y_BR1, A_Z - 6.0, cz + CHEEK_R - 2.0)
    shank = cyl_y(6.5, Y_BR1 - 0.01, Y_CAP0 - 0.3, A_X, A_Z).union(cyl_y(5.0, Y_CAP0 - 0.31, Y_BRG2 + BRG_W + 1.6, A_X, A_Z))    # shoulder (inner race of No 1) + shank through No 2, e-clip groove at the end
    s = cheeks.union(bridge).union(shank)
    s = s.cut(cyl_y(2.0, Y_C - 0.1, Y_BRG2 + BRG_W + 3.0, A_X, A_Z))             # cord bore dia 4, radiused entries (not modelled)
    s = s.cut(cyl_x(2.0, -10, 10, Y_C, cz))                                       # sheave pin dia 4 h7 (press in one cheek, slip in the other)
    s = s.cut(cyl_x(bc.SHEAVE_R + 0.8, -CHEEK_IN + 0.01, CHEEK_IN - 0.01, Y_C, cz))   # sheave pocket between the cheeks
    return s
def sheave():
    cz = A_Z + R_P; w = bc.SHEAVE_W / 2
    s = cyl_x(bc.SHEAVE_R, -w, w, Y_C, cz).cut(cq.Workplane("YZ").circle(bc.SHEAVE_R + 1).circle(R_P - 0.75).extrude(1.6).translate((-0.8, Y_C, cz)))   # groove root r 6.0
    return s.cut(cyl_x(2.75, -w - 1, w + 1, Y_C, cz))                             # DU bush seat (5.5 OD, press)

# ============================================================================================== 4. collar + lug (M2, arm frame)
SX_SLEEVE, SY_SLEEVE0, SY_SLEEVE1 = 32.55, -20.05, 26.3     # S3 sleeve + lateral web outer box (shoulder_stage.py)
COLLAR_Z0, COLLAR_Z1 = 80.0, 96.5
WALL, SIDE_W = 5.0, 8.0
LEG_X = (-10.0, 12.0)            # leg width (x); B pin at x = B_X
LEG_Y = (29.0, 43.0)             # leg thickness (y): pin shoulder y 43..53 (spacer 43..48.25, cord eye at Y_B = 50, washer), leg face 4.25 mm medial of the fork cheeks (y >= 47.25)
LEG_Z0 = B_Z - 11.0              # leg bottom (35): pin boss below B
PIN = dict(d=6.0, shoulder=10.0, head_d=10.0, head_h=4.5, thread="M5", insert="M5 heat-set 7.0 OD x 9.5", spacer="steel sleeve 6.1/9 x 5.25")
def collar():
    xi = SX_SLEEVE + CLR; xo = xi + SIDE_W
    y_in0, y_in1 = SY_SLEEVE0 - CLR, SY_SLEEVE1 + CLR
    y_out1 = y_in1 + WALL; y_end = y_in0 - 1.0
    sh = box(-xo, xo, y_end, y_out1, COLLAR_Z0, COLLAR_Z1).cut(box(-xi, xi, y_end - 1, y_in1, COLLAR_Z0 - 1, COLLAR_Z1 + 1))
    arm = box(LEG_X[0], LEG_X[1], y_out1 - 1.0, LEG_Y[1], COLLAR_Z0, 95.0)                   # top arm off the lateral wall (top 95: S3 lug sweeps z >= 100)
    leg = box(LEG_X[0], LEG_X[1], LEG_Y[0], LEG_Y[1], LEG_Z0, COLLAR_Z0 + 1.0)
    gus = cq.Workplane("YZ").polyline([(LEG_Y[0], COLLAR_Z0 + 0.5), (y_out1 - 1.0, COLLAR_Z0 + 0.5), (LEG_Y[0], COLLAR_Z0 - 22.0)]).close().extrude(LEG_X[1] - LEG_X[0]).translate((LEG_X[0], 0, 0))
    sh = sh.union(arm).union(leg).union(gus)
    sh = sh.cut(cyl_y(3.5, LEG_Y[1] - 9.5, LEG_Y[1] + 1.0, B_X, B_Z))           # M5 heat-set insert pocket (7.0 OD x 9.5) for the shoulder screw
    sh = sh.cut(cq.Workplane("XY").sphere(47.3).intersect(box(-60, 60, 0, 47.3, -60, 60)))   # 1 mm off the humeral-head dome (same body M2)
    for sx in (-1, 1):
        for z in (COLLAR_Z0 + 4.0, COLLAR_Z1 - 4.0):
            sh = sh.cut(cyl_y(2.8, y_end - 0.0, y_end + 8.0, sx * (xi + SIDE_W / 2), z))
    return sh, y_end
def collar_cap(y_end):
    xo = SX_SLEEVE + CLR + SIDE_W
    c = box(-xo, xo, y_end - 8.0, y_end, COLLAR_Z0, COLLAR_Z1)
    for sx in (-1, 1):
        for z in (COLLAR_Z0 + 4.0, COLLAR_Z1 - 4.0):
            c = c.cut(cyl_y((4.0 + 0.4) / 2, y_end - 9.0, y_end + 1.0, sx * (SX_SLEEVE + CLR + SIDE_W / 2), z))
            c = c.cut(cyl_y(7.4 / 2, y_end - 9.0, y_end - 8.0 + 3.0, sx * (SX_SLEEVE + CLR + SIDE_W / 2), z))
    c = c.cut(box(-(SX_SLEEVE + 0.5), SX_SLEEVE + 0.5, y_end - 1.0, y_end + 0.1, COLLAR_Z0 + 3.0, COLLAR_Z1 - 3.0))
    return c
def pin_proxy():
    return cyl_y(PIN["d"] / 2, LEG_Y[1], LEG_Y[1] + PIN["shoulder"], B_X, B_Z).union(cyl_y(PIN["head_d"] / 2, LEG_Y[1] + PIN["shoulder"], LEG_Y[1] + PIN["shoulder"] + PIN["head_h"], B_X, B_Z))

# ============================================================================================== 5. TPU soft-landing bumpers (S2 + and - stops)
# rev f 2026-10-16 (gI stop-rule fix): rev d/e put the bumper cap at the SOFT limit (+88.5 / -30.5) and called the full-crush angle the hard stop.
# That breaks the standing rule (passive stop = first contact 5 deg beyond soft).  Now:
#   th_c  = first TPU contact (touch <= 0.05 mm; target 0.1 deg early so the 0.5 deg penetration scan reads 91.0) = the locked passive-stop angle: S2- -35.0 (soft -30 - 5);  S2+ +91.0 (the locked rev-c capsule contact, SPEC_ball_stops;
#           the S3 lid meets the yoke floor at +94, so a +93 contact would leave 1 deg of stroke -> the + side keeps its locked 91).
#   th_r  = designed rigid backstop = the angle at which the blades reach the usable crush CR x Lb (bottomed: cap + folded blades + base act as a solid).
#           S2+ 94.0 (coincides with the lid/yoke-floor backstop), S2- -39.0.
#   The carrier face behind each bumper is trimmed flat (stl/balancer/s2_pad_{pos,neg}_bal) so the blades get the length L = Lb + T_CAP + T_BASE the stroke needs.
#   tpu_s2_*_bump_solid_R = the bottomed bumper as a rigid proxy (base side, thickness L - CR x Lb): verify_stops.py uses it for the rigid first contact.
E_TPU = 50.0; ETA = 0.90; M_PLATEAU_TARGET = 17.0
LID_X, LID_Z = 32.55, 68.0
STOP = {"pos": dict(sgn=-1, th_c=90.9, th_r=94.0, carrier="s2_pad_pos_R", old="tpu_s2_pos_R", M_target=22.2),     # + plateau 19 -> 21.8 N m as built: keeps E >= 0.9 J on the 3 deg stroke (min M_allow + stop 25.5 N m)
        "neg": dict(sgn=+1, th_c=-35.0, th_r=-39.0, carrier="s2_pad_neg_R", old="tpu_s2_neg_R", M_target=17.0)}    # - plateau unchanged (W at the - stop <= 0.58 J)
SOFT_S2 = {"pos": 88.0, "neg": -30.0}
Y_PATCH = (-19.0, -5.0)
def edge_pt(sgn, th):
    t = math.radians(th); xl, zl = sgn * LID_X, LID_Z
    return np.array([xl * math.cos(t) - zl * math.sin(t), xl * math.sin(t) + zl * math.cos(t)])
_LID = None
def _lid():
    global _LID
    if _LID is None: _LID = trimesh.load(os.path.join(STL, "s3_lid_R.stl"))
    return _LID
def _rotm(s2):
    c, sn = math.cos(math.radians(-s2)), math.sin(math.radians(-s2)); T = np.eye(4); T[0, 0], T[0, 2], T[2, 0], T[2, 2] = c, sn, -sn, c; return T
def first_touch(target, kind, rng, tol=0.05):
    from trimesh.collision import CollisionManager
    cm = CollisionManager(); cm.add_object("t", target); lid = _lid().copy()
    cm2 = CollisionManager(); cm2.add_object("lid", lid)
    for s2 in rng:
        cm2.set_transform("lid", _rotm(float(s2)))
        if cm.min_distance_other(cm2) <= tol: return float(s2)
    return None
CR = 0.60
T_CAP, T_BASE = 2.0, 2.5
N_BL, PITCH = 3, 6.5
BOND = 0.1                     # bond line between bumper base and trimmed carrier face (mm)
def _frame(u):
    v = np.array([0.0, 1.0, 0.0]); w = np.cross(v, u); return v, w
def _slab(O, u, a0, a1, wid):
    """box in the bumper frame: along u from a0 to a1, across w +-wid/2, y over the patch"""
    v, w = _frame(u); b = trimesh.creation.box(extents=[wid, Y_PATCH[1] - Y_PATCH[0], a1 - a0])
    Tm = np.eye(4); Tm[:3, 0], Tm[:3, 1], Tm[:3, 2] = w, v, u; Tm[:3, 3] = O + u * (a0 + a1) / 2 + v * ((Y_PATCH[0] + Y_PATCH[1]) / 2); b.apply_transform(Tm); return b
def bumper(kind):
    S = STOP[kind]; Pc = edge_pt(S["sgn"], S["th_c"])
    Pm = edge_pt(S["sgn"], (S["th_c"] + S["th_r"]) / 2)
    tdir = np.array([-Pm[1], Pm[0]]) * (1 if S["th_r"] > S["th_c"] else -1); tdir /= np.linalg.norm(tdir)
    u = np.array([tdir[0], 0.0, tdir[1]]); v, w = _frame(u)
    O0 = np.array([Pc[0], 0.0, Pc[1]])
    car = trimesh.load(os.path.join(STL, "stops", S["carrier"] + ".stl"))
    pts = [O0 + w * a + v * y for a in np.linspace(-7, 7, 8) for y in np.linspace(Y_PATCH[0] + 1, Y_PATCH[1] - 1, 6)]
    loc, ridx, _ = car.ray.intersects_location(pts, [u] * len(pts)); hits = {}
    for p, i in zip(loc, ridx): d = float(np.dot(p - pts[i], u)); hits[i] = min(hits.get(i, 1e9), d)
    L_av0 = min(hits.values()) if hits else 20.0
    def make(delta, L, t_b=1.5):
        O = O0 + u * delta; Lb = L - T_CAP - T_BASE; wid = (N_BL - 1) * PITCH + t_b + 4.0
        c = box(-wid / 2, wid / 2, Y_PATCH[0], Y_PATCH[1], 0.0, T_CAP).union(box(-wid / 2, wid / 2, Y_PATCH[0], Y_PATCH[1], L - T_BASE, L))
        for i in range(N_BL):
            cx = (i - (N_BL - 1) / 2) * PITCH; c = c.union(box(cx - t_b / 2, cx + t_b / 2, Y_PATCH[0], Y_PATCH[1], T_CAP - 0.05, L - T_BASE + 0.05))
        tmp = os.path.join(OUT, "_tmp_bump.stl"); cq.exporters.export(c, tmp, tolerance=0.02, angularTolerance=0.05); m = trimesh.load(tmp); os.remove(tmp)
        Tm = np.eye(4); Tm[:3, 0], Tm[:3, 1], Tm[:3, 2], Tm[:3, 3] = w, v, u, O; m.apply_transform(Tm); return m, Lb, wid
    # 1. cap position (delta along u) so the lid's first touch is th_c (bisection; contact only depends on the cap face)
    th_target = S["th_c"]; rng = (np.arange(S["th_c"] - 6.0, S["th_r"] + 3.0, 0.05) if kind == "pos" else np.arange(S["th_c"] + 6.0, S["th_r"] - 3.0, -0.05))
    lo, hi = -4.0, 6.0
    for _ in range(10):
        mid = 0.5 * (lo + hi); m_, _, _ = make(mid, 8.0)
        th = first_touch(m_, kind, rng)
        early = th is not None and ((th < th_target) if kind == "pos" else (th > th_target))
        if early: lo = mid
        else: hi = mid
    delta = 0.5 * (lo + hi); O = O0 + u * delta
    # 2. crush depth needed to reach th_r: deepest lid penetration past the cap plane at th_r (lid vertices near the patch)
    lid = _lid()
    def pen(th):
        V = trimesh.transform_points(lid.vertices, _rotm(th)); q = V - O; sel = (np.abs(q @ w) <= 12.0) & (q[:, 1] >= Y_PATCH[0] - 1) & (q[:, 1] <= Y_PATCH[1] + 1)
        return float(np.max(q[sel] @ u)) if np.any(sel) else 0.0
    s_req = pen(S["th_r"])
    # bisect the crush depth so the bottomed slab is first touched at th_r (the vertex estimate above is a lower bound)
    lo_s, hi_s = s_req, s_req + 3.0
    for _ in range(12):
        mid = 0.5 * (lo_s + hi_s); tb_ = first_touch(_slab(O, u, mid, mid + 6.0, 20.0), kind, rng, tol=0.0)
        late = tb_ is None or ((tb_ > S["th_r"]) if kind == "pos" else (tb_ < S["th_r"]))
        if late: hi_s = mid
        else: lo_s = mid
    s_req = lo_s
    Lb = s_req / CR; L = Lb + T_CAP + T_BASE
    L_av = L_av0 - delta                       # free length to the untrimmed carrier
    trim = max(L + BOND - L_av, 0.0)
    # 3. blade thickness for the plateau moment
    rc = math.hypot(LID_X, LID_Z); F_target = S.get("M_target", M_PLATEAU_TARGET) / (rc * 1e-3); b_w = Y_PATCH[1] - Y_PATCH[0]
    t_b = (F_target / N_BL * Lb ** 2 * 12 / (4 * math.pi ** 2 * E_TPU * b_w)) ** (1 / 3); t_b = math.floor(t_b * 100) / 100
    m, Lb, wid = make(delta, L, t_b)
    th_c = first_touch(m, kind, rng)
    # 4. trimmed carrier: remove everything on the cap side of the plane u = L + BOND
    carT = car
    if trim > 0:
        cut = _slab(O, u, -40.0, L + BOND, 60.0)
        carT = trimesh.boolean.difference([car, cut], engine="manifold"); carT = max(carT.split(only_watertight=False), key=lambda x: abs(x.volume))
    depth_left = float(np.max((carT.vertices - O) @ u) - (L + BOND))
    # 5. bottomed bumper (rigid proxy): from the crushed cap plane (CR x Lb) to the base face
    s_bot = CR * Lb; solid = _slab(O, u, s_bot, L, wid)
    th_bot = first_touch(solid, kind, rng, tol=0.0)
    s_use = s_bot                               # stroke contact -> bottoming
    I = b_w * t_b ** 3 / 12; Fp = N_BL * 4 * math.pi ** 2 * E_TPU * I / Lb ** 2
    E_b = ETA * Fp * s_use * 1e-3
    # bearing on the trimmed carrier face at the plateau (and a 3 x plateau bottoming spike)
    A_face = wid * b_w
    info = dict(kind=kind, soft_limit_deg=SOFT_S2[kind], contact_S2_deg=th_c, contact_target_deg=S["th_c"], rigid_target_deg=S["th_r"], bottomed_rigid_S2_deg=th_bot,
                cap_shift_along_strike_mm=round(delta, 2), crush_to_rigid_target_mm=round(s_req, 2),
                usable_crush_mm=round(s_use, 2), L_total_mm=round(L, 2), blade_free_len_mm=round(Lb, 2), blade_t_mm=t_b, n_blades=N_BL, blade_width_y_mm=b_w, footprint_w_mm=round(wid, 2),
                bottomed_thickness_mm=round(L - s_bot, 2), F_plateau_N=round(Fp, 0), M_plateau_Nm=round(Fp * rc * 1e-3, 1), energy_J=round(E_b, 2), strike_dir=[round(float(x), 3) for x in u],
                origin=[round(float(x), 2) for x in O], vol_cm3=round(m.volume / 1e3, 3), mass_g_TPU=round(m.volume * 1.20e-3, 1),
                carrier=dict(trim_mm=round(trim, 2), depth_left_behind_bumper_mm=round(depth_left, 2), vol_cm3_before=round(car.volume / 1e3, 3), vol_cm3_after=round(carT.volume / 1e3, 3),
                             watertight=bool(carT.is_watertight), face_bearing_MPa_plateau=round(Fp / A_face, 2), face_bearing_MPa_3x=round(3 * Fp / A_face, 2)))
    return m, info, carT, solid
# ============================================================================================== exports
def mirror_export(m, name):
    m.export(os.path.join(OUT, name + "_R.stl")); ml = m.copy(); ml.apply_transform(np.diag([1, -1, 1, 1.0]))
    if ml.volume < 0: ml.invert()
    ml.export(os.path.join(OUT, name + "_L.stl"))
def export_cq(w, name, mirror_left=True):
    p = os.path.join(OUT, name + "_R.stl")
    cq.exporters.export(w, p, tolerance=0.03, angularTolerance=0.05)
    m = trimesh.load(p)
    if mirror_left: mirror_export(m, name)
    return m
DIST = os.path.join(HERE, "..", "stl", "distal")
def postcut(name, pad=3.2):
    """notch the collar / cap around the S3 stop posts and their TPU pads (stops_M2, arm frame) with `pad` mm clearance (bounding boxes)"""
    p = os.path.join(OUT, name + "_R.stl"); m = trimesh.load(p); v0 = m.volume; cuts = []
    for f in ("s3_post_neg", "s3_post_pos", "tpu_s3_neg", "tpu_s3_pos"):
        bb = trimesh.load(os.path.join(DIST, "stops_M2", f + ".stl")).bounds; cuts.append(trimesh.creation.box(bounds=[bb[0] - pad, bb[1] + pad]))
    r = trimesh.boolean.difference([m] + cuts, engine="manifold"); bodies = r.split(only_watertight=False)
    mirror_export(r, name)
    return dict(removed_cm3=round((v0 - r.volume) / 1e3, 3), bodies=len(bodies), watertight=bool(r.is_watertight), vol_cm3=round(r.volume / 1e3, 2), bbox_mm=[round(float(x), 1) for x in (r.bounds[1] - r.bounds[0])])

HARDWARE = [
    ("compression spring, music wire ASTM A228, d %.1f, mean D %.2f (OD %.2f), %.2f active / %.2f total coils, squared + ground, free length %.1f, solid %.1f, k %.3f N/mm, "
     "shot-peened, set removed (custom wound -- PLACEHOLDER vendor/part number; maker trims N_a to hit k on test)"
     % (SP["wire_d"], SP["D_mean"], SP["OD"], SP["N_active"], SP["N_total"], SP["L_free"], SP["L_solid"], SP["k_N_per_mm"]), 1),
    ("cord: %s, ~260 mm, spliced eye at B (over the d6 pin), lateral end on the adjuster sleeve (PLACEHOLDER MBL %.0f N)" % (bc.CORD["type"], bc.CORD["MBL_N"]), 1),
    ("6061-T6 sheave OD 17, groove root dia 12, width 5, bore 5.5 (turned, PLACEHOLDER shop part); GGB 0404DU bush 4 x 5.5 x 4; ISO 8734 pin 4 x 14 (hardened)", 1),
    ("6800ZZ deep-groove bearing 10 x 19 x 5 (swivel shank: No 1 in the cap flange, No 2 in the cap hub, span 23 mm)", 2),
    ("DIN 6799 retaining washer RS 8 (e-clip) on the swivel shank groove d 8", 1),
    ("ISO 4762 M3 x 8 SHCS (tube cap -> tube flange)", 3), ("M3 heat-set insert 4.0 OD x 5.7 (tube flange)", 3),
    ("ISO 4762 M4 x 30 SHCS cord adjuster + 2 x M4 nut + d4/6 x 6 sleeve (cord eye), Loctite 222", 1), ("M4 heat-set insert 5.6 OD x 4 (piston)", 1),
    ("ISO 7379 shoulder screw d6 x 10, M5 (cord pin B) + spacer sleeve 6.1/9 x 5.25 + M6 washer (cord eye centred on y 50)", 1), ("M5 heat-set insert 7.0 OD x 9.5 (collar leg)", 1),
    ("ISO 7380 button-head screw M4 x 10 (anchor foot -> yoke disc)", 6), ("M4 heat-set insert 5.6 OD x 4 (yoke disc, depth 4.2)", 6),
    ("ISO 4762 M4 x 16 SHCS (collar cap bolts)", 4), ("M4 heat-set insert 5.6 OD x 8 (collar side-wall end faces)", 4),
    ("TPU 95A friction shim 1.0 x 30 x 20 (collar clamp)", 2),
]
def main():
    meta = dict(rev="f", spring=SP, tube=TB)
    a = anchor(); ma = export_cq(a, "bal_anchor")
    tc = tubecap(); mtc = export_cq(tc, "bal_tubecap")
    sw = swivel(); msw = export_cq(sw, "bal_swivel")
    sv = sheave(); msv = export_cq(sv, "bal_sheave")
    pis = piston(TB["piston_y_S2_0"]); mpi = export_cq(pis, "bal_piston")
    bu = buffer_ring(); mbu = export_cq(bu, "tpu_bal_buffer")
    c, y_end = collar(); export_cq(c, "bal_collar")
    cp = collar_cap(y_end); export_cq(cp, "bal_collar_cap")
    export_cq(pin_proxy(), "bal_pin_proxy")
    meta["postcut"] = {"bal_collar": postcut("bal_collar"), "bal_collar_cap": postcut("bal_collar_cap")}
    # spring proxy at S2 = 0 (inside the bore): tube OD/ID of the coil between seat and piston
    spr = ring_y(SP["OD"] / 2, SP["ID"] / 2, TB["seat_y"], TB["piston_y_S2_0"], A_X, A_Z); export_cq(spr, "spring_proxy")
    dens = dict(bal_anchor=1.20, bal_tubecap=1.20, bal_piston=1.20, bal_collar=1.20, bal_collar_cap=1.20, tpu_bal_buffer=1.21, bal_swivel=2.81, bal_sheave=2.70)
    mat = dict(bal_anchor="PA12-CF", bal_tubecap="PA12-CF", bal_piston="PA12-CF", bal_collar="PA12-CF", bal_collar_cap="PA12-CF", tpu_bal_buffer="TPU 85A", bal_swivel="7075-T6 (machined)", bal_sheave="6061-T6 (turned) + DU bush")
    for nm in dens:
        m = trimesh.load(os.path.join(OUT, nm + "_R.stl")); bb = m.bounds
        meta[nm] = dict(material=mat[nm], density=dens[nm], vol_cm3=round(m.volume / 1e3, 2), mass_g=round(m.volume * dens[nm] * 1e-3, 1), bbox_mm=[round(float(v), 1) for v in (bb[1] - bb[0])],
                        watertight=bool(m.is_watertight), fits_256=bool(np.all(bb[1] - bb[0] <= 256)), n_bodies=len(m.split(only_watertight=False)))
    yoke = trimesh.load(os.path.join(STL, "s1s2_yoke_R.stl")); cuts = []
    for (x, y) in INS_XY:
        cyl = trimesh.creation.cylinder(radius=2.8, height=4.2 + 1.0, sections=48); cyl.apply_translation([x, y, DISC_TOP - 4.2 + (4.2 + 1.0) / 2]); cuts.append(cyl)
    yb = trimesh.boolean.difference([yoke] + cuts, engine="manifold"); mirror_export(yb, "s1s2_yoke_bal")
    meta["s1s2_yoke_bal"] = dict(vol_cm3=round(yb.volume / 1e3, 2), delta_vs_original_cm3=round((yb.volume - yoke.volume) / 1e3, 3), watertight=bool(yb.is_watertight), note="rev d interface, unchanged")
    meta["bumpers"] = {}
    for kind in ("pos", "neg"):
        bm_, info, carT, solid = bumper(kind); mirror_export(bm_, "tpu_s2_%s_bump" % kind); info["watertight"] = bool(bm_.is_watertight); meta["bumpers"][kind] = info
        mirror_export(carT, "s2_pad_%s_bal" % kind); mirror_export(solid, "tpu_s2_%s_bump_solid" % kind)
    meta["hardware"] = [dict(item=i, qty=q) for i, q in HARDWARE]
    meta["points"] = dict(A_M1=[A_X, Y_B, A_Z], B_M2=[B_X, Y_B, B_Z], sheave_centre_y=Y_C, swivel_axis="y-parallel through (A_X, A_Z)", cord_plane_y=Y_B)
    meta["retired_rev_d"] = ["bal_cap_R/L (renamed bal_collar_cap)", "spring_proxy_1/2", "rev d boss block and hook bosses"]
    json.dump(meta, open(os.path.join(OUT, "balancer_parts.json"), "w"), indent=1, default=float); print(json.dumps({k: v for k, v in meta.items() if k not in ("spring", "hardware")}, indent=1, default=float))

if __name__ == "__main__":
    main()
