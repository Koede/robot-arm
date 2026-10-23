"""Claw hand, C3 (three-fold) punch-ring claw. Session 2026-10-13g. Replaces the planar 2+1 claw (rev 13d/e). Right arm, robot frame, mm.
CONCEPT: THREE IDENTICAL curl fingers 120 deg apart about the hand axis (z at x 0, y D). Each finger curls in its own RADIAL plane (double-jointed: proximal + distal
phalanx, distal coupled to the proximal by a ground-linked four-bar). ONE Feetech STS3215 (G, unchanged actuator count / DOF) stands with its shaft ON the hand axis (vertical,
shaft toward +z) and turns a helical-groove ROTOR (3-start). A PUNCH RING (nut) rides the rotor on three steel pins, is guided by two printed pillars, and PUNCHES up the axis;
three drive links (ring lug -> proximal phalanx) curl all three fingers together so the tips converge and clasp on the axis (pie-wedge close).
Finger frame (a inward, b up) as claw_synth.py; canonical finger = azimuth 0 (radial +x), tangent +y; instantiated at azimuth 90 + 120 k about (x 0, y D).
t = tangential layer coordinate (y - D): plate [-3, 3]; drive link +[3.5, 6.5]; coupling link -[-6.5, -3.5]; clevis ears +[3.5, 6.5] / -[-9.5, -3.5]; Q post -[-9.5, -6.5].
Hand frame = W3 frame of distal_arm.py; W3 arms / HB0 / D are the interface contract and are unchanged.  PLACEHOLDERS: SHAFT_OFF, horn disc (HORN_D/HORN_P),
servo mounting-ear pattern (EAR_*: +y end screwed through 2 ears, -y end clamped by 2 tabs; rev 2026-10-15), rotor-to-horn fixing (no horn screw holes modelled), TPU pad mu, CLAW_ZERO/CLAW_SIGN (firmware).
Outputs: stl/distal/claw/*.stl (+ proxies/), claw.json (layout).  Left hand = mirror in y about the spine plane.
PIN RETENTION (rev 2026-10-15, Forest-approved): every steel dowel is PRESSED in exactly one member (d2.4 hole for d2.5 pins, d2.9 for d3 pins, reamed to the
coupon) and RUNS in the other (d2.6 for d2.5 pins, d3.1/d3.2 for d3 pins). Press side = the thicker member of the joint; the running member is captured
axially between the press member and the next layer, so no pin can walk out.
  pivot pin  d3   : PRESS clevis bracket (hand_tray, d2.9)   | RUN prox_F (d3.1)
  Q pin      d3   : PRESS Q post (hand_tray, d2.9)           | RUN link_couple_F (d3.2)
  J pin      d3   : PRESS dist_F (d2.9)                      | RUN prox_F tongue (d3.1)
  P pin      d2.5 : PRESS prox_F (6.0 thick, d2.4)           | RUN link_drive_F (2.7 thick, d2.6)
  R pin      d2.5 : PRESS dist_F R-tab (3.0 thick, d2.4)     | RUN link_couple_F (2.7 thick, d2.6)
  lug pin    d2.5 : PRESS ring lug (3.5 thick, d2.4)         | RUN link_drive_F (2.7 thick, d2.6)
  ring pins  d2.5 : PRESS claw_ring radial holes (d2.4)      | RUN (ball end) in the rotor helical grooves"""
import cadquery as cq, numpy as np, math, os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scipy.spatial import ConvexHull
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "distal", "claw"); os.makedirs(OUT, exist_ok=True)
S = json.load(open(os.path.join(HERE, "claw_synth.json")))
import claw_synth as CS
# ---------------------------------------------------------------- layout shared with distal_arm.py (interface contract, unchanged)
D = 57.0; Z_W3 = 455.0; Z_W = 410.0; Z_TIP = 575.0
ST_L, ST_W, ST_H = 45.2, 24.7, 35.0; SHAFT_OFF = 11.0; HORN_P = 4.0; HORN_D = 20.0          # SHAFT_OFF, HORN_*: PLACEHOLDER (unverified)
CLR = 0.3; WALL = 2.4
HA_IN, HA_T = ST_H / 2 + HORN_P + 1.8, 3.5
HB0 = Z_W3 + SHAFT_OFF + ST_L / 2 + 3.0; FLOOR_TOP = HB0 + 2.0                               # 491.6 / 493.6
Z_BOT = FLOOR_TOP + CLR; Z_TOPB = Z_BOT + ST_H                                              # servo body 493.9 .. 528.9 (shaft vertical)
Z_HORN1 = Z_TOPB + HORN_P                                                                   # 532.9
# servo footprint: shaft on the hand axis (0, D); long axis along y, shaft SHAFT_OFF from the body centre toward +y  -> body y D-33.6 .. D+11.6
SV_YC = D - SHAFT_OFF; SV_Y0, SV_Y1 = SV_YC - ST_L / 2, SV_YC + ST_L / 2; SV_X = ST_W / 2   # centre D-11; y D-33.6 .. D+11.6
# ---------------------------------------------------------------- finger / linkage (claw_synth.json)
RB, ZB, L1, L2 = S["RB"], S["ZB"], S["L1"], S["L2"]; DP, NP, LD, RL = S["DP"], S["NP"], S["LD"], S["RL"]
QA, QB, LC, RY, RN = S["qa"], S["qb"], S["lc"], S["ry"], S["rn"]
TH1_O, TH1_C = S["TH1_O"], S["TH1_C"]; HARD = 5.0; TH1_HO, TH1_HC = TH1_O - HARD, TH1_C + HARD
PT, LT, GAP = 6.0, 3.0, 0.5                                                                 # plate, link thickness, layer gap
AZ = [90.0, 210.0, 330.0]                                                                   # finger azimuths (deg from +x toward +y about (0, D))
RING_H, RING_RI, RING_RO = 6.0, 8.0, 13.0                                                   # punch ring
ZN = lambda th: CS.nut_z(th)[0]
ZN_HO, ZN_O, ZN_C, ZN_HC = ZN(TH1_HO), ZN(TH1_O), ZN(TH1_C), ZN(TH1_HC)
ROT_R, ROT_RC = 7.6, 5.6                                                                    # rotor outer r, groove-bottom r
PHASE_FIX = 3.0                                                                             # deg: groove phase vs the pin azimuth. rev 2026-10-15: 4.0 -> 3.0 with the +0.50 cutter (phase study m4: 0.342 / 0.372 mm per flank)
PIN_D, PIN_R = 2.5, 6.9                                                                     # ring pins d2.5 (steel), centre radius in the groove (tip at 5.65)
PSI_SOFT = 90.0                                                                             # servo travel soft (deg); sets the helix lead
K_HELIX = (ZN_C - ZN_O) / math.radians(PSI_SOFT)                                            # mm of ring travel per rad of servo rotation
LEAD = 2 * math.pi * K_HELIX; ALPHA = math.degrees(math.atan(K_HELIX / PIN_R))
ROT_BASE0, ROT_BASE1 = Z_HORN1 + 0.1, ZN_HO - RING_H / 2 - 0.6                              # rotor base disc (on the horn)
ROT_TOP = ZN_HC + PIN_D / 2 + 0.6
EAR_AZ = [30.0, 150.0]                                                                      # ring ears / guide pillars (azimuth about the hand axis)
CAP_T, CAP_GAP = 2.5, 0.05                                                                   # closed-side hard-stop cap on the guide pillars: its underside is the rigid hard stop
PIL_R0, PIL_R1, PIL_T, PIL_TOP = 17.0, 25.0, 12.0, ZN_HC + RING_H / 2 - CAP_GAP         # ear top (ring z + 3) meets the cap underside at theta1 = TH1_HC (hard closed, soft + 5.0 deg)

def box(x0, x1, y0, y1, z0, z1):
    x0, x1 = min(x0, x1), max(x0, x1); y0, y1 = min(y0, y1), max(y0, y1); z0, z1 = min(z0, z1), max(z0, z1)
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cyl_y(r, y0, y1, cx=0.0, cz=0.0): return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((cx, y0, cz))
def cyl_x(r, x0, x1, cy=0.0, cz=0.0): return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, cy, cz))
def cyl_z(r, z0, z1, cx=0.0, cy=0.0): return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((cx, cy, z0))
def prism_y(pts, y0, y1):
    w = cq.Workplane("XZ").polyline([(float(p[0]), float(p[1])) for p in pts]).close().extrude(-(y1 - y0)); return w.translate((0, y0, 0))
def hull2d(circles, n=36):
    P = []
    for (cx, cz, r) in circles:
        for t in np.linspace(0, 2 * np.pi, n, endpoint=False): P.append((cx + r * np.cos(t), cz + r * np.sin(t)))
    P = np.array(P); h = ConvexHull(P); return P[h.vertices]
def Y(t): return D + t
def fm(a, b): return (RB - a, ZB + b)                                                       # finger frame (a inward, b up) -> canonical (x, z)
def at_az(w, az): return w.rotate((0, D, 0), (0, D, 1), az)                                 # about the hand axis

parts, prox = {}, {}
# ================================================================== FINGER (canonical, azimuth 0) - identical x3
Bx, Bz = fm(0, 0); Jx, Jz = fm(0, L1); Px, Pz = fm(-NP, DP); Rx, Rz = fm(RN, L1 + RY)      # pivot, J, drive-link point, coupling-link tab (reference pose th1=th2=0)
Qx, Qz = fm(QA, QB)
def phal1():
    poly = [fm(a, b) for a, b in hull2d([(0, 0, 7.5), (-NP, DP, 4.6), (0, L1, 5.5)], 40)]
    p = prism_y(poly, Y(-PT / 2), Y(PT / 2))
    # tongue: J end keeps the +t half (0..3); near half (-3..0) cut around J, and the sweep of the distal R-tab (near half), +0.4 mm
    p = p.cut(cyl_y(9.0, Y(-PT / 2) - 0.01, Y(0.2), Jx, Jz).intersect(box(-200, 200, Y(-PT / 2) - 1, Y(0.2), Jz - 9.0, Jz + 9.0)))
    for t2 in range(-25, 85, 5):
        c_, s_ = math.cos(math.radians(t2)), math.sin(math.radians(t2))
        pl = [(a * c_ + (b - L1) * s_, -a * s_ + (b - L1) * c_ + L1) for a, b in hull2d([(0, L1, 5.4), (RN, L1 + RY, 4.4)], 30)]
        p = p.cut(prism_y([fm(a, b) for a, b in pl], Y(-PT / 2) - 0.01, Y(0.2)))
    # pivot d3.1 RUN on the bracket-pressed pivot pin; P d2.4 PRESS (P pin d2.5 runs in link_drive d2.6); J d3.1 RUN (J pin pressed in dist_F)
    p = p.cut(cyl_y(1.55, Y(-PT / 2) - 1, Y(PT / 2) + 1, Bx, Bz)).cut(cyl_y(1.2, Y(-PT / 2) - 1, Y(PT / 2) + 1, Px, Pz)).cut(cyl_y(1.55, Y(-PT / 2) - 1, Y(PT / 2) + 1, Jx, Jz))
    return p
def phal2():
    tip = L1 + L2
    body = hull2d([(0, L1, 5.0), (0, tip - 3.0, 3.0)], 36); tab = hull2d([(0, L1, 5.0), (RN, L1 + RY, 4.0)], 30)
    p = prism_y([fm(a, b) for a, b in body], Y(-PT / 2), Y(PT / 2)).union(prism_y([fm(a, b) for a, b in tab], Y(-PT / 2), Y(0)))
    p = p.cut(cyl_y(9.0, Y(-0.2), Y(PT / 2) + 0.01, Jx, Jz).intersect(box(-200, 200, Y(-0.2), Y(PT / 2) + 1, Jz - 9.0, Jz + 9.0)))
    # J d2.9 PRESS (J pin runs in prox_F d3.1); R d2.4 PRESS in the 3.0 mm R-tab (R pin d2.5 runs in link_couple d2.6). rev 2026-10-15: R was d2.5 line-to-line
    p = p.cut(cyl_y(1.45, Y(-PT / 2) - 1, Y(PT / 2) + 1, Jx, Jz)).cut(cyl_y(1.2, Y(-PT / 2) - 1, Y(PT / 2) + 1, Rx, Rz))
    return p
def bar_link(length, y0, y1, hole_r=1.3, eye=4.2, hole_r0=None):
    """straight link, pin centres (0,0) and (length,0) in the (x,z) plane (canonical placement; posed by claw_asm). Links are always the RUNNING member:
    hole_r 1.3 = d2.6 running on d2.5 steel (P, lug, R pins); hole_r0 1.6 = d3.2 running on the d3 Q pin."""
    poly = [(u, v) for u, v in hull2d([(0, 0, eye), (length, 0, eye)], 40)]
    p = prism_y(poly, Y(y0), Y(y1))
    hole_r0 = hole_r if hole_r0 is None else hole_r0            # hole at the (0,0) end; rev 2026-10-14m: the coupling link turns on the d3 Q pin -> r 1.6 (was 1.3, interfered)
    return p.cut(cyl_y(hole_r0, Y(y0) - 1, Y(y1) + 1, 0, 0)).cut(cyl_y(hole_r, Y(y0) - 1, Y(y1) + 1, length, 0))
parts["prox_F"] = phal1(); parts["dist_F"] = phal2()
parts["link_drive_F"] = bar_link(LD, 3.5, 6.2, 1.3); parts["link_couple_F"] = bar_link(LC, -6.2, -3.5, 1.3, hole_r0=1.6)
tip = L1 + L2
pad = [fm(a, b) for a, b in [(5.0, L1 + 5.0), (7.0, L1 + 5.0), (5.4, tip - 0.4), (3.0, tip - 0.4)]]
parts["tpu_pad_F"] = prism_y(pad, Y(-PT / 2 + 0.6), Y(PT / 2 - 0.6))

# ================================================================== PALM TRAY: W3 arms, floor, servo pocket, 3 finger clevis brackets, 2 guide pillars
hand = None
for sx in (-1, 1):
    x0 = sx * HA_IN; x1 = sx * (HA_IN + HA_T)
    arm = cyl_x(16.0, min(x0, x1), max(x0, x1), D, Z_W3).union(box(min(x0, x1), max(x0, x1), D - 14.0, D + 14.0, Z_W3, Z_W3 + SHAFT_OFF + ST_L / 2 + 6.0))
    hand = arm if hand is None else hand.union(arm)
XF = HA_IN + HA_T
hand = hand.union(box(-XF, XF, SV_Y0 - WALL - CLR - 2.0, D + 20.0, HB0, FLOOR_TOP))                            # floor plate
for sx in (-1, 1):                                                                                                # servo pocket walls (to the body top)
    hand = hand.union(box(sx * (SV_X + CLR), sx * (SV_X + CLR + WALL), SV_Y0 - CLR - WALL, SV_Y1 + CLR + WALL, FLOOR_TOP, Z_TOPB))
hand = hand.union(box(-(SV_X + CLR + WALL), SV_X + CLR + WALL, SV_Y0 - CLR - WALL, SV_Y0 - CLR, FLOOR_TOP, Z_TOPB))     # -y end wall
hand = hand.union(box(-(SV_X + CLR + WALL), SV_X + CLR + WALL, SV_Y1 + CLR, SV_Y1 + CLR + WALL, FLOOR_TOP, Z_TOPB))      # +y end wall
# finger clevis brackets (canonical, then rotated)
def bracket():
    b = box(RB - 12.0, RB + 12.0, Y(-9.5), Y(6.5), HB0, ZB - 9.0)                                                # base block under the pivot
    ep = prism_y(hull2d([(Bx, Bz, 5.0), (Bx, Bz - 12.0, 5.0)], 30), Y(3.5), Y(6.5)).intersect(box(-500, 500, -500, 500, 0, Bz + 0.5))                                # + ear
    em = prism_y(hull2d([(Bx, Bz, 6.0), (Bx, Bz - 12.0, 6.0)], 30), Y(-9.5), Y(-3.5))                              # - ear (pivot zone)
    qp = prism_y(hull2d([(Bx, Bz, 6.0), (Qx, Qz, 4.8), (Bx, Bz - 12.0, 6.0)], 30), Y(-9.5), Y(-6.5))              # Q post: spans pivot to Q
    br = b.union(ep).union(em).union(qp)
    br = br.cut(cyl_y(1.45, Y(-9.5) - 1, Y(6.5) + 1, Bx, Bz)).cut(cyl_y(1.45, Y(-9.5) - 1, Y(-6.5) + 1, Qx, Qz))        # d2.9 press holes (pivot pin d3, Q pin d3)
    return br
br0 = bracket()
for az in AZ: hand = hand.union(at_az(br0, az - 0.0))
# guide pillars (open-topped slot for the ring ears)
def pillar():
    pl = box(PIL_R0, PIL_R1, D - PIL_T / 2, D + PIL_T / 2, FLOOR_TOP - 0.5, PIL_TOP)
    pl = pl.cut(box(PIL_R0 - 1.0, PIL_R1 - 3.5, D - 2.9, D + 2.9, FLOOR_TOP + 4.0, PIL_TOP + 1.0))
    for ty in (-4.45, 4.45): pl = pl.cut(cyl_z(0.85, PIL_TOP - 5.0, PIL_TOP + 0.1, 0.5 * (PIL_R0 + PIL_R1 - 3.5), D + ty))      # M2 self-tapping pilot d1.7 x 5 for the cap screws
    return pl
def stop_cap():
    c = box(PIL_R0, PIL_R1, D - PIL_T / 2, D + PIL_T / 2, PIL_TOP, PIL_TOP + CAP_T)
    for ty in (-4.45, 4.45): c = c.cut(cyl_z(1.1, PIL_TOP - 1, PIL_TOP + CAP_T + 1, 0.5 * (PIL_R0 + PIL_R1 - 3.5), D + ty))    # M2 clearance d2.2
    return c
parts["stop_cap"] = stop_cap()                                                                       # canonical (azimuth 0); claw_asm places it at EAR_AZ (x2), M2x6 self-tapping x2 each
CAP_SCR = [(0.5 * (PIL_R0 + PIL_R1 - 3.5), ty) for ty in (-4.45, 4.45)]                              # canonical (radial, t) of the 2 cap screws per cap
pil0 = pillar()
for az in EAR_AZ: hand = hand.union(at_az(pil0, az))
_cs = []
for az in EAR_AZ:
    for (rr, tt) in CAP_SCR:
        a_ = math.radians(az); _cs.append((rr * math.cos(a_) - tt * math.sin(a_), D + rr * math.sin(a_) + tt * math.cos(a_)))
# insert pockets (M3 heat-set 4.0 OD x 5) for the servo-tab screws on the wall tops (screws along z)
# rev 2026-10-15: only the two -y tabs remain. The +y pair (x +-16, y 64.1) sat inside the az-30/150 guide pillars (16.9 mm3 tab / 11.3 mm3 screw-head overlap);
# the +y end is now held by screws through the STS3215's own mounting ears (EAR_*, below).
TAB_PTS = [(sx * 16.0, SV_Y0 + 4.5) for sx in (-1, 1)]
for (tx, ty) in TAB_PTS:
    hand = hand.union(box((1 if tx > 0 else -1) * 12.65, (1 if tx > 0 else -1) * 19.5, ty - 4.0, ty + 4.0, FLOOR_TOP, Z_TOPB))
    hand = hand.cut(cyl_z(2.2, Z_TOPB - 6.0, Z_TOPB + 0.1, tx, ty))
# +y end: STS3215 mounting ears. PLACEHOLDER pattern (unverified vendor geometry, measure the servo before printing the bosses' insert pockets):
# two ear lugs on the +y end face, top EAR_DZ below the body top, EAR_T thick, holes at (+-EAR_X, EAR_Y). EAR_DZ 3: with ears flush to the body top the
# ear corner / screw head come within 1.8 / 2.0 mm of the az-90 drive link at hard-open (2.9 / 3.0 at soft open); at 3 mm down they clear by >= 4.4. The ears rest on two bosses on the tray; M2 x 6 ISO 7380
# button-head screws go down through the ears into M2 heat-set inserts (3.2 OD x 4) in the bosses. Bosses and ears sit at |x| <= 11.3, clear of the pillars (x >= 11.7).
EAR_X, EAR_Y, EAR_T, EAR_HW, EAR_DZ = 8.5, SV_Y1 + 3.5, 2.5, 2.5, 3.0                      # PLACEHOLDER ear pattern: hole x, hole y, thickness, lug half-width, top below body top
EAR_L = EAR_Y + 3.0 - SV_Y1                                                               # ear projection past the +y body face (6.5)
Z_EAR1 = Z_TOPB - EAR_DZ; Z_EAR0 = Z_EAR1 - EAR_T                                         # ear top / ear underside = boss seat
hand = hand.cut(box(-(EAR_X + EAR_HW + CLR), EAR_X + EAR_HW + CLR, SV_Y1 + CLR - 0.01, SV_Y1 + CLR + WALL + 0.01, Z_EAR0, Z_TOPB + 0.1))   # +y end wall notched to the ear seat
for sx in (-1, 1):
    hand = hand.union(box(sx * 5.5, sx * 13.0, SV_Y1 + CLR, EAR_Y + 3.5, FLOOR_TOP, Z_EAR0))                       # ear boss (merges into the pillar foot at x > 11.7)
    hand = hand.cut(cyl_z(1.6, Z_EAR0 - 4.8, Z_EAR0 + 0.1, sx * EAR_X, EAR_Y))                                      # M2 heat-set insert pocket d3.2 x 4.8
# servo pocket kept clear (rev 2026-10-15): the az-330 clevis-bracket base block reached 1.1 mm into the STS3215 envelope (x 11.3..12.35, y 38.9..41.4,
# 22.2 mm3; found by verify_claw's new fixed-vs-fixed check). Cut the servo envelope + CLR through everything in the pocket.
hand = hand.cut(box(-(SV_X + CLR), SV_X + CLR, SV_Y0 - CLR, SV_Y1 + CLR, FLOOR_TOP, Z_TOPB))
# servo cable slot in the -y end wall (PLACEHOLDER position)
hand = hand.cut(box(-5.0, 5.0, SV_Y0 - CLR - WALL - 1, SV_Y0 - CLR + 0.5, FLOOR_TOP + 1.0, FLOOR_TOP + 9.0))
# relief for the W3 servo body (sts_w3 proxy x +-17.5, y 44.7..69.3, z 443.4..488.6), swept through W3 -35..+40 about the W3 axis, +3.2 mm
for _w3 in range(-35, 45, 5):
    _c, _s = math.cos(math.radians(_w3)), math.sin(math.radians(_w3))
    _pts = [(y, z) for y in (44.7 - 3.2, 69.3 + 3.2) for z in (443.4 - 3.2, 488.6 + 3.2)]
    _rp = [(D + (y - D) * _c + (z - Z_W3) * _s, Z_W3 - (y - D) * _s + (z - Z_W3) * _c) for y, z in _pts]
    _hull = ConvexHull(np.array(_rp)); _poly = [tuple(_rp[i]) for i in _hull.vertices]
    hand = hand.cut(cq.Workplane("YZ").polyline(_poly).close().extrude(17.5 + 3.2, both=True))
parts["hand_tray"] = hand
prox["sts_g"] = box(-SV_X, SV_X, SV_Y0, SV_Y1, Z_BOT, Z_TOPB)                                                    # G servo envelope (shaft vertical on the hand axis, PLACEHOLDER offset)
for sx in (-1, 1):                                                                                               # + its two +y mounting ears (PLACEHOLDER pattern), resting on the bosses
    prox["sts_g"] = prox["sts_g"].union(box(sx * (EAR_X - EAR_HW), sx * (EAR_X + EAR_HW), SV_Y1 - 0.5, SV_Y1 + EAR_L, Z_EAR0, Z_EAR1)
                                        .cut(cyl_z(1.1, Z_EAR0 - 1, Z_TOPB + 1, sx * EAR_X, EAR_Y)))
# fastener proxies (hand frame; steel; used by verify_claw's fixed-vs-fixed check and the moving-part matrix). Shanks of thread-forming screws are modelled at the
# core diameter so they sit inside their pilots; machine screws into heat-set inserts are modelled at nominal d inside the insert pocket.
def screw(x, y, z_head, d_head, h_head, d_shank, L_):
    return cyl_z(d_head / 2, z_head, z_head + h_head, x, y).union(cyl_z(d_shank / 2, z_head - L_, z_head + 0.01, x, y))
for i, (tx, ty) in enumerate(TAB_PTS): prox[f"scr_tab{i}"] = screw(tx, ty, Z_TOPB + 2.2, 5.5, 3.0, 3.0, 6.0)          # M3 x 6 SHCS through the tab into the M3 insert
for i, (cx_, cy_) in enumerate(_cs): prox[f"scr_cap{i}"] = screw(cx_, cy_, PIL_TOP + CAP_T, 3.8, 2.0, 1.6, 6.0)    # M2 x 6 thread-forming (core d1.6 in the d1.7 pilot)
for i, sx in enumerate((-1, 1)): prox[f"scr_ear{i}"] = screw(sx * EAR_X, EAR_Y, Z_EAR1, 3.5, 1.1, 2.0, 6.0)          # M2 x 6 ISO 7380 through the ear into the M2 insert
prox["horn_disc"] = cyl_z(HORN_D / 2, Z_TOPB, Z_HORN1, 0, D)                                                     # PLACEHOLDER horn disc
tab = box(-6.0, 3.2, -4.0, 4.0, Z_TOPB, Z_TOPB + 2.2).cut(cyl_z(1.7, Z_TOPB - 1, Z_TOPB + 3, 0, 0))              # servo tab (x2, identical, rev 2026-10-15): clamps the -y body top edge
parts["servo_tab"] = tab                                                                                         # placed by claw_asm at TAB_PTS

# ================================================================== ROTOR (helical 3-start grooves) and PUNCH RING
def helix_groove(start_deg, z0):
    """groove cutter: d2.9 circle swept along a helix of radius PIN_R, starting at z0; azimuth DECREASES with z (left-hand), lead LEAD."""
    h = ZN_HC - ZN_HO + 2.4
    path = cq.Workplane("XY").newObject([cq.Wire.makeHelix(LEAD, h, PIN_R, lefthand=True)])
    prof = cq.Workplane("XZ").center(PIN_R, 0).circle((PIN_D / 2 + 0.2) / math.cos(math.radians(ALPHA)) + 0.50)      # profile is in the vertical plane, helix is tilted ALPHA: widen so the normal section clears the pin
    g = prof.sweep(path, isFrenet=True)
    return g.translate((0, 0, z0 - 0.8)).rotate((0, 0, 0), (0, 0, 1), start_deg + 360.0 * 0.8 / LEAD + PHASE_FIX)
rot = cyl_z(10.5, ROT_BASE0, ROT_BASE1, 0, 0).union(cyl_z(ROT_R, ROT_BASE1 - 0.5, ROT_TOP, 0, 0))
for k in range(3): rot = rot.cut(helix_groove(AZ[0] + 120.0 * k, ZN_HO))
rot = rot.cut(cyl_z(1.6, ROT_BASE0 - 1, ROT_BASE0 + 1.5, 0, 0)).cut(cyl_z(1.3, ROT_TOP - 7.0, ROT_TOP + 1, 0, 0))    # horn-bond relief (PLACEHOLDER: no horn screws) / top M2.5 tap hole
parts["claw_rotor"] = rot.translate((0, D, 0))
# ring: tube + 3 lug bosses (drive links) + 2 ears + 3 radial pin holes
def ring_canon():
    rg = cyl_z(RING_RO, -RING_H / 2, RING_H / 2, 0, 0).cut(cyl_z(RING_RI, -RING_H, RING_H, 0, 0))
    return rg
rg = ring_canon()
lug0 = box(RING_RO - 2.0, RING_RO + 1.6, 6.5 - 0.0, 10.0, -RING_H / 2, RING_H / 2)                               # lug boss at +t 6.5..10 (outside the drive link layer), radial 11..14.6
NOTCH_R = 10.5
for az in AZ:                                                                                                    # link slot (t 3.2..6.5, r>=RL-5) and finger-plane notch (t -3.5..3.2, r>NOTCH_R): finger clears the ring
    rg = rg.cut(box(RL - 5.0, RING_RO + 3.0, 3.2, 6.5, -RING_H, RING_H).rotate((0, 0, 0), (0, 0, 1), az))
    rg = rg.cut(box(NOTCH_R, RING_RO + 3.0, -6.8, 3.2, -RING_H, RING_H).rotate((0, 0, 0), (0, 0, 1), az))
    rg = rg.cut(box(RING_RI - 1.0, RING_RO + 3.0, -4.5, 3.2, 0.8, RING_H).rotate((0, 0, 0), (0, 0, 1), az))       # top pocket: finger pad/phalanx dips into the ring top at hard-closed
for az in AZ:
    lug = lug0.cut(cyl_y(1.2, 3.0, 12.0, RL, 0.0))                                                              # lug pin hole d2.4 PRESS (rev 2026-10-15, was d2.6); lug pin runs in link_drive d2.6
    rg = rg.union(lug.rotate((0, 0, 0), (0, 0, 1), az))
for az in EAR_AZ:
    e_ = box(RING_RO - 1.0, PIL_R1 - 3.5 - 0.4, -2.5, 2.5, -RING_H / 2, RING_H / 2).rotate((0, 0, 0), (0, 0, 1), az); rg = rg.union(e_)
for az in AZ:                                                                                                    # radial pin holes at the finger azimuths, d2.4 (press for d2.5 steel)
    rg = rg.cut(cq.Workplane("YZ").circle(1.2).extrude(RING_RO + 2.0).rotate((0, 0, 0), (0, 0, 1), az))
parts["claw_ring"] = rg.translate((0, D, ZN_O))                                                                  # posed by claw_asm; reference at soft-open height
for k, az in enumerate(AZ):                                                                                      # ring pins (steel d2.5 x 8.1), proxies
    pn = cq.Workplane("YZ").circle(PIN_D / 2).extrude(NOTCH_R - 5.65).faces("<X").fillet(PIN_D / 2 - 0.02).translate((5.65, 0, 0)).rotate((0, 0, 0), (0, 0, 1), az).translate((0, D, ZN_O))
    prox[f"pin_ring{k}"] = pn
# hardware proxies per finger (canonical): pivot pin d3 x 16, Q pin d3 x 6.3, J pin d3 x 6.4, R pin d2.5 x 3.5, P pin d2.5 x 9.5, lug pin d2.5 x 6.5
prox["pin_pivot_F"] = cyl_y(1.5, Y(-9.5), Y(6.5), Bx, Bz); prox["pin_Q_F"] = cyl_y(1.5, Y(-9.5), Y(-3.5), Qx, Qz)
prox["pin_J_F"] = cyl_y(1.5, Y(-3.0), Y(3.0), Jx, Jz); prox["pin_R_F"] = cyl_y(1.25, Y(-6.5), Y(0.0), Rx, Rz)
prox["pin_P_F"] = cyl_y(1.25, Y(-3.0), Y(6.5), Px, Pz); prox["pin_lug_F"] = cyl_y(1.25, Y(3.5), Y(10.0), RL, ZN_O)

def export(w, name, sub=""):
    os.makedirs(os.path.join(OUT, sub), exist_ok=True)
    p = os.path.join(OUT, sub, name + ".stl"); cq.exporters.export(w, p, tolerance=0.03, angularTolerance=0.06); return p
if __name__ == "__main__":
    meta = {}
    for k, v in parts.items():
        export(v, k); sol = v.val(); bb = sol.BoundingBox()
        meta[k] = dict(vol_cm3=round(sol.Volume() / 1e3, 3), valid=sol.isValid(), bbox=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)]); print(k, meta[k])
    for k, v in prox.items(): export(v, k, "proxies")
    meta["_layout"] = dict(D=D, RB=RB, ZB=ZB, L1=L1, L2=L2, DP=DP, NP=NP, LD=LD, RL=RL, QA=QA, QB=QB, LC=LC, RY=RY, RN=RN, TH1_O=TH1_O, TH1_C=TH1_C, TH1_HARD=[TH1_HO, TH1_HC],
                           ZN=dict(hard_open=ZN_HO, open=ZN_O, closed=ZN_C, hard_closed=ZN_HC), K_HELIX=K_HELIX, LEAD=LEAD, ALPHA_DEG=ALPHA, PSI_SOFT=PSI_SOFT, PIN_R=PIN_R,
                           AZ=AZ, EAR_AZ=EAR_AZ, RING=dict(H=RING_H, RI=RING_RI, RO=RING_RO), ROT=dict(R=ROT_R, base=[ROT_BASE0, ROT_BASE1], top=ROT_TOP),
                           SERVO=dict(x=SV_X, y=[SV_Y0, SV_Y1], z=[Z_BOT, Z_TOPB], horn_top=Z_HORN1), TAB_PTS=TAB_PTS, Z_TIP=Z_TIP,
                           EARS_PLACEHOLDER=dict(x=EAR_X, y=EAR_Y, t=EAR_T, half_width=EAR_HW, z_seat=Z_EAR0, z_top=Z_EAR1, projection=EAR_L), PHASE_FIX=PHASE_FIX, PIL_TOP=PIL_TOP, CAP_T=CAP_T,
                           SHAFT_OFF_PLACEHOLDER=SHAFT_OFF, HORN_PLACEHOLDER=[HORN_D, HORN_P])
    json.dump(meta, open(os.path.join(OUT, "claw.json"), "w"), indent=1)
    print("ring z hard-open/open/closed/hard-closed:", [round(x, 2) for x in (ZN_HO, ZN_O, ZN_C, ZN_HC)], " helix lead %.1f mm/rev  alpha %.1f deg  K %.3f mm/rad" % (LEAD, ALPHA, K_HELIX))
