"""Upper arm + elbow stage (right arm, robot frame as exported by shoulder_stage.py): origin P, +x rearward, +y wearer's right, +z up.
S3 axis = +z; the S3 XM540 body sits at z 68..112, its output horn plane at z = 112 (horn protrusion UNVERIFIED -> 1 mm gap left).
The upper arm points +z at S3 = 0; the elbow axis is along y at (x=0, z=230). Positive elbow flexion E swings the forearm from +z toward -x (forward):
R = Ry(-E).  Left arm = mirror in y.
Printed parts (per arm): ua_lat, ua_med (shell halves, split plane y = 0), fork_drive, fork_idler, fork_bridge.
VERIFIED: XM540-W270 33.5 x 58.5 x 44 mm (axis along the 44 mm depth, output horn in the 58.5 x 33.5 face), 165 g, 10.6 N.m stall @12 V.
UNVERIFIED PLACEHOLDERS (do not print the marked features until replaced): HORN_P (horn protrusion), HORN_PCD/HORN_N (horn screw pattern),
IDLER_* (rear idler boss/threads of the XM540), S3 horn plane height.

Rev 2026-10-13b (elbow ball): the elbow is a BALL-AND-SOCKET-STYLED hinge (standing decision: ball-styled everywhere, passive stops, DOF and
actuator count unchanged: the XM540 stays a 1-DOF joint; forearm roll is W1). Geometry:
  * R_E = 44 mm sphere centred on the elbow axis point (0, 0, 230).
  * FIXED belt (upper arm, ua_lat/ua_med): the sphere cut to |y| <= 24.7, hollow (2.4 wall), carrying the XM540 pocket frame (the old box walls,
    clipped to the sphere) and the tube. The two halves close with the tube tabs plus two outer lap plates at azimuth +-125 deg (below the bridge
    sweep), M3 x 10 radially into heat-set inserts in inner bosses (same scheme as the shoulder ball).
  * MOVING polar caps (fork_drive, fork_idler): the same sphere R_E in the zones |y| = 25.7 .. 29.7 (drive) / 31.7 (idler), turning with the forearm,
    1.0 mm axial running gap to the belt. They carry the horn (drive) / 623ZZ bearing (idler), the neck to the rear crossbar and the forearm mount.
  * Passive E stops: solid root blocks inside the belt at azimuth +-(90..112) deg tie the shell to the pocket frame; stop posts + TPU 95A facings
    (distal_stops.py) stand on them and are struck by the crossbar (fork_bridge) at the hard angles -5 / +150.
Azimuth convention in this file: from +z toward +x about the elbow axis (rot_e)."""
import cadquery as cq, math, os, json
import numpy as np
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "upperarm"); os.makedirs(os.path.join(OUT, "proxies"), exist_ok=True)

XM_X, XM_Z, XM_D = 58.5, 33.5, 44.0       # elbow XM540: x (in the flexion plane, long), z (along the arm), y depth = axis direction
CLR, WALL, WALL_H = 0.3, 2.4, 2.4
Z_HORN = 112.0                            # S3 horn plane (UNVERIFIED horn protrusion)
Z_ELB = 230.0                             # elbow axis height (upper arm 230 mm from P)
HORN_P = 2.5                              # UNVERIFIED horn protrusion beyond the XM face
HORN_N, HORN_PCD = 4, 24.0                # UNVERIFIED horn screw pattern (4 x M2.5 on 24 mm PCD)
IDLER_D = 3.0                             # UNVERIFIED: M3 shoulder screw into a tapped rear boss on the XM540
Y_IN = XM_D / 2 + CLR                     # 22.3
Y_OUT = Y_IN + WALL                       # 24.7 housing outer
XH = XM_X / 2 + CLR + WALL                # 31.95 housing half-width in x
ZH0 = Z_ELB - XM_Z / 2 - CLR - 6.0        # housing bottom (6 mm bottom wall for closure screws)
ZH1 = Z_ELB + XM_Z / 2 + CLR + 6.0        # housing top    (6 mm top wall)
PZ0 = Z_ELB - XM_Z / 2 - CLR; PZ1 = Z_ELB + XM_Z / 2 + CLR   # pocket z range
# ---- elbow ball
R_E = 44.0                                # ball outer radius: pocket corner 40.9 from the axis -> 3.1 mm wall at the corner
R_EI = R_E - WALL                         # 41.6 inner radius of the belt shell
LAP_AZ = 125.0                            # deg, lap-plate azimuth (+-; i.e. 125 and 235): clear of the bridge sweep (+-88) and the stop blocks (90..112)
LAP_T, LAP_HW, LAP_Y0 = 2.0, 5.5, 0.6     # lap plate thickness above R_E, half width (tangential), y where the plate fuses to the lateral half
STOP_BLK = ((88.0, 114.0), (-114.0, -88.0))   # azimuth bands of the solid stop root blocks (rear: E- stop, front: E+ stop)

def box(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))
def cyl_y(r, y0, y1, cx=0.0, cz=0.0):
    return cq.Workplane("XZ").circle(r).extrude(-(y1 - y0)).translate((cx, y0, cz))     # XZ normal is -y: extrude(-d) goes +y
def cyl_x(r, x0, x1, cy=0.0, cz=0.0):
    return cq.Workplane("YZ").circle(r).extrude(x1 - x0).translate((x0, cy, cz))
def cyl_z(r, z0, z1, cx=0.0, cy=0.0):
    return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((cx, cy, z0))
def sph(r):
    return cq.Workplane("XY").sphere(r).translate((0, 0, Z_ELB))
def slab(y0, y1):
    return box(-80, 80, min(y0, y1), max(y0, y1), Z_ELB - 80, Z_ELB + 80)
def loft(sections):
    z0, a0, b0 = sections[0]
    wp = cq.Workplane("XY").workplane(offset=z0).ellipse(a0, b0); zp = z0
    for z, a, b in sections[1:]:
        wp = wp.workplane(offset=z - zp).ellipse(a, b); zp = z
    return wp.loft(combine=True)
def rot_e(w, ang): return w.rotate((0, 0, Z_ELB), (0, 1, Z_ELB), ang)          # azimuth +z -> +x
def wedge(a0, a1, r0, r1, yh, n=14):
    """solid sector about the elbow axis: azimuth a0..a1 (deg, +z toward +x), radii r0..r1, |y| <= yh"""
    pts = []
    for k in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * k / n); pts.append((r1 * math.sin(a), r1 * math.cos(a)))
    for k in range(n, -1, -1):
        a = math.radians(a0 + (a1 - a0) * k / n); pts.append((r0 * math.sin(a), r0 * math.cos(a)))
    return cq.Workplane("XZ").polyline(pts).close().extrude(-2 * yh).translate((0, -yh, Z_ELB))

# ---------------------------------------------------------------- tube sections (outer ellipse half-axes a (x), b (y)) + the ball belt
SEC = [(Z_HORN + 1.0, 32.5, 20.0), (150.0, 32.0, 21.0), (185.0, 32.0, 23.0), (ZH0 + 2.0, 31.9, Y_OUT)]
inner_sec = [(Z_HORN + 1.0 + 5.0, 32.5 - WALL, 20.0 - WALL), (150.0, 32.0 - WALL, 21.0 - WALL), (185.0, 32.0 - WALL, 23.0 - WALL), (ZH0 + 1.0, 31.9 - WALL, Y_OUT - WALL)]
tube_outer = loft(SEC)
belt_outer = sph(R_E).intersect(slab(-Y_OUT, Y_OUT))
outer = tube_outer.union(belt_outer)
cavity = loft(inner_sec).union(sph(R_EI).intersect(slab(-Y_IN, Y_IN)))
frame = box(-XH, XH, -Y_OUT, Y_OUT, ZH0, ZH1).edges("|Y").fillet(6.0).intersect(sph(R_E))        # the old box walls, clipped to the ball
shell = outer.cut(cavity).union(frame)
# stop root blocks: solid sectors from the pocket frame to the shell (the crossbar strikes posts that stand on them)
for a0, a1 in STOP_BLK:
    shell = shell.union(wedge(a0, a1, 27.0, R_E - 0.3, 8.0).intersect(sph(R_E)))
# elbow pocket (XM540) and the passage from the tube cavity to the pocket bottom (harness)
shell = shell.cut(box(-(XM_X / 2 + CLR), XM_X / 2 + CLR, -Y_IN, Y_IN, PZ0, PZ1))
shell = shell.cut(box(-9.0, 9.0, -4.0, 4.0, ZH0 - 1, PZ0 + 1))                    # 18 x 8 harness slot through the 6 mm bottom wall
# horn window on the lateral (+y) end wall, bearing/pin hole on the medial end wall
shell = shell.cut(cyl_y(16.5, Y_IN - 1, Y_OUT + 1, 0, Z_ELB))                     # O33 window: horn + cap boss pass through
shell = shell.cut(cyl_y(IDLER_D / 2 + 0.2, -Y_OUT - 1, -Y_IN + 1, 0, Z_ELB))      # O3.4 pin clearance (medial wall)
# S3 horn plate: z 113..118 across the whole root ellipse, 4 horn screw holes (head counterbores open into the cavity)
plate = tube_outer.intersect(box(-60, 60, -60, 60, Z_HORN + 1.0, Z_HORN + 6.0))      # plate = the outer solid's own bottom 5 mm (no sliver steps)
shell = shell.union(plate)
for i in range(HORN_N):
    a = math.radians(45 + 90 * i); x, y = HORN_PCD / 2 * math.cos(a), HORN_PCD / 2 * math.sin(a)
    shell = shell.cut(cyl_z(1.4, Z_HORN, Z_HORN + 8, x, y))                       # O2.8 clearance for M2.5
    shell = shell.cut(cyl_z(2.7, Z_HORN + 3.0, Z_HORN + 8, x, y))                 # head pocket

# ---------------------------------------------------------------- in-arm wiring (rev 2026-10-17): cable windows through the tube wall
# windows.json is written by arm_route.py (python3 arm_route.py windows) from the chosen cable chain (arm_chain.json): oriented boxes (centre, wall normal n, width 10 along the wall, height 12 in z).
# Each window passes the cable bundle (OD <= 4.73) through the 2.4 mm wall between the tube cavity and the outside anchor clip; edges are broken with 3 mm wide EPDM edge trim / heat-shrink (not modelled).
_EL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "electronics")
WINDOWS = json.load(open(os.path.join(_EL, "windows.json"))) if os.path.exists(os.path.join(_EL, "windows.json")) else []
def win_box(w):
    n = w["n"]; ang = math.degrees(math.atan2(n[1], n[0]))
    return cq.Workplane("XY").box(w["depth"], w["width"], w["height"]).rotate((0, 0, 0), (0, 0, 1), ang).translate(tuple(w["center"]))
for _w in WINDOWS: shell = shell.cut(win_box(_w))

# ---------------------------------------------------------------- split into lateral / medial halves (y = 0)
# tube closure: tabs on the outside of the tube, M3 along y. Ball closure: two outer lap plates (rev 2026-10-13b), see below.
STN = [128.0, 166.0, 196.0]          # tab stations (z); the two former stations at ZH0+8 / ZH1-8 sit inside the ball and are replaced by the laps
def xr(z):                                                  # tube half-width in x at height z
    return 32.5 if z < 150 else 32.0
TAB_H, TAB_W, TAB_T = 11.0, 8.0, 6.5
def tabs(sign):
    t = None
    for z in STN:
        for sx in (-1, 1):
            x0 = sx * (xr(z) - 1.5); x1 = sx * (xr(z) + TAB_W - 1.5)
            b = box(min(x0, x1), max(x0, x1), 0.0 if sign > 0 else -TAB_T, TAB_T if sign > 0 else 0.0, z - TAB_H / 2, z + TAB_H / 2)
            t = b if t is None else t.union(b)
    return t
lat = shell.intersect(box(-80, 80, 0, 60, 90, 330)).union(tabs(+1))
med = shell.intersect(box(-80, 80, -60, 0, 90, 330)).union(tabs(-1))
for z in STN:
    for sx in (-1, 1):
        x = sx * (xr(z) + TAB_W / 2 - 1.5)
        lat = lat.cut(cyl_y(1.7, -1, TAB_T + 1, x, z)).cut(cyl_y(3.1, 2.0, TAB_T + 1, x, z))      # M3 clearance + head counterbore (lateral side)
        med = med.cut(cyl_y(2.3, -5.7, 0.5, x, z))                                                  # O4.6 x 5.7 heat-set pocket (M3 insert), pressed in from the split face
# ball laps: outer plate fused to the lateral half (y >= LAP_Y0), overhanging the seam onto the medial half (0.4 mm gap);
# inner boss on the medial half carries the M3 insert (pressed in from the cavity side before the halves are closed)
LY = -4.5                                                    # screw line y (on the medial half)
def lap_local():
    win = box(-LAP_HW, LAP_HW, -8.0, 8.0, Z_ELB + 30.0, Z_ELB + R_E + LAP_T + 4.0)
    a = sph(R_E + LAP_T).cut(sph(R_E - 0.3)).intersect(win).intersect(slab(LAP_Y0, 8.0))
    b = sph(R_E + LAP_T).cut(sph(R_E + 0.4)).intersect(win).intersect(slab(-8.0, LAP_Y0))
    return a.union(b)
def boss_local():
    return cyl_z(4.5, Z_ELB + R_EI - 5.0, Z_ELB + R_E - 0.1, 0.0, LY).intersect(sph(R_E - 0.05))
def lap_holes_local():
    h = cyl_z(1.7, Z_ELB + R_EI - 1.0, Z_ELB + R_E + LAP_T + 2.0, 0.0, LY)                      # M3 clearance through wall + lap
    h = h.union(cyl_z(2.3, Z_ELB + R_EI - 5.0, Z_ELB + R_EI + 0.2, 0.0, LY))                      # O4.6 x 5.2 insert pocket (from the cavity side)
    return h
for az in (LAP_AZ, -LAP_AZ):
    lat = lat.union(rot_e(lap_local(), az)).cut(rot_e(lap_holes_local(), az))
    med = med.union(rot_e(boss_local(), az)).cut(rot_e(lap_holes_local(), az))
# hub board H1 mounting posts (arm_hub.json): 4 posts O7 on the medial inner wall carrying M2.5 heat-set inserts (O3.6 x 4.5 deep, from the top); board on the posts, M2.5 x 6 pan head from the +y side
_HUB = os.path.join(_EL, "arm_hub.json")
def y_in(x, z):                                              # medial inner wall y (negative) of the tube cavity at (x, z): interpolate the inner ellipse sections
    zs = [s_[0] for s_ in inner_sec]; a = float(np.interp(z, zs, [s_[1] for s_ in inner_sec])); b = float(np.interp(z, zs, [s_[2] for s_ in inner_sec]))
    return -b * math.sqrt(max(1.0 - (x / a) ** 2, 0.0))
HUB_POSTS = []
if os.path.exists(_HUB):
    _h = json.load(open(_HUB)); _yb, _zb = _h["y_board"], _h["z_board"]; _ytop = _yb - 0.8
    for hx, hz in _h["holes"]:
        z = _zb + hz; ylo = y_in(hx, z) - 1.2
        post = cyl_y(3.5, ylo, _ytop, hx, z).cut(cyl_y(1.8, _ytop - 4.5, _ytop + 0.01, hx, z)); med = med.union(post); HUB_POSTS.append(dict(x=hx, z=z, y0=round(ylo, 2), y1=round(_ytop, 2)))
parts = {"ua_lat": lat, "ua_med": med}

# ---------------------------------------------------------------- elbow fork: polar caps + crossbar (rotate with the forearm about the elbow axis)
# rev 2026-10-12c (I6): the cross bridge moves to the REAR of the elbow (PHI_B = 70 deg from +z toward +x at E = 0, like an olecranon)
# so it never sweeps into the upper-arm tube: over E -5..150 it stays inside -75..80 deg. The forearm no longer sits on the bridge:
# it bolts to the outer face of the drive cap (+y) and is offset laterally (distal_arm.py D_OFF), which lets the elbow fold to 150.
# rev 2026-10-13b: the flat cheek discs become spherical caps (zones of the R_E ball), R_BRIDGE 54 unchanged.
R_BRIDGE = 54.0     # crossbar CENTRE radius; inner face 48 (ball belt R_E 44 -> 4.0 mm radial gap)
PHI_B = 70.0        # deg, bridge position about the elbow axis, +z toward +x
CH_T = 4.0          # drive cap thickness along y
CH_TI = 6.0         # idler cap thickness (4 mm bearing seat + 2 mm outer lip)
CHG = 1.0                                   # cap-to-belt running gap (printed parts)
Y_CH_L0 = Y_OUT + CHG                       # 25.7: drive cap inner face
Y_CH_M0 = -(Y_OUT + CHG)                    # -25.7: idler cap inner face
Z_MOUNT = Z_ELB + 62.0                      # drive-cap forearm mount neck top (z 292)
def cap(y0, sign, thick, with_horn):
    y1 = y0 + sign * thick
    c = sph(R_E).intersect(slab(y0, y1))                                   # spherical-zone cap (a lens of the ball)
    neck = box(-12.0, 12.0, min(y0, y1), max(y0, y1), Z_ELB, Z_ELB + R_BRIDGE + 6.0)
    for sx in (-6.0, 6.0):       # two M3 screws along y join each neck to the crossbar end (counterbored from the outside face)
        neck = neck.cut(cyl_y(1.7, min(y0, y1) - 1, max(y0, y1) + 1, sx, Z_ELB + R_BRIDGE)).cut(
            cyl_y(3.1, (y1 - 1.5) if sign > 0 else (y1 - 1), (y1 + 1) if sign > 0 else (y1 + 1.5), sx, Z_ELB + R_BRIDGE))
    c = c.union(rot_e(neck, PHI_B))
    if with_horn:
        c = c.union(cyl_y(14.0, Y_IN + HORN_P - 0.4, y0 + 0.01, 0, Z_ELB))      # spacer boss to the horn face; UNVERIFIED horn protrusion
        for i in range(HORN_N):
            a = math.radians(45 + 90 * i); x, z = HORN_PCD / 2 * math.cos(a), HORN_PCD / 2 * math.sin(a)
            c = c.cut(cyl_y(1.4, Y_IN, y1 + 1, x, Z_ELB + z)).cut(cyl_y(2.7, y1 - 1.5, y1 + 1, x, Z_ELB + z))
        # forearm mount neck straight up (+z) on the drive side: the forearm root bolts to its outer face (4 x M3 along y)
        mount = box(-14.0, 14.0, min(y0, y1), max(y0, y1) + 2.0, Z_ELB + 20.0, Z_MOUNT)
        for xx in (-8.0, 8.0):
            for zz in (Z_MOUNT - 9.0, Z_MOUNT - 27.0):
                mount = mount.cut(cyl_y(2.3, max(y0, y1) + 2.0 - 6.0, max(y0, y1) + 2.0 + 0.5, xx, zz))     # M3 heat-set pockets from the outer face
        c = c.union(mount)
    else:
        # 623ZZ (3 x 10 x 4) seat O10.2 x 4 deep from the inner face, 2 mm outer lip with O3.4 for the M3 shoulder screw (rev 2026-10-13b: the old seat went through)
        c = c.cut(cyl_y(5.1, y0 - 1 if sign > 0 else y1 + 0.0 - 0.0, y0 + sign * 4.0, 0, Z_ELB) if sign > 0 else cyl_y(5.1, y0 - 4.0, y0 + 1, 0, Z_ELB))
        c = c.cut(cyl_y(1.7, min(y0, y1) - 1, max(y0, y1) + 1, 0, Z_ELB))
    return c
fork_drive = cap(Y_CH_L0, +1, CH_T, True)
fork_idler = cap(Y_CH_M0, -1, CH_TI, False)
bridge = box(-12.0, 12.0, -(Y_OUT + CHG - 0.2), Y_OUT + CHG - 0.2, Z_ELB + R_BRIDGE - 6.0, Z_ELB + R_BRIDGE + 6.0)       # sits BETWEEN the cap necks (0.2 mm side play)
for sx in (-6.0, 6.0):
    for ys in (+1, -1):    # O4.6 x 6 deep M3 heat-set pockets in both ends
        bridge = bridge.cut(cyl_y(2.3, ys * (Y_OUT + CHG - 0.2) - (6.0 if ys > 0 else 0), ys * (Y_OUT + CHG - 0.2) + (0 if ys > 0 else 6.0), sx, Z_ELB + R_BRIDGE))
bridge = rot_e(bridge, PHI_B)
parts.update({"fork_drive": fork_drive, "fork_idler": fork_idler, "fork_bridge": bridge})

# ---------------------------------------------------------------- non-printed envelopes (collision proxies)
prox = {"xm_elbow": box(-XM_X / 2, XM_X / 2, -XM_D / 2, XM_D / 2, Z_ELB - XM_Z / 2, Z_ELB + XM_Z / 2)}
meta = {}
for k, v in parts.items():
    sol = v.val(); meta[k] = dict(vol_cm3=round(sol.Volume() / 1000, 1), valid=sol.isValid(),
                                  bbox=[round(b, 1) for b in (sol.BoundingBox().xlen, sol.BoundingBox().ylen, sol.BoundingBox().zlen)])
    cq.exporters.export(v, os.path.join(OUT, k + "_R.stl"), tolerance=0.03, angularTolerance=0.05)
import trimesh
for k in parts:     # drop zero-volume single-face tessellation slivers: keep only the bodies with real volume, require watertight
    f = os.path.join(OUT, k + "_R.stl"); m = trimesh.load(f); bodies = [b for b in m.split(only_watertight=False) if abs(b.volume) > 1.0]
    m2 = trimesh.util.concatenate(bodies); m2.export(f); meta[k].update(bodies=len(bodies), watertight=bool(m2.is_watertight), mass_g_petg=round(abs(m2.volume) / 1e3 * 1.27, 1))
for k, v in prox.items(): cq.exporters.export(v, os.path.join(OUT, "proxies", k + ".stl"), tolerance=0.05)
meta["_wiring"] = dict(windows=[dict(center=w["center"], n=w["n"], width=w["width"], height=w["height"], depth=w["depth"]) for w in WINDOWS], hub_posts=HUB_POSTS)
meta["_elbow_ball"] = dict(R_E=R_E, R_EI=R_EI, LAP_AZ=LAP_AZ, STOP_BLK=STOP_BLK, R_BRIDGE=R_BRIDGE, PHI_B=PHI_B, CHG=CHG, Y_OUT=Y_OUT)
json.dump(meta, open(os.path.join(OUT, "meta.json"), "w"), indent=1)
for k, m in meta.items(): print("%-12s %s" % (k, m))
