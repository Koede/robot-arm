"""WRIST ROTARY COUPLING v3 UNIFIED (2026-10-30): monolithic PA12 cross-strip flexure supports (concept C) at +-x, crank + push-rod W3 drive, rigid R stage (R servo at -y): plane-to-plane hand coupling that replaces the old W3 "H" yoke (two 3.5 mm ears + floor plate cast into the hand palm_tray).
Forest (2026-10-30): the hand couples plane-to-plane through interconnects that route the wires AND carry load; the plane also rotates the wrist in-plane (8th powered DOF "R",
one more STS3215, W3 hinge KEPT).  Right arm, robot frame, mm.  W3 axis = x through (y D, z Z_W3).  R axis = hand axis (x 0, y D), along z.
Stack (z): W3 servo top 488.6 | carrier plate 493.8..495.8 (r >= 38.8 from the W3 axis = 3 mm swept clearance to the servo corner) | gear band + bearing 496..503 |
rotor web 503..505 | rotor neck to the MATING PLANE z = 491.6 + DZ  (hand palm floor underside; hand lifted rigidly by DZ).
Parts:  carrier (arm side: slim teardrop ears, plate, hub spindle that IS the ring-contact interconnect, servo arm), rotor (hand side: external gear 52T m1.25, web, neck, mating flange),
        pinion 16T, R servo STS3215 (vertical, below the plate, outboard on -x), bearing 6808-2RS (same part as W1), contact lands (4 concentric rings) + 8 pogo pins.
PLACEHOLDERS: STS3215 horn/shaft offset, servo-ear pattern, pogo pin / PCB vendor parts, gear tooth profile (trapezoid approximation, replace with involute), bearing retention.
Densities g/cm3: PA-CF 1.20, PETG-CF 1.30, brass 8.5, steel 7.85."""
import cadquery as cq, numpy as np, trimesh, json, os, math, sys
from scipy.spatial import ConvexHull
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "wrist_rotary"); os.makedirs(OUT, exist_ok=True)
sys.path.insert(0, HERE)
D, Z_W3 = 57.0, 455.0
ST_L, ST_W, ST_H, SHAFT_OFF = 45.2, 24.7, 35.0, 11.0          # STS3215 body (L along x here), shaft 11 mm from the body centre (PLACEHOLDER)
DZ = 24.0                                                     # lift of the whole hand along z (hand floor underside 491.6 -> 515.6)
PL0, PL1 = 493.8, 495.8                                       # carrier plate
GZ0, GZ1 = 496.0, 503.0                                       # gear band / bearing
WEB1 = 505.0                                                  # rotor web top
MATE = 491.6 + DZ                                             # mating plane (hand floor underside)
NG, NP, M = 52, 16, 1.25                                      # gear teeth, pinion teeth, module
RG, RP = NG * M / 2, NP * M / 2                               # pitch radii 32.5 / 10
CD = RG + RP                                                  # centre distance 42.5
BR_ID, BR_OD = 40.0, 52.0                                     # 6808-2RS, 7 wide
def box(x0, x1, y0, y1, z0, z1): return cq.Workplane("XY").box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), centered=False).translate((min(x0, x1), min(y0, y1), min(z0, z1)))
def cyl_z(r, z0, z1, cx=0.0, cy=D): return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((cx, cy, z0))
def tube_z(ro, ri, z0, z1, cx=0.0, cy=D): return cyl_z(ro, z0, z1, cx, cy).cut(cyl_z(ri, z0 - 1, z1 + 1, cx, cy))
def gear(n, mod, z0, z1, cx, cy, bore=0.0):
    rp = n * mod / 2; ra = rp + mod; rf = rp - 1.25 * mod; p = 2 * math.pi / n; pts = []
    for i in range(n):
        a = i * p
        for da, r in ((0.0, rf), (0.16, ra), (0.40, ra), (0.58, rf)): pts.append((cx + r * math.cos(a + da * p), cy + r * math.sin(a + da * p)))
    g = cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).translate((0, 0, z0))
    return g.cut(cyl_z(bore, z0 - 1, z1 + 1, cx, cy)) if bore else g
parts, prox = {}, {}

# ---------------------------------------------------------------- carrier (arm side, rotates with W3)
def hull2(circ):
    P = np.array([(cy + r * np.cos(t), cz + r * np.sin(t)) for cy, cz, r in circ for t in np.linspace(0, 2 * np.pi, 40, endpoint=False)]); return P[ConvexHull(P).vertices]
HUB_BORE = 6.0                                                # hollow spindle: brass tube OD 12 / ID 9 pressed in a d12 bore, wires run inside it
YS = D - CD                                                   # R servo / pinion sit on the -y side (x = 0): both x sides are free for the W3 flexure supports
# ---- carrier (moving with W3): plate A + arm to the R servo + hub + drive lug.  No ears, no paddle, no yoke.
carrier = box(-46.0, 46.0, D - 26.0, D + 26.0, PL0, PL1).union(box(-34.0, 14.0, D - 56.0, D - 20.0, PL0, PL1)).union(box(-34.0, -32.0, D - 56.0, D - 20.0, PL1, PL1 + 3.0))
carrier = carrier.union(box(20.2, 22.8, D - 5.0, D + 5.0, 489.8, PL0))                                      # push-rod lug (r 39 from the W3 axis)
hub = cyl_z(20.0, PL1, 502.4)
parts["carrier"] = carrier.union(hub).cut(cyl_z(HUB_BORE, PL0 - 1, 503.5)).cut(cyl_z(5.0, PL0 - 1, PL1 + 1, 0.0, YS))
# ---- W3 supports: monolithic PA12 cross-strip flexure (concept C, 0.7 x 24 x 20 leaves at +-45 deg on the W3 axis), one at +x and its mirror at -x
LF_T, LF_W, LF_L, X0F = 0.7, 20.0, 24.0, 25.0
def flexure(sgn):
    a_ = LF_L / 2 / math.sqrt(2)
    xa, xb = (X0F, X0F + LF_W) if sgn > 0 else (-(X0F + LF_W), -X0F)
    fl = box(xa, xb, D - 13, D + 13, Z_W3 - a_ - 4.0, Z_W3 - a_ + 0.5)                                    # base bar
    fl = fl.union(box(20.2 * sgn, xa if sgn > 0 else xb, D - 12, D + 12, Z_W3 - a_ - 4.0, Z_W3 - a_ - 0.5))   # bracket to the W2 cradle face (2 x M3 PLACEHOLDER)
    fl = fl.union(box(xa, xb, D - 13, D + 13, Z_W3 + a_ - 0.5, Z_W3 + a_ + 4.0))                         # moving top bar
    for sg in (1, -1):
        leaf = cq.Workplane("XY").box(LF_W, LF_L + 1.0, LF_T, centered=(False, True, True)).translate((xa, 0, 0)).rotate((0, 0, 0), (1, 0, 0), sg * 45.0).translate((0, D, Z_W3))
        fl = fl.union(leaf)
    fl = fl.union(box(xa, xb, D - 7, D + 7, Z_W3 + a_ + 3.5, 489.8)).union(box(xa - 2, xb + 2, D - 16, D + 16, 489.8, PL0))   # strut + top pad; plate A sits on the pad
    for yy in (-10, 10):
        for xx in (xa + 2, xb - 2): fl = fl.cut(cq.Workplane("XY").circle(1.7).extrude(10).translate((xx, D + yy, 484.8)))   # 4 x M3 clearance (plate A tapped / inserts, PLACEHOLDER)
    return fl
parts["flexure_px"] = flexure(1); parts["flexure_nx"] = flexure(-1)
prox["w3_crank"] = cq.Workplane("YZ").circle(6.0).extrude(2.3).translate((20.6, D, Z_W3)).union(box(20.6, 22.9, D - 3.0, D + 3.0, Z_W3, Z_W3 + 15.0))                                   # crank r15 on the W3 horn (+x side)
prox["w3_rod"] = cq.Workplane("XY").circle(1.2).extrude(489.8 - (Z_W3 + 15.0)).translate((21.5, D, Z_W3 + 15.0))   # push-rod d3 to the lug
prox["spindle_tube"] = tube_z(6.0, 4.5, PL0 - 2.0, 513.0, 0.0, D)   # brass OD 12 / ID 9, structural + wire conduit, perpendicular to both planes
rotor = gear(NG, M, GZ0, GZ1, 0.0, D, bore=BR_OD / 2 - 0.05)
rotor = rotor.union(tube_z(33.75, 15.0, GZ1, WEB1))                                           # web over the gear band (open centre r15 for the spindle)
rotor = rotor.union(tube_z(19.0, 15.0, WEB1, MATE))                                           # hollow neck r15..19 = wire cavity, clear of the index knuckle (r >= 22)
rotor = rotor.union(tube_z(19.0, 11.0, MATE - 2.0, MATE))                                     # mating flange, 4 x M3 heat-set (PLACEHOLDER pattern) into the hand floor
for ang in (90, 210, 330): rotor = rotor.union(box(0, 15.5, -1.0, 1.0, 511.6, 513.6).rotate((0, 0, 0), (0, 0, 1), ang).translate((0, D, 0)))   # 3 spider spokes
rotor = rotor.union(tube_z(7.5, 6.05, 510.6, 513.6, 0.0, D))                                  # top bushing (PTFE-lined) on the spindle tube: second support 12 mm above the bearing
parts["rotor"] = rotor
parts["pinion"] = gear(NP, M, GZ0, GZ1, 0.0, YS, bore=3.0)
prox["bearing_6808"] = tube_z(BR_OD / 2, BR_ID / 2, GZ0, GZ1)
prox["sts_R"] = box(-SHAFT_OFF - ST_L / 2, -SHAFT_OFF + ST_L / 2, YS - ST_W / 2, YS + ST_W / 2, PL0 - ST_H, PL0)
# ---------------------------------------------------------------- hand palm_tray without the old ears, lifted by DZ
import hand as Hd
palm = Hd.parts["palm_tray"]
palm = palm.cut(box(-30, 30, D - 20, D + 20, 430, 491.6 - 0.001)).cut(box(20, 30, D - 20, D + 20, 493.6, 496.0)).cut(box(-30, -20, D - 20, D + 20, 493.6, 496.0))
parts["palm_tray_wrist"] = palm.translate((0, 0, DZ))
DENS = dict(carrier=1.30, rotor=1.20, pinion=1.20, palm_tray_wrist=1.30, flexure_px=1.01, flexure_nx=1.01)
def export(w, name, sub=""):
    p = os.path.join(OUT, sub, name + ".stl"); os.makedirs(os.path.dirname(p), exist_ok=True); cq.exporters.export(w, p, tolerance=0.02, angularTolerance=0.05); return p
if __name__ == "__main__":
    meta = {}
    for k, v in parts.items():
        sol = v.val(); export(v, k); vol = sol.Volume() / 1e3; bb = sol.BoundingBox(); tm = trimesh.load(os.path.join(OUT, k + ".stl"))
        meta[k] = dict(vol_cm3=round(vol, 3), g=round(vol * DENS[k], 1), valid=bool(sol.isValid()), watertight=bool(tm.is_watertight), bbox=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)])
        print(k, meta[k])
    for k, v in prox.items(): export(v, k, "proxies")
    meta["_layout"] = dict(DZ=DZ, MATE=MATE, plate=[PL0, PL1], gear_band=[GZ0, GZ1], gear_pitch_R=RG, pinion_R=RP, centre_distance=CD, ratio=NG / NP, bearing="6808-2RS 40x52x7",
                          wrist_to_tip_open_mm=round(219.0 + DZ, 1), ratio_gear=NG / NP)
    json.dump(meta, open(os.path.join(OUT, "wrist_rotary.json"), "w"), indent=1)
