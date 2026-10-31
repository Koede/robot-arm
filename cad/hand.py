"""HAND: human-layout 5-digit micro-servo hand with CHUCK mode.  Canonical printable parts -> stl/hand/*.stl ; pose assembler -> stl/hand/poses/.
Frames: hand frame (x palm normal, u = y - D, z absolute).  Part-local frames: X = a (along phalanx), Y = t (width), Z = b (palmar normal); a x t = b.
PLACEHOLDERS: micro-servo dimensions/horn (MG90S class), rod-end balls, hinge pins, 6 V rail, driver board.  Densities g/cm3: PA-CF 1.20, PETG-CF 1.30, TPU 1.21, steel 7.85."""
import cadquery as cq, numpy as np, trimesh, json, os, sys, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K
HERE = os.path.dirname(os.path.abspath(__file__)); OUT = os.path.join(HERE, "..", "stl", "hand"); os.makedirs(OUT + "/poses", exist_ok=True)
FB = json.load(open(OUT + "/fourbars.json"))
D, Z_W3 = 57.0, 455.0
BW, BT = 8.0, 10.0                         # phalanx width (t) and thickness (b)
def prism(pts, t0, t1): return cq.Workplane("XZ").polyline([(float(a), float(b)) for a, b in pts]).close().extrude(-(t1 - t0)).translate((0, t0, 0))
def hull(circ, n=28):
    from scipy.spatial import ConvexHull
    P = np.array([(cx + r*np.cos(t), cz + r*np.sin(t)) for cx, cz, r in circ for t in np.linspace(0, 2*np.pi, n, endpoint=False)]); return P[ConvexHull(P).vertices]
def cylt(r, t0, t1, a, b): return cq.Workplane("XZ").circle(r).extrude(-(t1 - t0)).translate((a, t0, b))
def box(x0, x1, y0, y1, z0, z1): return cq.Workplane("XY").box(x1-x0, y1-y0, z1-z0, centered=False).translate((x0, y0, z0))
PIN_R, PIN_RUN = 1.05, 1.0                  # d2 pins: press hole r1.0, running hole r1.05 (print-compensated later)
def stadium(length, w, t0, t1, hr=1.1):
    p = prism(hull([(0, 0, w), (length, 0, w)], 24), t0, t1)
    return p.cut(cylt(hr, t0 - 1, t1 + 1, 0, 0)).cut(cylt(hr, t0 - 1, t1 + 1, length, 0))

parts = {}; meta = {}
def finger_parts(nm):
    L1, L2, L3 = K.LEN[nm]; fb = FB[nm]; Gp, Hp = fb['pip']['G'], fb['pip']['H']; Gd, Hd = fb['dip']['G'], fb['dip']['H']
    W2 = BW/2
    # proximal: MCP end r5.5, PIP end r4.8, tail (rod lever) behind the MCP: ball stud at a=-8 (b 0)
    p = prism(hull([(0, 0, 5.5), (L1, 0, 4.8), (-8, 0, 3.0)]), -W2, W2)
    p = p.cut(cylt(PIN_RUN + 0.05, -W2-1, W2+1, 0, 0)).cut(cylt(PIN_R - 0.05, -W2-1, W2+1, L1, 0))          # MCP running, PIP press
    p = p.union(cylt(1.0, -W2-3.3, -W2, L1 + Gd[0], Gd[1]))                                                       # G2 post for the DIP bar (-t side)
    p = p.union(cq.Workplane("XY").sphere(1.9).translate((-8, 0, 0)).union(cylt(0.8, -2, 2, -8, 0)))        # ball stud (PLACEHOLDER rod end)
    parts[f"{nm}_prox"] = p
    m = prism(hull([(0, 0, 4.8), (L2, 0, 4.2)]), -W2, W2)
    m = m.cut(cylt(PIN_RUN + 0.05, -W2-1, W2+1, 0, 0)).cut(cylt(PIN_R - 0.05, -W2-1, W2+1, L2, 0)).union(cylt(1.0, W2, W2 + 3.3, Hp[0], Hp[1]))   # H1 post for the PIP bar (+t)
    parts[f"{nm}_mid"] = m
    d = prism(hull([(0, 0, 4.2), (L3 - 3.2, 0, 3.4), (L3 - 2.6, 0, 3.4)]), -W2, W2)
    d = d.cut(cylt(PIN_RUN + 0.05, -W2-1, W2+1, 0, 0)).union(cylt(1.0, -W2-3.3, -W2, Hd[0], Hd[1]))        # H2 post for the DIP bar (-t)
    parts[f"{nm}_dist"] = d
    parts[f"{nm}_pad"] = prism([(L3*0.18, 4.0), (L3*0.18, 6.0), (L3 - 1.2, 5.4), (L3 - 1.2, 4.0)], -W2 + 0.4, W2 - 0.4)
    parts[f"{nm}_bar_pip"] = stadium(fb['pip']['lb'], 2.4, W2 + 0.4, W2 + 2.2)
    parts[f"{nm}_bar_dip"] = stadium(fb['dip']['lb'], 2.4, -W2 - 2.2, -W2 - 0.4)
    kn = box(-14, 6, -6.4, 6.4, -6.5, 6.5).cut(cylt(PIN_R - 0.05, -9, 9, 0, 0)).union(cylt(1.0, W2, W2 + 3.3, Gp[0], Gp[1]))   # knuckle/yaw block with G1 post
    kn = kn.cut(box(-14, -3, -W2 - 0.3, W2 + 0.3, -3, 7)).union(box(-14, -3, -7.5, -W2-0.3, -6.5, 6.5)) if False else kn
    parts[f"{nm}_knuckle"] = kn
for nm in K.LEN: finger_parts(nm)
# ---------------------------------------------------------------- thumb segments (straight canonical)
TL = K.TLEN
parts["thumb_meta"] = prism(hull([(0, 0, 7.0), (TL[0], 0, 5.8)]), -6, 6).cut(cylt(1.1, -7, 7, TL[0], 0))
parts["thumb_prox"] = prism(hull([(0, 0, 5.8), (TL[1], 0, 5.0)]), -5.2, 5.2).cut(cylt(1.0, -7, 7, 0, 0)).cut(cylt(1.0, -7, 7, TL[1], 0))
parts["thumb_dist"] = prism(hull([(0, 0, 5.0), (TL[2] - 3.5, 0, 4.0), (TL[2] - 2.8, 0, 4.0)]), -5.2, 5.2).cut(cylt(1.0, -7, 7, 0, 0))
parts["thumb_pad"] = prism([(TL[2]*0.2, 4.0), (TL[2]*0.2, 6.0), (TL[2] - 1.0, 5.4), (TL[2] - 1.0, 4.0)], -4.8, 4.8)
# ---------------------------------------------------------------- palm tray v2 (PETG-CF): base plate ON the wrist mating plane (no ears, no yoke), palm slab, thenar block, servo pockets
DZ = K.DZ; XF = 26.8; HB0 = 491.6 + DZ; FT = 493.6 + DZ                    # mating plane z = HB0 (rotor flange top, wrist_rotary.py MATE)
SV = dict(L=22.8, W=12.2, H=22.7, shaft=6.0)                              # MG90S-class micro servo (PLACEHOLDER)
PX0, PX1 = -60.0, -34.0                                                   # palm slab x range
palm = box(PX0, XF, D - 52, D + 54, HB0, FT).union(box(PX0, PX1, D - 52, D + 56, FT, 560.0 + DZ))
palm = palm.cut(box(PX1 + 0.01, XF + 1, D + 14.0, D + 62, HB0 - 1, FT + 1))                  # trim the floor plate ahead of the index/thumb so a curled index clears it
palm = palm.cut(cq.Workplane("XY").circle(11.0).extrude(FT - HB0 + 2).translate((0, D, HB0 - 1)))          # wire port d22 over the rotor neck (neck bore r15, flange r11..19)
for ang in (45, 135, 225, 315):                                                                               # 4 x M3 clearance on the flange r15 circle (rotor inserts PLACEHOLDER)
    palm = palm.cut(cq.Workplane("XY").circle(1.7).extrude(FT - HB0 + 2).translate((15.0 * math.cos(math.radians(ang)), D + 15.0 * math.sin(math.radians(ang)), HB0 - 1)))
FSERV = {}                                                                 # servo pocket centres (u, z_c, x range)
UF = dict(index=13.5, middle=0.0, ring=-13.5, pinky=-26.5)        # servo-body centre u (fingers sit at K.U; rods fan out on ball ends)
ZS = 506.0 + DZ                                                                  # finger-servo shaft z (shaft at the distal end of the body)
for nm, u in UF.items():
    zc = ZS - (SV['L']/2 - SV['shaft'])                                     # body centre z (shaft 6 mm from the distal end -> body extends proximally)
    FSERV[nm] = dict(u=u, z0=ZS - SV['L'] + SV['shaft'], z1=ZS + SV['shaft'], x0=PX1 - SV['H'], x1=PX1)
    palm = palm.cut(box(PX1 - SV['H'] - 0.3, PX1 + 0.01, D + u - SV['W']/2 - 0.3, D + u + SV['W']/2 + 0.3, FSERV[nm]['z0'] - 0.3, FSERV[nm]['z1'] + 0.3))
# thenar block servos (shaft || u): swing + lift at z 496..518.8, flex at z 521..543.8 ; yaw servo (shaft || x) at u 0, z 522..544.8
TH_U0, TH_U1 = 25.0, 25.0 + SV['H']
THS = dict(t_swing=dict(x0=PX0 + 0.5, z0=490.0 + DZ), t_lift=dict(x0=PX0 + 0.5 + SV['W'] + 0.8, z0=490.0 + DZ), t_flex=dict(x0=PX0 + 0.5, z0=515.5 + DZ))
for k_, v_ in THS.items():
    palm = palm.cut(box(v_['x0'] - 0.3, v_['x0'] + SV['W'] + 0.3, D + TH_U0 - 0.3, D + TH_U1 + 0.3, v_['z0'] - 0.3, v_['z0'] + SV['L'] + 0.3))
YAW = dict(u=0.0, z0=514.0 + DZ, x0=PX1 - SV['H'], x1=PX1)
palm = palm.cut(box(YAW['x0'] - 0.3, PX1 + 0.01, D - SV['W']/2 - 0.3, D + SV['W']/2 + 0.3, YAW['z0'] - 0.3, YAW['z0'] + SV['L'] + 0.3))
palm = palm.cut(box(-14.3, -9.2, D - 28.3, D + 15.1, FT - 1.2, FT + 0.01))                                  # collector groove, crosses the port edge at x -11
for nm_, u_ in UF.items():                                                  # branch grooves 5 x 1.2 from the collector to each servo pocket
    palm = palm.cut(box(PX1 - 1.0, -9.7, D + u_ - 2.5, D + u_ + 2.5, FT - 1.2, FT + 0.01))
parts["thumb_cmc"] = box(-8, 8, -8, 8, -7, 7).cut(cylt(1.1, -10, 10, 0, 0))                                            # CMC housing/cradle (PETG-CF), placed at T0
parts["palm_tray"] = palm
parts_info = dict(FSERV=FSERV, THS=THS, YAW=YAW)
def export(w, name, sub=""):
    p = os.path.join(OUT, sub, name + ".stl"); os.makedirs(os.path.dirname(p), exist_ok=True); cq.exporters.export(w, p, tolerance=0.02, angularTolerance=0.05); return p
if __name__ == "__main__":
    DENS = lambda n: 1.30 if n in ("palm_tray", "thumb_cmc") or n.endswith("knuckle") else (1.21 if n.endswith("pad") else 1.20)
    tot = 0.0
    for k, v in parts.items():
        sol = v.val(); export(v, k); vol = sol.Volume()/1e3; m = vol*DENS(k); tot += m; bb = sol.BoundingBox()
        meta[k] = dict(vol_cm3=round(vol, 3), g=round(m, 2), valid=bool(sol.isValid()), bbox=[round(bb.xlen, 1), round(bb.ylen, 1), round(bb.zlen, 1)])
    json.dump(dict(parts=meta, printed_total_canonical_g=round(tot, 1)), open(OUT + "/hand_parts.json", "w"), indent=1)
    print(len(parts), "parts exported; canonical printed mass (1 of each, unrotated) g:", round(tot, 1)); print({k: v['valid'] for k, v in meta.items() if not v['valid']})
