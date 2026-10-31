"""CONCEPT B: coil-spring compliant core (fork study, 2026-10-30).  Plane F (fixed on the W3 servo top) -> close-wound steel coil core around a PTFE-lined wire bore -> plane A' (carries the rigid R stage).
4 Dyneema tendons at r 22 through fairleads on radially curved rim plates; W3 servo capstan pulls the +-y antagonistic pair.  Units mm, N, deg."""
import sys, os, json, math; import numpy as np, cadquery as cq, trimesh
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP
from shapely.geometry import Polygon
HERE = os.path.dirname(os.path.abspath(__file__)); ST = os.path.join(HERE, "..", "..", "stl"); L = lambda p: trimesh.load(os.path.join(ST, p))
D, ZW3 = 57.0, 455.0
# ---------------- spring + loads (closed-form)
d, OD, Na, NT = 1.5, 24.0, 6, 8; Dm = OD - d; E, G, nu = 200e3, 79.3e3, 0.3
I = math.pi * d ** 4 / 64; J = 2 * I
kb = 2 * I / (math.pi * Dm * Na * (1 / E + 1 / (2 * G)))            # N.mm/rad, bending (Castigliano, one coil = wire loop under M about a diameter)
kt = E * I / (math.pi * Dm * Na)                                    # N.mm/rad, torsion about the spring axis (wire bends)
ka = G * d ** 4 / (8 * Dm ** 3 * Na)                                # N/mm axial (open coil)
PITCH = 1.55; CORE_H = NT * PITCH; L0_over_D = CORE_H / Dm
Mpay = 0.5 * 9.81 * 2 * 0.200; Mhand = 0.5 * 9.81 * 2 * 0.100; M = Mpay + Mhand      # N.m about the core, 2 g
rt, rc, Lt = 22.0, 6.0, 70.0                                         # tendon radius, capstan radius, tendon free length (mm)
T0 = 30.0                                                            # preload per tendon (N)
Tload = M * 1e3 / rt + T0; Ttot = 3 * T0 + Tload; Tservo = (Tload - T0) * rc / 1e3
EA = 100e3 * 0.5; k_t = EA / Lt                                      # Dyneema ~100 GPa x 0.5 mm2 (d 0.8)
th = math.radians(30); sig_b = (kb * th) * (d / 2) / I / 1  # per-coil moment ~ total M (series)
Mc = kb * th; sig_bend = Mc * (d / 2) / I; tau = Mc * (d / 2) / J; vm = math.sqrt(sig_bend ** 2 + 3 * tau ** 2)
hertz_w = Ttot / (math.pi * Dm); Rs = d / 4; Es = E / (2 * (1 - nu ** 2)); p0 = math.sqrt(hertz_w * Es / (math.pi * Rs))
stretch = Tload / k_t; ang_defl = stretch / rt; Fs = 0.5 * 9.81 * 2 + 0.5 * 9.81 * 2   # lateral load N (hand+payload, 2 g)
k_lat = (4 * T0 + Tload - T0) / CORE_H; tip_lat = Fs / k_lat; tip = ang_defl * 200 + tip_lat
Ktw = (4 * T0 + Tload - T0) * rt ** 2 / CORE_H                       # N.mm/rad tendon geometric twist stiffness
res = dict(spring=dict(d=d, OD=OD, mean_D=Dm, active_coils=Na, total_coils=NT, pitch_assembled=PITCH, core_height=CORE_H, C_index=round(Dm / d, 1)),
 k_bend_Nm_per_deg=round(kb / 1e3 * math.pi / 180, 6), k_torsion_spring_Nm_per_deg=round(kt / 1e3 * math.pi / 180, 6), k_axial_open_N_per_mm=round(ka, 3),
 axial_note="open coil carries ~0.7 N/mm: useless as a column. Tendon + payload compression (%.0f N) is carried by COIL BIND (close-wound, solid-height contact)" % Ttot,
 buckling="L0/Dm = %.2f < 2.63 (Wahl squat limit, pinned) -> unconditionally stable; solid-wound anyway" % L0_over_D,
 loads=dict(M_Nm_2g=round(M, 2), tendon_load_side_N=round(Tload, 0), tendon_total_axial_compression_N=round(Ttot, 0), servo_capstan_torque_Nm=round(Tservo + T0 * rc / 1e3, 2), servo_SF_vs_2p94=round(2.94 / (Tservo + T0 * rc / 1e3), 2)),
 tendon=dict(material="Dyneema 0.8 mm (break ~500 N class, vendor PLACEHOLDER)", SF_break=round(500 / Tload, 1), stiffness_N_per_mm=round(k_t, 0), capstan_turn_for_30deg_deg=round(math.degrees(rt * th / rc), 0)),
 wire_stress_30deg=dict(bending_MPa=round(sig_bend, 0), torsion_MPa=round(tau, 0), vonMises_MPa=round(vm, 0), music_wire_yield_MPa=1500),
 contact_Hertz_p0_MPa=round(p0, 0),
 tip_deflection_2g=dict(tendon_stretch_mm=round(stretch, 2), angle_deg=round(math.degrees(ang_defl), 2), lateral_shear_mm=round(tip_lat, 1), at_200mm_tip_mm=round(tip, 1), lateral_stiffness_N_per_mm=round(k_lat, 1)),
 twist=dict(spring_alone_Nm_per_deg=round(kt / 1e3 * math.pi / 180, 6), tendon_geometric_Nm_per_deg=round(Ktw / 1e3 * math.pi / 180, 3), deg_at_0p35Nm=round(0.35e3 / Ktw * 180 / math.pi, 1)))
# ---------------- geometry
ZF0, ZF1 = 489.0, 491.0; ZA0 = ZF1 + CORE_H; ZA1 = ZA0 + 2.0; SH = ZA1 - 495.8                       # plane F on the W3 servo top (888.6+0.4), plane A' above the core
def box(x0, x1, y0, y1, z0, z1): return cq.Workplane("XY").box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), centered=False).translate((min(x0, x1), min(y0, y1), min(z0, z1)))
def cyl(r, z0, z1, cx=0.0, cy=D): return cq.Workplane("XY").circle(r).extrude(z1 - z0).translate((cx, cy, z0))
def tube(ro, ri, z0, z1, cx=0.0, cy=D): return cyl(ro, z0, z1, cx, cy).cut(cyl(ri, z0 - 1, z1 + 1, cx, cy))
P = {}
P["plane_F"] = box(-26.8, 26.8, D - 30, D + 30, ZF0, ZF1).cut(cyl(8.0, ZF0 - 1, ZF1 + 1))
P["plane_A2"] = box(-38.0, 26.8, D - 26, D + 26, ZA0, ZA1).union(box(-80.5, -38.0, D - 15, D + 15, ZA0, ZA1)).cut(cyl(8.0, ZA0 - 1, ZA1 + 1)).cut(cyl(5.0, ZA0 - 1, ZA1 + 1, -42.5, D))
P["hub"] = cyl(20.0, ZA1, ZA1 + 6.6).cut(cyl(8.0, ZA1 - 1, ZA1 + 8))
# helix spring
r_mid = Dm / 2
path = cq.Wire.makeHelix(PITCH, CORE_H - d, r_mid).translate(cq.Vector(0, 0, d / 2))
spring = cq.Workplane("XZ").center(r_mid, d / 2).circle(d / 2).sweep(cq.Workplane(obj=path), isFrenet=True).translate((0, D, ZF1))
P["coil_spring"] = spring
P["ptfe_liner"] = tube(7.0, 5.5, ZF0 - 1.0, ZA1 + 1.0)
# curved rim plates: 70-deg arcs, R 27 x 1.5 thick x 4 tall, on F (anchors/fairleads) and A' (fairleads), at 0/90/180/270
def arc(r0, r1, a0, a1, z0, z1):
    n = 24; pts = [(r1 * math.cos(math.radians(a0 + (a1 - a0) * i / n)), r1 * math.sin(math.radians(a0 + (a1 - a0) * i / n))) for i in range(n + 1)] + [(r0 * math.cos(math.radians(a1 - (a1 - a0) * i / n)), r0 * math.sin(math.radians(a1 - (a1 - a0) * i / n))) for i in range(n + 1)]
    return cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).translate((0, D, z0))
rimF = None; rimA = None
for a in (0, 90, 180, 270):
    f = arc(25.5, 27.0, a - 35, a + 35, ZF1, ZF1 + 4.0); g = arc(25.5, 27.0, a - 35, a + 35, ZA0 - 4.0, ZA0)
    for ang in (a,):
        hole = cq.Workplane("XY").circle(0.8).extrude(10).translate((0, 0, -5)).translate((rt * math.cos(math.radians(ang)), D + rt * math.sin(math.radians(ang)), 0))
    rimF = f if rimF is None else rimF.union(f); rimA = g if rimA is None else rimA.union(g)
P["rim_F"] = rimF; P["rim_A2"] = rimA
tend = None
for a in (0, 90, 180, 270):
    x = rt * math.cos(math.radians(a)); y = D + rt * math.sin(math.radians(a)); t = cyl(0.4, ZF1 - 2.0, ZA0, x, y); tend = t if tend is None else tend.union(t)
P["tendons"] = tend
P["capstan"] = cq.Workplane("YZ").circle(rc).extrude(8.0).translate((29.0, D, ZW3))
def exp(w, n):
    p = os.path.join(HERE, n + ".stl"); cq.exporters.export(w, p, tolerance=0.03, angularTolerance=0.1); return p
meta = {}
for k, v in P.items():
    exp(v, k); m = trimesh.load(os.path.join(HERE, k + ".stl")); meta[k] = dict(watertight=bool(m.is_watertight), vol_cm3=round(float(abs(m.volume)) / 1e3, 3))
res["parts"] = meta; res["layout"] = dict(plane_F=[ZF0, ZF1], core=[ZF1, ZA0], plane_A2=[ZA0, ZA1], hand_lift_extra_mm=round(SH, 1), reach_open_mm=round(219 + 24 + SH, 1), virtual_pivot_z=round(ZF1 + CORE_H / 2, 1),
    ROM_note="+-30 deg: outer-side gap opening %.1f mm over %d coils (%.2f mm per coil), inner side stays at bind" % (math.sin(th) * OD / 2 * 1.0 + 0, Na, math.sin(th) * OD / 2 / Na))
json.dump(res, open(os.path.join(HERE, "concept.json"), "w"), indent=1)
# ---------------- section render
items = [("forearm", L("distal/forearm_dist_R.stl"), (.55, .62, .72)), ("yoke", L("distal/w2_yoke_R.stl"), (.40, .5, .66)), ("sts_w3", L("distal/proxies/sts_w3_R.stl"), (.25, .25, .28))]
for k, c in (("plane_F", (.4, .5, .66)), ("plane_A2", (.95, .55, .15)), ("hub", (.95, .55, .15)), ("coil_spring", (.15, .2, .8)), ("ptfe_liner", (.85, .85, .9)), ("rim_F", (.4, .5, .66)), ("rim_A2", (.95, .55, .15)), ("tendons", (.9, .1, .1)), ("capstan", (.85, .75, .15))):
    items.append((k, trimesh.load(os.path.join(HERE, k + ".stl")), c))
for n, c in (("rotor", (.15, .65, .6)), ("pinion", (.1, .45, .42)), ("palm_tray_wrist", (.72, .74, .8)), ("proxies/bearing_6808", (.8, .8, .85)), ("proxies/sts_R", (.25, .25, .28)), ("proxies/spindle_tube", (.85, .75, .15))):
    m = L("wrist_rotary/%s.stl" % n).copy(); m.apply_translation([0, 0, SH]); items.append((n, m, c))
def sect(plane, fn, xlim, title):
    fig, ax = plt.subplots(figsize=(11, 9), dpi=120)
    for n, m, c in items:
        try:
            s = m.section(plane_origin=[0, D, 0], plane_normal=[0, 1, 0]) if plane == "xz" else m.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
            if s is None: continue
            for e in s.discrete:
                e = np.asarray(e); pts = np.column_stack([e[:, 0], e[:, 2]]) if plane == "xz" else np.column_stack([e[:, 1], e[:, 2]])
                if len(pts) < 3: continue
                pg = Polygon(pts).buffer(0)
                for g in (pg.geoms if hasattr(pg, "geoms") else [pg]):
                    if g.is_empty: continue
                    ax.add_patch(MP(np.array(g.exterior.coords), closed=True, fc=c, ec="k", lw=0.4, zorder=2))
                    for h in g.interiors: ax.add_patch(MP(np.array(h.coords), closed=True, fc="w", ec="k", lw=0.3, zorder=3))
        except Exception as ex: print("skip", n, ex)
    ax.axhline(ZW3, color="b", lw=.7, ls="--"); ax.axvline(0 if plane == "xz" else D, color="r", lw=.7, ls="--"); ax.axhline(ZF1 + CORE_H / 2, color="g", lw=.7, ls="-."); ax.text(xlim[0] + 2, ZF1 + CORE_H / 2 + 1, "virtual pivot (core mid-height)", color="g", fontsize=8)
    ax.set_xlim(*xlim); ax.set_ylim(425, 560); ax.set_aspect("equal"); ax.grid(alpha=.2); ax.set_title(title, fontsize=10); fig.tight_layout(); fig.savefig(os.path.join(HERE, fn)); plt.close(fig)
sect("xz", "B_section_xz.png", (-95, 60), "Concept B, y = D (XZ): coil core between plane F (on W3 servo) and plane A' (R stage), tendons red")
sect("yz", "B_section_yz.png", (0, 120), "Concept B, x = 0 (YZ): close-wound coil around PTFE-lined spindle, tendons at +-y (W3 pair), curved rim plates")
print(json.dumps(res, indent=1))
