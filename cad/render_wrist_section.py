"""Cross-sections of the draft wrist: plane y = D (XZ, through the W3 axis, R servo, gear mesh) and plane x = 0 (YZ, through the W3 servo, hub, hand axis)."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly
from shapely.ops import unary_union
import hand_pose as HP, wrist_rotary as W
D = 57.0; ST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl")
def L(p): return trimesh.load(os.path.join(ST, p))
col = dict(forearm=(0.55, 0.62, 0.72), yoke=(0.40, 0.50, 0.66), servo=(0.25, 0.25, 0.28), carrier=(0.95, 0.55, 0.15), hub=(0.95, 0.55, 0.15), rotor=(0.15, 0.65, 0.60), pinion=(0.10, 0.45, 0.42),
           bearing=(0.80, 0.80, 0.85), palm=(0.72, 0.74, 0.80), digit=(0.85, 0.50, 0.25), pad=(0.15, 0.15, 0.18), pcb=(0.85, 0.75, 0.15), pogo=(0.85, 0.75, 0.15), thumb=(0.80, 0.35, 0.45))
items = [("forearm_dist", L("distal/forearm_dist_R.stl"), "forearm"), ("w2_yoke", L("distal/w2_yoke_R.stl"), "yoke"),
         ("sts_w2", L("distal/proxies/sts_w2_R.stl"), "servo"), ("sts_w3 (W3)", L("distal/proxies/sts_w3_R.stl"), "servo"),
         ("carrier", L("wrist_rotary/carrier.stl"), "carrier"), ("rotor", L("wrist_rotary/rotor.stl"), "rotor"), ("pinion", L("wrist_rotary/pinion.stl"), "pinion"),
         ("bearing 6808", L("wrist_rotary/proxies/bearing_6808.stl"), "bearing"), ("sts_R (R)", L("wrist_rotary/proxies/sts_R.stl"), "servo"),
         ("spindle tube", L("wrist_rotary/proxies/spindle_tube.stl"), "pcb"), ("flexure +x", L("wrist_rotary/flexure_px.stl"), "pogo"), ("flexure -x", L("wrist_rotary/flexure_nx.stl"), "pogo"),
         ("W3 crank", L("wrist_rotary/proxies/w3_crank.stl"), "pad"), ("W3 rod", L("wrist_rotary/proxies/w3_rod.stl"), "pad"), ("palm", L("wrist_rotary/palm_tray_wrist.stl"), "palm")]
P = HP.make_poses(); scene, _ = HP.build("open", P["open"]); sc = HP.to_robot(scene)
for n, m in sc:
    m = m.copy(); m.apply_translation([0, 0, W.DZ]); items.append((n, m, "pad" if n.endswith("pad") else ("thumb" if n.startswith("thumb") else "digit")))
XCUT = 0.0
def section(plane):
    out = []
    for n, m, c in items:
        try:
            if plane == "xz": s = m.section(plane_origin=[0, D, 0], plane_normal=[0, 1, 0]); T = lambda v: (v[:, 0], v[:, 2])
            else: s = m.section(plane_origin=[XCUT, 0, 0], plane_normal=[1, 0, 0]); T = lambda v: (v[:, 1], v[:, 2])
            if s is None: continue
            for e in s.discrete:
                a, b = T(np.asarray(e)); out.append((n, c, np.column_stack([a, b])))
        except Exception as ex: print("skip", n, ex)
    return out
def draw(plane, xlim, zlim, fn, title, notes, xcut=0.0):
    global XCUT; XCUT = xcut
    segs = section(plane); fig, ax = plt.subplots(figsize=(13, 9.5), dpi=130)
    from shapely.geometry import Polygon
    for n, c, pts in segs:
        if len(pts) < 3: continue
        poly = Polygon(pts)
        if not poly.is_valid: poly = poly.buffer(0)
        if poly.is_empty: continue
        for g in (poly.geoms if hasattr(poly, "geoms") else [poly]):
            ax.add_patch(MPoly(np.array(g.exterior.coords), closed=True, facecolor=col[c], edgecolor="k", linewidth=0.5, alpha=0.95, zorder=2))
            for hole in g.interiors: ax.add_patch(MPoly(np.array(hole.coords), closed=True, facecolor="white", edgecolor="k", linewidth=0.4, zorder=3))
    ax.axvline(0 if plane == "xz" else D, color="r", lw=0.7, ls="--", zorder=1); ax.axhline(455, color="b", lw=0.7, ls="--", zorder=1)
    ax.text(xlim[0] + 2, 456.5, "W3 axis (z 455)", color="b", fontsize=8); ax.text((0 if plane == "xz" else D) + 1.5, zlim[1] - 6, "R / hand axis", color="r", fontsize=8)
    ax.axhline(W.MATE, color="g", lw=1.0, ls="-.", zorder=1); ax.text(xlim[1] - 2, W.MATE + 1.2, f"MATING PLANE z {W.MATE:.1f}", color="g", fontsize=8, ha="right")
    for (x, z, t) in notes: ax.annotate(t, xy=(x[0], x[1]), xytext=(x[2], x[3]), fontsize=8.5, arrowprops=dict(arrowstyle="->", lw=0.7), zorder=6, bbox=dict(boxstyle="round,pad=0.2", fc="w", ec="0.6", alpha=0.9))
    ax.set_xlim(*xlim); ax.set_ylim(*zlim); ax.set_aspect("equal"); ax.set_xlabel("x (mm)" if plane == "xz" else "y (mm)"); ax.set_ylabel("z (mm)"); ax.set_title(title); ax.grid(alpha=0.2)
    fig.tight_layout(); fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", fn)); plt.close(fig)
if __name__ == "__main__":
    draw("xz", (-60, 62), (425, 560), "wrist_section_xz.png", "Unified wrist v3, section y = D (XZ): monolithic flexures at both sides of the W3 servo, crank + rod, hand base",
         [((34, 465, 44, 500), 0, "PA12 cross-strip flexure (+x)"), ((-34, 465, -52, 500), 0, "flexure (-x)"), ((21.5, 475, 8, 531), 0, "crank + push-rod to plate A lug"),
          ((0, 520, -30, 548), 0, "hand floor / palm"), ((0, 465, -8, 440), 0, "W3 servo (fixed)"), ((0, 499, -50, 520), 0, "rotor: 52T band + web + hollow neck"), ((0, 508, 20, 548), 0, "spindle tube = wire conduit")])
    draw("yz", (0, 120), (425, 560), "wrist_section_yz.png", "Unified wrist v3, section x = 0 (YZ): R servo + pinion at -y, hollow spindle, W3 servo",
         [((D, 505, 85, 548), 0, "spindle tube (wires inside)"), ((D - 42.5, 475, 2, 500), 0, "R servo STS3215 (-y side)"), ((D, 470, 82, 455), 0, "W3 servo"), ((D, 440, 90, 432), 0, "W2 yoke")])
    draw("yz", (25, 95), (435, 500), "wrist_section_flexure.png", "Unified wrist v3, section x = +35 (YZ): crossed PA12 leaves 0.7 x 24 x 20 on the W3 axis",
         [((D, 455, 80, 475), 0, "two 0.7 mm leaves at +-45 deg"), ((D, 447, 30, 440), 0, "base bar + bracket to the W2 cradle"), ((D, 485, 85, 497), 0, "top pad under plate A")], xcut=35.0)
    # 3D views of the unified wrist (open hand)
    import render_hand as RH, glob
    ms = []
    for n, m, c in items:
        m = m.copy()
        if len(m.faces) > 2500:
            try: m = m.simplify_quadric_decimation(face_count=2500)
            except Exception: pass
        base = {"forearm": (150, 165, 190), "yoke": (120, 135, 165), "servo": (70, 70, 78), "carrier": (240, 140, 40), "rotor": (30, 170, 160), "pinion": (20, 120, 110), "bearing": (205, 205, 215),
                "palm": (190, 195, 210), "digit": (215, 125, 65), "pad": (45, 45, 50), "pcb": (220, 190, 40), "pogo": (200, 80, 120), "thumb": (200, 90, 110)}[c]
        ms.append((n, m, np.tile(np.array([*base, 255.0]), (len(m.faces), 1))))
    import matplotlib.pyplot as plt2
    fig = plt2.figure(figsize=(16, 7), dpi=110)
    for j, (el, az, t) in enumerate([(22, -48, "3/4 view"), (8, -118, "rear-side view")]):
        ax = fig.add_subplot(1, 2, j + 1, projection="3d"); RH.draw(ax, ms, el, az, t, dict(c=[10, 57, 500], r=95))
    fig.suptitle("Unified wrist v3: carrier plate (orange) on two PA12 flexures (yellow/pink) + crank, R stage (teal), hand lifted 24 mm", fontsize=11); fig.tight_layout()
    fig.savefig(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "wrist_v3_3d.png")); plt2.close(fig)
