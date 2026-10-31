"""Cadquery model of the best concept C geometry (cross-strip, PA12 MJF/SLS): monolithic base bar + 2 crossed leaves (0.7 x 24 x 20) + top bar + strut + top pad.  Plus crank/push-rod proxies and section PNGs."""
import cadquery as cq, trimesh, numpy as np, os, json, math
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MPoly
from shapely.geometry import Polygon
HERE = os.path.dirname(os.path.abspath(__file__)); ST = os.path.join(HERE, "..", "..", "stl"); D, ZW3 = 57.0, 455.0
L, T, B, X0 = 24.0, 0.7, 20.0, 24.0                                  # leaf length, thickness, width along x; flexure x start
def box(x0, x1, y0, y1, z0, z1): return cq.Workplane("XY").box(abs(x1 - x0), abs(y1 - y0), abs(z1 - z0), centered=False).translate((min(x0, x1), min(y0, y1), min(z0, z1)))
a = L / 2 / math.sqrt(2)                                              # leaf end offset from the axis (8.49)
fl = box(X0, X0 + B, D - 13, D + 13, ZW3 - a - 4.0, ZW3 - a + 0.5)    # base bar
fl = fl.union(box(20.2, X0, D - 12, D + 12, ZW3 - a - 4.0, ZW3 - a - 0.5))                 # bracket to the W2 cradle face x 20.2 (2 x M3, PLACEHOLDER)
fl = fl.union(box(X0, X0 + B, D - 13, D + 13, ZW3 + a - 0.5, ZW3 + a + 4.0))               # top (moving) bar
for s in (1, -1):
    leaf = cq.Workplane("XY").box(B, L + 1.0, T, centered=(False, True, True)).translate((X0, 0, 0)).rotate((0, 0, 0), (1, 0, 0), s * 45.0 + 0.0).translate((0, D, ZW3))
    fl = fl.union(leaf)
fl = fl.union(box(X0, X0 + B, D - 7, D + 7, ZW3 + a + 3.5, 489.8)).union(box(X0 - 2, X0 + B + 2, D - 16, D + 16, 489.8, 493.8))     # strut + top pad (plane A bolts on top)
for yy in (-10, 10):
    for xx in (X0 + 2, X0 + B - 2): fl = fl.cut(cq.Workplane("XY").circle(1.7).extrude(10).translate((xx, D + yy, 484.8)))        # 4 x M3 clearance
cq.exporters.export(fl, os.path.join(HERE, "flexure_mono.stl"), tolerance=0.02)
m = trimesh.load(os.path.join(HERE, "flexure_mono.stl")); sol = fl.val()
info = dict(watertight=bool(m.is_watertight), valid=bool(sol.isValid()), volume_cm3=round(sol.Volume() / 1e3, 3), mass_g_PA12_1p01=round(sol.Volume() / 1e3 * 1.01, 1), bbox=[round(sol.BoundingBox().xlen, 1), round(sol.BoundingBox().ylen, 1), round(sol.BoundingBox().zlen, 1)])
# context + proxies
ctx = {"W3 servo": trimesh.load(os.path.join(ST, "distal/proxies/sts_w3_R.stl")), "W2 yoke / cradle": trimesh.load(os.path.join(ST, "distal/w2_yoke_R.stl")), "flexure_mono": m}
crank = trimesh.creation.box([4.0, 6.0, 15.0]); crank.apply_translation([19.5, D, ZW3 + 7.5 + 7.5 - 7.5 + 0.0]); rod = trimesh.creation.cylinder(radius=1.5, height=24.0, sections=16); rod.apply_translation([22.0, D, 482.0])
ctx["crank r15"] = crank; ctx["push-rod d3"] = rod
colors = {"W3 servo": (0.25, 0.25, 0.28), "W2 yoke / cradle": (0.40, 0.50, 0.66), "flexure_mono": (0.95, 0.55, 0.15), "crank r15": (0.1, 0.6, 0.3), "push-rod d3": (0.1, 0.6, 0.3)}
def sect(ax, plane, xlim, zlim, title):
    for n, mm in ctx.items():
        s = mm.section(plane_origin=[34.0 if plane == "yz" else 0, D if plane == "xz" else 0, 0], plane_normal=[1, 0, 0] if plane == "yz" else [0, 1, 0])
        if s is None: continue
        for e in s.discrete:
            e = np.asarray(e); p = np.column_stack([e[:, 1], e[:, 2]]) if plane == "yz" else np.column_stack([e[:, 0], e[:, 2]])
            if len(p) < 3: continue
            g = Polygon(p).buffer(0)
            for gg in (g.geoms if hasattr(g, "geoms") else [g]):
                if gg.is_empty: continue
                ax.add_patch(MPoly(np.array(gg.exterior.coords), closed=True, facecolor=colors[n], edgecolor="k", linewidth=0.4))
                for hh in gg.interiors: ax.add_patch(MPoly(np.array(hh.coords), closed=True, facecolor="white", edgecolor="k", linewidth=0.4))
    ax.axhline(ZW3, color="b", lw=0.6, ls="--"); ax.set_xlim(*xlim); ax.set_ylim(*zlim); ax.set_aspect("equal"); ax.set_title(title, fontsize=9); ax.grid(alpha=0.2)
fig, axs = plt.subplots(1, 2, figsize=(14, 7), dpi=120)
sect(axs[0], "yz", (25, 90), (435, 500), "Section x = 34 (YZ): crossed PA12 leaves 0.7 x 24 x 20 on the W3 axis (blue dashed)")
sect(axs[1], "xz", (-30, 60), (435, 500), "Section y = D (XZ): flexure at +x, crank r15 + push-rod from the W3 horn")
fig.tight_layout(); fig.savefig(os.path.join(HERE, "section_c.png")); json.dump(info, open(os.path.join(HERE, "model_info.json"), "w"), indent=1); print(info)
