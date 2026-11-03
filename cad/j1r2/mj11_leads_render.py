"""Renders of the MJ11 electrode relief slots / lead corridors / jack plate with the body (2026-11-03).
Skin-side views look from inside the torso (-x) through a window cut in the back-skin patch over the liner, so the skin face of the liner (slots) is visible.
usage: J1R2_ROOT=<root> python3 mj11_leads_render.py [cache_dir]  -> renders_j1r2/mounts/MJ11_leads_<view>.png + _sheet.png"""
import os, sys, json, math, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
from shapely.geometry import Point
import clib as L, mj_frame as M, mj_concepts3 as K, mj11_leads as LD
REPO = M.REPO; MD = os.path.join(REPO, "stl", "j1r2_mounts", "MJ11"); OUTD = os.path.join(REPO, "renders_j1r2", "mounts"); os.makedirs(OUTD, exist_ok=True)
CACHE = sys.argv[1] if len(sys.argv) > 1 else "/tmp/mj11_leads_cache"; os.makedirs(CACHE, exist_ok=True)
def save(name, m, color, metal=0.0, rough=0.6):
    p = os.path.join(CACHE, name + ".stl"); m.export(p); return dict(stl=p, color=color, metal=metal, rough=rough)
b = L.body(); c = b.triangles_center; dom = LD.domain_poly().buffer(3.0)
keep = (c[:, 2] > -430) & (c[:, 2] < 60) & (c[:, 1] > -200) & (c[:, 1] < 110) & (c[:, 0] > -200) & (c[:, 0] < -60)
inwin = np.array([dom.contains(Point(y, z)) for y, z in c[:, 1:]])
hole = keep & inwin & (c[:, 0] > -150)
parts = [save("body_window", b.submesh([np.where(keep & ~hole)[0]], append=True), [214, 182, 160], rough=0.85)]
w = trimesh.load(os.path.join(REPO, "stl", "shoulder", "scapular", "wing_R.stl")); w = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100)); parts.append(save("wing_rail", w, [70, 74, 82], rough=0.6))
for k, col, mt in (("liner", [225, 228, 235], 0.0), ("seat_dish", [40, 60, 110], 0.1), ("jack_plate", [230, 140, 30], 0.3), ("rail_spacers", [150, 154, 160], 0.5)):
    parts.append(save("mj11_" + k, trimesh.load(os.path.join(MD, f"MJ11_{k}_R.stl")), col, metal=mt))
Y_, Z_, skin_, inner_, plane_ = K.seat_surfaces()
for k_, poly_ in LD.slot_polys().items():
    for g_ in (poly_.geoms if poly_.geom_type == "MultiPolygon" else [poly_]):
        fl = L.I(L.height_solid(Y_, Z_, skin_ + LD.RELIEF_DEPTH - 0.4, skin_ + LD.RELIEF_DEPTH + 0.1, "x"), L.prism([(float(a), float(b_)) for a, b_ in g_.exterior.coords], "x", K.X0 - 90.0, K.X0 + 5.0))
        parts.append(save("slotfloor_" + k_, fl, [255, 190, 60]))
def tube(pts, d=2.4):
    segs = [trimesh.creation.cylinder(radius=d / 2, segment=[pts[i], pts[i + 1]], sections=10) for i in range(len(pts) - 1)]
    return trimesh.util.concatenate(segs)
def xs(y, z): return float(M.back_x(y, z))
ELEC = {"infraspin": [20, 140, 120], "teres_maj": [200, 60, 40], "rhomboid": [150, 60, 170]}
disc = lambda y, z, h=2.0: L.cyl("x", LD.DISC_R, xs(y, z) - 0.5, xs(y, z) + h, c=(y, z), n=32)
for k, col in ELEC.items():
    a, bb, ctr, s = LD.pair(k); d = LD.dvec(*ctr); tr = s["travel_mm"] * LD.DESIGN_X
    parts.append(save("elec_" + k, trimesh.util.concatenate([disc(*a), disc(*bb)]), col))
    parts.append(save("elec_end_" + k, trimesh.util.concatenate([disc(*(a + d * tr), 0.8), disc(*(bb + d * tr), 0.8)]), [int(0.5 * v + 128) for v in col]))
    if k in ("infraspin", "teres_maj"):
        P_, _ = LD.lead_path(k); pts = list(P_)
        pts3 = [np.array([xs(y, z) + 2.0, y, z]) for y, z in pts]
        parts.append(save("lead_" + k, tube(pts3), [20, 20, 20]))
mid = np.array([K.Y1 - 20.0, K.Z1 - 15.0])
views_skin = [dict(name="skin_front", dir=[-1.0, 0.0, 0.0], label="skin side, from inside the torso (slots, discs, leads)"),
              dict(name="skin_up", dir=[-1.0, -0.55, 0.45], label="skin side, oblique medial-high"),
              dict(name="skin_low", dir=[-1.0, 0.55, -0.45], label="skin side, oblique lateral-low")]
sc = dict(out_prefix=os.path.join(OUTD, "MJ11_leads"), title="MJ11 liner: electrode relief slots, lead corridors, jack plate (orange); discs: infraspinatus green, teres major red, rhomboid purple; pale = end of travel", width=900, height=900,
          parts=parts, up=[0, 0, 1], time_budget=int(os.environ.get("TB", 60)), fill=0.85, views=views_skin)
js = os.path.join(CACHE, "scene_skin.json"); json.dump(sc, open(js, "w")); subprocess.run([sys.executable, os.path.join(REPO, "cad", "render_three.py"), js], check=True)
