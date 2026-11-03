"""Renders of MJ11 with the option (a) balancer concept: spring tube on the lower back (y -126, x -84, z -300..-491), cord A -> B at park.  PLACEHOLDER geometry (tube proxy, cord route not designed)."""
import os, sys, json, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import mj_render as R, mj11_balancer as MB, j1r2_params as P, clib as L
d = json.load(open(os.path.join(MB.OUTD, "balancer_cordfix.json"))); r = [x for x in d if x["psi"] == 120][0]; x = np.array(r["x"]); A = MB.P1 + x[0:3]; Bl = x[3:6]
F0, FA, s, a, dd = P.shoulder_frames(MB.B.D0, P.PARK["S1p"], P.PARK["S2"]); Bw = (FA @ np.r_[Bl, 1.0])[:3]
CD = os.path.join(R.MD, "_bal_render"); os.makedirs(CD, exist_ok=True)
def seg(p0, p1, rad):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float); v = p1 - p0; h = np.linalg.norm(v); m = trimesh.creation.cylinder(rad, h, sections=24)
    z = np.array([0, 0, 1.0]); ax = np.cross(z, v / h); an = np.arccos(np.clip(z @ (v / h), -1, 1))
    if np.linalg.norm(ax) > 1e-9: m.apply_transform(trimesh.transformations.rotation_matrix(an, ax))
    m.apply_translation((p0 + p1) / 2); return m
parts = list(R.shared()); MJ = os.path.join(R.MD, "MJ11")
met = json.load(open(os.path.join(MJ, "metrics.json")))
for k, rr in met["parts"].items(): parts.append(dict(stl=os.path.join(MJ, f"MJ11_{k}_R.stl"), color=R.rgb(met["colours"].get(k, [0.2, 0.34, 0.52])), metal=0.05, rough=0.5))
tube_top = np.array([-84.0, -126.0, -300.0]); tube_bot = tube_top + np.array([0, 0, -191.0])
items = {"bal_tube": (seg(tube_top, tube_bot, 22.25), [235, 120, 30]), "bal_cord_up": (seg(A, tube_top, 1.5), [220, 40, 40]), "bal_cord_arm": (seg(A, Bw, 1.5), [220, 40, 40]),
         "bal_sheave_A": (trimesh.creation.icosphere(radius=7), [235, 120, 30]), "bal_pin_B": (trimesh.creation.icosphere(radius=6), [235, 120, 30])}
items["bal_sheave_A"][0].apply_translation(A); items["bal_pin_B"][0].apply_translation(Bw)
for k, (m, c) in items.items(): p = os.path.join(CD, k + ".stl"); m.export(p); parts.append(dict(stl=p, color=c, metal=0.2, rough=0.5))
sc = dict(out_prefix=os.path.join(R.REPO, "renders_j1r2", "mj11_balancer_a"), title="MJ11 + option (a) balancer concept (tube on the lower back, cord redirected; PLACEHOLDER)", width=820, height=820, parts=parts, up=[0, 0, 1], time_budget=int(os.environ.get("TB", 50)), fill=0.86,
          views=[dict(name="rear34", dir=[1.0, 0.70, 0.55], label="rear-right 3/4"), dict(name="rear", dir=[1.0, -0.12, 0.18], label="from behind"), dict(name="side", dir=[0.15, 1.0, 0.10], label="from the right"), dict(name="low", dir=[0.8, 0.3, -0.5], label="from below-behind")])
js = os.path.join(CD, "scene.json"); json.dump(sc, open(js, "w")); subprocess.run([sys.executable, os.path.join(R.REPO, "cad", "render_three.py"), js], check=True)
