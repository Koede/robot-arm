"""Render MJ11 with the rev 2 pad-spring balancer hardware placed on the lower back (candidate home: tube straight up beside the spine, psi 95 deg, 3:1 tackle 175 mm).
usage: J1R2_ROOT=/home/claude/ts python3 mj11_balancer_render.py -> renders_j1r2/mounts/MJ11_balancer_*.png"""
import os, sys, json, math, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import mj_render as R, mj_concepts as C, j1r2_params as P
DER = R.DER; P1 = np.array([DER["PX"], DER["y1"], DER["z1"]]); A = P1 + np.array([-10.018, -19.708, 73.434]); psi, Lt = 95.0, 175.0
u = np.array([0.0, math.cos(math.radians(psi)), math.sin(math.radians(psi))])
cap = C.tubebeam(A - u * 8.0, A, 56.0, None); tube = C.tubebeam(A + u * 6.0, A + u * (6.0 + Lt), 44.5, 2.5); post = C.tubebeam(np.array([DER["XB"], A[1], A[2]]), A - np.array([14.0, 0, 0]), 26.0, None)
d = os.path.join(R.CACHE, "bal"); os.makedirs(d, exist_ok=True); parts = list(R.scene.__globals__["shared"]())
for nm, m, col in (("tube", tube, [226, 120, 30]), ("cap", cap, [226, 120, 30]), ("post", post, [226, 120, 30])):
    p_ = os.path.join(d, nm + ".stl"); m.export(p_); parts.append(dict(stl=p_, color=col, rough=0.5))
met = json.load(open(os.path.join(R.MD, "MJ11", "metrics.json"))); cols = met.get("colours", {})
for k, r in met["parts"].items():
    p_ = os.path.join(R.MD, "MJ11", f"MJ11_{k}_R.stl"); parts.append(dict(stl=p_, color=R.rgb(cols.get(k, [0.2, 0.34, 0.52])), metal=0.05, rough=0.5))
sc = dict(out_prefix=os.path.join(R.OUTD, "MJ11_balancer"), title="MJ11 + rev 2 pad-spring balancer hardware (orange): tube up beside the spine, 3:1 tackle 175 mm - neck extension contact at 32 deg", width=820, height=820, parts=parts, up=[0, 0, 1],
          time_budget=int(os.environ.get("TB", 50)), fill=0.86, views=[dict(name="rear34", dir=[1.0, 0.70, 0.55], label="rear-right 3/4"), dict(name="rear", dir=[1.0, -0.12, 0.18], label="from behind"),
          dict(name="side", dir=[0.15, 1.0, 0.10], label="from the right"), dict(name="top", dir=[0.30, 0.10, 1.0], label="from above")])
js = os.path.join(R.CACHE, "scene_bal.json"); json.dump(sc, open(js, "w")); subprocess.run([sys.executable, os.path.join(R.REPO, "cad", "render_three.py"), js], check=True)
