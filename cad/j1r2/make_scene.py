"""Export the shared render assets (wearer, stage proxy, wing/rail proxy, seat markers) to render/assets and write render/scene.json."""
import sys, os, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh, clib as L
A = os.path.join(L.ROOT, "render", "assets"); os.makedirs(A, exist_ok=True); sc = {"shared": []}
b = L.body(); keep = (b.triangles_center[:, 2] > -700) & (b.triangles_center[:, 1] < 260); sub = b.submesh([np.where(keep)[0]], append=True); sub.export(os.path.join(A, "body.stl"))
sc["shared"].append(dict(file="assets/body.stl", colour=list(L.COL["body"]), kind="body", smooth=True))
for k, (m, c, g) in L.stage_parts(arm=False).items():
    m.export(os.path.join(A, f"stage_{k}.stl")); sc["shared"].append(dict(file=f"assets/stage_{k}.stl", colour=list(c), kind="stage", smooth=True))
for k, s in L.env_parts().items():
    s["mesh"].export(os.path.join(A, f"env_{k}.stl")); sc["shared"].append(dict(file=f"assets/env_{k}.stl", colour=[0.30, 0.34, 0.42], kind="env", smooth=False))
for k, (p, n) in L.seats().items():
    m = L.cyl("z", 12, p[2] + 0.0, p[2] + 5.0, c=(p[0], p[1])); m.export(os.path.join(A, f"seat_{k}.stl")); sc["shared"].append(dict(file=f"assets/seat_{k}.stl", colour=[0.95, 0.75, 0.15], kind="seat", smooth=False))
json.dump(sc, open(os.path.join(L.ROOT, "render", "scene.json"), "w"), indent=1); print(len(sc["shared"]), "shared assets")
