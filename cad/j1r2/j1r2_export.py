"""Export J1 rev 2 at the chosen pad position, posed at park, plus the distal arm proxies, as concept 'J1R2' for rendering + metrics."""
import sys, os, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, j1r2_params as P, j1r2_geom as G, j1r2_kin as K
y1, z1, so = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (10.0, -30.0, 22.0)
parts, info = G.build(y1=y1, z1=z1, standoff=so, full=True); D = info["derived"]
posed = G.pose(parts, P.PARK["S1p"], P.PARK["S2"], D); out = {}
GM = {"F": "joint", "M1": "joint", "M2": "joint", "ARM": "arm"}
for k, p in parts.items():
    if k.startswith("_"): continue
    kind = p.get("kind", ""); grp = "bought" if kind == "bought" else "armour" if kind in ("armour", "fairing") else GM.get(p["group"], "joint")
    mat = p.get("mat", "PACF"); mat = mat if mat in L.DENS else "TPU"
    d = dict(mesh=posed[k], mat=mat, group=grp, colour=p.get("colour", L.COL["pacf"]))
    if p.get("mass_g") is not None: d["mass_g"] = p["mass_g"]
    out[k] = d
kin = K.Kin(D); q = dict(P.PARK); T = kin.group_transforms(q)
for g, ms in kin.proxies().items():
    if g == "M3": continue
    for nm, m in ms.items():
        mm = m.copy(); mm.apply_transform(T[g]); out["dist_" + nm] = dict(mesh=mm, mat="PETG", group="arm", colour=L.COL["stage"])
meta = L.export_concept("J1R2", "J1 rev 2 integrated (pad y=%g z=%g)" % (y1, z1), out, extra=dict(own_stage=True, derived={k: (float(v)) for k, v in D.items()}, kinds=sorted({p.get("kind", "") for p in parts.values()})))
print(meta["mount_mass_g"], [(r["name"], r["group"]) for r in meta["parts"]][:12], meta["extra"]["kinds"])
