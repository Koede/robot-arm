"""J1 rev 2 integration: geometry (j1r2_geom) + kinematics (j1r2_kin) + sweep (j1r2_sweep).  usage: j1r2_integrate.py [shoulder|full] y1 z1 standoff label"""
import sys, os, json, time; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, j1r2_params as P, j1r2_geom as G, j1r2_kin as K, j1r2_sweep as S
def make_scene(y1=-70.0, z1=-35.0, standoff=22.0, joints_upto="S3", gkw=None, name="J1r2", drop=("column_anchor",)):
    parts, info = G.build(y1=y1, z1=z1, standoff=standoff, full=True, **(gkw or {})); D = info["derived"]
    fixed, m1, m2 = {}, [], []
    for k, p in parts.items():
        g = p["group"]
        if p.get("kind") == "cable" or k.startswith("cable") or k in drop: continue          # cable proxies are checked separately (bend radius), not collision bodies
        if g == "F": fixed[k] = p["mesh"]
        elif g == "M1": m1.append(p["mesh"])
        elif g == "M2": m2.append(p["mesh"])
    env = L.env_parts(); fixed["wing"] = env["wing"]["mesh"]
    sh = dict(M1=L.U(*m1), M2=L.U(*m2)); wearer = S.make_wearer_parts(); kin = K.Kin(D)
    return S.scene_from_kin(sh, fixed, wearer, kin=kin, name=name, joints_upto=joints_upto), parts, info
if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "shoulder"; y1 = float(sys.argv[2]) if len(sys.argv) > 2 else -70.0; z1 = float(sys.argv[3]) if len(sys.argv) > 3 else -35.0
    so = float(sys.argv[4]) if len(sys.argv) > 4 else 22.0; label = sys.argv[5] if len(sys.argv) > 5 else f"j1r2_{mode}"
    t0 = time.time(); sc, parts, info = make_scene(y1, z1, so, "S3" if mode == "shoulder" else "W3"); print("scene", round(time.time() - t0, 1), "s", list(sc.fixed), list(sc.groups))
    steps = {"S1p": 2.5, "S2": 2.5, "S3": [-70.0, 0.0, 90.0]} if mode == "shoulder" else {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}
    out = S.run_sweep(sc, dict(steps=steps, procs=2, label=label)); print(S.table(out)); print("timing", out["timing"])
