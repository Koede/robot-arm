"""MJ11 WHOLE-ARM first-contact sweep (S1p,S2,S3,E,W1,W2,W3) against the real MJ11 fixed parts + rail-only wing + clasp bracket (CLASP_PSI) + wearer (skin 2 mm, head/neck 60 mm).
Steps as j1r2_integrate full mode.  usage: J1R2_ROOT=<work root> python3 mj11_whole.py [label]  -> stl/j1r2_sweep/sweep_<label>.json, tables_<label>.npz, mj11_whole_summary.json"""
import os, sys, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import clib as L, j1r2_geom as G, j1r2_kin as K, j1r2_sweep as S, j1r2_opt as O, mj_frame as M, mj_concepts3 as C3
import mj11_sweep as W
def scene(mj_parts, upto="W3"):
    parts, info = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True); D = info["derived"]; fixed, m1, m2 = {}, [], []
    for k, p in parts.items():
        g = p["group"]
        if p.get("kind") == "cable" or k.startswith("cable") or k in W.REPLACED: continue
        if g == "F": fixed[k] = p["mesh"]
        elif g == "M1": m1.append(p["mesh"])
        elif g == "M2": m2.append(p["mesh"])
    for k, p in mj_parts.items(): fixed["mj11_" + k] = p["mesh"]
    w = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); fixed["wing"] = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100))
    sh = dict(M1=L.U(*m1), M2=L.U(*m2)); return S.scene_from_kin(sh, fixed, S.make_wearer_parts(), kin=K.Kin(D), name="MJ11_whole", joints_upto=upto), info
if __name__ == "__main__":
    label = sys.argv[1] if len(sys.argv) > 1 else "mj11_whole"; t0 = time.time()
    parts = C3.mj11_geo(None); sc, info = scene(parts); print("scene", round(time.time() - t0, 1), "s fixed:", list(sc.fixed), "groups:", list(sc.groups), flush=True)
    steps = {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}
    out = S.run_sweep(sc, dict(steps=steps, procs=2, label=label)); print(S.table(out), flush=True); print("timing", out["timing"], flush=True)
    json.dump(dict(pad=[M.Y1, M.Z1, M.DER["standoff"]], clasp_psi=C3.CLASP_PSI, summary=out["summary"], timing=out["timing"]), open(os.path.join(S.OUT, "mj11_whole_summary.json"), "w"), indent=1, default=float)
