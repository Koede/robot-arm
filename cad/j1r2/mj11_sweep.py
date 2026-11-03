"""MJ11 shoulder-level collision sweep with the REAL MJ11 fixed parts (pad, seat dish, liner, rail spacers, clasp bracket) + the rail-only wing, in place of the J1 rev 2 riser / standoffs / fairing lap /
full wing (which the pad scan, mj11_scan.py, carried).  Same method as j1r2_opt.evaluate: first-contact frontier over S1' x S2 (5 deg), best passive-stop box whose +-5 deg hard-stop expansion is free.
usage: J1R2_ROOT=<work root> J1_PAD=-60,-190,12 python3 mj11_sweep.py [S1p_step S2_step]  -> stl/j1r2_mounts/MJ11/sweep.json"""
import os, sys, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import clib as L, j1r2_params as P, j1r2_geom as G, j1r2_kin as K, j1r2_sweep as S, j1r2_opt as O, mj_frame as M, mj_concepts3 as C3
REPLACED = ("pad_riser", "standoffs", "fairing_lap", "fairing_fixed", "pad", "column_anchor")
def scene(mj_parts, name="MJ11"):
    parts, info = G.build(y1=M.Y1, z1=M.Z1, standoff=M.DER["standoff"], full=True); D = info["derived"]; fixed, m1, m2 = {}, [], []
    for k, p in parts.items():
        g = p["group"]
        if p.get("kind") == "cable" or k.startswith("cable") or k in REPLACED: continue
        if g == "F": fixed[k] = p["mesh"]
        elif g == "M1": m1.append(p["mesh"])
        elif g == "M2": m2.append(p["mesh"])
    for k, p in mj_parts.items(): fixed["mj11_" + k] = p["mesh"]
    w = trimesh.load(os.path.join(M.REPO, "stl", "shoulder", "scapular", "wing_R.stl")); fixed["wing"] = L.I(w, L.box(-300, 100, -150, -100.0, -500, 100))      # rail strip only
    sh = dict(M1=L.U(*m1), M2=L.U(*m2)); return S.scene_from_kin(sh, fixed, S.make_wearer_parts(), kin=K.Kin(D), name=name, joints_upto="S3"), info
def run(mj_parts, steps=None, label="mj11"):
    t0 = time.time(); sc, info = scene(mj_parts); steps = steps or {"S1p": 5.0, "S2": 5.0, "S3": [0.0]}
    out = S.run_sweep(sc, dict(steps=steps, procs=2, label=label)); ok, vals, worst = O.allowed_map(out); box = O.best_box(ok, vals)
    reach = out["_internal"]["reach"].reshape(out["_internal"]["reach"].shape[0], out["_internal"]["reach"].shape[1], -1).any(axis=2); th_b, ph_b = O._preimage(); S1, S2 = vals["S1p"], vals["S2"]
    ti = np.clip(np.round(np.interp(th_b, S1, np.arange(len(S1)))).astype(int), 0, len(S1) - 1); pj = np.clip(np.round(np.interp(ph_b, S2, np.arange(len(S2)))).astype(int), 0, len(S2) - 1)
    inr = (th_b >= S1[0] - 2.5) & (th_b <= S1[-1] + 2.5) & (ph_b >= S2[0] - 2.5) & (ph_b <= S2[-1] + 2.5)
    return dict(pad=[M.Y1, M.Z1, M.DER["standoff"]], reach_cov=round(float(np.mean(inr & reach[ti, pj])), 3), box=box, worst=sorted(((k, round(float(v), 1)) for k, v in worst.items()), key=lambda kv: kv[1])[:10], t=round(time.time() - t0, 1)), out
if __name__ == "__main__":
    parts = C3.mj11_geo(None); st = {"S1p": float(sys.argv[1]) if len(sys.argv) > 1 else 5.0, "S2": float(sys.argv[2]) if len(sys.argv) > 2 else 5.0, "S3": [0.0]}
    r, _ = run(parts, st); print(json.dumps({k: v for k, v in r.items() if k != "worst"}, indent=1, default=float)); print(r["worst"])
    d = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11"); os.makedirs(d, exist_ok=True); json.dump(r, open(os.path.join(d, "sweep.json"), "w"), indent=1, default=float)
