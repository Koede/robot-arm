"""MJ11 WHOLE-ARM first-contact sweep (S1p,S2,S3,E,W1,W2,W3; HAND rev B/C proxies from j1r2_kin) against the real MJ11 fixed parts, the rail-only wing, the clasp bracket (CLASP_PSI) and the wearer incl. head/neck.
Steps as j1r2_integrate full mode.  usage: J1R2_ROOT=<work root> python3 mj11_whole_arm.py [label] -> stl/j1r2_sweep/sweep_<label>.json, tables_<label>.npz, mj11_whole_arm_summary.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mj11_sweep as W, mj_concepts3 as C3, mj_frame as M, j1r2_sweep as S, j1r2_opt as O
STEPS = {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}
if __name__ == "__main__":
    label = sys.argv[1] if len(sys.argv) > 1 else "mj11_whole_arm"
    parts = C3.mj11_geo(None); sc, info = W.scene(parts, name="MJ11_whole", joints_upto="W3")
    print("fixed", list(sc.fixed), "groups", list(sc.groups), flush=True)
    out = S.run_sweep(sc, dict(steps=STEPS, procs=2, label=label)); print(S.table(out)); print("timing", out["timing"])
    json.dump(dict(pad=[M.Y1, M.Z1, M.DER["standoff"]], clasp_psi=C3.CLASP_PSI, steps=STEPS), open(os.path.join(S.OUT, f"{label}_meta.json"), "w"), indent=1)
