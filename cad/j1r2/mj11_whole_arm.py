"""MJ11 WHOLE-ARM first-contact sweep (all 7 joints, HAND rev B/C proxies via j1r2_kin) against the real MJ11 fixed parts, the rail-only wing, clasp bracket (CLASP_PSI) and the wearer incl. head/neck.
Steps as j1r2_integrate full mode.  usage: J1R2_ROOT=<work root> python3 mj11_whole_arm.py  -> stl/j1r2_sweep/sweep_mj11_whole.json, tables_mj11_whole.npz, mj11_whole_report.json"""
import os, sys, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, j1r2_sweep as S, j1r2_opt as O, mj11_sweep as W, mj_concepts3 as C3, mj_frame as M
STEPS = {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}
if __name__ == "__main__":
    t0 = time.time(); parts = C3.mj11_geo(None); sc, info = W.scene(parts, "MJ11_whole", "W3"); print("scene", round(time.time() - t0, 1), list(sc.groups), flush=True)
    out = S.run_sweep(sc, dict(steps=STEPS, procs=2, label="mj11_whole")); txt = S.table(out); print(txt, flush=True)
    open(os.path.join(S.OUT, "mj11_whole_table.txt"), "w").write(txt)
    ok, vals, worst = O.allowed_map(out); print("pad", M.Y1, M.Z1, M.DER["standoff"], "clasp_psi", C3.CLASP_PSI, "total", round(time.time() - t0, 1))
