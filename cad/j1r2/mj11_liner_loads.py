"""MJ11 liner normal-pressure map (increments from mj_eval's frame model) at park and reach, right liner springs with positions.
usage: J1R2_ROOT=/home/claude/ts python3 mj11_liner_loads.py  -> stl/j1r2_mounts/MJ11/liner_pressure.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import mj_frame as M, mj_concepts as C, mj_concepts2, mj_concepts3   # noqa
MOUNT_G = float(json.load(open(os.path.join(M.REPO, "stl", "j1r2_mounts", "summary.json")))["MJ11"]["mount_mass_g"])
out = {"mount_g": MOUNT_G}
for lc, sides in M.LCS.items():
    fr = M.Frame(); M.harness(fr, "R"); M.harness(fr, "L"); M.spine_bridge(fr)
    pads = {s: C.CONCEPTS["MJ11"]["frame"](fr, s) for s in ("R", "L")}
    for s in ("R", "L"):
        th, ph, pay = sides["right" if s == "R" else "left"]; M.apply_arm(fr, pads[s], s, th, ph, pay, extra_g=MOUNT_G)
    fr.solve(); rows = []
    for sp in fr.spring_forces():
        if sp["kind"] == "liner" and sp["label"].startswith("R_"):
            i = int(sp["label"].split("liner")[1]); p = fr.nodes[fr.idx[f"R_ln{i}"]]
            rows.append(dict(y=round(float(p[1]), 1), z=round(float(p[2]), 1), F_N=round(sp["F"], 3), p_kPa=round(sp["p_kPa"], 3)))
    out[lc] = rows
    ps = np.array([r["p_kPa"] for r in rows]); print(lc, len(rows), "increment kPa min %.2f max %.2f mean %.2f" % (ps.min(), ps.max(), ps.mean()), "sum N %.1f" % sum(r["F_N"] for r in rows))
json.dump(out, open(os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11", "liner_pressure.json"), "w"), indent=1)
