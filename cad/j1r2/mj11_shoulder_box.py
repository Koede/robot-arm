"""MJ11 shoulder-level passive-stop box re-check next to the whole-arm sweep: S1' x S2 5 deg, S3 = {0} (reproduces the 2026-11-02 adoption numbers) and S3 = {-70,0,90}.
usage: J1R2_ROOT=<work root> python3 mj11_shoulder_box.py -> stl/j1r2_sweep/mj11_shoulder_box.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, mj11_sweep as W, mj_concepts3 as C3, j1r2_sweep as S, j1r2_opt as O
if __name__ == "__main__":
    parts = C3.mj11_geo(None); res = {}
    for tag, s3 in (("S3_0", [0.0]), ("S3_m70_0_90", [-70.0, 0.0, 90.0])):
        r, out = W.run(parts, {"S1p": 5.0, "S2": 5.0, "S3": s3}, label="mj11_sh_" + tag); res[tag] = r
        if tag != "S3_0":                       # keep-out mask for the firmware table: 1 = free AND reachable from park, 0 = contact; blocker = first pair (by table order) in contact
            ok, vals, _ = O.allowed_map(out); I = out["_internal"]; reach = I["reach"]; J = list(vals); blk = {}
            for pi, (p, d) in enumerate(zip(I["pairs"], I["designed"])):
                if d: continue
                c = S._bcast(np.asarray(I["tabs"][pi]) < p.MM, p.dims, J)
                for idx in zip(*np.where(np.broadcast_to(c, ok.shape))): blk.setdefault(idx, p.key)
            res["keepout"] = dict(S1p=vals["S1p"].tolist(), S2=vals["S2"].tolist(), S3=vals["S3"].tolist(), free_reachable=reach.astype(int).tolist(), free_any=ok.astype(int).tolist(),
                                  blocker={f"{vals['S1p'][i]:.1f},{vals['S2'][j]:.1f},{vals['S3'][k]:.0f}": b for (i, j, k), b in blk.items() if reach[i, j, k] == 0 and ok[i, j, k] == 0 and any(abs(vals['S1p'][i] - vals['S1p'][i2]) <= 5 for i2 in [i])})
        print(tag, json.dumps({k: v for k, v in r.items() if k != "worst"}, default=float)); print(S.table(out)[:6000])
    json.dump(res, open(os.path.join(S.OUT, "mj11_shoulder_box.json"), "w"), indent=1, default=float)
