"""MJ11 balancer: scan vertical tube homes (y_axis, z_top, x_axis) on the lower back for the first position that passes every gate (2026-11-03)."""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mj11_bal_pack as K, mj11_balancer as MB, mj_frame as M
d = json.load(open(os.path.join(MB.OUTD, "balancer_pack.json"))); r = [x for x in d if x["psi"] == 120 and x["fcap"] == 130.0][0]; L_t = r["spring"]["tube_len_mm"]; res = []
for ztop in (-240.0, -270.0, -300.0, -330.0):
    for yt in (-126.0, -122.0, -118.0, -112.0):
        best = None
        for xs in np.arange(-84.0, -40.0, 4.0):
            c = K.census_tube(np.array([xs, yt, ztop]), [0, 0, -1.0], L_t); c.update(x_axis=float(xs), y_axis=yt, z_top=ztop)
            if c["pass"]: best = c; break
        res.append(best or dict(y_axis=yt, z_top=ztop, pass_none=True)); print(best if best else (ztop, yt, "none"), flush=True)
json.dump(res, open(os.path.join(MB.OUTD, "balancer_home2.json"), "w"), indent=1, default=float)
