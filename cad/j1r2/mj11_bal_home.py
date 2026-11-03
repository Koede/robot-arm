"""MJ11 balancer packaging, vertical tube homes on the lower back (2026-11-03): tube D44.5 hanging below a redirect sheave, x set to clear the rail rear face by 3 mm.  Uses the psi 120 / F 130 N solution of mj11_bal_pack.py."""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mj11_bal_pack as K, mj11_balancer as MB, mj_frame as M, mj_concepts as C
d = json.load(open(os.path.join(MB.OUTD, "balancer_pack.json"))); r = [x for x in d if x["psi"] == 120 and x["fcap"] == 130.0][0]; L_t = r["spring"]["tube_len_mm"]; A = MB.P1 + np.array(r["x"][0:3]); out = []
for ztop in (A[2] - 20.0, -240.0):
    for yt in (-122.0, -126.0, -128.0, -130.0, -134.0):
        zs = np.arange(ztop - L_t, ztop + 1, 10.0); rail_rear = max(C.rail_x(-119.0, z) for z in zs); xs = rail_rear + 3.0 + K.TD / 2
        c = K.census_tube(np.array([xs, yt, ztop]), [0, 0, -1.0], L_t); c.update(x_axis=round(float(xs), 1), y_axis=yt, z_top=round(float(ztop), 0), depth_behind_back_mm=round(float(xs + K.TD / 2 - max(M.back_x(yt, z) for z in zs)), 0)); out.append(c)
        print(c, flush=True)
json.dump(out, open(os.path.join(MB.OUTD, "balancer_home.json"), "w"), indent=1, default=float)
