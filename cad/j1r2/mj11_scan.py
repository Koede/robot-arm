"""MJ11 (2026-11-02, SHOULDER, concept): where can the J1 pancake sit so its big flat face lies on Forest's back?
Scans the S1' axis position (y1, z1) with a small seat standoff and reports, per position:
  gap_mean / gap_min / gap_max : pad front face (x0) to the skin over the R66 pad footprint [mm]  (a flat pad lying on the back wants all three small and similar)
  tilt_deg                     : angle between the best-fit plane of the skin under the footprint and the pad plane (x = const)
  reach_cov / box_cov          : j1r2_opt.evaluate scores (share of the baseline arm-direction set reachable contact-free / inside the best passive-stop box)
  box ranges                   : best passive-stop box (S1' lo..hi, S2 lo..hi)
usage: J1R2_ROOT=<work root> python3 mj11_scan.py [standoff]  -> stl/j1r2_mounts/mj11_scan.json"""
import os, sys, json, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L, j1r2_params as P, j1r2_opt as O, mj_frame as M
OUT = os.path.join(M.REPO, "stl", "j1r2_mounts"); os.makedirs(OUT, exist_ok=True)
def footprint(y1, z1, x0, R=66.0):
    Y, Z, X = L.rear_surface(y1 - R - 6, y1 + R + 6, z1 - R - 6, z1 + R + 6, 4.0, 1.0); m = np.hypot(Y - y1, Z - z1) <= R
    y, z, x = Y[m], Z[m], X[m]; A = np.c_[y - y1, z - z1, np.ones(len(y))]; c = np.linalg.lstsq(A, x, rcond=None)[0]
    tilt = float(np.degrees(np.arctan(np.hypot(c[0], c[1])))); res = x - A @ c
    return dict(gap_min=round(float((x0 - x).min()), 1), gap_mean=round(float((x0 - x).mean()), 1), gap_max=round(float((x0 - x).max()), 1), tilt_deg=round(tilt, 1), plane_rms=round(float(np.sqrt((res ** 2).mean())), 1))
if __name__ == "__main__":
    so = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
    ys = [-90.0, -60.0, -30.0, 0.0, 10.0, 30.0]; zs = [-30.0, -60.0, -90.0, -120.0, -150.0]; rows = []; t0 = time.time()
    for z1 in zs:
        for y1 in ys:
            D = P.derive(y1, z1, so); row = dict(y1=y1, z1=z1, x0=round(D["x0"], 1), driver="sweep" if D["xs_sw"] + 2.3 > D["xs_fp"] + so else "footprint", **footprint(y1, z1, D["x0"]))
            try:
                r, _ = O.evaluate(y1, z1, so); row.update(reach_cov=round(r["reach_cov"], 3), box_cov=round(r["box"]["cov"], 3) if r["box"] else None, box=r["box"]["ranges"] if r["box"] else None,
                                                          park_ok=r["park_ok"], worst=sorted(((k, round(v, 1)) for k, v in r["worst"].items()), key=lambda kv: kv[1])[:5])
            except Exception as e: row.update(error=repr(e)[:160])
            rows.append(row); print("[%4.0fs]" % (time.time() - t0), {k: v for k, v in row.items() if k not in ("worst", "box")}, flush=True)
            json.dump(rows, open(os.path.join(OUT, "mj11_scan.json"), "w"), indent=1, default=float)
