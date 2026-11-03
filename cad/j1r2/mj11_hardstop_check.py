"""Exact check of the MJ11 passive-stop box corners incl. the +-5 deg hard-stop expansion (not just the grid cell next to the box): min gate margin over M1/M2/M3 x every fixed object/wearer at the 4 corners x S3 {-70,0,90}, sampled along the hard-stop boundary at 1 deg.  -> stl/j1r2_sweep/mj11_hardstop_check.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, j1r2_sweep as S, mj11_sweep as W, mj_concepts3 as C3
sc, _ = W.scene(C3.mj11_geo(None)); F = S.Fcl(sc); pairs = [p for p in S.build_pairs(sc) if p.g in ("M1", "M2", "M3")]
BOX = dict(S1p=(-95.0, 5.0), S2=(-15.0, 17.4)); lo1, hi1, lo2, hi2 = BOX["S1p"][0] - 5, BOX["S1p"][1] + 5, BOX["S2"][0] - 5, BOX["S2"][1] + 5
def _d(p, q): return F.dist_obj(p.g, sc.pose_fn(p.g, q), p.obj) if p.kind == "obj" else F.dist_gg(p.other, sc.pose_fn(p.other, q), p.g, sc.pose_fn(p.g, q))
pairs = [p for p in pairs if not (_d(p, sc.park) < p.MM and not p.obj.startswith("wearer_"))]      # drop designed-contact pairs (closer than the gate at park: stage bearings, bumpers, stops, pad)
res = {}; worst = {}
bnd = [(a, b) for a in np.arange(lo1, hi1 + 1e-9, 1.0) for b in (lo2, hi2)] + [(a, b) for a in (lo1, hi1) for b in np.arange(lo2, hi2 + 1e-9, 1.0)]
for s3 in (-70.0, 0.0, 90.0):
    for (a, b) in bnd:
        q = dict(sc.park); q.update(S1p=float(a), S2=float(b), S3=s3)
        for p in pairs:
            d = F.dist_obj(p.g, sc.pose_fn(p.g, q), p.obj) if p.kind == "obj" else F.dist_gg(p.other, sc.pose_fn(p.other, q), p.g, sc.pose_fn(p.g, q))
            m = d - p.MM
            if p.key not in worst or m < worst[p.key][0]: worst[p.key] = (float(m), float(a), float(b), s3, float(d))
rep = {k: dict(margin_mm=round(v[0], 2), S1p=v[1], S2=v[2], S3=v[3], d_mm=round(v[4], 2)) for k, v in worst.items() if v[0] < 5.0}
json.dump(dict(box=BOX, hard_stop_corner=[lo1, hi1, lo2, hi2], worst_margin_pairs=rep), open(os.path.join(S.OUT, "mj11_hardstop_check.json"), "w"), indent=1); print(json.dumps(rep, indent=1))
