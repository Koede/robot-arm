"""S1' hard-stop slack: M1 (yoke group, depends on S1' only) vs every fixed object/wearer, S1' -110..20 at 0.5 deg.  First S1' below the park where margin (d-MM) < 0.  -> stl/j1r2_sweep/mj11_s1p_edge.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, j1r2_sweep as S, mj11_sweep as W, mj_concepts3 as C3
sc, _ = W.scene(C3.mj11_geo(None)); F = S.Fcl(sc); pairs = [p for p in S.build_pairs(sc) if p.g == "M1" and p.kind == "obj"]; out = {}
for p in pairs:
    a = np.arange(-110.0, 20.0 + 1e-9, 0.5); d = []
    for s in a: q = dict(sc.park); q["S1p"] = float(s); d.append(F.dist_obj("M1", sc.pose_fn("M1", q), p.obj) - p.MM)
    d = np.array(d); ip = int(np.argmin(abs(a - sc.park["S1p"]))); lo = next((float(a[i]) for i in range(ip, -1, -1) if d[i] < 0), None); hi = next((float(a[i]) for i in range(ip, len(a)) if d[i] < 0), None)
    out[p.key] = dict(first_contact_below_park=lo, first_contact_above_park=hi, margin_at_park=round(float(d[ip]), 1), min_margin_in_soft=round(float(d[(a >= -95) & (a <= 35)].min()), 1))
json.dump(out, open(os.path.join(S.OUT, "mj11_s1p_edge.json"), "w"), indent=1); print(json.dumps({k: v for k, v in out.items() if v["first_contact_below_park"] is not None or v["first_contact_above_park"] is not None}, indent=1))
