"""Clamp feasibility of every cable-search candidate anchor: best tie-saddle post direction (arm_clips.best_post_dir) over the legal poses.
A candidate is feasible when BOTH anchors have a post <= 16 mm whose swept margin (min K - clamp half width) is >= 2 mm.
Output: stl/electronics/anchor_feas.json {joint: [{A, B, feasA, feasB, marginA, marginB, distA, distB}]}.   Usage: python3 arm_anchor_feas.py [topK] [joint ...]   (with joints: only those are recomputed and merged into the existing file)"""
import os, sys, glob, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_wire_spec as S, arm_keepout as K, arm_wiring_verify as V, arm_clips as C
OUT = os.path.join(HERE, "..", "stl", "electronics")
if __name__ == "__main__":
    topk = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    FRL, n = V.legal_poses(K.frames_list(K.pose_list(400, 123)) + K.frames_list(K.pose_list(130, 7)), 3.0); print(n, "legal poses", flush=True)
    fn = os.path.join(OUT, "anchor_feas.json"); only = sys.argv[2:]; res = json.load(open(fn)) if (only and os.path.exists(fn)) else {}; cache = {}
    for j in (only or ["S1", "S2", "S3", "E", "W1", "W2", "W3"]):
        od = S.bundle(j, "ultra", 1)["od_mm"]; halfw = od / 2 + 0.05 + 1.8 + 0.6; out = []
        seen = set(); cs = []
        for f in sorted(glob.glob(os.path.join(OUT, "wire_search_%s*_K.json" % j))):
            d = json.load(open(f))
            for c in d.get("found", []):
                key = (tuple(c["A"]), tuple(c["B"]))
                if key in seen or abs(d["bundle"]["od_mm"] - od) > 0.05: continue
                seen.add(key); cs.append(c)
        cs.sort(key=lambda c: (-min(c["clr"], 6.0), c["L"])); cs = cs[:topk]
        for c in cs:
            r = {}
            for e, g in (("A", S.CROSS[j]["frm"]), ("B", S.CROSS[j]["to"])):
                k = (g, tuple(c[e]))
                if k not in cache: cache[k] = C.best_post_dir(g, c[e], c["t" + e], halfw, FRL)
                v, dist, m = cache[k]; r["margin" + e] = None if m is None else round(m, 2); r["dist" + e] = None if dist is None else round(dist, 1)
                r["feas" + e] = bool(v is not None and dist <= 16.0 and m >= 2.0)
            r.update(A=c["A"], B=c["B"], clr=c["clr"], L=c["L"]); out.append(r)
        res[j] = out; print(j, [(r["feasA"], r["feasB"], r["marginA"], r["marginB"]) for r in out], flush=True)
    json.dump(res, open(fn, "w"), indent=1)
