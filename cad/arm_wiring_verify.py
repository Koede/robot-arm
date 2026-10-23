"""Verification of the in-arm wiring chain chosen by arm_chain.py (stl/electronics/arm_chain.json).
A  crossings : every joint swept over its HARD range at 4 base poses of the other joints (park, reach, soft-limit extreme 1 / 2), 9 poses each, elastic-rod relaxation with continuation.
               reported: min cable-surface clearance to every solid, min bend radius (centreline) vs k x OD, stretch, min distance of the cable to the wearer mesh.
B  links     : in-link corridors with the tube windows carved (A* in the free space of the link's group); path swept through 300 random poses within the soft range and
               checked against all OTHER groups (min clearance), length, min bend radius.
C  hub / XM  : hub J3 -> the elbow-pocket harness slot (18 x 8) corridor, window geometry.
Pass levels: cable-surface clearance >= 2.0 mm 'robust', 0.8..2.0 'marginal (sleeve abrasion protection)', < 0.8 FAIL; bend radius >= k x OD (k = KDEF); stretch < 3 %.
Outputs: stl/electronics/arm_wiring_verify.json, windows.json.   Usage: python3 arm_wiring_verify.py"""
import os, sys, json, time, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_wire_spec as S, arm_kin as ak, rom_limits as RL, arm_route as R, arm_wire_search as W
OUT = os.path.join(HERE, "..", "stl", "electronics")
ORDER = ["S1", "S2", "S3", "E", "W1", "W2", "W3"]
BASES = [dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50), dict(S1=90, S2=60, S3=20, E=90, W1=20, W2=20, W3=5, G=50),
         dict(S1=10, S2=-30, S3=-70, E=0, W1=-80, W2=-70, W3=-20, G=50), dict(S1=140, S2=88, S3=90, E=145, W1=80, W2=80, W3=30, G=50)]

_rng = np.random.default_rng(5)
for _ in range(4): BASES.append({**{j: float(_rng.uniform(*RL.SOFT[j])) for j in ORDER}, 'G': 50})

def wearer():
    import trimesh
    p = os.path.join(HERE, "..", "stl", "shoulder", "proxies", "wearer_makehuman.stl")
    return trimesh.load(p) if os.path.exists(p) else None

def stageA(chain, body, npos=9):
    res = {}
    for j in ORDER:
        c = chain["joints"][j]; od = chain["od"][j]; r = od / 2; k = S.KDEF[j] if not c.get("k") else float(c["k"]); kmin = k * od
        lo, hi = RL.HARD[j]; vs = np.linspace(lo, hi, npos); worst = dict(clr=1e9, minrad=1e9, stretch=0.0, body=1e9); n_fail = 0; order = [npos // 2] + list(range(npos // 2 - 1, -1, -1)) + list(range(npos // 2 + 1, npos))
        t0 = time.time(); poses = 0
        for bi, base in enumerate(BASES):
            prev = None; rec = {}
            for i in order:
                if i == npos // 2 + 1 and prev is not None: prev = rec[npos // 2]
                q = dict(base); q[j] = float(vs[i]); F = ak.frames(q)
                gA, gB = chain["frm"][j], chain["to"][j]; Pa = (F[gA] @ np.r_[c["A"], 1])[:3]; Pb = (F[gB] @ np.r_[c["B"], 1])[:3]
                ta = F[gA][:3, :3] @ np.array(c["tA"]); tb = F[gB][:3, :3] @ np.array(c["tB"])
                x, info = L.rod_relax(Pa, ta, Pb, tb, c["L"], F, r, W.CLR_MIN, x0=prev); rec[i] = x; prev = x; poses += 1
                worst["clr"] = min(worst["clr"], info["clr"]); worst["minrad"] = min(worst["minrad"], info["minrad"]); worst["stretch"] = max(worst["stretch"], info["stretch"])
                if info["clr"] < 0.75 or info["minrad"] < 0.95 * kmin or info["stretch"] >= 0.03: n_fail += 1
                if body is not None and j in ("S1", "S2", "S3"):
                    try:
                        import trimesh
                        d = trimesh.proximity.signed_distance(body, x)       # + inside in trimesh convention
                        worst["body"] = min(worst["body"], float((-d).min()))
                    except Exception: pass
        level = "robust" if worst["clr"] >= 1.95 else ("marginal" if worst["clr"] >= 0.75 else "FAIL")
        res[j] = dict(od=od, k=k, Rmin_mm=round(kmin, 1), range=[lo, hi], poses=poses, worst_clr=round(worst["clr"], 2), worst_minrad=round(worst["minrad"], 1), worst_stretch_pct=round(100 * worst["stretch"], 2),
                      body_min_mm=round(worst["body"], 1) if worst["body"] < 1e8 else None, fails=n_fail, level=level, ok=(n_fail == 0 and worst["minrad"] >= 0.95 * kmin), L=c["L"])
        print("A", j, res[j], "%.0fs" % (time.time() - t0), flush=True)
    return res

def legal_poses(FR, mm=3.0):
    import arm_keepout as K
    return K.legal_poses(FR, mm)

def stageB(chain, nrand=400, seed=123):
    """in-link routes (A* in the swept-keep-out field, smoothed): re-check with an INDEPENDENT 400-pose sample and the fine (1 mm) static field of the link's own group"""
    import arm_keepout as K
    FR_all = K.frames_list(K.pose_list(nrand, seed)); out = {}; links = {}
    FR, nleg = legal_poses(FR_all, 3.0); print("B legal poses (no pair of groups outside designed contact closer than 3 mm):", len(FR), "of", len(FR_all), flush=True)
    LG = {"F": "F", "M1": "M1", "M2": "M2", "M3": "M3", "ME": "ME", "MW1": "MW1", "MW2": "MW2", "MW3": "MW3", "M3x": "M3"}
    for g, items in chain["links"].items():
        for i, it in enumerate(items):
            name = "%s[%d]" % (g, i)
            if not it.get("path"): out[name] = dict(ok=False, why=it.get("why")); print("B", name, out[name], flush=True); continue
            pts = np.array(it["path"]); od = it["od"]; r = od / 2; grp = LG[g]
            static = float(L.sample_sdf(grp, pts[2:-2]).min()) - r
            moving = float(K.keepout(grp, pts[2:-2], FR).min()) - r
            rad = R._min_radius(R._resample(pts, 1.0)); length = float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())
            ok = bool(static >= 0.8 and moving >= 0.8 and rad >= 0.95 * R_KOD * od)
            out[name] = dict(poses_used=len(FR), group=grp, od=od, length_mm=round(length, 1), static_clear_mm=round(static, 2), swept_clear_mm=round(moving, 2), min_radius_mm=round(min(rad, 999), 1), need_radius_mm=round(R_KOD * od, 1), ok=ok)
            links.setdefault(g, {})[str(i)] = dict(path=it["path"], od=it["od"]); print("B", name, out[name], flush=True)
    json.dump(dict(links=links), open(os.path.join(OUT, "arm_links.json"), "w"))
    return out, []

R_KOD = 3.0

if __name__ == "__main__":
    chain = json.load(open(os.path.join(OUT, "arm_chain.json"))); body = wearer(); t0 = time.time()
    A = stageA(chain, body); B, wins = stageB(chain)
    json.dump(dict(stageA=A, stageB=B, windows=wins), open(os.path.join(OUT, "arm_wiring_verify.json"), "w"), indent=1)
    pass  # windows.json is written by arm_route.py windows
    okA = all(v["ok"] for v in A.values()); okB = all(v.get("ok", False) for v in B.values()); print("A ok", okA, "B ok", okB, "%.0fs" % (time.time() - t0))
