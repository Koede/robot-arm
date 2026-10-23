"""Choose one crossing candidate per joint (wire_search_<J>*_K.json, anchors already clear of every other link's swept volume) so the whole in-arm cable chain is consistent:
dynamic programming over the 7 crossings, cost = loop length + clearance penalty + in-link corridor cost.  In-link corridors are A* routes in the link's own frame through the combined field
min(own solids, swept keep-out of every other group over ~150 poses) (arm_route.field_K / corridor_k), then smoothed to a bend radius >= 3 x OD.
Link endpoints: F gland->S1.A ; M1 S1.B->S2.A ; M2 S2.B->S3.A ; M3 S3.B->hub J1 and hub J4->E.A (+ J3 -> XM slot) ; ME E.B->W1.A ; MW1 W1.B->W2.A ; MW2 W2.B->W3.A ; MW3 W3.B->G servo slot (PLACEHOLDER).
Output: stl/electronics/arm_chain.json.   Usage: python3 arm_chain.py [topK]"""
import os, sys, glob, json, time, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_wire_spec as S, arm_route as R, arm_keepout as K
OUT = os.path.join(HERE, "..", "stl", "electronics")
ORDER = ["S1", "S2", "S3", "E", "W1", "W2", "W3"]
PF = np.array([-75.0, -108.0, -125.0])        # F frame: 3 mm outside the right end wall of the main box (pods.json: box centre y -150, z -176; wall at y -111); gland position T
GSLOT = np.array([0.0, 19.0, 498.6])          # MW3 frame: 3 mm outside the -y end-wall cable slot of the claw hand (claw_hand.py, PLACEHOLDER position)
XMSLOT = np.array([0.0, 0.0, 210.0])          # M3: inside the 18 x 8 harness slot under the elbow XM540 pocket
KOD = 3.0                                      # bend radius of an in-link route >= 3 x OD (clips every ~40 mm hold it)

def cands(j, topk):
    out = []; seen = set()
    for f in sorted(glob.glob(os.path.join(OUT, "wire_search_%s*_K.json" % j))):
        d = json.load(open(f))
        for c in d.get("found", []):
            key = (tuple(c["A"]), tuple(c["B"]))
            if key in seen: continue
            seen.add(key); c = dict(c); c["od"] = d["bundle"]["od_mm"]; c["k"] = d.get("k"); c["src"] = os.path.basename(f); out.append(c)
    out = [c for c in out if abs(c["od"] - S.bundle(j, "ultra", 1)["od_mm"]) < 0.05]
    ff = os.path.join(OUT, "anchor_feas.json")                                    # tie-saddle feasibility (arm_anchor_feas.py): both anchors need a post <= 16 mm with >= 2 mm swept margin
    if os.path.exists(ff):
        fe = {(tuple(r["A"]), tuple(r["B"])): r["feasA"] and r["feasB"] for r in json.load(open(ff)).get(j, [])}
        out = [c for c in out if fe.get((tuple(c["A"]), tuple(c["B"])), False)]
    out.sort(key=lambda c: (-min(c["clr"], 6.0), c["L"])); return out[:topk]

def box_of(pts, pad=30.0):
    pts = np.array(pts); return pts.min(0) - pad, pts.max(0) + pad

def run(topk=8, pitch=2.0, nposes=130):
    t0 = time.time(); C = {j: cands(j, topk) for j in ORDER}
    for j in ORDER: print(j, "candidates", len(C[j]), [round(c["clr"], 2) for c in C[j]], flush=True)
    miss = [j for j in ORDER if not C[j]]
    if miss: print("missing candidates:", miss); return None
    import arm_wiring_verify as V
    FR_all = K.frames_list(K.pose_list(nposes, seed=7)); FR, _nl = V.legal_poses(FR_all, 3.0); print("keep-out sample: %d legal poses of %d (poses where the arm hits itself or the fixed structure are outside the firmware envelope)" % (_nl, len(FR_all)), flush=True)
    boxes, hubd = R.hub_boxes(); hp = R.hub_points(hubd)
    od = {j: S.bundle(j, "ultra", 1)["od_mm"] for j in ORDER}
    LINK = {"S2": "M1", "S3": "M2", "E": "M3", "W1": "ME", "W2": "MW1", "W3": "MW2"}
    FLD = {}
    ends = {"F": [PF] + [c["A"] for c in C["S1"]], "M1": [c["B"] for c in C["S1"]] + [c["A"] for c in C["S2"]], "M2": [c["B"] for c in C["S2"]] + [c["A"] for c in C["S3"]],
            "M3": [c["B"] for c in C["S3"]] + [c["A"] for c in C["E"]] + list(hp.values()) + [XMSLOT], "ME": [c["B"] for c in C["E"]] + [c["A"] for c in C["W1"]],
            "MW1": [c["B"] for c in C["W1"]] + [c["A"] for c in C["W2"]], "MW2": [c["B"] for c in C["W2"]] + [c["A"] for c in C["W3"]], "MW3": [c["B"] for c in C["W3"]] + [GSLOT]}
    for g, pts in ends.items():
        lo, hi = box_of(pts); FLD[g] = R.field_K(g, lo, hi, FR, pitch, boxes if g == "M3" else None); print("field", g, FLD[g][0].shape, "%.0fs" % (time.time() - t0), flush=True)
    cache = {}
    def corr(g, P, Q, odp):
        key = (g, tuple(np.round(P, 1)), tuple(np.round(Q, 1)), odp)
        if key not in cache: cache[key] = R.corridor_k(FLD[g], P, Q, odp, kmin=KOD)
        return cache[key]
    def cst(r): return (r["length_mm"] + 8 * max(0.0, 3.0 - r["min_clear_mm"]) + (0 if r["ok"] else 600)) if r.get("length_mm") else 5000.0
    def link_cost(g, j1, c1, j2, c2):
        if g == "M3":
            r1 = corr("M3", hp["J1"], np.array(c1["B"]), od[j1]); r2 = corr("M3", hp["J4"], np.array(c2["A"]), od[j2]); return cst(r1) + cst(r2), [r1, r2]
        r = corr(g, np.array(c1["B"]), np.array(c2["A"]), max(od[j1], od[j2])); return cst(r), [r]
    pen = lambda c: 120 * max(0, 3.0 - c["clr"])
    dp = [{i: (c["L"] + pen(c) + cst(corr("F", PF, np.array(c["A"]), od["S1"])), None) for i, c in enumerate(C["S1"])}]
    for n in range(1, len(ORDER)):
        j1, j2 = ORDER[n - 1], ORDER[n]; cur = {}
        for i2, c2 in enumerate(C[j2]):
            best = None
            for i1, c1 in enumerate(C[j1]):
                cc, _ = link_cost(LINK[j2], j1, c1, j2, c2); tot = dp[n - 1][i1][0] + cc + c2["L"] + pen(c2)
                if best is None or tot < best[0]: best = (tot, i1)
            cur[i2] = best
        dp.append(cur); print("DP", j2, "done %.0fs" % (time.time() - t0), flush=True)
    fin = {i: dp[-1][i][0] + cst(corr("MW3", np.array(c["B"]), GSLOT, od["W3"])) for i, c in enumerate(C["W3"])}
    iw = min(fin, key=fin.get); idx = {"W3": iw}
    for n in range(len(ORDER) - 1, 0, -1): idx[ORDER[n - 1]] = dp[n][idx[ORDER[n]]][1]
    chosen = {j: C[j][idx[j]] for j in ORDER}; links = {}
    def pack(r): return dict(od=r.get("od"), path=r.get("path"), ok=r.get("ok", False), length_mm=r.get("length_mm"), min_clear_mm=r.get("min_clear_mm"), min_radius_mm=r.get("min_radius_mm"), why=r.get("why"))
    links["F"] = [pack(corr("F", PF, np.array(chosen["S1"]["A"]), od["S1"]))]
    for n in range(1, len(ORDER)):
        j1, j2 = ORDER[n - 1], ORDER[n]; cc, rs = link_cost(LINK[j2], j1, chosen[j1], j2, chosen[j2]); links[LINK[j2]] = [pack(r) for r in rs]
    links["MW3"] = [pack(corr("MW3", np.array(chosen["W3"]["B"]), GSLOT, od["W3"]))]
    links["M3x"] = [pack(corr("M3", hp["J3"], XMSLOT, 2.6))]
    res = dict(joints={j: {k: (v if not isinstance(v, np.generic) else v.item()) for k, v in chosen[j].items()} for j in ORDER}, frm={j: S.CROSS[j]["frm"] for j in ORDER}, to={j: S.CROSS[j]["to"] for j in ORDER},
               od=od, links=links, hub_points={k: v.tolist() for k, v in hp.items()}, g_slot=GSLOT.tolist(), f_start=PF.tolist(), xm_slot=XMSLOT.tolist(), poses_for_keepout=len(FR))
    json.dump(res, open(os.path.join(OUT, "arm_chain.json"), "w"), indent=1)
    for j in ORDER: c = chosen[j]; print(j, "A", c["A"], "B", c["B"], "L", c["L"], "clr", c["clr"], "minrad", c["minrad"], c["src"])
    for g, rs in links.items(): print("link", g, [(r["ok"], r["length_mm"], r["min_clear_mm"], r["min_radius_mm"], r.get("why")) for r in rs])
    return res

if __name__ == "__main__":
    run(int(sys.argv[1]) if len(sys.argv) > 1 else 8)
