"""Re-search a joint-crossing cable loop against COMBINED poses. Same machinery as arm_wire_search (anchors from the free space of each link, elastic-rod relaxation, clearance >= 0.8 mm, bend >= k x OD, stretch < 3 %)
but the joint under test is swept over its HARD range with the other joints at the 4 original bases PLUS NB random firmware-legal soft-range poses (fixed seed, same in every process).
Output: stl/electronics/wire_search_<J>_CB.json.  Usage: python3 wire_search_combined.py <J> [n_candidates] [seed] [NB]"""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wire_search as W, arm_keepout as K, arm_kin as ak, rom_limits as RL
j = sys.argv[1]; n = int(sys.argv[2]) if len(sys.argv) > 2 else 60; seed = int(sys.argv[3]) if len(sys.argv) > 3 else 1; NB = int(sys.argv[4]) if len(sys.argv) > 4 else 8
rng = np.random.default_rng(2024); qs = []
for _ in range(NB * 8):
    q = {k: float(rng.uniform(*RL.SOFT[k])) for k in RL.SOFT if k != "G"}; q["G"] = 50.0; qs.append(q)
FR = [ak.frames(q) for q in qs]; leg, nl = K.legal_poses(FR, 3.0); ids = {id(F) for F in leg}
extra = [q for q, F in zip(qs, FR) if id(F) in ids][:NB]; n0 = len(W.BASES); W.BASES.extend(extra); print("bases: %d original + %d random legal" % (n0, len(extra)), flush=True)
_orig = W.check_candidate
W.check_candidate = lambda joint, cand, r, kmin, nposes, bases=None: _orig(joint, cand, r, kmin, nposes, bases=tuple(range(len(W.BASES))))
res = W.run(j, n, "ultra", seed); res["bases"] = len(W.BASES)
json.dump(res, open(os.path.join(W.OUT, "wire_search_%s_CB.json" % j), "w"), indent=1); print(j, "found", len(res["found"]))
