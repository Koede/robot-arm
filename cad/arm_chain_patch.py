"""Re-route selected in-link runs of arm_chain.json with a denser swept keep-out (union of several pose samples).  Usage: python3 arm_chain_patch.py MW3 [MW2 ...]
Used when arm_wiring_verify.py stage B (independent poses) finds a swept-clearance violation that the 130-pose field of arm_chain.py missed."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_wire_spec as S, arm_route as R, arm_keepout as K, arm_chain as C
OUT = C.OUT
ch = json.load(open(os.path.join(OUT, "arm_chain.json")))
poses = K.pose_list(130, 7) + K.pose_list(400, 99) + K.pose_list(400, 4242)
import arm_wiring_verify as V
FR, _n = V.legal_poses(K.frames_list(poses), 3.0); print(len(FR), "legal poses of", len(poses), "in the union keep-out sample")
def pack(r): return dict(od=r.get("od"), path=r.get("path"), ok=r.get("ok", False), length_mm=r.get("length_mm"), min_clear_mm=r.get("min_clear_mm"), min_radius_mm=r.get("min_radius_mm"), why=r.get("why"))
ENDS = {"MW3": lambda: (np.array(ch["joints"]["W3"]["B"]), np.array(C.GSLOT), ch["od"]["W3"]), "MW2": lambda: (np.array(ch["joints"]["W2"]["B"]), np.array(ch["joints"]["W3"]["A"]), max(ch["od"]["W2"], ch["od"]["W3"])),
        "MW1": lambda: (np.array(ch["joints"]["W1"]["B"]), np.array(ch["joints"]["W2"]["A"]), max(ch["od"]["W1"], ch["od"]["W2"]))}
for g in sys.argv[1:]:
    P, Q, od = ENDS[g](); lo, hi = C.box_of([P, Q]); fld = R.field_K(g, lo, hi, FR, 2.0)
    r = R.corridor_k(fld, P, Q, od, kmin=C.KOD); print(g, {k: r.get(k) for k in ("ok", "length_mm", "min_clear_mm", "min_radius_mm", "why")}, flush=True)
    if r.get("path"): ch["links"][g] = [pack(r)]
json.dump(ch, open(os.path.join(OUT, "arm_chain.json"), "w"), indent=1)
