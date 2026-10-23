"""Which poses break a wrist loop? Usage: LOOPS=... python3 wire_fail_poses.py <J> [n]"""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L, arm_wire_spec as S, rom_limits as RL, arm_keepout as K
j = sys.argv[1]; N = int(sys.argv[2]) if len(sys.argv) > 2 else 16
d = json.load(open(os.environ.get("LOOPS", "../stl/electronics/arm_chain_loops_v2.json"))); c = d["joints"][j]; gA, gB = d["frm"][j], d["to"][j]; r = d["od"][j] / 2
rng = np.random.default_rng(11); PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50); qs = []
for i in range(N * 6):
    q = {n: float(rng.uniform(*RL.SOFT[n])) for n in RL.SOFT if n != "G"}; q["G"] = 50.0; qs.append(q)
FRall = [ak.frames(q) for q in qs]; leg, nl = K.legal_poses(FRall, 3.0); ids = {id(F) for F in leg}; legal = [q for q, F in zip(qs, FRall) if id(F) in ids][:N]
for q in legal:
    x = None
    for a_ in np.linspace(0, 1, 13)[1:]:
        qq = {n: PARK[n] + a_ * (q[n] - PARK[n]) for n in PARK}; F = ak.frames(qq)
        Pa = (F[gA] @ np.r_[c["A"], 1])[:3]; Pb = (F[gB] @ np.r_[c["B"], 1])[:3]
        x, info = L.rod_relax(Pa, F[gA][:3, :3] @ np.array(c["tA"]), Pb, F[gB][:3, :3] @ np.array(c["tB"]), c["L"], F, r, 0.8, x0=x, iters=400)
    print("%s clr %5.1f rad %5.1f st %6.1f%% | E %4.0f W1 %4.0f W2 %4.0f W3 %4.0f S3 %4.0f" % (j, info["clr"], info["minrad"], 100 * info["stretch"], q["E"], q["W1"], q["W2"], q["W3"], q["S3"]), flush=True)
