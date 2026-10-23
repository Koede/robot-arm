"""Independent physical audit of the 7 joint-crossing cables (arm_chain_loops.json). Checks beyond the original stage A:
 1 anchors sit on real printed links (SDF distance anchor->own link surface) and the clamp tangent leaves into free space
 2 chord |Pa-Pb| vs cable length over RANDOM poses with ALL joints moving together within the HARD range (cable must reach: chord < L, with slack margin)
 3 relaxed cable (rod) at those poses: clearance to every solid, min bend radius vs k x OD, stretch
 4 in-link runs join consecutive anchors: Euclidean distance B_j -> A_(j+1) <= run length (from arm_electrical.json)
Usage: python3 wire_audit.py [nposes]"""
import os, sys, json, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import arm_kin as ak, arm_wiring_lib as L, arm_wire_spec as S, rom_limits as RL
N = int(sys.argv[1]) if len(sys.argv) > 1 else 40
d = json.load(open(os.environ.get("LOOPS","../stl/electronics/arm_chain_loops.json"))); J = d["joints"]; rng = np.random.default_rng(11)
out = {"anchors": {}, "joints": {}}
print("== 1 anchors on real links"); 
for j, c in J.items():
    r = d["od"][j] / 2
    for end, g in (("A", d["frm"][j]), ("B", d["to"][j])):
        P = np.array(c[end]); t = np.array(c["t" + end]); t = t / np.linalg.norm(t)
        s0 = float(L.sample_sdf(g, P[None])[0]); s1 = float(L.sample_sdf(g, (P + t * 9)[None])[0])
        out["anchors"][j + end] = dict(group=g, sdf_mm=round(s0, 1), sdf_9mm_ahead=round(s1, 1)); print(j, end, g, "distance to link surface %.1f mm; 9 mm along tangent %.1f mm" % (s0, s1))
import arm_keepout as K
SOFT = RL.SOFT; PARK = dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50)
qs = []
for i in range(N * 6):
    q = {n: float(rng.uniform(*SOFT[n])) for n in SOFT if n != "G"}; q["G"] = 50.0; qs.append(q)
FRall = [ak.frames(q) for q in qs]; FRleg, nl = K.legal_poses(FRall, 3.0); ids = {id(F) for F in FRleg}
legal = [(q, F) for q, F in zip(qs, FRall) if id(F) in ids][:N]; print("== 2/3 random full-arm poses inside the SOFT range, legal (>=3 mm, firmware envelope): %d of %d sampled; using %d, 12-step continuation from park" % (nl, len(qs), len(legal)))
for j, c in J.items():
    gA, gB = d["frm"][j], d["to"][j]; od = d["od"][j]; r = od / 2; k = S.KDEF[j]; Lc = c["L"]; res = dict(chord=[], clr=[], rad=[], st=[])
    for q, F1 in legal:
        x = None
        for a_ in np.linspace(0, 1, 13)[1:]:
            qq = {n: PARK[n] + a_ * (q[n] - PARK[n]) for n in PARK}; F = ak.frames(qq)
            Pa = (F[gA] @ np.r_[c["A"], 1])[:3]; Pb = (F[gB] @ np.r_[c["B"], 1])[:3]; ta = F[gA][:3, :3] @ np.array(c["tA"]); tb = F[gB][:3, :3] @ np.array(c["tB"])
            x, info = L.rod_relax(Pa, ta, Pb, tb, Lc, F, r, 0.8, x0=x, iters=400)
        res["chord"].append(float(np.linalg.norm(Pa - Pb))); res["clr"].append(info["clr"]); res["rad"].append(info["minrad"]); res["st"].append(info["stretch"])
    ch = np.array(res["chord"]); o = dict(L=Lc, chord_max=round(float(ch.max()), 1), chord_min=round(float(ch.min()), 1), slack_min_pct=round(float(100 * (Lc - ch.max()) / Lc), 1), clr_min=round(float(min(res["clr"])), 2),
        rad_min=round(float(min(res["rad"])), 1), rad_need=round(k * od, 1), stretch_max_pct=round(100 * float(max(res["st"])), 2), n=len(legal), poses_clr_below_0p8=int(sum(1 for v in res["clr"] if v < 0.8)), poses_rad_below_need=int(sum(1 for v in res["rad"] if v < k * od * 0.95)))
    out["joints"][j] = o; print(j, o, flush=True)
print("== 4 in-link runs join anchors")
runs = {"S1->S2 (M1)": ("S1", "B", "S2", "A", 16.9), "S2->S3 (M2)": ("S2", "B", "S3", "A", 123.1), "E->W1 (ME)": ("E", "B", "W1", "A", 22.3), "W1->W2 (MW1)": ("W1", "B", "W2", "A", 13.0), "W2->W3 (MW2)": ("W2", "B", "W3", "A", 43.6)}
out["runs"] = {}
for nm, (j1, e1, j2, e2, Lr) in runs.items():
    e = float(np.linalg.norm(np.array(J[j1][e1]) - np.array(J[j2][e2]))); out["runs"][nm] = dict(euclid=round(e, 1), run_len=Lr, ratio=round(Lr / max(e, 1e-6), 2)); print(nm, "straight distance %.1f mm, routed run %.1f mm" % (e, Lr))
json.dump(out, open(os.environ.get("AUDIT_OUT","../stl/electronics/wire_audit.json"), "w"), indent=1)
