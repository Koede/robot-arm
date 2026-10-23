"""Cable-loop search across each joint with an elastic-rod relaxation (arm_wiring_lib.rod_relax) against the real parts.
Anchors A (group frm) and B (group to) are the points where the bundle is CLAMPED by a printed strain-relief feature; tangent = direction the bundle leaves the clamp.
Pass criteria at every pose over the joint HARD range (soft + 5 deg): clearance >= CLR_MIN to every solid (cable surface), bend radius >= K x OD, stretch error < 3 %.
Kinematic envelope, not a mechanics simulation.  Usage: python3 arm_wire_search.py W3 [n_candidates] [kind] [seed]
Writes stl/electronics/wire_search_<joint>.json."""
import sys, os, json, time, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_wiring_lib as L, arm_wire_spec as S, arm_kin as ak, rom_limits as RL, arm_keepout as K
OUT = os.path.join(HERE, "..", "stl", "electronics")
CLR_MIN = float(os.environ.get('CLRMIN', '0.8')); D = ak.D; BAND = 10.0
REGION = {"S1": ((25, 0, -52), 95), "S2": ((0, -10, 40), 120), "S3": ((0, 0, 112), 75), "E": ((0, 0, 230), 78),
          "W1": ((0, D, 345), 48), "W2": ((0, D, 410), 58), "W3": ((0, D, 455), 50)}
BASES = [dict(S1=45, S2=25, S3=0, E=45, W1=0, W2=0, W3=0, G=50), dict(S1=90, S2=60, S3=20, E=90, W1=20, W2=20, W3=5, G=50),
         dict(S1=10, S2=-30, S3=-70, E=0, W1=-80, W2=-70, W3=-20, G=50), dict(S1=140, S2=88, S3=90, E=145, W1=80, W2=80, W3=30, G=50)]
KCLR = float(os.environ.get('KCLR', '2.0'))
AXES = [np.array(v, float) for v in ([1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1])]

def q_at(joint, v, base=0):
    q = dict(BASES[base]); q[joint] = float(v); return q

def anchor_points(g, centre, rad, r):
    sdf, lo, p = L.sdf_of(g); ii = np.argwhere((sdf >= r + 1.0) & (sdf <= r + BAND)); P = lo + ii * p
    return P[np.linalg.norm(P - np.asarray(centre), axis=1) <= rad]

def tangent_ok(g, P, t, r, ahead=9.0):
    q = np.array([P + t * ahead * f for f in (0.5, 1.0)]); v = L.sample_sdf(g, q); return bool((v >= r + 1.0).all())

def solve_pose(joint, cand, v, r, kmin, base, x0=None):
    q = q_at(joint, v, base); F = ak.frames(q); gA, gB = S.CROSS[joint]["frm"], S.CROSS[joint]["to"]
    Pa = (F[gA] @ np.r_[cand["A"], 1])[:3]; Pb = (F[gB] @ np.r_[cand["B"], 1])[:3]
    ta = F[gA][:3, :3] @ cand["tA"]; tb = F[gB][:3, :3] @ cand["tB"]
    x, info = L.rod_relax(Pa, ta, Pb, tb, cand["L"], F, r, CLR_MIN, x0=x0)
    ok = info["clr"] >= CLR_MIN - 0.05 and info["minrad"] >= kmin * 0.95 and info["stretch"] < 0.03
    return x, info, ok

def check_candidate(joint, cand, r, kmin, nposes, bases=(0, 1, 2, 3)):
    lo, hi = RL.HARD[joint]; vs = np.linspace(lo, hi, nposes); worst = dict(clr=1e9, minrad=1e9); order = [len(vs) // 2] + list(range(len(vs) // 2 - 1, -1, -1))
    for b in bases:
        prev = None; recs = {}
        for i in order + list(range(len(vs) // 2 + 1, len(vs))):
            if i == len(vs) // 2 + 1 and prev is not None: prev = recs[len(vs) // 2]
            x, info, ok = solve_pose(joint, cand, vs[i], r, kmin, b, x0=prev); recs[i] = x; prev = x
            worst = dict(clr=min(worst["clr"], info["clr"]), minrad=min(worst["minrad"], info["minrad"]))
            if not ok: return None
    return worst

def run(joint, n_cand=150, kind="ultra", seed=1, split=None, k=None):
    split = 1 if split is None else split; k = S.KDEF[joint] if k is None else k
    rng = np.random.default_rng(seed); bd = S.bundle(joint, kind, split); od = bd["od_mm"]; r = od / 2; kmin = k * od
    gA, gB = S.CROSS[joint]["frm"], S.CROSS[joint]["to"]; cen, rad = REGION[joint]; rad = float(os.environ.get("RAD", rad))
    PA = anchor_points(gA, cen, rad, r); PB = anchor_points(gB, cen, rad, r)
    FR = K.frames_list(K.pose_list(130, seed=100 + seed))
    def keep(g, P):
        if len(P) > 40000: P = P[rng.choice(len(P), 40000, replace=False)]
        return P[K.keepout(g, P, FR) >= r + KCLR]
    def post_ok(g, P, halfw=r + 2.4 + 0.6, margin=float(os.environ.get("PFMARGIN", "1.5")), maxn=5000):
        """optional (env POSTFEAS=1): a tie-saddle post straight toward the host (along -grad sdf) must stay >= halfw + margin clear of the swept neighbours (cheap proxy of arm_clips.best_post_dir)"""
        if len(P) > maxn: P = P[rng.choice(len(P), maxn, replace=False)]
        h = 1.0; gv = np.stack([(L.sample_sdf(g, P + e * h) - L.sample_sdf(g, P - e * h)) / (2 * h) for e in np.eye(3)], 1); nrm = np.linalg.norm(gv, axis=1); P = P[nrm > 1e-3]; gv = gv[nrm > 1e-3] / nrm[nrm > 1e-3, None]
        ok = np.ones(len(P), bool)
        for sv in np.arange(0.0, 14.0, 2.0):
            Q = P - sv * gv; m = ok & (L.sample_sdf(g, Q) >= 3.0)
            if m.any(): ok[m] &= K.keepout(g, Q[m], FR) >= halfw + margin
        return P[ok]
    n0 = (len(PA), len(PB)); PA = keep(gA, PA); PB = keep(gB, PB)
    def comp_ok(g, P, rr=r, thr=1.5, frac=0.2):
        """optional (env CONNECT=1): keep anchors that lie in a large connected free region of the link's combined field (own solids + swept keep-out, legal poses): rejects closed pockets that no route can leave"""
        import arm_route as R
        from scipy import ndimage
        if len(P) == 0: return P
        lo_, hi_ = P.min(0) - 12.0, P.max(0) + 12.0; vf, lo_, p_ = R.field_K(g, lo_, hi_, FRL, 2.0); lab, n = ndimage.label(vf >= rr + thr)
        if n == 0: return P[:0]
        sizes = np.bincount(lab.ravel()); sizes[0] = 0; main = sizes.max()
        idx = np.clip(np.round((P - lo_) / p_).astype(int), 0, np.array(vf.shape) - 1); lb = lab[idx[:, 0], idx[:, 1], idx[:, 2]]
        return P[(lb > 0) & (sizes[lb] >= frac * main)]
    if os.environ.get("POSTFEAS") == "1":
        m0 = (len(PA), len(PB)); PA = post_ok(gA, PA); PB = post_ok(gB, PB); print(joint, "anchors after post-feasibility:", m0, "->", (len(PA), len(PB)), flush=True)
    if os.environ.get("CONNECT") == "1":
        FRL, _nl = K.legal_poses(FR, 3.0); m1 = (len(PA), len(PB)); PA = comp_ok(gA, PA); PB = comp_ok(gB, PB); print(joint, "anchors in the main connected free region:", m1, "->", (len(PA), len(PB)), flush=True)
    print(joint, "anchors after swept keep-out (>= r + %.1f):" % KCLR, n0, "->", (len(PA), len(PB)), flush=True)
    print(joint, "bundle", bd, "Rmin %.1f" % kmin, "anchors", len(PA), len(PB), flush=True)
    if len(PA) == 0 or len(PB) == 0: return dict(joint=joint, bundle=bd, found=[], note="no free anchor points")
    lo, hi = RL.HARD[joint]; good = []; t0 = time.time(); tried = 0
    while tried < n_cand and time.time() - t0 < 2400:
        A = PA[rng.integers(len(PA))]; B = PB[rng.integers(len(PB))]
        tAs = [t for t in AXES if tangent_ok(gA, A, t, r)]; tBs = [t for t in AXES if tangent_ok(gB, B, t, r)]
        if not tAs or not tBs: continue
        tA = tAs[rng.integers(len(tAs))]; tB = tBs[rng.integers(len(tBs))]; tried += 1
        ch = []
        for v in (lo, (lo + hi) / 2, hi):
            for b in (0, 1):
                F = ak.frames(q_at(joint, v, b)); ch.append(np.linalg.norm((F[gA] @ np.r_[A, 1])[:3] - (F[gB] @ np.r_[B, 1])[:3]))
        for u, off in ((1.05, 4.0), (1.25, 14.0), (1.6, 14.0)):
            cand = dict(A=A, tA=tA, B=B, tB=tB, L=max(ch) * u + off)
            if cand["L"] > 340: continue
            w = check_candidate(joint, cand, r, kmin, 3)
            if w is None: continue
            w2 = check_candidate(joint, cand, r, kmin, 9)
            if w2 is None: continue
            good.append(dict(A=A.round(1).tolist(), tA=tA.tolist(), B=B.round(1).tolist(), tB=tB.tolist(), L=round(cand["L"], 1), clr=round(w2["clr"], 2), minrad=round(w2["minrad"], 1)))
            good.sort(key=lambda d: (-min(d["clr"], 6.0), d["L"])); good = good[:10]; print(" tried", tried, "good", len(good), good[0], "%.0fs" % (time.time() - t0), flush=True); break
        if tried % 25 == 0: print(" tried", tried, "good", len(good), "%.0fs" % (time.time() - t0), flush=True)
    return dict(joint=joint, bundle=bd, k=k, kind=kind, Rmin=round(kmin, 1), tried=tried, found=good)

if __name__ == "__main__":
    j = sys.argv[1]; n = int(sys.argv[2]) if len(sys.argv) > 2 else 150; kind = sys.argv[3] if len(sys.argv) > 3 else "ultra"; seed = int(sys.argv[4]) if len(sys.argv) > 4 else 1
    kk = float(sys.argv[5]) if len(sys.argv) > 5 else None
    res = run(j, n, kind, seed, k=kk); tag = ("" if kind == "ultra" else "_" + kind) + ("" if kk is None else "_k%g" % kk) + ("" if CLR_MIN == 0.8 else "_c%g" % CLR_MIN) + ("_PF" if os.environ.get("POSTFEAS") == "1" else "") + ("C" if os.environ.get("CONNECT") == "1" else "") + "_K"
    json.dump(res, open(os.path.join(OUT, "wire_search_%s%s.json" % (j, tag)), "w"), indent=1); print(j, "found", len(res["found"]))
