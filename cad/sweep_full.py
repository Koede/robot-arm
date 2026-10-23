"""Full-arm sweep (replaces sweep_arm.py for the I6 arm): every arm group (M2..MW3) vs the wearer, the harness (plate, clasp, wing,
spine bridge), the fixed stage (F) and the S1-side parts (M1), over S1 x S2 (soft box) x S3 x E, wrist neutral and at its extremes.
Produces the elbow software-limit table elbow_limits_15mm.json (largest E with >= 15 mm margin to body/harness and >= 60 mm to head/neck for every S3 and wrist tested)
and the minimum clearances. Writes stl/sweep_full.json.
rev 2026-10-13f (unify): the hand is the claw (arm_kin MW3), the sweep runs the claw at G 0 %; its envelope is the same at every G (x +-54.5, y 34.2..90.8, z max 562.1 / 563.5 / 562.5 at G 0 / 50 / 100: 1.4 mm spread, far below the 15 mm margin), so one G is representative. Wrist at neutral and the extremes of the wrist box; the claw is 165 mm long from W3, so the tables are regenerated with it."""
import os, sys, json, itertools, time, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, rom_limits as rl, sweep_shoulder as ss
S = os.path.join(HERE, "..", "stl", "shoulder") + "/"
G = ak.groups()
# rev 2026-10-16 (gI merge): the S2 balancer (I1b rev e) is installed. Its bodies are obstacles for the arm: anchor / tube / cap / buffer / bumpers
# (turn with S1), swivel + sheave (S1 + swivel angle), collar / cap / pin B (S2 output, M2) and the free cord (capsule r 1.25). They enter the table
# gate like the stage: >= 3 mm to every arm group ME..MW3; the upper arm M3 is checked against all of them except the collar set (it clamps on
# the S3 sleeve next to M3; collar vs the S3 lug is verify_balancer's collar_vs_M3_over_S3 check). BAL=0 in the environment restores the old sweep.
USE_BAL = ak.has_balancer() and os.environ.get("BAL", "1") != "0"
if USE_BAL: ak.install_balancer(G); BALP = ak.balancer_parts()
# rev 2026-10-13: head + neck split out of the body and given the 60 mm keep-out margin (ISO/TS 15066 allows no transient contact
# with the face/skull, and the straight-arm keep-out table does not cover a folded forearm); the rest of the body keeps 15 mm.
_b = trimesh.load(S + "proxies/wearer_makehuman.stl"); _SP = json.load(open(S + "scapular/scapular_frame.json"))["spine_y"]
_c = _b.triangles_center; _hd = (_c[:, 2] > -110.0) & (np.abs(_c[:, 1] - _SP) < 110.0)
EXT = {"body": _b.submesh([np.where(~_hd)[0]], append=True)}; HEAD = {"head": _b.submesh([np.where(_hd)[0]], append=True)}
HEAD_MARG = 60.0
HARN = {"plate": trimesh.load(S + "chassis_plate_R.stl"), "clasp": trimesh.load(S + "clasp_frame_R.stl"), "wing": trimesh.load(S + "scapular/wing_R.stl"),
        "spine": trimesh.load(S + "scapular/spine_bridge.stl"), "pauldron": trimesh.load(S + "pauldron/pauldron_R.stl")}
ARM = ["M3", "ME", "MW1", "MW2", "MW3"]
MG = {k: ss.mgr(G[k]) for k in ARM}; mB = ss.mgr(EXT); mHd = ss.mgr(HEAD); mH = ss.mgr(HARN); mF = ss.mgr(G["F"]); mM1 = ss.mgr(G["M1"])
if USE_BAL:
    mBal = ss.mgr({k: v[0] for k, v in BALP.items()}); mBalN = ss.mgr({k: v[0] for k, v in BALP.items() if v[1] != "M2"})
def setq(q):
    T = ak.frames(q)
    for g in ARM:
        for k in G[g]: MG[g].set_transform(k, T[g])
    for k in G["M1"]: mM1.set_transform(k, T["M1"])
    if USE_BAL:
        TB = ak.balancer_frames(q)
        for k in BALP:
            mBal.set_transform(k, TB[k])
            if BALP[k][1] != "M2": mBalN.set_transform(k, TB[k])
        cord = ak.balancer_cord(q)
        for m_ in (mBal, mBalN):
            if "cord" in m_._objs: m_.remove_object("cord")
            m_.add_object("cord", cord)
def check():
    dB = min(MG[g].min_distance_other(mB) for g in ARM)
    dH = min(MG[g].min_distance_other(mH) for g in ARM)
    dF = min(MG[g].min_distance_other(mF) for g in ARM[1:])        # M3 vs F is checked in verify_shoulder (upper arm root)
    dM1 = min(MG[g].min_distance_other(mM1) for g in ARM[1:])
    dHd = min(MG[g].min_distance_other(mHd) for g in ARM)
    dBal = 1e3
    if USE_BAL: dBal = min([MG["M3"].min_distance_other(mBalN)] + [MG[g].min_distance_other(mBal) for g in ARM[1:]])
    return dB, dH, dF, dM1, dHd, dBal
if __name__ == "__main__":
    (a0, a1), (b0, b1) = rl.SOFT["S1"], rl.SOFT["S2"]
    S1v = [float(v) for v in np.linspace(a0, a1, 6)]; S2v = [float(v) for v in np.linspace(b0, b1, 8)]; S3v = [-70.0, 0.0, 90.0]; Ev = [0.0, 30.0, 60.0, 90.0, 120.0, 145.0]
    wrists = [dict(W1=0, W2=0, W3=0), dict(W1=80, W2=80, W3=30), dict(W1=-80, W2=-70, W3=-20)]
    # --restricted: the head-proximal mode. In cells where the full table gives no safe elbow angle, the firmware restricts
    # S3 to +-30, W2 to +-30 and W3 to +-15 and uses this second table instead (RESTRICT in control.h).
    RESTRICTED = "--restricted" in sys.argv
    if RESTRICTED:
        S3v = [-30.0, 0.0, 30.0]; wrists = [dict(W1=0, W2=0, W3=0), dict(W1=80, W2=30, W3=15), dict(W1=-80, W2=-30, W3=-15)]
    t0 = time.time(); res = {}
    for s1, s2, s3, e, wi in itertools.product(S1v, S2v, S3v, Ev, range(len(wrists))):
        q = dict(S1=s1, S2=s2, S3=s3, E=e, G=0, **wrists[wi]); setq(q); res[(s1, s2, s3, e, wi)] = check()
    MARG = 15.0; lim = {}; table = []
    for s1 in S1v:
        row = []
        for s2 in S2v:
            ok = -1
            for e in Ev:
                if all(min(res[(s1, s2, s3, e, wi)][0], res[(s1, s2, s3, e, wi)][1]) >= MARG and min(res[(s1, s2, s3, e, wi)][2], res[(s1, s2, s3, e, wi)][3]) >= 3.0
                       and res[(s1, s2, s3, e, wi)][4] >= HEAD_MARG and res[(s1, s2, s3, e, wi)][5] >= 3.0
                       for s3 in S3v for wi in range(len(wrists))): ok = e
                else: break
            row.append(ok); lim[f"{s1:.0f},{s2:.0f}"] = ok
        table.append(row)
    allv = list(res.values())
    free = [v for v in allv if min(v[:4]) > 0]
    out = dict(poses=len(res), seconds=round(time.time() - t0), min_body_mm=round(min(v[0] for v in allv), 1), min_harness_mm=round(min(v[1] for v in allv), 1),
               min_stage_mm=round(min(v[2] for v in allv), 1), min_M1_mm=round(min(v[3] for v in allv), 1), min_head_mm=round(min(v[4] for v in allv), 1),
               poses_head_inside_60mm=sum(1 for v in allv if v[4] < HEAD_MARG),
               poses_with_contact=sum(1 for v in allv if min(v) <= 0), balancer_included=bool(USE_BAL), min_balancer_mm=round(min(v[5] for v in allv), 1),
               poses_balancer_below_3mm=sum(1 for v in allv if v[5] < 3.0), poses_balancer_contact=sum(1 for v in allv if v[5] <= 0),
               S1=S1v, S2=S2v, E_tested=Ev, elbow_max_15mm=table)
    sfx = "_restricted" if RESTRICTED else ""
    json.dump(lim, open(os.path.join(HERE, "..", "stl", f"elbow_limits_15mm{sfx}.json"), "w"), indent=1)
    json.dump(out, open(os.path.join(HERE, "..", "stl", f"sweep_full{sfx}.json"), "w"), indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != "elbow_max_15mm"}, indent=1))
    print("E max (deg) with 15 mm margin to body+harness, 60 mm to head/neck and 3 mm to stage and balancer, worst over S3 and wrist, rows S1, cols S2")
    print("S1\\S2 " + " ".join("%5.0f" % s for s in S2v))
    for s1, row in zip(S1v, table): print("%5.0f " % s1 + " ".join("%5d" % v for v in row))
