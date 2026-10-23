"""Rev e (zero-length) S2 balancer: interference / clearance sweeps, two-arm reach, internal fits, mass and CG.  python-fcl via trimesh.  mm, deg, right arm, robot frame.
Kinematic bodies (balancer_calc.part_transforms):
  Yb  (M1 = Rz(-S1)):  bal_anchor (foot + web + beam + guide tube), bal_tubecap, tpu_bal_buffer, TPU S2 bumpers           [rotate with S1 only]
  SW  (M1 x swivel(S2)): bal_swivel + bal_sheave, turned about the tube axis through A so the fork points at B
  Ab  (M2 = M1 Ry(-S2)): bal_collar, bal_collar_cap, bal_pin_proxy (ISO 7379 d6 shoulder screw)
  Cd  free cord T2 -> B: capsule r 0.75 + 0.5 mm braid / flutter allowance (r 1.25)
  stage: Yo = yoke group M1 (yoke_bal, S2 clamp, AK70-S2, cowl, S2 pad carriers), Ao = arm group M2 (S3 bracket, adapter, lid, XM-S3, head dome, S3 posts)
  F = S1 base / clamp / AK70-S1 + harness (plate, clasp, wing, spine bridge, pauldron) ; W = wearer mesh ; arm = M3, ME, MW1, MW2, MW3 (claw at G 50)
Sweeps: S1 soft 10..140 x S2 soft -30..88 (all pairs incl. F / W);  arm_check (S1 = 0: the arm and the balancer both ride on S1, the relative geometry is S1-free):
  S2 {-30, 0, 45, 88} x S3 {-70, -30, 0, 30, 90} x E 0..145 step 5 x W2 {-70, 0, 80}; each hit is checked against the firmware elbow tiers (elbow_limits_15mm*.json).
Two-arm: right anchor+tube vs its mirror (left arm, plane y = -150) with both S1 at the same angle, S1 100..150 (soft max 140, hard 150).
Targets: >= 3 mm part-part, >= 2 mm to skin, < 1 mm fails.  Writes stl/balancer/verify_balancer.json.
Run: nice -n 10 python3 -I verify_balancer.py [quick]"""
import os, sys, json, time, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, rom_limits as rl, sweep_shoulder as ss, balancer_calc as bc
S = os.path.join(HERE, "..", "stl", "shoulder") + "/"; B = os.path.join(HERE, "..", "stl", "balancer") + "/"; ROOT = os.path.join(HERE, "..", "stl") + "/"
QUICK = "quick" in sys.argv
ld = lambda p: trimesh.load(p)
mgr = ss.mgr
SP = bc.SPRING; TB = bc.tube_layout()
def capsule(p0, p1, r):
    v = p1 - p0; L = float(np.linalg.norm(v)); c = trimesh.creation.capsule(height=max(L, 1e-3), radius=r, count=[8, 8])
    z = np.array([0, 0, 1.0]); u = v / L; ax = np.cross(z, u); s = np.linalg.norm(ax); T = np.eye(4)
    if s > 1e-9: T = trimesh.transformations.rotation_matrix(np.arctan2(s, z @ u), ax / s)
    T[:3, 3] = (p0 + p1) / 2; c.apply_transform(T); return c
R_CORD = 1.25

# ---- parts
G = ak.groups(include_fingers=True, g_pct=50.0)
G["M1"]["yoke"] = ld(B + "s1s2_yoke_bal_R.stl")
for k in ("tpu_s2_pos_R", "tpu_s2_neg_R"):                 # replaced by the bumpers
    for g in G: G[g].pop(k, None)
PADS = {"pad_" + k: ld(B + "s2_pad_%s_bal_R.stl" % k) if os.path.exists(B + "s2_pad_%s_bal_R.stl" % k) else ld(S + "stops/s2_pad_%s_R.stl" % k) for k in ("pos", "neg")}; G["M1"].update(PADS)   # rev f: trimmed carriers
BAL = {nm: ld(B + nm + "_R.stl") for nm in bc.PART_GROUP}
YB = [k for k, g in bc.PART_GROUP.items() if g == "M1"]; SWP = ["bal_swivel", "bal_sheave"]; AB = ["bal_collar", "bal_collar_cap", "bal_pin_proxy"]
_b = ld(S + "proxies/wearer_makehuman.stl")
HARN = {"plate": ld(B + "chassis_plate_pocketed_R.stl") if os.path.exists(B + "chassis_plate_pocketed_R.stl") else ld(S + "chassis_plate_R.stl"), "clasp": ld(S + "clasp_frame_R.stl"),
        "wing": ld(S + "scapular/wing_R.stl"), "spine": ld(S + "scapular/spine_bridge.stl"), "pauldron": ld(S + "pauldron/pauldron_R.stl")}
FIX = dict(G["F"]); FIX.update({"h_" + k: v for k, v in HARN.items()})
mYb = mgr({k: BAL[k] for k in YB}); mYbA = mgr({"bal_anchor": BAL["bal_anchor"], "bal_tubecap": BAL["bal_tubecap"], "tpu_bal_buffer": BAL["tpu_bal_buffer"]})
mSW = mgr({k: BAL[k] for k in SWP}); mSWc = mgr({"bal_swivel": BAL["bal_swivel"]})                 # cheeks/fork only (cord vs fork)
mAb = mgr({k: BAL[k] for k in AB}); mAbC = mgr({k: BAL[k] for k in ("bal_collar", "bal_collar_cap")})
mYo = mgr({k: v for k, v in G["M1"].items() if k not in ("yoke",) + tuple(PADS)})                 # yoke + carriers are the anchor / bumper mating parts: separate
mYoke = mgr({"yoke": G["M1"]["yoke"]}); mYo_all = mgr(G["M1"])
mAo = mgr({k: v for k, v in G["M2"].items() if k != "s3_lid"}); mAoB = mgr({k: v for k, v in G["M2"].items() if k not in ("s3_bracket", "s3_lid")})
mLid = mgr({"lid": G["M2"]["s3_lid"]}); mBrk = mgr({"b": G["M2"]["s3_bracket"]}); mF = mgr(FIX); mW = mgr({"body": _b})
ARMG = ("M3", "ME", "MW1", "MW2", "MW3"); mArm = {g: mgr(G[g]) for g in ARMG}

def pose(s1, s2, s3=0.0, e=0.0, w2=0.0, with_arm=False):
    T = ak.frames(dict(S1=s1, S2=s2, S3=s3, E=e, W1=0, W2=w2, W3=0, G=50)); R1, R2 = T["M1"], T["M2"]; TT = bc.part_transforms(R1, R2, s2)
    for k in YB: mYb.set_transform(k, TT[k])
    for k in ("bal_anchor", "bal_tubecap", "tpu_bal_buffer"): mYbA.set_transform(k, TT[k])
    for k in SWP: mSW.set_transform(k, TT[k])
    mSWc.set_transform("bal_swivel", TT["bal_swivel"])
    for k in AB: mAb.set_transform(k, TT[k])
    for k in ("bal_collar", "bal_collar_cap"): mAbC.set_transform(k, TT[k])
    for k in G["M1"]:
        mYo_all.set_transform(k, R1)
        if k == "yoke": mYoke.set_transform("yoke", R1)
        elif k not in PADS: mYo.set_transform(k, R1)
    for k in G["M2"]:
        if k == "s3_lid": mLid.set_transform("lid", R2); continue
        mAo.set_transform(k, R2)
        if k == "s3_bracket": mBrk.set_transform("b", R2); continue
        mAoB.set_transform(k, R2)
    T2, Bp, _, _ = bc.cord_points(R1, R2, s2)
    u = (Bp - T2) / np.linalg.norm(Bp - T2)
    mCd = mgr({"cord": capsule(T2 + u * 1.5, Bp - u * 3.5, R_CORD)})            # ends trimmed: 1.5 mm off the sheave tangent point, 3.5 mm off the pin centre (eye on the pin)
    global mCdF; mCdF = mgr({"cord": capsule(T2 + u * 12.0, Bp - u * 3.5, R_CORD)})   # for the fork check: the first 12 mm run between the cheeks by design (sheave block)
    if with_arm:
        for g in ARMG:
            for k in G[g]: mArm[g].set_transform(k, T[g])
    return mCd

def dist(m1, m2): return float(m1.min_distance_other(m2))
CLS = ["Yb_F", "Yb_W", "SW_F", "SW_W", "Ab_F", "Ab_W", "Cd_F", "Cd_W",
       "Ab_Yo", "Ab_Yoke", "Ab_Yb", "Ab_SW", "Yb_Ao", "SW_Ao", "SW_Yo", "SW_Yoke", "Cd_Yb", "Cd_Yo", "Cd_Ao", "Cd_AbCollar", "Cd_SWfork", "Yb_Lid", "Ab_Brk", "Yb_Yo", "SW_Yb_running"]
MATING = ("Yb_Yo", "Ab_Brk", "SW_Yb_running")              # designed seats / running gaps, reported but not gated at 3 mm
def sweep():
    (a0, a1), (b0, b1) = rl.SOFT["S1"], rl.SOFT["S2"]
    S1v = [float(v) for v in np.linspace(a0, a1, 5 if QUICK else 12)]; S2v = [float(v) for v in np.linspace(b0, b1, 8 if QUICK else 30)]
    best = {c: (1e9, None) for c in CLS}; per = {c: {} for c in CLS}; t0 = time.time()
    for s1 in S1v:
        for s2 in S2v:
            mCd = pose(s1, s2)
            d = {"Yb_F": dist(mYb, mF), "Yb_W": dist(mYb, mW), "SW_F": dist(mSW, mF), "SW_W": dist(mSW, mW), "Ab_F": dist(mAb, mF), "Ab_W": dist(mAb, mW), "Cd_F": dist(mCd, mF), "Cd_W": dist(mCd, mW),
                 "Ab_Yo": dist(mAb, mYo), "Ab_Yoke": dist(mAb, mYoke), "Ab_Yb": dist(mAb, mYb), "Ab_SW": dist(mAb, mSW), "Yb_Ao": dist(mYbA, mAo), "SW_Ao": dist(mSW, mAo), "SW_Yo": dist(mSW, mYo),
                 "SW_Yoke": dist(mSW, mYoke), "Cd_Yb": dist(mCd, mYb), "Cd_Yo": dist(mCd, mYo_all), "Cd_Ao": dist(mCd, mAo), "Cd_AbCollar": dist(mCd, mAbC), "Cd_SWfork": dist(mCdF, mSWc),
                 "Yb_Lid": dist(mYbA, mLid), "Ab_Brk": dist(mAb, mBrk), "Yb_Yo": dist(mYb, mYo_all), "SW_Yb_running": dist(mSW, mYbA)}
            for c, v in d.items():
                per[c][(s1, s2)] = v
                if v < best[c][0]: best[c] = (v, (s1, s2))
    print("sweep %.0fs" % (time.time() - t0), flush=True)
    return S1v, S2v, best, per

def first_contact(per, S1v, S2v, thr):
    out = {}
    for c, tab in per.items():
        hits = []
        for s1 in S1v:
            below = [s2 for s2 in S2v if tab[(s1, s2)] < thr]
            if below: hits.append((s1, min(below), max(below)))
        out[c] = hits
    return out

def _el_tables():
    def ld_(f):
        p = ROOT + f
        if not os.path.exists(p): return None
        d = json.load(open(p)); return {tuple(float(x) for x in k.split(",")): float(v) for k, v in d.items()}
    return ld_("elbow_limits_15mm.json"), ld_("elbow_limits_15mm_restricted.json")
def _allowed(tab, s1, s2, e):
    """permissive: the balancer-arm geometry does not depend on S1, so a pose counts as reachable if ANY S1 row allows E at this S2
    (limit of a row = min of the two table cells bracketing S2)"""
    if tab is None: return True
    s2k = sorted(set(k[1] for k in tab)); lo = max([v for v in s2k if v <= s2] or [s2k[0]]); hi = min([v for v in s2k if v >= s2] or [s2k[-1]])
    rows = sorted(set(k[0] for k in tab)); lim = max(min(tab.get((r, lo), -1), tab.get((r, hi), -1)) for r in rows); return e <= lim
def arm_check():
    ELF, ELR = _el_tables()
    S2s = (-30.0, 0.0, 45.0, 88.0); S3s = (-70.0, -30.0, 0.0, 30.0, 90.0); W2s = (-70.0, 0.0, 80.0)
    if QUICK: S3s = (-70.0, 0.0, 90.0); W2s = (0.0,)
    dmin = {}; first3 = {}; hits_all = []; t0 = time.time()
    for s2 in S2s:
        for s3 in S3s:
            for w2 in W2s:
                f3 = {}
                for e in np.arange(0.0, 150.0, 5.0):
                    mCd = pose(0.0, s2, s3, float(e), w2, True)
                    for nm, m in (("Yb", mYb), ("SW", mSW), ("Ab", mAb), ("Cd", mCd)):
                        for g in ARMG:
                            if nm == "Ab" and g == "M3": continue                     # collar rides on M2 next to M3: covered by the stage sweep (S3 lug) below
                            d = dist(m, mArm[g]); key = nm + "_" + g
                            if d < dmin.get(key, (1e9,))[0]: dmin[key] = (d, dict(S2=s2, S3=s3, E=float(e), W2=w2))
                            if d < 3.0:
                                f3.setdefault(key, float(e))
                                inF = _allowed(ELF, 0, s2, e); inR = (abs(s3) <= 30 and abs(w2) <= 30) and _allowed(ELR, 0, s2, e); neutral = e <= 5 and abs(s3) <= 10 and abs(w2) <= 10
                                hits_all.append(dict(pair=key, S2=s2, S3=s3, E=float(e), W2=w2, d=round(d, 2), allowed_by_tiers=bool(inF or inR or neutral)))
                first3["S2=%d,S3=%d,W2=%d" % (s2, s3, w2)] = f3
    # S3 lug (M3) vs collar over S3 -100..100
    s3lug = 1e9
    for s3 in np.arange(-100.0, 101.0, 5.0):
        pose(0.0, 0.0, float(s3), 0.0, 0.0, True); s3lug = min(s3lug, dist(mAb, mArm["M3"]))
    print("arm_check %.0fs" % (time.time() - t0), flush=True)
    allowed = [h for h in hits_all if h["allowed_by_tiers"]]
    return dict(min_mm={k: dict(d=round(v[0], 2), at=v[1]) for k, v in sorted(dmin.items())}, collar_vs_M3_over_S3_mm=round(s3lug, 2),
                n_poses_below_3mm=len(hits_all), n_below_3mm_inside_firmware_tiers=len(allowed), below_3mm_inside_tiers=allowed[:40],
                below_3mm_examples=sorted(hits_all, key=lambda h: h["d"])[:20], first_E_below_3mm=first3)

def elbow_keepout():
    """2-D firmware table E_bal(S3, W2): largest E (2.5 deg grid) with >= 3 mm between every balancer body (anchor/tube, swivel, collar/pin) and the distal arm
    (ME..MW3, claw G 50), for S3 in a 20 deg grid and W2 in bins (each bin sampled at its ends and centre), over S2 {-30, 0, 45, 88} x W1 {-80, 0, 80} x W3 {-20, 0, 30}.
    The tube lies on the S2 axis line, so the limit is (almost) S2-free.  Firmware use: E <= min over the two S3 rows bracketing S3 of the W2 bin's value.
    Writes stl/balancer/balancer_elbow_keepout.json"""
    t0 = time.time(); allD = mgr({})
    for g in ("ME", "MW1", "MW2", "MW3"):
        for k, m in G[g].items(): allD.add_object(g + ":" + k, m)
    allB = mgr({})
    for k in YB + SWP + AB: allB.add_object(k, BAL[k])
    W13 = [(w1, w3) for w1 in (-80.0, 0.0, 80.0) for w3 in (-20.0, 0.0, 30.0)] if not QUICK else [(0.0, 0.0)]
    S2s = (-30.0, 0.0, 45.0, 88.0) if not QUICK else (0.0,)
    BINS = [(-70.0, -35.0), (-35.0, 0.0), (0.0, 40.0), (40.0, 80.0)]
    def clear(s3, e, w2s):
        worst = (1e9, None)
        for s2 in S2s:
            for w2 in w2s:
                for (w1, w3) in W13:
                    T = ak.frames(dict(S1=0.0, S2=s2, S3=s3, E=e, W1=w1, W2=w2, W3=w3, G=50)); TT = bc.part_transforms(T["M1"], T["M2"], s2)
                    for k in YB + SWP + AB: allB.set_transform(k, TT[k])
                    for g in ("ME", "MW1", "MW2", "MW3"):
                        for k in G[g]: allD.set_transform(g + ":" + k, T[g])
                    d = float(allB.min_distance_other(allD))
                    if d < 3.0: return (d, dict(S2=s2, W1=w1, W2=w2, W3=w3))
                    if d < worst[0]: worst = (d, dict(S2=s2, W1=w1, W2=w2, W3=w3))
        return worst
    tab = {}
    for s3 in np.arange(-70.0, 90.1, 20.0):
        for (b0, b1) in BINS:
            w2s = (b0, 0.5 * (b0 + b1), b1); e = 145.0; w = clear(s3, e, w2s)
            while w[0] < 3.0 and e > 0: e -= 5.0; w = clear(s3, e, w2s)
            if e < 145.0:
                e2 = e + 2.5; w2_ = clear(s3, e2, w2s)
                if w2_[0] >= 3.0: e, w = e2, w2_
            tab["S3=%g,W2=%g..%g" % (s3, b0, b1)] = dict(S3=float(s3), W2_bin=[b0, b1], E_max_deg=float(e), min_mm_at_E_max=round(w[0], 2), at=w[1])
        print("  S3 %g:" % s3, [tab["S3=%g,W2=%g..%g" % (s3, b0, b1)]["E_max_deg"] for b0, b1 in BINS], flush=True)
    restricted = {k: v for k, v in tab.items() if v["E_max_deg"] < 145.0}
    out = dict(rule="E <= E_bal(S3, W2): use the lower of the two S3 rows bracketing S3 and the bin containing W2; any S1 / S2 / W1 / W3 (sampled)", table=tab,
               restricted_cells=restricted, n_restricted=len(restricted), min_E_max=min(v["E_max_deg"] for v in tab.values()), _runtime_s=round(time.time() - t0))
    json.dump(out, open(B + "balancer_elbow_keepout.json", "w"), indent=1, default=float); return out

def two_arm():
    """right anchor+tube (Yb) vs its own mirror about y = -150 (left arm), both S1 equal"""
    Mir = np.diag([1, -1, 1, 1.0]); Mir[1, 3] = -300.0
    out = {}
    for s1 in (100.0, 120.0, 130.0, 140.0, 145.0, 150.0):
        R1 = ak.frames(dict(S1=s1, S2=0, S3=0, E=0, W1=0, W2=0, W3=0, G=0))["M1"]
        a = mgr({"t": BAL["bal_anchor"]}); a.set_transform("t", R1)
        mL = BAL["bal_anchor"].copy(); mL.apply_transform(Mir @ R1)
        if mL.volume < 0: mL.invert()
        b = mgr({"t": mL}); out["S1=%g" % s1] = round(dist(a, b), 2)
    return out

def bumper_contact():
    out = {}
    for kind, rng in (("tpu_s2_pos_bump", np.arange(84.0, 96.0, 0.25)), ("tpu_s2_neg_bump", np.arange(-26.0, -40.0, -0.25))):
        lid = mgr({"lid": G["M2"]["s3_lid"]}); mb = mgr({kind: BAL[kind]}); first = None
        for s2 in rng:
            T = ak.frames(dict(S1=0, S2=float(s2), S3=0, E=0, W1=0, W2=0, W3=0, G=0)); lid.set_transform("lid", T["M2"]); mb.set_transform(kind, T["M1"])
            if first is None and float(lid.min_distance_other(mb)) <= 0.05: first = float(s2)
        out[kind] = dict(first_contact_S2_deg=first, soft_limit=88.0 if "pos" in kind else -30.0, hard_stop=91.0 if "pos" in kind else -35.0)
    return out

def internal():
    """fits inside the tube: piston travel, spring to tube bore, cord through the swivel bore, swivel angle range"""
    t = np.arange(bc.S2_NEG_STOP, bc.S2_BACKSTOP + 1e-9, 1.0); py = np.array([TB["piston_y_S2_0"] + bc.piston_shift(x) for x in t])
    phi = np.degrees([bc.swivel_angle(x) for x in t])
    return dict(piston_y_range=[round(float(py.min()), 1), round(float(py.max()), 1)], piston_to_buffer_gap_at_S2_0_mm=round(TB["inner_end_y"] - TB["buffer_t"] - (TB["piston_y_S2_0"] + TB["piston_t"]), 2),
                spring_margin_to_solid_at_backstop_mm=SP["margin_to_solid_at_94_mm"], spring_OD_at_solid=SP["OD_at_solid"], tube_ID=TB["ID"],
                spring_to_bore_radial_clearance_mm=round((TB["ID"] - SP["OD_at_solid"]) / 2, 2), cord_bore_mm=4.0, cord_d_mm=bc.CORD["d"],
                swivel_angle_deg_range=[round(float(phi.min()), 1), round(float(phi.max()), 1)], swivel_sweep_deg=round(float(phi.max() - phi.min()), 1),
                running_gap_fork_to_cap_mm=round(bc.tube_layout()["cap_y0"] - 68.75, 2),
                cap_hub_end_to_piston_spigot_at_backstop_mm=round(TB["seat_y"] + SP["L_at_S2_94"] - 3.0 - TB["hub_y1"], 2),
                spring_ID_to_hub_radial_mm=round((SP["ID"] - TB["hub_OD"]) / 2, 2))

def mass_cg():
    pj = json.load(open(B + "balancer_parts.json"))
    TT0 = bc.part_transforms(np.eye(4), np.eye(4), 0.0)
    def item(nm): m = BAL[nm]; c = (TT0[nm] @ np.r_[m.center_mass, 1.0])[:3]; return (pj[nm]["mass_g"], c)
    spring = (SP["mass_g"], np.array([bc.A_X, (TB["seat_y"] + TB["piston_y_S2_0"]) / 2, bc.A_Z]))
    brg = (2 * 5.5 + 1.2, np.array([bc.A_X, 75.0, bc.A_Z]))                                       # 2 x 6800ZZ (5.5 g) + 693ZZ
    hw_Y = (6 * 1.6 + 6 * 0.5 + 3 * 0.9 + 3 * 0.3 + 4.5 + 0.6 + 1.0 + 0.4, np.array([0.0, 60.0, -30.0]))   # 6 M4x10 BHCS + inserts, 3 M3x8 + inserts, M4x30 adjuster + nuts, M4 insert, e-clip, pin
    bumps = [(pj["bumpers"][k]["mass_g_TPU"], BAL["tpu_s2_%s_bump" % k].center_mass) for k in ("pos", "neg")]
    Y_items = [item(n) for n in ("bal_anchor", "bal_tubecap", "tpu_bal_buffer", "bal_piston", "bal_swivel", "bal_sheave")] + [spring, brg, hw_Y] + bumps
    pin = (3.2 + 1.5 + 0.6 + 0.9, np.array([bc.B_X, 50.0, bc.B_Z]))                                       # ISO 7379 d6 x 10 M5 (~3.2 g) + spacer, 2 washers, M5 insert
    hw_A = (4 * 2.2 + 4 * 0.4 + 2 * 2.0, np.array([0.0, 0.0, 88.0]))                               # collar: 4 M4x16, 4 inserts, 2 TPU shims
    cord = (0.6, np.array([0.0, 50.0, 40.0]))
    A_items = [item(n) for n in ("bal_collar", "bal_collar_cap")] + [pin, hw_A, cord]
    def agg(it): M = sum(m for m, _ in it); c = sum(m * np.asarray(p) for m, p in it) / M; return M, c
    MY, cY = agg(Y_items); MA, cA = agg(A_items)
    return dict(masses_g={n: pj[n]["mass_g"] for n in ("bal_anchor", "bal_tubecap", "tpu_bal_buffer", "bal_piston", "bal_swivel", "bal_sheave", "bal_collar", "bal_collar_cap")} |
                dict(spring=SP["mass_g"], bearings=round(brg[0], 1), hardware_Y=round(hw_Y[0], 1), bumpers=round(sum(b[0] for b in bumps), 1), pin_set=round(pin[0], 1), hardware_A=round(hw_A[0], 1), cord=cord[0]),
                density_note="printed parts at solid density (PA12-CF 1.20, TPU 1.21; conservative vs 60 % infill), 7075 2.81, POM 1.41, steel 7.85",
                yoke_side_g=round(float(MY), 1), yoke_side_cg=[round(float(x), 1) for x in cY], arm_side_g=round(float(MA), 1), arm_side_cg=[round(float(x), 1) for x in cA],
                arm_side_cg_xz=[round(float(cA[0]), 1), round(float(cA[2]), 1)], total_per_arm_g=round(float(MY + MA), 1), total_both_arms_g=round(float(2 * (MY + MA)), 1),
                rev_d_total_per_arm_g=230.8)

if __name__ == "__main__":
    t0 = time.time(); res = dict(rev="e")
    S1v, S2v, best, per = sweep()
    res["sweep_grid"] = dict(S1=S1v, S2=S2v)
    res["min_distance_mm"] = {c: dict(d=round(v[0], 2), S1_S2=v[1]) for c, v in best.items()}
    fc3 = first_contact(per, S1v, S2v, 3.0); fc1 = first_contact(per, S1v, S2v, 1.0)
    res["poses_below_3mm"] = {c: v for c, v in fc3.items() if v and c not in MATING}; res["poses_below_1mm"] = {c: v for c, v in fc1.items() if v and c not in MATING}
    res["two_arm_tube_mm"] = two_arm(); print("two-arm", res["two_arm_tube_mm"], flush=True)
    res["internal"] = internal()
    res["bumper_first_contact"] = bumper_contact()
    res["mass"] = mass_cg()
    res["arm_check"] = arm_check()
    res["elbow_keepout"] = elbow_keepout()
    skin = min(best[c][0] for c in CLS if c.endswith("_W")); pp = min(best[c][0] for c in CLS if not c.endswith("_W") and c not in MATING)
    arm_pp = min(v["d"] for v in res["arm_check"]["min_mm"].values())
    res["gates"] = dict(skin_min_mm=round(skin, 2), skin_ge_2=bool(skin >= 2.0), part_part_min_mm=round(pp, 2), part_part_ge_3=bool(pp >= 3.0), part_part_fail_lt_1=bool(pp < 1.0),
                        arm_min_mm_raw=round(arm_pp, 2), arm_below_3mm_inside_tiers=res["arm_check"]["n_below_3mm_inside_firmware_tiers"],
                        elbow_keepout_restricted_cells=res["elbow_keepout"]["n_restricted"], E_bal_min=res["elbow_keepout"]["min_E_max"],
                        two_arm_soft_S1_140_mm=res["two_arm_tube_mm"]["S1=140"], two_arm_hard_S1_150_mm=res["two_arm_tube_mm"]["S1=150"])
    res["_runtime_s"] = round(time.time() - t0)
    json.dump(res, open(B + "verify_balancer.json", "w"), indent=1, default=lambda o: float(o) if isinstance(o, np.floating) else str(o))
    print(json.dumps({k: res[k] for k in ("min_distance_mm", "gates", "two_arm_tube_mm", "internal", "bumper_first_contact", "mass")}, indent=1, default=str))
    print(json.dumps({k: v for k, v in res["arm_check"].items() if k in ("min_mm", "n_poses_below_3mm", "n_below_3mm_inside_firmware_tiers", "collar_vs_M3_over_S3_mm", "below_3mm_inside_tiers")}, indent=1, default=str))
