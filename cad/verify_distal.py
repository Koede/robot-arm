"""Distal-arm gate (I6): per-joint first-contact scans (expect the TPU facing exactly at the hard stop, nothing inside the soft range),
cross-joint corner checks, minimum static clearances, torque margins, pinch forces, masses. Writes stl/distal/verify_distal.json.
Designed contacts excluded: W1 pinion <-> internal ring gear (meshing teeth), elbow cheek <-> housing axial running gap (1.0 mm),
servo horn faces on their yoke arms.
rev 2026-10-13f (unify): hand = claw (arm_kin MW3). Masses and torques are now summed from the actual meshes (volume x density, centroids) instead of
the old scalar hand estimate. Torque safety factors: SF = rated / required, gate SF >= 2.
Densities (g/cm3): PETG 1.27, PETG-CF 1.30, PA-CF 1.20, TPU 95A 1.21. Bought: STS3215 55 g, XM540 165 g, 6808-2RS 22 g, AK70-10 621 g."""
import os, sys, json, itertools, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, rom_limits as rl, sweep_shoulder as ss
G = ak.groups(); MG = {k: ss.mgr(v) for k, v in G.items()}
EXCL = {frozenset(("w1_pinion", "forearm_dist"))}
def setq(q):
    T = ak.frames(q)
    for g, m in MG.items():
        for k in G[g]: m.set_transform(k, T[g])
def pairs_hit(a, b):
    h, n = MG[a].in_collision_other(MG[b], return_names=True)
    return [p for p in sorted(set(map(tuple, n))) if frozenset(p) not in EXCL] if h else []
CHAIN = ["M2", "M3", "ME", "MW1", "MW2", "MW3"]
JOINT_GROUP = {"S3": "M3", "E": "ME", "W1": "MW1", "W2": "MW2", "W3": "MW3"}
def downstream(g): return CHAIN[CHAIN.index(g):]
def upstream(g): return CHAIN[:CHAIN.index(g)]
BASE = dict(S1=45.0, S2=25.0, S3=0.0, E=60.0, W1=0.0, W2=0.0, W3=0.0, G=0.0)
def scan(j, base, step=0.5):
    out = {}
    g = JOINT_GROUP[j]; lo, hi = rl.SOFT[j]; HL, HH = rl.HARD[j]
    for side, start, end in ((+1, base[j], HH + 15.0), (-1, base[j], HL - 15.0)):
        first = None
        for a in np.arange(start, end, side * step):
            q = dict(base); q[j] = float(a); setq(q); hits = []
            for dg in downstream(g):
                for ug in upstream(g): hits += pairs_hit(dg, ug)
            if hits: first = (float(a), hits); break
        out["pos" if side > 0 else "neg"] = first
    return out
res = {"scans": {}, "pass": {}}
for j in ("S3", "E", "W1", "W2", "W3"):
    base = dict(BASE); base[j] = 0.5 * sum(rl.SOFT[j])
    sc = scan(j, base); res["scans"][j] = sc
    ok = True
    for tag, hard in (("pos", rl.HARD[j][1]), ("neg", rl.HARD[j][0])):
        f = sc[tag]
        if f is None: ok = False; continue
        a, hits = f; tpu_first = all(any(x.startswith("tpu") for x in p) for p in hits)
        ok &= tpu_first and abs(a - hard) <= 1.0
    res["pass"][j] = bool(ok)
    print(j, {k: (v[0], v[1]) if v else None for k, v in sc.items()}, "PASS" if ok else "FAIL")
# corners: every combination of soft-limit extremes for E, W1, W2, W3 (S3 0) must be contact-free between non-adjacent groups too
bad = []
for e, w1, w2, w3 in itertools.product(rl.SOFT["E"], rl.SOFT["W1"], rl.SOFT["W2"], rl.SOFT["W3"]):
    q = dict(BASE, E=e, W1=w1, W2=w2, W3=w3); setq(q); hits = []
    for a_, b_ in itertools.combinations(CHAIN, 2): hits += pairs_hit(a_, b_)
    if hits: bad.append((dict(E=e, W1=w1, W2=w2, W3=w3), hits))
res["corner_contacts"] = bad; res["pass"]["corners"] = not bad
print("soft-corner contacts:", len(bad), bad[:4])
# static clearances at the park pose between adjacent groups
setq(dict(rl.PARK)); cl = {}
for a_, b_ in zip(CHAIN[:-1], CHAIN[1:]):
    d, n = MG[b_].min_distance_other(MG[a_], return_names=True); cl[f"{b_}-{a_}"] = (round(float(d), 2), list(n))
res["park_adjacent_clearance"] = cl; print("park clearances:", cl)

# ================================================================== masses and centres (neutral pose = arm straight along +z), from the meshes
LAY = ak.LAY; ZE, ZW, ZW3 = ak.ZE, ak.ZW, ak.ZW3
DENS_CLAW = dict(petg_cf=1.30, pa_cf=1.20, petg=1.27, tpu=1.21)
CLAW_MAT = {"hand_tray": "petg_cf", "gear_cover": "petg_cf", "servo_tab": "petg_cf", "claw_pinion": "pa_cf", "claw_idler": "pa_cf", "claw_sector_A": "pa_cf", "claw_sector_B": "pa_cf",
            "tpu_stop_sleeve": "tpu", "spacer_A_mid": "petg", "spacer_A_top": "petg", "spacer_B_low": "petg", "spacer_B_top": "petg"}
for n in ("A1", "A2", "T"): CLAW_MAT.update({"prox_" + n: "pa_cf", "dist_" + n: "pa_cf", "link_" + n: "pa_cf", "tpu_pad_" + n: "tpu"})
# C3 punch-ring claw names (rev 2026-10-14h; trailing finger index stripped by dens()): materials as in verify_claw.py / SPEC_claw_hand.md
CLAW_MAT.update({"prox_": "pa_cf", "dist_": "pa_cf", "link_drive_": "pa_cf", "link_couple_": "pa_cf", "claw_rotor": "pa_cf", "claw_ring": "pa_cf", "stop_cap": "petg_cf"})
def dens(k):
    if k.startswith("tpu"): return 1.21
    base = k.rstrip("0123456789")
    return DENS_CLAW[CLAW_MAT[base]] if base in CLAW_MAT else 1.27
PROXY = ("sts", "xm_", "ak_", "shaft", "pin_", "stop_pin", "bearing", "scr_")   # rev 2026-10-15: scr_* fastener proxies are steel, counted in the hardware line     # rev 2026-10-14h: all C3 pin proxies are steel, counted in the hardware line below (8.4 g, verify_claw.json)
items = {g: [] for g in CHAIN}          # (name, mass_kg, z_cg_mm)
for g in CHAIN:
    for k, m in G[g].items():
        if k.startswith(PROXY): continue
        vol = abs(m.volume) / 1e3
        if vol <= 0: continue
        c = m.center_mass if m.is_watertight else m.bounds.mean(0)
        items[g].append((k, vol * dens(k) / 1e3, float(c[2])))
def prox_z(g, key): return float(G[g][key].bounds.mean(0)[2])
SERVO = 0.055; XM = 0.165
# rev 2026-10-15: the claw numbers are REQUIRED (verify_claw.py must run first). Previously a missing verify_claw.json silently dropped res['gripper'] and
# safety_limits.py died with KeyError 'gripper'.
CJ = os.path.join(HERE, "..", "stl", "distal", "claw", "verify_claw.json")
if not os.path.exists(CJ): sys.exit("verify_distal.py: stl/distal/claw/verify_claw.json missing -> run cad/verify_claw.py first (pipeline order)")
CV = json.load(open(CJ)); HW_CLAW = CV["mass_hardware_g"] / 1e3
bought = {"M2": [("xm_s3", XM, prox_z("M2", "xm_s3"))], "M3": [("xm_elbow", XM, prox_z("M3", "xm_elbow"))],
          "ME": [("sts_w1", SERVO, prox_z("ME", "sts_w1")), ("wiring/fasteners ME", 0.025, 330.0)],
          "MW1": [("sts_w2", SERVO, prox_z("MW1", "sts_w2")), ("6808-2RS", 0.022, float(LAY["BRG_Z0"]) + 3.5), ("wiring/fasteners MW1", 0.030, 380.0)],
          "MW2": [("sts_w3", SERVO, prox_z("MW2", "sts_w3"))],
          "MW3": [("sts_g", SERVO, prox_z("MW3", "sts_g")), ("C3 steel pins + inserts/screws (verify_claw hardware)", HW_CLAW, 520.0)]}
for g, lst in bought.items(): items[g] += lst
grp_m = {g: sum(i[1] for i in v) for g, v in items.items()}
def moment(groups, z0):
    m = sum(i[1] for g in groups for i in items[g]); M = sum(i[1] * abs(i[2] - z0) / 1e3 if False else i[1] * (i[2] - z0) / 1e3 for g in groups for i in items[g])
    return m, M
res["mass_kg"] = {g: round(v, 3) for g, v in grp_m.items()}
m_hand = grp_m["MW3"]; m_fore = grp_m["ME"] + grp_m["MW1"] + grp_m["MW2"] + grp_m["MW3"]
print("masses (kg):", res["mass_kg"], " hand %.0f g, everything distal of E %.0f g" % (m_hand * 1e3, m_fore * 1e3))
res["forearm_hand_mass_g"] = dict(total=round(m_fore * 1e3, 1), hand=round(m_hand * 1e3, 1), cg_z_mm=round(moment(["ME", "MW1", "MW2", "MW3"], 0.0)[1] / m_fore * 1e3, 1), note="everything distal of the elbow axis: ME + MW1 + MW2 + MW3 (claw hand, STS3215s, bearings, wiring estimate)")
res["printed_part_masses_g"] = {g: {i[0]: round(i[1] * 1e3, 1) for i in v} for g, v in items.items()}
# ---------------- torques (static, worst pose = link horizontal; signed CG lever from the joint axis, payload at the claw tip)
g0 = 9.81; pay = 0.5; Z_TIP = LAY["Z_TIP"]
def tau(groups, z0, pay_lever_mm):
    m, M = moment(groups, z0); return (abs(M) + pay * pay_lever_mm / 1e3) * g0, m
tau_w3, m_w3 = tau(["MW3"], ZW3, Z_TIP - ZW3)
tau_w2, m_w2 = tau(["MW2", "MW3"], ZW, Z_TIP - ZW)
tau_e, m_e = tau(["ME", "MW1", "MW2", "MW3"], ZE, Z_TIP - ZE)
tau_s2, m_s2 = tau(["M2", "M3", "ME", "MW1", "MW2", "MW3"], 0.0, Z_TIP)
ST12, XMs, AK_R, AK_P = 2.94, 10.6, 8.3, 24.8      # STS3215 12 V stall, XM540 stall @12 V, AK70-10 rated / peak
res["torque_Nm"] = dict(
    W3=round(tau_w3, 2), W2=round(tau_w2, 2), E=round(tau_e, 2), S2=round(tau_s2, 2), payload_kg=pay, tip_z_mm=Z_TIP,
    W3_SF_vs_STS3215_stall=round(ST12 / tau_w3, 2), W2_SF_vs_STS3215_stall=round(ST12 / tau_w2, 2), E_SF_vs_XM540_stall=round(XMs / tau_e, 2),
    S2_SF_vs_AK70_rated=round(AK_R / tau_s2, 2), S2_SF_vs_AK70_peak=round(AK_P / tau_s2, 2),
    W1="roll about the forearm axis; payload offset <= 20 mm -> < 0.2 N m", note="gate: SF >= 2 against rated (XM540/STS3215 stall is the only published figure, used as rated)")
print(json.dumps(res["torque_Nm"], indent=1))
res["pass"]["torque_W2"] = bool(ST12 / tau_w2 >= 2); res["pass"]["torque_W3"] = bool(ST12 / tau_w3 >= 2); res["pass"]["torque_E"] = bool(XMs / tau_e >= 2)
res["pass"]["torque_S2_rated"] = bool(AK_R / tau_s2 >= 2)
# ---------------- elbow current limit vs pinch (exposed fold begins at the forearm root, ~56 mm from the elbow axis)
tau_lim = 2 * tau_e if False else tau_e; r_exposed = 0.056
res["elbow_pinch"] = dict(current_limit_Nm=round(2 * tau_e + 0.4, 2), exposed_radius_m=r_exposed, force_at_exposed_N=round((2 * tau_e + 0.4) / r_exposed, 1), limit_hand_N=140,
                          pass_=bool((2 * tau_e + 0.4) / r_exposed <= 140))
# ---------------- gripper: use the claw verification numbers (verify_claw.json, run before this script)
g75 = CV["grip"]["0.75"]                                                                   # 0.75 N.m servo torque limit = STS register 255 (claw_limit_reg())
res["gripper"] = dict(source="verify_claw.json", claw="C3 punch-ring claw", torque_limit_Nm=0.75, torque_limit_reg=255,
                      grip_SF=g75["grip_SF"], Fn_min_N=g75["Fn_min_N"], Fn_max_N=g75["Fn_max_N"],
                      F_inward_min_N=g75["Fn_min_N"],                                        # alias read by safety_limits.py (per-finger normal force at the limit)
                      hand_mass_g=CV["mass_hand_total_g"], hand_mass_mesh_g=round(m_hand * 1e3, 1), pass_=bool(g75["grip_SF"] >= 2))
res["pass"]["claw_grip_SF2"] = res["gripper"]["pass_"]
json.dump(res, open(os.path.join(HERE, "..", "stl", "distal", "verify_distal.json"), "w"), indent=1, default=str)
print(json.dumps({k: res[k] for k in ("pass", "elbow_pinch", "gripper") if k in res}, indent=1, default=str))
