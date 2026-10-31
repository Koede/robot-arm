"""Verify the yokeless wrist v2: (1) W3 sweep of everything beyond W3 vs the fixed arm, (2) R sweep of the hand vs the carrier stack, per pose, (3) flexure + load numbers, (4) mass.
Pairs excluded as designed contact: pinion/bearing vs rotor/hub, flexure leaves, spindle tube."""
import sys, os, json, math; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import hand_pose as HP, wrist_rotary as W
D, ZW3 = 57.0, 455.0
ST = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl"); L = lambda p: trimesh.load(os.path.join(ST, p))
def rot(axis, deg, pivot):
    a = math.radians(deg); c, s = math.cos(a), math.sin(a); R = np.eye(4)
    if axis == "x": R[1, 1], R[1, 2], R[2, 1], R[2, 2] = c, -s, s, c
    else: R[0, 0], R[0, 1], R[1, 0], R[1, 1] = c, -s, s, c
    T1 = np.eye(4); T1[:3, 3] = -np.array(pivot); T2 = np.eye(4); T2[:3, 3] = pivot; return T2 @ R @ T1
fixed = {k: L(p) for k, p in dict(forearm_dist="distal/forearm_dist_R.stl", w2_yoke="distal/w2_yoke_R.stl", sts_w2="distal/proxies/sts_w2_R.stl", sts_w3="distal/proxies/sts_w3_R.stl").items()}
carrier_grp = {k: L(p) for k, p in dict(carrier="wrist_rotary/carrier.stl", sts_R="wrist_rotary/proxies/sts_R.stl", w3_crank="wrist_rotary/proxies/w3_crank.stl", w3_rod="wrist_rotary/proxies/w3_rod.stl").items()}
flex_grp = {k: L(f"wrist_rotary/{k}.stl") for k in ("flexure_px", "flexure_nx")}
stage = {k: L(p) for k, p in dict(pinion="wrist_rotary/pinion.stl", bearing="wrist_rotary/proxies/bearing_6808.stl").items()}
rotor_grp = {"rotor": L("wrist_rotary/rotor.stl"), "palm": L("wrist_rotary/palm_tray_wrist.stl")}
P = HP.make_poses()
def hand_meshes(pose):
    scene, _ = HP.build(pose, P[pose]); out = {}
    for n, m in HP.to_robot(scene): m = m.copy(); m.apply_translation([0, 0, W.DZ]); out[n] = m
    st = []; HP.static_scene(st)
    for n, m in st:
        if n.startswith("servo_"): m = m.copy(); m.apply_translation([0, 0, W.DZ]); out[n] = m
    return out
def mgr(d, T=None):
    cm = trimesh.collision.CollisionManager()
    for n, m in d.items(): cm.add_object(n, m, transform=T)
    return cm
res = {}
# (1) W3 sweep: carrier group + stage + rotor + hand (open pose) rotate about x through (D, 455)
hand_open = hand_meshes("open"); moving = {**carrier_grp, **flex_grp, **stage, **rotor_grp, **hand_open}
fx = mgr(fixed); w3 = {}
for ang in range(-35, 45, 5):
    T = rot("x", ang, [0, D, ZW3]); mv = mgr(moving, T)
    hit, names = mv.in_collision_other(fx, return_names=True)
    mv_nf = mgr({k: v for k, v in moving.items() if not k.startswith("flexure")}, T); dmin = mv_nf.min_distance_other(fx)             # bonded flexure brackets excluded
    fl_nf = mgr(flex_grp, T); fx_nb = mgr({k: v for k, v in fixed.items() if k != "w2_yoke"}); fdist = fl_nf.min_distance_other(fx_nb)
    bad = sorted({(a, b) for a, b in names if not (a.startswith("flexure") and b in ("w2_yoke",)) and not (a == "w3_crank" and b == "sts_w3")})
    w3[ang] = dict(min_dist=round(float(dmin), 2), flexure_to_servo_forearm=round(float(fdist), 2), collisions=[f"{a}/{b}" for a, b in bad])
res["w3_sweep_open_pose"] = w3
# (2) R sweep per pose: rotor group + hand rotate about z through (0, D)
stat = {**carrier_grp}; sm = mgr(stat); r_res = {}
for pose in P:
    hm = hand_meshes(pose); mov = {**rotor_grp, **hm}; first = {}
    ok = []
    for psi in range(-170, 175, 10):
        T = rot("z", psi, [0, D, 0]); mv = mgr(mov, T); hit, names = mv.in_collision_other(sm, return_names=True)
        bad = sorted({(a, b) for a, b in names}); dmin = float(mv.min_distance_other(sm))
        if not bad: ok.append(psi)
        first[psi] = (round(dmin, 2), [f"{a}/{b}" for a, b in bad][:4])
    r_res[pose] = dict(free_psi_deg=ok, free_range=[min(ok), max(ok)] if ok else None, n_free=len(ok), at_0=first[0], at_pm90=[first[-90], first[90]])
res["R_sweep"] = r_res

# (2b) hand digits vs the rotor stack itself (rigid on the same turntable) and vs hub/plate: index-curl scan in the fist pose
rotor_only = mgr({"rotor": rotor_grp["rotor"]}); hub_plate = mgr(carrier_grp)
def digits_hit(pose_dict):
    scene, _ = HP.build("thumbsup", pose_dict); bad = set()
    for n, m in HP.to_robot(scene):
        m = m.copy(); m.apply_translation([0, 0, W.DZ])
        for nm, cmx in (("rotor", rotor_only), ("carrier", hub_plate)):
            if cmx.in_collision_single(m): bad.add(f"{n}/{nm}")
    return sorted(bad)
res["index_curl_scan"] = {}
for th in range(0, 100, 5):
    pd = json.loads(json.dumps({k: v for k, v in P["thumbsup"].items() if k != "f"}, default=list)); pd = dict(P["thumbsup"]); pd["f"] = dict(P["thumbsup"]["f"]); pd["f"]["index"] = (th, 0)
    res["index_curl_scan"][th] = digits_hit(pd)
free = [t for t, b in res["index_curl_scan"].items() if not b]; res["index_curl_free_up_to_deg"] = max(free) if free else None
print("index curl free up to (deg MCP):", res["index_curl_free_up_to_deg"], " first hit:", {t: b for t, b in res["index_curl_scan"].items() if b and t == min(k for k, v in res["index_curl_scan"].items() if v)})
# (2c) digits + palm only vs carrier group min distance at psi 0 / +-90 (rotor/hub running gap excluded)
dd = {}
for pose in P:
    hm = {k: v for k, v in hand_meshes(pose).items()}; mv0 = {"palm": rotor_grp["palm"], **hm}
    dd[pose] = {psi: round(float(mgr(mv0, rot("z", psi, [0, D, 0])).min_distance_other(sm)), 2) for psi in (-90, 0, 90, 180)}
res["hand_to_carrier_min_dist"] = dd; print("hand->carrier min dist", dd)
# (3) flexure and loads
# two monolithic PA12 cross-strip flexures (concept C: 0.7 x 24 x 20 mm leaves, +-45 deg), one each side of the W3 servo, 68 mm apart in x
k1 = 0.00141; res["flexure"] = dict(material="PA12 (SLS/MJF)", per_flexure_k_Nm_per_deg=k1, total_k_Nm_per_deg=round(2 * k1, 5), strain_pct_30deg=0.76, strain_pct_35deg=round(0.76 * 35 / 30, 2), static_limit_pct=3.0,
    parasitic_axis_shift_mm=0.97, buckling_SF_axial_per_flexure=4.8, plate_buckling_SF_C_alone=1.49)
m_pl = 0.5; M_y = 9.81 * 2.0 * (0.328 * 0.069 + 0.176 * 0.055 + m_pl * 0.034)          # tipping about y (hand + stage + payload at the chuck axis, 2 g), lever from the +x flexure centre ~ x 34
F_cpl = M_y / 0.068                                                                    # couple over the 68 mm spacing
ax = (m_pl + 0.328 + 0.176) * 9.81 * 2.0 / 2                                           # axial per flexure
res["flexure"].update(M_y_Nm_2g=round(M_y, 2), couple_force_N=round(F_cpl, 1), axial_per_flexure_N=round(ax, 1), est_leaf_compression_N=round(ax / 2 + F_cpl / 2, 1),
                      est_plate_buckling_SF=round(1.49 * 19.6 / (ax / 2 + F_cpl / 2) , 2), roll_stiff_Nm_per_deg_each=0.454, tip_moment_x_Nm_2g=1.96,
                      drive="crank r15 + push-rod to a lug at r39: 2.6x, 78 deg servo for 30 deg hand, 7.6 N.m available vs 1.96 N.m (SF 3.9)")
res["R_stage"] = dict(ratio=W.NG / W.NP, servo_stall_Nm=2.94, output_Nm_after_gear=round(2.94 * W.NG / W.NP * 0.9, 2), holding_load_Nm_est=0.35)
meta = json.load(open(os.path.join(ST, "wrist_rotary", "wrist_rotary.json")))
res["mass_g"] = dict(carrier=meta["carrier"]["g"], rotor=meta["rotor"]["g"], pinion=meta["pinion"]["g"], flexures_2x=round(meta["flexure_px"]["g"] + meta["flexure_nx"]["g"], 1), sts_R=55.0, bearing_6808=46.0, spindle_tube_brass=4.0, crank_rod_lug=2.0, hardware=8.0)
res["mass_g"]["total_added"] = round(sum(res["mass_g"].values()), 1)
json.dump(res, open(os.path.join(ST, "wrist_rotary", "verify_wrist_rotary.json"), "w"), indent=1)
print(json.dumps(res["flexure"], indent=1)); print(json.dumps(res["mass_g"])); print(json.dumps(res["R_stage"]))
for a, v in w3.items(): print("W3", a, v["min_dist"], v["collisions"][:3])
for p, v in r_res.items(): print("R", p, v["free_range"], v["n_free"], "psi0:", v["at_0"], "+-90:", v["at_pm90"])
