"""Full-arm kinematics + mesh groups for verification and rendering (right arm, robot frame, mm).
q = dict(S1, S2, S3, E, W1, W2, W3, G) in degrees (G = gripper opening %). Transforms are 4x4 in the robot frame.
Chain: R1 = Rz(-S1); R2 = R1 Ry(-S2); R3 = R2 Rz(S3); RE = R3 T(0,0,230) Ry(-E) T(0,0,-230);
       RW1 = RE T(0,D,0) Rz(W1) T(0,-D,0); RW2 = RW1 T(0,D,Zw) Ry(-W2) T(0,-D,-Zw); RW3 = RW2 T(0,D,Zw3) Rx(W3) T(0,-D,-Zw3).
rev 2026-10-13f (unify): MW3 now holds the CLAW hand (claw_hand.py / claw_asm.py) posed at gripper opening g_pct; the old parallel-gripper
parts (hand_base, finger_a/b, tpu_a/b, g_pinion) no longer exist. The claw parts are all in the W3 frame (hand frame = W3 frame)."""
import os, sys, json, numpy as np, trimesh
from trimesh.transformations import rotation_matrix as rot, translation_matrix as tr
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.join(HERE, "..", "stl")
sys.path.insert(0, HERE)
LAY = json.load(open(os.path.join(R, "distal", "distal.json")))["_layout"]
D, ZW, ZW3, ZE = LAY["D_OFF"], LAY["Z_W"], LAY["Z_W3"], 230.0
def L(p): return trimesh.load(os.path.join(R, p + ".stl"))
def about(axis_pt, axis_dir, ang):
    return tr(axis_pt) @ rot(np.radians(ang), axis_dir) @ tr(-np.asarray(axis_pt, float))
def frames(q):
    R1 = rot(np.radians(-q["S1"]), [0, 0, 1]); R2 = R1 @ rot(np.radians(-q["S2"]), [0, 1, 0]); R3 = R2 @ rot(np.radians(q["S3"]), [0, 0, 1])
    RE = R3 @ about([0, 0, ZE], [0, 1, 0], -q["E"])
    RW1 = RE @ about([0, D, 0], [0, 0, 1], q["W1"])
    RW2 = RW1 @ about([0, D, ZW], [0, 1, 0], -q["W2"])
    RW3 = RW2 @ about([0, D, ZW3], [1, 0, 0], q["W3"])
    return dict(F=np.eye(4), M1=R1, M2=R2, M3=R3, ME=RE, MW1=RW1, MW2=RW2, MW3=RW3)
def claw_parts(g_pct=0.0, hard=None):
    """claw hand meshes (hand/W3 frame) at gripper opening g_pct (%), or at an explicit drive angle hard=theta1 (deg)."""
    import claw_asm as ca
    out, _ = ca.pose_th1(hard) if hard is not None else ca.pose(g_pct)
    return out
def groups(include_fingers=True, g_pct=0.0):
    sh = "shoulder/"; ua = "upperarm/"; ds = "distal/"
    G = {"F": {"s1_base": L(sh + "s1_base_R"), "s1_clamp": L(sh + "s1_clamp_R"), "ak_s1": L(sh + "proxies/ak_s1_body")},
         "M1": {"yoke": L(sh + "s1s2_yoke_R"), "s2_clamp": L(sh + "s2_clamp_R"), "ak_s2": L(sh + "proxies/ak_s2_body")},   # cowl (M1) and head dome (M2) added below when built
         "M2": {"s3_bracket": L(sh + "s3_bracket_R"), "s2_adapter": L(sh + "s2_adapter_R"), "s3_lid": L(sh + "s3_lid_R"), "xm_s3": L(sh + "proxies/xm_s3_body")},
         "M3": {"ua_lat": L(ua + "ua_lat_R"), "ua_med": L(ua + "ua_med_R"), "xm_elbow": L(ua + "proxies/xm_elbow")},
         "ME": {"fork_drive": L(ua + "fork_drive_R"), "fork_idler": L(ua + "fork_idler_R"), "fork_bridge": L(ua + "fork_bridge_R"),
                "root_a": L(ds + "forearm_root_a_R"), "root_b": L(ds + "forearm_root_b_R"), "sts_w1": L(ds + "proxies/sts_w1_R"), "w1_pinion": L(ds + "w1_pinion_R")},
         "MW1": {"forearm_dist": L(ds + "forearm_dist_R"), "sts_w2": L(ds + "proxies/sts_w2_R")},
         "MW2": {"w2_yoke": L(ds + "w2_yoke_R"), "sts_w3": L(ds + "proxies/sts_w3_R")},
         "MW3": {}}
    for g_, k_, p_ in (("M1", "cowl", sh + "cowl/cowl_R"), ("M2", "head", sh + "cowl/head_dome_R")):
        if os.path.exists(os.path.join(R, p_ + ".stl")): G[g_][k_] = L(p_)
    if include_fingers:
        G["MW3"].update(claw_parts(g_pct))
    else:                                          # static hand only (used while the W3 stops are placed): tray, cover, servo, tabs, shafts
        full = claw_parts(100.0)
        for k, v in full.items():
            if not k.startswith(("prox_", "dist_", "link_", "tpu_pad_")): G["MW3"][k] = v
    for g_, extra in (("F", "stops_F"), ("M1", "stops_M1"), ("M3", "stops_M3"), ("ME", "stops_ME"), ("M2", "stops_M2"), ("MW1", "stops_MW1"), ("MW2", "stops_MW2"), ("MW3", "stops_MW3")):
        d = os.path.join(R, "distal", extra)
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if f.endswith(".stl"): G[g_][f[:-4]] = trimesh.load(os.path.join(d, f))
    return G
def tip(q):
    return (frames(q)["MW3"] @ np.array([0, D, LAY["Z_TIP"], 1.0]))[:3]

# ---------------------------------------------------------------------------------------------- S2 balancer (I1b rev e), rev 2026-10-16 (gI merge)
# The balancer is installed in the combined state. Its bodies do not all ride on one arm_kin group: the anchor / tube / cap / buffer / S2 bumpers
# turn with S1 (M1), the collar / collar cap / pin B ride on the S2 output (M2), the swivel + sheave turn with S1 AND about the tube axis by
# balancer_calc.swivel_angle(S2), the piston slides inside the tube (never leaves the bore, not an obstacle), and the cord runs T2 -> B.
# Callers that want the balancer use these helpers (default groups() is unchanged so older checks reproduce):
#   install_balancer(G)      -> swaps the yoke for s1s2_yoke_bal and the rev-c TPU S2 pads for the rev-e bumpers (in place, returns G)
#   balancer_parts()         -> {name: (mesh, group)}  group in M1 / M2 / SWIVEL (piston and spring proxy excluded)
#   balancer_frames(q)       -> {name: 4x4} robot-frame transform of every balancer part at pose q
#   balancer_cord(q, r)      -> trimesh capsule of the free cord (sheave tangent T2 -> pin B), ends trimmed as in verify_balancer.py
BAL_DIR = os.path.join(R, "balancer")
def has_balancer(): return os.path.exists(os.path.join(BAL_DIR, "bal_anchor_R.stl"))
def install_balancer(G):
    p = os.path.join(BAL_DIR, "s1s2_yoke_bal_R.stl")
    if os.path.exists(p) and "yoke" in G.get("M1", {}): G["M1"]["yoke"] = trimesh.load(p)
    for g in G:
        for k in ("tpu_s2_pos_R", "tpu_s2_neg_R"): G[g].pop(k, None)
    return G
def balancer_parts(include_bumpers=True):
    import balancer_calc as bc
    out = {}
    for nm, grp in bc.PART_GROUP.items():
        if grp == "PISTON" or (not include_bumpers and nm.startswith("tpu_s2_")): continue
        p = os.path.join(BAL_DIR, nm + "_R.stl")
        if os.path.exists(p): out[nm] = (trimesh.load(p), grp)
    return out
def balancer_frames(q):
    import balancer_calc as bc
    T = frames(q); return bc.part_transforms(T["M1"], T["M2"], float(q["S2"]))
def balancer_cord(q, r=1.25):
    import balancer_calc as bc
    T = frames(q); T2, Bp, _, _ = bc.cord_points(T["M1"], T["M2"], float(q["S2"]))
    v = Bp - T2; Lc = float(np.linalg.norm(v)); u = v / Lc; p0, p1 = T2 + u * 1.5, Bp - u * 3.5; v = p1 - p0; Lc = float(np.linalg.norm(v)); u = v / Lc
    c = trimesh.creation.capsule(height=max(Lc, 1e-3), radius=r, count=[8, 8]); z = np.array([0, 0, 1.0]); ax = np.cross(z, u); s = np.linalg.norm(ax); Tm = np.eye(4)
    if s > 1e-9: Tm = rot(np.arctan2(s, z @ u), ax / s)
    Tm[:3, 3] = (p0 + p1) / 2; c.apply_transform(Tm); return c
