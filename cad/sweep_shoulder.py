"""Collision / clearance sweep of the shoulder stage over S1 x S2 (S3 is rotationally symmetric
about the upper-arm proxy, so it is not swept here). Right arm (+y = wearer's right, right-handed frame). Angles in degrees.
S1: rotation of the arm plane about +z, positive = lateral (R = Rz(-theta1)).
S2: rotation about +y, positive = arm swings forward (-x) then up (R = Ry(+theta2))."""
import numpy as np, trimesh, os, json, sys
from trimesh.collision import CollisionManager
from trimesh.transformations import rotation_matrix as rot

HERE = os.path.dirname(__file__)
STL = os.path.join(HERE, "..", "stl", "shoulder")
ld = lambda n, sub="": trimesh.load(os.path.join(STL, sub, n + ".stl"))

def build_proxies():
    # backplate (aluminium or CF plate, front face at x=-80) and wearer keep-out volumes
    pp = os.path.join(STL, "chassis_plate_R.stl")
    plate = trimesh.load(pp) if os.path.exists(pp) else trimesh.creation.box(bounds=[[-88, -90, -145], [-80, 85, -55]])   # contoured chassis plate (true hull), was a 480 x 345 backplate
    back_x = -118.0                                  # wearer's back surface (plate 8 + 30 foam standoff)
    torso = trimesh.creation.box(bounds=[[-338, -340, -500], [back_x, 40, -100]])
    neck = trimesh.creation.cylinder(radius=55, height=80, transform=trimesh.transformations.translation_matrix([-178, -150, -60]))
    head = trimesh.creation.cylinder(radius=90, height=180, transform=trimesh.transformations.translation_matrix([-218, -150, 70]))
    return plate, torso, neck, head

fixed_stage = {n: ld(n + ("_R" if n.startswith("s1_") and n != "s1s2_yoke" else ""), "") for n in []}  # placeholder

def load_all():
    F = {"s1_base": ld("s1_base_R"), "s1_clamp": ld("s1_clamp_R"), "ak_s1": ld("ak_s1_body", "proxies")}
    M1 = {"yoke": ld("s1s2_yoke_R"), "s2_clamp": ld("s2_clamp_R"), "ak_s2": ld("ak_s2_body", "proxies")}
    M2 = {"s3_bracket": ld("s3_bracket_R"), "s2_adapter": ld("s2_adapter_R"), "s3_lid": ld("s3_lid_R"), "xm_s3": ld("xm_s3_body", "proxies"),
          "arm": ld("upper_arm_proxy", "proxies")}
    return F, M1, M2

def mgr(d, pre=""):
    m = CollisionManager()
    for k, v in d.items():
        m.add_object(pre + k, v)
    return m

def run(s1_vals, s2_vals):
    F, M1, M2 = load_all()
    plate, torso, neck, head = build_proxies()
    mF, mM1, mM2 = mgr(F), mgr(M1), mgr(M2)
    mP = mgr({"plate": plate})
    wp = os.path.join(STL, "proxies", "wearer_makehuman.stl")
    mW = mgr({"body": trimesh.load(wp)}) if os.path.exists(wp) else mgr({"torso": torso, "neck": neck, "head": head})
    cp = os.path.join(STL, "clasp_frame_R.stl")
    mC = mgr({"clasp": trimesh.load(cp)}) if os.path.exists(cp) else None
    res = {}
    for t1 in s1_vals:
        R1 = rot(np.radians(-t1), [0, 0, 1])
        for k in M1: mM1.set_transform(k, R1)
        for t2 in s2_vals:
            R2 = R1 @ rot(np.radians(-t2), [0, 1, 0])      # arm rests along +z; positive S2 swings it forward (-x)
            for k in M2: mM2.set_transform(k, R2)
            r = {}
            r["M1_F"], r["M1_F_d"] = mM1.in_collision_other(mF), mM1.min_distance_other(mF)
            r["M2_F"], r["M2_F_d"] = mM2.in_collision_other(mF), mM2.min_distance_other(mF)
            r["M2_M1"], r["M2_M1_d"] = mM2.in_collision_other(mM1), mM2.min_distance_other(mM1)
            r["M1_P"], r["M1_P_d"] = mM1.in_collision_other(mP), mM1.min_distance_other(mP)
            r["M2_P"], r["M2_P_d"] = mM2.in_collision_other(mP), mM2.min_distance_other(mP)
            r["M1_W"] = mM1.in_collision_other(mW)
            r["M2_W"], r["M2_W_d"] = mM2.in_collision_other(mW), mM2.min_distance_other(mW)
            if mC is not None:
                r["M1_C"], r["M1_C_d"] = mM1.in_collision_other(mC), mM1.min_distance_other(mC)
                r["M2_C"], r["M2_C_d"] = mM2.in_collision_other(mC), mM2.min_distance_other(mC)
            res[(t1, t2)] = r
    return res

if __name__ == "__main__":
    s1_vals = list(range(-60, 151, 15))
    s2_vals = list(range(0, 171, 10))   # 0 = arm up (rest), 90 = arm horizontal forward, 170 = nearly down
    res = run(s1_vals, s2_vals)
    json.dump({f"{a},{b}": v for (a, b), v in res.items()}, open(os.path.join(STL, "..", "sweep_shoulder.json"), "w"))
    # character map: . ok  S stage self/ fixed-stage collision  B backplate  W wearer (keep-out)
    print("rows S1 (deg), cols S2 (deg)   . ok   S stage-vs-stage   B backplate   W wearer keep-out")
    print("S1\\S2 " + " ".join(f"{b:3d}" for b in s2_vals))
    for a in s1_vals:
        row = []
        for b in s2_vals:
            r = res[(a, b)]
            c = "."
            if r["M2_W"] or r["M1_W"]: c = "W"
            if r["M1_P"] or r["M2_P"]: c = "B"
            if r.get("M1_C") or r.get("M2_C"): c = "C"
            if r["M1_F"] or r["M2_F"] or r["M2_M1"]: c = "S"
            row.append(f"  {c}")
        print(f"{a:5d} " + " ".join(row))
    # minimum clearances over the poses that are free of stage self-collision
    ok = [(k, v) for k, v in res.items() if not (v["M1_F"] or v["M2_F"] or v["M2_M1"])]
    def mn(key): return min(v[key] for k, v in res.items())
    print("\nmin clearance over ALL poses (mm): M1-vs-fixed %.1f | M2-vs-fixed %.1f | M2-vs-M1 %.1f | M1-vs-plate %.1f | M2-vs-plate %.1f" %
          (mn("M1_F_d"), mn("M2_F_d"), mn("M2_M1_d"), mn("M1_P_d"), mn("M2_P_d")))
    n_self = sum(1 for k, v in res.items() if v["M1_F"] or v["M2_F"] or v["M2_M1"])
    print("poses with stage self-collision:", n_self, "of", len(res))
