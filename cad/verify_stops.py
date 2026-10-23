"""Verify the passive stops (python-fcl): first contact with the TPU capsule pad (should be the hard-stop angle) and first
contact with rigid material (should come later, or be the TPU fully compressed). Prints a transition log and stops_verify.json."""
import sys, os, numpy as np, trimesh, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trimesh.transformations import rotation_matrix as rot
import sweep_shoulder as ss, rom_limits as rl
S = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "stl", "shoulder", "stops") + "/"
ld = lambda n: trimesh.load(S + n + "_R.stl")
F, M1, M2 = ss.load_all()
M1s = dict(M1, s1_lug=ld("s1_lug"), pad_pos=ld("s2_pad_pos"), pad_neg=ld("s2_pad_neg"), tpu_s2_pos=ld("tpu_s2_pos"), tpu_s2_neg=ld("tpu_s2_neg"))
Fs = dict(F, post_a=ld("s1_post_a"), post_b=ld("s1_post_b"), tpu_s1_a=ld("tpu_s1_a"), tpu_s1_b=ld("tpu_s1_b"))
# rev 2026-10-16 (gI merge): S2 balancer installed: patched yoke, the rev-e crush bumpers in place of the rev-c TPU S2 pads (keys kept: tpu_s2_pos/neg),
# anchor + tube + cap + buffer on M1, collar + cap + pin B on M2, swivel + sheave posed per S2. Expected: S2+ TPU first contact at the bumper
# (designed 88.5), rigid S2+ only at the lid/yoke backstop. BAL=0 restores the old check.
import arm_kin as ak
USE_BAL = ak.has_balancer() and os.environ.get("BAL", "1") != "0"
SWV = {}
if USE_BAL:
    BALP = ak.balancer_parts(); M1s["yoke"] = trimesh.load(os.path.join(ak.BAL_DIR, "s1s2_yoke_bal_R.stl"))
    M1s["tpu_s2_pos"] = BALP["tpu_s2_pos_bump"][0]; M1s["tpu_s2_neg"] = BALP["tpu_s2_neg_bump"][0]
    for k, (m, g) in BALP.items():
        if k.startswith("tpu_s2_"): continue
        if g == "M1": M1s[("b" if k == "tpu_bal_buffer" else "") + k] = m          # buffer renamed so it is not taken for a TPU stop pad
        elif g == "M2": M2[k] = m
        elif g == "SWIVEL": SWV[k] = m
    # rev f (2026-10-16): trimmed carriers (bumper seats) + the bottomed bumpers as rigid bodies -> the rigid first contact is the real backstop
    for kind in ("pos", "neg"):
        p_ = os.path.join(ak.BAL_DIR, "s2_pad_%s_bal_R.stl" % kind)
        if os.path.exists(p_): M1s["pad_" + kind] = trimesh.load(p_)
        p_ = os.path.join(ak.BAL_DIR, "tpu_s2_%s_bump_solid_R.stl" % kind)
        if os.path.exists(p_): M1s["bumped_%s_solid" % kind] = trimesh.load(p_)       # name must not start with "tpu": rigid
mF, mM1, mM2 = ss.mgr(Fs), ss.mgr(M1s), ss.mgr(M2); mSW = ss.mgr(SWV)
M1_nb = ss.mgr({k: v for k, v in M1s.items() if not k.startswith(("bal_", "btpu_bal", "bumped_"))})   # swivel runs in the tube cap (verify_balancer running gap)
def names(a, b):
    h, n = a.in_collision_other(b, return_names=True); return sorted(set(map(tuple, n))) if h else []
def pose(t1, t2):
    R1 = rot(np.radians(-t1), [0, 0, 1])
    for k in M1s: mM1.set_transform(k, R1)
    for k in M2: mM2.set_transform(k, R1 @ rot(np.radians(-t2), [0, 1, 0]))
    extra = []
    if SWV:
        TB = ak.balancer_frames(dict(S1=float(t1), S2=float(t2), S3=0, E=0, W1=0, W2=0, W3=0, G=50))
        for k in SWV: mSW.set_transform(k, TB[k])
        for k in M1s:
            if not k.startswith(("bal_", "btpu_bal", "bumped_")): M1_nb.set_transform(k, R1)
        extra = names(mSW, mF) + names(mSW, mM2) + names(mSW, M1_nb)
    return names(mM1, mF) + names(mM2, mF) + names(mM2, mM1) + extra
tpu = lambda c: [p for p in c if any(x.startswith("tpu") for x in p)]
rigid = lambda c: [p for p in c if not any(x.startswith("tpu") for x in p)]
def scan(path):
    first_t = first_r = None
    for q in path:
        c = pose(*q)
        if first_t is None and tpu(c): first_t = (q, tpu(c))
        if first_r is None and rigid(c): first_r = (q, rigid(c)); break
    return first_t, first_r
res = {}
for t2 in (25.0, 60.0):
    res[f"S1- @S2={t2:.0f}"] = scan([(t1, t2) for t1 in np.arange(40, -25, -0.5)])
    res[f"S1+ @S2={t2:.0f}"] = scan([(t1, t2) for t1 in np.arange(120, 175, 0.5)])
res["S2+ @S1=60"] = scan([(60.0, t2) for t2 in np.arange(60, 110, 0.5)])
res["S2- @S1=60"] = scan([(60.0, t2) for t2 in np.arange(0, -70, -0.5)])
# soft box must be free of any contact (TPU included): corners and edges at 2.5 deg
(a0, a1), (b0, b1) = rl.SOFT["S1"], rl.SOFT["S2"]
bad = [(a, b) for a in np.arange(a0, a1 + 0.1, 2.5) for b in (b0, b1) if pose(a, b)] + [(a, b) for b in np.arange(b0, b1 + 0.1, 2.5) for a in (a0, a1) if pose(a, b)]
out = {}
for k, (ft, fr) in res.items():
    out[k] = dict(tpu_contact=(ft[0] if ft else None), rigid_contact=(fr[0] if fr else None), rigid_pair=(fr[1] if fr else None))
    print(f"{k:14s} TPU contact at {ft[0] if ft else '-'}   rigid contact at {fr[0] if fr else '-'} {fr[1] if fr else ''}")
print("soft-box edge poses with any contact:", bad)
out["soft_box_contacts"] = bad
json.dump(out, open(S + "stops_verify.json", "w"), indent=1, default=float)
