"""J1 "Back-normal S1 pancake + side-hung S2".  S1' = AK70 flat on the back, axis along +x (back normal), output yoke turns in the back plane.
S2 axis lies in the back plane (direction s = Rx(th1) y), its AK70 hangs LATERALLY of the S1 motor (clears the S1 disc), the arm (S3 along the arm) leaves its flange.
Kinematics: th1 about +x, arm rest direction a = Rx(th1) z = (0,-sin th1, cos th1); S2 swings the arm toward -x:  d = a cos(phi) - xhat sin(phi)."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
g = 9.81
# ---------------------------------------------------------------- kinematics / coverage
def dirJ1(th1, phi):
    t, p = np.radians(th1), np.radians(phi); return np.array([-np.sin(p), -np.sin(t) * np.cos(p), np.cos(t) * np.cos(p)])
TH = (-95.0, 35.0); PH = (-30.0, 88.0)                                    # soft ranges of S1', S2
dirs = np.array([dirJ1(a, b) for a in np.arange(TH[0], TH[1] + .1, 1.0) for b in np.arange(PH[0], PH[1] + .1, 1.0)])
cov = L.coverage(dirs, 3.0)
B = L.baseline_dir_set(); from scipy.spatial import cKDTree
dd, _ = cKDTree(dirs).query(B); miss = B[dd > 2 * np.sin(np.radians(3) / 2)]
# park-equivalent pose
dp = L.baseline_dir(45, 25); phi_p = float(np.degrees(np.arcsin(-dp[0]))); th_p = float(np.degrees(np.arctan2(-dp[1], dp[2])))
assert np.allclose(dirJ1(th_p, phi_p), dp, atol=1e-6)
# ---------------------------------------------------------------- packaging
AKR, AKL = 44.5, 50.25
TS = AKR + 3.0                      # S2 motor body starts 47.5 along s (clears the S1 disc)
T_ARM = TS + AKL + 8.0              # arm axis crosses the S2 axis here (flange + 8 mm adapter)
y1, z1 = -70.0, -35.0               # S1' axis position in the back plane
Y, Z, X = L.rear_surface(y1 - 160, y1 + 160, z1 - 160, z1 + 160, 6.0, 3.0)
fp = (np.hypot(Y - y1, Z - z1) < 60); sw = (np.hypot(Y - y1, Z - z1) < TS + AKL + 8 + 45)
xs_fp, xs_sw = float(X[fp].max()), float(X[sw].max())
x0 = max(xs_fp + 22.0, xs_sw + 2.3)             # base plate FRONT face (towards the body); S1 body sits on its rear face
PL_T = 6.0; XB = x0 + PL_T                       # S1 mounting face
XO = XB + AKL                                    # S1 output face
PX = XO + 6.0                                    # S2 axis plane x (yoke bar 12 thick centred on it)
th_hi = TH[1] + 5; th_lo = TH[0] - 5
def Rx(a): return rot(np.radians(a), [1, 0, 0])
def s_of(th): return np.array([0, np.cos(np.radians(th)), np.sin(np.radians(th))])
def a_of(th): return dirJ1(th, 0)
P1 = np.array([PX, y1, z1])
def frame(o, ex, ey, ez):
    M = np.eye(4); M[:3, 0], M[:3, 1], M[:3, 2], M[:3, 3] = ex, ey, ez, o; return M
def place(m, M): m = m.copy(); m.apply_transform(M); return m
def parts_at(th, phi):
    s, a = s_of(th), a_of(th); ex0 = np.cross(s, a)                          # = +x
    F0 = frame(P1, ex0, s, a)                                               # yoke frame (phi = 0): local y along s, local z along a
    d = dirJ1(th, phi); w = np.cross(s, d); FA = frame(P1 + T_ARM * s, w, s, d)   # arm frame (about the S2 axis)
    return F0, FA, s, a, d
F0, FA, s0, a0, d0 = parts_at(th_p, phi_p)
P = {}
# --- fixed base: planar pad on the back plane (PA-CF) + S1 motor + stop posts
pad = U(cyl("x", 62.0, x0, x0 + PL_T, c=(y1, z1)))
P["base_pad"] = dict(mesh=pad, mat="PACF", group="joint", colour=COL["pacf"])
P["S1_AK70"] = dict(mesh=cyl("x", AKR, XB, XO, c=(y1, z1)), mat="AL", mass_g=621.0, group="bought", colour=COL["motor"])
posts = []
for th in (th_lo, th_hi):                                                    # S1' passive stop posts at the HARD limits (5 deg beyond soft), r 54, dia 12
    c = np.array([y1, z1]) + 54.0 * np.array([np.cos(np.radians(th)), np.sin(np.radians(th))]) * np.array([1, 1]) * 0
    # lug travels on a circle in the yz-plane; angle th measured about +x from +z: unit tangent point = P1 + 54 * (-sin th, cos th) in (y,z)
    c = np.array([y1 - 54.0 * np.sin(np.radians(th)), z1 + 54.0 * np.cos(np.radians(th))]); posts.append(cyl("x", 6.0, XB, XO + 6.0, c=tuple(c)))
P["S1_stop_posts"] = dict(mesh=U(*posts), mat="PETGCF", fill=0.6, group="joint", colour=COL["accent"])
# --- moving yoke (rotates with S1')
def yoke_local():
    disc = cyl("x", 47.0, XO, XO + 12.0, c=(0, 0))                           # absolute x; centred on axis later
    return disc
disc = place(cyl("x", 47.0, XO, XO + 12.0, c=(0, 0)), tr([0, y1, z1]))
disc = place(disc, tr([0, -y1, -z1])); disc = place(disc, Rx(th_p)); disc = place(disc, tr([0, y1, z1]))   # rotate about the S1 axis through (y1,z1)
bar = place(box(-6, 6, 0.0, TS + 3.0, -26, 26), F0)                           # local x -> +x, y -> s, z -> a ; bar centred on x = PX
bar.apply_translation([0, 0, 0])
yoke = U(disc, bar)
lug = cyl("x", 6.0, XO, XO + 12.0, c=(0, 54.0)); lug = place(lug, tr([0, 0, 0]))   # stop lug on the disc rim at angle th_p+... (rotates with the yoke)
lug = place(lug, tr([0, -0, 0])); lug = place(lug, Rx(th_p)); lug = place(lug, tr([0, y1, z1]))
P["S1_yoke"] = dict(mesh=U(yoke, lug), mat="PETGCF", fill=0.6, group="joint", colour=COL["steel"])
# --- S2 AK70 (fixed to the yoke), clamp ring
s2m = place(cyl("y", AKR, TS, TS + AKL, c=(0, 0)), F0)                       # centred on the S2 axis, x = PX
P["S2_AK70"] = dict(mesh=s2m, mat="AL", mass_g=621.0, group="bought", colour=COL["motor"])
clamp = place(cyl("y", AKR + 4.0, TS + 4.0, TS + 4.0 + 30.0, c=(0, 0), r_in=AKR + 0.1), F0)
P["S2_clamp"] = dict(mesh=clamp, mat="PETGCF", fill=0.6, group="joint", colour=COL["steel"])
# --- arm frame parts: adapter, S3 bracket, XM540, upper arm, elbow ball
ad = place(cyl("y", 40.0, TS + AKL, T_ARM, c=(0, 0)), FA)
P["S2_adapter"] = dict(mesh=ad, mat="PETGCF", fill=0.6, group="joint", colour=COL["steel"])
brk = place(box(-22, 22, T_ARM - TS - AKL - 8.0 + (TS + AKL) - 0, T_ARM + 12.0, 0.0, 68.0), FA)                  # bracket plate rises from the adapter to the S3 body
brk = place(box(-24, 24, TS + AKL, T_ARM + 12.0, 6.0, 70.0), FA)
P["S3_bracket"] = dict(mesh=brk, mat="PETGCF", fill=0.6, group="joint", colour=COL["steel"])
xm = place(box(-29.25, 29.25, T_ARM - 16.75, T_ARM + 16.75, 68.0, 112.0), FA)
P["S3_XM540"] = dict(mesh=xm, mat="AL", mass_g=165.0, group="bought", colour=COL["motor"])
ua = place(cyl("z", 32.0, 112.0, 230.0 - 40.0, c=(0, T_ARM)), FA)
P["arm_tube"] = dict(mesh=ua, mat="PETG", group="arm", colour=COL["stage"])
eb = place(sph(44.0, (0, T_ARM, 230.0)), FA)
P["arm_elbow_ball"] = dict(mesh=eb, mat="PETG", group="arm", colour=COL["stage"])
# --- S2 passive stops: pad posts on the clamp (fixed) at the S2 hard limits, lug on the adapter (moving)
sp = []
for ph in (PH[0] - 5.0, PH[1] + 5.0):
    dd_ = dirJ1(th_p, ph); wv = np.cross(s0, dd_); Fh = frame(P1 + (TS + 4.0 + 30.0) * s0, wv, s0, dd_)
    sp.append(place(cyl("y", 6.0, 0, 10.0, c=(0, 52.0)), Fh))
P["S2_stop_posts"] = dict(mesh=U(*sp), mat="PETGCF", fill=0.6, group="joint", colour=COL["accent"])
lg = place(cyl("y", 6.0, TS + AKL - 14.0, TS + AKL, c=(0, 52.0)), FA)
P["S2_stop_lug"] = dict(mesh=lg, mat="PETGCF", fill=0.6, group="joint", colour=COL["accent"])
# --- fairing: S2 sleeve (moves with S1') and S1 cap ring (static)
sleeve = place(cyl("y", AKR + 2.6, TS - 1.0, TS + AKL, c=(0, 0), r_in=AKR + 0.8), F0)
P["fairing_S2_sleeve"] = dict(mesh=sleeve, mat="PACF", group="armour", colour=(0.2, 0.34, 0.52))
cap = cyl("x", 66.0, x0 - 0.0, XO + 14.0, c=(y1, z1), r_in=64.2)
cap = I(cap, box(x0 - 1, XO + 15, y1 - 70, y1 + 70, z1 - 70, z1 + 70))
P["fairing_S1_ring"] = dict(mesh=cap, mat="PACF", group="armour", colour=(0.2, 0.34, 0.52))
# ---------------------------------------------------------------- static skin check, swept over S1' (S2 motor + sleeve + yoke bar)
sweep_min = {}
for th in np.arange(TH[0] - 5, TH[1] + 5.1, 10.0):
    F0t, _, _, _, _ = parts_at(th, 0.0)
    for nm, m0 in (("S2_AK70", place(cyl("y", AKR + 2.6, TS - 1, TS + AKL, c=(0, 0)), F0t)),):
        sweep_min[round(float(th))] = round(L.skin_gap(m0, 2500)[0], 1)
# head/neck proximity of the S2 sleeve over the sweep (z > -60 within the neck/head cylinder)
body = L.body(); head_sel = body.vertices[(body.vertices[:, 2] > -70) & (np.abs(body.vertices[:, 1] + 150) < 120) & (body.vertices[:, 0] < -90)]
hk = cKDTree(head_sel); head_min = {}
for th in np.arange(TH[0] - 5, TH[1] + 5.1, 10.0):
    F0t, _, _, _, _ = parts_at(th, 0.0); m0 = place(cyl("y", AKR + 2.6, TS - 1, TS + AKL, c=(0, 0)), F0t)
    head_min[round(float(th))] = round(float(hk.query(trimesh.sample.sample_surface(m0, 2500)[0])[0].min()), 1)
# ---------------------------------------------------------------- torque study (grid minimax of zero-length-spring balancers)
mA, LA, mP, LP, mM, mY = 1.489, 0.249, 0.5, 0.575, 0.621, 0.15
TT = np.radians(np.arange(TH[0], TH[1] + .1, 2.5)); PP = np.radians(np.arange(PH[0], PH[1] + .1, 2.5)); T_, P_ = np.meshgrid(TT, PP, indexing="ij")
def tau1(pl):                                           # gravity torque about S1' axis (x): sum m g (y_i - y1) ; lengths in m
    sy = np.cos(T_); ay = -np.sin(T_) * np.cos(P_)
    ya = (T_ARM / 1000) * sy + LA * ay; yp = (T_ARM / 1000) * sy + LP * ay; ym = (TS + AKL / 2) / 1000 * sy; yy = 0.03 * sy
    return g * (mA * ya + pl * yp + mM * ym + mY * yy)
def tau2(pl): return g * (mA * LA + pl * LP) * np.sin(P_) * np.cos(T_)
u1 = max(np.abs(tau1(0.5)).max(), np.abs(tau1(0.0)).max()); u2 = max(np.abs(tau2(0.5)).max(), np.abs(tau2(0.0)).max())
best1 = (1e9, None)
for K in np.arange(0, 8.01, 0.1):
    for d0_ in np.radians(np.arange(-180, 180, 5)):
        bal = K * np.sin(T_ + d0_); r = max(np.abs(tau1(0.5) - bal).max(), np.abs(tau1(0.0) - bal).max())
        if r < best1[0]: best1 = (r, (K, np.degrees(d0_)))
best2 = (1e9, None)
for K in np.arange(0, 8.01, 0.05):
    bal = K * np.sin(P_); r = max(np.abs(tau2(0.5) - bal).max(), np.abs(tau2(0.0) - bal).max())
    if r < best2[0]: best2 = (r, K)
# ---------------------------------------------------------------- export
allj = [v["mesh"] for v in P.values() if v["group"] in ("joint", "bought", "armour")]
depth = float(max(m.bounds[1][0] for m in allj) + 108.0)
base_B = P1 + T_ARM * s0
extra = dict(own_stage=True,
  summary="S1' (AK70) lies flat on a planar back-plane pad, axis along +x; a 12 mm yoke bar swings the S2 AK70 (axis in the back plane) round it; the S2 motor sits beside the S1 disc, the arm leaves its flange. Depth 115 mm behind the back (baseline 181) but the S2 axis is offset %.0f mm from the S1' axis and S1' now carries gravity." % T_ARM,
  axes=dict(S1p=dict(dir=[1, 0, 0], at_yz=[y1, z1], actuator="AK70-10 flat on the pad"), S2=dict(dir="s = (0, cos th1, sin th1) (in the back plane, turns with S1')", at="crosses the S1' axis at x=%.1f; arm axis crosses it %.1f mm along s" % (PX, T_ARM), actuator="AK70-10"), S3=dict(dir="arm axis d", actuator="XM540")),
  kinematics="d = (-sin phi, -sin th1 cos phi, cos th1 cos phi); park-equivalent th1=%.1f phi=%.1f (baseline S1 45/S2 25)" % (th_p, phi_p),
  soft_ranges=dict(S1p_deg=TH, S2_deg=PH, hard_deg=dict(S1p=[th_lo, th_hi], S2=[PH[0] - 5, PH[1] + 5])),
  coverage_of_baseline_dirs_3deg=round(cov, 4), uncovered_dir_count=int(len(miss)),
  uncovered_note="near-forward directions (phi near 88) sit at a gimbal singularity of S1' (arm along -x): coverage is geometric; the controller cannot hold th1 there" if len(miss) else "none",
  depth_behind_back_mm=round(depth, 1), baseline_depth_mm=181,
  P=dict(S1p_axis_x=round(float(PX), 1), arm_base_at_park_xyz=[round(float(v), 1) for v in base_B], baseline_P=[0, 0, 0], arm_base_offset_from_S1p_axis_mm=T_ARM,
         arm_base_wander="arm base moves on a circle of radius %.1f mm about the S1' axis as th1 changes (non-concurrent S1'/S3)" % T_ARM),
  masses_g=dict(bought=2 * 621 + 165, baseline_bought=1407, note="structure/armour from proxy volumes (see parts)"),
  torque=dict(
    inputs="arm beyond S2 1.489 kg CG 0.249 m; payload 0.5 kg at 0.575 m; S2 AK70 0.621 kg at t=%.1f mm; yoke 0.15 kg; g=9.81" % (TS + AKL / 2),
    S1p="tau1 = g*sum(m_i*(y_i-y1)),  y_arm = T_ARM*cos(th1) - L*sin(th1)*cos(phi)  (m)",
    S1p_unbalanced_worst_Nm=round(float(u1), 2), S1p_SF_unbalanced=round(8.3 / u1, 2),
    S1p_best_zero_length_spring=dict(K_Nm=round(best1[1][0], 2), phase_deg=round(best1[1][1], 0), residual_Nm=round(float(best1[0]), 2), SF=round(8.3 / best1[0], 2)),
    S2="tau2 = g*(mA*LA+payload*LP)*sin(phi)*cos(th1)",
    S2_unbalanced_worst_Nm=round(float(u2), 2), S2_SF_unbalanced=round(8.3 / u2, 2),
    S2_best_yoke_anchored_spring=dict(K_Nm=round(best2[1], 2), residual_Nm=round(float(best2[0]), 2), SF=round(8.3 / best2[0], 2)),
    baseline_reference="baseline S1 carries no gravity (vertical axis); S2 6.49 unbalanced / 2.89 balanced (SF 2.87)",
    transmission="direct drive (no gearing), no added backlash"),
  skin_sweep_S2_motor_min_gap_mm_by_th1=sweep_min, head_neck_min_distance_mm_by_th1=head_min,
  stops="S1': two PETG-CF posts at +-r54 on the pad at the hard limits (soft +-5 deg), lug on the yoke disc; TPU 95A caps as baseline. S2: two pad posts on the clamp ring at the hard limits, lug on the adapter. Passive, 5 deg margin kept.",
  balancer="S2: zero-length spring anchored on the yoke (rotates with S1') torque K sin(phi); S1': zero-length spring base-to-yoke with a phase offset (needs a second balancer, packaged in the 6 mm gap behind the yoke bar / on the pad rim). PLACEHOLDER vendor springs.",
  wiring="no hollow shaft assumed (AK70 solid output: PLACEHOLDER, verify): 8.2 mm bundle runs in a flat clockspring/service loop in the 12 mm yoke gap, then along the yoke bar to the S2 motor and through the S3 bracket; S1' range 130 deg fits a single flat-flex loop.",
  conflicts=["Forest override: shoulder no longer ball-styled and axes not concurrent (S1'-S2 concurrent at x=%.1f; S3 offset %.1f mm). Everything else kept." % (PX, T_ARM), "S1' now carries a gravity moment (baseline S1 had none); 5-deg rule kept"],
  contract_changes="chassis plate + S1 base block + pauldron keel/crown retired; the new base pad (R62 x 6) bolts to a riser from the wing tongue (not modelled; pad front face %.1f mm = 22 mm off the skin max over the pad, S2 sweep needs +%.1f mm)." % (x0, xs_sw - xs_fp),
  pros=["depth 115 mm vs 181", "no S1 vertical base block (80 mm deep) and no big cowl", "all direct drive, same actuators"],
  cons=["S1' needs a gravity balancer (SF %.2f unbalanced) in addition to S2" % (8.3 / u1), "arm base offset %.0f mm from the S1' axis wanders on a circle: keep-out tables, homology map, head clearance all change" % T_ARM, "S2 motor sweeps a large disc over the back (needs 20 mm over the blade: sets pad height)", "gimbal singularity at arm-forward (phi 90)"],
  risks=["head/neck proximity of the swept S2 motor (see head_neck_min_distance)", "no hollow shaft: cable loop life", "S1' balancer must be re-sized if payload changes"],
  gates_to_rerun=["verify_shoulder/verify_assembly (new groups)", "sweep_full + restricted + keepout_map + gen_tables (kinematics changed)", "control_map homology (S1' axis x, offset base)", "harness_loads (mass moves 100 mm closer, new moment arm)", "balancer design x2", "body_loads/stall limits", "free-fall energy (S1' also falls)"],
  assumptions=["F1/F4 seats and wing are generic proxies", "AK70 solid, no hollow shaft", "pad y1,z1 chosen by hand (%.0f,%.0f), not optimised" % (y1, z1), "torque study uses the project mass numbers, payload 0.5 kg at 575 mm"])
meta = L.export_concept("J1", "J1 Back-normal S1 pancake + side-hung S2", P, notes=dict(pose="park-equivalent th1 %.1f phi %.1f" % (th_p, phi_p)), extra=extra)
print(meta["mount_mass_g"], depth, cov, len(miss)); print(json.dumps(extra["torque"], indent=1)); print(sweep_min, head_min, x0, xs_fp, xs_sw)
for r in meta["parts"]: print(r["name"], r["mass_g"], r.get("skin_min_mm"), r["watertight"], r.get("protrusion_mm"))
