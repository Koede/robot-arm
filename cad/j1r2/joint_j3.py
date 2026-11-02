"""J3 'Slewing-ring S1 + offset low S2/S3': S1 = thin-section ring bearing (axis x = back normal) flat on the wing, belt-driven 3:1 by a flat AK70;
the ring's carrier swings the S2 axis out on a radius RHO so the arm can leave the plate footprint.  All proxy geometry (vendor ring/belt PLACEHOLDER)."""
import sys, os, json; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
from shapely.geometry import Point
RHO = 120.0; XP = -34.0; Q = np.array([XP, 0.0, 0.0]); C = np.array([XP, 0.0, -RHO])        # P at rest = baseline P in y,z; ring centre RHO below
R_ID, R_IN_O, R_OUT_O, R_OD = 80.0, 90.0, 100.0, 100.0
S1M = (-35.0, -RHO - 150.0)                                                                   # S1 AK70 centre (y,z)
# ---- ranges from the baseline direction set: d = (-sin t2, -cos t2 sin a, cos t2 cos a)
B = baseline_dir_set(2.0); T2 = np.degrees(np.arcsin(-B[:, 0])); A = np.degrees(np.arctan2(-B[:, 1], B[:, 2]))
soft = dict(S1=(float(np.floor(A.min() / 5) * 5), float(np.ceil(A.max() / 5) * 5)), S2=(float(np.floor(T2.min() / 5) * 5), float(np.ceil(T2.max() / 5) * 5)))
aS, tS = soft["S1"], soft["S2"]
ga, gt = np.meshgrid(np.radians(np.arange(aS[0], aS[1] + .1, 1.0)), np.radians(np.arange(tS[0], tS[1] + .1, 1.0)))
dirs = np.c_[(-np.sin(gt)).ravel(), (-np.cos(gt) * np.sin(ga)).ravel(), (np.cos(gt) * np.cos(ga)).ravel()]
cov = coverage(dirs)
dp = baseline_dir(45, 25); th2 = float(np.degrees(np.arcsin(-dp[0]))); a_pose = float(np.degrees(np.arctan2(-dp[1], dp[2])))
def Rx(a, c=C):   # rotation about the x axis through the ring centre (S1)
    return tr(c) @ rot(np.radians(a), [1, 0, 0]) @ tr(-c)
def Ry2(t, q=Q):  # S2 about y through Q, R = Ry(-theta2)
    return tr(q) @ rot(np.radians(-t), [0, 1, 0]) @ tr(-q)
T1 = Rx(a_pose); T12 = T1 @ Ry2(th2)
def mv(m, T): m = m.copy(); m.apply_transform(T); return m
circ = lambda cy, cz, r, n=72: [(cy + r * np.cos(t), cz + r * np.sin(t)) for t in np.linspace(0, 2 * np.pi, n, endpoint=False)]
hullpoly = lambda *polys: [tuple(p) for p in np.array(__import__("scipy.spatial", fromlist=["ConvexHull"]).ConvexHull(np.vstack([np.array(p) for p in polys])).points[__import__("scipy.spatial", fromlist=["ConvexHull"]).ConvexHull(np.vstack([np.array(p) for p in polys])).vertices])]
P = {}
add = lambda k, m, mat, g, col, **kw: P.__setitem__(k, dict(mesh=m, mat=mat, group=g, colour=col, **kw))
# ---------------- fixed base: ribbed PA-CF base plate (x -88 face seats the wing tongue / clasp feet, contract unchanged) + ring races + skirt
pp = hullpoly(circ(0, C[2], 106), circ(S1M[0], S1M[1], 52))
plate = prism(pp, "x", -88, -85)
ribs = [cyl("x", 106, -85, -80, c=(0, C[2]), r_in=100)] + [box(-85, -80, -1, 1, 62, 104).apply_transform(tr([0, 0, 0]) ) or None for _ in []]
rib = U(*([cyl("x", 106, -85, -80, c=(0, C[2]), r_in=102)] + [mv(box(-85, -80, -1.0, 1.0, 64, 102), rot(np.radians(k * 30), [1, 0, 0]) @ np.eye(4)) for k in range(12)]))
rib.apply_translation([0, 0, 0]); rib = mv(rib, np.eye(4))
# the box ribs were built about the world origin axis (y=z=0): move them to the ring centre
ribs2 = U(*([cyl("x", 106, -85, -80, c=(0, C[2]), r_in=102)] + [mv(mv(box(-85, -80, -1.0, 1.0, 64, 102), rot(np.radians(k * 30), [1, 0, 0])), tr([0, 0, C[2]])) for k in range(12)]))
tab_ribs = U(cyl("x", 52, -85, -80, c=S1M, r_in=49))
whole = U(plate, ribs2, tab_ribs)
add("base_plate_upper", I(whole, box(-100, -70, -200, 200, C[2] - 80, 60)), "PACF", "joint", COL["pacf"], contact=True)
add("base_plate_lower_tab", I(whole, box(-100, -70, -200, 200, -400, C[2] - 70)), "PACF", "joint", COL["pacf"], contact=True)
add("ring_outer_race_fixed", cyl("x", R_OUT_O, -80, -62, c=(0, C[2]), r_in=R_IN_O), "PACF", "joint", COL["steel"])
inner = U(cyl("x", R_IN_O, -80, -62, c=(0, C[2]), r_in=R_ID), cyl("x", 94, -62, -50, c=(0, C[2]), r_in=R_ID))
add("ring_inner_race_pulley", mv(inner, T1), "PACF", "joint", COL["teal"])
tor = trimesh.creation.torus(90.0, 2.0); tor.apply_transform(rot(np.pi / 2, [0, 1, 0])); tor.apply_translation([-71, 0, C[2]])
add("balls_d4_x40", tor, "STEEL", "bought", COL["steel"], mass_g=12.0, fill=1.0)
add("skirt_fixed", cyl("x", 103, -80, -58, c=(0, C[2]), r_in=100), "PACF", "armour", (0.20, 0.34, 0.52))
# stops: 2 posts on the fixed side at +-5 deg beyond soft S1 limits, lug on the carrier at azimuth 0
def post(phi_deg):
    b = box(-58, -48, -3.5, 3.5, 95, 101); return mv(mv(b, rot(np.radians(phi_deg), [1, 0, 0])), tr([0, 0, C[2]]))
add("stop_posts_fixed", U(post(aS[0] - 5), post(aS[1] + 5)), "STEEL", "joint", COL["accent"])
add("stop_lug_carrier", mv(mv(box(-58, -48, -3, 3, 90, 99), tr([0, 0, C[2]])), T1), "STEEL", "joint", COL["accent"])
# ---------------- S1 drive: flat AK70 beside the ring, belt on the pulley rim (3:1)
add("ak70_S1", cyl("x", 44.5, -50, 0.25, c=S1M), "AL", "bought", COL["motor"], mass_g=621.0, fill=1.0)
add("S1_pulley_r30", cyl("x", 30, -62, -50, c=S1M), "AL", "joint", COL["al"])
add("S1_motor_boss", cyl("x", 46, -80, -50, c=S1M, r_in=36), "PACF", "joint", COL["pacf"])
outer = hull(prism(circ(0, C[2], 94 + 1.5), "x", -60, -52), prism(circ(S1M[0], S1M[1], 30 + 1.5), "x", -60, -52)); inn = hull(prism(circ(0, C[2], 94), "x", -61, -51), prism(circ(S1M[0], S1M[1], 30), "x", -61, -51))
add("belt_HTD5M_10mm", D(outer, inn), "TPU", "joint", COL["tpu"])
# ---------------- carrier (turns with S1)
plateC = prism(hullpoly(circ(0, C[2], 90), [(-46, -60), (-30, -60), (-46, -46), (-30, -46)]), "x", -62, -56)
clamp = U(cyl("y", 49.6, -46, -30, c=(XP, 0), r_in=44.6), box(-62, -28, -46, -30, -53, -45))
add("S1_carrier_plate_clamp", mv(U(plateC, clamp), T1), "PACF", "joint", COL["pacf"])
add("ak70_S2", mv(cyl("y", 44.5, -50.25, 0, c=(XP, 0)), T1), "AL", "bought", COL["motor"], mass_g=621.0, fill=1.0)
# ---------------- S2-carried: adapter, bracket, S3 sleeve, XM540, upper arm
adapter = cyl("y", 40, 0.5, 14, c=(XP, 0)); brk = box(XP - 30, XP + 30, 14, 24, 0, 110)
sleeve = D(box(XP - 20.25, XP + 20.25, -32.5, 32.5, 57, 109), box(XP - 17.25, XP + 17.25, -29.5, 29.5, 60, 110))
add("S2_adapter_bracket_S3_sleeve", mv(U(adapter, brk, sleeve), T12), "PACF", "joint", COL["pacf"])
add("xm540_S3", mv(box(XP - 16.75, XP + 16.75, -29.25, 29.25, 62, 106), T12), "AL", "bought", COL["motor"], mass_g=165.0, fill=1.0)
add("upper_arm_tube", mv(U(cyl("z", 32, 109, 200, c=(XP, 0)), sph(44, (XP, 0, 230))), T12), "PACF", "arm", COL["stage"])
# ---------------- rotating cowl round the S2 cluster, slot cut by the S2 sweep; rotating ring cover disc
core_o = hull(cyl("y", 52.5, -56, 8, c=(XP, 0)), box(XP - 28, XP + 4, -56, 8, -76, -30)); core_i = hull(cyl("y", 50.1, -54, 6, c=(XP, 0)), box(XP - 27, XP + 3, -54, 6, -77, -31))
cowl = D(core_o, core_i)
cut = []
for t in np.arange(tS[0] - 4, tS[1] + 4.1, 8.0):
    T = Ry2(t); cut += [mv(cyl("z", 36, 0, 150, c=(XP, 0)), T), mv(box(XP - 23.5, XP + 23.5, -35.5, 35.5, 54, 112), T)]
cowl = D(cowl, U(*cut))
cover = D(cyl("x", 98, -49, -46.6, c=(0, C[2])), core_o, cyl("y", 52.5, -56, 8, c=(XP, 0)))
add("cowl_rotating", mv(cowl, T1), "PETG", "armour", (0.20, 0.34, 0.52))
add("ring_cover_rotating", mv(cover, T1), "PETG", "armour", (0.20, 0.34, 0.52))
import trimesh as _tm
meta = export_concept("J3", "Slewing-ring S1 + offset S2/S3", P, notes={}, extra={"own_stage": True})
rows = {r["name"]: r for r in meta["parts"]}
_t, _pts, _n = L._tree(); BODY = L.body()
from scipy.interpolate import RegularGridInterpolator as _RGI
_Y, _Z, _X = rear_surface(-230, 140, -420, 20, 5.0, 1.0); _rg = _RGI((_Y[:, 0], _Z[0, :]), _X, bounds_error=False, fill_value=-1e3)
_fx = lambda y, z: _rg(np.c_[y, z])
for k, r in rows.items():
    if P[k]["group"] == "stage": continue
    sm = _tm.sample.sample_surface(P[k]["mesh"], 2500)[0]; dd, _ = _t.query(sm); inside = int(((sm[:, 0] < _fx(sm[:, 1], sm[:, 2])) & (sm[:, 1] < 40) & (sm[:, 2] < -10)).sum())
    r["skin_min_mm"] = round(float(dd.min()), 1); r["penetrating_samples"] = inside; r["fits_256"] = bool(np.all(P[k]["mesh"].extents <= 256.0))
counted = [k for k, s in P.items() if s["group"] in ("joint", "bought", "armour")]
depth = max(P[k]["mesh"].bounds[1][0] for k in counted) + 108.0
# ---------------- numbers
g = 9.81; m_arm, cg_arm = 1.489, 0.249; m_stage = 0.621 + 0.165 + 0.35; m_pl, L_pl = 0.5, 0.575
M_s1 = g * (m_stage * RHO / 1e3 + m_arm * (RHO / 1e3 + cg_arm) + m_pl * (RHO / 1e3 + L_pl)); ratio, eff = 180.0 / 60.0, 0.95
tau_m_s1 = M_s1 / (ratio * eff)
m_tot = m_stage + m_arm + m_pl; lever_x = 0.16; M_ring_static = 6.49 + g * m_tot * lever_x; M_ring_dyn = 2 * M_ring_static
n_b, R_b = 40, 0.09; F_ball = 2 * M_ring_dyn / (n_b * R_b) + g * m_tot * 2 / n_b
wing_M = float(np.hypot(M_s1, M_ring_static))
mass = lambda gs: round(sum(rows[k]["mass_g"] for k in P if P[k]["group"] in gs), 1)
extra = dict(own_stage=True,
  summary=f"S1 becomes a flat Ø200 x 18 mm printed ball-race ring on the wing (axis x, back normal), belt-driven 3:1 by a flat AK70; the carrier swings the S2 axis out on RHO={RHO:.0f} mm so the arm can leave the plate footprint. Joint stack depth {depth:.0f} mm behind the back (baseline 181).",
  axes=dict(S1="+x (back normal) through ring centre C=(%.0f,0,%.0f); AK70 #1 at (y %.0f, z %.0f), belt 3:1" % (C[0], C[2], S1M[0], S1M[1]),
            S2="+y in the ring frame through Q=(%.0f,0,0) (P at rest = baseline P in y,z, x moved 34 mm toward the body); AK70 #2, direct" % XP,
            S3="along the arm, XM540 in a sleeve on the S2 bracket; R = Rx(a) Ry(-t2) Rz(t3), d = (-sin t2, -cos t2 sin a, cos t2 cos a)"),
  ranges_soft=dict(S1_ring_deg=aS, S2_deg=tS, S3_deg=[-70, 90]), ranges_hard="5 deg beyond each soft limit (ring posts at %.0f/%.0f deg; S2 pads as baseline)" % (aS[0] - 5, aS[1] + 5),
  coverage_of_baseline_directions=round(cov, 3), unreachable="none by kinematics (every baseline direction maps to one (a, t2)); S1 range %s is wider than baseline's 130 deg span" % (aS,),
  pose_for_render=dict(a_deg=round(a_pose, 1), t2_deg=round(th2, 1)),
  reach_map_not_a_sweep={"baseline P, no base plate": 1.0, "rho 0 (concurrent at ring centre)": 0.597, "rho 60": 0.671, "rho 90": 0.887, "rho 110": 0.978, "rho 120": "see rho 110/130", "rho 130": 1.0, "method": "straight capsule r32 P..230, >=15 mm to body, outside the r106 plate disc; 648 baseline directions; cad/j3_reach.py"},
  depth_behind_back_mm=round(depth, 1), baseline_depth_mm=181, depth_saved_mm=round(181 - depth, 1),
  P=dict(rel_back="P at rest is %.0f mm behind the back (baseline 108)" % (XP + 108), rel_baseline_P="dx = %+.0f mm (toward the body), dy = dz = 0 at rest; P now moves on a circle of radius %.0f about C" % (XP, RHO)),
  mass_g=dict(structure_joint=mass(("joint",)), bought_listed=mass(("bought",)), armour=mass(("armour",)), baseline_bought=1407, baseline_mount_proxy=565.6,
              note="bought here = 2 AK70 1242 + XM540 165 + balls 12; bearing races are printed PA-CF, vendor ring would add ~0.4-0.6 kg (PLACEHOLDER)"),
  torque=dict(S1_gravity_Nm=round(M_s1, 2), formula="M = g [ m_stage RHO + m_arm (RHO + cg) + m_pl (RHO + L) ], m_stage 1.136 kg, m_arm 1.489 kg, cg 0.249 m, payload 0.5 kg at 0.575 m, RHO 0.120 m (arm horizontal, ring at +-90 deg)",
              ratio=ratio, efficiency=eff, motor_torque_Nm=round(tau_m_s1, 2), SF_vs_rated_8p3=round(8.3 / tau_m_s1, 2), unbalanced_direct_SF=round(8.3 / M_s1, 2),
              S2="unchanged: 6.49 N.m worst unbalanced (SF 1.28), 2.89 N.m with the zero-length balancer (SF 2.87); S1 now carries gravity (baseline S1 had none), the 3:1 belt is what makes it SF >= 2",
              backlash="belt stage: assume 0.1 deg at the ring (unverified); tip error 575 mm * 0.1 deg = 1.0 mm"),
  ring_bearing=dict(M_static_Nm=round(M_ring_static, 2), M_dyn_2g_Nm=round(M_ring_dyn, 2), formula="M = S2 reaction 6.49 + g m_tot x_lever (m_tot %.2f kg, lever 0.16 m)" % m_tot,
                    ball_load_N=round(F_ball, 1), formula_ball="F = 2M/(n R) + 2 g m/n, n 40 balls D4, R 0.09 m", allowable_N="80 per D4 ball printed race (UNVERIFIED), SF %.1f" % (80 / F_ball),
                    tilt_tip="radial clearance 0.1 mm / R 90 mm = 1.1 mrad -> %.2f mm at the 575 mm tip" % (0.1 / 90 * 575)),
  wing_load=dict(moment_Nm=round(wing_M, 1), baseline_Nm=11.3, formula="sqrt(M_S1^2 + M_ring^2): S1 gravity torque is reacted in the plate plane (new) and the ring tilt moment as before"),
  stops="S1: 2 steel posts on the fixed plate + lug on the carrier, 5 deg beyond soft; S2: TPU crush pads on the carrier as baseline (not drawn)",
  balancer="S2 zero-length-spring balancer anchors on the carrier plate behind the S2 motor (tube along the carrier, in the ring frame): free space x -60..-30 between the plate and the S2 clamp; not drawn",
  wiring="8.2 mm bundle: AK70 #2 + XM540 leads run on the carrier plate to the ring centre, through the ring bore (r 80) as a flat service loop over +-55 deg of ring travel (clockspring / flat-flex PLACEHOLDER), out the plate hole to the main box",
  decision_conflicts=["ball-styled shoulder and concurrent axes: dropped for this study per Forest; S1/S2 axes are NOT concurrent (offset RHO %.0f mm), so the DLS shoulder map, the keep-out tables and homology need re-derivation" % RHO,
                      "S1 now carries gravity (baseline: none): AK60 swap rejection is moot, but S1 needs the belt reduction",
                      "no big backplate: a Ø212 ribbed plate floats 20-80 mm off the back; it is a structural base, not skin contact, but Forest should confirm it does not read as the backplate he removed"],
  contract_changes=["plate becomes a Ø212 + tab ribbed PA-CF base (was 158 x 100 Al); it must bolt to a taller wing tongue; clasp feet seats unchanged at x -88", "S1 base block, yoke, cowl, dome, pauldron all retired", "ring centre sits 120 mm below baseline P"],
  pros=["depth 181 -> %.0f mm; the stack is a flat disc (x -80..-46) plus a small cowl around the S2 motor" % depth, "S1 gets 3x torque from the belt, which also relieves S2-balancer sizing for S1", "ring bore gives a natural wiring pass-through", "base plate carries load over a Ø200 bolt circle: low bolt loads"],
  cons=["P is no longer fixed: it circles on r=120, forward reach needs the offset (reach map: rho 0 gives only 60 %% of directions, rho 120 about 98-100 %%)", "S1 gravity torque 10.3 N.m needs the 3:1 belt (backlash, belt creep)", "wing moment +%d %% over baseline" % round((wing_M / 11.3 - 1) * 100), "swing-arm carrier sweeps a r=164 circle: an exposed pinch zone behind the cowl needs the rotating cover disc (drawn)", "printed ball race rated by guess"],
  risks=["ring stiffness/tilt and ball allowable unverified", "S1 sector a %s keeps the carrier off the S1 motor (S1 motor at 150 mm below C), but a = +-180 is not reachable" % (aS,), "belt tension adds a radial load to the ring"],
  gates_to_rerun=["verify_shoulder + collision frontier with the new groups", "keep_out tables (elbow_limits.h, keepout_table.h, balancer_keepout.h) and gen_tables.py", "control_map homology and DLS map for a non-concurrent shoulder", "harness_loads (mass +/-, top pad 14.28/14.7), body_loads", "verify_armor/pauldron replaced by the skirt, cowl, cover", "print_table"],
  assumptions=["ring and belt vendor data PLACEHOLDER; AK70 shaft protrusion (12 mm) PLACEHOLDER", "stage mass 1.136 kg = 2 motors share + clamps (S2 0.621 + XM540 0.165 + 0.35)", "P at rest x moved to -34 mm; plate x -88..-80 contract kept", "reach map is a straight-capsule screen, not a collision sweep"])
meta["extra"] = extra; json.dump(meta, open(os.path.join(L.ROOT, "stl/concepts/J3/metrics.json"), "w"), indent=1, default=float)
print("depth", depth, "cov", cov, "soft", soft, "pose", a_pose, th2); print({k: (r["mass_g"], r.get("skin_min_mm"), r["watertight"], r["fits_256"]) for k, r in rows.items()})
print("S1 torque", M_s1, tau_m_s1, "ring", M_ring_static, F_ball, "wing", wing_M)
