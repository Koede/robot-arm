"""J6 differential bevel shoulder (flat): two AK70-10 (axes along x, side by side at y = -/+70) drive a miter-bevel differential whose
side-gear axis u is the lateral axis y through P.  phi = rotation of the cage about u (forward swing, ~baseline S2 role),
psi = spin of the cross shaft p (tilt of the arm toward +y, ~baseline lateral/S1 role).  d = (cos psi sin phi, sin psi, cos psi cos phi).
Proxy geometry, PLACEHOLDER gears/bearings.  Robot frame, mm."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
PX = -23.25                                  # P x: motors seat on a 6 mm saddle at x -110..-104 (motor back face -104)
YC, RM, LM = 70.0, 44.5, 50.25               # motor y offset, AK70 radius, length
R1, R0, FW = 22.5, 15.0, 7.5                  # miter pitch radius (m1.5 Z30), inner cone start, backing width
PSI0, PS1, S0, S1 = -15.0, 50.0, -87.0, 88.0  # soft ranges psi, s  (s = -phi, forward positive)
def frus_y(sign, r0, r1, a0, a1, c=(PX, 0)):  # solid cone frustum about the y axis through (PX,0): |y| from a0..a1, radius r0..r1
    ys = np.array([a0, a1]) * sign; m = trimesh.creation.cone(1, 1)
    # build by revolve: use cylinders stack (6 steps) for a good-enough frustum
    parts = []; n = 6
    for i in range(n):
        t0, t1 = a0 + (a1 - a0) * i / n, a0 + (a1 - a0) * (i + 1) / n; r = r0 + (r1 - r0) * (i + .5) / n
        y0, y1 = sorted([t0 * sign, t1 * sign]); parts.append(cyl("y", r, y0, y1, c=c, n=48))
    return U(*parts)
def frus_x(sign, r0, r1, a0, a1, c):         # frustum about an x-parallel axis through (y,z)=c : x from -a0..-a1 (sign -1 = toward body)
    parts = []; n = 6
    for i in range(n):
        t0, t1 = a0 + (a1 - a0) * i / n, a0 + (a1 - a0) * (i + 1) / n; r = r0 + (r1 - r0) * (i + .5) / n
        x0, x1 = sorted([PX + sign * t0, PX + sign * t1]); parts.append(cyl("x", r, x0, x1, c=c, n=48))
    return U(*parts)
def steel_mass(m, fill=0.6): return m.volume / 1000 * DENS["STEEL"] * fill
P = {}
def add(name, mesh, mat, group, colour, mass=None, fill=1.0, **k):
    d = dict(mesh=mesh, mat=mat, group=group, colour=colour, fill=fill, **k)
    if mass is not None: d["mass_g"] = mass
    P[name] = d
# ---------------- motors (bought) : back face x = -104, output face x = -53.75 ; AK70 mass 621 g
for nm, yc in (("A", -YC), ("B", YC)):
    add(f"ak70_{nm}", cyl("x", RM, -104.0, -104.0 + LM, c=(yc, 0)), "AL", "bought", COL["motor"], mass=621.0)
# ---------------- motor-side bevel pinion (steel) + A_in/B_in, side gears A_out/B_out, shafts
gears = {}
for nm, sg in (("A", -1), ("B", 1)):
    yc = sg * YC
    mp = U(frus_x(-1, R0, R1, R0, R1, (yc, 0)), cyl("x", R1, PX - R1 - 7.5, PX - R1, c=(yc, 0)), cyl("x", 10, -53.75, PX - R1 - 7.5, c=(yc, 0)))
    gin = U(frus_y(sg, R0, R1, YC - R1, YC - R0 if False else YC - R1 + (R1 - R0), (PX, 0)) if False else cyl("y", R1, *sorted([sg * (YC - R1 - 7.5), sg * (YC - R1)]), c=(PX, 0)),
            frus_y(sg, R0, R1, YC - R1, YC - R0 + 0, (PX, 0)) if False else frus_y(sg, R1, R0, YC - R1 - 7.5 + 7.5 - 0, YC - R0 + 0.0 if False else YC - R1 + 0, (PX, 0)) if False else frus_y(sg, R0, R1, YC - R1 - 0.0, YC - R0, (PX, 0)) if False else cyl("y", R0, *sorted([sg * (YC - R1 + 0), sg * (YC - R0 - 0)]), c=(PX, 0)))
    gout = U(frus_y(sg, R0, R1, R0, R1, (PX, 0)), cyl("y", R1, *sorted([sg * R1, sg * (R1 + FW)]), c=(PX, 0)))
    shaft = cyl("y", 8, *sorted([sg * (R1 + FW), sg * (YC - R1 - 7.5)]), c=(PX, 0))
    add(f"motor_pinion_{nm}", mp, "STEEL", "joint", COL["steel"], mass=steel_mass(mp)); add(f"gear_in_{nm}", gin, "STEEL", "joint", COL["steel"], mass=steel_mass(gin))
    add(f"side_gear_{nm}", gout, "STEEL", "joint", COL["steel"], mass=steel_mass(gout)); add(f"side_shaft_{nm}", shaft, "STEEL", "joint", COL["steel"], mass=steel_mass(shaft, 1.0))
# ---------------- carrier (rotates by phi about y through P) : cross shaft + 2 pinions + bearing blocks + side plates + hub/boom + S3 + upper arm
def Pm(m): m = m.copy(); m.apply_translation([PX, 0, 0]); return m
cs = cyl("x", 5, -40, 40); pin = []
for sg in (-1, 1):
    pin.append(U(*[cyl("x", R0 + (R1 - R0) * (i + .5) / 6, *sorted([sg * (R0 + (R1 - R0) * i / 6), sg * (R0 + (R1 - R0) * (i + 1) / 6)])) for i in range(6)], cyl("x", R1, *sorted([sg * R1, sg * (R1 + 5)]))))
pins = U(*pin); cage_blocks = U(box(-52, -40, -14, 14, -14, 14), box(40, 52, -14, 14, -14, 14))
side_pl = U(*[box(-52, 52, sg * 31.0 if sg > 0 else sg * 36.0, sg * 36.0 if sg > 0 else sg * 31.0, -34, 6) for sg in (-1, 1)])
strap = box(-52, 52, -36, 36, -34, -28)
cage = U(cage_blocks, side_pl, strap)
hub = cyl("x", 11, -16, 16)
def arm_frame(m, psi, phi):                       # arm group: tilt by psi about -x (Rx(-psi)), then whole carrier Ry(phi) about P
    m = m.copy(); m.apply_transform(rot(-np.radians(psi), [1, 0, 0])); return m
BOOM0, S3A, S3B, ELB = 0.0, 70.0, 114.0, 230.0
boom = box(-7, 7, -7, 7, 0, S3A); s3 = box(-16.75, 16.75, -29.25, 29.25, S3A, S3B)   # XM540 33.5 x 58.5 x 44
tube = cyl("z", 32.0, S3B, ELB - 44.0); ball = sph(44.0, (0, 0, ELB))
def carrier(psi, phi, pieces):
    out = []
    for m in pieces:
        m = m.copy(); m.apply_transform(rot(np.radians(phi), [0, 1, 0])); m.apply_translation([PX, 0, 0]); out.append(m)
    return out
S_PARK, PSI_PARK = np.degrees(np.arctan2(0.299, 0.906)), np.degrees(np.arcsin(0.299))
phi = -S_PARK
armset = [U(hub, boom), s3, tube, ball]; armset = [arm_frame(m, PSI_PARK, phi) for m in armset]
cc = carrier(PSI_PARK, phi, [cs, pins, cage] + armset)
add("cross_shaft_pinions", cc[0], "STEEL", "joint", COL["steel"], mass=steel_mass(cs, 1.0) + steel_mass(pins))
add("cage_plates", cc[2], "PACF", "joint", (0.12, 0.55, 0.55)); add("hub_boom", cc[3], "PACF", "arm", (0.9, 0.6, 0.15), fill=0.5)
add("xm540_S3", cc[4], "AL", "bought", COL["motor"], mass=165.0); add("upper_arm", cc[5], "PETGCF", "arm", COL["stage"], fill=0.3); add("elbow_ball", cc[6], "PETGCF", "arm", COL["stage"], fill=0.2)
# ---------------- saddle (6 mm, joint) under both motors, bolted to the wing tongue region: stadium y +-116, z +-50
sad = prism([(-116 + 50 * np.cos(a) if a > np.pi / 2 and a < 1.5 * np.pi else 116 - 50 * 0 + 0, 0) for a in []] or [(-116, -48), (116, -48), (116, 48), (-116, 48)], "x", -110.0, -104.0)
add("motor_saddle", sad, "PACF", "joint", COL["pacf"], fill=0.5)
# ---------------- fairing: hull(motor A, motor B, sphere R40 about P) 2.2 mm shell, arm slot = hull of arm tubes at the soft-range corners
env = hull(P["ak70_A"]["mesh"], P["ak70_B"]["mesh"], sph(40.0, (PX, 0, 0)))
outer = env.copy(); outer.apply_scale(1.0); outer = hull(P["ak70_A"]["mesh"], P["ak70_B"]["mesh"], sph(40.0, (PX, 0, 0)))
scale_in = trimesh.creation.icosphere(1, 1)  # unused
inner = hull(cyl("x", RM - 2.2, -104.0, -104.0 + LM + 2, c=(-YC, 0)), cyl("x", RM - 2.2, -104.0, -104.0 + LM + 2, c=(YC, 0)), sph(37.8, (PX, 0, 0)))
shell = D(outer, inner, box(-120, -102.5, -130, 130, -60, 60))
slot = []
for (ss, pp) in [(S0, PSI0), (S0, PS1), (S1, PSI0), (S1, PS1), (0, PSI0), (0, PS1), (-S_PARK * -1, PSI_PARK)]:
    ph = -ss; t = cyl("z", 40.0, 0, 130); t.apply_transform(rot(-np.radians(pp), [1, 0, 0])); t.apply_transform(rot(np.radians(ph), [0, 1, 0])); t.apply_translation([PX, 0, 0]); slot.append(t)
shell = D(shell, hull(*slot))
add("fairing", shell, "PACF", "armour", (0.20, 0.34, 0.52))
# ---------------- numbers
phis = np.radians(np.arange(S0, S1 + .1, 2.5)); psis = np.radians(np.arange(PSI0, PS1 + .1, 2.5))
Dr = np.array([[np.cos(p) * np.sin(-f), np.sin(p), np.cos(p) * np.cos(-f)] for f in phis for p in psis]); cov = L.coverage(Dr, 3.0)
B = L.baseline_dir_set(2.5); miss = B[np.array([np.min(np.linalg.norm(Dr - b, axis=1)) > 2 * np.sin(np.radians(1.5)) for b in B])]
miss_psi = np.degrees(np.arcsin(miss[:, 1])) if len(miss) else np.array([0.0])
G = 6.46                                          # N.m: (1.489 kg*0.249 m + 0.5 kg*0.575 m)*9.81 (project numbers)
def loads(s, psi):
    f = -np.radians(s); p = np.radians(psi); d = np.array([np.cos(p) * np.sin(f), np.sin(p), np.cos(p) * np.cos(f)])
    M = G * np.cross(np.array([0, 0, 1.0]), d)       # moment of the weight about P (N.m), |M| = G sin(angle from vertical)
    u = np.array([0, 1.0, 0]); pa = np.array([-np.cos(f), 0, np.sin(f)])
    return abs(M @ u), abs(M @ pa), np.linalg.norm(M)
worst = 0; worstbal = 0; wp = None
for s_ in np.arange(S0, S1 + .1, 2.5):
    for p_ in np.arange(PSI0, PS1 + .1, 2.5):
        tphi, tpsi, mm = loads(s_, p_); tm = (tphi + tpsi) / 2; tb = tpsi / 2           # per-motor torque, k_in = 1 ; balancer ideal on phi
        if tm > worst: worst, wp = tm, (s_, p_, tphi, tpsi, mm)
        worstbal = max(worstbal, tb)
depth = max(m["mesh"].bounds[1][0] for m in P.values() if m["group"] in ("joint", "bought", "armour")) + 108.0
extra = dict(own_stage=True, summary="Two flat AK70s (x axis) at y -/+70 drive a miter-bevel differential (side-gear axis = lateral y through P, cross shaft along x); cage rotation phi = forward swing, cross-shaft spin psi = lateral tilt of the arm; S3 XM540 along the arm.",
  axes=dict(u_phi="lateral y through P (motors A+B common mode), cage rotation, ~baseline S2 role", p_psi="cross shaft, x at phi=0, rotates with the cage (motors A-B differential), arm tilt toward +y", S3="XM540 along the arm axis, 70..114 mm from P", motors="AK70-10 A at y=-70 and B at y=+70, axes along x, back faces at x=-104 on a 6 mm PA-CF saddle (x -110..-104)"),
  kinematics="d = (cos psi sin phi, sin psi, cos psi cos phi), phi = -s (s forward positive).  Gear angles alpha (A), beta (B) about +y: phi=(alpha+beta)/2, psi=(alpha-beta)/2 * Zg/Zp (=1, miter Z30/Z30).  Motor angles theta_A = sigma_A alpha, theta_B = sigma_B beta (k_in = 1; sign pattern sigma = +/-1 set at bench).  Torques: T_phi = tau_A+tau_B, T_psi = (Zp/Zg)(tau_A-tau_B); each motor tau = (T_phi +/- T_psi)/2.",
  soft_ranges=dict(s_forward_deg=[S0, S1], psi_deg=[PSI0, PS1], hard_deg=dict(s=[S0 - 5, S1 + 5], psi=[PSI0 - 5, PS1 + 5])),
  coverage_baseline_dirs_3deg=round(cov, 3), unreachable=f"baseline directions with lateral latitude asin(d_y) > {PS1:.0f} deg (arm raised sideways, S2 > ~60 at S1 ~ 90) and < {PSI0:.0f} deg (back-medial, S2 < ~-12): {100*(1-cov):.1f} % of the baseline set; psi range is capped by the sector-gear window (pinion mesh arc + boom window must not overlap: delta psi < ~65 deg for a 14 mm boom starting 70 mm from P, tube R32 beyond the gear radius)",
  full_baseline_psi_requirement_deg=[-30, 87.5],
  depth_behind_back_mm=round(depth, 1), depth_swept_bound_mm=round(PX + np.hypot(52, 34) + 108, 1), head_neck_note=f'closest part to skin: motor A {42.5} mm (head/neck 60 mm rule not met statically at P_z = 0; drop P_z by ~20-30 mm or narrow)', baseline_depth_mm=181, P_vs_back=f"P at x = {PX} (85 mm behind the back surface x -108... i.e. {PX+108:.0f} mm), baseline 108 mm: P moves {PX} mm toward the body; z and y unchanged (P_z 0, y 0)",
  torque=dict(G_Nm=G, worst_motor_torque_unbalanced_Nm=round(worst, 2), at=dict(s=wp[0], psi=wp[1], T_phi=round(wp[2], 2), T_psi=round(wp[3], 2), M=round(wp[4], 2)), SF_vs_8p3_unbalanced=round(8.3 / worst, 2), worst_with_ideal_phi_balancer_Nm=round(worstbal, 2), SF_with_balancer=round(8.3 / max(worstbal, 1e-6), 2),
     note="tau_motor = (|T_phi|+|T_psi|)/2 with k_in = 1; baseline S2 alone 6.49 N.m on one motor (SF 1.28). Single-axis peak: 2 x 8.3 = 16.6 N.m on either axis if the other is unloaded. SF>=2 needs tau <= 4.15 N.m: unbalanced worst case inside the soft range passes (see SF); a phi balancer (zero-length spring, K ~3.8 N.m) fixes it; an option is k_in = 2 (motor pinion Z20 : gear Z40) halving motor torque (and speed range) at +gear size.",
     efficiency="two bevel meshes in series per path (0.97 each, assumed) = 0.94"),
  gears=dict(module=1.5, side_gears="Z30 miter (PD 45), 2", cross_pinions="Z30 miter, 2 on a 10 mm shaft", motor_pairs="Z30:Z30 miter, 2 (A_in/B_in + motor pinions)", vendor="PLACEHOLDER: hardened steel bevel gears (e.g. KHK-class SB1.5-30), none selected; bearing sizes PLACEHOLDER"),
  backlash=dict(assumed_arcmin_per_mesh=10, at_output_arcmin="10 (pinion/side-gear mesh) + 10/1 (motor mesh, k_in 1) = 20 worst, RSS 14", tip_mm_at_575=round(np.radians(20 / 60) * 575, 1), tip_mm_rss=round(np.radians(14 / 60) * 575, 1), note="AK70 internal planetary backlash not included (unverified)"),
  stops="passive, 5 deg beyond soft: phi (cage) by TPU-bumped lug on the cage plate vs two posts on the saddle housing; psi by the hub boom hitting TPU bumpers on the cage plates at psi hard -15 / +55.  Stops sit at the OUTPUT, so they load the housing/cage and never the gear train.  Because motor angles couple both axes, the motor-side travel limit is the L-infinity box in (phi,psi); the firmware must clamp (phi,psi), not motor angles.",
  balancer="a zero-length spring/cord on the cage about the lateral axis (u), mounted on the saddle above/behind motor A; it cancels T_phi only; the psi component of the arm weight (|T_psi|<= G sin(psi) cos(.)) stays on the motors.",
  wiring="8.2 mm bundle: motors' CAN/power stay on the fixed saddle (no wire crosses a joint for the motors); the XM540 + elbow/hand wires cross the differential: route as a service loop through the hollow cross shaft (bore 10 mm needed, shaft shown solid: PLACEHOLDER) then along the boom; limited by psi/phi travel (clockspring not needed for +/-90).",
  decision_conflicts="Overrides (per Forest, this study only): ball-styled shoulder and axes concurrent at P (they remain concurrent, but the S1/S2 roles are replaced by phi/psi; 'S1 vertical' no longer exists).  Keeps: 3 DOF + same actuators, passive stops, 5 deg margin, no backplate/hip belt.  Head/neck keep-out: medial motor B/A at y -115..-26 is at head height (P_z = 0); static skin gap reported - needs the 60 mm head/neck rule checked.",
  contract_changes="S1 base block/plate 4xM5 pattern, keel/crown/spaulder, yoke, S2 clamp, balancer anchors, S1 stop posts and lug all retire; new 6 mm saddle (2 x motor pad) bolts to the wing tongue; wing tongue seat height/width must widen to y -116..+116.  ROM/keep-out tables, control homology map (S1/S2 -> phi/psi, differential decoupling) and firmware gravity model change.",
  pros=["max single-axis torque 16.6 N.m (SF 2.6 alone vs 1.28 today)", "all heavy motors fixed on the wing (no motor mass on the moving arm base; low inertia at S1/S2)", "no cables cross the first two joints for the motors", "depth ~ 128 mm vs 181 mm"],
  cons=["both motors work on every move (differential); unbalanced SF 2.57 inside the range but 1.81 if gravity splits 45/45 between phi and psi (outside this range): balancer still advisable", f"psi range capped at {PS1-PSI0:.0f} deg span (window): {100*(1-cov):.0f} % of baseline directions unreachable", "gear train 8 steel bevel gears, backlash 14-20 arcmin ~ 2.4 mm at tip, noise, cost", "230 mm wide (y -115..+115) at head height: neck/head clearance and shoulder-lateral overhang", "pinion/cage loads and sector-gear windows are fragile"],
  risks=["sector-gear window geometry not proven (proxy only)", "head/neck keep-out static gap", "gear tooth stress with 16.6 N.m peaks (not calculated)", "controls: coupling + singularity at psi = +/-90 (outside range)"],
  gates_to_rerun=["verify_shoulder/verify_armor replaced (new parts)", "full sweep + keep-out tables (keepout_map, sweep_full, gen_tables)", "control_map.py homology (S1/S2 -> phi/psi) + control.h + grav_model.h", "harness_loads (top-pad margin 2 %: mass change)", "S2 free-fall braking (T12) both motors, dual windings", "print table"],
  assumptions=["miter gears Z30 module 1.5, 10 arcmin/mesh, 0.97 efficiency", "arm weight moment G = 6.46 N.m worst (payload 0.5 kg at the claw tip)", "P_z = 0 and y = 0 kept", "AK70 length 50.25 and radius 44.5 (verified), flange pattern PLACEHOLDER", "gear blanks modelled as stacked cylinders; mass = volume x 7.85 x 0.6"],
  mass_g=dict(bought=621 * 2 + 165, structure_and_gears=None))
meta = export_concept("J6", "Differential bevel shoulder (flat)", P, notes=dict(), extra=extra)
extra["mass_g"]["structure_and_gears"] = round(sum(r["mass_g"] for r in meta["parts"] if r["group"] in ("joint", "armour")), 1)
import json; json.dump(dict(meta, extra=extra), open(os.path.join(L.ROOT, "stl/concepts/J6/metrics.json"), "w"), indent=1)
for r in meta["parts"]: print(r["name"], r["group"], r["mass_g"], r.get("skin_min_mm"), r.get("protrusion_mm"), r["watertight"])
print("depth", depth, "coverage", cov, "worst motor", worst, wp, "balanced", worstbal)
