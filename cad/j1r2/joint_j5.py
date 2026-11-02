"""J5: gimbal hoops, flat. Outer ring = S1 (axis +x, back normal), inner hoop = S2 (A2, in the ring plane), arm along the hoop plane. Robot frame, mm. PROXY."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
PX = -30.0                                   # P: 78 mm off the back (baseline 108)
d0 = L.baseline_dir(45, 25); TH0 = float(np.degrees(np.arcsin(-d0[0]))); PH0 = float(np.degrees(np.arctan2(d0[1], d0[2])))
def R1(phi): return rot(-np.radians(phi), [1, 0, 0])
def Ry(a): return rot(np.radians(a), [0, 1, 0])
def Tm(phi, th): return tr([PX, 0, 0]) @ R1(phi) @ Ry(-th)
def dirv(phi, th): return (R1(phi) @ Ry(-th))[:3, 2]
def mv(m, T): m = m.copy(); m.apply_transform(T); return m
def build(phi=PH0, th=TH0):
    P = {}; AL, PC = COL["al"], COL["pacf"]
    # ---- fixed: base plate (6061, 6 mm, x -94..-88), support rib, bearing arc (fixed race, 205 deg), standoffs
    plate = prism([(-82, -165), (76, -165), (76, -58), (-60, -58), (-82, -90)], "x", -94, -88)
    P["plate_6061"] = dict(mesh=plate, mat="AL", group="joint", colour=AL)
    P["support_rib"] = dict(mesh=box(-94, PX - 7, 70, 76, -120, 28), mat="AL", group="joint", colour=AL)
    wedge = prism([(0, 0)] + [(120 * np.cos(np.radians(a)), 120 * np.sin(np.radians(a))) for a in np.linspace(-130, 75, 40)], "x", PX - 8, PX + 8)
    arc = I(cyl("x", 68, PX - 7, PX + 7, r_in=60.6), wedge)
    P["bearing_arc_fixed"] = dict(mesh=arc, mat="AL", group="joint", colour=(0.62, 0.64, 0.68))
    for k, a in enumerate((-100, -45)):
        y, z = 64 * np.cos(np.radians(a)), 64 * np.sin(np.radians(a)); P[f"standoff{k}"] = dict(mesh=cyl("x", 4.5, -88, PX - 7, c=(y, z)), mat="AL", group="joint", colour=AL)
    # ---- S1 stage: ring (rotating race, axis x) with belt flange; S1 AK70 flat on the plate, belt 4:1
    ring = U(cyl("x", 60, PX - 7, PX + 7, r_in=52), cyl("x", 62, PX + 7, PX + 15, r_in=52))
    for s in (1, -1): ring = U(ring, mv(cyl("y", 9, s * 52, s * 62), R1(phi) @ tr([0, 0, 0]) if False else np.eye(4)) if False else mv(cyl("y", 9, s * 52, s * 62, c=(PX, 0)), R1(phi)).copy()) if False else ring
    bosses = [mv(cyl("y", 9, s * 52, s * 62, c=(0, 0)), tr([PX, 0, 0]) @ R1(phi)) for s in (1, -1)]
    P["ring_S1"] = dict(mesh=U(ring, *bosses), mat="AL", group="joint", colour=(0.85, 0.55, 0.15))
    ys, zs = 15.0, -118.0
    P["ak70_S1"] = dict(mesh=cyl("x", 44.5, -88, -37.75, c=(ys, zs)), mat="AL", group="bought", mass_g=621, colour=COL["motor"])
    P["s1_shaft_pulley"] = dict(mesh=U(cyl("x", 4, -37.75, PX + 15, c=(ys, zs)), cyl("x", 15.5, PX + 7, PX + 15, c=(ys, zs))), mat="AL", group="joint", colour=AL)
    belt = D(hull(cyl("x", 62.8, PX + 7.5, PX + 14.5), cyl("x", 16.3, PX + 7.5, PX + 14.5, c=(ys, zs))), hull(cyl("x", 61.3, PX + 6, PX + 16), cyl("x", 14.8, PX + 6, PX + 16, c=(ys, zs))))
    P["belt_S1"] = dict(mesh=belt, mat="TPU", group="joint", colour=(0.1, 0.1, 0.1))
    # ---- S2 stage (rides on the ring): hoop + pivots + S2 AK70 (-A2 side) + S3 XM540 + arm
    T = Tm(phi, th)
    hoop = U(cyl("y", 44, -5, 5, r_in=38), box(-10, 10, -15, 15, 38, 48)); pins = U(cyl("y", 5, -62, -38), cyl("y", 5, 38, 62))
    P["hoop_S2"] = dict(mesh=mv(U(hoop, pins), T), mat="AL", group="joint", colour=(0.30, 0.55, 0.85))
    P["ak70_S2"] = dict(mesh=mv(cyl("y", 44.5, -110.25, -62), T), mat="AL", group="bought", mass_g=621, colour=COL["motor"])
    P["xm540_S3"] = dict(mesh=mv(box(-16.75, 16.75, -29.25, 29.25, 46, 90), T), mat="AL", group="bought", mass_g=165, colour=(0.12, 0.55, 0.55))
    P["upper_arm"] = dict(mesh=mv(U(cyl("z", 32, 90, 192), sph(44, (0, 0, 230))), T), mat="PACF", group="arm", colour=COL["stage"])
    # ---- fairing over the lower motor bay (armour)
    outer = hull(box(-96, -12, -84, 80, -168, -66)); inner = hull(box(-94, -14, -82, 78, -166, -68.2))
    fair = D(outer, inner, box(-24, -14, -12, 48, -72, -60)); fair = D(fair, box(-93, -88, -82, 78, -170, -60))
    P["fairing_lower"] = dict(mesh=fair, mat="PACF", group="armour", colour=(0.20, 0.34, 0.52))
    for k, s in env_parts().items(): pass
    return P
def reach_set(phis, ths): return np.array([dirv(a, b) for a in phis for b in ths])
if __name__ == "__main__":
    P = build()
    dirs = reach_set(np.arange(-40, 90.1, 2.0), np.arange(-50, 80.1, 2.0)); cov = L.coverage(dirs)
    # swept envelope: moving parts at the 4 corners of the (phi, theta) box
    xs = []
    for a in (-40, 0, 90):
        for b in (-50, 0, 80):
            Q = build(a, b); xs.append(max(Q[k]["mesh"].bounds[1][0] for k in ("hoop_S2", "ak70_S2", "xm540_S3")))
    cnt = ("joint", "armour", "bought")
    depth = union_protrusion([v["mesh"] for v in P.values() if v["group"] in cnt]); depth_sw = max(depth, max(xs) + 108.0)
    M, Kbal = 6.49, 3.245
    resid = max(abs(M * np.cos(np.radians(f)) - Kbal) for f in np.arange(-40, 90.1, 1.0)) * np.sin(np.radians(80))
    extra = dict(own_stage=True,
      summary="Gimbal hoops lie in the back plane at rest: S1 = ring about +x (back normal) driven 4:1 by a flat AK70 and belt, S2 = inner hoop about an in-plane diameter driven direct by a second AK70 riding on the ring, S3 XM540 on the hoop, arm along the hoop plane. Flat at rest, but the tilting hoop sweeps a sphere of R 44 about P, so the real envelope is not much flatter than a 'dome <= 40 mm'.",
      axes=dict(S1="ring axis +x (back normal) through P=(%.0f,0,0), AK70 #1 flat on the plate at (y 15, z -118), belt 4:1" % PX, S2="in-plane diameter A2=(0,cos phi,-sin phi) through P, AK70 #2 on the ring, direct drive", S3="arm axis (in the hoop plane), XM540 on the hoop"),
      ranges_deg=dict(phi_S1=[-40, 90], theta_S2=[-50, 80], hard_stops="5 deg beyond, passive (ring stop lugs on the fixed arc; hoop stop lugs on the ring)", display_pose=dict(phi=round(PH0, 1), theta=round(TH0, 1))),
      coverage_of_baseline_arm_directions_pct=round(100 * cov, 1), unreachable="none inside the box; the box also reaches more directions than the baseline (forward-horizontal at theta 80, backward-down region is not needed)",
      depth_behind_back_mm=dict(displayed_pose=round(depth, 1), swept_over_range=round(depth_sw, 1), baseline=181),
      P=dict(x=PX, off_back_mm=round(-108 - PX, 1) * -1 if False else round(PX + 108, 1), vs_baseline="P moves 30 mm toward the body (x 0 -> %.0f); head/neck keep-out tables shift" % PX),
      mass_baseline=dict(bought_g=1407, mount_proxy_g=565.6),
      torque=dict(S2_unbalanced="tau_S2 = M sin(theta) cos(phi), M = 6.49 N.m worst: 6.49 N.m vs 8.3 rated = SF 1.28 (same as baseline, FAIL < 2)",
                  S2_balancer="balancer fixed to the ring cannot know phi: K = M/2 = %.2f N.m, residual max|M cos(phi) - K| sin(80) = %.2f N.m -> SF %.2f (PASS) over phi -40..90" % (Kbal, resid, 8.3 / resid),
                  S1="arm horizontal, theta 0: 6.49 + S2 motor 0.621 kg x 9.81 x 0.085 m = 0.52 -> 7.0 N.m at the ring; motor torque = 7.0/(4.0 x 0.92) = 1.9 N.m -> SF 4.4; belt tension F = (7.0/4)/0.0155 m = 113 N (belt rating PLACEHOLDER)",
                  speed="S1 joint speed cap 49.5 deg/s -> 198 deg/s at the motor shaft (4:1), AK70 speed limit PLACEHOLDER"),
      stops="passive: S1 ring lugs on the fixed arc; S2 lugs between hoop pivot boss and ring; TPU bumpers at +5 deg beyond soft",
      balancer="K sin(theta) cord/sheave or clockspring at the passive +A2 pivot between ring and hoop, K = 3.2 N.m (half the worst, see torque)",
      wiring="S2 motor and S3 bundle (8.2 mm) leave the ring through a flex loop along the ring face; total phi travel 130 deg so no slip ring; the hoop pivot has no hollow shaft so S3/E cables run in a loop along the hoop edge",
      decision_conflicts=["ball-styled shoulder / concurrent axes: axes stay concurrent but the ball is gone (Forest override)", "arm rest direction changes: rest is in the back plane, not up (control homology map and keep-out tables change)", "S1 now carries gravity (baseline S1 was vertical, no load)"],
      contract_changes=["plate shrinks to 94-88 mm and moves 6 mm rearward, outline z -165..-58; S1 4 x M5 block pattern replaced by the rib/arc standoffs", "clasp feet and wing tongue seats unchanged in y,z but the plate x face moves from -88 to -94 (wing lap +6 mm)"],
      pros=["flat at rest (ring plane 28 mm thick, fairing 82 mm incl. motors)", "concurrent axes, same actuators, SF 4.4 on S1 via the belt", "ranges cover 100 % of the baseline direction set"],
      cons=["swept hoop sphere R44 about P: no flat dome, depth to +%d mm behind back over the range" % round(depth_sw), "S2 SF 1.28 without a balancer, balancer cannot track phi (residual 3.25 N.m)", "ring needs >= 180 deg of retention but the S2 motor orbit blocks 90..220 deg of the track: bearing is UNRESOLVED (a segmented race is a PLACEHOLDER)", "exposed pinch points between hoop and ring (ISO 13857 4 mm rule) need a guard"],
      risks=["ring/hoop alignment tolerance, belt stretch (backlash at the tip: 0.5 deg ring = 5 mm at 575 mm)", "S2 motor mass on the ring (0.52 N.m on S1)", "head clearance with P 30 mm closer"],
      gates_to_rerun=["verify_shoulder", "sweep_full + keep-out tables (kinematics changed)", "control_map homology / task-space DLS", "harness_loads (mass moves)", "balancer design", "firmware gravity model grav_model.h"],
      assumptions=["rated 8.3 N.m, belt efficiency 0.92, belt/bearing/pulley vendor parts PLACEHOLDER", "F1/F4 seats assumed", "arm stub = upper arm to the elbow ball only"])
    meta = export_concept("J5", "Gimbal hoops in a flat bay", P, notes={}, extra=extra)
    print(round(cov * 100, 1), depth, depth_sw, resid, meta["mount_mass_g"])
    for r in meta["parts"]: print(r["name"], r["mass_g"], r["skin_min_mm"] if "skin_min_mm" in r else "", r.get("protrusion_mm"), r["watertight"])
