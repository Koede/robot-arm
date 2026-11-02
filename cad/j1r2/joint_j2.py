"""J2 Dual pancake + right-angle (bevel) transmission. Both AK70 lie flat (axes along x). S1 about x (belt 2:1, fixed motor medial), S2 about the radial axis of the
S1 carrier (motor on the carrier, 2:1 bevel pair), S3 XM540 along the arm. PROXY geometry; gears/belts/bearings are PLACEHOLDERS."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
TH1, TH2 = -18.3, 17.4                       # park pose: arm direction = baseline_dir(45,25)
PX = -8.0                                    # P' x (S2 axis / arm pivot); baseline P x = 0
def Rx(a): return rot(np.radians(a), [1, 0, 0])
def moved(m, T): m = m.copy(); m.apply_transform(T); return m
def arm_dir(t1, t2):
    t1, t2 = np.radians(t1), np.radians(t2); return np.array([-np.sin(t2), -np.cos(t2) * np.sin(t1), np.cos(t2) * np.cos(t1)])
def build():
    P = {}
    R2 = 95.0                                                                   # S2 motor orbit radius (carrier link length)
    plate = prism(hull_pts := [tuple(p) for p in np.array(sg.Polygon(np.vstack([[(60 * np.cos(a), 60 * np.sin(a)) for a in np.linspace(0, 6.28, 40)], [(-100 + 50 * np.cos(a), 50 * np.sin(a)) for a in np.linspace(0, 6.28, 40)]])).convex_hull.exterior.coords)], "x", -90, -86)
    P["plate"] = dict(mesh=plate, mat="AL", group="joint", colour=COL["al"])
    hub = cyl("x", 48, -86, -36, r_in=28)
    P["hub_fixed"] = dict(mesh=hub, mat="AL", fill=0.3, group="joint", colour=COL["steel"])
    P["S1_AK70"] = dict(mesh=cyl("x", 44.5, -86, -35.75, c=(-100, 0)), mat="AL", mass_g=621, group="bought", colour=COL["motor"])
    P["S1_belt_pulley"] = dict(mesh=cyl("x", 24, -35.75, -28, c=(-100, 0)), mat="AL", mass_g=14, group="bought", colour=COL["accent"])
    # rotating frame (about x by TH1)
    T1 = Rx(TH1)
    link = U(cyl("x", 52, -36, -28), hull(cyl("x", 40, -36, -28), cyl("x", 40, -36, -28, c=(R2, 0))))
    P["carrier_link"] = dict(mesh=moved(link, T1), mat="AL", fill=0.5, group="joint", colour=COL["al"])
    P["S2_AK70"] = dict(mesh=moved(cyl("x", 44.5, -86.25, -36, c=(R2, 0)), T1), mat="AL", mass_g=621, group="bought", colour=COL["motor"])
    gear = U(cyl("x", 26, -28, -2, c=(R2, 0)), cyl("y", 13, 0, R2, c=(PX, 0)), cyl("y", 20, 0, 60, c=(PX, 0)))
    P["S2_bevel_housing"] = dict(mesh=moved(gear, T1), mat="AL", fill=0.5, group="joint", colour=COL["steel"])
    P["S2_gearset_2to1"] = dict(mesh=moved(U(cyl("x", 12, -28, -10, c=(R2, 0)), cyl("y", 30, 62, 74, c=(PX, 0))), T1), mat="STEEL", mass_g=70, group="bought", colour=COL["teal"])
    # arm (S2 about the radial axis y, then S3) : local frame at P' with arm along +z
    TA = tr([PX, 0, 0]) @ T1 @ rot(np.radians(-TH2), [0, 1, 0])
    s3 = box(-16.75, 16.75, -29.25, 29.25, 56, 100)
    P["S3_XM540"] = dict(mesh=moved(s3, TA), mat="AL", mass_g=165, group="bought", colour=COL["motor"])
    P["S3_bracket"] = dict(mesh=moved(U(box(-19, 19, -32, 32, 52, 56), box(-19, 19, -32, 32, 100, 104)), TA), mat="AL", fill=0.4, group="joint", colour=COL["al"])
    P["upper_arm"] = dict(mesh=moved(cyl("z", 32, 104, 230), TA), mat="AL", group="arm", mass_g=0, colour=COL["stage"])
    P["elbow_ball"] = dict(mesh=moved(sph(44.0, (0, 0, 230)), TA), mat="AL", group="arm", mass_g=0, colour=COL["stage"])
    # fairing: low cover over the fixed stack and the S2 motor orbit sector (layer x -90..-30), 1.8 mm shell, hole r 56 round the S1 axis
    ang = np.radians(np.linspace(-102, 42, 40)); R_out = R2 + 44.5 + 3
    sector = [(0, 0)] + [(R_out * np.cos(a), R_out * np.sin(a)) for a in ang]
    outline = U(prism([tuple(p) for p in sector], "x", -91, -30), cyl("x", 62, -91, -30), cyl("x", 50, -91, -30, c=(-100, 0)).copy())
    inner = U(prism([tuple(p) for p in [(0, 0)] + [((R_out - 1.8) * np.cos(a), (R_out - 1.8) * np.sin(a)) for a in ang]], "x", -91, -31.8), cyl("x", 60.2, -91, -31.8), cyl("x", 48.2, -91, -31.8, c=(-100, 0)))
    cover = D(outline, inner, cyl("x", 56, -40, -25))
    P["fairing"] = dict(mesh=cover, mat="PACF", group="armour", colour=(0.20, 0.34, 0.52))
    return P
def coverage_report():
    ok = []; allr = []
    for t1 in np.arange(-95, 35.1, 2.0):
        for t2 in np.arange(-52, 80.1, 2.0):
            d = arm_dir(t1, t2); allr.append(d)
            beta = np.degrees(np.arccos(np.clip(-d[0], -1, 1)))          # angle to -x (into the stack axis)
            if beta > 54.5: ok.append(d)                                  # tube r32 vs stack cylinder r52 over 60 mm: blocked inside 54.5 deg
    return L.coverage(allr), L.coverage(ok), len(ok), len(allr)
if __name__ == "__main__":
    P = build(); cov_free, cov_blk, n1, n2 = coverage_report()
    cnt = [s["mesh"] for s in P.values() if s["group"] in ("joint", "bought", "armour")]
    depth = max(m.bounds[1][0] for m in cnt) + 108
    d_ref = baseline_dir(45, 25); d_new = arm_dir(TH1, TH2)
    # gravity / torque numbers
    TAU = 6.49; eta_b, eta_s = 0.95, 0.97; N = 2.0
    tau_s2 = TAU / (N * eta_b); tau_s1 = TAU / (N * eta_s); tau_s2_bal = 2.89 / (N * eta_b)
    tip_err = {a: round(575 * np.tan(np.radians(a)), 2) for a in (0.1, 0.2, 0.3)}
    extra = dict(own_stage=True, summary="Both AK70 lie flat (axes along x, back normal). S1 turns a carrier disc about x through a 2:1 belt from a fixed medial motor; the S2 motor rides on the carrier and drives the radial S2 axis through a 2:1 bevel pair; S3 XM540 along the arm. Joint stack is 4 mm plate + 50 mm motor layer + 8 mm carrier = 62 mm, but the arm base and tube set the overall depth. FINDING: with S1 along the back normal and the stack on the S1 axis, the arm cannot point within ~55 deg of -x (into the stack) so forward reach is lost unless S1 is offset (non-concurrent, see J4).",
        axes=dict(S1="about +x (back normal) through (y 0, z 0); AK70 fixed at y -100, 2:1 belt (PLACEHOLDER 60T/30T) to the carrier disc r52",
                  S2="radial axis in the carrier plane (= +y at S1 0), through P' = (x %.0f, 0, 0); AK70 on the carrier at radius 95, 2:1 bevel (PLACEHOLDER 20T/40T)" % PX, S3="along the arm, XM540 (33.5 mm in x)"),
        ranges_deg=dict(S1_soft=[-95, 35], S2_soft=[-52, 80], S3_soft=[-70, 90], hard="soft +-5 (passive stops on the carrier disc and the radial shaft, TPU bumpers)"),
        park_pose=dict(theta1=TH1, theta2=TH2, dir_new=[round(float(x), 3) for x in d_new], dir_baseline_S1_45_S2_25=[round(float(x), 3) for x in d_ref]),
        coverage=dict(kinematic_only=round(cov_free, 3), with_stack_exclusion=round(cov_blk, 3), reachable_samples=f"{n1}/{n2}",
                      lost="arm directions within 54.5 deg of -x (forward horizontal, e.g. baseline S2 > 60 with S1 < 40) are blocked: tube r32 hits the stack cylinder r52 over its 60 mm height; also a pole singularity at d = -x (S1 and S3 coaxial)"),
        depth_behind_back_mm=round(float(depth), 1), baseline_depth_mm=181,
        P_prime=dict(x=PX, behind_back_mm=round(108 + PX, 1), vs_baseline_P_mm=PX, note="P' is 8 mm closer to the back than baseline P; y, z unchanged"),
        torque=dict(inputs="worst gravity torque 6.49 N.m unbalanced (arm 1489 g, CG 249 mm, 0.5 kg payload, project number), 2.89 with the zero-length balancer; S1 now sees the same worst case because its axis is horizontal (arm horizontal along +-y)",
                    formula="tau_motor = tau_joint / (N * eta)", N=N, eta_bevel=eta_b, eta_belt=eta_s,
                    S2_motor_Nm=round(tau_s2, 2), S2_SF_vs_8p3=round(8.3 / tau_s2, 2), S2_with_balancer_Nm=round(tau_s2_bal, 2), S1_motor_Nm=round(tau_s1, 2), S1_SF_vs_8p3=round(8.3 / tau_s1, 2),
                    note="the 2:1 stages give SF>=2 on BOTH axes unbalanced (baseline S2 1.28 direct, balancer needed); balancer optional; speed: motor runs 2x the joint rate (tip cap 49.5 deg/s -> ~99 deg/s motor), within AK70 range (PLACEHOLDER limit)",
                    holding_heat="copper loss ~ tau^2: (3.42/6.49)^2 = 0.28 of the baseline direct-drive hold loss"),
        backlash=dict(assumed_deg=[0.1, 0.2, 0.3], tip_error_mm_at_575=tip_err, formula="e = 575 mm * tan(backlash)", belt="S1 belt: tensioned, no backlash, ~0.05 deg windup assumed", efficiency="bevel 0.95 (assumed 0.93-0.97): at 6.49 N.m, 49.5 deg/s (0.864 rad/s) output 5.6 W, loss 0.29 W; static hold motor torque x 1/eta"),
        stops="passive: S1 TPU bumpers on the carrier disc rim vs posts on the hub at soft +-5; S2 stop blocks on the bevel housing; hard stops 5 deg beyond soft; S2 crush-blade bumpers as baseline (PLACEHOLDER)",
        balancer="not required for SF>=2; if kept (free-fall energy rule, T12), a zero-length spring on the carrier acting on the radial shaft crank, in the x layer between carrier and cover",
        wiring="AK70 S1 bundle stays fixed; the S2 motor and XM540 wires pass through the hub bore (r28, 8.2 mm bundle fits) with a clock-spring over +-95 deg S1",
        mass_g=dict(baseline_bought=1407, baseline_mount_proxy=565.6, note="see metrics mount_mass_g (joint+bought+armour) and per-part rows"),
        decision_conflicts=["overrides ball-styled shoulder and concurrent S1/S2/S3 at P (Forest's request); S1 axis now horizontal so S1 carries gravity (baseline: none)", "no balancer needed (baseline I1b) but 2:1 stages add backlash"],
        contract_changes=["plate outline/holes (S1 base 4xM5 pattern gone), wing tongue seat must move to the new plate (y -148..60, z -62..62), clasp feet bolts re-sited", "keel/crown/spaulder replaced by the fairing; spaulder over the shoulder top not part of this concept"],
        pros=["thin 62 mm joint stack, everything but the arm base behind x = -28", "SF>=2 on S1 and S2 without a balancer", "motor heat down to ~0.28 of baseline hold loss", "hollow hub gives a clean cable path"],
        cons=["cannot point within ~55 deg of -x without offsetting S1; pole singularity", "bevel backlash 1-3 mm at the tip", "two gear stages to source (vendor PLACEHOLDER)", "exposed gear layer needs a lid / pinch review; ISO 13857"],
        risks=["belt/bevel stiffness unknown", "S2 motor orbit sweeps a 142 mm radius disc behind the shoulder", "S1 is now gravity-loaded: hold current continuous"],
        gates_to_rerun=["firmware keep-out tables (kinematics changed)", "control homology map and DLS mapping", "verify_shoulder equivalents, full sweep", "harness_loads (mass, CG)", "S2/S1 torque and thermal"],
        assumptions=["bevel eta 0.95, belt eta 0.97, backlash 0.2 deg (0.1-0.3), plate 4 mm AL, fill factors on pocketed parts, tube/stack clash rule from simple cylinders", "wing/clasp contracts as in clib.env_parts"])
    m = export_concept("J2", "Dual pancake + bevel (flat stack)", P, notes=dict(), extra=extra)
    print(m["mount_mass_g"], "depth", depth, "cov", cov_free, cov_blk)
    for r in m["parts"]: print(r["name"], r["group"], r["mass_g"], r.get("skin_min_mm"), r.get("protrusion_mm"))
