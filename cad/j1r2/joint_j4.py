"""J4 'Offset clavicle-style flat shoulder' (non-concurrent). S1 = AK70 flat on the wing plate, axis along x (back normal), turns a 92 mm clavicle link in the back (y,z) plane.
S2 = AK70 on the link end, axis t in the back plane (perpendicular to the link), disc in the (x,u) plane. S3 = XM540 along the arm. Proxy geometry; vendor details PLACEHOLDER."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, clib as L
from clib import *
A = np.array([0.0, -5.0, -60.0]); LK = 92.0; XC2 = -32.0            # S1 axis point (x set per part), link length, S2 disc centre x
ROPE = 44.6
def Rx(a): return rot(a, [1, 0, 0])
def uvec(phi): p = np.radians(phi); return np.array([0, np.sin(p), np.cos(p)])
def tvec(phi): p = np.radians(phi); return np.array([0, -np.cos(p), np.sin(p)])
def arm_frame(phi, s2):
    s = np.radians(s2); u = uvec(phi); x = np.array([1.0, 0, 0]); d = np.cos(s) * u - np.sin(s) * x; w = np.sin(s) * u + np.cos(s) * x; return d, w, tvec(phi)
def Bpt(phi): return np.array([XC2, A[1], A[2]]) + LK * uvec(phi)
def dir_fk(phi, s2): return arm_frame(phi, s2)[0]
# park-equivalent joint angles for baseline S1 45 / S2 25
d0 = baseline_dir(45, 25); S2P = np.degrees(np.arcsin(-d0[0])); PHIP = np.degrees(np.arctan2(d0[1], d0[2]))
def build():
    P = {}
    # ---- fixed side: flange (bolted to the plate rear face x -80) + ring clamp round the S1 AK70, motor axis x through (A_y, A_z)
    fl = box(-80, -74, A[1] - 54, A[1] + 54, A[2] - 54, A[2] + 54); fl = D(fl, cyl("x", 44.7, -81, -73, c=(A[1], A[2])))
    ring = cyl("x", 52.0, -74, -48, c=(A[1], A[2]), r_in=44.7); s1f = U(fl, ring)
    P["S1_flange_clamp"] = dict(mesh=s1f, mat="PETGCF", fill=0.45, group="joint", colour=COL["steel"])
    P["ak70_S1"] = dict(mesh=cyl("x", 44.5, -79.0, -28.75, c=(A[1], A[2])), mat="AL", mass_g=621.0, group="bought", colour=COL["motor"])
    # ---- moving side, built at phi = 0 (u = +z, t = -y) then rotated about the S1 axis to the park angle
    B0 = np.array([XC2, A[1], A[2] + LK]); mv = {}
    link = prism([(A[1] - 52, A[2]), (A[1] + 52, A[2]), (A[1] + 50, A[2] + LK - 43.5), (A[1] - 50, A[2] + LK - 43.5)], "x", -28.75, -22.0)
    link = U(link, cyl("x", 52, -28.75, -22.0, c=(A[1], A[2])))
    mv["link_plate"] = dict(mesh=link, mat="AL", fill=0.6, group="joint", colour=COL["al"])
    s2body = cyl("y", 44.5, B0[1] - 50.25, B0[1], c=(XC2, B0[2]))                       # flange end at y = B0_y (t = 0), body toward -y (t > 0)
    mv["ak70_S2"] = dict(mesh=s2body, mat="AL", mass_g=621.0, group="bought", colour=COL["motor"])
    clamp = cyl("y", 51.0, B0[1] - 42, B0[1] - 8, c=(XC2, B0[2]), r_in=44.7)
    clamp = U(clamp, box(XC2 - 50, XC2 + 50, B0[1] - 42, B0[1] - 8, B0[2] - 52, B0[2] - 40))
    mv["S2_clamp"] = dict(mesh=clamp, mat="PETGCF", fill=0.5, group="joint", colour=COL["steel"])
    fair = D(cyl("y", 52.8, B0[1] - 50.25 - 2, B0[1] + 1, c=(XC2, B0[2])), cyl("y", 51.0, B0[1] - 52.3, B0[1] - 0.0, c=(XC2, B0[2])))
    mv["fairing_S2"] = dict(mesh=fair, mat="PACF", group="armour", colour=(0.20, 0.34, 0.52))
    cover = U(prism([(A[1] - 53, A[2]), (A[1] + 53, A[2]), (A[1] + 51, A[2] + LK - 43), (A[1] - 51, A[2] + LK - 43)], "x", -22.0, -20.2), cyl("x", 53, -22.0, -20.2, c=(A[1], A[2])))
    mv["fairing_link"] = dict(mesh=cover, mat="PACF", group="armour", colour=(0.20, 0.34, 0.52))
    # S2 output: adapter disc on the flange (y > B0_y), arm axis at t0 = -6 mm, XM540 + tube + elbow ball along +z at phi 0, rotated by S2 about the S2 axis
    adp = cyl("y", 40, B0[1] + 0.5, B0[1] + 12, c=(XC2, B0[2]))
    arm = {}
    arm["s2_adapter"] = dict(mesh=adp, mat="PETGCF", fill=0.5, group="joint", colour=COL["steel"])
    xm = box(XC2 - 29.25, XC2 + 29.25, B0[1] + 6 - 16.75, B0[1] + 6 + 16.75, B0[2] + 64, B0[2] + 108)
    arm["xm540_S3"] = dict(mesh=xm, mat="AL", mass_g=165.0, group="bought", colour=COL["motor"])
    br = D(box(XC2 - 33, XC2 + 33, B0[1] + 6 - 20, B0[1] + 6 + 20, B0[2] + 61, B0[2] + 111), box(XC2 - 29.5, XC2 + 29.5, B0[1] + 6 - 17, B0[1] + 6 + 17, B0[2] + 60, B0[2] + 110))
    br = U(br, box(XC2 - 30, XC2 + 30, B0[1] + 11, B0[1] + 14, B0[2] + 30, B0[2] + 111))
    arm["s3_bracket"] = dict(mesh=br, mat="PETG", fill=0.5, group="joint", colour=COL["stage"])
    arm["upper_arm"] = dict(mesh=cyl("z", 32, B0[2] + 112, B0[2] + 230 - 38, c=(XC2, B0[1] + 6)), mat="PETG", group="arm", colour=COL["stage"], mass_g=0)
    arm["elbow_ball"] = dict(mesh=sph(44, (XC2, B0[1] + 6, B0[2] + 230)), mat="PETG", group="arm", colour=COL["stage"], mass_g=0)
    # S2 rotation: positive S2 swings +z toward -x about the t axis through B0 (t = -y): rotation about +y by -S2
    Ts2 = tr(B0) @ rot(-np.radians(S2P), [0, 1, 0]) @ tr(-B0)
    T1 = tr([0, A[1], A[2]]) @ Rx(-np.radians(PHIP)) @ tr([0, -A[1], -A[2]])
    for k, s in mv.items():
        m = s["mesh"].copy(); m.apply_transform(T1); s["mesh"] = m; P[k] = s
    for k, s in arm.items():
        m = s["mesh"].copy(); m.apply_transform(T1 @ Ts2); s["mesh"] = m; P[k] = s
    return P
def torque_study():
    g = 9.81; m_arm = 1.489; m_s2 = 0.621 + 0.12; res = {}
    cs = [0.10, 0.249, 0.444]            # CG distance (m) of the 1489 g beyond the S2 stage: folded .. fully extended (6.49 = m g c_max -> c_max 0.444)
    def tau(phi, s2, c, a):
        p = np.radians(phi); u = uvec(phi); d, w, t = arm_frame(phi, s2); B = np.array([0, LK * np.sin(p), LK * np.cos(p)]) / 1000.0
        F = np.array([0, 0, -g]); r_arm = B + c * d; r_s2 = B * 0.5                     # S2 motor+clamp CG roughly half-way to B (rotating with the link): lever 0.5 L .. B
        M = m_arm * np.cross(r_arm, F) + m_s2 * np.cross(0.75 * B, F)
        return float(np.dot(M, a))
    S1s = np.arange(-30, 90.1, 4); S2s = np.arange(-30, 88.1, 4); out = {}
    t1 = np.array([[[tau(p, s, c, np.array([1.0, 0, 0])) for c in cs] for s in S2s] for p in S1s])         # about S1 (x axis)
    t2 = np.array([[[tau(p, s, c, tvec(p)) for c in cs] for s in S2s] for p in S1s])
    out["S1_unbalanced_max"] = float(np.abs(t1).max()); out["S2_unbalanced_max"] = float(np.abs(t2).max())
    # zero-length-spring balancer: torque = K sin(phi) (S1) ; K sin(s2) cos(.) not needed for S2 baseline (K 3.789). minimax K for S1 over the grid
    Ks = np.linspace(-12, 12, 961); sinp = np.sin(np.radians(S1s))[:, None, None]
    best = min(((np.abs(t1 - K * sinp).max(), K) for K in Ks)); out["S1_balanced_residual"] = float(best[0]); out["S1_K_Nm"] = float(best[1])
    out["S2_balanced_residual_baseline"] = 2.89
    return out
def kin_study():
    # joint ranges that cover the baseline direction set (every baseline direction has dz > 0): phi = -30..90, s2 = -30..88; verify numerically
    S1s = np.arange(-31, 91, 1.0); S2s = np.arange(-31, 89, 1.0)
    dirs = np.array([dir_fk(p, s) for p in S1s for s in S2s]); cov = coverage(dirs, 3.0)
    S1a = np.arange(-30, 90.1, 1.0); S2a = np.arange(-30, 88.1, 1.0); dirs2 = np.array([dir_fk(p, s) for p in S1a for s in S2a]); cov2 = coverage(dirs2, 3.0)
    Bs = np.array([Bpt(p) for p in S1a]); lever = float(np.max(np.linalg.norm(np.array([0, 0, 0]) + (np.array([0, 0, 0])), axis=0)))
    # tip lever for the tip-speed cap: baseline 578 mm (P to claw tip); J4: S1 lever = |tip - S1 axis| max ; S2 lever = 578 (tip from B)
    tip = lambda p, s: Bpt(p) + 578.0 * dir_fk(p, s)
    lev1 = max(np.hypot(*(tip(p, s)[1:] - A[1:])) for p in S1a[::6] for s in S2a[::6])
    return dict(cov_soft=cov2, cov_with_5deg=cov, B_arc=dict(phi_min=Bs[0].round(0).tolist(), phi0=Bpt(0).round(0).tolist(), phi_max=Bs[-1].round(0).tolist(),
                dy_range=float(Bs[:, 1].max() - Bs[:, 1].min()), dz_range=float(Bs[:, 2].max() - Bs[:, 2].min())), lever_S1_mm=float(lev1), lever_S2_mm=578.0)
def spot_checks(P):
    rows = []
    for nm, (phi, s2) in dict(park=(PHIP, S2P), up=(0, 0), lateral=(90, 0), fwd_lat=(60, 60), medial=(-30, 0), fwd=(0, 85)).items():
        B = Bpt(phi); d = dir_fk(phi, s2); cap = cyl("z", 32, 0, 590)
        z = np.array([0, 0, 1.0]); ax = np.cross(z, d); n = np.linalg.norm(ax)
        T = tr(B) if n < 1e-9 else tr(B) @ trimesh.transformations.rotation_matrix(np.arccos(np.clip(z @ d, -1, 1)), ax / n)
        cap.apply_transform(T); rows.append((nm, round(skin_gap(cap, 5000)[0], 1)))
    return rows
def sweep_s2(P):
    out = {}
    for phi in (-30, 0, 45, 90):                                                    # static spot checks of the S2 body + clamp ring against the skin (not a sweep)
        B0 = np.array([XC2, A[1], A[2] + LK]); m = U(cyl("y", 51, B0[1] - 50.25, B0[1], c=(XC2, B0[2])))
        m.apply_transform(tr([0, A[1], A[2]]) @ Rx(-np.radians(phi)) @ tr([0, -A[1], -A[2]])); out[phi] = round(skin_gap(m, 4000)[0], 1)
    return out
if __name__ == "__main__":
    P = build(); kin = kin_study(); tq = torque_study(); spots = spot_checks(P); s2s = sweep_s2(P)
    arm_m = [s["mesh"] for k, s in P.items() if s["group"] in ("joint", "armour", "bought")]; depth = union_protrusion(arm_m)
    # plate clearance of the S2 body: plate rear face x = -80, S2 rear x = XC2 - 44.5
    extra = dict(own_stage=True, summary="J4: S1 (AK70, axis x = back normal) flat on the plate turns a 92 mm clavicle link; S2 (AK70) side-by-side on the link end with its axis in the back plane; S3 XM540 along the arm. Non-concurrent: the arm root moves on a 92 mm arc. Reaches %.0f %% of the baseline arm-direction set, %d mm deep vs 181, but S1 now sees gravity (%.1f N.m unbalanced)." % (100 * kin["cov_soft"], round(depth), tq["S1_unbalanced_max"]),
        axes=dict(S1="AK70 #1, axis +x through (y %.0f, z %.0f), turns the link; phi measured from +z toward +y" % (A[1], A[2]), S2="AK70 #2 at the link end B(phi) = (x %.0f, y %.0f + 92 sin phi, z %.0f + 92 cos phi), axis t = (0, -cos phi, sin phi) in the back plane, positive swings the arm toward -x" % (XC2, A[1], A[2]), S3="XM540 along the arm axis d = cos(s2) u - sin(s2) x, offset 6 mm from the S2 flange plane"),
        ranges=dict(S1_soft_deg=[-30, 90], S2_soft_deg=[-30, 88], hard="5 deg beyond each (passive stops: lug + posts on the S1 flange ring, S2 crush bumpers on the clamp)", park_equiv=dict(S1_deg=round(PHIP, 1), S2_deg=round(S2P, 1))),
        coverage=dict(baseline_direction_set_covered_soft_range=kin["cov_soft"], with_5deg_margin=kin["cov_with_5deg"], unreachable="none in the baseline set (every baseline direction has dz>0: phi=atan2(dy,dz) in (-90,90), s2=asin(-dx) in [-30,88]); J4 additionally reaches directions with dz<0 only beyond these ranges", formula="d = (-sin s2, cos s2 sin phi, cos s2 cos phi)"),
        arm_root_workspace=dict(B_arc=kin["B_arc"], note="root moves 92 mm radius arc: ends %s mm .. %s mm; baseline P fixed at (0,0,0)" % (kin["B_arc"]["phi_min"], kin["B_arc"]["phi_max"])),
        depth_behind_back_mm=round(depth, 1), baseline_depth_mm=181, P_note="no single P: S2 axis point B(phi) at x=%.0f (baseline P x=0): arm root sits %.0f mm closer to the back" % (XC2, 0 - XC2),
        torque=dict(unbalanced_S1_Nm=round(tq["S1_unbalanced_max"], 2), unbalanced_S2_Nm=round(tq["S2_unbalanced_max"], 2), S1_balanced_residual_Nm=round(tq["S1_balanced_residual"], 2), S1_K_Nm=round(tq["S1_K_Nm"], 2), S2_balanced_baseline_Nm=2.89,
                    SF_S1_unbalanced=round(8.3 / tq["S1_unbalanced_max"], 2), SF_S1_balanced=round(8.3 / tq["S1_balanced_residual"], 2), SF_S2_unbalanced=round(8.3 / tq["S2_unbalanced_max"], 2), SF_S2_balanced=round(8.3 / 2.89, 2),
                    formulas="tau_a = a . sum m_i (r_i x g); arm 1.489 kg at CG c = 0.10..0.444 m along d from B (c_max from 6.49 = m g c); S2 motor+clamp 0.741 kg at 0.75 B; g = 9.81; S1 axis a = x, S2 axis a = t(phi); balancer tau = K sin(phi), minimax K over the grid phi -30..90 step 4, s2 -30..88 step 4, c in {.10,.249,.444}",
                    inputs="m_arm 1.489 kg, m_s2 0.741 kg, L 0.092 m", transmission="none (direct drive AK70 10:1 internal), backlash as baseline"),
        spot_checks_static_not_a_sweep=dict(straight_arm_capsule_R32_L590_skin_gap_mm=dict(spots), S2_body_clamp_skin_gap_mm=s2s),
        stop_strategy="S1: lug on the link plate against two TPU-capped posts on the flange ring at phi -35 / +95; S2: crush bumpers on the clamp vs the adapter ears at -35 / +93 (soft -30/+88, 5 deg margin); PLACEHOLDER geometry, not printed",
        balancer="S2 zero-length spring as baseline (K 3.789 N.m) between the link plate and the S2 adapter; S1 NEEDS a second zero-length balancer: cord from the link tip over a sheave on the flange ring, torque K1 sin(phi), K1 = %.2f N.m (+~330 g, not modelled)" % tq["S1_K_Nm"],
        wiring="AK70 S1 pigtail exits the rear of the flange ring (x -79); link bundle passes over the S1 output face under the link cover to the S2 clamp (bend radius > 25 mm at the plate); S2 -> S3 through the adapter bore; 8.2 mm bundle fits the 2 mm gap under fairing_link only if routed in a 10 mm trench: PLACEHOLDER, open",
        contract_changes=["plate: top edge raised from z -54 to about -16 and S1 pattern changed to 4 x M5 on a 76 mm square about (y -5, z -60) (was 4 x M5 at y +-62, z -68/-94); S1 base block, crown, keel and spaulder are removed", "the 2 x M3 spaulder top holes lose their part (a slim spaulder can be re-added)"],
        decision_conflicts=["ball-styled shoulder and concurrent S1/S2/S3 at P: dropped (Forest's request); cowl/dome retired", "S1 now has a horizontal axis: gravity load on S1 (baseline S1 has none): AK60-6 swap stays out, needs the second balancer", "SPEC homology map and keep-out tables assume a fixed P"],
        pros=["depth +%d mm vs 181 (-%d %%) with the same actuators" % (round(depth), round(100 * (1 - depth / 181))), "arm root can translate like a clavicle (dy %.0f mm, dz %.0f mm) which helps reach over the shoulder and gives the scapular-follow behaviour for free" % (kin["B_arc"]["dy_range"], kin["B_arc"]["dz_range"]), "no cowl, dome or ball: two flat discs under 1.8 mm fairings", "same actuators and ranges; the baseline direction set is fully covered"],
        cons=["S1 carries gravity: 7-8 N.m unbalanced against the 8.3 cap, a second balancer is mandatory", "rotating S2 disc sweeps a 140 mm radius ring 3 mm in front of the plate: wing/clasp feet must stay behind x -80, any cable or plate screw head in that zone collides", "arm root dips to z %.0f at phi 90, lateral arm runs low near the deltoid (see spot checks)" % Bpt(90)[2], "non-concurrent: new kinematics, keep-out tables, homology map and tip-speed levers (S1 lever %.0f vs 578 mm => S1 cap 49.5 -> %.1f deg/s)" % (kin["lever_S1_mm"], 49.5 * 578 / kin["lever_S1_mm"])],
        risks=["S2 disc/plate 3.5 mm static gap only; link plate flex under the 8.3 N.m S2 reaction (aluminium 6 mm: check)", "head/neck keep-out must be re-derived: arm root moves 51 mm medially at phi -30"],
        gates_to_rerun=["verify_shoulder (all moving groups re-made), collision_frontier / sweep_full with the new groups", "keepout_map.py -> gen_tables.py (elbow_limits.h, keepout_table.h, balancer_keepout.h, grav_model.h)", "control_map.py homology: replace the spherical DLS inverse by the (phi, s2, s3) Jacobian; tip-speed levers", "balancer_calc for two balancers, s2_grid / s2_massmodel", "harness_loads (mass), verify_harness (top pad margin 2 %)", "chassis_axilla.py (new plate outline + S1 pattern)"],
        assumptions=["F1/F4 seats, wing and rail are proxies; S2 clamp/flange geometry and AK70 flange pattern PLACEHOLDER", "balancer masses not modelled; elbow lever c_max 0.444 m derived from 6.49 N.m", "arm shown at park-equivalent direction, elbow ball only"])
    meta = export_concept("J4", "Offset clavicle-style shoulder", P, notes={}, extra=extra)
    print(meta["mount_mass_g"], depth); import json; print(json.dumps(extra["torque"]), json.dumps(extra["coverage"]["baseline_direction_set_covered_soft_range"]), extra["spot_checks_static_not_a_sweep"], kin["B_arc"], kin["lever_S1_mm"])
    for r in meta["parts"]: print(r["name"], r["mass_g"], r.get("skin_min_mm"), r.get("protrusion_mm"), r["watertight"], r["fits_256"])
