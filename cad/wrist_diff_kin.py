"""Bevel-gear DIFFERENTIAL wrist (alternative to the serial W2/W3 wrist): kinematics, servo travel/torque, gear sizing, backlash.
Writes stl/wrist_diff/wrist_diff_kin.json (new folder, new file only). Single core, ~10 s.  Run: python3 wrist_diff_kin.py

ARCHITECTURE (the only one that reproduces the serial W2/W3 pointing set; see docs/wrist_diff_report.md)
  side gears A, B on a common axis y (the W2 axis direction, through C = (0, D, Zc)), driven by two forearm-fixed STS3215 (MW1 frame).
  Pinion(s) P on a cross shaft carried by the free-spinning carrier ring (axis y); the pinion axis is x (rotated by the carrier).
  The HAND is bolted to the pinion with the claw axis along z, i.e. PERPENDICULAR to the pinion axis.  Then
      carrier angle  th  (about y)           = (a + b)/2
      pinion spin    ph  (about carrier x)   = rho (a - b)/2,   rho = Zg/Zp
  and R_hand = Ry(th) Rx(ph)  ==  the serial chain Ry(-W2) Rx(W3)  with  W2 = -th,  W3 = ph.  Pointing AND roll are identical to the serial wrist.
  (Mounting the hand ALONG the pinion axis would give pitch + spin of an axis that only pitches, i.e. no yaw: not usable.)
"""
import os, sys, json, itertools, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import rom_limits as rl
OUT = os.path.join(HERE, "..", "stl", "wrist_diff")
D2R = np.pi / 180.0
def rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

# ------------------------------------------------------------------ kinematics
def servo_to_hand(a, b, rho=1.0):
    """side-gear angles a, b (deg, about +y) -> hand rotation matrix, W2, W3 (serial-equivalent joint angles, deg)."""
    th = 0.5 * (a + b); ph = 0.5 * rho * (a - b)
    return ry(th * D2R) @ rx(ph * D2R), -th, ph
def hand_to_servo(W2, W3, rho=1.0):
    th, ph = -W2, W3
    return th + ph / rho, th - ph / rho            # a, b
def serial_R(W2, W3): return ry(-W2 * D2R) @ rx(W3 * D2R)    # arm_kin: RW2 = Ry(-W2), RW3 = RW2 Rx(W3)
def twist_deg(R):
    """roll of R about its own claw axis relative to the minimal (tilt-only) rotation that takes z to R z."""
    z = np.array([0, 0, 1.0]); v = R @ z; ax = np.cross(z, v); s = np.linalg.norm(ax); c = float(z @ v)
    if s < 1e-12: Rm = np.eye(3)
    else:
        ax /= s; K = np.array([[0, -ax[2], ax[1]], [ax[2], 0, -ax[0]], [-ax[1], ax[0], 0]]); Rm = np.eye(3) + s * K + (1 - c) * K @ K
    Rt = Rm.T @ R
    return float(np.degrees(np.arctan2(Rt[1, 0], Rt[0, 0])))

def kinematics(rho):
    w2lo, w2hi = rl.SOFT["W2"]; w3lo, w3hi = rl.SOFT["W3"]
    g2 = np.linspace(w2lo, w2hi, 31); g3 = np.linspace(w3lo, w3hi, 26)
    maxdiff = 0.0; tw = []; ab = []; jac = []
    for W2 in g2:
        for W3 in g3:
            a, b = hand_to_servo(W2, W3, rho); ab.append((a, b))
            R, w2b, w3b = servo_to_hand(a, b, rho)
            maxdiff = max(maxdiff, float(np.abs(R - serial_R(W2, W3)).max()), abs(w2b - W2), abs(w3b - W3))
            tw.append(twist_deg(R)); jac.append(np.cos(W3 * D2R))
    ab = np.array(ab)
    return dict(rho=rho, servo_a_deg=[float(ab[:, 0].min()), float(ab[:, 0].max())], servo_b_deg=[float(ab[:, 1].min()), float(ab[:, 1].max())],
                max_abs_diff_vs_serial=maxdiff, twist_deg=[float(min(tw)), float(max(tw))], pointing_jacobian_min=float(min(jac)),
                singular_W3_deg=[-90.0, 90.0], note="pointing Jacobian = cos(W3); singular only at W3 = +-90, 60 deg outside the soft range; the gear train itself has no singularity")

def tip_difference(Zc, Zw=410.0, Zw3=455.0, Ztip=575.0):
    """tip position of the serial wrist (W2 about z=Zw, W3 about z=Zw3) vs the differential pivoting about the single centre Zc (same angles, same hand)."""
    out = 0.0
    for W2 in np.linspace(*rl.SOFT["W2"], 15):
        for W3 in np.linspace(*rl.SOFT["W3"], 11):
            R2 = ry(-W2 * D2R); p = np.array([0, 0, Ztip]); c2 = np.array([0, 0, Zw]); c3 = np.array([0, 0, Zw3])
            ps = c2 + R2 @ (c3 - c2) + R2 @ rx(W3 * D2R) @ (p - c3)
            cc = np.array([0, 0, Zc]); pd = cc + R2 @ rx(W3 * D2R) @ (p - cc)
            out = max(out, float(np.linalg.norm(ps - pd)))
    return out

# ------------------------------------------------------------------ hand mass and holding torque
def hand_mass():
    """hand (arm_kin MW3 group) mass and CG z from the meshes, same density rules as verify_distal.py (that script needs stl/distal/claw/verify_claw.json, which is
    not in the repo, so the claw hardware line is the 8.4 g quoted in verify_distal's own comment, at z=520)."""
    import arm_kin as ak
    G = ak.groups()
    PROXY = ("sts", "xm_", "ak_", "shaft", "pin_", "stop_pin", "bearing", "scr_")
    def dens(k):
        if k.startswith("tpu"): return 1.21
        b = k.rstrip("0123456789")
        if b in ("hand_tray", "gear_cover", "servo_tab", "stop_cap"): return 1.30
        if b in ("claw_rotor", "claw_ring", "prox_", "dist_", "link_drive_", "link_couple_", "claw_pinion", "claw_idler"): return 1.20
        return 1.27
    items = []
    for k, m in G["MW3"].items():
        if k.startswith("w3_lug") or k.startswith(PROXY): continue      # w3_lug: serial-wrist fixtures, not part of the differential hand
        vol = abs(m.volume) / 1e3
        if vol <= 0: continue
        c = m.center_mass if m.is_watertight else m.bounds.mean(0)
        items.append((k, vol * dens(k) / 1e3, float(c[2])))
    items += [("sts_g", 0.055, float(G["MW3"]["sts_g"].bounds.mean(0)[2])), ("claw hardware", 0.0084, 520.0)]
    m = sum(i[1] for i in items); zcg = sum(i[1] * i[2] for i in items) / m
    yoke = abs(G["MW2"]["w2_yoke"].volume) / 1e3 * 1.20 / 1e3
    return dict(m_hand_kg=m, z_cg_mm=zcg, yoke_kg=yoke, yoke_z_mm=float(G["MW2"]["w2_yoke"].center_mass[2]), sts_w3_kg=0.055,
                sts_w3_z_mm=float(G["MW2"]["sts_w3"].bounds.mean(0)[2]), n_items=len(items))

def holding_torques(Zc, hm, pay=0.5, Ztip=575.0, g=9.81):
    """static, worst pose (claw horizontal) exactly as verify_distal.py: (|sum m*(z-z0)| + payload*(Ztip-z0)) * g, payload 0.5 kg at Z_TIP.
    Differential: only the hand moves, about the common centre Zc, for BOTH axes.  Serial W3 about 455 moves the hand; serial W2 about 410 moves yoke + W3 servo + hand."""
    m = hm["m_hand_kg"]
    t_diff = (abs(m * (hm["z_cg_mm"] - Zc) / 1e3) + pay * (Ztip - Zc) / 1e3) * g
    t3 = (abs(m * (hm["z_cg_mm"] - 455.0) / 1e3) + pay * (Ztip - 455.0) / 1e3) * g
    M2 = m * (hm["z_cg_mm"] - 410.0) / 1e3 + hm["yoke_kg"] * (hm["yoke_z_mm"] - 410.0) / 1e3 + hm["sts_w3_kg"] * (hm["sts_w3_z_mm"] - 410.0) / 1e3
    t2 = (abs(M2) + pay * (Ztip - 410.0) / 1e3) * g
    return dict(tau_diff_each_axis_Nm=t_diff, tau_serial_W3_Nm=t3, tau_serial_W2_Nm=t2, payload_kg=pay)

# ------------------------------------------------------------------ gears
LEWIS_Y = [(12, .245), (13, .261), (14, .277), (15, .290), (16, .296), (17, .303), (18, .309), (19, .314), (20, .322), (21, .328), (22, .331), (24, .337), (26, .346),
           (28, .353), (30, .359), (34, .371), (38, .384), (43, .397), (50, .409), (60, .422), (75, .435), (100, .447), (200, .463), (400, .480)]   # Shigley Table 14-2, 20 deg full depth
def lewis_Y(zv): return float(np.interp(zv, [a for a, _ in LEWIS_Y], [b for _, b in LEWIS_Y]))
def bevel(m, Zp, Zg, b, n_pin=2):
    """straight bevel pair, 90 deg shaft angle, 20 deg PA.  Zg = side gears, Zp = pinion.  Lewis at the mean section (virtual teeth Z/cos(delta))."""
    dp, dg = m * Zp, m * Zg
    dlt_p = np.degrees(np.arctan2(Zp, Zg)); dlt_g = 90.0 - dlt_p
    A0 = dg / 2 / np.sin(np.radians(dlt_g))
    r_mean_g = dg / 2 * (1 - b / (2 * A0)); r_mean_p = dp / 2 * (1 - b / (2 * A0)); m_mean = m * (1 - b / (2 * A0))
    Zv_p = Zp / np.cos(np.radians(dlt_p)); Zv_g = Zg / np.cos(np.radians(dlt_g))
    od_g = dg + 2 * m * np.cos(np.radians(dlt_g)); od_p = dp + 2 * m * np.cos(np.radians(dlt_p))
    tip_r = lambda d, dl: np.hypot(d / 2 + m * np.cos(np.radians(dl)), d / 2 / np.tan(np.radians(dl)) - m * np.sin(np.radians(dl)))
    sphere = 2 * max(tip_r(dg, dlt_g), tip_r(dp, dlt_p))      # bounding sphere about the apex over the toothed heel corners
    return dict(m=m, Zp=Zp, Zg=Zg, b=b, rho=Zg / Zp, dlt_p=dlt_p, dlt_g=dlt_g, A0=A0, b_max=A0 / 3.0, od_gear=od_g, od_pinion=od_p, sphere_dia=sphere,
                r_mean_g=r_mean_g, r_mean_p=r_mean_p, m_mean=m_mean, Zv_p=Zv_p, Zv_g=Zv_g, Yp=lewis_Y(Zv_p), Yg=lewis_Y(Zv_g), n_pin=n_pin)
def lewis(gp, tauA_Nm, Ko=1.0, Kv=1.0):
    """bending stress, MPa.  tauA = torque a side gear passes to the pinion set; n_pin pinions share it."""
    Ft = tauA_Nm * 1e3 / gp["r_mean_g"] / gp["n_pin"]
    return Ft, Ko * Kv * Ft / (gp["b"] * gp["m_mean"] * gp["Yp"]), Ko * Kv * Ft / (gp["b"] * gp["m_mean"] * gp["Yg"])

def servo_loads(rho, tau_w2, tau_w3, i=1.0, eta=0.9):
    """P = tau_th th' + tau_ph ph' with th' = (a'+b')/2, ph' = rho (a'-b')/2  ->  tauA = tau_th/2 + rho tau_ph/2, tauB = tau_th/2 - rho tau_ph/2
    (worst sign combination).  Servo torque = tauA / (i eta), i = transfer ratio servo->side gear (1 = servo shaft drives the side gear directly)."""
    worst = max(max(abs(s2 * tau_w2 / 2 + s3 * rho * tau_w3 / 2), abs(s2 * tau_w2 / 2 - s3 * rho * tau_w3 / 2)) for s2, s3 in itertools.product((-1, 1), repeat=2))
    return worst, worst / i / eta

def backlash(gp, jt_mm, servo_bl_deg=0.5, i=1.0, lever_mm=165.0, lever_far_mm=345.0):
    """hand angle error, deg.  Each mesh has circumferential play jt -> angle e = jt/r_mean_g at the side gear.  With both servos held the pinion/carrier can move inside
    |th + ph/rho| <= e/2, |th - ph/rho| <= e/2: pitch error +-e/2, tilt (W3) error +-rho e/2.  Servo output slack (ASSUMED 0.5 deg at the servo shaft, /i) adds to e."""
    e = jt_mm / gp["r_mean_g"]; et = e + np.radians(servo_bl_deg) / i
    pitch = np.degrees(et / 2); tilt = np.degrees(gp["rho"] * et / 2)
    return dict(pitch_gear_only_pm_deg=np.degrees(e / 2), tilt_gear_only_pm_deg=np.degrees(gp["rho"] * e / 2), pitch_pm_deg=pitch, tilt_pm_deg=tilt,
                tip_mm_at_lever=np.radians(max(pitch, tilt)) * lever_mm, lever_mm=lever_mm,
                tip_mm_far=np.radians(max(pitch, tilt)) * lever_far_mm, lever_far_mm=lever_far_mm)

# ------------------------------------------------------------------ main
if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True); res = {}
    print("== kinematics (rho = Zg/Zp) ==")
    res["kin"] = {}
    for rho in (1.0, 2.0):
        k = kinematics(rho); res["kin"][str(rho)] = k
        print(f"rho {rho}: servo a {k['servo_a_deg'][0]:.1f}..{k['servo_a_deg'][1]:.1f}, b {k['servo_b_deg'][0]:.1f}..{k['servo_b_deg'][1]:.1f} deg; "
              f"max |R_diff - R_serial| = {k['max_abs_diff_vs_serial']:.1e}; tilt-twist {k['twist_deg'][0]:.1f}..{k['twist_deg'][1]:.1f} deg (identical to serial); min Jacobian {k['pointing_jacobian_min']:.3f}")
    from scipy.optimize import least_squares
    worst_W1 = 0.0; worst_tw = 0.0; fail = 0; n = 0
    for W2 in np.linspace(*rl.SOFT["W2"], 7):
        for W3 in np.linspace(*rl.SOFT["W3"], 6):
            n += 1; R0 = serial_R(W2, W3); worst_tw = max(worst_tw, abs(twist_deg(R0))); v0 = R0 @ np.array([0, 0, 1.0])
            def f(x):
                R = rz(x[0] * D2R) @ serial_R(x[1], x[2]); return np.r_[R @ np.array([0, 0, 1.0]) - v0, np.radians(twist_deg(R))]
            sol = least_squares(f, [0.0, W2, W3], bounds=([rl.SOFT["W1"][0], *[-179] * 2], [rl.SOFT["W1"][1], 179, 179]))
            ok = np.abs(sol.fun).max() < 1e-6 and rl.SOFT["W2"][0] - 1e-6 <= sol.x[1] <= rl.SOFT["W2"][1] + 1e-6 and rl.SOFT["W3"][0] - 1e-6 <= sol.x[2] <= rl.SOFT["W3"][1] + 1e-6
            if ok: worst_W1 = max(worst_W1, abs(sol.x[0]))
            else: fail += 1
    res["twist_absorb"] = dict(max_abs_twist_deg=worst_tw, max_W1_needed_deg=worst_W1, unsolved=fail, n=n)
    print("tilt-twist: max |twist| %.1f deg over the range; W1 re-solve (zero twist, same claw axis, W2/W3 inside soft limits): max |W1| %.1f deg, %d/%d poses unsolved" % (worst_tw, worst_W1, fail, n))

    print("\n== hand mass and holding torque ==")
    hm = hand_mass(); res["hand"] = hm
    print("hand: %.0f g, CG z %.1f mm (%d items); serial extras: W2 yoke %.0f g @ z %.0f, STS3215 W3 55 g @ z %.0f" % (hm["m_hand_kg"] * 1e3, hm["z_cg_mm"], hm["n_items"], hm["yoke_kg"] * 1e3, hm["yoke_z_mm"], hm["sts_w3_z_mm"]))
    res["tip_pos_diff_mm"] = {}; res["torque"] = {}
    for Zc in (410.0, 417.0, 425.0, 455.0):
        res["tip_pos_diff_mm"][str(Zc)] = tip_difference(Zc); res["torque"][str(Zc)] = t = holding_torques(Zc, hm)
        print(f"centre Zc={Zc:.0f}: diff tau per axis {t['tau_diff_each_axis_Nm']:.2f} N.m | serial W2 {t['tau_serial_W2_Nm']:.2f}, W3 {t['tau_serial_W3_Nm']:.2f} | tip position diff vs serial <= {res['tip_pos_diff_mm'][str(Zc)]:.1f} mm")

    print("\n== gear options (Zc 417, tau as above, Ko 1.5 on rated; stall = 2.94 N.m straight into one side gear; jt 0.20 mm) ==")
    Zc = 417.0; tau = res["torque"][str(Zc)]["tau_diff_each_axis_Nm"]; tau_stall = 2.94; opts = []
    for m, Zp, Zg in [(1.0, 14, 14), (1.5, 14, 14), (1.5, 16, 16), (2.0, 14, 14), (2.0, 16, 16), (2.5, 14, 14), (3.0, 14, 14), (1.0, 14, 28), (1.25, 14, 28), (1.5, 14, 28), (2.0, 14, 28)]:
        for b in (5.0, 6.0, 8.0):
            gp = bevel(m, Zp, Zg, b)
            if b > gp["b_max"] + 1e-9: continue
            tA, tsv = servo_loads(gp["rho"], tau, tau)
            Ft, sp, sg = lewis(gp, tA, Ko=1.5); Fts, sps, sgs = lewis(gp, tau_stall)
            bl = backlash(gp, 0.20)
            opts.append(dict(gp, tauA_req=tA, servo_tau_req=tsv, servo_SF=tau_stall / tsv, Ft_req=Ft, sigma_pin_req_MPa=sp, sigma_gear_req_MPa=sg, sigma_pin_stall_MPa=sps, backlash=bl,
                             fits60=bool(gp["sphere_dia"] <= 60.0 and gp["od_gear"] <= 60.0)))
    for o in opts:
        print(f"m{o['m']:<4} Z{o['Zp']}/{o['Zg']} b{o['b']:<3.0f} OD g/p {o['od_gear']:.1f}/{o['od_pinion']:.1f} sphere {o['sphere_dia']:.1f} fits60 {o['fits60']} | servo tau {o['servo_tau_req']:.2f} N.m (SF {o['servo_SF']:.2f}) | "
              f"Ft {o['Ft_req']:.0f} N sig_pin {o['sigma_pin_req_MPa']:.0f} (stall {o['sigma_pin_stall_MPa']:.0f}) MPa | gear-only play pitch +-{o['backlash']['pitch_gear_only_pm_deg']:.2f} tilt +-{o['backlash']['tilt_gear_only_pm_deg']:.2f} deg")
    res["options"] = opts
    rec = [o for o in opts if o["m"] == 2.5 and o["Zp"] == 14 and o["Zg"] == 14 and o["b"] == 8.0][0]
    res["recommended"] = dict(m=2.5, Zp=14, Zg=14, b=8.0, n_pin=2, note="miter pair, two pinions; largest module whose bounding sphere stays <= 60 mm", detail=rec)
    print("\nrecommended m2.5 Z14/14 b8 miter, 2 pinions:", {k: round(v, 2) for k, v in rec.items() if isinstance(v, float)})
    res["backlash_sweep"] = {}
    for label, jt in (("as printed 0.30 mm", 0.30), ("tuned 0.20 mm", 0.20), ("spring-preloaded 0.08 mm eff.", 0.08), ("metal 0.05 mm", 0.05)):
        bl = backlash(rec, jt); res["backlash_sweep"][label] = bl
        print(f"{label}: gear-only +-{bl['pitch_gear_only_pm_deg']:.2f} deg; with servo slack 0.5 deg: +-{bl['pitch_pm_deg']:.2f} deg -> +-{bl['tip_mm_at_lever']:.1f} mm at 165 mm (centre->claw tip), +-{bl['tip_mm_far']:.1f} mm at 345 mm")
    # stress verdict
    print("Allowables used (ASSUMED, to be verified by coupon test): PA-CF printed bending static 60 MPa XY, fatigue 20 MPa; hardened steel 250; brass 100.")
    json.dump(res, open(os.path.join(OUT, "wrist_diff_kin.json"), "w"), indent=1, default=float)
    print("wrote", os.path.join(OUT, "wrist_diff_kin.json"))
