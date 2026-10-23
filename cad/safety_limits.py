"""Human-robot interface safety numbers for the Third Shoulder (session 2026-10-12b). Feeds SPEC_safety.md and control_map.py.

Sources (see DESIGN_STATE §4b):
  ISO/TS 15066:2016 Annex A body-model values as reproduced in arXiv 2311.13814 (Table 1: max permissible force, effective
  spring constant, effective human mass); transient contact allowed at 2x the quasi-static value except skull/forehead/face,
  where transient contact is not permitted (TS 15066 §5.5.5.4); v_rel,max = F / sqrt(mu k), mu = (1/m_H + 1/m_R)^-1,
  m_R = M/2 + m_L.
  Pressure pain threshold, trapezius 88..574 kPa, deltoid 91..529 kPa (Persson et al., J Rehabil Med 36(1), 2004).
  Capillary closing pressure ~32 mmHg = 4.3 kPa (Landis 1930; classic value used for sustained-contact limits).
  ISO 13854 minimum gap to avoid crushing a finger: 25 mm.
Masses: rev 2026-10-13f (unify): arm beyond S2 1.58 kg (ball-elbow upper arm + claw hand, harness_loads.py; 1.13 kg before); payload 0.5 kg;
AK70-10 621 g; yoke+clamps 0.25 kg is an ESTIMATE.
Run: python3 cad/safety_limits.py -> stl/control/safety_limits.json
"""
import numpy as np, json, os, sys
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.join(HERE, "..")
sys.path.insert(0, HERE)
import biomimetic_shoulder as bs

TS15066 = {  # region: (F_max quasi-static N, k N/mm, m_H kg, transient allowed)
 "skull_forehead": (130, 150, 4.4, False), "face": (65, 75, 4.4, False), "neck": (150, 50, 1.2, True),
 "back_shoulders": (210, 35, 40, True), "chest": (140, 25, 40, True), "abdomen": (110, 10, 40, True),
 "upper_arm_elbow": (150, 30, 3.0, True), "lower_arm_wrist": (160, 40, 2.0, True), "hand_fingers": (140, 75, 0.6, True)}

M_ARM, M_PAY, M_S2_ACT, M_YOKE_EST, M_BALL = 1.58, 0.50, 0.621, 0.25, 0.139
CASES = {"arm_swing_S2_S3_E": M_ARM / 2 + M_PAY, "whole_stage_S1": (M_ARM + M_S2_ACT + M_YOKE_EST + M_BALL) / 2 + M_PAY}

def vmax(F, k_Nmm, mH, mR):
    mu = 1.0 / (1.0 / mH + 1.0 / mR); return F / np.sqrt(mu * k_Nmm * 1e3)

def head_keepout():
    """Minimum distance from the straight arm (P -> 500 mm, elbow at 0) to a head sphere over the soft-limit grid.
    Head centre ESTIMATE: neck joint landmark (-176,-150,-87) + 130 mm up; radius 95 mm (half head breadth + hair).
    (The arm is 575 mm to the claw tip now; the controller keep-out and keepout_map.py use the full 575 + 15 mm capsule, this quick estimate keeps 500.)"""
    C = np.array([-176.0, -150.0, 43.0]); R = 95.0; worst = (1e9, None)
    for t1 in np.arange(-15, 140.1, 1):
        for t2 in np.arange(-55, 88.1, 1):
            d = bs.fk((t1, t2, 0)) @ bs.Z
            s = np.clip(C @ d, 0, 500.0); dist = np.linalg.norm(C - s * d) - R
            if dist < worst[0]: worst = (dist, (t1, t2, s))
    return C, R, worst

if __name__ == "__main__":
    out = {"effective_mass_kg": {k: round(v, 3) for k, v in CASES.items()}, "speed_limits_m_s": {}}
    for case, mR in CASES.items():
        rows = {}
        for reg, (F, k, mH, tr) in TS15066.items():
            v_qs = vmax(F, k, mH, mR); v_tr = vmax(2 * F, k, mH, mR) if tr else None
            rows[reg] = dict(v_quasi_static=round(v_qs, 3), v_transient=(round(v_tr, 3) if v_tr else "no transient contact allowed -> keep-out"))
        out["speed_limits_m_s"][case] = rows
    # Clamping / pinch forces at actuator limits
    pinch = {}
    pinch["ball_slot_edge_S2_peak"] = dict(r_m=0.09, tau_Nm=24.8, F_N=round(24.8 / 0.09), limit_hand_N=140, verdict="FAIL -> close the slot with a TPU lip/brush (no finger entry) and limit S2 current near the slot edges")
    pinch["elbow_fold_XM540_stall"] = dict(r_m=0.03, tau_Nm=10.6, F_N=round(10.6 / 0.03), limit_hand_N=140, verdict="FAIL -> XM540 current limit for <= 4.2 N m (140 N at 30 mm) AND fold gap >= 25 mm at E=110 (ISO 13854)")
    pinch["elbow_current_limit_Nm"] = dict(tau_needed_static=1.87, tau_with_SF2=3.75, tau_limit_for_140N_at_30mm=4.2, ok=3.75 <= 4.2)
    pinch["S2_clamp_upper_arm_at_0p23m"] = dict(F_N=round(24.8 / 0.23), limit_N=150, ok=24.8 / 0.23 <= 150)
    pinch["S2_clamp_hand_at_0p5m"] = dict(F_N=round(24.8 / 0.5), limit_N=140, ok=24.8 / 0.5 <= 140)
    out["pinch_and_clamp"] = pinch
    # Skin interface pressure targets (engineering targets derived from the sources above)
    PPT_MIN = 88.0
    targets = {"sustained_nerve_sensitive_kPa (axilla, brachial plexus, over spinous processes)": 4.3,
               "sustained_padded_stable_zone_kPa (paraspinal rail, clasp top/front facets, C7/T1)": round(PPT_MIN / 6, 1),
               "transient_peak_any_zone_kPa": round(PPT_MIN / 2, 1)}
    cradle_area_m2 = 0.048 * 0.058      # rev 2026-10-13 cradle 48 x 58 mm (clasp_shoulder.py)
    out["skin_pressure"] = dict(targets=targets, axilla_cradle=dict(previous_estimate_kPa=24, max_load_at_4p3kPa_N=round(4.3e3 * cradle_area_m2, 1),
                                verdict="FAIL: the cradle must be a locating anchor, not a load path; carry load on the paraspinal rail + clasp top/front"))
    C, R, (dmin, at) = head_keepout()
    out["head_keepout"] = dict(head_centre_est_mm=C.tolist(), radius_mm=R, min_clearance_straight_arm_mm=round(float(dmin), 1),
                               at_S1_S2_and_distance_along_arm=[round(float(x), 1) for x in at],
                               rule="controller keep-out: no link point may enter head sphere + 60 mm; speed scales to 0 from +160 mm")
    out["electrical"] = {
        "EMG applied part": "treat as IEC 60601-1 type BF applied part: patient leakage <= 100 uA normal / 500 uA single fault (AC); DC 10 / 50 uA",
        "isolation barrier": "2 x MOPP between electrodes and everything tied to the 24 V bus or charger: 4000 Vac test, 8 mm creepage, 5 mm clearance",
        "parts": "isolated DC-DC rated 2 x MOPP (medical, e.g. RECOM REM series; verify rating) + ISO7762 (5 kVrms reinforced) per AFE",
        "electrode protection": "series resistor >= 10 kOhm per electrode lead (limits fault current), ADS1299 lead-off detection on every channel",
        "charging": "charger port interlock: system cannot be armed while the charger is connected; never wear while charging",
        "battery": "6S Li-ion with BMS (cell OV/UV, overcurrent, NTC), pack in a fire-resistant enclosure away from the spine, 20 A slow-blow main fuse + BMS 20 A cut-off (sized in power_budget.py)"}
    # ---- rev 2026-10-13: hazard status after mitigation, read from the verification outputs (findings above = unmitigated hazards)
    def _j(rel):
        try: return json.load(open(os.path.join(ROOT, "stl", rel)))
        except Exception: return None
    vd, cc, vc = _j("distal/verify_distal.json"), _j("shoulder/clasp_cradle.json"), _j("distal/claw/verify_claw.json")
    st = {}
    st["shoulder_slot_edge"] = ("MITIGATED: the socket cowl's arm slot is cut 3.5 mm off the swept arm (>= 3.0 measured, < 4 mm: no fingertip entry, "
                                "ISO 13857); cowl bore / humeral-head dome gap 3.2..3.8 mm; cowl-to-pauldron rim gap 3.4 mm (shoulder_cowl.py, verify_shoulder.py). "
                                "OPEN for T4: a finger resting in the open part of the slot ahead of the S3 lid can be closed against the S2 clamp ear "
                                "(3.2 mm at S2 82): S2 torque near the soft limit must stay current-limited (lid at r 75 -> 140 N needs <= 10.5 N m)")
    if vd: st["elbow_fold"] = ("MITIGATED (ball elbow, rev 13b): XM540 current limit %.2f N m -> %.0f N at the nearest exposed radius %.0f mm (limit 140 N); verify_distal.py"
                              % (vd["elbow_pinch"]["current_limit_Nm"], vd["elbow_pinch"]["force_at_exposed_N"], 1e3 * vd["elbow_pinch"]["exposed_radius_m"]))
    # rev 2026-10-15: C3 punch-ring claw text; numbers from verify_distal.json['gripper'] (written from verify_claw.json grip['0.75']). Missing key = pipeline-order error.
    if vc and vd:
        if "gripper" not in vd: sys.exit("safety_limits.py: verify_distal.json has no 'gripper' -> run verify_claw.py then verify_distal.py first")
        gp = vd["gripper"]; g75 = vc["grip"]["0.75"]
        st["claw_hand"] = ("MITIGATED (C3 punch-ring claw, rev 14m/15): STS3215 torque limit 0.75 N m (register 255) -> fingertip normal force %.1f..%.1f N per finger, "
                           "3 fingers <= %.0f N total (limit 140 N hand/finger clamping, ISO/TS 15066). Fingertip speed <= %.3f m/s relative to the hand (no-load servo). "
                           "Rigid closed-side hard stop (stop caps) at theta1 soft + 5 deg; pad-to-ring pinch is limited by the same 0.75 N m. Re-assess finger entry between pad and ring in T4."
                           % (gp["F_inward_min_N"], g75["Fn_max_N"], 3 * g75["Fn_max_N"], vc["speed"]["tip_speed_max_m_s"]))
        st["claw_hold"] = ("%s (function): 0.5 kg payload held by friction on 3 TPU pads (mu 0.8) at 0.75 N m -> grip SF %.2f (gate 2). The screw is not self-locking: "
                           "the servo must hold torque; unpowered, the claw opens." % ("PASS" if gp["grip_SF"] >= 2 else "OPEN", gp["grip_SF"]))
    if cc: st["axilla_cradle"] = ("MITIGATED: locating anchor only, %.0f x %.0f mm face %.0f mm below the axillary apex, strap tension <= %.1f N at 4.3 kPa"
                                  % (cc["CL"], cc["CH"], cc["apex_z"] - (cc["A"][2] + cc["CH"] / 2), 4.3e3 * cradle_area_m2))
    st["head"] = "MITIGATED (passive): soft box S1 10..140 / S2 -30..88 keeps the straight arm (capsule r 32 to tip + 15 mm) >= 64.9 mm from the head mesh (keepout_map.py); software keep-out + elbow/wrist tiers (60 mm to the head) on top. Re-verified with the 575 mm claw tip in sweep_full.py (unify 2026-10-13f)"
    if vd: st["S2_torque_margin"] = ("OPEN: worst S2 static %.2f N m vs 8.3 rated = SF %.2f (< 2). Firmware S2 duty monitor (60 s RMS <= 4.15 N m) until the "
                                     "gravity balancer (I1b) lands" % (vd["torque_Nm"]["S2"], 8.3 / vd["torque_Nm"]["S2"]))
    out["status_after_mitigation"] = st
    os.makedirs(os.path.join(ROOT, "stl", "control"), exist_ok=True)
    json.dump(out, open(os.path.join(ROOT, "stl", "control", "safety_limits.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))
