"""Harness statics and skin-pressure gate (per side, sagittal plane x-z, robot frame, SI inside, mm in prints).
The rig (arm + stage + plate + wing + clasp) is one rigid body resting on three contact groups:
  TOP   : clasp facets F1 (upper trapezius) + F4 (acromion), vertical support (body pushes +z)
  FRONT : clasp facets F2 (anterior deltoid) + F3 (pectoral), body pushes the rig forward (-x)
  RAIL  : wing paraspinal rail foam, body pushes the rig backward (+x)
Unknowns N_top, N_front, N_rail from sum Fx = 0, sum Fz = 0, sum My = 0 (statically determinate). The axilla cradle and front
strap are NOT counted as load paths (they only locate; SPEC_safety axilla target 4.3 kPa -> <= 12 N).
Pressure = N / pad area; dynamic factor 1.3 (arm accelerations at the 0.5 m/s tip cap, hand estimate).
Targets (SPEC_safety): padded stable zones <= 14.7 kPa sustained, <= 44 kPa transient.
Masses: verified AK70-10 621 g, XM540 165 g; STS3215 55 g (vendor, not re-verified); printed parts from mesh volumes.
rev 2026-10-13f (unify): arm beyond S2 now = ball-elbow upper arm (meshes) + distal chain from verify_distal.json (claw hand);
forearm CG from the mesh moments (forearm_hand_mass_g.cg_z_mm) and the payload sits at the claw tip z (verify_distal torque_Nm.tip_z_mm)."""
import numpy as np, json, os, sys, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import biomimetic_shoulder as bs
S = os.path.join(HERE, "..", "stl", "shoulder")
sites = json.load(open(os.path.join(S, "clasp_sites.json")))
g = 9.81
def vol(p, rho):
    try: return trimesh.load(p).volume * rho * 1e-3        # g
    except Exception: return 0.0
fixed_mass_g = {"AK70 S1": 621, "s1 base+clamp (PETG)": vol(os.path.join(S, "s1_base_R.stl"), 1.27) + vol(os.path.join(S, "s1_clamp_R.stl"), 1.27),
                "chassis plate (6061)": vol(os.path.join(S, "chassis_plate_R.stl"), 2.70), "wing (CF-PA)": vol(os.path.join(S, "scapular", "wing_R.stl"), 1.15),
                "spine bridge half (CF-PA)": 0.5 * vol(os.path.join(S, "scapular", "spine_bridge.stl"), 1.15), "clasp (PETG)": vol(os.path.join(S, "clasp_frame_R.stl"), 1.27),
                "stops + fasteners (est.)": 60.0, "electronics share (est.)": 0.0,
                "pauldron (PA-CF)": vol(os.path.join(S, "pauldron", "pauldron_R.stl"), 1.15)}      # rev 2026-10-13
# rev 2026-10-12c: real pods (sensor_pods.py): AFE pod per side, half of the shared main box, battery and cradle on the spine
_pj = os.path.join(HERE, "..", "stl", "pods", "pods.json")
if os.path.exists(_pj):
    _p = json.load(open(_pj)); _m, _c = _p["printed_mass_g"], _p["contents_mass_g"]
    PODS = {"afe": (_m["afe_pod_R"] + _m["afe_pod_lid_R"] + _c["afe_pod_R"], np.array(_p["place"]["afe_pod_R"]["origin"])),
            "main_half": (0.5 * (_m["main_box"] + _m["main_box_lid"] + _c["main_box"]), np.array(_p["place"]["main_box"]["origin"])),
            "battery_half": (0.5 * (_m["battery_box"] + _m["battery_box_lid"] + _m["battery_cradle"] + _c["battery_box"]), np.array(_p["place"]["battery_box"]["origin"]))}
else:
    PODS = {}
_vd = os.path.join(HERE, "..", "stl", "distal", "verify_distal.json")
m1_mass_g = {"AK70 S2": 621, "yoke + S2 clamp (PETG)": vol(os.path.join(S, "s1s2_yoke_R.stl"), 1.27) + vol(os.path.join(S, "s2_clamp_R.stl"), 1.27),
             "socket cowl + head dome (PETG)": vol(os.path.join(S, "cowl", "cowl_R.stl"), 1.27) + vol(os.path.join(S, "cowl", "head_dome_R.stl"), 1.27)}   # rev 2026-10-13 (was ball + lip)
# rev 2026-10-16 (gI merge): the S2 balancer (I1b rev e) is installed. Its masses come from verify_balancer.json (mass_cg: printed parts at solid
# density, spring, bearings, hardware): the yoke side (anchor + tube + cap + spring + piston + swivel + bumpers, CG ~ (0, 90, 7)) rides on S1 and is
# lumped with M1; the arm side (collar + cap + pin set, CG z ~ 78 in the arm frame) is added to the upper arm below. Was missing before (329 g/arm).
_vb = os.path.join(HERE, "..", "stl", "balancer", "verify_balancer.json")
BAL_ARM = None
if os.path.exists(_vb):
    _bm = json.load(open(_vb))["mass"]
    m1_mass_g["S2 balancer, yoke side"] = float(_bm["yoke_side_g"]); BAL_ARM = (float(_bm["arm_side_g"]), float(_bm["arm_side_cg"][2]))
RAIL_Z, RAIL_W = (-80.0, -262.0), 30.0          # overridden by scapular_frame.json if present
ARM_MASS_G, ARM_CG_MM, PAYLOAD_G, PAYLOAD_R_MM = 1130.0, 230.0, 500.0, 500.0     # beyond S2 (fallback), CG along the arm
if os.path.exists(_vd):     # unify 2026-10-13f: upper arm from meshes (ball elbow) + distal total/CG from verify_distal (claw hand)
    # rev 2026-10-15 (S2 reconciliation): the elbow fork parts (fork_drive / fork_idler / fork_bridge, ~78 g) belong to the forearm group ME and are
    # already in verify_distal's forearm_hand_mass_g -> removed here (they were counted twice).  Upper-arm CG from the mesh centroids
    # (was a flat 150 mm); wiring/fastener allowance 80 g at z 190 (rev f3: 67.6 g in-arm wiring from arm_electrical.json + ~12 g fasteners; was 40 g at z 150).  Head dome stays in m1_mass_g (lumped at P).
    _U = os.path.join(HERE, "..", "stl", "upperarm")
    def _vol_cgz(p, rho):
        try:
            m = trimesh.load(p); c = m.center_mass if m.is_watertight else m.bounds.mean(0); return m.volume * rho * 1e-3, float(c[2])
        except Exception: return 0.0, 150.0
    def _pz(p):
        try: return float(trimesh.load(p).bounds.mean(0)[2])
        except Exception: return 150.0
    UPPER_PARTS = [_vol_cgz(os.path.join(_U, f + "_R.stl"), 1.30) for f in ("ua_lat", "ua_med")] + \
                  [_vol_cgz(os.path.join(S, f + "_R.stl"), 1.27) for f in ("s3_bracket", "s3_lid", "s2_adapter")] + \
                  [(165.0, _pz(os.path.join(S, "proxies", "xm_s3_body.stl"))), (165.0, _pz(os.path.join(_U, "proxies", "xm_elbow.stl"))), (80.0, 190.0)]
    if BAL_ARM is not None: UPPER_PARTS.append(BAL_ARM)                       # rev 2026-10-16 (gI): balancer collar + cap + pin set
    _upper = sum(m for m, _ in UPPER_PARTS); UPPER_CG_MM = sum(m * z for m, z in UPPER_PARTS) / _upper
    _vj = json.load(open(_vd)); _fh = _vj["forearm_hand_mass_g"]; _fore = _fh["total"]; _fore_cg = _fh.get("cg_z_mm", 230.0 + 125.0)
    ARM_MASS_G = _upper + _fore; ARM_CG_MM = (_upper * UPPER_CG_MM + _fore * _fore_cg) / ARM_MASS_G
    PAYLOAD_R_MM = float(_vj["torque_Nm"]["tip_z_mm"])
X_FIXED_CG = -55.0     # mm: fixed hardware CG (stage under P at x ~ -40, plate -84, wing -110, clasp ~ -170) -> computed below
def cg_fixed():
    parts = {"stage": (621 + fixed_mass_g["s1 base+clamp (PETG)"], np.array([-30.0, 0, -80])), "plate": (fixed_mass_g["chassis plate (6061)"], np.array([-84.0, 0, -100])),
             "wing": (fixed_mass_g["wing (CF-PA)"], np.array([-105.0, -110, -150])), "clasp": (fixed_mass_g["clasp (PETG)"], np.array([-185.0, -20, -150])),
             "misc": (fixed_mass_g["stops + fasteners (est.)"] + fixed_mass_g["electronics share (est.)"] + fixed_mass_g["spine bridge half (CF-PA)"], np.array([-100.0, -120, -170])),
             "pauldron": (fixed_mass_g["pauldron (PA-CF)"], np.array([-40.0, 0, -95]))}
    parts.update(PODS)
    M = sum(m for m, _ in parts.values()); return M, sum(m * p for m, p in parts.values()) / M
def contact(name):
    s = sites[name]; p = np.array(s["p"]); c = s.get("contour")
    if c and "u" in s:       # rev 2026-10-14: reshaped plates (F2, F4) are not centred on the atlas/electrode point
        n = np.array(s["n"]); u = np.array(s["u"]); v = np.cross(n, u); cu, cv = c["centre_uv"]; p = p + cu * u + cv * v
    return p
def area(name, pad=None, fillet=None):
    if pad is None and fillet is None and sites[name].get("area_mm2"): return float(sites[name]["area_mm2"])     # rev 2026-10-14: built net bearing area of the contoured plate (F2 trimmed at F4, F4 minus the AC-joint window)
    w, h = (pad or sites[name]["plate"]); f = fillet if fillet is not None else sites[name].get("fillet", 12.0)
    r = min(f, min(w, h) / 2 - 1.0); return w * h - (4 - np.pi) * r * r
def solve(pose, pads, dyn=1.0):
    Mf, cf = cg_fixed(); Mm1 = sum(m1_mass_g.values()); cm1 = np.array([0.0, -20, -20])
    d = bs.fk((pose[0], pose[1], 0)) @ bs.Z * 1.0
    arm = d * ARM_CG_MM; pay = d * PAYLOAD_R_MM
    masses = [(Mf, cf), (Mm1, cm1), (ARM_MASS_G, arm), (PAYLOAD_G, pay)]
    W = sum(m for m, _ in masses) * 1e-3 * g * dyn; xc = sum(m * p[0] for m, p in masses) / sum(m for m, _ in masses); zc = sum(m * p[2] for m, p in masses) / sum(m for m, _ in masses)
    # contact points (x, z): area-weighted centroids of each group
    top = [("F1_upper_trap", pads.get("F1_upper_trap")), ("F4_acromion", pads.get("F4_acromion"))]; front = [("F2_ant_delt", pads.get("F2_ant_delt")), ("F3_pec", pads.get("F3_pec"))]
    def cen(gr): A = np.array([area(n, p) for n, p in gr]); P = np.array([contact(n) for n, _ in gr]); return (A[:, None] * P).sum(0) / A.sum(), A.sum()
    (pt, At), (pf, Af) = cen(top), cen(front)
    # rail: y 8..38 from the spine, z RAIL_Z; the rig rotates about the shoulder-top contact, so the rail pressure is triangular
    # (zero at the top end, peak at the bottom end): resultant at 2/3 of the length, peak pressure = 2 N / (w L)
    z0, z1 = RAIL_Z; Lr = z0 - z1; pr = np.array([-112.0, -135.0, z0 - 2.0 / 3.0 * Lr]); Ar = RAIL_W * Lr / 2.0     # Ar: equivalent area for the PEAK pressure
    # unknowns [Nt (+z at pt), Nf (-x at pf), Nr (+x at pr)];  Fz: Nt = W ; Fx: Nr - Nf = 0 ; My about the origin (y axis, right-hand: My = z Fx - x Fz)
    # moment of a force F=(Fx,0,Fz) at (x,z) about y: m = z*Fx - x*Fz
    A = np.array([[1, 0, 0], [0, -1, 1], [-(pt[0]) * 1.0, (pf[2]) * -1.0, pr[2] * 1.0]], float)
    b = np.array([W, 0.0, -(xc * W)])                                    # weight (0,0,-W) at (xc,zc): m = xc*W ; equilibrium: sum m = 0
    Nt, Nf, Nr = np.linalg.solve(A, b)
    return dict(W_N=round(W, 1), cg_x_mm=round(xc, 1), N_top=round(Nt, 1), N_front=round(Nf, 1), N_rail=round(Nr, 1),
                p_top_kPa=round(Nt / At * 1e3, 1), p_front_kPa=round(Nf / Af * 1e3, 1), p_rail_kPa=round(Nr / Ar * 1e3, 1),
                area_mm2=dict(top=round(At), front=round(Af), rail=round(Ar)))
if __name__ == "__main__":
    import rom_limits as rl
    pads = json.load(open(os.path.join(S, "clasp_pads.json"))) if os.path.exists(os.path.join(S, "clasp_pads.json")) else {}
    sf = os.path.join(S, "scapular", "scapular_frame.json")
    if os.path.exists(sf):
        zr = json.load(open(sf))["zones"]["RAIL"]; z = zr["z"]; RAIL_Z = (float(z[1]), float(z[0])); RAIL_W = float(zr["y"][1] - zr["y"][0])
    if len(sys.argv) > 1: RAIL_Z = (-80.0, float(sys.argv[1]))
    if len(sys.argv) > 2:
        w, h = map(float, sys.argv[2].split("x")); pads = {k: (w, h) for k in ("F1_upper_trap", "F2_ant_delt", "F3_pec")}; pads["F4_acromion"] = (h, h)
    poses = {"park (S1 45, S2 25)": (45, 25), "arm up (S2 0)": (45, 0), "forward horizontal, 0.5 kg (S1 15, S2 88)": (15, 88),
             "lateral horizontal, 0.5 kg (S1 90, S2 88)": (90, 88), "back-lateral (S1 140, S2 60)": (140, 60), "back lean (S1 45, S2 -30)": (45, -30)}
    out = {}
    for k, q in poses.items():
        out[k] = solve(q, pads, dyn=1.3)
    worst = {g_: max(v["p_" + g_ + "_kPa"] for v in out.values()) for g_ in ("top", "front", "rail")}
    neg = {k: v for k, v in out.items() if min(v["N_top"], v["N_front"], v["N_rail"]) < 0}
    res = dict(fixed_mass_g={k: round(v) for k, v in fixed_mass_g.items()}, m1_mass_g={k: round(v) for k, v in m1_mass_g.items()},
               pods_g={k: round(v[0]) for k, v in PODS.items()}, arm_beyond_S2_g=round(ARM_MASS_G), arm_cg_mm=round(ARM_CG_MM), payload_radius_mm=PAYLOAD_R_MM,
               total_per_side_kg=round((sum(fixed_mass_g.values()) + sum(m1_mass_g.values()) + sum(v[0] for v in PODS.values()) + ARM_MASS_G + PAYLOAD_G) / 1e3, 2),
               poses=out, worst_kPa=worst, target_kPa=14.7, passes=all(v <= 14.7 for v in worst.values()) and not neg,
               poses_needing_tension_or_strap=list(neg))
    json.dump(res, open(os.path.join(S, "harness_loads.json"), "w"), indent=1)
    print(json.dumps(res, indent=1))
