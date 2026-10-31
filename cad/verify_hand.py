"""Hand verification: servo crank/rod curves, fingertip + chuck forces, thumb force balance, mass budget, S2/harness impact, printability, watertight."""
import sys, json, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, trimesh
import hand_kin as K, hand as Hd, hand_pose as HP
OUT = Hd.OUT; G = 9.81
R = {}
# ---------------------------------------------------------------- servo crank + rod per finger
A_CR = 8.0                                            # crank radius (mm)
TAU_S = 0.216                                         # MG90S class stall torque at 4.8 V (N.m) PLACEHOLDER vendor value; 0.25 at 6 V
def tail(nm, th):
    c = K.chain(nm, th, 0.0); L = K.LEN[nm]; d = np.array([np.sin(np.radians(th)), 0, np.cos(np.radians(th))])
    return c['M'] - 8.0*d
def pin(nm, phi):
    u_s = Hd.UF[nm]; return np.array([K.XMD[nm], u_s + A_CR*np.sin(np.radians(phi)), Hd.ZS + A_CR*np.cos(np.radians(phi))])
def curve(nm, lr):
    ths = np.linspace(0, 100, 41); phis = []; phi = 150.0
    for th in ths:
        T = tail(nm, th)
        f = lambda p: np.linalg.norm(pin(nm, p) - T) - lr
        grid = np.linspace(-170, 170, 681); v = np.array([f(p) for p in grid]); idx = [i for i in range(len(grid)-1) if v[i]*v[i+1] < 0]
        if not idx: return None
        cand = [grid[i] - v[i]*(grid[i+1]-grid[i])/(v[i+1]-v[i]) for i in idx]; phi = min(cand, key=lambda p: abs(p - phi)); phis.append(phi)
    return ths, np.array(phis)
res = {}
for nm in K.LEN:
    best = None
    for lr in np.arange(12, 70, 0.5):
        out = curve(nm, lr)
        if out is None: continue
        ths, ph = out; rng = ph.max() - ph.min(); mono = np.all(np.diff(ph) > 0) or np.all(np.diff(ph) < 0)
        if mono and 20 <= rng <= 165 and (best is None or rng < best[1]): best = (lr, rng, ths, ph)
    if best is None: res[nm] = None; continue
    lr, rng, ths, ph = best; dphi = np.gradient(np.radians(ph), np.radians(ths))
    res[nm] = dict(rod_len_mm=round(float(lr), 1), servo_range_deg=round(float(rng), 1), phi_at_0=round(float(ph[0]), 1), phi_at_95=round(float(np.interp(95, ths, ph)), 1), min_abs_dphi_dth=round(float(np.abs(dphi).min()), 2), max_abs_dphi_dth=round(float(np.abs(dphi).max()), 2))
R['servo_curves'] = res
# ---------------------------------------------------------------- fingertip force from servo torque (virtual work) at the pad, chuck pose
P = HP.make_poses()['chuck']; MU = 0.5; RB = 5.0
Fn = {}
for nm in ('index', 'middle', 'ring'):
    th, ps = P['f'][nm]; sc = res[nm]
    if sc is None: continue
    h = 1e-3
    c0, c1 = K.chain(nm, th, ps), K.chain(nm, th + 0.5, ps); dp = (c1['pad'] - c0['pad']) / np.radians(0.5)      # mm per rad of MCP
    lever = np.dot(dp, c0['n'])                                                                                 # mm of pad motion along the pad normal per rad MCP
    dphi = (sc['min_abs_dphi_dth'] + sc['max_abs_dphi_dth']) / 2                                                 # servo rad per MCP rad (mean; pose-dependent)
    # servo torque at 70 % of stall (hold); finger torque at MCP = tau_s / dphi_dth ; F = tau_mcp / (lever mm * 1e-3)
    tau_mcp = 0.7 * TAU_S / dphi
    Fn[nm] = dict(F_N=round(float(tau_mcp / (abs(lever)*1e-3)), 2), lever_mm_per_rad=round(float(lever), 1), n_h=np.round(c0['n'][:2], 2).tolist())
R['finger_pad_force_chuck'] = Fn
if Fn:
    Fsum = sum(Fn[n]['F_N'] * np.array(Fn[n]['n_h']) for n in Fn)
    TH_CAP = 0.7*TAU_S/ (np.linalg.norm((lambda tf_: tf_[2][0] + 0.62*K.TLEN[2]*tf_[2][1] - tf_[1][0])(K.thumb_frames(*P['th'][:2], P['th'][2])))*1e-3)
    sc_ = min(1.0, TH_CAP/abs(Fsum[0])); Ftot = sum(Fn[n]['F_N'] for n in Fn)*sc_ + min(abs(Fsum[0]), TH_CAP)
    R['chuck_balance'] = dict(finger_sum_xu_N=np.round(Fsum, 2).tolist(), thumb_needed_at_full_finger_torque_N=round(float(abs(Fsum[0])), 2), thumb_capacity_N=round(float(TH_CAP), 2), finger_torque_scale=round(float(sc_), 3),
                              pad_forces_scaled_N={n: round(Fn[n]['F_N']*sc_, 2) for n in Fn}, hold_torque_Nm=round(float(MU * Ftot * RB * 1e-3), 4), mu_assumed=MU, rod_radius_mm=RB,
                              note='drill-chuck grip torque at W1; mu TPU-on-steel assumed (unmeasured)')
# thumb force: flex servo through the MCP/IP coupled bar, lever ~ pad distance from MCP axis
tf = K.thumb_frames(*P['th'][:2], P['th'][2]); mcp = tf[1][0]; padc = tf[2][0] + 0.62*K.TLEN[2]*tf[2][1]
arm = np.linalg.norm(padc - mcp)*1e-3
# thumb flex servo: horn arm 10 mm, tendon-free link ratio 1:1 to the MCP lever 12 mm -> MCP torque ~ tau_s*(12/10)? use 1:1 conservative
R['thumb_force_N'] = round(float(0.7*TAU_S / arm), 2); R['thumb_arm_mm'] = round(arm*1e3, 1)
# ---------------------------------------------------------------- mass budget
dens = lambda n: 1.30 if n in ("palm_tray", "thumb_cmc") or n.endswith("knuckle") else (1.21 if n.endswith("pad") else 1.20)
INF = dict(palm_tray=0.55)                        # 40 % infill + 4 walls ~ 55 % of solid for the big tray; small parts print solid
mass = {}; tot_print = 0.0
for n in Hd.parts:
    m = trimesh.load(os.path.join(OUT, n + ".stl")); g = m.volume/1e3 * dens(n) * INF.get(n, 1.0); mass[n] = round(g, 1); tot_print += g
N_SERVO = 8; SERVO_G = 13.4                       # 4 finger + 3 thumb + 1 yaw, MG90S class (PLACEHOLDER)
hw = dict(servos=N_SERVO*SERVO_G, pushrods_balls_pins=12.0, yaw_linkage=3.0, screws_inserts=5.0, pwm_driver_and_6V_buck_PLACEHOLDER=10.0, wiring=4.0)
hand_g = tot_print + sum(hw.values())
R['mass'] = dict(printed_g=round(tot_print, 1), hardware_g=hw, hand_total_g=round(hand_g, 1), claw_total_g=178.9, delta_g=round(hand_g - 178.9, 1), per_part_g=mass)
# ---------------------------------------------------------------- S2 + harness impact (scaling from DESIGN_STATE rev f3 numbers)
dm = (hand_g - 178.9)/1e3; lever_cg = 0.535; tip_extra = (628.0 - 575.0)*1e-3
dS2 = G*(dm*lever_cg + 0.5*tip_extra*0.6)           # hand CG ~535 mm out; payload held ~0.6 of the tip extension further out
S2_new = 2.89 + dS2
R['S2'] = dict(delta_Nm=round(dS2, 3), worst_residual_Nm=round(S2_new, 2), SF_vs_8p3=round(8.3/S2_new, 2), gate_SF2='PASS' if 8.3/S2_new >= 2 else 'FAIL', note='scaled from rev f3 (balancer K 3.789): re-run s2_grid.py for a locked number')
press = 14.5*(5.83 + dm)/5.83
R['harness_top_pad'] = dict(est_kPa=round(press, 2), limit_kPa=14.7, margin_pct=round((14.7-press)/14.7*100, 1), gate='PASS' if press <= 14.7 else 'FAIL', note='mass-proportional estimate; run harness_loads.py + verify_harness.py')
# ---------------------------------------------------------------- watertight / printability
chk = {}
for n in Hd.parts:
    m = trimesh.load(os.path.join(OUT, n + ".stl")); chk[n] = dict(watertight=bool(m.is_watertight), vol_pos=bool(m.volume > 0), bbox=np.round(m.extents, 1).tolist(), fits_256=bool(max(m.extents) <= 256))
R['printability'] = dict(all_watertight=all(v['watertight'] for v in chk.values()), all_fit_256=all(v['fits_256'] for v in chk.values()), failing=[k for k, v in chk.items() if not (v['watertight'] and v['vol_pos'] and v['fits_256'])])
pr = json.load(open(OUT + "/pose_report.json")); R['poses'] = pr
# envelope
pm = trimesh.load(os.path.join(OUT, "poses", "hand_open_posed.stl")); R['open_envelope'] = dict(z_max=round(float(pm.bounds[1][2]), 1), contract_Z_TIP=575.0, wrist_to_tip_from_W2_mm=round(float(pm.bounds[1][2]) - 410.0, 1), y_range=np.round(pm.bounds[:, 1], 1).tolist(), x_range=np.round(pm.bounds[:, 0], 1).tolist())
json.dump(R, open(OUT + "/verify_hand.json", "w"), indent=1, default=lambda o: o.tolist() if hasattr(o, 'tolist') else str(o))
print(json.dumps({k: R[k] for k in R if k not in ('mass', 'poses')}, indent=1, default=str)); print('mass', {k: v for k, v in R['mass'].items() if k != 'per_part_g'})
