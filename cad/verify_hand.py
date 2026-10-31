"""Rev B verification: watertight/printability, mass budget, fingertip + chuck forces (virtual work), envelope, S2/harness estimates, pose report."""
import numpy as np, trimesh, json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hand_kin as K
OUT = K.OUT; G = 9.81; R = {}
PJ = json.load(open(OUT + "/hand_parts.json")); PL = json.load(open(OUT + "/poses.json")); K.RHO_T = PL['RHO_T']; CH = json.load(open(OUT + "/chuck_solve.json"))
# printability + mass
INF = dict(palm_back=0.55, palm_cover=0.6)
pr, tot = {}, 0.0
for n, v in PJ['parts'].items():
    m = trimesh.load(f"{OUT}/{n}.stl"); g = v['g_solid']*INF.get(n, 1.0); tot += g
    pr[n] = dict(watertight=bool(m.is_watertight), fits_256=bool(max(m.extents) <= 256), g=round(g, 2))
R['printability'] = dict(n_parts=len(pr), all_watertight=all(p['watertight'] for p in pr.values()), all_fit_256=all(p['fits_256'] for p in pr.values()))
SERVO_G, N_SERVO = 13.4, 8
hw = dict(servos=SERVO_G*N_SERVO, pins_d2_steel=6.0, rods_and_ball_links=6.0, cover_screws_inserts=3.0, driver_board_and_6V_buck_PLACEHOLDER=10.0, wiring=4.0)
R['mass'] = dict(printed_g=round(tot, 1), hardware_g=hw, hand_g=round(tot + sum(hw.values()), 1), claw_g=178.9, delta_g=round(tot + sum(hw.values()) - 178.9, 1), per_part_g={n: p['g'] for n, p in pr.items()})
# forces: MCP torque from the servo through crank r 8 mm and the flexor lever (ball radius + 2.2 mm); 70 % of stall for holding
TAU_S = 0.216
def tip_force(nm, th, ps, ncontact):
    c0, c1 = K.chain(nm, th, ps), K.chain(nm, th + 0.25, ps); dp = (c1['tipc'] - c0['tipc'])/np.radians(0.25)
    lev = 0.9*min(K.WID[nm][0], K.THK[nm][0]) + 2.2; tau = 0.7*TAU_S*(lev/8.0)
    return tau/(abs(dp @ ncontact)*1e-3 + 1e-9)
def chuck_forces(key, rb, digits):
    v = CH[key]; p0 = np.array(v[8:11]); a = np.array([1.0, 0, 0]); out = {}
    for nm in digits:
        if nm == 'thumb':
            t0, t1 = K.thumb(*v[4:7]), K.thumb(v[4], v[5], v[6] + 0.25); dp = (t1['tipc'] - t0['tipc'])/np.radians(0.25); c = t0['tipc']
        else:
            th, ps = (v[0], v[3]) if nm == 'index' else ((v[1], 0.0) if nm == 'middle' else (v[2], v[3])); c = K.chain(nm, th, ps)['tipc']
        w = c - p0; r = w - (w @ a)*a; ncon = -r/np.linalg.norm(r)
        if nm == 'thumb': out[nm] = round(float(0.7*TAU_S*(8.0/8.0)/(abs(dp @ ncon)*1e-3)), 2)
        else: out[nm] = round(float(tip_force(nm, th, ps, ncon)), 2)
    mu = 0.5; Fmin = min(out.values()); Fsum = Fmin*len(out)                 # conservative: every jaw limited to the weakest jaw (balanced squeeze)
    return dict(single_jaw_capacity_N=out, balanced_jaw_force_N=round(Fmin, 2), hold_torque_Nm=round(mu*Fsum*rb*1e-3, 4), axial_pull_N=round(mu*Fsum, 2), mu_assumed=mu,
                note='virtual-work capacity per jaw at 70 % servo stall; balanced squeeze limited by the weakest jaw')
R['chuck4_d13'] = chuck_forces('chuck_d13', 6.5, ('index', 'middle', 'ring', 'thumb'))
R['chuck3_d6'] = chuck_forces('chuck3_d6', 3.0, ('index', 'middle', 'thumb'))
# envelope (open pose) and contract
pm = trimesh.load(f"{OUT}/poses/hand_open_posed.stl")
R['open_envelope'] = dict(z_max=round(float(pm.bounds[1][2]), 1), wrist_to_tip_from_W2_mm=round(float(pm.bounds[1][2]) - 410.0, 1), contract_mm=165.0, x_range=np.round(pm.bounds[:, 0], 1).tolist(), y_range=np.round(pm.bounds[:, 1], 1).tolist())
# S2 / harness estimates (scaled from DESIGN_STATE rev f3, balancer K 3.789)
dm = (R['mass']['hand_g'] - 178.9)/1e3; tip_ext = (R['open_envelope']['z_max'] - 575.0)*1e-3
S2 = 2.89 + G*(dm*0.53 + 0.5*tip_ext*0.6); R['S2'] = dict(worst_residual_Nm=round(S2, 2), SF=round(8.3/S2, 2), gate='PASS' if 8.3/S2 >= 2 else 'FAIL')
press = 14.5*(5.83 + dm)/5.83; R['harness_top_pad'] = dict(est_kPa=round(press, 2), limit=14.7, gate='PASS' if press <= 14.7 else 'FAIL')
R['poses'] = json.load(open(OUT + "/pose_report.json"))
json.dump(R, open(OUT + "/verify_hand.json", "w"), indent=1)
print(json.dumps({k: v for k, v in R.items() if k not in ('poses',)}, indent=1, default=str)[:3000])
