"""MJ11 liner shear / friction / heat estimate (2026-11-03).  Normal loads: mj_eval run_frame (stl/j1r2_mounts/MJ11/metrics.json) + assumed strap preload cases.
Model: the PTFE-fabric slip face slides on the OUTER face of the compression base layer; the base layer stays on the skin (mu_skin > mu_slip), so the skin shear is
tau = mu_slip * p, transmitted through the base layer.  If mu_slip rises above mu_skin (sweat, contamination) the slip moves to the skin: blister/abrasion regime.
usage: python3 mj11_shear.py -> stl/j1r2_mounts/MJ11/shear.json"""
import os, json, math
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))); MD = os.path.join(REPO, "stl", "j1r2_mounts", "MJ11")
met = json.load(open(os.path.join(MD, "metrics.json"))); led = json.load(open(os.path.join(MD, "leads.json")))
A = led["liner_area"]["remaining_mm2"] * 1e-6                                  # m2, liner footprint left after the relief slots
dp_max = max(met["frame"][lc]["contact"]["liner"]["max_kPa"] for lc in ("park", "reach")); dp_min = min(met["frame"][lc]["contact"]["liner"]["min_kPa"] for lc in ("park", "reach"))
DYN = 2.0                                                                       # dynamic factor on the increment (SPEC_j1r2_blade_seat bolts x2)
P0 = (2.0, 4.3, 8.0)                                                            # kPa preload cases: 2 = collar-only; 4.3 = within-17 mm band budget (SPEC_flex_electrode_leads s6); 8 = tight strap
MU_SLIP = (0.05, 0.10, 0.20, 0.30)                                              # PTFE-fabric on knit: no sourced value; 0.2 = NICE HTG355 low-friction fabric; PTFE lowest of skin materials (Veijgen 2013)
MU_SKIN = (0.3, 0.52, 0.7)                                                      # skin vs standard textiles 0.3-0.7 (NICE); median static 0.52 (Veijgen 2013)
SWEAT = 3.0                                                                     # overhydration up to x3 on skin-fabric friction (Jayawardana 2017)
TRAVEL = (56.0, 96.0)                                                           # mm, DESIGN_STATE s4 (user-accepted)
V_SKIN = (0.010, 0.037)                                                         # m/s: typical 10 mm/s; worst = 96 mm in 2.6 s (0.5 m/s tip cap at 0.5 m reach -> ~1 rad/s, 150 deg elevation)
K_LD33 = 66.0 / 0.25 / 8.0                                                      # kPa per mm of liner compression: Plastazote LD33 secant at 25% (66 kPa, Zotefoams LD33-December-2017) over 8 mm
K_MODEL = 150.0 / 8.0                                                           # kPa/mm: E_FOAM 0.15 MPa (mj_frame, foam + tissue lumped) over 8 mm
out = dict(inputs=dict(area_mm2=round(A * 1e6), dp_max_kPa=dp_max, dp_min_kPa=dp_min, dyn=DYN, p0_cases=P0, mu_slip=MU_SLIP, mu_skin=MU_SKIN, sweat_factor=SWEAT, travel_mm=TRAVEL, v_skin_m_s=V_SKIN,
           k_LD33_kPa_per_mm=round(K_LD33, 1), k_model_kPa_per_mm=K_MODEL), rows=[])
for p0 in P0:
    for mode, p in (("static", p0 + dp_max), ("dynamic peak", p0 + DYN * dp_max)):
        N = p * 1e3 * A
        for mu in MU_SLIP:
            F = mu * N; tau = mu * p; out["rows"].append(dict(p0=p0, case=mode, p_kPa=round(p, 2), N=round(N, 1), mu_slip=mu, F_shear_N=round(F, 2), tau_kPa=round(tau, 2),
                      slip_at_skin_dry=bool(mu >= MU_SKIN[0]), slip_at_skin_if_pair_x2_sweat=bool(2 * mu >= MU_SKIN[0]),
                      E_per_stroke_J=[round(F * s * 1e-3, 3) for s in TRAVEL], P_W_worst=round(F * V_SKIN[1], 3), q_W_m2_worst=round(tau * 1e3 * V_SKIN[1], 1)))
# conduction temperature rise, semi-infinite skin, flux q for a stroke of t = s/v:  dT = 2 q sqrt(t / (pi k rho c)) ; k rho c = (0.37 W/mK)(1100 kg/m3)(3400 J/kgK)  (textbook skin values, assumption)
krc = 0.37 * 1100.0 * 3400.0
def dT(q, t): return 2.0 * q * math.sqrt(t / (math.pi * krc))
w = [r for r in out["rows"] if r["p0"] == 4.3 and r["case"] == "dynamic peak" and r["mu_slip"] == 0.30][0]
out["heat_worst_case"] = dict(row=w, stroke_s=TRAVEL[1] * 1e-3 / V_SKIN[1], dT_K=round(dT(w["q_W_m2_worst"], TRAVEL[1] * 1e-3 / V_SKIN[1]), 3))
out["fit_sensitivity"] = dict(note="the dish is rigid and spaced from the rail, so liner pressure is displacement-controlled: p = k * delta",
    per_mm_LD33_kPa=round(K_LD33, 1), per_mm_model_kPa=K_MODEL, mm_to_reach_4p3kPa_LD33=round(4.3 / K_LD33, 3), mm_to_reach_4p3kPa_model=round(4.3 / K_MODEL, 3),
    precompression_in_mj_eval_mm=1.9, p_at_1p9mm_LD33_kPa=round(1.9 * K_LD33, 1), p_at_1p9mm_model_kPa=round(1.9 * K_MODEL, 1))
day = 500; out["abrasion"] = dict(strokes_per_day_assumed=day, sliding_m_per_day=[round(day * s * 1e-3, 1) for s in TRAVEL], sliding_km_per_year=[round(day * s * 1e-3 * 365 / 1000, 1) for s in TRAVEL])
json.dump(out, open(os.path.join(MD, "shear.json"), "w"), indent=1)
print(json.dumps({k: out[k] for k in ("inputs", "heat_worst_case", "fit_sensitivity", "abrasion")}, indent=1))
print("p0 case        p   N    mu  F_N  tau_kPa  E_J(56,96)  slip@skin")
for r in out["rows"]:
    if r["case"] == "static" or r["p0"] == 4.3: print(r["p0"], r["case"][:4], r["p_kPa"], r["N"], r["mu_slip"], r["F_shear_N"], r["tau_kPa"], r["E_per_stroke_J"], r["slip_at_skin_dry"], r["slip_at_skin_if_pair_x2_sweat"])
