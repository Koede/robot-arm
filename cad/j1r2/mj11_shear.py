"""MJ11 liner shear / friction / heat on the moving blade skin (2026-11-03).  usage: J1R2_ROOT=<root> python3 mj11_shear.py -> stl/j1r2_mounts/MJ11/shear.json
Normal loads: metrics.json frame.{park,reach}.contact.liner (run_frame, E_FOAM 0.15 MPa).  Friction coefficients are ASSUMED ranges (no direct PTFE-fabric / tricot / skin
measurement found; see spec for sources).  Skin-contact area: the dish/liner footprint minus the lead channels."""
import os, sys, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, shapely.geometry as sg
import mj_frame as M, mj_concepts3 as K
OUT = os.path.join(M.REPO, "stl", "j1r2_mounts", "MJ11"); met = json.load(open(os.path.join(OUT, "metrics.json")))["frame"]
disc = sg.Point(K.Y1, K.Z1).buffer(66.0, 96); fp = disc.intersection(sg.box(K.RAIL_EDGE_Y, -500, 100, 100))
chan = sum(sg.LineString([np.array([K.Y1, K.Z1]) + 18 * np.array([math.cos(math.radians(p)), math.sin(math.radians(p))]), np.array([K.Y1, K.Z1]) + 80 * np.array([math.cos(math.radians(p)), math.sin(math.radians(p))])]).intersection(fp).length * 8.0 for p in K.LEAD_PSI)
A = fp.area - chan                                                                      # mm2, skin-facing liner area
res = dict(area_mm2=round(A, 0), channels_mm2=round(chan, 0))
inc = {lc: met[lc]["contact"]["liner"] for lc in ("park", "reach")}
res["increment_kPa"] = inc
P_BUDGET = 4.3                                                                          # kPa, project band budget (SPEC_flex_electrode_leads s5; ~32 mmHg capillary closing, order of magnitude)
p_lift = -min(inc["park"]["min_kPa"], inc["reach"]["min_kPa"]); p_max_inc = max(inc["park"]["max_kPa"], inc["reach"]["max_kPa"])
res["preload_window_kPa"] = dict(min_to_avoid_lift_off=round(p_lift, 2), max_for_budget=round(P_BUDGET - p_max_inc, 2))
MU = dict(slip_dry=(0.10, 0.25), slip_wet=(0.20, 0.40), foam_on_tricot=(0.6, 1.0))        # ASSUMED, see spec
S = dict(travel=(56e-3, 96e-3)); T_STROKE = 1.0; V = 96e-3 / T_STROKE                    # m ; s per elevation stroke (assumed 1 s) -> up to 0.096 m/s
EFF = 1180.0                                                                              # skin thermal effusivity W s^0.5 /m2K (k 0.37, rho 1100, c 3400)
rows = []
for p0 in (2.0, 2.1, 3.0):
    for lc, pk in (("park", inc["park"]["max_kPa"]), ("reach", inc["reach"]["max_kPa"])):
        p = p0 + pk
        for k, (lo, hi) in MU.items():
            for mu in (lo, hi):
                tau = mu * p; F = mu * p * 1e3 * A * 1e-6; q = tau * 1e3 * V; dT = 2 * q * math.sqrt(T_STROKE / math.pi) / EFF
                rows.append(dict(p0=p0, case=lc, p_total_kPa=round(p, 2), interface=k, mu=mu, shear_kPa=round(tau, 2), drag_N=round(F, 1), heat_W_m2=round(q, 0), dT_per_stroke_K=round(dT, 3),
                                 work_per_stroke_J_per_m2=round(tau * 1e3 * 96e-3, 1)))
res["cases"] = rows
# stick-mode check: if the foam gripped the base layer the tissue would have to take the travel as shear strain across the 8 mm foam
G_FOAM = 0.05                                                                               # MPa ASSUMED shear modulus closed-cell PE/EVA foam
res["stick_mode"] = dict(gamma=round(76.0 / K.LINER_T, 1), tau_MPa=round(G_FOAM * 76.0 / K.LINER_T, 2), note="travel 76 mm over 8 mm foam: shear strain ~9.5, impossible; the slip face must slip, so mu_slip sets the skin shear")
res["blister_heat_ref"] = dict(median_rise_K=5.1, threshold_K=3.0, src="Hashmi 2013 (as summarised by blister-prevention.com); Kirkham 2014 threshold")
# cycles: assumed 2000 strokes per hour of active use, 56..96 mm -> sliding distance per hour
res["sliding_m_per_hour"] = dict(strokes_per_h=2000, low=round(2000 * 56e-3 * 2, 0), high=round(2000 * 96e-3 * 2, 0))
json.dump(res, open(os.path.join(OUT, "shear.json"), "w"), indent=1)
for r in rows:
    if r["case"] == "reach": print(r)
print({k: res[k] for k in ("area_mm2", "channels_mm2", "preload_window_kPa", "stick_mode", "sliding_m_per_hour")})
