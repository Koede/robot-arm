"""MJ11 liner shear / friction / heat / abrasion budget (SHOULDER / HARNESS, 2026-11-03).  SI-ish: mm, N, kPa, W, K.
Normal load = strap/garment preload p0 (ASSUMPTION, swept) + increments from mj11_liner_loads.py (mj_frame model).  Slip face (PTFE fabric) slides on the base-layer fabric, which is held to the skin by skin-textile friction.
usage: python3 mj11_shear.py -> stl/j1r2_mounts/MJ11/shear.json"""
import os, sys, json, math
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
LP = json.load(open(os.path.join(REPO, "stl", "j1r2_mounts", "MJ11", "liner_pressure.json")))
import mj_frame as M, mj_concepts3 as K            # liner mesh volume -> area
A_LINER = abs(K.liner_mesh().volume) / K.LINER_T    # mm2 (8 mm thick extrusion of the skin-contact area, trenches excluded)
cells = [(r["p_kPa"], r["F_N"]) for r in LP["reach"]]; A_CELL = 16.0 ** 2
inc = {lc: np.array([r["p_kPa"] for r in LP[lc]]) for lc in ("park", "reach")}
incF = {lc: sum(r["F_N"] for r in LP[lc]) for lc in ("park", "reach")}
P0 = (0.5, 1.0, 1.5, 2.0)                           # kPa preload sweep
MU = dict(PTFE_film_best=0.05, slip_fabric_design=0.15, slip_fabric_degraded_wet=0.30, skin_textile_dry_nylon=0.37, skin_dry_mean=0.46, silicone_liner_dry=0.61)
TRAVEL = (56.0, 96.0)                               # mm per stroke (DESIGN_STATE s4)
T_STROKE = (1.0, 3.0)                               # s per full elevation stroke (ASSUMPTION; tip cap 0.5 m/s, wearer-led)
STROKES_DAY = 400                                   # full-range strokes per day of use (ASSUMPTION)
EFF = math.sqrt(0.37 * 1100.0 * 3500.0)             # skin thermal effusivity W s^0.5 /m2 /K  (k 0.37, rho 1100, c 3500: typical dermis values, ASSUMPTION)
out = dict(A_liner_mm2=round(A_LINER), inc_range_kPa={lc: [round(float(v.min()), 2), round(float(v.max()), 2)] for lc, v in inc.items()}, inc_sumN={k: round(v, 2) for k, v in incF.items()}, mu=MU, rows=[])
for p0 in P0:
    N0 = p0 * A_LINER / 1000.0                      # N (kPa * mm2 / 1000)
    for lc in ("park", "reach"):
        N = N0 + incF[lc]; pk = p0 + float(inc[lc].max()); pmin = p0 + float(inc[lc].min())
        row = dict(p0=p0, lc=lc, N_total=round(N, 1), p_peak_kPa=round(pk, 2), p_min_kPa=round(pmin, 2), liftoff=bool(pmin < 0), F={}, tau_peak_kPa={})
        for k, mu in MU.items():
            row["F"][k] = round(mu * N, 2); row["tau_peak_kPa"][k] = round(mu * pk, 3)
        out["rows"].append(row)
# heat and work (design slip fabric, nominal preload 1.5, reach load)
N = 1.5 * A_LINER / 1000.0 + incF["reach"]; res = {}
for mu_k in ("slip_fabric_design", "slip_fabric_degraded_wet", "skin_dry_mean"):
    mu = MU[mu_k]; F = mu * N
    for tr in TRAVEL:
        for ts in T_STROKE:
            v = tr / ts / 1000.0; q = mu * (N / (A_LINER * 1e-6)) * v           # W/m2 over the contact area (all dissipated at the sliding interface)
            dT = 2 * (0.5 * q) * math.sqrt(ts / math.pi) / EFF                  # K, half the heat into the skin side, semi-infinite solid, constant flux for one stroke
            res[f"{mu_k}|travel{tr:.0f}|t{ts:.0f}s"] = dict(F_N=round(F, 2), W_J=round(F * tr / 1000.0, 3), v_mm_s=round(v * 1000, 1), q_W_m2=round(q, 1), P_W=round(F * v, 3), dT_skin_K=round(dT, 4))
out["heat"] = res
out["wear"] = dict(strokes_day=STROKES_DAY, slide_m_day={f"{tr:.0f}mm": round(STROKES_DAY * tr / 1000.0, 1) for tr in TRAVEL}, slide_km_90day={f"{tr:.0f}mm": round(90 * STROKES_DAY * tr / 1e6, 2) for tr in TRAVEL},
                   energy_J_day_design=round(MU["slip_fabric_design"] * N * STROKES_DAY * 0.096, 1))
out["skin_attach_margin"] = {k: round(MU["skin_textile_dry_nylon"] / MU[k], 2) for k in ("slip_fabric_design", "slip_fabric_degraded_wet", "PTFE_film_best")}
json.dump(out, open(os.path.join(REPO, "stl", "j1r2_mounts", "MJ11", "shear.json"), "w"), indent=1)
print("A_liner %.0f mm2; increments" % A_LINER, out["inc_range_kPa"], out["inc_sumN"])
for r in out["rows"]:
    if r["p0"] in (1.0, 1.5): print(r["p0"], r["lc"], "N %.1f pk %.2f min %.2f" % (r["N_total"], r["p_peak_kPa"], r["p_min_kPa"]), "F", r["F"]["slip_fabric_design"], r["F"]["slip_fabric_degraded_wet"], r["F"]["skin_dry_mean"], "tau_pk", r["tau_peak_kPa"]["slip_fabric_design"])
for k in list(res)[:12]: print(k, res[k])
print(out["wear"], out["skin_attach_margin"])
