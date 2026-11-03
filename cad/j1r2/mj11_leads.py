"""MJ11 electrode-lead study (SENSING / WING / SHOULDER, 2026-11-03).  Robot frame, mm, kPa.
Which atlas electrodes lie under the MJ11 dish + liner, how far the skin (and the electrode, which sits in the compression base layer) slides under it,
what lead-length variation that causes for each candidate liner channel, and the liner pressure the electrodes see.
Skin-travel model = muscle_atlas.travel(): rigid-blade rotation about PIV (y 14.75, z -116.22) by the site angle (chord = travel_mm), sign UNKNOWN -> both signs are evaluated and the worse one is kept.
usage: python3 mj11_leads.py   -> stl/j1r2_mounts/MJ11/leads.json"""
import os, sys, json, math, itertools
import numpy as np
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
ATL = json.load(open(os.path.join(REPO, "stl", "shoulder", "emg_atlas.json")))["sites"]
LP = json.load(open(os.path.join(REPO, "stl", "j1r2_mounts", "MJ11", "liner_pressure.json")))
PIV = np.array([14.75, -116.22]); C = np.array([-60.0, -190.0]); R_PAD, R_LIN, EDGE_Y = 66.0, 62.0, -96.0   # liner lattice radius PAD_R-4, dish cut y >= -96
P0_NOM = 1.5            # kPa strap/garment preload on the liner (ASSUMPTION; sweep 0.5..2.0).  Electrode optimum 1.0-2.0 kPa (Wearable EMG polyurethane foam paper, PMC11124951)
T_REACH = 0.5           # fraction of full-elevation skin travel at the 'reach' load case (arm horizontal; scapulohumeral ~2:1) ASSUMPTION
def rotp(p, ang):
    r = p - PIV; c, s = math.cos(ang), math.sin(ang); return PIV + np.array([r[0] * c - r[1] * s, r[0] * s + r[1] * c])
def path(p, travel, sign, n=21):
    r = np.linalg.norm(p - PIV); ang = 2 * math.asin(min(1.0, travel / (2 * r))) if travel else 0.0
    return np.array([rotp(p, sign * ang * t) for t in np.linspace(0, 1, n)])
def in_liner(q): return (np.hypot(q[..., 0] - C[0], q[..., 1] - C[1]) <= R_LIN) & (q[..., 0] >= EDGE_Y + 2)
def inc_at(q, lc):
    rows = LP[lc]; yz = np.array([[r["y"], r["z"]] for r in rows]); pk = np.array([r["p_kPa"] for r in rows]); d = np.hypot(*(yz - q).T)
    if d.min() > 14: return None
    w = np.exp(-(d / 10.0) ** 2); return float((w * pk).sum() / w.sum())
ELE = {}
for k, v in ATL.items():
    tr = v.get("travel_mm")
    if tr is None or v["a"][0] < -170.0: continue          # back-of-blade sites only (muscle_atlas.travel() rule; excludes the chest / front sites)
    for e in ("a", "b"): ELE[f"{k}.{e}"] = (k, np.array(v[e][1:3]), tr)
out = dict(assumptions=dict(P0_kPa=P0_NOM, T_reach=T_REACH, travel_sign="both evaluated", liner_R=R_LIN, edge_y=EDGE_Y), electrodes={})
for name, (site, p, tr) in ELE.items():
    rec = dict(site=site, travel_mm=tr, park_in_liner=bool(in_liner(p)), d_center_mm=round(float(np.hypot(*(p - C))), 1))
    for sg, lab in ((+1, "plus"), (-1, "minus")):
        P = path(p, tr, sg); m = in_liner(P); rec[f"dwell_{lab}"] = round(float(m.mean()), 2); rec[f"end_{lab}"] = np.round(P[-1], 1).tolist()
        pr = {}
        for lc, t in (("park", 0.0), ("reach", T_REACH)):
            q = path(p, tr, sg)[int(round(t * 20))]; inc = inc_at(q, lc); pr[lc] = None if (inc is None or not in_liner(q)) else round(inc, 2)
        rec[f"inc_kPa_{lab}"] = pr
    rec["ever_in_liner"] = rec["park_in_liner"] or rec["dwell_plus"] > 0 or rec["dwell_minus"] > 0
    out["electrodes"][name] = rec
UNDER = sorted({r["site"] for r in out["electrodes"].values() if r["ever_in_liner"]})
out["sites_ever_under_liner"] = UNDER; out["sites_parked_under_liner"] = sorted({r["site"] for r in out["electrodes"].values() if r["park_in_liner"]})
# ---------------------------------------------------------------- channel search: lead-length variation  dL = max_t D - min_t D, D = |e(t) - c(r_in)| (c on the channel centre line), worst of both signs and both electrodes
def exit_r(psi):
    a = math.radians(psi); cx = math.cos(a)
    return R_PAD if cx > -1e-6 else min(R_PAD, (EDGE_Y - C[0]) / cx)       # channel ends at the dish rim or at the y = -96 cut (rail edge)
def dL(psi, site, r_in):
    a = math.radians(psi); c = C + r_in * np.array([math.cos(a), math.sin(a)]); worst = 0.0; dmax = 0.0
    for e in ("a", "b"):
        _, p, tr = ELE[f"{site}.{e}"]
        for sg in (1, -1):
            D = np.hypot(*(path(p, tr, sg) - c).T); worst = max(worst, D.max() - D.min()); dmax = max(dmax, D.max())
    return worst, dmax
def best_inlet(psi, site):
    rs = np.arange(18.0, exit_r(psi) + 0.1, 2.0); v = [dL(psi, site, r) for r in rs]; i = int(np.argmin([x[0] for x in v])); return float(rs[i]), v[i][0], v[i][1]
PSIS = list(range(120, 331, 15)); tab = {}
for site in ["infraspin", "teres_maj", "rhomboid"]:
    tab[site] = {psi: best_inlet(psi, site) for psi in PSIS}
out["channel_table"] = {s: {str(psi): dict(r_in=v[0], dL_mm=round(v[1], 1), Dmax_mm=round(v[2], 1), exit_r=round(exit_r(psi), 1)) for psi, v in d.items()} for s, d in tab.items()}
cur = (225, 270, 315); CH_R0 = 18.0; out["current_channels"] = {s: {str(psi): dict(zip(("r_in", "dL", "Dmax"), np.round(best_inlet(psi, s), 1).tolist())) for psi in cur} for s in ("infraspin", "teres_maj")}
# best single channel per site, then best 3-channel set minimising the worst dL over {infraspin, teres_maj} (rhomboid shares the medial channel)
best = {s: min(tab[s].items(), key=lambda kv: kv[1][1]) for s in tab}
out["best_channel_per_site"] = {s: dict(psi=psi, r_in=v[0], dL_mm=round(v[1], 1), Dmax_mm=round(v[2], 1)) for s, (psi, v) in best.items()}
# ---------------------------------------------------------------- accordion (serpentine) slack: flat flex tail 0.1 mm, centreline bend radius RB (ASSUMPTION: 100 x thickness for a dynamic flex, to be confirmed by bench test T-L3)
RB = 5.0; PIT = 2 * RB + 2.0; AMP = 32.0; STORED = math.pi * RB + (AMP - 2 * RB) - PIT      # length stored per half-turn when compressed (pitch PIT)
def acc(psi, site, r):
    a = math.radians(psi); c = C + r * np.array([math.cos(a), math.sin(a)]); Dm, DM, dl = 1e9, 0.0, 0.0
    for e in ("a", "b"):
        _, p, tr = ELE[f"{site}.{e}"]
        for sg in (1, -1):
            D = np.hypot(*(path(p, tr, sg) - c).T); Dm = min(Dm, D.min()); DM = max(DM, D.max()); dl = max(dl, D.max() - D.min())
    n = math.ceil(1.25 * dl / STORED); Lc = (n + 1) * PIT
    return dict(Dmin=round(Dm, 1), Dmax=round(DM, 1), dL=round(dl, 1), slack_req=round(1.25 * dl, 1), n_half_turns=n, L_compressed=Lc, fits=bool(Dm >= Lc))
out["accordion"] = dict(RB=RB, pitch=PIT, amplitude=AMP, stored_per_half_turn=round(STORED, 1), table={})
for site in ("infraspin", "teres_maj"):
    out["accordion"]["table"][site] = {}
    for psi in cur:
        rows = [(r, acc(psi, site, r)) for r in np.arange(CH_R0, exit_r(psi) + 0.1, 2.0)]; ok = [x for x in rows if x[1]["fits"]]
        out["accordion"]["table"][site][str(psi)] = dict(feasible=len(ok) > 0, best=(min(ok, key=lambda x: x[1]["Dmax"]) if ok else min(rows, key=lambda x: x[1]["Dmin"] - x[1]["L_compressed"]))[1] | dict(r_in=float((min(ok, key=lambda x: x[1]["Dmax"]) if ok else min(rows, key=lambda x: x[1]["Dmin"] - x[1]["L_compressed"]))[0])))
# ---------------------------------------------------------------- electrode contact pressure p0 + increment (kPa) for electrodes under the liner
out["electrode_pressure"] = {}
for name, r in out["electrodes"].items():
    if not r["ever_in_liner"]: continue
    incs = [v for lab in ("plus", "minus") for v in r[f"inc_kPa_{lab}"].values() if v is not None]
    out["electrode_pressure"][name] = {str(p0): [round(p0 + min(incs), 2), round(p0 + max(incs), 2)] for p0 in (0.5, 1.0, 1.5, 2.0)} if incs else None
json.dump(out, open(os.path.join(REPO, "stl", "j1r2_mounts", "MJ11", "leads.json"), "w"), indent=1)
print("sites ever under liner:", UNDER, " parked:", out["sites_parked_under_liner"])
for n, r in out["electrodes"].items():
    if r["ever_in_liner"]: print(n, "d=%.0f" % r["d_center_mm"], "tr", r["travel_mm"], "dwell+/-", r["dwell_plus"], r["dwell_minus"], "inc+", r["inc_kPa_plus"], "inc-", r["inc_kPa_minus"], "end", r["end_plus"], r["end_minus"])
print("current channels:", json.dumps(out["current_channels"]))
print("best per site:", out["best_channel_per_site"])
for s in tab: print(s, {psi: round(v[1], 0) for psi, v in tab[s].items()})

print("accordion:", json.dumps(out["accordion"]["table"]))
print("electrode pressure:", json.dumps(out["electrode_pressure"]))
