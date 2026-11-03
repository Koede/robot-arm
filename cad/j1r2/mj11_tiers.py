"""MJ11 elbow/wrist tier tables (FULL / RESTRICT / NEUTRAL) + straight-arm keep-out map + candidate firmware headers, derived from the whole-arm sweep tables
(stl/j1r2_sweep/tables_mj11_whole.npz + sweep_mj11_whole.json, written by mj11_whole.py).  Reads raw distance tables, so the gate can be re-applied:
  fw15  : firmware convention of the legacy elbow_limits_15mm tables (arm groups M3..MW3 only; M1/M2 keep the sweep gates): 15 mm to skin and to harness parts (pad, dish, liner, spacers, clasp bracket, wing), 60 mm head/neck, 3 mm stage + part-part
  sweep : the sweep gates (2 mm skin, 3 mm parts/harness, 60 mm head)
E_max(S1p,S2,tier) = largest grid elbow angle E_k such that every pose of the tier with E <= E_k (all S3/W1/W2/W3 values the tier allows, as sampled) is clear of every non-designed pair; -1 = not even E = 0.
Tiers (control.h): FULL any S3/wrist; RESTRICT |S3|<=30, |W2|<=30, |W3|<=15 (grid: S3=W2=W3=0, W1 any); NEUTRAL E<=5, |S3|,|W2|,|W3|<=10 (grid: E=0, S3=W2=W3=0, W1 any).
usage: python3 mj11_tiers.py [sweep dir]  -> mj11_tiers.json, elbow_limits_15mm_mj11*.json, sweep_full_mj11.json, keepout_map_mj11.npz, firmware_candidate/*.h"""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); D = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "..", "..", "stl", "j1r2_sweep"); D = os.path.abspath(D)
SOFT = dict(S1p=(-95.0, 35.0), S2=(-30.0, 88.0), S3=(-70.0, 90.0), E=(0.0, 145.0), W1=(-80.0, 80.0), W2=(-70.0, 80.0), W3=(-20.0, 30.0)); PARK = dict(S1p=-18.2, S2=17.4, S3=0.0, E=45.0, W1=0.0, W2=0.0, W3=0.0)
STEPS = {"S1p": 10.0, "S2": 10.0, "S3": [-70.0, 0.0, 90.0], "E": 20.0, "W1": [-80.0, 0.0, 80.0], "W2": [-70.0, 0.0, 80.0], "W3": [-20.0, 0.0, 30.0]}; J = list(SOFT)
def grid():                                              # same construction as j1r2_sweep.grid_values
    vals = {}
    for j in J:
        lo, hi = SOFT[j]; st = STEPS[j]
        v = np.array(sorted(set(np.round(st, 4)))) if isinstance(st, list) else np.append(np.arange(lo, hi - 1e-9, st), hi)
        v = np.array(sorted(set(np.round(np.append(v, PARK[j]), 4)))); vals[j] = v[(v >= lo - 1e-9) & (v <= hi + 1e-9)]
    return vals
vals = grid(); GS = tuple(len(vals[j]) for j in J)
sw = json.load(open(os.path.join(D, "sweep_mj11_whole.json"))); z = np.load(os.path.join(D, "tables_mj11_whole.npz")); assert [sw["grid"][j][2] for j in J] == list(GS), "grid mismatch"
STAGE = ("rail", "S1_AK70", "S1_stop_posts", "tpu_S1_bumpers")
def gate(key, conv):
    g, o = key.split("|"); 
    if o in ("M1", "M2", "M3", "ME", "MW1", "MW2", "MW3"): return 3.0
    if o == "wearer_head": return 60.0
    arm = g in ("M3", "ME", "MW1", "MW2", "MW3") and conv == "fw15"          # legacy tables gated only the arm groups at 15 mm; the yoke groups M1/M2 sit within 15 mm of the wing / pad by design
    if o == "wearer_skin": return 15.0 if arm else 2.0
    if o in STAGE: return 3.0
    return 15.0 if arm else 3.0                # harness parts: pad, seat dish, liner, rail spacers, clasp bracket, wing
def bcast(t, dims):
    return t.reshape([t.shape[dims.index(j)] if j in dims else 1 for j in J])
def bad_mask(conv, only=None):
    bad = np.zeros(GS, bool)
    for k, r in sw["pairs"].items():
        if r["designed"] or (only and k.split("|")[0] not in only): continue
        t = z[k.replace("|", "__")]; bad |= np.broadcast_to(bcast(t, r["dims"]) < gate(k, conv), GS)
    return bad
ax = {j: i for i, j in enumerate(J)}
def sel(j, ok): return [i for i, v in enumerate(vals[j]) if ok(v)]
TIERS = {"FULL": {}, "RESTRICT": dict(S3=lambda v: abs(v) <= 30, W2=lambda v: abs(v) <= 30, W3=lambda v: abs(v) <= 15), "NEUTRAL": dict(S3=lambda v: abs(v) <= 10, W2=lambda v: abs(v) <= 10, W3=lambda v: abs(v) <= 10)}
def emax(bad, tier):
    b = bad
    for j, f in TIERS[tier].items(): b = np.take(b, sel(j, f), axis=ax[j])
    if tier == "NEUTRAL": b = np.take(b, sel("E", lambda v: v <= 5.0), axis=ax["E"]); Ev = vals["E"][vals["E"] <= 5.0]
    else: Ev = vals["E"]
    b = b.any(axis=(ax["S3"], ax["W1"], ax["W2"], ax["W3"]))            # -> (S1p, S2, E)
    first = np.where(b.any(axis=2), np.argmax(b, axis=2), len(Ev)); out = np.where(first == 0, -1.0, Ev[np.minimum(first, len(Ev)) - 1] if len(Ev) else -1.0)
    out = np.where(first == 0, -1.0, np.array(Ev)[np.clip(first - 1, 0, len(Ev) - 1)]); return out
res = {"grid": {j: vals[j].tolist() for j in J}, "park": PARK, "note": "E_max per (S1p,S2); -1 = not allowed in that tier. Candidate tables for the MJ11 geometry; axes are S1' (S1p) and S2, NOT the legacy S1/S2 of elbow_limits.h."}
S1, S2 = vals["S1p"], vals["S2"]
for conv in ("fw15", "sweep"):
    bad = bad_mask(conv); res[conv] = {t: emax(bad, t).tolist() for t in TIERS}
    res[conv]["shoulder_only_bad"] = bad_mask(conv, only=("M1", "M2", "M3")).any(axis=(ax["S3"], ax["E"], ax["W1"], ax["W2"], ax["W3"])).tolist()
# straight-arm keep-out: min distance of M3..MW3 to the wearer (skin and head/neck volume) at E=0, S3=W=0
kc = np.full((len(S1), len(S2)), 1e9); idx = {j: int(np.argmin(abs(vals[j] - (0.0)))) for j in ("S3", "E", "W1", "W2", "W3")}
for k, r in sw["pairs"].items():
    g, o = k.split("|")
    if r["designed"] or o not in ("wearer_skin", "wearer_head") or g not in ("M3", "ME", "MW1", "MW2", "MW3"): continue
    t = np.broadcast_to(bcast(z[k.replace("|", "__")], r["dims"]), GS); kc = np.minimum(kc, t[:, :, idx["S3"], idx["E"], idx["W1"], idx["W2"], idx["W3"]])
res["straight_arm_clearance_mm"] = np.round(kc, 1).tolist()
json.dump(res, open(os.path.join(D, "mj11_tiers.json"), "w"), indent=1); np.savez_compressed(os.path.join(D, "keepout_map_mj11.npz"), S1=S1, S2=S2, C=kc)
key = lambda a, b: f"{a:.0f},{b:.0f}"
for conv, sfx in (("fw15", ""), ("sweep", "_sweepgate")):
    for t, nm in (("FULL", ""), ("RESTRICT", "_restricted")):
        json.dump({key(a, b): float(res[conv][t][i][k]) for i, a in enumerate(S1) for k, b in enumerate(S2)}, open(os.path.join(D, f"elbow_limits_15mm_mj11{sfx}{nm}.json"), "w"), indent=1)
json.dump({"S1": S1.tolist(), "S2": S2.tolist()}, open(os.path.join(D, "sweep_full_mj11.json"), "w"))
# ---- candidate firmware headers (same layout as firmware/gen_tables.py; NOT drop-in: axes are S1p/S2, control.h/protocols.h still use the legacy S1)
FW = os.path.join(D, "firmware_candidate"); os.makedirs(FW, exist_ok=True); el, elr = res["fw15"]["FULL"], res["fw15"]["RESTRICT"]
with open(os.path.join(FW, "elbow_limits_mj11.h"), "w") as f:
    f.write("// CANDIDATE (not adopted): generated by cad/j1r2/mj11_tiers.py from stl/j1r2_sweep/tables_mj11_whole.npz -- do not edit\n// MJ11 geometry, axes S1' and S2 (deg), NOT the legacy S1/S2 of elbow_limits.h. Gate: 15 mm to skin+harness, 60 mm head/neck, 3 mm stage/parts.\n"
            "// EL_F: any S3 / wrist.  EL_R: S3 0, W2 0, W3 0 sampled (|S3|<=30, |W2|<=30, |W3|<=15 intended; grid has no point between 0 and the extremes).  -1 = not allowed in that tier (use NEUTRAL).\n#pragma once\n")
    f.write(f"const int EL_N1 = {len(S1)}, EL_N2 = {len(S2)};\nconst float EL_S1[EL_N1] = {{{','.join(f'{v:.2f}f' for v in S1)}}};\nconst float EL_S2[EL_N2] = {{{','.join(f'{v:.2f}f' for v in S2)}}};\n")
    for nm, tab in (("EL_F", el), ("EL_R", elr)): f.write(f"const float {nm}[EL_N1][EL_N2] = {{\n" + ",\n".join("{" + ",".join(f"{v:.1f}f" for v in row) + "}" for row in tab) + "};\n")
with open(os.path.join(FW, "keepout_table_mj11.h"), "w") as f:
    f.write("// CANDIDATE (not adopted): straight-arm (E 0, S3 0, wrist 0) minimum distance of M3..MW3 to skin and head/neck, mm, on the (S1', S2) sweep grid (0 = contact). mj11_tiers.py\n#pragma once\n#include <stdint.h>\n")
    f.write(f"const int KO_N1 = {len(S1)}, KO_N2 = {len(S2)};\nconst float KO_S1[KO_N1] = {{{','.join(f'{v:.2f}f' for v in S1)}}};\nconst float KO_S2[KO_N2] = {{{','.join(f'{v:.2f}f' for v in S2)}}};\n")
    f.write("const int16_t KO_MM[KO_N1][KO_N2] = {\n" + ",\n".join("{" + ",".join(str(int(round(max(-999, min(999, v))))) for v in row) + "}" for row in kc) + "};\n")
def show(tab, title):
    print(title); print("S2:  " + " ".join(f"{v:5.0f}" for v in S2))
    for i, a in enumerate(S1): print(f"{a:6.1f} " + " ".join(f"{v:5.0f}" for v in tab[i]))
for conv in ("fw15", "sweep"):
    for t in TIERS: show(res[conv][t], f"== {conv} {t} E_max (deg), rows S1p, cols S2")
