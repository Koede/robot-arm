"""S2 reconciliation step 1 (copied into the pipeline 2026-10-15 for the I1b zero-length balancer): per-part masses and full 3-D first moments of every group beyond S2, from the actual meshes
(same density map and bought-part list as verify_distal.py rev 2026-10-14h), in each group's own frame at q = 0 (arm straight up).
Writes stl/balancer/s2_massmodel.json.  Units: g, mm, kg*mm."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak
G = ak.groups(g_pct=50.0)
CHAIN = ["M2", "M3", "ME", "MW1", "MW2", "MW3"]
CLAW_MAT = {"hand_tray": "petg_cf", "servo_tab": "petg_cf", "prox_": "pa_cf", "dist_": "pa_cf", "link_drive_": "pa_cf", "link_couple_": "pa_cf",
            "claw_rotor": "pa_cf", "claw_ring": "pa_cf", "stop_cap": "petg_cf"}
DENS = dict(petg_cf=1.30, pa_cf=1.20, petg=1.27, tpu=1.21)
def dens(k):
    if k.startswith("tpu"): return 1.21
    b = k.rstrip("0123456789"); return DENS[CLAW_MAT[b]] if b in CLAW_MAT else 1.27
PROXY = ("sts", "xm_", "ak_", "shaft", "pin_", "stop_pin", "bearing")
items = {g: [] for g in CHAIN}
for g in CHAIN:
    for k, m in G[g].items():
        if k.startswith(PROXY): continue
        v = abs(m.volume) / 1e3
        if v <= 0: continue
        c = m.center_mass if m.is_watertight else m.bounds.mean(0)
        items[g].append((k, v * dens(k), [float(x) for x in c], "mesh"))
def pc(g, k): return [float(x) for x in G[g][k].bounds.mean(0)]
LAY = ak.LAY
bought = {"M2": [("xm_s3", 165.0, pc("M2", "xm_s3"), "bought")], "M3": [("xm_elbow", 165.0, pc("M3", "xm_elbow"), "bought")],
          "ME": [("sts_w1", 55.0, pc("ME", "sts_w1"), "bought"), ("wiring/fasteners ME (est.)", 25.0, [0, 40.0, 330.0], "est")],
          "MW1": [("sts_w2", 55.0, pc("MW1", "sts_w2"), "bought"), ("6808-2RS", 22.0, [0, 57.0, LAY["BRG_Z0"] + 3.5], "bought"), ("wiring/fasteners MW1 (est.)", 30.0, [0, 57.0, 380.0], "est")],
          "MW2": [("sts_w3", 55.0, pc("MW2", "sts_w3"), "bought")],
          "MW3": [("sts_g", 55.0, pc("MW3", "sts_g"), "bought"), ("C3 pins + inserts/screws", 8.4, [0, 57.0, 520.0], "bought")]}
for g, l in bought.items(): items[g] += l
out = {}
for g in CHAIN:
    m = sum(i[1] for i in items[g]); M = np.sum([i[1] * np.array(i[2]) for i in items[g]], axis=0)
    out[g] = dict(mass_g=round(m, 1), first_moment_gmm=[round(float(x), 1) for x in M], cg_mm=[round(float(x), 1) for x in M / m],
                  parts={i[0]: dict(g=round(i[1], 1), c=[round(x, 1) for x in i[2]], src=i[3]) for i in items[g]})
tot = sum(out[g]["mass_g"] for g in CHAIN); Mz = sum(out[g]["first_moment_gmm"][2] for g in CHAIN)
out["_summary"] = dict(total_beyond_S2_g=round(tot, 1), cg_z_mm=round(Mz / tot, 1), Z_TIP=LAY["Z_TIP"], D=LAY["D_OFF"],
                       note="claw posed at G 50; masses verify_distal.py density map; head dome is in M2; no M2/M3 wiring line (verify_distal has none)")
json.dump(out, open(os.path.join(HERE, "..", "stl", "balancer", "s2_massmodel.json"), "w"), indent=1)
for g in CHAIN: print(g, out[g]["mass_g"], out[g]["cg_mm"])
print(out["_summary"])
