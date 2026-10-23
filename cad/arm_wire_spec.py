"""In-arm wiring specification (right arm, one side): wire catalogue, bundles per joint crossing, connector ratings, bus currents.
Provenance: R = retrieved from a vendor/distributor page (stl/electronics/arm_vendor_data.json, retrieved 2026-10-17), F = family inference,
T = typical value, UNVERIFIED (PLACEHOLDER: measure or buy-and-measure before printing/ordering), C = Claude design choice.
Everything here is data; arm_wire_search.py / arm_wire_verify.py read it."""
import math

# ---------------------------------------------------------------- wire catalogue: OD of the insulated conductor (mm)
# 'ref' = thick-wall UL3135 silicone (R, supplier datasheets) ; 'ultra' = ultra-flexible thin-wall silicone, typical values (T, NOT retrieved)
OD = {"16": dict(ref=3.06, ultra=2.5), "18": dict(ref=2.70, ultra=2.1), "20": dict(ref=2.5, ultra=1.8), "22": dict(ref=2.32, ultra=1.6),
      "24": dict(ref=2.15, ultra=1.4), "26": dict(ref=2.05, ultra=1.25), "28": dict(ref=1.9, ultra=1.1)}
BEND_K = {"static": 7.5, "flex": 15.0}          # x OD, Helukabel UL3135 (R); 'low-cycle ultra-flex' design value 6 x OD is a C assumption
K_DESIGN = 6.0
FILL = 0.70                                     # bundle packing fraction (round bundle in a sleeve)
SLEEVE = dict(S1=0.6, S2=0.6, S3=0.6, E=0.6, W1=0.2, W2=0.2, W3=0.2)   # mm added to the bundle OD (braided sleeve on the big hops, bare twisted leads on the wrist hops) (C)
KDEF = dict(S1=6.0, S2=6.0, S3=6.0, E=6.0, W1=4.0, W2=4.0, W3=4.0)      # min bend radius / bundle OD (C; wrist hops use thin servo-lead-type bundles, T)

# ---------------------------------------------------------------- wires per crossing: (label, AWG, count)
CROSS = {
    "S1": dict(frm="F", to="M1", wires=[("24 V S2 AK70 +/-", "22", 2), ("CAN S2 pair", "28", 2), ("12 V servo +/-", "18", 2), ("XM bus data", "28", 1), ("STS bus data", "28", 1)],
               note="ONE sleeve for all 8 conductors (24 V + CAN to the S2 AK70, 12 V + 2 data to the servos); 24 V pair is 22 AWG because the AK70 DC-bus current is <= 2 A under the torque/speed caps and is fused locally at 3 A (C)"),
    "S2": dict(frm="M1", to="M2", wires=[("12 V servo +/-", "18", 2), ("XM bus data", "26", 1), ("STS bus data", "26", 1)]),
    "S3": dict(frm="M2", to="M3", wires=[("12 V servo +/-", "18", 2), ("XM bus data", "26", 1), ("STS bus data", "26", 1)]),
    "E":  dict(frm="M3", to="ME", wires=[("12 V STS +/-", "20", 2), ("STS bus data", "26", 1)]),
    "W1": dict(frm="ME", to="MW1", wires=[("12 V STS +/-", "22", 2), ("STS bus data", "26", 1)]),
    "W2": dict(frm="MW1", to="MW2", wires=[("12 V STS +/-", "22", 2), ("STS bus data", "26", 1)]),
    "W3": dict(frm="MW2", to="MW3", wires=[("12 V G +/-", "24", 2), ("STS bus data", "26", 1)]),
}
def bundle(name, kind="ultra", split=1):
    w = CROSS[name]["wires"]; grp = CROSS[name].get("groups")
    if grp:      # explicit sub-bundles: the OD of the LARGEST one is used for both (conservative for clearance, bend radius and sleeve length)
        areas = [sum(w[i][2] * math.pi / 4 * OD[w[i][1]][kind] ** 2 for i in gi) for gi in grp]; a = max(areas); split = len(grp)
    else: a = sum(n * math.pi / 4 * OD[g][kind] ** 2 for _, g, n in w) / split
    d = math.sqrt(a / FILL * 4 / math.pi) + SLEEVE[name]; return dict(area_mm2=round(a, 2), od_mm=round(d, 2), n_wires=sum(n for _, _, n in w), split=split)

# ---------------------------------------------------------------- currents (A) at 12 V unless stated; firmware limits are REQUIREMENTS recorded here
LIM = dict(
    XM540_stall_12V=4.4,                 # R  (ROBOTIS e-Manual): 10.6 N.m
    JST_EH_rating=3.0,                   # R  (JST EH, AWG22); applies to the XM540 3-pin TTL port (-T) and to the hub outputs
    STS3215_stall_12V=2.7, STS3215_rated_12V=0.9,    # R (openelab / datasheet)
    STS_KK254_rating=4.0,                # F  (Molex KK 254-class crimp; 5264 housing not stated by Feetech)
    AK70_rated=7.2, AK70_peak=23.2, XT30_cont=15.0, XT30_peak=30.0,   # R
    CAN_conn_rating=1.0,                 # R  A1257 1.25 mm, 1 A / contact
    XM_current_limit_A=2.5,              # C  requirement: Current Limit register (2.69 mA/count) <= 2.5 A ~ 6.0 N.m (elbow limit 6.04 N.m; JST EH 3 A; see arm_electrical.json)
    STS_chain_limit_A=3.0,               # C  sum of the four STS torque limits expressed as current
)
