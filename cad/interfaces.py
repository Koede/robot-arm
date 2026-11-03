"""Interface registry (contracts between subsystems), robot frame, mm.  Created 2026-11-03: the project instructions name cad/interfaces.py as the home of every
boundary contract, but the file did not exist in the repo; the older J1 rev 2 boundaries live in stl/j1r2/interfaces_j1r2.json (written by j1r2_integrate.py) and are
SUPERSEDED for the shoulder by the MJ11 entries below.  Data only: `python3 cad/interfaces.py` prints the registry and, when the J1 work root is set
(J1R2_ROOT, see docs/ENV_SETUP.md), cross-checks the numbers against cad/j1r2/mj_concepts3.py.

Status words: DECIDED (Forest), DERIVED (from the generating script), PLACEHOLDER (unverified vendor or unselected part: do not print, do not order)."""
import os, sys, json

CONTRACTS = {
    # ------------------------------------------------------------------------------------------------ WING <-> SHOULDER (J1 = MJ11), 2026-11-02 decision, 2026-11-03 detail
    "WING_SHOULDER_J1_MJ11": dict(
        status="DECIDED 2026-11-02 (Forest adopted MJ11); detail DERIVED 2026-11-03 (cad/j1r2/mj_concepts3.py); bosses and flange PLACEHOLDER",
        retired=["bridge (scapular wing)", "tongue", "chassis plate", "J1 rev 2 pad_riser, standoffs, fairing_lap"],
        kept=["paraspinal rail y -138..-100 (b 38, t 7, gap 7.5 off the skin, z -440..-80, nodes every 20 mm)", "spine bridge (3 PA-CF straps 60 x 5 at z -130/-180/-230)"],
        datum=dict(pad_centre_yz=(-60.0, -190.0), pad_plane_x=-97.5, pad_rear_face_x=-91.5, pad="R66 x 6 PA-CF disc, trimmed flat at y -121 (spine-bridge ears, 3.3 mm)", standoff_mm=12),
        rail_bosses=dict(
            count=4, where="rail rear face (the face toward +x), pairs at z -160 and z -220",
            yz=[(-113.0, -160.0), (-105.0, -160.0), (-113.0, -220.0), (-105.0, -220.0)],
            rail_rear_face_x={"z -160": -102.32, "z -220": -105.76},
            spacer_length_mm={"z -160": 4.82, "z -220": 8.26},          # = pad plane x -97.5 minus rail rear-face x; the spec rounds these to 4.8 / 8.3
            spacer="D10 PETG-CF cylinder from the rail rear face to the pad plane",
            insert="M4 heat-set insert, 5.6 mm OD x 4 mm (PLACEHOLDER boss geometry: wall, hole dia, boss height not designed)",
            fastener="M4 x 20 SHCS through the pad and the spacer (length not verified against the real pad hole depth)",
            loads_N={"park": 231.5, "reach": 313.2, "note": "worst single bolt, mj_eval run_frame (frame model, foam E 0.15 MPa); x2 dynamic = 626 N"},
            pullout_N_typical_UNVERIFIED=1000.0,
            pullout_SF={"static_reach": round(1000.0 / 313.2, 2), "dynamic_x2": round(1000.0 / 626.4, 2)},
            status="FAILS SF 2 at x2 dynamic if the 1 kN typical value holds: needs the bench test below or more inserts / bigger bosses"),
        keep_out=["jack plate and flex leads (below)", "clasp bracket feet at the pad rim psi 120", "spine-bridge ears (pad trim y -121)"],
        gates=dict(rail_spacers_to_wing_rail_mm="designed contact", jack_plate_to_wing_rail_mm=4.3, jack_plate_to_spine_bridge_mm=19.6)),
    # ------------------------------------------------------------------------------------------------ SHOULDER: S1' actuator on the pad rear face
    "SHOULDER_S1P_AK70_PAD": dict(
        status="PLACEHOLDER (no vendor drawing retrieved; position only, as in interfaces_j1r2.json)",
        location="pad rear face, x -91.5, concentric with the pad centre (-60, -190)",
        placeholders=[
            "AK70-10 output flange bolt pattern: 6 x M4 on 60 mm PCD (position only; thread, depth, rotation phase unverified)",
            "AK70-10 pilot / spigot diameter and depth on the output side",
            "AK70-10 housing diameter 89 (motor R44.5 inside the pad R66) and body length behind the pad",
            "AK70-10 cable exit side / connector position (phase and power) and the encoder cable bend radius",
            "flange-to-pad fastener length and any locating dowel",
            "S1' stop hardware (collar, posts) re-seat on the new pad: not re-checked",
        ],
        rule="do not print the flange holes until the vendor drawing replaces this list"),
    # ------------------------------------------------------------------------------------------------ SENSING <-> SHOULDER: liner, electrode relief, flex leads, jack
    "SENSING_SHOULDER_LINER_LEADS": dict(
        status="DERIVED 2026-11-03 (cad/j1r2/mj11_leads.py, mj_concepts3.py); header, lead product, electrode stack height PLACEHOLDER",
        liner="8 mm closed-cell foam + PTFE-fabric slip face toward the base layer; footprint R66 cut at y -96 (11347 mm2) minus relief slots",
        relief_slots=dict(depth_mm=5.5, face="skin side", pairs=["infraspin", "teres_maj", "rhomboid (entry notch)"], build="mj11_leads.slot_polys(): swept electrode footprint (atlas travel x 1.25, 1.5 mm margin) + 8 mm lead corridor along -d"),
        electrode_stack_height_mm_PLACEHOLDER=3.5,
        lead=dict(od_mm_assumed=1.6, bend_r_min_mm=16.0, construction="shielded 3-conductor 28 AWG silicone, >=100 % elongation (SPEC_flex_electrode_leads 2.1), product not selected"),
        jack_plate=dict(psi_deg=240.0, radial_mm=(62.0, 74.0), width_mm=26.0, thickness_mm=5.0, x_mm=(-103.5 + 0.0, -98.5),
                        header="16-way keyed, 1.27 mm pitch footprint: PLACEHOLDER (part and pinout not selected)", material="PETG-CF", mounting="2 bosses on the dish rim (not designed)",
                        clearances_mm=dict(arm_stage_M1=8.9, arm_stage_M2=20.0, skin=18.7, wing_rail=4.3, spine_bridge=19.6),
                        why_psi_240="lateral half of the rim (psi 290..70) is swept by the S1'/S2 stage (0.0..0.3 mm); psi 90..285 is clear of the stage but only psi 235..285 also clears rail (>=3) and spine bridge (>=3); 240 is the first and gives the shortest lead path: jack_scan.json"),
        old_channels="rev 2026-11-02 radial channels psi 225/270/315 RETIRED: psi 225 is 90..104 deg off the skin-motion direction, psi 270 25..60 deg, and the pairs sit 33 mm off the psi 315 axis"),
}

# ------------------------------------------------------------------------------------------------ bench tests for the new WING boss interface
BENCH_M4_INSERT_PULLOUT = dict(
    purpose="verify the M4 heat-set insert (5.6 mm OD x 4 mm) in a PA-CF boss carries the worst MJ11 bolt load with SF >= 2: 313 N static at reach, 626 N with the x2 dynamic factor => required mean-3sigma pull-out >= 1.25 kN (typical 1 kN is UNVERIFIED)",
    coupons="boss as designed on the rail rear face (not a block): n >= 10 per condition; print the real rail material and orientation",
    variables=[
        "layer orientation: pull axis along Z (layers peel) and in XY",
        "hole diameter / insert installation temperature and depth (record the melt-back height): nominal, -0.2, +0.2 mm hole",
        "boss wall: 2.0 / 3.0 / 4.0 mm around the insert",
        "moisture: dry-as-printed and conditioned (PA absorbs water): 23 C / 50 % RH two weeks, plus soak 24 h",
        "temperature: 23 C and 50 C (skin side, closed liner)",
    ],
    procedure=["pull-out at 5 mm/min with a bolt of the real length and the real spacer stack; record peak force and failure mode",
               "torque-out: install torque to strip, M4 SHCS target 2.5 N.m",
               "cyclic: 0..313 N, 5 Hz, 100,000 cycles, then pull-out (residual strength >= 80 %)",
               "creep: 313 N held 24 h at 50 C, displacement < 0.1 mm",
               "combined load: axial 313 N with 40 N shear (pad weight and reaction), no relaxation of preload",
               "assembly: 5 install / remove cycles of the bolt"],
    acceptance="mean - 3 sigma pull-out >= 1.25 kN in every condition, torque-out >= 5 N.m, creep < 0.1 mm, residual after cycling >= 80 %. If not met: 2 inserts per bolt position, a thicker boss, a through-bolt with a nut plate, or a rail insert of knurled brass with a larger OD (redesign, Forest's call).",
    status="NOT RUN (bench)")

def check():
    """cross-check against the generating script when the J1 work root is available"""
    os.environ.setdefault("J1R2_ROOT", "/home/claude/j1root")
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "j1r2"))
    import mj_concepts3 as K, mj11_leads as LD
    bolts = [(y, z) for (_, y, z, _) in K.rail_bolt_pts()]
    ok = {"bosses": sorted(bolts) == sorted(CONTRACTS["WING_SHOULDER_J1_MJ11"]["rail_bosses"]["yz"]),
          "pad": (K.Y1, K.Z1) == CONTRACTS["WING_SHOULDER_J1_MJ11"]["datum"]["pad_centre_yz"], "pad_plane_x": abs(K.X0 + 97.5) < 0.01,
          "spacers": all(abs((K.X0 - rx) - v) < 0.01 for (_, _, _, rx), v in zip(K.rail_bolt_pts(), [4.82, 4.82, 8.26, 8.26])),
          "jack_psi": K.JACK_PSI == CONTRACTS["SENSING_SHOULDER_LINER_LEADS"]["jack_plate"]["psi_deg"], "relief": LD.RELIEF_DEPTH == CONTRACTS["SENSING_SHOULDER_LINER_LEADS"]["relief_slots"]["depth_mm"]}
    return ok

if __name__ == "__main__":
    print(json.dumps(dict(contracts=CONTRACTS, bench_M4_insert_pullout=BENCH_M4_INSERT_PULLOUT), indent=1, default=str))
    try: print("cross-check vs mj_concepts3:", check())
    except Exception as e: print("cross-check skipped:", type(e).__name__, e)
