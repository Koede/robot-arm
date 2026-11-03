"""Interface contracts between subsystems (Third Shoulder).  Robot frame, mm, deg, N.
Created 2026-11-03 (the project instructions name cad/interfaces.py as the registry, but the repo had no copy: it was reconstructed from DESIGN_STATE.md, specs/ and the J1 rev 2 / MJ11 code).
Each contract: parties, status (CURRENT / CHANGED / RETIRED / PLACEHOLDER), parameters, source, and the gates that must be re-run when it changes.
`python3 interfaces.py` prints the table; `check()` compares the MJ11 numbers here with cad/j1r2/mj_concepts3.py (needs the J1R2_ROOT work root, see docs/ENV_SETUP.md).
"""
import json, os, sys

# ---------------------------------------------------------------------------------------------------------------- MJ11 (adopted 2026-11-02)
MJ11_PAD = dict(y1=-60.0, z1=-190.0, standoff=12.0, x0=-97.5, pad_R=66.0, pad_t=6.0, pad_trim_y=-121.0, dish_edge_y=-96.0)   # J1_PAD default in cad/j1r2/mj_frame.py
RAIL = dict(y_centre=-119.0, width=38.0, thickness=7.0, foam_gap=7.5, z=(-440.0, -80.0), node_dz=20.0, material="PA-CF", source="stl/shoulder/scapular/wing_R.stl, mj_frame.RAIL")
RAIL_BOSSES = dict(                      # NEW M4 heat-set insert bosses on the rail REAR face (+x side, the pad side); positions are PLACEHOLDER until the rail is re-cut
    n=4, z=(-160.0, -220.0), y=(-113.0, -105.0),
    insert="M4 heat-set, 5.6 mm OD x 4 mm long (project standard)", hole="D5.0..5.2 x 5.8 deep (PLACEHOLDER: set by test T-I1)", boss="D12 x 2 raised pad (PLACEHOLDER)",
    spacer=dict(od=10.0, thickness_mm={-160.0: 4.82, -220.0: 8.26}, note="rail_x(-119, z) to the pad plane x0; the rail face x varies 0.4..0.8 mm across y -113..-105, spacers are cut to the local x"),
    bolt={-160.0: "M4 x 16 SHCS", -220.0: "M4 x 20 SHCS"}, bolt_note="length = pad 6 + spacer + insert 4 (+ <= 1.5 mm tip overshoot); mj_concepts3 note says M4 x 20 for all: wrong for the 4.8 mm spacers (tip 5 mm past the insert into a 7 mm rail)",
    load_N=dict(park=231.5, reach=312.9, note="worst bolt-group max tension per bolt, MJ11 frame model (stl/j1r2_mounts/MJ11/metrics.json, 2026-11-03)"),
    pullout_need_N=1250.0, pullout_need_note="2 (SF) x 2 (dynamic) x 312.9 N; the 1 kN PA-CF figure is UNVERIFIED",
    risks=["bolt pitch y -113 / -105 = 8 mm: only 2.4 mm of material between the two 5.6 mm inserts (rule of thumb >= 2 x OD pitch = 11.2 mm)", "y -105 is 5 mm from the rail lateral edge (y -100): 2.2 mm wall from the insert OD to the edge",
           "pull axis = print layer normal when the rail is printed flat (weakest direction)", "trap_low electrode pair sits 12.9 mm from the z -220 bosses (flex-leads rule: no rigid within 20 mm of an electrode; the rail already covers it)"])
CONTRACTS = {
    "WING<->SHOULDER (J1 / S1' pad)": dict(status="CHANGED 2026-11-02 (MJ11 adopted), recorded 2026-11-03",
        old="J1 rev 2: chassis plate x -88..-80 carries the S1 base block (4 x M5), wing tongue (y_rel 72..104, z -150..-76), spine bridge, pauldron feet (2 x M4 csk at y +-50, z -109), spaulder rear feet (2 x M3 in the plate top edge y +-40, x -84)",
        new="pad plane x0 = %.1f, 4 x M4 through spacers into RAIL_BOSSES; the S1' AK70 sits on the pad rear face; load path arm -> S1' -> pad -> 4 bolts -> rail (+ dish liner as a compliant skin seat) -> clasp / paraspinal pads" % MJ11_PAD["x0"],
        retired=["chassis plate (chassis_axilla.py, plate.json)", "wing tongue", "spine bridge as a load path (bridge still joins L and R rails; verify_harness 'bridge' rows are void)", "S1 base 4 x M5 pattern", "pauldron feet 2 x M4 csk + spaulder feet 2 x M3 (ORPHANED: the armour has nothing to bolt to; fairing / pauldron re-fit is open)"],
        pad=MJ11_PAD, rail=RAIL, bosses=RAIL_BOSSES,
        rerun=["harness_loads.py", "body_loads.py", "verify_harness.py (bridge / tongue / plate gates)", "scapular_frame.py (insert bosses, rail cut)", "pauldron.py + verify_armor.py (feet)", "sweep_full.py + gen_tables.py"]),
    "SHOULDER S1' AK70-10 <-> pad": dict(status="PLACEHOLDER (flange pattern NOT sourced)",
        sourced=dict(body_diameter=89.0, length=50.25, mass_g=621.0, rated_Nm=8.3, peak_Nm=24.8, ratio="10:1", backlash_deg=0.2, bearing_CC0_N=(2000, 2520), connectors="CAN A1257WR-S-4P, UART A1257WR-S-3P, power XT30PW-M", source="specs/vendor_patterns.json (cubemars.com/goods-1031-AK70-10.html, 2D drawing text layer, retrieved 2026-11-02)"),
        placeholder_list=["output flange: hole count, PCD, thread (drawing text shows 6-M3 and 8-M3 (x2) callouts, which one is the output flange is NOT readable; the project's 6 x M4 on PCD 60 is probably wrong)",
                          "rear (fixed-side) mounting pattern that the pad rear face must match: count, PCD, thread, depth", "pilot / spigot diameter and height on the output side and the rear side (drawing callouts 7, 10, 3.5, 2 (+0.02/0), 0.5, 11.2 are unassigned)",
                          "output flange protrusion and the rear boss height (sets the pad-to-yoke gap)", "cable exit position and bend allowance (assumed inner end face, medial)", "centre bore diameter (D20 assumed)",
                          "M3 / M4 insert or through-hole choice in the 6 mm PA-CF pad (pad is thin: 6 mm; inserts need local bosses or a steel / AL backing plate)", "STEP file (CubeMars download) not opened this session"],
        pad_face=dict(x=MJ11_PAD["x0"] + MJ11_PAD["pad_t"], note="AK70 stands on the pad rear face XB = x0 + 6; motor R44.5 inside pad R66"), rerun=["j1r2_geom.py", "mj_eval.py MJ11"]),
    "SENSING <-> SHOULDER liner (electrode leads)": dict(status="NEW 2026-11-03 (MJ11 lead trenches)",
        trenches=dict(radial_psi_deg=(225.0, 270.0, 315.0), r_range_mm=(18.0, 80.0), width=8.0, depth="full liner thickness 8", rim_collector=dict(r=56.0, psi_deg=(315.0, 218.0)),
                      exit="one fixed exit at the rail edge: y = -96, z = -224 (psi 218 at r 56), dock block 8 x 14 x 26 on the rail lateral edge (PLACEHOLDER)"),
        routed=dict(infraspin="channel psi 225, inlet r 18, accordion 1..2 half-turns; slack required 24 mm; fixed run in trench ~33 mm + ~54 mm to the spine-bridge jack"),
        not_routable=dict(teres_maj="skin travel 56 mm sweeps the electrodes across every inlet: no inlet satisfies D_min >= compressed accordion length; Forest to decide: drop / relocate"),
        lead="flat 3-conductor shielded flex tail ~3 x 0.1 mm (requirement; product NOT selected; PLACEHOLDER), dynamic bend radius assumed 5 mm (T-L3)", jack="spine-bridge keyed 16-way header per SPEC_flex_electrode_leads section 3 (header part NOT selected, PLACEHOLDER)",
        rerun=["mj11_leads.py", "mj11_shear.py", "harness_loads.py"]),
}
BENCH_INSERT_PULLOUT = [
    "T-I1  insert installation: M4 heat-set 5.6 x 4 in PA-CF coupons (7 mm plate printed flat AND on edge); hole D 5.0 / 5.2 / 5.4; iron temperature and dwell; depth; check flush and perpendicularity, N = 10 per cell",
    "T-I2  static axial pull-out (insert axis = layer normal, and = in-plane), crosshead 5 mm/min, report peak and mean - 3 sigma; PASS >= 1250 N (2 x 2 x 312.9 N) per insert, in the 'dry as printed' and the 'conditioned' state",
    "T-I3  conditioning: PA absorbs water: repeat T-I2 after 7 days at 23 C / 50 % RH and after 40 C / 80 % RH 48 h; and with the insert hot-soaked at 60 C (summer back, 8 h)",
    "T-I4  geometry: pitch 8 mm (as drawn) vs 11.2 mm vs a 12 x 24 shared boss; edge distance 5 mm (as drawn, y -105 to the rail edge) vs 8 mm; record the failure mode (insert pull, wall split, layer delamination)",
    "T-I5  torque-out and over-torque: M4 torque 2.5 N.m and 3.5 N.m (insert must not spin: PASS torque-out >= 4 N.m); 5 cycles of re-tightening",
    "T-I6  cyclic: 0..313 N at 2 Hz, 1e5 cycles on 8 inserts (as 2 groups of 4 with the 2 spacer heights), then residual pull-out >= 80 % of T-I2",
    "T-I7  bolt tip: M4 x 16 on the 4.8 mm spacer and M4 x 20 on the 8.3 mm spacer; hole depth 5.8; floor thickness to the skin side >= 1 mm; check no tip contact, no bulge",
    "T-I8  creep: 313 N held 24 h at 40 C, displacement < 0.1 mm",
]
def check():
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "j1r2"))
    import mj_frame as M, mj_concepts as C, mj_concepts3 as K          # noqa
    ok = (abs(K.X0 - MJ11_PAD["x0"]) < 0.05 and K.BOLT_Z == RAIL_BOSSES["z"] and K.BOLT_Y == RAIL_BOSSES["y"] and abs(M.Y1 - MJ11_PAD["y1"]) < 1e-6 and abs(M.Z1 - MJ11_PAD["z1"]) < 1e-6)
    sp = {z: round(K.X0 - C.rail_x(-119.0, z), 2) for z in K.BOLT_Z}
    return dict(consistent=bool(ok), spacers=sp, spacer_contract=RAIL_BOSSES["spacer"]["thickness_mm"], trench_exit_ok=abs(K.RIM_PSI[1] - 218.0) < 1e-6)
if __name__ == "__main__":
    for k, v in CONTRACTS.items(): print("==", k, "|", v["status"])
    print("\n".join(BENCH_INSERT_PULLOUT))
    if "--check" in sys.argv: print(check())
