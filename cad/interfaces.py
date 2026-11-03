"""Interface contracts between subsystems (CLAUDE.md / project instructions: cad/interfaces.py + DESIGN_STATE section 13).
Created 2026-11-03: the repo had no cad/interfaces.py (only stl/j1r2/interfaces_j1r2.json for J1 rev 2); this file starts the registry with the contracts that
changed with the MJ11 adoption.  Units mm, deg.  PLACEHOLDER = unverified vendor / position data: never print, never order."""
CONTRACTS = {
 "WING_RAIL <-> J1_PAD (MJ11)": dict(
    status="CHANGED 2026-11-03 (MJ11 adopted 2026-11-02)", subsystems=("WING", "SHOULDER"),
    retired=("spine-bridge-to-chassis-plate tongue", "wing bridge over the blade", "8 mm 6061 chassis plate (338 g)", "J1 rev 2 riser + 4 standoffs at the rail top (z -93..-54)"),
    kept=("paraspinal rail y -138..-100, z -440..-80, 38 x 7 mm PA-CF, 7.5 mm foam gap", "spine bridge (3 straps z -130/-180/-230)"),
    new_bosses=dict(count=4, insert="M4 heat-set, 5.6 mm OD x 4 mm", where="rail rear face (+x)", y=(-113.0, -105.0), z=(-160.0, -220.0), note="PLACEHOLDER positions; rail wall 7 mm vs insert 4 mm leaves 3 mm"),
    spacers=dict(count=4, od=10.0, length_mm=(4.8, 8.3), mat="PETG-CF", note="4.8 mm at z -160, 8.3 mm at z -220 (rail rear face to the pad plane x -97.5)"),
    fasteners="4 x M4 x 20 SHCS (length to be re-checked against the 4.8 / 8.3 mm spacers)",
    loads=dict(bolt_max_N=dict(park=231, reach=312), insert_pullout_N_UNVERIFIED=1000, SF=dict(park=3.2, reach=1.6)),
    checks="spacers touch the rail by design (0 mm); pad to rail 3.7 mm, pad to spine bridge 3.3 mm (mj_eval harness_clearance)",
    open=("insert pull-out in PA-CF: bench test T-W-INS (SPEC_j1r2_mj11_leads_shear s6)", "boss wall thickness vs the electrode rule: trap_low disc is 1.9 mm from the z -220 spacer (rigid within 20 mm of an electrode)")),
 "S1' AK70 <-> J1 pad": dict(
    status="PLACEHOLDER", subsystems=("SHOULDER",), where="pad rear face (x -91.5), centred (y -60, z -190)",
    placeholder_pattern=dict(note="AK70-10 output flange pattern NOT verified (vendor drawing not retrieved this session); DO NOT PRINT holes",
        candidates=["6 x M4 on 60 mm PCD (J1 rev 2 position-only assumption, stl/j1r2/interfaces_j1r2.json)", "8 x M3 on 70 mm PCD (typical frameless-style, unverified)", "centre bore D20-25 for the output shaft cover (unverified)"],
        envelope="motor R44.5 inside pad R66, flange boss <= D90 at x >= -91.5"),
    fasteners="M4 SHCS (PLACEHOLDER)"),
 "ELECTRODE LEADS <-> JACK": dict(
    status="PROPOSED, not built", subsystems=("SENSING", "HARNESS", "WING"), jack=dict(y=-108.0, z=-272.0, size=(24, 12, 8), note="ASSUMED 16-way keyed header block on the rail rear face, PLACEHOLDER part"),
    lead=dict(OD=2.2, R_min=22.0, note="ASSUMED 3-conductor shielded 28 AWG silicone; 10 x OD bend rule from SPEC_flex_electrode_leads"),
    liner_channels=dict(psi=(225, 270, 315), width=8.0, height=8.0, note="see SPEC_j1r2_mj11_leads_shear s3: fixed channels cannot guide a lead that rides on the sliding base layer")),
}
BENCH_TESTS = {
 "T-W-INS": "M4 heat-set insert (5.6 x 4) pull-out and torque-out in the printed PA-CF rail wall: >= 10 coupons, wall 7 mm and boss-locally 7+; target mean >= 2 x the 312 N reach bolt load x 2 dynamic = 1.25 kN per insert, min >= 1.0 kN; also cyclic 10^5 at 150 N",
 "T-L-SHEAR": "sliding friction of the slip face on the base layer on a forearm/back sled, dry and with 0.5 mg/cm2 artificial sweat, p = 2, 4 kPa",
 "T-L-RIDGE": "lead ridge under the liner: pressure map of 0.35, 1.0, 2.2 mm leads under 8 mm foam at 2 kPa",
}
