# SPEC: MJ11 whole-arm first-contact sweep (2026-11-03)
Subsystem: CONTROL / SHOULDER. Scripts: `cad/j1r2/mj11_whole_arm.py` (sweep), `mj11_tables.py` (frontier + tier tables), `mj11_shoulder_box.py` (5 deg shoulder box + keep-out mask), `mj11_gen_elbow_h.py`, `mj11_plots.py`. Outputs: `stl/j1r2_sweep/` (`sweep_mj11_whole_arm.json`, `tables_mj11_whole_arm.npz`, `mj11_whole_arm_summary.json`, `mj11_elbow_limits_{full,restricted}.json`, `mj11_shoulder_box.json`, `firmware_candidate/elbow_limits.h`, `mj11_whole_arm_tiers.png`).

## 1. Setup
Method: `specs/SPEC_collision_method.md` (first-contact frontier only; deep contact dropped). Scene = `mj11_sweep.scene(..., joints_upto="W3")`: real MJ11 fixed parts (pad (-60,-190,12), seat dish, liner, rail spacers, clasp bracket at CLASP_PSI 120), rail-only wing, rail, S1 AK70, stop posts, bumpers, wearer skin + head/neck (60 mm gate); moving groups M1, M2, M3 (upper arm + elbow ball), ME, MW1, MW2, MW3 (HAND rev B/C envelope proxy from `j1r2_kin`, +-60 mm box, conservative). Gates 3 mm parts, 2 mm skin, 60 mm head. Grid (soft range): S1' and S2 10 deg (+park -18.2 / 17.4), S3 {-70,0,90}, E 20 deg (+park 45), W1 {-80,0,80}, W2 {-70,0,80}, W3 {-20,0,30}: 170,100 poses, 789 s on 2 cores. Note: first-contact cells are grid samples; frontier d = 0.0 means the first sampled pose already penetrates (20 deg E / 10 deg S steps), not a measured clearance.

## 2. Result
114,760 contact, 55,340 reach-free from park (32.5 %), 35,073 frontier, 79,687 deep (dropped). Frontier cells by moving group: M1 11,864; M2 4,680; M3 7,731; ME 3,930; MW1 6,983; MW2 11,318; MW3 15,285 (a cell counts once per group; groups overlap). Per-pair detail in `mj11_whole_arm_summary.json`. Hand / wrist groups dominate (proxy box). Distal pairs against MJ11 parts (pad, dish, liner, clasp bracket) all have frontier cells: min reach-free clearance MW3|clasp bracket 3.3 mm, MW3|wing 3.4, MW2|clasp bracket 3.6, MW2|pad 3.8, MW1|clasp bracket 6.4. M1/M2 vs all MJ11 fixed parts PASS (M2|clasp bracket contact only in the unreachable region).

## 3. Shoulder-level contacts to keep out (S1' x S2 5 deg, S3 -70/0/90; `mj11_shoulder_box.json`)
| pair | first-contact range (sub-grid) | worst pose | action |
|---|---|---|---|
| M2 vs skin (2 mm) | S1' -65..-10, S2 40.7..44.7 | S1' -25, S2 45 | S2 soft max < 40 for S1' -65..-10 (table) |
| M3 vs skin | S1' -95..-10, S2 32.9..44.0 | S1' -95, S2 35, S3 -70 | S2 cap 30 at S1' <= -10 (table) |
| M3 vs clasp bracket (psi 120) | S1' -6.9..5, S2 28.2..40 | S1' -5, S2 40, S3 -70 | S2 cap 25 at S1' >= -10 (table); bracket clearance is an open item |
| M3 vs head (60 mm gate) | S1' 9.4..35, S2 -1.2..25 | 43.7 mm at S1' 30, S2 5 | S1' soft max < 9 or keep-out; inside the hard-stop corner S1' 10 stays free (first contact at S2 in -20..22.4 is S1' 12.4) |
| M1 vs M3 | S2 -21.9 (all S1') | 1.5 mm at S2 -25, S3 -70 | S2 soft min -20 at S3 -70 |
Passive-stop box (S1' x S2 5 deg, +-5 deg hard-stop expansion free): S1' -95..5, S2 -15..17.4, coverage 0.466; reach-free coverage 0.786. Identical for S3 = {0} and S3 = {-70,0,90} (reproduces the 2026-11-02 adoption). `keepout` in `mj11_shoulder_box.json` holds the free/reachable mask per (S1', S2, S3) and the blocking pair for firmware keep-out tables.

## 4. Elbow / wrist tier tables (`mj11_tables.py`, 15 x 14 on (S1', S2); figure `mj11_whole_arm_tiers.png`)
E_max = largest sampled E with every E' <= E contact-free for all sampled combinations of the tier; -1 = E = 0 not free. FULL (S3 x W1 x W2 x W3 all): 0 cells unrestricted, 129/210 forbidden; park cell E_max 0 (the E = 45 park pose is free only because park wrists are neutral); best 80 deg at S1' -65..-75, S2 -20..-10. RESTRICT (S3 0, W2 0, W3 0 at grid resolution of |S3|,|W2| <= 30, |W3| <= 15; W1 any): 0 unrestricted, 111 forbidden, 120 deg over the S1' -75..5 / S2 -20..20 core, 100 at S2 30..40, 45-60 for S1' >= 15. NEUTRAL (E 0, all wrists 0): not free in 111 of 210 cells, none inside it (checked against the 5 deg shoulder reach mask) (S2 >= 50, S2 -30, S1' >= 15 with S2 >= 17, S1' <= -75 with S2 >= 40), so NEUTRAL stays always allowed where the shoulder keep-out allows the pose. Caveats: E step is 20 deg and wrists are 3 values (a sampled-grid bound, not certified below 2 deg; fine/full 7-joint grids need the Lipschitz fallback `collision_screw.py`); gates differ from the legacy tables (legacy 15 mm to body, here project gates 2/3/60 mm), so the FULL tier is not comparable cell for cell.

## 5. Firmware tables (`firmware/gen_tables.py`, headers from the Project docs)
Done: `elbow_limits.h` CANDIDATE in the gen_tables format (`stl/j1r2_sweep/firmware_candidate/elbow_limits.h`; axis 1 = S1'). NOT done (inputs missing):
- `keepout_table.h`: needs `stl/shoulder/keepout_map.npz` (not in repo; `cad/keepout_map.py` uses the pre-J1' `arm_kin` geometry, no MJ11 version exists).
- `gen_tables.py` itself also needs `stl/elbow_limits_15mm.json` and `stl/sweep_full.json` (not in repo; only `elbow_limits_15mm_restricted.json` exists), so it cannot be run as is; the candidate was written by `mj11_gen_elbow_h.py` instead.
- `balancer_keepout.h`, `grav_model.h`: need `stl/balancer/balancer_elbow_keepout.json`, `s2_massmodel.json`, `balancer_calc.json`, `verify_balancer.json`, `s2_grid.npz`, `stl/distal/distal.json` (not in repo; the balancer has no MJ11 home; S1' now carries gravity).
- `control.h` / `protocols.h` still assume the old S1 axis; the candidate table must be wired to S1' and the shoulder keep-out of section 3 before use. Decision for Forest.
