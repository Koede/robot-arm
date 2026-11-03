# SPEC: MJ11 whole-arm first-contact sweep, tier tables, shoulder keep-outs (2026-11-03)

Subsystem: CONTROL / SHOULDER (cross-subsystem integration). Unattended scheduled run; no geometry, limit or firmware file was changed. Decisions that follow from these numbers are Forest's (section 7).

## 1. Scene and method
- Fixed: the real MJ11 parts (`mj11_pad`, `seat_dish`, `liner`, `rail_spacers`, `clasp_bracket` at CLASP_PSI 120), `rail`, `S1_AK70`, `S1_stop_posts`, `tpu_S1_bumpers`, rail-only wing (`wing_R.stl` cut to the paraspinal strip box x -300..100, y -150..-100, z -500..100, as `mj11_sweep.scene`), wearer skin (2 mm gate) and head/neck (60 mm gate). Pad (y -60, z -190, standoff 12). Part gate 3 mm.
- Moving: M1, M2, M3 (upper arm + elbow ball), ME, MW1, MW2, MW3 (HAND rev B/C envelope proxy from `j1r2_kin.Kin.proxies`: box x +-27.5, u +-60, z 491.6..624, i.e. the conservative +-60 mm hand box). Joints S1', S2, S3, E, W1, W2, W3, `joints_upto W3`.
- Method: SPEC_collision_method (first-contact frontier only; deep contact dropped), separable pair tables (`j1r2_sweep.py`), 2 cores. Designed contact (closer than the gate at park) excluded: M1|S1_AK70, M1|S1_stop_posts, M1|mj11_pad, M1|mj11_clasp_bracket and the adjacent group pairs.
- Grid (as `j1r2_integrate full`, the park value is inserted into every axis): S1' and S2 10 deg (15 x 14 points, S1' -95..35 incl. park -18.2, S2 -30..88 incl. 17.4), S3 {-70,0,90}, E 0..145 (0,20,40,45,60,..,140,145), W1 {-80,0,80}, W2 {-70,0,80}, W3 {-20,0,30}: 170,100 cells. Runtime 655 s.
- Scripts: `cad/j1r2/mj11_whole_arm.py` (modes full / restricted / straight / shoulder5), `mj11_whole_arm_report.py` (analysis, tier tables), `mj11_gen_tables.py` (proposed headers), `mj11_whole_arm_render.py`. Outputs in `stl/j1r2_sweep/` (`sweep_/tables_ mj11_*`, `mj11_whole_arm_report.json`), renders in `renders_j1r2/mj11_whole_arm/`.
- Re-analysis from the saved tables reproduces the run summary exactly (asserted in the report script).

## 2. Result: 170,100 cells
contact 114,760 (67.5 %), reach-free from park 55,340 (32.5 %), first-contact frontier 35,073, deep (dropped) 79,687. Park is free.

First-contact frontier cells per moving group (7-D cells; partner categories overlap, so they sum to more than the group total only where cells touch several partners):

| group | frontier cells | fixed parts | skin | head/neck | other groups |
|---|---|---|---|---|---|
| M1 (yoke) | 0 | 0 | 0 | 0 | 0 |
| M2 | 3,636 | 0 | 3,636 | 0 | 0 |
| M3 | 17,311 | 1,000 (clasp bracket) | 5,092 | 1,639 | 9,580 (M1) |
| ME | 3,930 | 91 | 1,287 | 2,648 | 0 |
| MW1 | 6,983 | 1,159 | 3,717 | 4,454 | 298 |
| MW2 | 11,360 | 2,325 | 7,499 | 6,721 | 1,178 |
| MW3 (hand box) | 16,005 | 2,499 | 11,638 | 8,803 | 1,737 |

The distal groups dominate because the hand envelope is the +-60 mm box. Distal frontier cells by partner: MW3 vs skin 11,638 and vs head 8,803; MW2 vs skin 7,499 and head 6,721; MW1 vs head 4,454; ME vs head 2,648. Min clearance on the frontier is 0.0 at the first-contact cell (grid cell), 38.5 mm for M3 vs head (below the 60 mm gate, not touching). Pairs without any frontier cell and PASS: M1 vs rail / wing / seat dish / liner / spacers / bumpers / skin / head, M2 and M3 vs every MJ11 part except the clasp bracket (M2 min free clearance 23.3 mm to pad and dish, M3 22.3 mm), ME vs every fixed part except the clasp bracket and wearer. Full per-pair list: `mj11_whole_arm_report.json` (`pairs_with_frontier`, `per_group`).

## 3. Shoulder-level contacts to keep out (S3-independent on this grid)
First-contact angles (sub-grid, linear zero crossing of d - gate) from the **5 deg shoulder grid** (`mj11_shoulder5`, `mj11_shoulder5_first_contacts.json`; the 10 deg whole-arm grid gives the same pairs but coarser crossings, e.g. M3 vs skin S2 35.7 instead of 32.9 at S1' -95, `mj11_shoulder_first_contacts.json`; use the 5 deg values). Constraints hold for S3 = -70, 0 and 90 alike.

| pair | first contact (S2 at given S1'; S1' at given S2) | as a limit / table |
|---|---|---|
| M1 vs M3 (yoke vs upper arm), 3 mm | S2 -21.9 for every S1' and S3 | S2 soft min -15 (box floor; hard stop -20 clears it by 1.9 deg); global S2 min -30 must not be used |
| M3 vs skin, 2 mm | S1' -95: 32.9; -90: 33.9; -85: 34.9; -80: 38.4; -75: 39.1; -70: 41.4; -65: 43.4; -60..-10: 43.3..44.0; at S2 35 S1' >= -84.7, at S2 40 S1' >= -71.4 | S2 <= 31 for S1' <= -85 |
| M2 vs skin, 2 mm | S1' -65: 44.7; -30: 44.3; -25: 43.1; -20: 42.6; -18.2: 42.2; -15: 40.7; -10: 42.3 | inside the M3 skin curve for S1' <= -75 only; S2 <= 39 for S1' -20..-10 |
| M3 vs clasp bracket (psi 120), 3 mm | S1' 5: 28.2; 0: 28.4; -5: 32.3; at S2 30 S1' <= -2.9, S2 35 <= -6.5, S2 40 <= -6.9 (nothing for S1' < -7) | S2 <= 26 for S1' >= -5 |
| M3 vs head/neck, 60 mm | S1' 10: 24.2; 15: 14.3; 20: 7.4; 25: 3.3; 30: 0.6; 35: -1.2; at S2 0 S1' <= 31.5, S2 5 <= 22.7, S2 10 <= 17.8, S2 17.4 <= 13.4, S2 20 <= 12.4, S2 25 <= 9.4 | S1' soft max 5 (box) keeps it out |

The M1|M3, clasp-bracket, head and M3-skin curves agree with the independent run of the same task on branch `claude/mj11-whole-arm-sweep-2026-11-03` (commit a24f6cb, see section 9).

Gate-free envelope for all of M1..M3 and every S3 (10 deg grid, `shoulder_ok_map_S3all` in the report): S2 from -20 up to 30 (S1' -95..-75), 40 (S1' -65..-15), 30 (-5), 20 (5), 10 (15), 0 (25), -10 (35). Soft S2 limits in the firmware (rom_limits S2 -30..88) therefore need the keep-out above; without it first contact with skin / clasp / head occurs inside the soft range. Points other than M1|M3 lie outside the passive box (software keep-out only, not stops).

## 4. Passive-stop box
Reproduced on the 5 deg shoulder grid with the whole-arm scene (`mj11_shoulder5`, S3 {-70,0,90}, straight distal arm): **S1' -95..5, S2 -15..17.4, coverage 0.466; reach-free 0.786** (identical to `mj11_sweep.py`, and identical for each S3 value). Hard stops at +-5 deg beyond the soft limits (S1' -100..10, S2 -20..22.4) are free of all gates. The 10 deg tier grid is coarser (one-cell expansion = 10 deg) and gives the conservative S1' -95..-5, S2 -10..17.4, coverage 0.343; use the 5 deg result. The distal groups do not constrain the shoulder box (they are handled by the tier tables).

## 5. Elbow / wrist tier tables (S1', S2) -> max elbow angle E (deg)
Rule as `sweep_full.py`: from E = 0 upward, the first E at which any S3 x wrist sample, in the tier, violates a gate ends the scan; -1 = even E = 0 violates (use NEUTRAL). Gates are the project gates of the collision method (3 mm part, 2 mm skin, 60 mm head/neck), NOT the 15 mm body margin of the 2026-10-13 `sweep_full.py` tables (the first-contact method replaced it); say so if the 15 mm margin is wanted back and the tables will be regenerated with it. E axis is the sweep grid (0,20,40,45,60,80,100,120,140,145), so a table value is the last passing grid point.
- FULL (any S3 / wrist, 3 x 27 samples): 80 of 210 cells allowed, max 80 deg (S1' -75..-65, S2 -20..-10), 28 cells >= 45 deg, none >= 90. Park cell (S1' -18.2, S2 17.4): 0 deg. EL_F is min(FULL grid, RESTRICT grid) (one cell lowered where the restricted corner samples failed earlier than the FULL grid samples; the RESTRICT box is inside the FULL range).
- RESTRICT (|S3| <= 30, |W2| <= 30, |W3| <= 15, W1 +-80; its own 3 x 3 x 3 x 3 sweep, 661 s, 29,526 frontier cells): 98 of 210 cells, max 100 deg (S1' -75..-55, S2 -20..-10), 80 cells >= 45, 5 cells >= 90. Park cell: 45 deg (the park elbow 45 sits exactly on the limit).
- NEUTRAL (E <= 5, |S3|, |W2|, |W3| <= 10): not unconditionally free. On the 10 deg grid 111 of 210 shoulder poses (straight arm, hand box) violate a gate; in the 2.5 deg straight-arm map the reach-free region from park (E = 0) is 1,206 of 2,700 cells, S2 -20..42.5, S1' over the full range. NEUTRAL can only be "always allowed" inside the shoulder keep-out of section 3 plus `keepout_table.h`. The old premise (neutral capsule >= 64.9 mm from head/body everywhere in the soft box) does not hold for the MJ11 geometry with the rev B/C hand box.
- Files: `mj11_elbow_limits.json`, `mj11_elbow_limits_restricted.json` (key "S1',S2", rounded to whole degrees as `gen_tables.py` expects), `mj11_keepout_map.npz` (S1, S2, C = min(skin, head) mm, `margin_gate` = min(skin-2, head-60, part-3), `reach_free_E0`), maps in `renders_j1r2/mj11_whole_arm/sweep_maps.png`.

## 6. Firmware tables
`gen_tables.py` (Project doc `firmware/gen_tables.py`) was read, not run. It reads `stl/shoulder/keepout_map.npz` (S1/S2, 75 x 70, from `keepout_map.py`), `stl/elbow_limits_15mm(.json|_restricted.json)` and `stl/sweep_full.json` (S1/S2 of the baseline ball model, 15 mm margin, from `sweep_full.py`), `stl/balancer/balancer_elbow_keepout.json`, `s2_massmodel.json`, `s2_grid.npz`, `balancer_calc.json`, `verify_balancer.json` and `stl/distal/distal.json`. Not available / not valid for this design: `keepout_map.npz`, `s2_massmodel.json`, `s2_grid.npz`, `distal.json` are not in the Project or the repo; the balancer files are for the removed I1b balancer (MJ11 round 2 is no-balancer, torque-gate decision open) and the S1 -> S1' change moves gravity onto S1 (control.h / protocols.h change, see SPEC_j1r2_integration section 3). So `balancer_keepout.h` and `grav_model.h` are NOT regenerated (stopped; need the balancer / torque-gate decision and a rebuilt mass model).
Done: the two headers that depend only on this sweep, written in the exact `gen_tables.py` format to `stl/j1r2_sweep/firmware_proposed/elbow_limits.h` (15 x 14, EL_F / EL_R on the S1' axis) and `keepout_table.h` (53 x 47, 2.5 deg lattice, KO_MM = straight-arm clearance to skin/head mm). They are proposals: they are not in `firmware/` (not in the repo), `control.h` indexes S1 not S1', and the tier / keep-out logic (NEUTRAL premise, section 5) needs a decision first.

## 7. Decisions for Forest (nothing was changed)
1. Soft limits / keep-out lines of section 3 (S2 max by S1' row, S1' max by S2 column, S2 min -15) for the MJ11 shoulder, to enter `rom_limits.py` and `keepout_table.h`. Option A: adopt the lines as software keep-out inside the passive box S1' -95..5, S2 -15..17.4 (all hard stops free, nothing else needed). Option B: widen the passive box by moving CLASP_PSI (the clasp bracket stops S1' > 5 at S2 30) or shaving the bracket.
2. Whether the firmware elbow tiers use the project gates (3 / 2 / 60 mm) as here or the old 15 mm body margin. With the rev B/C +-60 mm hand box the FULL tier is mostly 0-60 deg and 0 at park; HAND rev B on the wrist-v3 coupling (open item 6) will replace the box and should re-run this sweep (`mj11_whole_arm.py full`, ~11 min on 2 cores) before tables are fixed.
3. NEUTRAL tier: keep it "always allowed" only together with the shoulder keep-out, or re-define it (E <= 5 plus a straight-arm map check).

## 8. Open / not covered
Fairing re-fit, clasp bracket clearance past the first-contact table, harness_loads, balancer / torque gate, lead routing + liner shear (CLAUDE.md item 7); full 3 deg refinement of the hand-vs-head boundaries (sphere tracing) is not done; grid values at 10 deg are cell-wise and conservative (contact between grid points is possible for thin features, the 5 deg shoulder grid and the sub-grid first-contact angles of section 3 cover the shoulder).

## 9. Duplicate run
A second scheduled firing of this task pushed `claude/mj11-whole-arm-sweep-2026-11-03` (a24f6cb, 06:47 UTC) before this run pushed; this run is on `claude/mj11-whole-arm-sweep-2026-11-03-run2`. Cross-check: the whole-arm summary is identical (170,100 / 114,760 / 55,340 / 35,073 / 79,687), per-group frontier cells identical, shoulder first-contact curves and passive box identical. Differences: the other run applies the legacy 15 mm skin / harness margin to the arm groups (`elbow_limits_15mm_mj11*.json`, park FULL 0 / RESTRICT 120) and samples RESTRICT at S3 / W2 / W3 = 0 only; this run uses the project gates (3 / 2 / 60 mm), samples the RESTRICT box corners in its own 661 s sweep (park RESTRICT 45) and adds a 2.5 deg straight-arm map. Merge the two branches by hand (the spec and DESIGN_STATE lines collide); the numbers do not conflict.
