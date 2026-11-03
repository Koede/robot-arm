# MJ11 whole-arm first-contact sweep (2026-11-03) — CONTROL / SHOULDER

Scene: real MJ11 fixed parts (pad, seat dish, liner, rail spacers, clasp bracket at CLASP_PSI 120), J1 rev 2 stage fixed parts, rail-only wing, wearer skin (2 mm) and head/neck (60 mm); parts gate 3 mm. Pad (-60, -190), standoff 12. Moving groups M1, M2, M3, ME, MW1, MW2, MW3 with the HAND rev B/C envelope proxy of `j1r2_kin.py` (a +-60 mm box: conservative, the sign of the hand u offset is unknown). Method: `SPEC_collision_method.md` (separable pair tables, first-contact frontier only, designed-contact pairs excluded).
Scripts: `cad/j1r2/mj11_whole_arm.py` (sweep, 518 s on 2 cores), `mj11_whole_post.py` (frontier by group, shoulder box, tier tables), `mj11_shoulder5.py` + `mj11_shoulder5_post.py` (5 deg cross-check), `mj11_hardstop_check.py`, `mj11_s1p_edge.py`, `mj11_whole_fig.py`. Results in `stl/j1r2_sweep/` (`sweep_mj11_whole.json`, `tables_mj11_whole.npz`, `mj11_whole_report.json`, `elbow_tiers_mj11.json`, `firmware_preview/elbow_limits_mj11.h`, logs). Figure: `renders_j1r2/mj11_whole_arm_sweep.png`.

## 1. Grid and totals
S1' / S2 10 deg (park -18.2 / 17.4 added), S3 {-70,0,90}, E 20 deg (+park 45, +145), W1 {-80,0,80}, W2 {-70,0,80}, W3 {-20,0,30}: 170,100 poses. Contact 114,760; reach-free from park 55,340 (32.5 %); frontier 35,073; deep (dropped) 79,687.

## 2. First-contact frontier cells by moving group
(cell counted for a group if any non-designed pair of that group is in contact there; the same cell can count for several groups)
| group | frontier cells | main obstacles (pair-cells) |
|---|---|---|
| M1 | 0 | none (M1|M3 3 cells counted under M3) |
| M2 | 3636 | skin 7 |
| M3 | 17311 | skin 30, MJ11 clasp bracket 6, head 8, M1 3 |
| ME | 3930 | head 133, skin 84, clasp bracket 12 |
| MW1 | 6983 | skin 683, head 685, MJ11 parts 724, stage/rail/bumpers 74, wing 63 |
| MW2 | 11360 | skin 2797, head 2416, MJ11 parts 2030, stage/rail/bumpers 724, wing 375 |
| MW3 | 16005 | skin 11638, head 8803, MJ11 parts 3872, stage/rail/bumpers 2418, wing 1146 |
Per pair: `mj11_whole_report.json` -> `frontier_by_pair`. The hand/wrist groups dominate as before (box proxy).

## 3. Shoulder level (M1/M2/M3, S1' x S2 x S3)
The 5 deg cross-check with S3 {-70,0,90} (`mj11_shoulder5.py`) reproduces the adopted numbers exactly: reach-free 0.786, passive-stop box **S1' -95..5, S2 -15..17.4, coverage 0.466**, and the box is the same for every S3. (The 10 deg whole-arm grid gives -95..-5 / -10..17.4, cov 0.343: a grid artefact; use the 5 deg result.)
What ends the box beyond the hard-stop ring: S1' hi -> **M3 vs head** (60 mm gate, at S1' 15); S2 hi -> **M3 vs head** (S2 25); S2 lo -> **M1 vs M3** (yoke group vs upper arm, S2 -25 at S3 -70); S1' lo = soft min -95.
Exact check of the +-5 deg hard-stop boundary (1 deg steps, S3 {-70,0,90}, `mj11_hardstop_check.json`, designed-contact pairs dropped): M3|head min 64.5 mm (margin 4.5) at S1' 10 / S2 22.4; M1|M3 3.88 mm (margin 0.9) at S2 -20; M1|seat dish 6.5 mm; TPU bumper reaches the 3 mm gate at S1' -98.5 and is 1.6 mm at the -100 hard stop (it is the stop). **Passive-stop box holds with the hard stops.**
Shoulder-level first contacts outside the box (keep-out by limits / tables; sub-grid first-contact angles, 5 deg):
- M2 vs skin: S2 40.7..48 for S1' -65..-15 (S3 all) -> inside the S2 soft max 88, so soft S2 cap needed only if the box is not enforced mechanically; box S2 hi 17.4 already excludes it.
- M3 vs skin: S2 32.9..44.0, S1' -95..-10 (example S1' -71.4 / S2 40, S3 -70).
- M3 vs MJ11 clasp bracket: S1' -6.9..5, S2 28.2..40 (example S1' -2.9 / S2 30, S3 90).
- M3 vs head (60 mm gate, d_min 43.7 mm on the frontier): S1' 9.4..35, S2 -1.2..25 (example S1' 31.5 / S2 0 / S3 90). With the box S1' max 5 this is excluded mechanically.
- M1 vs M3: S2 -21.9 at S3 -70 -> covered by the box S2 lo -15.
Table form for firmware (if the box is not relied on): S1' soft max 5, S2 soft range -15..17.4 (the passive box), i.e. every shoulder contact above is outside the box + hard stops. The existing soft limits (S1' -95..35, S2 -30..88) must be narrowed accordingly **(Forest's decision)**.

## 4. Elbow / wrist tier tables (proposal, not wired)
`elbow_tiers_mj11.json`, `firmware_preview/elbow_limits_mj11.h` (same format as `elbow_limits.h`, axes now S1' and S2, 15 x 14, EL_F and EL_R). Definitions as `control.h`: FULL any S3/wrist; RESTRICT |S3|<=30, |W2|<=30, |W3|<=15; E_max = largest E (floor 2.5 deg, linear zero crossing of d-MM between grid E values; contiguous from E 0) with every pose and wrist combination in the tier contact-free over all non-designed pairs. NEUTRAL (E<=5, |S3|,|W2|,|W3|<=10): on this grid it is the E=0, S3 0, W2 0, W3 0 slice and **is contact-free at every shoulder-reach-free cell** (the always-allowed invariant holds).
Within the shoulder reach-free region (S3 = 0): FULL is 0..80 deg (0 at park cell; 0-20 for S1' > -35), RESTRICT 45..132 deg (122 at park cell; 20..45 for S1' >= 15 and S2 <= -10). Figure: left panel contact causes, middle FULL, right RESTRICT.
Caveats (state once): coarse 10 deg grid with RESTRICT/NEUTRAL wrist values collapsing to {0}; E_max interpolates d linearly (not certified; use `collision_screw.py` sphere tracing before release); gates are 3/2/60 mm, the old tables used 15 mm to body/harness; hand = +-60 mm box. The FULL tier collapses at the park region mainly because of W2 -70/80 with the box hand.

## 5. Firmware tables: what was and was not done
Done: elbow tier tables as above (preview header, firmware headers not touched; firmware lives in the Project, not the repo). NOT done:
- `keepout_table.h` (straight-arm clearance): `gen_tables.py` reads `stl/shoulder/keepout_map.npz`, made by `keepout_map.py` on the old S1/S2 kinematics and capsule; not rerun for S1' / MJ11 (input missing).
- `balancer_keepout.h`, `grav_model.h`: inputs `stl/balancer/balancer_elbow_keepout.json`, `s2_massmodel.json`, `balancer_calc.json`, `verify_balancer.json`, `s2_grid.npz`, `stl/distal/distal.json` are not in the repo and the MJ11 balancer / torque-gate decision is open (Forest).
- `control.h` axis semantics (S1 -> S1') and `gen_tables.py` itself need updating to take the new tier JSON; Forest's decision before wiring.
