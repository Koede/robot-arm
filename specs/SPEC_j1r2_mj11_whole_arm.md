# MJ11 whole-arm first-contact sweep (2026-11-03)

Subsystem: CONTROL / SHOULDER integration. Geometry: MJ11 blade-seated pancake (pad y -60, z -190, standoff 12, `specs/SPEC_j1r2_blade_seat.md`), real MJ11 fixed parts (pad, seat dish, liner, rail spacers, clasp bracket at CLASP_PSI 120), rail-only wing, S1 stage parts, wearer (skin 2 mm gate, head/neck 60 mm). Moving groups M1, M2, M3, ME, MW1, MW2, MW3 (HAND rev B proxies as `j1r2_kin.proxies()`: hand envelope box x +-27.5, y D +-60, z 491.6..624, deliberately conservative). Method: `specs/SPEC_collision_method.md` (first-contact frontier only, deep cells dropped, designed contact at park excluded; gates 3 mm part-part and to the harness/stage, 2 mm skin, 60 mm head).

Scripts: `cad/j1r2/mj11_whole.py` (sweep), `mj11_shoulder_s3.py` (5 deg shoulder-level run incl. S3), `mj11_whole_report.py`, `mj11_tiers.py` (tier tables, keep-out map, candidate headers), `mj11_tier_plot.py`. Results: `stl/j1r2_sweep/` (`sweep_mj11_whole.json`, `tables_mj11_whole.npz`, `mj11_whole_report.json`, `mj11_tiers.json`, `mj11_tiers.png`, `sweep_mj11_shoulder_s3.json`, `mj11_shoulder_s3_summary.json`, `firmware_candidate/`).

## 1. Whole-arm sweep (soft range, 534 s, 2 cores)
Grid: S1' and S2 10 deg (park 17.4 / -18.2 inserted), S3 {-70,0,90}, E 20 deg (+ park 45, 145), W1 {-80,0,80}, W2 {-70,0,80}, W3 {-20,0,30}: 15 x 14 x 3 x 10 x 3 x 3 x 3 = 170,100 poses. Park is free.

| | cells | share |
|---|---|---|
| any contact | 114,760 | 67.5 % |
| reachable contact-free from park | 55,340 | 32.5 % |
| first-contact frontier (reported) | 35,073 | 20.6 % |
| deep (dropped) | 79,687 | 46.8 % |

Compare J1 rev 2 (2026-11-02): 57.4 % reach-free. MJ11 is lower because the hand box, the clasp bracket and the blade-seated pad (skin / head now closer to the arm) all gate the distal groups; the shoulder-level result is unchanged (section 3).

First-contact frontier cells per moving group (cells where that group, as downstream group of a non-designed pair, is in contact next to the reachable region; a pose can be counted for several groups): M1 0, M2 3,636, M3 17,311, ME 3,930, MW1 6,983, MW2 11,360, MW3 16,005. Per-pair status in `mj11_whole_report.json`. Pairs with a frontier (FAIL <1 mm = frontier contact reached at a grid cell, BELOW = 1..gate):
- M2: skin. M3: skin, clasp bracket, head (BELOW, first contact at the 60 mm gate). ME: skin, head, clasp bracket. MW1: rail, S1_AK70 (2.7 mm, BELOW), pad, dish, liner, clasp bracket, wing, skin, head. MW2 and MW3: every fixed part and the wearer (rail, AK70, stop posts, bumpers, pad, dish, liner, spacers, clasp bracket, wing, skin, head). Group pairs: M1-M3, M1-MW1, M1-MW2, M1-MW3, M2-MW3.
- Clean (PASS everywhere): M1 against every fixed part and the wearer; M2 / M3 against rail, AK70, stop posts, bumpers, pad, dish, liner, spacers (M2 vs wing and rail, M3 vs wing only touch in the dropped region); ME vs rail, stage, pad, dish, liner, spacers, wing; M2 vs ME / MW1 / MW2; M3 vs MW1..MW3; ME vs MW2/MW3; MW1 vs MW3.
- The hand-level counts are dominated by the +-60 mm hand box (SPEC_j1r2_integration section 5 saw the same). They are limits for the elbow / wrist tiers, not geometry changes.

## 2. Shoulder-level contacts to keep out (limits / tables)
From the 5 deg run (S1' x S2 x S3 {-70,0,90}; all contacts are independent of S3 in this geometry). First-contact curves (sub-grid), S2 at which the contact starts for a given S1':
1. **M1 vs M3 (yoke vs upper arm), 1.5 mm at the 3 mm gate: S2 = -21.9 deg at every S1' and S3.** The passive box floor (S2 -15, hard stop -20) clears it by 1.9 deg. Soft minimum S2 -15 (box) keeps it out; the global soft minimum -30 must not be used.
2. **M3 vs clasp bracket (psi 120): S1' 5 -> S2 28.2; 0 -> 28.4; -2.9 -> 30.0; -5.0 -> 32.3; -6.5 -> 35.0; -6.9 -> 40.0.** Nothing for S1' < -7. Keep-out: S2 <= 26 for S1' >= -5.
3. **M3 vs head/neck, 60 mm gate: S1' 9.5 -> S2 25.0; 12.4 -> 20.0; 13.4 -> 17.4 (park S2); 15.0 -> 14.3; 17.8 -> 10.0; 22.7 -> 5.0; 25 -> 3.3; 30 -> 0.6; 35 -> -1.2.** S1' soft max 5 (box) keeps it out; at the hard-stop corner (S1' 10, S2 22.4) the frontier is 1.8 deg away (S2 24.2).
4. **M3 vs skin (2 mm): S1' -95 -> S2 32.9; -90 -> 33.9; -85 -> 34.9; -80 -> 38.4; -75 -> 39.1; -70 -> 41.4; -65 -> 43.4; -60..-10 -> 43.3..44.0.** Keep-out S2 <= 31 for S1' <= -85.
5. **M2 vs skin: S1' -65 -> S2 44.7; -30 -> 44.3; -18.2 -> 42.2; -15 -> 40.7; -10 -> 42.3.** Inside the M3 skin curve everywhere; no extra limit if 4 holds.
Points 2 to 5 lie outside the passive box, so they are software keep-out only (tables), not stops.

**Passive-stop box (5 deg, S3 {-70,0,90}, hard-stop expansion free): S1' -95..5, S2 -15..17.4, arm-direction coverage 0.466; reach-free coverage 0.786.** Identical to the 2026-11-02 shoulder-level result (`mj11_scan`). Not tested: the hard stop at S1' -100 (grid ends at the soft limit -95; range 'hard' not run).

## 3. Elbow / wrist tier tables (FULL / RESTRICT / NEUTRAL), candidate
`mj11_tiers.py` re-applies the gates to the raw distance tables. Gate convention `fw15` = legacy `elbow_limits_15mm`: arm groups M3..MW3 keep 15 mm to skin and harness parts (pad, dish, liner, spacers, clasp bracket, wing), 60 mm head/neck, 3 mm stage and part-part; M1 / M2 keep the sweep gates (they sit within 15 mm of the wing / pad by design). `_sweepgate` files use 2 / 3 / 60 mm. E_max(S1', S2) = largest grid E with every tier pose at E' <= E clear; -1 = not allowed. Tiers: FULL any S3 / wrist; RESTRICT |S3| <= 30, |W2| <= 30, |W3| <= 15 (the grid only has 0 in those ranges, W1 any); NEUTRAL E <= 5, |S3|, |W2|, |W3| <= 10 (grid: E 0, S3 = W2 = W3 = 0, W1 any). Tables in `mj11_tiers.json`, plot `mj11_tiers.png`, legacy-schema JSON `elbow_limits_15mm_mj11[_restricted].json`.

Park cell (S1' -18.2, S2 17.4), fw15: **FULL 0 deg, RESTRICT 120, NEUTRAL 0.** Over the passive box with its 5 deg expansion (72 grid cells): RESTRICT 60..120, never closed; NEUTRAL 0 everywhere (always allowed, as control.h requires); FULL closed in 9 cells, otherwise 0..80.

Consequences:
- FULL is nearly closed in the whole box (E_max 0..20 at S1' >= -35, park E is 45): any S3 / wrist, including S3 +-70..90 with W2 -70 / 80 and the hand box, hits skin / clasp bracket / head. RESTRICT carries normal operation. A tighter hand envelope (hand rev B mesh instead of the +-60 mm box) is the first lever, not geometry.
- Deep elbow (E >= 140) with S3 near 85 to 90 reaches the clasp bracket (ME), and the hand hits the yoke M1 from E ~100 at S3 90.
- NEUTRAL E <= 5 is sampled at E 0 only; E 5 is not on the grid.
- The RESTRICT tier is sampled at S3 / W2 / W3 = 0 only, so the +-30 / +-30 / +-15 bands are not verified between grid points (the legacy tables sampled finer).

## 4. Firmware tables
`firmware/gen_tables.py` fetched from the Project docs (`third_shoulder_project_knowledge/firmware/gen_tables.py`). It cannot run on MJ11 as is:
- `keepout_table.h` / `elbow_limits.h` read `stl/shoulder/keepout_map.npz`, `stl/elbow_limits_15mm(.json|_restricted.json)` and `stl/sweep_full.json`, all written by the legacy scripts (`keepout_map.py`, `sweep_full.py` on `arm_kin` + pauldron + balancer). Those scripts model J1 rev 0 geometry, not MJ11. Replaced by the candidate headers below.
- `balancer_keepout.h` and `grav_model.h` read `stl/balancer/balancer_elbow_keepout.json`, `s2_massmodel.json`, `verify_balancer.json`, `balancer_calc.json`, `s2_grid.npz`, `stl/distal/distal.json` (legacy, rev e). MJ11 has no balancer (balancer "no home") and S1' now carries gravity (SPEC_j1r2_integration section 3). **Not regenerated.**
- `control.h` / `protocols.h` index the tables by legacy S1 and a sign convention that changed with S1'. The candidate headers use S1' / S2 axes and are **not drop-in**: `stl/j1r2_sweep/firmware_candidate/elbow_limits_mj11.h` (EL_F, EL_R, 15 x 14) and `keepout_table_mj11.h` (straight-arm min distance to skin / head, mm, 15 x 14, 10 deg grid vs the legacy 75 x 70). Firmware itself lives only in the Project, not in this repo, so no firmware file was changed.

## 5. For Forest
1. Adopt the shoulder keep-outs of section 2 (S2 soft min -15; S1' soft max 5; S2 <= 26 for S1' >= -5; S2 <= 31 for S1' <= -85) into the limits / tables? Geometry-neutral but changes firmware limits.
2. FULL tier is effectively closed at park; accept RESTRICT as the normal tier, or I refine the hand envelope and rerun (~9 min).
3. The clasp bracket at psi 120 is hit by M3 (S1' >= -7, S2 >= 28) and by the elbow at deep flexion; moving psi would change the bracket geometry (not done).
4. Firmware headers: needs a control.h / protocols.h port to S1' before the candidate tables can replace the legacy ones.
