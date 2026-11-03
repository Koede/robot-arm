# MJ11 whole-arm first-contact sweep (2026-11-03) — CONTROL / SHOULDER

Scene: real MJ11 fixed parts (pad -60/-190/12, seat dish, liner, rail spacers, clasp bracket at CLASP_PSI 120), rail-only wing, S1 AK70 + stop posts + TPU bumpers, rail, wearer incl. head/neck; moving groups M1, M2, M3, ME, MW1, MW2, MW3 (HAND rev B conservative envelope as MW3, `j1r2_kin.Kin.proxies`). Method: `specs/SPEC_collision_method.md` (separable pair tables, first-contact frontier only, deep contact dropped). Gates: 3 mm parts/harness, 2 mm skin, 60 mm head/neck; designed contact at park excluded.
Scripts: `cad/j1r2/mj11_whole_arm.py` (sweep; `--restricted` for the RESTRICT box), `mj11_shoulder_ko.py` (shoulder-level S3 triple at 5 deg + straight-arm map), `mj11_tiers.py` (post-processing), `gen_tables_j1r2.py` (candidate header). Outputs in `stl/j1r2_sweep/` (`sweep_/tables_mj11_*`, `mj11_tiers.json`, `keepout_map_mj11.npz`, `firmware_candidate/elbow_limits_j1r2.h`, logs).

## 1. Full sweep (steps as j1r2_integrate full mode; 170,100 poses, soft range, 517 s on 2 cores)
Grid S1' 15 x S2 14 (10 deg, park -18.2 / 17.4 added) x S3 {-70,0,90} x E 10 (20 deg + 145 + park 45) x W1 {-80,0,80} x W2 {-70,0,80} x W3 {-20,0,30}.
Park is free. 114,760 poses touch something, 55,340 are reachable contact-free from park, **35,073 are first-contact frontier**, 79,687 deep (dropped).

First-contact frontier cells per moving group (union over the group's pairs; a group-group pair is counted for the downstream group, so the M1|M3 contacts appear under M3):

| group | frontier cells | pairs with frontier |
|---|---|---|
| M1 | 0 (its only contact, M1\|M3, is under M3) | 0 |
| M2 | 3,636 | 1 (wearer skin) |
| M3 | 17,311 | 4 (skin, head, clasp bracket, M1) |
| ME | 3,930 | 3 (skin, head, clasp bracket) |
| MW1 | 6,983 | 10 |
| MW2 | 11,360 | 13 |
| MW3 | 16,005 | 14 |

(Cells are 7-D grid cells; they overlap across groups, so the column does not sum to 35,073.) Distal frontier contacts are dominated by the conservative +-60 mm hand envelope (MW3/MW2 vs skin 11,638 / 2,797 cells; vs head 8,803 / 2,416). Frontier cells sit at d = 0 on the grid (the free neighbour is >= MM); the sub-grid first-contact angles are in `sweep_mj11_whole_arm.json` (`first_contact`).

## 2. Shoulder-level first contacts (finer, 5 deg x S3 {-70,0,90}; `sweep_mj11_shoulder_s3.json`)
Reach-free 0.786, **passive-stop box S1' -95..5, S2 -15..17.4, coverage 0.466** (identical to the S3 = 0 result; +-5 deg hard-stop expansion free). The 10 deg whole-arm grid gives a smaller box (S1' -95..-5, S2 -10..17.4, cov 0.343) only because one grid cell of hard-stop margin is 10 deg there. Every shoulder-level frontier contact lies **outside** the box:

| pair | first contact (S1', S2) | min clearance in the reach-free region | outside the box by |
|---|---|---|---|
| M2 vs wearer skin (2 mm) | S2 40.7..44.7 for S1' -65..-10 | 2.35 mm | S2 >= 40.7 vs box 17.4 |
| M3 vs wearer skin (2 mm) | S2 32.9..44.0 for S1' -95..-10 | 2.78 mm | S2 >= 32.9 |
| M3 vs clasp bracket (3 mm) | S1' -6.9..5, S2 28.2..40 | 5.2 mm | S2 >= 28.2 |
| M3 vs head (60 mm gate) | S1' 9.4..35, S2 -1.2..25 | 62.0 mm (frontier cell at 43.7 mm, S1' 30 / S2 5) | S1' >= 9.4; the hard stop at +10 sits on the gate |
| M1 vs M3 (3 mm) | S2 -21.9 (all S1', S3 -70 worst, 1.5 mm at S2 -25) | 3.88 mm | box S2 min -15, hard stop -20: only 1.9 deg of margin at S3 -70 |

Software keep-out to carry (soft limits are wider than the box: S1' -95..35, S2 -30..88): S2 <= ~28 (clasp) / ~32 (skin) for the S1' range, S1' <= ~9 (head), S2 >= -21 at S3 -70. These need limit/table decisions (Forest); nothing was changed in `j1r2_params.SOFT`. The head gate at the S1' +10 hard stop and the 1.9 deg M1|M3 margin are the two thin spots.

## 3. Elbow / wrist tier tables (S1' x S2, 15 x 14 nodes; `mj11_tiers.json`)
Rule as firmware/gen_tables.py: E_max = largest grid E with every lower E also free for all combinations in the tier box; -1 = tier not allowed. EL_F: all S3/W1/W2/W3 grid combinations. EL_R: dedicated sweep with S3 {-30,0,30}, W2 {-30,0,30}, W3 {-15,0,15}, W1 {-80,0,80} (`sweep_mj11_whole_arm_restricted.json`, 522 s). NEUTRAL (E 0, S3/W2/W3 0): free on every node inside the passive-stop box (S1' -95..-5 rows, S2 -10..17.4) and on the rest of the soft box except S2 -30 and S2 >= 40-50; `NEUTRAL_ok` in the JSON.
- At park (S1' -18.2, S2 17.4): **EL_F = 0**, **EL_R = 45** — the park elbow (45) is only allowed in the RESTRICT tier (and in the NEUTRAL-adjacent set up to E 5); a FULL-tier wrist (hand envelope vs rail/pad/clasp/skin) collides at any E at park.
- EL_R inside the passive box is 45..100 (S1' -95..-55: 60..100; S1' -35..-15: 45..60); the legacy table was 90..145. The larger envelope (hand +-60 mm box), 2/3 mm gates and the MJ11 pad/clasp bracket account for it. The coarse RESTRICT cross-check (S3/W2/W3 = 0 only, from the full grid) is in the JSON as `EL_R_coarse_from_full_grid` and is 20..60 deg more permissive; use the dedicated sweep.
- 1 node (S1' -95, S2 30) had EL_F > EL_R (-1): EL_F clamped to EL_R.

## 4. Firmware tables — what was and was not regenerated
Done: candidate `stl/j1r2_sweep/firmware_candidate/elbow_limits_j1r2.h` (EL_F, EL_R, NEUTRAL_OK on S1' x S2 axes). **Not written into the firmware tree** (headers live in the Project doc store, not this repo) because:
1. `control.h`, PARK, soft limits and `tab4` use the legacy S1 axis (park S1 45); they must be ported to S1' (S1' now carries gravity) before the candidate can replace `elbow_limits.h`.
2. `keepout_table.h` needs `keepout_map.npz` (mesh-based straight-arm clearance, legacy `cad/keepout_map.py` on the old scene). A new S1' x S2 straight-arm map on a 2.5 deg grid is in `keepout_map_mj11.npz` (C_skin, C_head, C_fixed_margin, C_any_margin); the KO_MM threshold semantics in `control.h` (and a capsule check with the claw tip) have to be agreed before a header is cut.
3. `balancer_keepout.h` / `grav_model.h` need `balancer_elbow_keepout.json`, `s2_massmodel.json`, `balancer_calc.json`, `verify_balancer.json`, `distal.json`, `s2_grid.npz` for J1R2; the balancer packaging is still unresolved (SPEC_j1r2_integration section 6), so these were not regenerated.

## 5. Caveats
Grid is 10 deg (S1', S2, E 20 deg), so tier entries are 'free at every sampled node', not certified between nodes; the fine/certified fallback (`collision_screw.py`) was not run on the full 7-joint grid. HAND rev B is the conservative envelope box, not the ported wrist-v3 mesh. The wing is the rail strip only; harness_loads and the pad stall cap decision remain open.
