# DESIGN_STATE addendum, 2026-10-22 (paste into DESIGN_STATE.md section 11 and the changelog; DESIGN_STATE.md itself not edited: it is 40 KB and a retype risks corruption)

Subsystem: cross-subsystem integration (collision method, in-arm wiring, wearer / harness rebuild).

## Changelog
- 2026-10-22: Wearer (MakeHuman, --back_to_p 108), atlas, clasp, connector, chassis, scapular frame, cowl and pauldron regenerated; all meshes watertight (clasp 144.5 cm3, plate 125.3, wing 108.6, spine bridge 24.8, pauldron 158.3, cowl 91.8, head dome 17.4). The muscle_atlas.py copy in the project was re-typed from the previous read and reproduces emg_rules.json and every emg_atlas site/landmark to the printed precision.
- 2026-10-22: Collision method validated. `collision_frontier.py` (grid) re-run with the current cowl + head dome: 178 contact poses (was 77 without the cowl), 164 frontier, 14 deep. `collision_separable.py` reproduces it exactly with 32,262 queries (4.5x fewer, 117 s vs 359 s). `collision_screw.py` (POE kinematics, error 1.7e-13; certified Lipschitz refinement) reproduces it exactly but needs 64,474 queries: at this grid the separable tables are the cheapest. Standing text "sphere tracing" in the instructions should read "separable tables; Lipschitz refinement for fine / 7-D grids".
- 2026-10-22: `collision_body.py`: arm vs wearer skin (2 mm), head (60 mm) and harness (3 mm), 10,368 poses: 1099 contact, 802 frontier, 297 deep dropped; balancer parts change nothing. Frontier by body: head 526, harness 221, skin 55. Nearest approach at park: M1 to harness 5.4 mm.
- 2026-10-22: In-arm wiring: elbow loop replaced (L 191.6 mm, clearance 10.0 mm, bend 28.5 vs 23.9 mm needed). W2 (6/16 fresh poses fail, clearance down to -2.2 mm) and W3 (4/16 fail, 0.09 mm) still fail combined poses; S3 marginal.

## Open items (updates)
1. W2 / W3 loop re-search with 30 combined-pose bases (`wire_search_combined.py W2 400 2 30`, same for W3): packaged for a many-core machine (`CURSOR_INSTRUCTIONS.md`, `run_local.sh`). If it still fails: firmware limit W1 >= -30 deg when E < 60 deg (needs Forest's decision), or a rubbing sleeve.
2. In-link runs, arm_chain DP, hub link, clips and boot sizes not re-done for the new elbow anchors (E to W1 straight distance 118.9 mm vs the old 22.3 mm run).
3. `sweep_full.py` and `--restricted` not re-run after the cowl / pauldron rebuild (local package). The firmware keep-out tables (`gen_tables.py`) follow from them.
4. Harness top-pad margin (2.0 %) vs boot mass (~99 g per arm at 0.7x TPU wall): `harness_loads.py` needs `sensor_pods.py` + `electronics_fit.py`; not run this session.
5. Pad stall cap proposal (AK70 S1/S2 cap 8.3 to 8.0 N.m): pad stall limit 16.6 N.m gives 2.00x at 8.3 and 2.075x at 8.0; S2 torque SF with balancer stays 2.77 (need 2.89 N.m vs 8.0 cap bound) , needs Forest's decision. S2 free fall T12 (phase-short braking f >= 0.80) and the electronics fit follow-ups (relays, 1.10 s off-delay stage, main-box heat) are bench / parts items, not computable here.
6. GitHub push not done: `gh` reports the stored token as invalid and no repository is named.
