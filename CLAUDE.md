# Third Shoulder — project context for Claude Code

Third Shoulder is a wearable robot arm described as CAD-as-code. Read `README.md` for setup and the build commands. This file covers the conventions and the current state of the design.

## Conventions
- Units are millimetres and degrees. Robot frame. The right arm (`*_R.stl`) is the generated set; left-arm meshes are mirrors written by stages that build both sides.
- `cad/` holds the build and check scripts. `stl/` holds their meshes and JSON results. Never edit or delete files in `stl/` by hand; rerun the script that writes them.
- Do not pipe a script's output through `head`: that kills it before it writes its JSON.
- `cad/wearer_makehuman.py` reads the body from `/home/claude/external/makehuman_base/base.obj`. Symlink that path to `external/makehuman_base/base.obj` (see README).
- Order of rebuilds: `cd cad && python3 arm_wiring_lib.py --rebuild` (SDF cache), then `./harness_chain.sh` (wearer + harness, ~15 min), then `./run_local.sh` (wire searches + full sweeps, many cores).
- `DESIGN_STATE.md` is the design record. The current file is a reconstruction from the repo (2026-10-25); the original (about 40 KB) lives in the claude.ai project. If the original is added, it replaces the reconstruction. Keep it current: add dated changelog entries and update the open items when results change.

## Collision checking
Spec: `specs/SPEC_collision_method.md`. Report and keep out only the first-contact frontier; deep (unreachable) contact poses are dropped.
- Default implementation: separable pair tables (`cad/collision_separable.py`).
- Certified fallback for fine (< ~2 deg) or full 7-joint grids: Lipschitz refinement / sphere tracing (`cad/collision_screw.py`).
- Reference grid for validation: `cad/collision_frontier.py`. Any new method must reproduce its contact mask exactly.
- Gates: 3 mm between parts and to the harness, 2 mm to skin, 60 mm to the head. Pairs closer than the gate at park are designed contact and excluded.

## Open items (summary; full list in `DESIGN_STATE.md` section 12)
1. W2 / W3 cable-loop re-search with 30 combined-pose bases (`run_local.sh`). If it still fails: firmware limit W1 >= -30 deg when E < 60 deg, or a rubbing sleeve. Needs Forest's decision.
2. In-link runs, arm_chain DP, hub link, clips and boot sizes not redone for the new elbow anchors.
3. `sweep_full.py` and `--restricted` not rerun after the cowl / pauldron rebuild; the firmware keep-out tables (`gen_tables.py`) follow from them.
4. Harness top-pad margin vs boot mass: `harness_loads.py` needs `sensor_pods.py` + `electronics_fit.py`.
5. Pad stall cap proposal (AK70 S1/S2 cap 8.3 to 8.0 N.m). Needs Forest's decision.

Scripts, specs and data referenced but not in the repo are listed in `DESIGN_STATE.md` section 12, item 7.

6. HAND rev B is the project hand (2026-10-30, `specs/SPEC_hand.md`). Still to do: port it onto the wrist-v3 coupling (branch `claude/hand-coupling-2026-10-30`), re-run the sweeps and `harness_loads.py`.

## Working rules
- Design decisions that change geometry, limits or firmware tables are Forest's call. Lay out the options and numbers; don't pick silently.
- Record decisions and changelog entries in the design-state docs, dated.
