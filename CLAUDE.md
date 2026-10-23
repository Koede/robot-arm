# Third Shoulder — Claude Code handoff

Wearable back-mounted 7-DOF bionic arms (owner: Forest). Right arm, robot frame, origin P (shoulder centre),
+x away from back, +y wearer's right, +z up, mm/g/N·m/deg. Left arm = mirror in y.

## Read first
1. `specs/DESIGN_STATE_ADDENDUM_2026-10-22.md` — latest changelog + open items (DESIGN_STATE.md itself is in the claude.ai Project, not here).
2. `specs/SPEC_collision_method.md` — collision gate (first-contact frontier only; separable pair tables; screw-theory kinematics).
3. `CURSOR_INSTRUCTIONS.md` — setup + heavy local jobs (works unchanged for Claude Code). Note: "sphere tracing" there is superseded by separable tables.

## Setup
pip install --break-system-packages cadquery trimesh manifold3d scikit-image rtree python-fcl scipy fast-simplification
Run scripts from `cad/` (they use ../stl relative paths). SDF rebuild: `cd cad && python3 arm_wiring_lib.py --rebuild`.

## Standing decisions (do not reverse)
Passive stops only (hard stops 5° beyond soft limits); nothing rigid on moving-scapula skin; EMG electrodes over blade live in the
compression base layer; no backplate/hip belt; clasp ~270° with armpit anchor + front strap; MakeHuman CC0 body until measurements;
collision keep-out only at the first-contact frontier. Safety: tip speed ≤0.5 m/s, head/neck keep-out, 300 ms EMG/IMU dead-man,
E-stop, EMG front end galvanically isolated from the 24 V bus. Mark unverified vendor geometry PLACEHOLDER; never print it.
Gates: SF≥2 torque; ≥3 mm part clearance, ≥2 mm skin, 60 mm head; meshes watertight; verify before reporting.

## Open work (priority order)
1. `./run_local.sh` — W2/W3 cable-loop re-search (`wire_search_combined.py <J> 400 2 30`), `sweep_full`, `sweep_full --restricted`.
2. Re-run `harness_loads`, `body_loads`, `verify_balancer` after the cowl/pauldron rebuild (scripts missing here; restore from Project).
3. Decisions needed from Forest: pad-stall cap 8.0 N·m (gives 2.075× vs 2.00×); W1 ≥ −30° firmware limit (wait for re-search); S2 free-fall T12.
4. Cursor's curved `cad/pauldron.py` is authoritative — do not overwrite.
5. Write results back to `specs/` + changelog; commit on a branch; send Forest renders (renders/) and STLs.
