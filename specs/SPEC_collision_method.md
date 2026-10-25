# SPEC_collision_method: how collisions are checked from now on (Forest's request, 2026-10-22)
Replaces the brute-force pose-grid sweep (sweep_full.py style: every pose x every pair) for the arm's self / fixed-structure / wearer checks. The grid script `cad/collision_frontier.py` stays as the validation reference.

## Principle
Only the **first contact** (the interaction point) matters. A large range of joint angle beyond the first contact is collision the arm cannot reach without crossing it, and the passive stops sit before it. Report and keep out only the frontier.

## Setup (in order)
1. **Screw-theory kinematics.** Each joint j is a unit twist xi_j = (w_j, v_j), v_j = -w_j x q_j; rotation by theta is exp([xi_j] theta) (Rodrigues). T(q) = exp([xi_1] q_1) ... exp([xi_7] q_7) M. Joint signs and axes as in DESIGN_STATE section 2.
2. **Separability (exact).** The relative pose of groups A and B is the product of exponentials over only the joints BETWEEN them; upstream joints cancel. Each non-adjacent group pair is tabulated only over those joints (1-D or 2-D). Pairs with the fixed frame F, the wearer or the harness depend on every upstream joint and use step 3.
3. **Sphere tracing in joint space (certified).** For a pair, a point at perpendicular distance r_j from axis j moves at most r_j |dq_j|, so the distance d changes by at most sum_j r_j |dq_j| over the joints between the pair. Starting from the park pose along each ray, step by dq = (d - MM) / sum_j r_j (MM = 3 mm gate, 2 mm to skin, head 60 mm) until d <= MM. 5-10 distance queries per ray, no skipped contact. r_j is the max perpendicular distance of the downstream group's vertices from axis j (the firmware lever table). Distances come from the existing SDFs.
4. **Frontier surface.** The level set d(q) = MM in joint space. Marching cubes (or marching squares per slice) on the tabulated distances gives first-contact angles between grid points. Pose classes: contact = d < MM; reachable-free = free component containing the park pose; **frontier = contact poses next to a reachable free pose (kept)**; deep = contact poses with no reachable free neighbour (dropped).
5. **Pair exclusion.** Group pairs closer than MM at the park pose (adjacent links, bearings, passive stops) are designed contact and are excluded (same rule as arm_keepout.legal_poses).
6. **Exact polish (optional).** For the final contact angles, vertex-face / edge-edge roots a cos(theta) + b sin(theta) = c per revolute joint (continuous collision detection, FCL screw motion).

## Validation (this session, `collision_frontier.py`, arm groups only, wrist neutral)
Grid S1 x S2 x S3 x E at 15 / 15 / 30 / 20 deg steps, soft range: 3456 poses, 262 s. Contact 77 poses (2.2 %), reachable free 3379, deep 0, frontier 77. Pairs: M1-MW2 63, M1-MW3 9, F-MW3 1, MW2-F 2, MW3-F 2. **The deep filter removed nothing here**: the arm-group contact set is already all boundary. The earlier "66 poses touch the balancer" figures involve the wearer / harness / balancer meshes, which are not in this workspace (wearer mesh stage not rebuilt); run the filter on those once rebuilt. Not yet written: `collision_screw.py` (steps 2-4); it must reproduce this table from far fewer queries before it replaces the grid.

## Paste-ready DESIGN_STATE entries
- Decision (Forest request, 2026-10-22): collision gate reports and keeps out only the first-contact frontier, computed by separable pair tables + sphere tracing (certified step d / sum r_j |dq_j|) + marching-cubes frontier; deep (unreachable) contact poses are dropped. Reference grid: cad/collision_frontier.py. Spec: specs/SPEC_collision_method.md.
- Changelog 2026-10-22: collision_frontier.py added; 3456-pose arm-group grid gives 77 contact poses, all frontier; wearer / harness / balancer not included (meshes missing in workspace).

## 6. Validation results (2026-10-22, session continuation)

Reference grid (`collision_frontier.py`, S1 x S2 x S3 x E = 9 x 8 x 6 x 8 = 3456 poses, wrist neutral, MM 3 mm, 42 ordered non-adjacent pairs, arm groups only).
**Geometry note:** the rebuilt shoulder cowl and head dome (M1 / M2) are now part of the groups. The first reference (no cowl, 77 contact poses) is kept as `collision_frontier_nocowl.json`. With the cowl: **178 contact poses, 164 frontier, 14 deep (dropped)**; by pair F-ME 4/8, F-MW1 22/27, F-MW2 22/26, F-MW3 44/45, M1-MW2 63/63, M1-MW3 9/9 (frontier / all contact).

| Implementation | Queries | Wall time (2 cores) | Reproduces reference |
|---|---|---|---|
| Grid (`collision_frontier.py`) | 145,152 | 359 s | reference |
| Separable pair tables (`collision_separable.py`) | 32,262 (4.5x fewer) | 117 s | contact mask identical, max |dD| 0.0 mm |
| Certified Lipschitz adaptive refinement per pair (`collision_screw.py`) | 64,474 (2.25x fewer) | 310 s | contact mask identical (0 mismatching cells); 360 frontier edges |

* Screw-theory kinematics (product of exponentials, twists in `collision_screw.py`) match `arm_kin.frames` to 1.7e-13 over 300 random soft-range poses.
* The Lipschitz radii r_j (sampled over 300 poses, padded 15 %) were checked against 1-joint perturbations: 0 violations.
* **Finding:** at this grid (4 joints, 15 / 15 / 30 / 20 degree steps) plain separable tables are the cheapest exact method; the adaptive bound refinement costs MORE queries than separable because the Lipschitz radii (up to ~350 mm) are loose against 15 degree cells. Sphere tracing / refinement pays only where the grid is too fine or too high-dimensional to tabulate (finer than ~2 degrees, or all 7 joints). Use the separable tables as the default implementation; keep `collision_screw.py` as the certified fallback for fine or 7-D runs.
* A pair table is correct only when the pair really depends on the joints between the two groups: with the wrist at neutral, pairs against F depend on S1..E (4 joints), neighbours on 1 to 2.

## 7. Arm versus wearer and harness (`collision_body.py`)

Wearer skin gate 2 mm, head/neck region gate 60 mm (wearer faces above the neck-joint height), harness (clasp frame, chassis plate, wing, spine bridge, pauldron) gate 3 mm; 3 mm voxel SDFs (resolution error up to ~1.5 mm). Grid S1 x S2 x S3 x E x W2 {-70, 0, 80} = 10,368 poses, W1 = W3 = 0, G 50; balancer parts included in a second run (`collision_body_bal.json`).

* 1099 contact poses; **802 frontier (73 %)**, **297 deep (dropped)**. Adding the balancer parts changes nothing (the arm limits first).
* Frontier by body: head 526 poses (MW3 263, ME 106, M3 75, MW1 63, MW2 19), harness 221 (MW3 195, MW2 8, MW1 18), skin 55 (MW3 53, MW2 1, MW1 1).
* Worst first-contact (frontier) poses are the head keep-out entries at S1 10 / S2 30-45 / S3 50 / E 100-120 (claw 29-30 mm from the head, gate 60 mm); these are the cells the firmware keep-out table must exclude.
* Designed-contact exclusions at park: none. Park distances: M1 to harness 5.4 mm (nearest; above the 3 mm gate), M2 to harness 35.9 mm, M1 to skin 88 mm.
* Caveats: wrist only varied in W2; W1 and W3 neutral; the firmware keep-out and the existing `sweep_full.py` tables were not re-run here (they need the 8-minute sweeps, queued in the local job package).
