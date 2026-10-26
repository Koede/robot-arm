# Parallel spherical shoulder study (3-RRR, agile-eye type), 2026-10-26

Scope: could three AK70-10 on the chassis plate / pauldron drive the arm through a 3-RRR spherical parallel mechanism (all axes through P) instead of the serial S1 / S2 / S3 stack? Code: `cad/shoulder_parallel_kin.py` (kinematics, search, torque), `cad/shoulder_parallel_pack.py` (proxies and fcl collisions), `cad/shoulder_parallel_mass.py` (mass / inertia). Results: `stl/shoulder_parallel/` (`kin_result_*.json`, `pack_result_*.json`, `mass_inertia.json`, proxy STLs). Nothing existing was edited.

**Verdict: DROP for this design cycle (do not build, keep the serial layout).** Kinematic coverage is possible, but the geometry that is kinematically good does not package, and the geometry that packages is singular and over-torqued. Caveats in section 1.

## 1. Method
- Required set: R3 = Rz(-S1) Ry(-S2) Rz(S3) over S1 10..140, S2 -30..88, S3 -70..90 at step 10 (3094 orientations; matches `arm_kin.frames` to 1e-16).
- Mechanism: 3 legs, motor axis u_i (through P), proximal arc a1_i, distal arc a2_i, platform axis w_i fixed in the arm frame. Closed-form IK (8 branches), Jacobian omega = G theta_dot, G = A^-1 B (A rows v x w, B = diag((u x v).w)). A leg is feasible iff |a1-a2| <= angle(u,w) <= a1+a2; the violation in degrees is the coverage error. IK and Jacobian were checked against FK and finite differences.
- Free parameters: 3 motor axes, 3 platform axes, 6 arc angles (35..100 deg), assembly branch (21 variables). Differential evolution, 3 to 4.5 min per seed on 1 core, step-20 grid in the loop, report at step 10. Penalties: coverage, sign change of det A or b_i (singularity crossing), sigma_min(G) >= 0.3, sigma_max(G) <= 3, distal arcs >= 38 deg from the arm axis; for the "packaged" runs also motor axes in the lower-rear region (u_z <= -0.25, u_x >= -0.35) with >= 60 deg between motor axes.
- The search is limited (5 seeds, not exhaustive, the penalty weights changed between seed 2 and seeds 3-4: arm clearance weight 2 -> 6). A better optimum may exist. The findings are what this budget gave.

## 2. Kinematic results

| Design | Search | Coverage (step 10) | sigma_min(G) | sigma_max(G) | cond max | singularity crossing | motor travel per leg (deg) |
|---|---|---|---|---|---|---|---|
| A (seed 2, no packaging limits) | 180 s | 3094/3094, max error 0 deg | 0.21 | 3.16 | 11.8 | none in branch (+,+,-); the other 7 of the 8 branches cross one | 213 / 360 / 185 |
| B (seed 4, packaging limits) | 270 s | 3094/3094, max error 0 deg | 0.016 | 13.5 | 85 | det A changes sign inside the set | 316 / 167 / 347 |
| seeds 0, 1, 3 (packaging limits) | 180-270 s | full coverage, but sigma_max 6 to 2400, sign changes | | | | |

- Coverage is not the problem: the SPM reaches the full arm direction + roll set. Arcs used: A a1 72-105 deg, a2 80-99 deg; B a1 75-92, a2 82-84. Arcs over 90 deg are awkward curved links.
- Assembly modes: all 8 IK sign patterns give real solutions at every pose, but only one pattern (design A) stays on one side of det A over the set. A wrong assembly at power-up crosses a singularity, so homing and a hard stop on the right branch are needed.
- Motor travel: A leg 2 needs about a full turn (359.9 deg; at the wrap-detection limit, read it as "continuous rotation"). B legs 1 and 3 need 316 and 347 deg. The AK70 can turn freely, but the passive ball stops of the serial design have no equivalent: limits would be firmware or per-leg end stops.
- sigma_min(G) is platform rad per motor rad in the weakest direction: A 0.21 (5x reduction), median 0.68.

## 3. Torque transmission
Gravity moment about P over the balancer grid (324 arm configs: E 0..145, wrist W1/W2/W3, payload 0 / 0.5 kg, every S3, S1, S2 of the set) reproduces `balancer_calc.json`: serial S2 max 6.54 N m (file: 6.554), serial S3 (XM540) 2.91 N m, S1 0. Motor torque in the parallel mechanism: tau_m = G^T M.

| Case | max motor torque N m | SF vs 8.3 rated | SF vs 24.8 peak |
|---|---|---|---|
| Serial now, S2 AK70, no balancer | 6.55 | 1.27 | 3.8 |
| Serial now, S2 AK70, with balancer (residual) | 2.89 | 2.87 | 8.6 |
| Serial now, S3 XM540 | 2.91 | 3.6 vs stall 10.6 | |
| Parallel A, no balancer, worst of 3 motors | 10.0 | 0.83 | 2.47 |
| Parallel A, with spatial balancer | 8.16 | 1.02 | 3.0 |
| Parallel B, no balancer | 25.9 | 0.32 | 0.96 |
| Parallel B, with spatial balancer | 20.9 | 0.40 | 1.19 |

The load is shared by three motors, but the mechanism amplifies it (max 2.4x for A, 6.7x for B). The total moment needed (max 6.55 N m) is no larger than now, but the worst motor in A is above rated (SF < 1 against the project gate of 2). Only the balancer-assisted case reaches SF 1.0, still below the gate. Peak SF 2.5 to 3 is OK for short moves only; continuous holding at 8 to 10 N m on an AK70 is not sustainable thermally. Dynamic loads were not computed.

## 4. Packaging (python-fcl, serial set, links on nested shells R = 62 / 80 / 98 mm, motor flange 12 mm beyond each shell)
Proxies: 3 AK70 cylinders, 6 links (tube r 5), platform hub + 3 spokes + arm stub (r 36), upper arm (arm_kin M3 group). Written for A (`*_free_*`) and B (`*_s4_*`). Link tubes are thinner than real arcs, so collisions are underestimated.

Design A fails by inspection of the geometry: motor axes 1 and 3 are 33 deg apart and the two bodies overlap (distance 0); motor 1 sits at (63, 76, 0), inside the arm sweep; one platform axis is 9.6 deg from the arm axis, so a distal link runs along the arm tube. The upper arm hits motors 1 and 3 in 35 and 13 of 210 poses (step 30).

Design B (step 20, 504 poses; gates 3 mm part-part and harness, 2 mm skin, 60 mm head):
- Static: motor-to-plate distance 16 / 88 / 121 mm, so none touches the plate; each needs a standoff and the "loads go straight into the plate" claim holds for at most one. Motor 2 intersects the pauldron (0 mm); motors 1 and 2 are 14.7 mm apart.
- Platform / stub against motors: below gate in 314 poses (motor 1) and 45 (motor 2); against the plate 35 and the pauldron 184 poses.
- Links: proximal 2 and 3, distal 2 and 3 hit the pauldron (402 / 64 / 425 / 289 poses) and motors (up to 370 poses); distal 3 hits the plate (95 poses); link-link and link-platform contacts remain even with nested shells (prox1-dist2 100 poses, platform-prox1 154).
- Arm and wearer: skin min 66.6 mm (fine); head min 47.5 mm, 17 of 504 poses under the 60 mm gate. That belongs to the serial pose set (the S1 10 / S2 30-45 entries already in the firmware keep-out), not to the mechanism. The mechanism itself stays >= 75 mm from skin and >= 10.8 mm from the clasp.
- Not done: forearm / claw against motors (`--forearm E` option exists, not run), real link sections, bearing stack-up.
The pauldron and S1 base occupy the region these motors need (below P, z -53 to -121), so the pauldron would be redesigned with the harness loads (`harness_loads.py` is not in the repo).

## 5. Effect on cable routing
- Motors on the plate: their power and CAN cables never cross a joint. Today the S1, S2, S3 loops (L 250.6 / 298.8 / 176.4 mm; S3 marginal: 0.80 mm clearance, 7 bend hits) carry the S2 AK70, the S3 XM540 and the downstream bus.
- The downstream bus (12 V servo bus to E, W1, W2, W3, G) still has to cross from plate to arm, now as one loop spanning the full 3-DOF spherical motion with the 160 deg roll folded in, in a centre surrounded by link shells at R 62 to 98 mm. I did not run the elastic-rod wire search for this, so there is no pass / fail number. Expectation: as marginal as the S3 loop or worse.
- Balancer cord: next section.

## 6. Does the S2 balancer concept apply?
In the physics yes, in the present hardware no. A zero-length spring from a base point A on the +z axis above P to a point B on the arm axis gives M_b = K e x z for any S1 and S2 (a spherical version of the S2 balancer). On the same 324-config grid the optimum K = 3.73 N m (S2 balancer K 3.78) leaves a residual of 2.91 N m max, the same as today's 2.89; the residual comes from S3, E and wrist offsets, not from S1. Problems:
- A must sit above P on the base axis, where the arm passes near S2 = 0, while the base is below P: a mast, or a cord over a sheave from a base-mounted spring.
- Even with it the worst AK70 is at SF 1.0 (A) or 0.4 (B).
- The existing balancer parts (tube on M1, yoke, collars) are built around the serial S2 axis and would not carry over.

## 7. Mass and inertia (`mass_inertia.json`; point-mass parts, park pose; the balancer, about 0.23 kg, is excluded from the serial figures)
| | Serial now | Parallel (B proxies) |
|---|---|---|
| Shoulder stage moving mass | M1 939 g + M2 425 g | platform 271 g + 6 links 174 g |
| Moving mass about P including the arm (1042 g beyond) | 2406 g | about 1400 g (links at half weight) |
| Actuators on the arm | AK70 S2 621 g + XM540 S3 165 g | none |
| Actuators on the harness | AK70 S1 621 g | 3 x AK70 1863 g (+1242 g) |
| Inertia about the vertical axis | 0.054 kg m^2 | 0.053 |
| Inertia about the S2 axis | 0.116 | 0.114 |
| Roll inertia | 0.014 | 0.014 |

The arm beyond the shoulder (1.04 kg, 575 mm) dominates the inertia, so removing about 1 kg from the stage changes the inertia by < 2 %. It moves about 1 kg off the arm and adds 1.24 kg to the harness (before brackets). Motor-side reflected inertia is worse by 1/sigma^2 (2x to 25x at weak poses). Link masses are rough (solid tube volume x 1.2 g/cm3 + 15 g each).

## 8. Recommendation: DROP
1. The one kinematically clean geometry (A: sigma 0.21 to 3.2, no singularity in one branch) cannot be packaged: overlapping motors, a motor in the arm sweep, a link along the arm axis.
2. The geometry that fits the lower-rear region (B) crosses a singularity (sigma_max 13.5), needs 25.9 N m on one motor (above the 24.8 peak) and still collides with the pauldron, plate and its own motors.
3. Even A has no torque margin (SF 0.83 rated, 1.02 with a spatial balancer) against the gate of 2 and today's S2 SF 2.87.
4. No cable gain for the downstream bus, no inertia gain, +1.2 kg on the harness, no passive ball stops, and the balancer / cowl / stop work is lost; new singularity and assembly-mode safety issues.
5. The real benefit: about 1 kg less on the arm and no motor body on the arm.
If it is worth another try (Forest's call): a longer search with a fixed ring of motor axes about the S1 axis and nested shells, or a 2-DOF spherical mechanism for the arm direction with S3 kept as the XM540 roll, which removes the 160 deg roll demand from the parallel part.
