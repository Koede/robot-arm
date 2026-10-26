# Third Shoulder — DESIGN_STATE (reconstructed)

> **Reconstructed 2026-10-25** from the repository: script docstrings in `cad/`, result JSON in `stl/`, logs, `specs/SPEC_collision_method.md` and `docs/DESIGN_STATE_ADDENDUM_2026-10-22.md`. The original DESIGN_STATE.md (about 40 KB, kept in the claude.ai project) was not available. Section numbers follow the references in the addendum (section 2 = joints, section 4 = harness/sensing, section 11 = integration); other numbering is new. Decisions, rationale and history that only lived in the original are missing; anything not traceable to a file here is left out rather than guessed. If the original turns up, it wins: merge this file's section 11 and changelog into it and delete this one.

Wearable supernumerary robot arm ("third shoulder"), one per shoulder, described as CAD-as-code. Right arm is generated; left = mirror in y (`*_L.stl`). Units mm, deg, N, N m, g.

## 1. Architecture

7 DOF + gripper per arm: S1, S2, S3 (concurrent shoulder axes through P), E (elbow), W1 (forearm roll), W2, W3 (2-DOF wrist), G (claw).

| Joint | Actuator | Source |
|---|---|---|
| S1, S2 | CubeMars AK70-10 (dia 89 x 50.25, 621 g, 8.3 N m rated / 24.8 peak, 24/48 V) | `shoulder_stage.py`, vendor-verified |
| S3, E | Dynamixel XM540-W270 (33.5 x 58.5 x 44, 165 g, 10.6 N m stall @ 12 V) | `shoulder_stage.py`, `upper_arm_elbow.py` |
| W1, W2, W3, G | Feetech STS3215 (55 g) | `distal_arm.py`, `claw_hand.py`, `verify_distal.py` |

Standing design language: ball-and-socket styling at every joint, passive hard stops with TPU capsule facings, DOF and actuator count unchanged by restyling.

Load path: stage -> pauldron -> 8 mm 6061-T6 chassis plate -> scapular frame / spine bridge -> hip belt. The faceted clasp locates sEMG pads and does not carry the arm.

## 2. Frames, kinematics, joint limits

Robot frame (right arm): origin P = shoulder centre where S1/S2/S3 meet; +x rearward, +y lateral (wearer's right), +z up. Arm rests pointing up (+z) at q = 0. Positive S2 swings the arm forward (-x). Human elevation = 180 - S2.

Chain (`arm_kin.py`): R1 = Rz(-S1); R2 = R1 Ry(-S2); R3 = R2 Rz(S3); RE = R3 T(0,0,230) Ry(-E) T(0,0,-230); RW1 = RE T(0,D,0) Rz(W1) T(0,-D,0); RW2 = RW1 T(0,D,Zw) Ry(-W2) T(0,-D,-Zw); RW3 = RW2 T(0,D,Zw3) Rx(W3) T(0,-D,-Zw3).

Layout (`stl/distal/distal.json`): elbow axis at z 230; D = 57; Zw = 410; Zw3 = 455; claw tip Z_TIP = 575. Kinematic groups: F (fixed stage + harness), M1, M2, M3, ME, MW1, MW2, MW3 (hand = W3 frame). The product-of-exponentials form in `collision_screw.py` matches `arm_kin.frames` to 1.7e-13.

Joint limits (`rom_limits.py`, single source; hard = soft +- 5, except S1 max):

| Joint | Soft | Hard | Park | Meaning |
|---|---|---|---|---|
| S1 | 10..140 | 5..150 | 45 | arm-plane swing; medial limit from head keep-out |
| S2 | -30..88 | -35..93 | 25 | elevation from up-rest; back lean capped by head keep-out |
| S3 | -70..90 | -75..95 | 0 | humeral rotation |
| E | 0..145 | -5..150 | 45 | elbow flexion |
| W1 | -80..80 | -85..85 | 0 | pronation / supination |
| W2 | -70..80 | -75..85 | 0 | wrist ext / flex |
| W3 | -20..30 | -25..35 | 0 | radial / ulnar |
| G | 0..100 | -5..105 | 50 | gripper % |

Rev 2026-10-12c (I1): S1 soft min -15 -> +10, S2 soft min -55 -> -30 because the straight arm came within -10 mm of the MakeHuman head/neck. Inside the new box the straight-arm capsule (P -> 590 mm, r 32) clears the head by >= 64.9 mm, so the passive stops alone keep a stretched arm out of the 60 mm head keep-out.

Software elbow tiers (`stl/elbow_limits_15mm_restricted.json`, from `sweep_full.py --restricted`): largest E with >= 15 mm to body/harness and >= 60 mm to head, per (S1, S2); -1 = no safe E. Examples: S1 10 / S2 >= 71 -> -1; S1 140 / S2 -30 -> -1; most of S1 62..140 allows 120-145. Not yet regenerated after the cowl / pauldron rebuild (section 11).

## 3. Shoulder stage and housings

- Stage parts (`shoulder_stage.py`, `stl/shoulder/meta.json`): s1_base 204.8 cm3, s1_clamp 33.6, s1s2_yoke 121.5, s2_clamp 36.9, s2_adapter 79.7, s3_bracket 103.4, s3_lid 12.7.
- Shoulder stops (`ball_stops.py`, `stops/stops.json`): S1 lug under the yoke disc striking two posts at r 54.75 (contact 5 / 150); S2 pads at -35 / 92. TPU 95A capsule pads, ~35 % strain at AK70 peak (459 N at r 54). TPU thickness S1 11, S2+ 7.5, S2- 11 mm.
- Socket cowl + humeral-head dome (`shoulder_cowl.py`, rev 2026-10-13, Forest: "make the shoulders not giant spheres"): replaces the 188 mm / 146 g ball (`ball_shell.py`, retired). Cowl 149.7 x 65 x 110 mm, 91.9 cm3, 117 g PETG, wall p5 3.2 mm, slot gap 3.2 mm (< 4 mm, no fingertip entry, ISO 13857), split at x = 0. Dome R 43.8..46.4, 17.4 cm3, 22 g.
- Pauldron (`pauldron.py`, group F, rev 2026-10-13 faceted socket + strut web; rev 2026-10-23 curved deltoid shell): socket housing round S1, 3 struts to the plate rear face (2 x M4 each), wrap-arms to clasp nodes F4 (acromion) and F1 (upper trapezius). The lateral side is a swept arc, crown 12 mm off the deltoid, rim curled inboard, outer edge near y 85 instead of 98. Split for 256 mm build volume: core 99 g, arm_lat 4 g, arm_med 36 g (PA-CF), half-lap splices 2 x M4 x 16.

## 4. Harness and sensing

- Wearer: MakeHuman CC0 base mesh re-posed and placed in the robot frame (`wearer_makehuman.py --back_to_p 108`).
- Clasp (`clasp_shoulder.py` + `organic_connector.py`): faceted, flat pads tangent to the body carrying inward sEMG electrodes; single smooth-blended SDF frame, 144.5 cm3, watertight. Bridge feet F5 mid-trap, F1 upper-trap, F4 acromion, R1 scapula, seated at x -88. Pulled on by a 32 mm webbing strap.
- Chassis plate (`chassis_axilla.py`): 6061-T6, 154 x 114 x 8 mm, x -88..-80, 18 holes, min web 10.7 mm, 338 g. Pocketing study `plate_pocket.py` (-80 g target, Morley/P2 FE, limit loads 24.8 N m about each axis + 75 N).
- Axilla cradle + front strap: cradle at (-183, 7, -262); strap 32 mm, 10 N spec, path 169 mm; edge to axilla apex 27.1, plexus proxy 64.2, AC joint 100.4 mm; max width-averaged line pressure 6.8 kPa at spec tension.
- Scapular frame (`scapular_frame.py`): the wing contacts only the paraspinal strip and bridges over the blade, which slides 5-10 cm under the skin at full elevation. Wing 108.6 cm3 / 125 g CF-PA; spine bridge 24.8 cm3 / 28 g; rail gap 7.5-15.8 mm, bridge gap 14-27 mm.
- sEMG atlas (`muscle_atlas.py`, rev 2026-10-14): bipolar pairs, 12 mm discs, IED 20 mm, sites from tape-measure landmark rules ray-cast to the mesh (`emg_rules.json`, `emg_atlas.json`). Rigid clasp pads on stable zones (delt_ant F2, pec_clav F3, ...); others on a compression base layer. Scapular bone landmarks are estimated and flagged. SENIAM pages were unreachable; rules come from secondary sources (B18, B20, Cram, IZ literature) and four textbook-only rules are flagged.

## 5. Upper arm and elbow

`upper_arm_elbow.py` (rev 2026-10-13b elbow ball): XM540 at the elbow axis (0, 0, 230), shell halves ua_lat 81.8 g / ua_med 84.6 g PETG, fork_drive 29.0, fork_idler 30.9, fork_bridge 18.1 g. Elbow is a ball-styled hinge: sphere R_E 44 (inner 41.6), fixed belt |y| <= 24.7 on the upper arm, moving polar caps on the forks with a 1.0 mm running gap. Hand calcs in `calc_elbow.py`.

## 6. Distal arm and claw

- `distal_arm.py` (I6): forearm root, W1 roll (pinion + internal ring gear, 6808-2RS bearing), distal forearm, W2 yoke. `distal_stops.py`: rigid sector posts with TPU 95A facings placed so first contact is at the hard stop (`stops_distal.json`).
- Claw C3 (`claw_hand.py`, `claw_synth.py`, `claw_asm.py`, session 2026-10-13g): three identical double-jointed curl fingers 120 deg apart, one STS3215 on the hand axis turning a 3-start helical rotor, punch ring on three steel pins drives all fingers. Hand tray 64.4 cm3. Envelope nearly G-independent (z max 562.1 / 563.5 / 562.5 at G 0 / 50 / 100). Pin retention (rev 2026-10-15, Forest-approved): each dowel pressed in one member, running in the other.

## 7. S2 gravity balancer (I1b)

`balancer_calc.py` (single source) / `balancer.py`, rev e 2026-10-15 (parts rev f): zero-length compression spring + Dyneema cord over a swivelling sheave, tau_b = K sin(S2), K = 3.78 N m. A = (0, 50, 22) on M1, B = (0, 50, 47) on M2.
- Spring: music wire d 3.2, OD 36.9, 7.5 active coils, k 3.65 N/mm, free 87.7, solid 30.4.
- Over 53,460 poses: no-balancer max 6.55 N m; minimax residual with balancer 2.89 N m. Peak cancellation 57.6 % at E 0 / 0.5 kg, 40.6 % at E 90, -15.9 % at E 145.
- Free-fall energy to the + stop: 4.69 J loaded (8.1 J without the balancer).
- AK70 S2 bearing: max radial 194.5 N, tilting moment 11.67 N m.
- Parts (PA12-CF unless noted): anchor 100.2 g, tubecap 20.8, piston 9.4, collar 42.0, collar cap 10.5, TPU buffer 7.8, swivel 11.7 (7075), sheave 2.3 (6061).
- Restricted sweep: balancer min clearance 20.0 mm, 0 poses below 3 mm.

## 8. In-arm wiring and electronics

- Method (`arm_wiring_lib.py`, `arm_wire_search.py`, `arm_chain.py`, `arm_wiring_verify.py`, `wire_audit.py`): per-group SDFs (`stl/electronics/_sdf/`), elastic-rod cable loops between printed clamps. Pass: clearance >= 2.0 mm robust, 0.8-2.0 marginal (sleeve), < 0.8 fail; bend radius >= k x OD; stretch < 3 %. Kinematic envelope, not mechanics; bench test T-W1 to confirm.
- Arm hub H1 (`arm_hub.py`): 12 V servo-bus distribution in the upper-arm tube (TVS, 2 x 470 uF, two resettable fuses), no MCU; buses driven by a Teensy in the main box. CAN termination plug at S2.
- Current loop audit (`stl/electronics/wire_audit_v2.json`, 16 combined poses each):

| Joint | L mm | min clr mm | min bend / need mm | fails |
|---|---|---|---|---|
| S1 | 250.6 | 5.06 | 33.2 / 34.7 | 0 |
| S2 | 298.8 | 9.63 | 40.6 / 28.4 | 0 |
| S3 | 176.4 | 0.80 | 21.8 / 28.4 | 2 clr, 7 bend (marginal) |
| E | 191.6 | 10.0 | 28.5 / 23.9 | 0 (new loop, 2026-10-22) |
| W1 | 203.1 | 0.80 | 13.3 / 13.2 | 3 clr |
| W2 | 97.5 | -2.18 | 2.8 / 13.2 | 6 clr, 6 bend (FAIL) |
| W3 | 111.3 | 0.09 | 5.9 / 12.0 | 4 clr, 2 bend (FAIL) |

- In-link runs vs anchor chord: E -> W1 (ME) run 22.3 mm for a 118.9 mm chord (ratio 0.19) and W1 -> W2 0.41 are invalid after the new elbow anchors.
- Through-axis routing is blocked on most axes (`axis_corridor.py`, `axis_offset.py`, `w1_axis.json`), hence external loops + soft boots (`render_boots.py`).
- Segmented fairing study (`fairing.py`, T 9): failed to build (meshes not volumes); no result.

## 9. Placeholders / unverified (do not print or order until replaced)

AK70-10 output-flange bolt pattern; XM540 horn protrusion, horn PCD / screw count, rear idler boss; S3 horn plane height; claw SHAFT_OFF, horn disc, servo ear pattern, rotor-to-horn fixing, TPU pad mu, CLAW_ZERO / CLAW_SIGN; balancer sheave shop part, cord MBL / splice / bend efficiencies, AK70 S2 bearing centre; hub component sizes marked T in `arm_hub.py` / `arm_wire_spec.py`.

## 10. Verification gates

- `gates.py` records every verify/sweep result in `stl/gates.json` (not in the repo yet; `gen_state_table.py` builds a table from it, also not present).
- Gate targets: part-part >= 3 mm, skin >= 2 mm, head >= 60 mm (software), elbow tiers 15 mm, torque SF = rated / required >= 2, stops first-contact at the hard angle.
- Check scripts: `verify_stops.py`, `verify_distal.py`, `verify_balancer.py`, `sweep_shoulder.py`, `sweep_full.py`, `arm_wiring_verify.py`, `wire_audit.py`.

## 11. Cross-subsystem integration (2026-10-22)

From `docs/DESIGN_STATE_ADDENDUM_2026-10-22.md` and `specs/SPEC_collision_method.md`:

- Wearer, atlas, clasp, connector, chassis, scapular frame, cowl and pauldron regenerated, all watertight.
- **Collision method (decision, Forest request 2026-10-22):** report and keep out only the first-contact frontier; deep contact poses are dropped. Default: separable pair tables (`collision_separable.py`, 4.5x fewer queries, exact). Certified Lipschitz refinement (`collision_screw.py`) only for fine or 7-D grids. Reference: `collision_frontier.py`.
  - Arm groups with cowl: 178 contact, 164 frontier, 14 deep.
  - The repo's `stl/electronics/collision_frontier.json` / `collision_separable.json` are the earlier no-cowl run (77 contact, all frontier).
- Arm vs wearer / harness (`collision_body.py`, not in the repo): 10,368 poses, 1099 contact, 802 frontier, 297 deep. By body: head 526, harness 221, skin 55. Nearest at park M1 -> harness 5.4 mm. Worst frontier: S1 10 / S2 30-45 / S3 50 / E 100-120, claw 29-30 mm from the head.
- Restricted full sweep (`stl/sweep_full_restricted.json`, 2592 poses, 1601 s; predates the curved pauldron meshes): min body -13.0, harness -8.4, head -4.3 mm; 181 contact poses, 179 inside the 60 mm head zone. The unrestricted `sweep_full.py` has not completed (`logs/sweep_full.log` empty).

## 12. Open items

1. **W2 / W3 loops.** The 30-base re-search (`run_local.sh`) did not finish: W2 stopped at 75/400 candidates with 1 marginal hit (clr 0.80, bend 13.2), W3 at 8/400 with 1 marginal hit (clr 0.80, bend 12.1). The `wire_search_W2_CB.json` / `W3_CB.json` in the repo are from a 12-base run. If it still fails: firmware limit W1 >= -30 deg when E < 60, or a rubbing sleeve. **Forest to decide.**
2. In-link runs, arm_chain DP, hub link, clips and boot sizes for the new elbow anchors.
3. `sweep_full.py` (both modes) on the current cowl / curved pauldron, then the firmware keep-out tables (`gen_tables.py`).
4. Harness top-pad margin (2.0 %) vs boot mass (~99 g per arm at 0.7x TPU wall): `harness_loads.py` needs `sensor_pods.py` + `electronics_fit.py`.
5. Pad stall cap: AK70 S1/S2 cap 8.3 -> 8.0 N m (pad stall limit 16.6 N m: 2.00x at 8.3, 2.075x at 8.0); S2 torque SF with the balancer stays 2.77. **Forest to decide.**
6. Bench / parts items: S2 free fall T12 (phase-short braking f >= 0.80), relays, 1.10 s off-delay stage, main-box heat, bench test T-W1.
7. Missing from the repo: `collision_body.py`, `gen_tables.py`, `gen_state_table.py`, `harness_loads.py`, `sensor_pods.py`, `electronics_fit.py`, `balancer_freefall.py`, `balancer_struct.py`, `arm_clips.py`, `control_map.py`; specs `SPEC_third_shoulder.md`, `SPEC_balancer.md`, `SPEC_ball_stops.md`; `stl/gates.json`; `stl/electronics/arm_vendor_data.json` and `arm_electrical.json`.

## Changelog (dated entries recoverable from the repo)

- 2026-10-12c (I1): head keep-out limits (S1 soft min +10, S2 soft min -30); TPU capsule stop pads; ball slot follows S2 hard range.
- 2026-10-13: shoulder cowl + dome replace the 190 mm ball; faceted clasp and pauldron (Forest: "facet it to my shoulder"); ball shell split in halves.
- 2026-10-13b: ball-styled elbow.
- 2026-10-13d/f/g: parallel gripper replaced by the claw; C3 punch-ring claw; sweeps unified on the claw at G 0.
- 2026-10-14: sEMG muscle atlas rev; distal masses from meshes (verify_distal 14h).
- 2026-10-15: S2 balancer rev e (zero-length spring); claw pin retention and servo ear pattern.
- 2026-10-17: wire / connector vendor data retrieved (`arm_vendor_data.json`).
- 2026-10-22: harness rebuild, collision method, collision_body, new elbow loop (see section 11).
- 2026-10-23: pauldron lateral shelf replaced by the curved deltoid shell; meshes regenerated.
- 2026-10-25: this file reconstructed from the repo.
