# SPEC: HAND v2, rebuilt on the plane-to-plane wrist (2026-10-30 late) -- READ THIS BLOCK FIRST; it supersedes the numbers below where they differ

The hand was regenerated around the new wrist coupling (`SPEC_wrist_rotary.md`): the palm base plate now sits on the rotor mating plane (**z 515.6 = 491.6 + DZ 24**, `hand_kin.DZ` shifts every absolute z: knuckle row, thumb CMC, servo pockets, palm), the old W3 ears and the post-cut hack are gone, and the base has a **d22 wire port** over the rotor neck, **4 x M3 clearance holes on the r15 flange circle**, and wire grooves (collector + 4 branches) to the servo pockets. Scripts re-run in order: `hand.py` -> `wrist_rotary.py` -> `hand_pose.py` -> `verify_hand.py` -> `verify_wrist_rotary.py` -> `render_hand.py` (renders now include the wrist stack).
| item | v2 result |
|---|---|
| open-hand wrist-to-tip from W2 | **243.2 mm** (219 + DZ 24; claw 165) |
| digit collisions, open / 👉 / ✌️ / 👍 | 0, now also against the rotor, carrier and flexures |
| 👍 thumbs up | index limited to **65 deg MCP** (95 deg collides with the hub and rotor web); middle, ring, pinky 95 deg |
| chuck | unchanged: one 0.15 mm middle_prox / ring_mid touch; pad z +24 mm vs the table below |
| R rotation -170..+170 | all 5 poses clear of the carrier stack (min gap >= 5.77 mm; chuck 10.1) |
| all 34 hand parts + palm | watertight, positive volume, fit the 256 mm cube |
| mass | hand 319.5 g (printed 178.3, hardware 141.2) + wrist stage 196 g = **+336.6 g vs the claw 178.9 g** |
| S2 worst residual (scaled estimate, not re-gridded) | 4.89 N.m, **SF 1.70 vs 8.3: FAIL** (was 2.17 for the hand alone on the old wrist) |
| harness top pad (mass-proportional estimate) | 15.34 kPa vs 14.7: **FAIL** (-4.3 %) |
Decisions for Forest (new): (1) the S2 gate now fails with the wrist stage; options: drop the R servo or move it, lighter servo classes, lattice the palm tray, a stronger balancer K, or accept an S2 duty derate; (2) fist: 65 deg index limit vs +10 mm lift; (3) the contract change is now 24 mm longer than the earlier hand (243 vs 219 mm).
Everything below this block is the 2026-10-30 first version; where it quotes z values, add 24 mm; where it quotes mass or S2, use this block.

# SPEC: HAND, human-layout micro-servo hand with CHUCK mode (2026-10-30, IN DESIGN, not locked)

Requested by Forest (2026-10-30): "a hand that can transform its configuration into a drill chuck ... close its hand organically finger by finger individually ... with micro servos"; refined: "3 of the fingers and the thumb do this ... it should have a human layout"; target poses 👉 👍 ✌️.
Subsystem: WRIST + GRIPPER (alternative end effector to the C3 claw; the claw spec and files are untouched). Right hand, hand frame = W3 frame (x palm normal, u = y - D thumb side, z absolute). Script chain: `hand_kin.py` -> `thumb_solve.py` -> `hand.py` -> `hand_pose.py` -> `verify_hand.py` -> `render_hand.py`. Design-study scripts: `chuck_scan.py`, `chuck_cad_scan.py`, `final_scan.py`. Outputs: `stl/hand/` (per-part STLs, `poses/hand_<pose>_posed.stl` for viewing, JSON), renders `hand_<pose>.png` at the repository root.

## 1. Concept
- **Human layout:** palm with a knuckle row, index/middle/ring/pinky each 3 phalanges, thumb on the radial (+u) side. 0.80 x 50th-percentile male hand lengths (index 32.0/17.6/13.6, middle 35.2/20.8/14.4, ring 32.8/19.2/13.6, pinky 25.6/14.4/12.8, thumb 42.3/29.4/23.9 mm; thumb 0.92 x).
- **Finger:** one micro servo per finger (individual curl). MCP is driven by a crank + pushrod to a ball stud on the proximal tail; **PIP = 1.10 x MCP and DIP = 0.70 x PIP by two ground-linked four-bars** (human synergy, rigid, repeatable). Bar fit error 0.02 mm at 100 deg; verified on the posed CAD: max bar-length error 0.005-0.022 mm in every pose.
- **Chuck mode:** the hand axis (= W1 roll axis, x 0 / u 0, so W1 spins the bit) is the chuck axis. Index and ring knuckle blocks yaw about a hinge tilted 45 deg from z (**one yaw servo, mirrored**), which tilts their curl planes toward the axis; middle finger curls from below; the thumb swings across and presses from the top (thumb pad normal -x). Four jaws: three finger pads below and to the sides (azimuth about 0 / +-41 deg of yaw) and the thumb pad on top, pad normals 84-94 % radial. Pads sit at z about 549 / 588 / 617 (index / ring / middle): the fingers pinch **staggered along the bit**, because three 8-13 mm wide fingers cannot pinch a thin bit at one z. The palm has no seat; the bit is held by pad friction.
- **Capacity:** solved for bit diameters 3-16 mm (table below). Above Ø16 the index/ring would need hyperextension.

| bit Ø mm | MCP index / middle / ring (deg) | yaw index+ring (deg) | pad z index / ring / middle |
|---|---|---|---|
| 3 | 3.2 / 11.3 / 3.2 | 46.0 | 547 / 586 / 615 |
| 6 | 2.5 / 10.3 / 2.3 | 43.7 | 548 / 587 / 616 |
| 10 | 1.5 / 9.0 / 1.3 | 41.5 | 549 / 588 / 617 |
| 16 | 0.2 / 7.2 / 0.0 | 37.4 | 550 / 589 / 619 |
The chuck "closing stroke" is only 3-11 deg of MCP, i.e. about 6-20 deg of servo: use servo position control with a force limit, not a long stroke.

## 2. Actuation (8 micro servos, MG90S class, PLACEHOLDER dimensions 22.8 x 12.2 x 22.7, 13.4 g, 0.216 N.m at 4.8 V)
4 finger flexion (index, middle, ring, pinky) + 3 thumb (swing about x, lift about u, MCP/IP flexion) + 1 yaw (index+ring, mirrored). Thumb roll is built in (190 deg, fixed). Index/middle/ring crank (r 8) + rod curves solved: index rod 12.5 mm, servo range 49 deg; middle 48.5 mm, 77 deg; ring 27.5 mm, 68 deg. **Pinky crank/rod: no valid solution found yet (open item).** Cranks pass near a dead point at one end of travel (dphi/dtheta 0.01-0.03) so grip force there is limited by the crank, not the servo.
Poses (finger MCP deg, yaw deg; thumb swing/lift/flex deg), from `hand_pose.make_poses()`:
| pose | index | middle | ring | pinky | thumb |
|---|---|---|---|---|---|
| open | 0, 0 | 0 | 0 | 0 | 0 / 0 / 0 |
| chuck Ø10 | 1.5, yaw 41.5 | 9.0 | 1.3, yaw 41.5 | 95 | 52.0 / 21.2 / 9.4 |
| 👉 point | 0 | 95 | 95 | 95 | 10 / 0 / 0 |
| 👍 thumbs up | 95 | 95 | 95 | 95 | 0 / 0 / 0 (thumb extended; render rotated 90 deg about x for the emoji read) |
| ✌️ victory | 0, yaw -30 (16.7 deg spread) | 0 | 95 | 95 | 80.2 / 72.0 / 10.3 (tucked over ring/pinky) |
The tilted hinge does both jobs: +yaw tilts the curl plane inward (chuck), -yaw spreads an extended finger (V sign).

## 3. Geometry (mm, hand frame)
Knuckle row (stair-stepped, needed for the chuck): MCP z index 498 / middle 558 / ring 535 / pinky 514; u +23 / 0 / -23 / -42.5; finger plane depth x -20 (index, ring, pinky) and -27 (middle, so the yawed index/ring pass in front of it). Palm slab x -60..-34, u -52..+56 (behind the hand axis: the axis passes 34 mm in front of the palm). W3 ears, floor plate and HB0/D/Z_W3 are the **unchanged interface contract** (ears at |x| 23.3-26.8, floor 491.6-493.6); the floor plate is trimmed for u > 14 ahead of x -34 so a curled index clears it. Thumb CMC at (-8, 41, 500). Open-hand envelope: z max 629.2 (**wrist-to-tip from W2 219 mm vs 165 mm in the contract**), y 5..144, x -60..26.8.
Finger layers (t): phalanx plate 8 wide, bars outside at 0.4-2.2 mm, posts d2 steel pins (press d2.0 / run d2.1 holes), TPU 85A pads 1.5 mm.

## 4. Verification (`verify_hand.py`, `stl/hand/verify_hand.json`)
| gate | result | status |
|---|---|---|
| Four-bar bar lengths on posed CAD | <= 0.022 mm, all 5 poses | pass |
| Digit collisions, open / 👉 / 👍 / ✌️ | 0 | pass |
| Digit collisions, chuck | 1 pair touching: middle_prox / ring_mid, 0.001 mm3, 0.15 mm penetration (tessellation level); the 3 mm part clearance target is NOT met between fingers (designed contact class) | pass with flag |
| Watertight, positive volume, fits 256 mm cube (all 34 parts) | yes | pass |
| Chuck jaw force | fingers 2.3-3.0 N each at 70 % servo stall, **thumb capacity 3.4 N vs 7.2 N needed** to balance them: finger torque must be scaled to 0.47 (1.1-1.4 N per pad) | pass with software cap |
| Chuck holding torque at W1 | **0.018 N.m** (mu 0.5 TPU on steel assumed, Ø10 bit) | light duty: hand-drive a small bit/screwdriver tip, foam/balsa; NOT metal drilling |
| Mass | hand 327.9 g (printed 186.7 incl. palm tray at 55 % of solid, servos 107.2, hardware 34): **+149 g vs the C3 claw (178.9 g)** | see decisions |
| S2 worst residual with balancer (scaled from rev f3, not re-gridded) | 3.83 N.m, SF 2.17 vs 8.3 | pass (thin), re-run `s2_grid.py` |
| Harness top pad (mass-proportional estimate) | 14.87 kPa vs 14.7 | **FAIL (-1.2 %)**, re-run `harness_loads.py` |
| Wrist sweep, whole-arm sweep, keep-out tables, W3 cable | **not run** | open |
PLACEHOLDERS: servo geometry/horn/torque, ball rod ends, 6 V rail (buck from the 12 V trunk) and PWM driver in the hand, hinge pins and the yaw linkage geometry (modelled as a servo pocket only), thumb servo shafts, TPU/steel mu.

## 5. Decisions for Forest (numbers, no silent choice)
1. **Contract change:** wrist-to-tip 165 mm (claw) -> 219 mm (open hand); palm 26 mm further toward the body than the claw envelope (x -60 vs -47.6). Invalidates `sweep_full` results and the firmware keep-out tables until re-run.
2. **Mass:** +149 g breaks the harness top pad (14.87 vs 14.7 kPa). Options: (a) lighter servo class for non-load fingers (4-5 g sub-micro on pinky + yaw: about -30 g), hollow/lattice the palm tray (about -25 g), (b) accept a harness re-fit, (c) keep the claw as the carried end effector and make the hand a swap-in module.
3. **Chuck scope:** 0.018 N.m is a light chuck. For real drilling torque the hand would need steel jaws on the fingertips or a locking pinch (bigger servos or worm-driven jaws).
4. Pinky and thumb-tuck refinements (pinky rod, tuck error 2.6 / 6.0 / 4.5 mm) can follow once 1-3 are decided.
