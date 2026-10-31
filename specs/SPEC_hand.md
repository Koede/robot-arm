# SPEC: HAND rev B, human-layout micro-servo hand with chuck modes (2026-10-30b; **adopted as the project hand by Forest 2026-10-30**; IN DESIGN, not locked)

Requested by Forest (2026-10-30): a hand that can turn into a drill chuck and close finger by finger with micro servos, with a human layout, and poses 👉 👍 ✌️.
**Rev B replaces rev A** (same day). Forest's verdict on rev A: "this looks terrible". Rev A had put the chuck ahead of the human layout: knuckles stair-stepped 60 mm, palm 34 mm behind the wrist, flat bar fingers with external links. It also broke the standing "ball-styled joints" rule.
Rev B puts the human layout first:
- human knuckle arch, palm centred on the wrist;
- tapered elliptical phalanges with ball knuckles in cups, coupling bars hidden in a dorsal groove;
- full TPU fingertip caps;
- servos enclosed in a sculpted palm (thenar and hypothenar bulges, dorsal knuckle hood).

Subsystem WRIST + GRIPPER: **the project's end effector** (adopted 2026-10-30). It replaces the C3 claw, whose files stay as the superseded alternate. Interface contract unchanged: W3 ears, floor plate HB0 491.6 / FT 493.6, D 57.

**Scripts (run in this order):**
1. `hand_kin.py`: kinematics and the four-bar fits.
2. `hand_poses.py`: chuck solver. Env: `RHO_FIX=110`, `RBS`, and `JAW3=1` for the 3-jaw mode.
3. `hand_pose_lib.py`: the pose library.
4. `hand.py`: SDF + marching-cubes parts with manifold booleans.
5. `hand_pose.py`: posed assemblies, bar audit, real-mesh collisions, render scenes.
6. `verify_hand.py`.
7. `render_hand.py`, which calls `render_three.py` (three.js in headless Chromium; installs `three` into `cad/_three/` on first run).

**Outputs:** `stl/hand/` holds 35 printable parts plus `poses/<pose>/` and `poses/hand_<pose>_posed.stl`. Renders are `hand_<pose>_sheet.png` at the repo root.

## 1. Layout (hand frame: x palm normal, u = y - D toward the thumb, z distal)
- **Scale:** 0.80 × the 50th-percentile male hand.
- **Phalanx lengths (mm):** index 32.0 / 18.4 / 14.4, middle 35.2 / 21.6 / 15.2, ring 32.8 / 20.8 / 14.4, pinky 26.4 / 15.2 / 13.6. Thumb 38.4 / 26.4 / 21.6.
- **Knuckles:**
  - MCP u: +20 / +3 / -14 / -30.
  - MCP z: 549 / 552 / 549 / 541. This is a human arch; the middle knuckle sits on the forearm line like the third metacarpal.
  - Finger planes at x 0. The palm spans x -13 … +15.
- **Finger width** tapers from 15.6 to 12.4 mm (index).
- **Fingertip:** a 6.3 mm sphere including a 1.2 mm TPU 85A cap.
- **Joints:** each phalanx ends in a ball (r = 0.9 × min half-width/thickness), seated in a cup (r + 0.4) in the previous part. Hinge pins are d2 steel: press fit in the cheeks (d1.9), running in the ball (d2.1). Pins sit at least 3.6 mm apart.
- **Envelope:** open-hand tip z 624.3, so wrist-to-tip from W2 is 214 mm against the 165 mm contract. x -26.8 … +33.2, which no longer reaches behind the wrist.

## 2. Actuation: 8 micro servos (MG90S class, PLACEHOLDER: 22.8 x 12.2 x 22.7 mm, 13.4 g, 0.216 N.m at 4.8 V)
- **Fingers:** 4 servos, one per finger, for individual curl. Shafts point along +x, under the palm cover. A crank (r 8) and rod pull a palmar flexor lever on the proximal ball, like a flexor tendon.
- **Synergy:** PIP = 1.10 × MCP and DIP = 0.70 × PIP, through ground-linked four-bars inside the dorsal groove. Bar fit error is 0.05–0.15 mm.
- **Abduction:** 1 servo, mirrored, driving the index and ring MCP carriers about the palm normal: +psi adducts, -psi spreads.
- **Thumb:** 3 servos. Two drive a 2-axis CMC (swing about x, lift about u). One drives the coupled MCP/IP flexion (IP = 0.75 × MCP, via a four-bar). Pronation is built in at 110 deg.
- **Servo pockets:** fingers in a row at z 518–541, u +19 / +5.5 / -8 / -21.5. Two thumb servos in the thenar. Thumb flexion and abduction sit in the carpal row.

## 3. Chuck modes (bit axis along +x, out of the palm; the wrist W2/W3 points it)
| mode | jaws | bit | jaw spacing (deg) | digit clearance | solve |
|---|---|---|---|---|---|
| 4-jaw | index, middle, ring, thumb | Ø13 (capacity about Ø10–16) | 94 / 83 / 93 / 90 | capsule -0.2 mm, real mesh: no interference | `chuck_d13` |
| 3-jaw (the human "three-jaw chuck" grasp) | thumb, index, middle | Ø6 (capacity about Ø4–8) | 98 / 124 / 138 | capsule -0.14 mm, real mesh: no interference | `chuck3_d6` |

Below about Ø10, four 12.6 mm fingertips spaced 90 deg apart physically overlap, so small bits use the 3-jaw mode. The fingertips grip with their full TPU caps; the thumb is required to use its pad.
**Grip (conservative: every jaw limited to the weakest one, mu 0.5 assumed, 70 % of stall):**
- 4-jaw: 1.81 N per jaw, holding **0.0235 N.m**.
- 3-jaw: 2.58 N per jaw, holding **0.0116 N.m**.

This is a light chuck: it can turn a small bit or screwdriver tip, not drill metal. The middle finger is the weak jaw in 4-jaw mode because its tip moves almost tangent to the bit.

## 4. Pose table (finger MCP deg, with abduction where non-zero; thumb swing / lift / flex deg; thumb pronation 110 deg)
| pose | index | middle | ring | pinky | thumb |
|---|---|---|---|---|---|
| open | 0.0 | 0.0 | 0.0 | 0.0 | 0.0 / 0.0 / 0.0 |
| chuck4_d13 | 36.9 / abd 7.5 | 32.1 | 37.1 / abd 7.5 | 76.0 | 48.3 / 12.2 / 0.0 |
| chuck3_d6 | 41.7 / abd 4.9 | 49.3 | 82.0 / abd 4.9 | 64.0 | 39.3 / 28.3 / 0.0 |
| thumbsup | 81.0 | 83.0 | 82.0 | 76.0 | 0.0 / -5.0 / 0.0 |
| point | 0.0 | 83.0 | 82.0 | 76.0 | 79.8 / 41.9 / 40.6 |
| victory | 0.0 / abd -15.0 | 0.0 | 82.0 | 76.0 | 60.3 / 26.1 / 0.0 |

The fist angle is the largest MCP that keeps 1 mm from the palm: {'index': 81.0, 'middle': 83.0, 'ring': 82.0, 'pinky': 76.0}.

## 5. Verification (`verify_hand.py`, `stl/hand/verify_hand.json`, `pose_report.json`)
| pose | four-bar length error on posed CAD (mm) | real-mesh interference between digits / palm | TPU cap compression on the bit (mm3, designed grip) |
|---|---|---|---|
| open | 0.136 | none | - |
| chuck4_d13 | 0.11 | none | {'index_cap': 13.19, 'middle_cap': 2.74, 'ring_cap': 14.76} |
| chuck3_d6 | 0.076 | pinky_cap/ring_cap 0.04 mm3 | - |
| thumbsup | 0.11 | none | - |
| point | 0.11 | none | - |
| victory | 0.11 | none | - |

| Gate | Result |
|---|---|
| Printability | All 35 parts watertight and inside the 256 mm cube. |
| Mass | **223.9 g** (87.7 g printed, palm at 55–60 % infill; 107.2 g servos; 29 g hardware). That is **+45.0 g** over the claw; rev A was +149 g. |
| S2 (scaled estimate) | 3.27 N.m, SF 2.54: pass. |
| Harness top pad (mass-proportional estimate) | 14.61 vs 14.7 kPa: **pass, margin about 0.6 %**. Confirm with `harness_loads.py`. |
| Not run | Wrist sweep, whole-arm sweep and keep-out tables (the envelope changed), crank/rod synthesis for the rev B servo row, the abduction linkage, cover screw bosses. |

## 6. Decisions for Forest
0. **Decided 2026-10-30:** rev B is the project hand. Still open: re-integrating it with the wrist-v3 coupling (`specs/SPEC_wrist_rotary.md`, branch `claude/hand-coupling-2026-10-30`, built on rev A).
1. Wrist-to-tip 214 mm against the 165 mm contract (human proportions at 0.8 scale). This needs the sweeps and keep-out tables re-run.
2. Hand mass +45 g. The harness passes only by an estimated 0.6 %, so a real `harness_loads.py` run decides.
3. The chuck is light duty (0.012–0.024 N.m). Real drilling would need locking jaws or bigger actuators.
4. The 3-jaw / 4-jaw split by bit size is my proposal from the geometry.
