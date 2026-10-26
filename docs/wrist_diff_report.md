# Bevel-gear differential wrist: study report (2026-10-26)

Branch `worktree-agent-a5686402ac2f58a49`. New files only: `cad/wrist_diff_kin.py`, `cad/wrist_diff_proxy.py`, `stl/wrist_diff/` (proxy meshes, `wrist_diff_kin.json`, `wrist_diff_clearance.json`), this report.
The repo has no `DESIGN_STATE.md` and no `specs/` (CLAUDE.md names them), so nothing here was cross-checked against them. Design decisions below are options for Forest, not choices made.
Units mm, deg, N.m. Robot frame, right arm. Re-run: `cd cad && ../.venv/bin/python wrist_diff_kin.py` (3 s) and `wrist_diff_proxy.py` (~2 min).

## 1. Architecture and kinematics

Layout that reproduces the serial set (W2 about y, then W3 about x, `arm_kin.frames`: `RW3 = Ry(-W2) Rx(W3)`):

- Side gears A and B on a common axis y through the wrist centre C = (0, 57, Zc), driven by two forearm-fixed STS3215 (MW1).
- Pinions (two, at +-x) on a cross shaft carried by a carrier ring that turns about y.
- The hand is bolted to the pinions with the claw axis along z, **perpendicular to the pinion axis**.
- Carrier angle th = (a+b)/2; pinion spin ph = rho (a-b)/2, rho = Zg/Zp.
- Hand rotation = Ry(th) Rx(ph), so **W2 = -th, W3 = ph** and a = th + ph/rho, b = th - ph/rho.
- Checked on a 31 x 26 grid over the soft range: the differential matrix equals the serial one to 0.0.

The brief says "equal turns give pitch, opposite turns give yaw". That holds only with the hand perpendicular to the pinion. If the hand sat on the pinion axis, opposite turns would only spin the hand about its own axis, and there would be no second pointing DOF.

- **Roll.** The serial wrist and the differential are the same Ry.Rx chain, so there is no additional roll error. The intrinsic tilt-twist of any pitch-then-tilt chain is -21 to +25 deg over the range, same as now.
- **Absorbing the twist with W1.** In principle yes: W1 (about the forearm z) is a third rotation. I re-solved a 7 x 6 grid for zero twist at the same claw axis with W2/W3 kept inside soft limits. That needs |W1| up to 18.5 deg and is solvable at only 22 of 42 poses. So W1 absorbs it only partly and only with W2/W3 re-planning. Nothing here requires it.
- **Singularities.** Pointing Jacobian = cos(W3), min 0.866 in range; singular at W3 = +-90 deg, outside the range. The gear train itself has no singularity.
- **Servo travel (rho = 1, miter pair).** Servo A -100..+100 deg, servo B -110..+90 deg, 200 deg each. That fits the STS3215's 360 deg and gives 0.044 deg of hand motion per count. With rho = 2: A -90..85, B -95..80, but the W3 load on each servo doubles (see section 2).
- **Pivot offset from the serial wrist.** The serial wrist pivots about z = 410 (W2) and z = 455 (W3); the differential has one centre Zc. Same angles move the claw tip by up to 23 mm (Zc 410/417), 27 mm (425) or 58 mm (455) relative to the serial wrist. So the workspace shifts a little, and the collision sweeps and keep-out tables would need rerunning.

## 2. Servo torque

`verify_distal.json` is not in the repo and `verify_distal.py` needs `verify_claw.json`, which is also missing. I re-implemented its torque formula: (|sum m (z - z0)| + payload x lever) g. The payload is 0.5 kg at Z_TIP = 575. The masses come from the meshes with its density rules, and I used its comment's 8.4 g for the claw hardware.

- Hand: 177 g, CG z = 511.8.
- Serial, recomputed: W2 1.02, W3 0.69 N.m. verify_distal's gate is SF >= 2 against the STS3215 stall of 2.94.
- Differential, Zc = 417: **0.94 N.m on each axis**. The payload lever is longer because the centre moves back; the W2 yoke and W3 servo are no longer carried.
- Side-gear torque: tauA = tau_W2/2 + rho tau_W3/2 = 0.94 (rho = 1, worst signs, W2 and W3 both at the horizontal-hold maximum, which is conservative).
- Servo torque at 90% mesh efficiency, direct drive: **1.04 N.m, SF 2.8 against the 2.94 N.m stall**. The gate is met. Holding is 35% of stall, so it runs warm but is feasible.
- rho = 2: 1.57 N.m, SF 1.88, which fails the gate. A 1.5:1 servo-to-gear transfer stage gives 0.70 N.m, SF 4.2 (but adds backlash: tuned case +-0.84 deg instead of +-0.64).

## 3. Gears (PA-CF, 60 mm envelope)

20 deg PA, 90 deg shaft angle, Lewis at the mean section (virtual teeth Z/cos(delta), Shigley form factors). Ko = 1.5 on the rated load. Two pinions share each side-gear torque.

| m | Z p/g | b | gear OD | bounding sphere | Ft | sigma rated x1.5 | sigma at servo stall | gear-only play, jt 0.2 |
|---|---|---|---|---|---|---|---|---|
| 1.5 | 16/16 | 5 | 26 | 34 | 46 N | 32 MPa | 67 | +-0.56 deg |
| 2.0 | 16/16 | 6 | 35 | 45 | 34 N | 15 | 31 | +-0.41 |
| **2.5** | **14/14** | **8** | **38.5** | **49.7** | **32 N** | **9** | **19** | **+-0.39** |
| 3.0 | 14/14 | 8 | 46 | 59.7 | 26 N | 6 | 12 | +-0.32 |
| 1.5 | 14/28 (rho 2) | 6 | 43 | 47 | 38 N | 25 | 35 | +-0.31 / tilt +-0.63 |

- The full option table is in `stl/wrist_diff/wrist_diff_kin.json`.
- Minimum module 1.0 and 14 pinion teeth are met; m = 1.0 is too weak (rho = 2: 73 MPa).
- **Recommended: miter m = 2.5, Z = 14/14, b = 8, two pinions.** The 60 mm bound is the bounding sphere about the apex, 49.7 mm. m = 3 reaches 59.7 mm and leaves no room for the carrier.
- Allowables are assumed and must be checked on coupons: PA-CF printed bending 60 MPa static in XY, 20 MPa fatigue; hardened steel 250; brass 100.
- Rated load is 9 MPa and servo stall is 19 MPa, just under the assumed fatigue allowable. Cap the servo torque register (about 1.5 N.m) to protect the teeth.
- Strength is not the issue. Axial separating force is about 8 N per mesh.

### Backlash

Pitch error is +-e/2 and tilt error is +-rho e/2, where e = jt / r_mean (r_mean = 14.7 mm). I assumed 0.5 deg total servo-shaft play (the repo has no figure for it).

| gear play jt | gear-only | with servo play | tip error at 165 mm (centre to claw tip) | at 575 mm |
|---|---|---|---|---|
| 0.30 mm (as printed) | +-0.59 deg | +-0.84 deg | +-2.4 mm | +-8.4 mm |
| 0.20 mm (tuned) | +-0.39 | +-0.64 | +-1.8 | +-6.4 |
| 0.08 mm (spring-preloaded, effective) | +-0.16 | +-0.41 | +-1.2 | +-4.1 |
| 0.05 mm (metal) | +-0.10 | +-0.35 | +-1.0 | +-3.5 |

- The serial wrist, servo play only, is +-0.25 deg or +-0.7 mm at the same lever. A tuned printed differential is about 2.6 times worse.
- The true lever for a wrist angle error is the wrist-centre-to-tip distance. The 575 mm column is the brief's conservative figure.
- Gravity preloads the pitch axis one way, so backlash mostly shows up on direction reversal and on tilt.
- **A printed differential is credible for strength.** For accuracy, print the gears tuned plus a spring preload, or use brass or steel gears, to get under +-0.5 deg.

## 4. Proxy meshes and clearance (`wrist_diff_proxy.py`)

The proxies are cones, cylinders and boxes, not teeth: two side gears with hubs and crown, a carrier ring (R 21..27), pinions with shaft stubs, a hand bracket at |x| = 31..35, and two servo boxes with shafts along z at y = +-21.5 behind the cluster. The existing hand (convex hull, conservative) rides on the bracket. The sweep covers 9 W2 x 6 W3 poses over the full soft range, against `forearm_dist_R.stl`. Zc = 417 (hand tray rear at 439 clears the gear sphere).

- **The existing distal tube does not fit.** The side gears intersect it (the tube ends at z = 430, the gear sphere reaches 442). The hand bracket hits it in 36 of 54 poses.
- **Tube trimmed to Zc - 30 (z = 387):** the bracket still hits it in 36 of 54 poses.
- **Tube trimmed to Zc - 45 (z = 372):** collision-free, min distance 3.0 mm (bracket), 16.5 mm (pinions), 21 mm (hand hull). That is thin for assembly tolerance, so use Zc - 52 (z = 365): min distance 10 mm.
- **Servos.** With the front face at Zc - 22 they collide with the swinging bracket. Zc - 45 clears it by 3 mm and Zc - 52 by 10 mm.
- **Consequence.** The servos then sit at z = 330..365, which overlaps the existing W1 pinion, ring-gear and bearing zone (z 339..349, `distal.json`). Nothing in this model checks that stack-up, but it means redoing the forearm root, the in-link runs and the boot sizes (open item 2).

## 5. Cable crossings

From `wire_audit_v2.json`, the serial wrist fails twice:

- **W2 loop** (MW1 to MW2): stretch 481%, min radius 2.8 against 13.2 needed, clearance -2.2.
- **W3 loop** (MW2 to MW3): stretch 64%, min radius 5.9 against 12 needed.

The differential removes both. There is no MW2 body, and both wrist servos are fixed on MW1, so their cables never cross a wrist joint.

What remains:

- **E, S1-S3 are unchanged.** The W1 loop is also unchanged, with clearance 0.8 at 3 poses.
- **The gripper servo (`sts_g`) still lives on the moving hand, and its bundle is now one loop crossing both pitch and tilt together.** It is a single bridge from MW1 to the hand across -70..80 deg of pitch and -20..30 deg of tilt, with the combined-pose problem of open item 1. Its minimum bend radius is not shown to pass: the serial W2 loop alone (150 deg, one axis) already failed.
- **Possible fix, not verified.** Route the bundle along the hollow axes: through a hollow side-gear hub along y, then through a hollow pinion shaft along x. Each segment then sees twist only, not bending, and twist of +-80 deg over about 100 mm is benign. The two axes cross at the centre, where the gears sit; whether three wires can bend through there is not checked.
- **Two servos instead of one W2.** The second servo adds one more cable run inside MW1 (a short one; the existing run ratio is 0.41).

## 6. Risks

1. The gripper cable loop is the same risk as now in a harder place.
2. The forearm tube must lose about 58 mm of length, and the servos collide with the W1 gear zone (section 4).
3. Backlash is 2.6 times worse unless preloaded or metal.
4. The centre moves 38-45 mm back and the hand's cantilever from the pivot grows to 165 mm. Tip position differs up to 23 mm from the serial wrist, so the wearer, collision and keep-out sweeps would have to be redone.
5. Part count: two side gears, two pinions, a carrier ring with bearings, a bracket, and (with a transfer stage) two crown/pinion pairs.
6. Gear allowables are assumed. Torque limiting is needed to protect the teeth at servo stall.

## 7. Recommendation (Forest's call)

**Do not adopt it as the primary wrist yet. Keep it as the fallback.**

- Strength, servo torque (SF 2.8) and the 60 mm envelope all work. The kinematics is exactly the serial set with no extra roll.
- The two failing loops disappear, but the gripper bundle becomes a single both-axes loop that has not been shown to pass. The forearm root must be redone, and backlash rises unless the gears are preloaded or metal.
- Suggested order: (1) decide the gripper cable route, e.g. a hollow-axis routing study; (2) if it passes and the five-bar wrist fails, build the differential with m 2.5 Z14/14 miter gears and spring-preloaded or brass gears; (3) otherwise take open item 1's alternatives (W1 firmware limit or rubbing sleeve).
