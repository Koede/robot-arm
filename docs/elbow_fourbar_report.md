# Elbow four-bar study (2026-10-26)

Scripts: `cad/elbow_fourbar.py` (kinematics, search, torque), `cad/elbow_fourbar_pack.py` (packaging, cable anchors). Results: `stl/elbow_fourbar/fourbar.json`, `packaging.json`, proxy meshes `links_S*_E*.stl`, `xm540_relocated.stl`. Frames: robot frame, mm, deg, right arm; four-bar in the x-z plane, elbow axis O4 = (0, 230).

## 0. Two corrections to the premise

- The XM540 is not above the elbow axis. In `upper_arm_elbow.py` its pocket is centred ON the axis (z 213..247, horn axis = elbow axis), and the body is already fixed in the upper arm (group M3). Only the horn, the two caps and the crossbar (`fork_*`, 78 g) rotate. A four-bar therefore does not remove the motor from the moving side; it only moves the motor away from the axis (the study puts it proximal, centre z 204) and replaces the horn-on-axis drive with a crank.
- The E cable loop does not cross a rotating motor today either (section 4).

## 1. Required elbow torque

2.79 N m, static, forearm horizontal, 0.5 kg at Z_TIP 575. Source: the torque section of `verify_distal.py` (ME + MW1 + MW2 + MW3 mesh volumes x density, bought masses, wiring) redone in `elbow_fourbar.required_torque()`, because `stl/distal/verify_distal.json` is not in the repo (it needs `verify_claw.json`; the 8.4 g claw hardware figure is taken from the comment in `verify_distal.py`). Distal mass 700.5 g, CG 160.1 mm above the axis: 1.10 N m gravity + 1.69 N m payload.

Direct drive (MA = 1): 10.6 N m stall, SF 3.8; continuous 5.3 N m (0.5 x stall), SF 1.9. The repo's gate (SF >= 2 against the stall figure used as rated) already passes. The continuous figure is below 2 x 2.79 only if the 0.5 derating is applied.

## 2. Four-bar search

Model: crank a on the horn at O2, coupler b, forearm lever c at O4 with offset beta from the forearm direction. MA = tau_elbow / tau_motor = |d theta / d E| (ideal, no friction). Gates: crank travel for E 0..145 <= 300 deg; transmission angle mu (coupler/lever) 30..150 over E 0..145 (searched with 2 deg margin); no toggle over -5..150 (mu and the crank-side angle nu kept >= 12 deg from 0/180, theta(E) monotonic); O4 at least 9 mm outside the XM540 rectangle. Differential evolution, both branches, both motor orientations, 2 seeds each, 1 core. Objective: maximise the minimum MA over E 0..145.

| Family | a / b / c (mm) | O2 from O4 | travel soft | mu soft | MA min / mean / max | tau stall min | tau cont min |
|---|---|---|---|---|---|---|---|
| A free | 51.9 / 67.2 / 81.1 | 44.7 mm | 255 deg | 32.0..80.7 | 1.385 / 1.76 / 2.46 | 14.7 N m | 7.3 N m |
| B in-line proximal | 56.5 / 71.7 / 87.8 | 48.1 mm, straight down | 254 deg | 32.0..81.2 | 1.385 / 1.75 / 2.42 | 14.7 | 7.3 |
| C compact (chosen for packaging) | 30.1 / 38.4 / 46.75 | 25.8 mm, straight down (x +1.2, z 204.2) | 255 deg (269 hard) | 32.0..81.4 | 1.385 / 1.76 / 2.45 | 14.7 | 7.3 |

C: beta = -165.8 deg, ground direction 177.2 deg (from +z toward +x), branch +1, XM540 long side across the arm. MA depends only on length ratios, so the optimum is the same at any scale; the scale is set by the motor body (g >= 25.8 mm). The minimum MA 1.385 is pinned by the mu >= 32 gate at both ends (E 0 and E 145) and cannot be raised inside the gates.

Sampled C table (hard range edges shown for information; mu below 30 at -5 and 150 is outside the soft range):

| E | crank rot from E=0 | mu | nu | MA | stall N m | continuous N m |
|---|---|---|---|---|---|---|
| -5 | +6 | 28.6 | 40 | 1.18 | 12.5 | 6.2 |
| 0 | 0 | 32.8 | 37 | 1.39 | 14.7 | 7.3 |
| 15 | -27 | 48.8 | 33 | 2.15 | 22.8 | 11.4 |
| 30 | -62 | 66.6 | 36 | 2.45 | 26.0 | 13.0 |
| 60 | -127 | 81.4 | 56 | 1.85 | 19.6 | 9.8 |
| 90 | -177 | 72.4 | 84 | 1.49 | 15.8 | 7.9 |
| 120 | -219 | 53.2 | 116 | 1.39 | 14.7 | 7.3 |
| 145 | -255 | 32.0 | 148 | 1.55 | 16.4 | 8.2 |
| 150 | -263 | 26.7 | 156 | 1.71 | 18.2 | 9.1 |

Versus direct drive at the ends: full extension (E 0) 14.7 vs 10.6 N m stall (+39 %), full flexion (E 145) 16.4 vs 10.6 (+55 %); continuous 7.3 / 8.2 vs 5.3. So yes, the four-bar beats direct drive everywhere in the range, but the margin that matters is already met by direct drive (SF 3.8 direct vs 5.3 for the four-bar at its minimum MA; both pass SF >= 2 at stall; with the 0.5 x derating direct drive is SF 1.9, the four-bar SF 2.6). Costs of the torque gain: the same factor in speed (output angular speed = motor speed / MA; at the XM540's nominal 30 rpm that is 180 deg/s direct and about 75..130 deg/s with this four-bar; 30 rpm is the manufacturer figure, not a repo number), the crank turns 255 deg through a position range that the XM540 only allows in extended-position (multi-turn) mode, the mu gate sits exactly at its limit at both ends so any build tolerance eats the 2 deg margin, and the real drive ratio is ideal-frictionless (pin friction in 5 pin joints lowers it).

## 3. Packaging (`elbow_fourbar_pack.py`)

Proxies: bars 10 wide x 8 thick with 5 mm end pads, XM540 as a 58.5 x 44 x 33.5 box, tested with python-fcl over E -5..150 in 1 deg steps against `ua_lat`, `ua_med`, the fixed stop posts and the whole ME group (forks, forearm root, stops, W1 servo).

- Layer stack S1 (crank y 26..34 on the horn, coupler 34.5..42.5, lever 43..51): FAILS. The crank collides with `fork_drive` at every angle (-5..150): the drive cap (y 24.4..31.7, sphere zone R 44 plus the forearm-mount neck to z 292) occupies the horn plane.
- Layer stack S2 (crank 33..41, coupler 41.5..49.5, lever 50..58): no collisions over -5..150. Minimum gaps: crank to shell 8.3 mm, coupler to shell 16.8, lever to shell 25.3, crank to fork group 1.3 (the 1.3 mm gap to the drive cap/mount neck at y 31.7), coupler to forks 11.8, lever to forks 20.3. Between links 0.5 mm by construction (pins not modelled).
- Pin/hub crossings: with six possible stacking orders, only crank (inner) / coupler / lever (outer), crank / lever / coupler and coupler / crank / lever let the O2 hub and O4 shaft pass the other layers (min centre distance to the bar >= 16 mm against 9 mm needed). S2 uses the first. The three orders with the lever innermost or the coupler/lever inside the crank fail because the lever bar passes through the O2 hub position.
- The lateral stack reaches y = 58, still inside the forearm root's lateral extent (y 20..94), but 33 mm beyond the current lateral surface of the elbow belt (y 24.7). It sits on the side where the wearer's arm is not, but it is exposed: guard or cover needed (pinch points at 3 moving joints). Not checked against the pauldron or body (they were not in scope).
- Not modelled: pin bosses, bearings, the stand-off from the drive cap to the lever (it replaces the horn spacer boss), fasteners.

How much of the ball-style elbow shell changes (volumes from trimesh boolean, `packaging.json`):
- The relocated XM540 (centre x +1.2, z 204.2, spanning z 187.5..221) needs a rectangular pocket 59 x 45 mm where the tube is an ellipse (outer half axes 32 x 23..24.7). The new pocket box cuts 19.9 cm3 of existing wall, 15.2 % of the 131 cm3 shell. The tube section at z 187..195 must bulge by about 1.5 mm each side in y (outer 23.2 -> 24.7) and keep x at 32; effectively the ball belt's rectangular frame is extended 40 mm proximally.
- The old pocket frame at the axis (box walls inside the ball) and its horn window are no longer needed. A new O33 horn window and a 623/6001 bearing for the lever shaft are needed at the new locations; the idler pin boss moves to the new motor axis on the medial wall.
- The R 44 belt/cap running-gap, the stop root blocks, the crossbar and the TPU stop posts can stay (the crossbar still strikes the stops at -5 and 150 hard angles; its angle is the forearm angle). `fork_drive` loses the horn boss and screw pattern and gains the lever stand-off.
- The sphere belt zone is 66 % of the shell volume; everything inside it gets re-checked (stops, laps, windows.json positions, hub posts are in the tube and unaffected).
Roughly: one half-new elbow region (about 15 % of the shell volume reworked, the pocket frame rebuilt in a different place) and two new parts-sets (three links, one lever stand-off), not a bolt-on.

## 4. E cable loop (`wire_search_E_CB.json`)

- The loop is two anchors, A = (42.1, -13.7, 168.0) on the upper arm and B = (55.3, -33.7, 268.3) on the forearm, both on the medial (-y) side and at the rear (x 42..55, outside the ball R 44), L = 191.6 mm, clearance 10.0, min radius 27.3. It does not pass through or over the XM540, which is a fixed body in the pocket, in the current design either. The idea's "cable need not cross a rotating motor" is therefore already true; the four-bar adds nothing for the cable.
- With the four-bar the cable still has to cross the joint (anchors on opposite sides of the elbow). The new XM540 box (z 187.5..221) lies 22.7 mm from A and 14.1 mm from the A-B chord (second candidate loop: A 25.1 mm, chord 11.1 mm), so the loop passes beside the relocated motor at the rear. The chord is a straight-line proxy; the real loop is an elastic rod and must be re-searched against the new motor, shell and link SDFs.
- The links are all at y >= 33 (lateral) and the anchors at y <= -3.7 (medial, both candidates): the cable and the linkage are on opposite sides of the arm, so the linkage adds no cable obstruction.
- Anchors: A stays in the tube (z 168, 62 mm proximal of the axis); if the tube is re-lofted for the motor (z 187..221), A is at its edge and will need re-checking. In-link runs, the arm_chain DP and clips are already flagged as not redone for the new elbow anchors (open item 2); a four-bar would extend that rework.

## 5. Recommendation: DROP (keep direct drive); revisit only if torque becomes the limit

- Direct drive already meets the repo's SF >= 2 gate (SF 3.8; required 2.79 N m vs 10.6 N m stall). The four-bar buys a 1.4x..2.4x torque factor that is not needed, and pays the same factor in speed.
- It does not simplify the E cable: the motor is already fixed, the loop already avoids it.
- It moves very little mass off the elbow: the XM540 is already on the fixed side; the moving side drops 78 g (forks) and gains a lever, so the net is about zero.
- The cost is a re-lofted proximal elbow region (about 15 % of the shell volume reworked), 3 links + 5 pin joints with ~58 mm lateral stack, a guard, a 255 deg multi-turn crank range, mu pinned at its 30 deg gate at both ends, and a repeat of the sweeps, stops and wiring work that open items 2 and 3 already carry.
- Payload headroom (tip load at Z_TIP, 3.38 N m per kg plus 1.10 N m of arm gravity): SF 2 against stall is lost at 1.24 kg direct vs 1.84 kg with the four-bar (minimum MA); against 0.5 x stall at 0.46 kg direct (the 0.5 kg design load is already at SF 1.9) vs 0.76 kg with the four-bar. If the elbow ever has to carry more, a simple reduction on the horn (belt or small gear stage) gives the same ratio without moving the axis geometry. Build the four-bar only if a continuous-duty 0.5 kg load becomes a hard requirement and the reduction option is rejected.

This is a geometry-level decision (it changes the elbow shell and the E cable); the call is Forest's. No existing file was edited; `DESIGN_STATE.md` is not in this worktree and was not changed, so a changelog entry for this study still needs to be added to it.
