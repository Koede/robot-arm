# Parallel wrist (spherical five-bar) - proxy build and check

Date 2026-10-26. Branch worktree-agent-ab4e0da4b2db08c19 (based on claude/parallel-wrist). New files only.

## What was built
- `cad/wrist_parallel.py`: five-bar kinematics (both branches, branch-continuous inverse over the serial grid) and proxy geometry at R_W = 70 mm about C = (0, D_OFF, Z_W3). Links are chains of boxes (8 x 6 mm) on great-circle arcs, 10 mm bosses, M2.5 pins. Links sit on four radial layers (70/78/86/94 mm): crank 1, floater 1, floater 2 (+ carrier strut to the tray), crank 2. Servo A/B are STS3215 proxies on the J0/J4 shaft axes outside the spheres, with strut brackets to the forearm tube rim. Hand = hand_tray + sts_g (fingers optional). Outputs `stl/wrist_parallel/wp_*.stl` (original kin.json) and `*_v3.stl` (final), plus `wrist_parallel_layout*.json`.
- `cad/wrist_parallel_check.py`: 96 serial targets (W2 -70..80 step 10, W3 -20..30 step 10), python-fcl pair clearance, roll, crank travel, cable count. Results `check.json` (kin.json params, psi 0), `check_v2.json`, `check_v3.json`.
- `cad/wrist_parallel_search.py`: re-search (own file, kin script untouched) with extra cost terms: cone clearance of ground axes and links around the claw axis (the hand fills a cone), a cone around the forearm axis, and roll span. Results `kin_v2.json`, `kin_v3.json`.

## Numbers
| | kin.json as given | v2 (cone+roll) | v3 (cone+roll+forearm cone) |
|---|---|---|---|
| ground axes J0 / J4 (az, pol) | 235/70, 55/110 | 91/90, 269/91 (+-y) | 91/86, 272/92 (+-y) |
| (a)(b) colliding poses of 96 | 82 (psi 0) | 19 (psi 180) | 18 (psi 180) |
| worst overlap | 14.7 mm (bracket A / tray) | 17.7 mm (forearm / floater 2) | 19.6 mm (forearm / floater 2) |
| (c) roll span vs serial | 70 deg (-18..+51) | 17.9 deg | 20.0 deg (+-10.0 when centred) |
| min crank gain sigma | 0.19 | 0.25 | 0.27 |
| crank travel th1 / th2 | n/a | 233 / 242 deg | 235 / 220 deg |

Direction coverage of the serial set stays within 0.7 deg worst case in v3.

(a) v3 smallest pair clearances: forearm/tube vs floater 2 -19.6 mm (W2 80, W3 30); bracket A vs floater 2 -10.8, vs floater 1 -8.0; bracket B vs floater 2 -9.5; servo A vs floater 1 -2.4; the rest >= 0.6 mm (bracket B vs crank 2 0.6, bracket A vs crank 1 1.7, servo A vs floater 2 2.1). Tray and hand clear everything by >= 7 mm once the ground axes are on +-y.

(b) v3 colliding poses (18): W2 = 70/80 at all W3, W2 50..60 at W3 -20..-10 and 20, and W2 -60/-70 at W3 0. W2 -50..+40 is clean at every W3. The servo rotation about its shaft matters (psi 0/90/180/270 gave 52/66/18/36 colliding poses).

(c) Roll: the original kin.json mechanism has a 70 deg roll span, which fails +-10 deg. After the re-search the span is 18-20 deg, i.e. +-10 deg only if the hand is mounted with a roughly 6 deg offset (centred). Zero margin. The roll is the twist of the floater-2 carrier frame against Ry(-W2) Rx(W3) at the same axis.

(d) Cables: the two servo cables cross 0 moving joints (servos are on the forearm). The claw G servo cable still crosses at least 2 joints (J3 and the crank-2 shaft J4); the J3 fold angle swings about 96 deg. The problem moves from W2/W3 to J3/J4; it is not gone. Bend radius of that cable was not assessed.

## Failure modes
1. Floater 2 sweeps through the forearm-axis direction at large tilts (W2 >= 50): its ~125 deg equator arc meets the distal tube. The forearm-cone cost did not remove it (residual cone violation mean 3 deg, max 27 deg), a coverage-versus-clearance conflict. forearm_dist also carries the serial W2 ball shell; tube-only numbers equal the full-mesh ones for floater 2.
2. The brackets are crude straight struts and cause most other overlaps; a real housing may fix them. Servo A sits ~100 mm from C.
3. Both ground axes must lie along +-y (tray cone), so servos sit 100+ mm out laterally on both sides of the forearm. Width and wearer clearance not checked.
4. Roll margin is zero. Crank travel ~235 deg (STS3215 limit 360) is fine.
5. Claw fingers are not in the clearance numbers (tray + G servo only); `--fingers` exists but was not run.

## Recommendation
Change geometry; do not build yet. The kin.json mechanism fails (82/96 poses collide, 70 deg roll). The re-searched v3 mechanism is plausible: 78/96 poses clean, roll +-10 deg when centred, well-conditioned cranks, servo cables off the moving joints. Before committing: add a hard forearm-axis exclusion to the search or cut the W2 range to about -50..+40 (a limits decision for Forest), replace the bracket struts with a real housing, run with the claw fingers, and decide whether the G cable crossing 2 joints is acceptable. Drop it if W2 +80 must stay and the forearm conflict cannot be searched away.
