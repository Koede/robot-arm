# SPEC: WRIST ROTARY COUPLING, UNIFIED v3 (hand-to-arm), 2026-10-30, IN DESIGN, not locked

Subsystem: WRIST + GRIPPER. Replaces the W3 "H" yoke (two 3.5 mm ears + floor plate inside `palm_tray`) for the HAND end effector. Claw untouched.
Scripts: `cad/wrist_rotary.py` (parts -> `stl/wrist_rotary/`), `cad/verify_wrist_rotary.py` (-> `stl/wrist_rotary/verify_wrist_rotary.json`), `cad/render_wrist_section.py` (sections + `wrist_v3_3d.png`).
Concept studies A-F (parallel run): `concepts/*/`, renders `concept_{A..F}_3d.png`. Chosen: **C (monolithic PA12 cross-strip flexure)**; A/B (tendon bend cores) kept as alternates; D, E rejected; F (3-SPS legs) rejected (singularity, +55 g).

## Forest's direction (2026-10-30)
Plane-to-plane coupling, interconnects carry wires and structure; in-plane rotation of the wrist; **W3 kept, R added as an 8th powered DOF (one more STS3215)**; no yoke; compliant design; radially curved planes may join outer regions; "integrate and unify" the best compliant concept.

## Layout (robot frame, mm; W3 axis x through (y 57, z 455); R axis = hand axis x 0, y 57, along z)
| item | value |
|---|---|
| plane A (carrier plate) | 92 x 52 plate z 493.8-495.8 on the two flexure top pads; 3.0 mm swept clearance to the W3 servo corner (old floor: 0.8 mm) |
| W3 supports (replace both ears) | **2 monolithic PA12 cross-strip flexures** at x +25..45 and its mirror -45..-25: leaves 0.7 x 24 x 20 mm at +-45 deg crossing on the W3 axis, base bar bracketed to the W2 cradle face (x +-20.2, 2 x M3 PLACEHOLDER), top pad under plate A (4 x M3, PLACEHOLDER), 14.9 g each |
| W3 drive | crank r15 on the W3 horn (+x, x 20.6..22.9) + d2.4 push-rod to a lug under plate A at r 39: 2.6x, **78 deg of servo = 30 deg of hand**, 7.6 N.m available vs 1.96 N.m (SF 3.9) |
| R bearing | 6808-2RS 40x52x7 (same part as W1), z 496-503, inner race on the carrier hub r20 |
| R gear | rotor 52T m1.25 (R 32.5) / pinion 16T (R 10), ratio 3.25, centre distance 42.5; **R servo STS3215 vertical at -y (pinion at x 0, y 14.5)** so both x sides stay free for the flexures |
| wire path | hollow brass spindle tube OD 12 / ID 9 along the hand axis, z 491.8..513, perpendicular to planes A and B; hub bore + plate hole feed it from below; rotor neck (r 15..19) is the wire cavity above; top bushing on 3 spokes (second support 12 mm above the bearing); service loop for +-170 deg |
| plane B / mating plane | z 515.6 = 491.6 + DZ 24; rotor flange r 11..19, 4 x M3 inserts (PLACEHOLDER) |
| hand lift DZ | **24 mm**: wrist-to-tip 219 -> **243 mm** |

## Verification (`verify_wrist_rotary.py`, `verify_wrist_rotary.json`)
| gate | result | status |
|---|---|---|
| Parts valid, watertight, positive volume | carrier, flexure_px/nx, rotor, pinion, palm_tray_wrist | pass |
| W3 sweep -35..+35 (everything beyond W3, open pose) vs the fixed arm | 0 collisions. Smallest gap 0.4 mm = crank disc to the cradle wall (designed running gap, like the horn); flexure to servo/forearm 2.7 mm | pass, 2 flagged < 3 mm |
| W3 first contact (R servo / carrier vs W2 yoke) | +38.5 / -37.5 deg, beyond the passive stops (+35 / -25) | pass |
| R sweep -170..+170 in open / chuck / point / victory | 0 collisions; palm + digits to the carrier >= 5.8 mm (open, point), 10.1 (chuck), 6.5 (victory); rotor/hub running gap 0.2 (designed) | pass |
| 👍 fist (index MCP 95 deg) | **collides** with hub / rotor web and neck; with the index at 65 deg the full R sweep is clear (hand v2 pose set) | limit applied |
| Index curl limit in the fist pose | free to **65 deg MCP** (hit at 70) | firmware limit (default taken, Forest to confirm) |
| Flexures (2 in parallel) | k = 0.0028 N.m/deg total; strain 0.76 % at 30 deg, 0.89 % at 35 deg (PA12 static limit 3 %); axis shift 0.97 mm; roll stiffness 0.454 N.m/deg each | pass (analytic) |
| Support loads, 2 g | tip moment about x 1.96 N.m (servo, via crank); tipping about y 0.97 N.m = 14 N couple over the 68 mm spacing; axial 9.8 N each; est. leaf compression 12 N, est. plate-buckling SF 2.4 (concept C alone was 1.49 for 19.6 N) | pass (estimate), print + bench T-C check |
| R stage torque | 2.94 N.m x 3.25 x 0.9 = 8.6 N.m vs ~0.35 N.m hold | pass |
| S2 torque with hand + wrist stage (scaled estimate) | worst residual 4.89 N.m, **SF 1.70 vs 8.3** | **FAIL**, re-run `s2_grid.py` |
| Added mass | **196 g** (R servo 55, bearing 46, carrier 25.9, flexures 30, rotor 22.9, tube 4, rod/lug 2, pinion 2.2, hardware 8) | harness top pad est. ~15.3 vs 14.7 kPa **FAIL** (slope 2.49 kPa/kg; `harness_loads.py` not re-run) |

## Open items
- **Fist**: accept the 65 deg index-curl limit (default) or lift +10 mm (reach 253 mm). **Mass**: lighter R servo class, lattice the palm tray, or harness re-fit.
- Flagged < 3 mm: crank-to-cradle 0.4 mm (the cradle top at z 455.4 grazes the horn plane; add a relief), flexure-to-servo 2.7 mm.
- PA12 flexure needs SLS/MJF; FDM can't print 0.7 mm leaves. Fatigue estimate ~1.5e6 cycles from assumed endurance strains: bench-test coupons first. PA-CF fails (plate buckling 0.93, 2e4 cycles).
- PLACEHOLDERS: STS3215 horn / shaft offset / servo ears, horn-to-crank fixing, gear tooth profile (trapezoid, needs involute), bearing retention, M3 patterns, spindle press fit.
- Not run: screw tool access, wire bend radius through the spindle, whole-arm sweeps, firmware keep-out tables, `s2_grid.py`, `harness_loads.py`, print orientation. Curved outer plates joining A to B are not used (would lock R unless slit).
- Interface contract change (W3 ears / floor at |x| 23.3-26.8, z 491.6-493.6 removed for the hand): to be recorded in `cad/interfaces.py` when the hand is adopted; the claw still uses the old contract.
