# SPEC: WRIST ROTARY COUPLING (hand-to-arm), draft v2, 2026-10-30, IN DESIGN, not locked

Subsystem: WRIST + GRIPPER. Replaces the W3 "H" yoke (two 3.5 mm ears + floor plate inside `palm_tray`) for the HAND end effector. Claw untouched.
Scripts: `cad/wrist_rotary.py` (parts -> `stl/wrist_rotary/`), `cad/verify_wrist_rotary.py` (-> `stl/wrist_rotary/verify_wrist_rotary.json`), `cad/render_wrist_section.py` (-> `wrist_section_{xz,yz,flexure}.png`).

## Forest's direction (2026-10-30)
1. Plane-to-plane coupling; the interconnects route the wires and are structural. 2. The plane also rotates the wrist in-plane. 3. Chosen: **keep W3 and add the rotary (8th powered DOF R, one more STS3215)** (breaks the 7-DOF / actuator-count standing decision; Forest's call). 4. No yoke; perpendicular tubes join the planes and carry the wires; W3 via a compliant design; radially curved planes may join outer regions. 5. Chosen motion layout: rotating planes + flexure W3.

## Layout (robot frame, mm; W3 axis x through (y 57, z 455); R axis = hand axis x 0, y 57, along z)
| item | value |
|---|---|
| carrier plate (plane A) | z 493.8-495.8, flat at r >= 38.8 from the W3 axis: **3.0 mm** swept clearance to the W3 servo corner (the old floor at 491.6 had 0.8 mm) |
| horn paddle | one curved teardrop arm at x -26.8..-23.3 from the W3 horn boss (r 16) into the plate edge; horn bore d6 PLACEHOLDER |
| W3 support (replaces the second ear) | cross-flexure on the W3 axis at +x: 2 spring-steel leaves 0.1 x 12 x 8 mm at +-45 deg, between a stub at x 23.3..26.8 and a fixed anchor on the W2 cradle (x 34.8..38.8, 2 x M3 PLACEHOLDER) |
| R bearing | 6808-2RS 40x52x7 (same part as W1), z 496-503, inner race on the carrier hub r20 |
| R gear | rotor 52T m1.25 (pitch R 32.5) driven by a 16T pinion (R 10), ratio 3.25, centre distance 42.5; R servo STS3215 vertical below the plate at x -76..-31 |
| wire path | hollow brass spindle tube OD 12 / ID 9, z 493.8..513, perpendicular to both planes; hub bore + plate hole feed it from below, rotor neck (r 15..19) is the wire cavity above; top bushing on 3 spokes (second support 12 mm above the bearing). Service loop for +-170 deg. |
| mating plane (plane B) | z 515.6 = 491.6 + DZ 24; rotor flange r 11..19 (4 x M3 inserts, PLACEHOLDER pattern) |
| hand lift DZ | **24 mm**: wrist-to-tip 219 -> **243 mm** (open hand) |

## Verification (`verify_wrist_rotary.py`)
| gate | result | status |
|---|---|---|
| Parts valid, watertight, positive volume | carrier, flex_anchor, rotor, pinion, palm_tray_wrist | pass |
| W3 sweep -35..+40 deg, everything beyond W3 vs the fixed arm | min distance 3.0 mm, 0 collisions | pass (at the 3 mm gate) |
| R sweep -170..+170 deg, poses open / chuck / point / victory vs the carrier stack | 0 collisions; palm + digits to carrier >= 5.8 mm (open, point), 10.1 (chuck), 6.5 (victory) | pass |
| 👍 fist (index MCP 95 deg) | **collides** (index_dist / index_mid vs hub, rotor web and neck) | **FAIL** |
| Index curl in the fist pose | free up to **65 deg MCP**; hit at 70 deg (index_dist vs rotor) | firmware limit or +lift |
| Cross-flexure | k = 0.0008 N.m/deg; strain 0.38 % at 35 deg (0.1 mm spring steel, 725 MPa vs ~1600 yield); leaf buckling 117 N vs ~10 N axial est. | pass (estimate) |
| W3 tip moment 1.96 N.m at 2 g (0.5 kg at 200 mm): couple over a 50 mm horn/flexure spacing = 39 N axial on the horn bearing | STS3215 thrust rating unknown | OPEN (PLACEHOLDER) |
| R stage torque | 2.94 N.m x 3.25 x 0.9 = 8.6 N.m available vs ~0.35 N.m estimated hold | pass |
| Added mass | 176 g (R servo 55, bearing 46, carrier 33.5, rotor 23.8, hardware ~16) | harness top pad est. 15.3 vs 14.7 kPa **FAIL** (slope 2.49 kPa/kg, `harness_loads.py` not re-run) |

## Open items
- Fist: +10 mm more lift (reach 253 mm) or limit index curl to 65 deg MCP. Forest's decision.
- Harness top pad over limit: lighter R servo class, lattice the palm tray, or accept the re-fit.
- PLACEHOLDERS: STS3215 horn / shaft offset / servo ears, W3 idler-side bearing capacity, spring-steel leaf vendor and clamp, gear tooth profile (trapezoid, needs involute), bearing retention, rotor-to-hand insert pattern, spindle tube press fit.
- Not run: tool access for the plate and flange screws, wire bend radius through the spindle, whole-arm sweeps, firmware keep-out tables, `s2_grid.py`, `harness_loads.py`, print orientation.
- Curved outer plates joining plane A to plane B at their rims are NOT in v2: a shell there would lock the R rotation unless slit into torsion leaves (range then about +-30 deg).
