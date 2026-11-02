# SPEC: Flat shoulder joint study, 2026-11-01 (SHOULDER, concept only)
Forest: "rethink the shoulder joint itself ... a sleek integrated design that sits flatter to my back". Overrides the ball-styled / concurrent-at-P shoulder decision for this study only. DOF, actuators, passive stops, 5 deg rule kept. Proxy geometry, no collision sweep, nothing pushed.
Depth behind the back (max x + 108): baseline 181 mm.
| ID | Concept | Depth | Coverage of baseline arm directions | Torque | Notes |
|---|---|---|---|---|---|
| J1 | Back-normal S1 pancake + side-hung S2 | 111.5 | 96.3 % | S1 6.94 N.m unbalanced (SF 1.20), S2 6.45 (1.29); both need balancers | non-concurrent, 105.75 mm arm-base circle; singularity near arm-forward |
| J2 | Dual pancake, belt + bevel | 130 | 100 % kinematic, 88.7 % with the tube | S1 3.35 (SF 2.48), S2 3.42 (2.43) | 0.2 deg backlash = 2 mm at tip; exposed gears |
| J3 | Slewing ring S1 + offset S2/S3 | 126.5 | see metrics | see metrics | heaviest (2746 g incl. motors) |
| J4 | Offset clavicle link | 128.8 | 96 % | S1 8.33 (SF 1.0) needs 2nd balancer | S1 lever 670 mm; plate top edge z -16 |
| J5 | Gimbal hoops in a bay | 122.5 (158 swept) | 100 % | S2 SF 1.28 unbalanced | bearing retention unresolved (segmented race) |
| J6 | Differential bevel | 144.8 | 80 % | 3.23 N.m per motor (SF 2.57) | 3.3 mm tip backlash; 230 mm wide |
Common finding: every flat layout puts gravity on S1, so S1 needs a balancer too. Depth floor is set by the AK70 disc (89 mm) plus arm base. Re-run keep-out tables, homology map, harness_loads for any adoption.
