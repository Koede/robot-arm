# SPEC: vendor mounting patterns, retrieved 2026-11-02 (fork result)
Sources: cubemars.com/goods-1031-AK70-10.html, its 2D drawing PDF (text layer only), emanual.robotis.com XM540-W270, robotis.us HN13-N101, servodatabase.com STS3215. Data: vendor_patterns.json.
| Item | Result | Status |
|---|---|---|
| AK70-10 size, mass, torque, ratio, backlash, connectors | Ø89 x 50.25, 621 g, 8.3 / 24.8 N.m, 10:1, 0.2 deg, A1257WR-S-4P / -3P, XT30PW-M | SOURCED, matches the project |
| AK70-10 drawing callouts | 6-M3, 8-M3 (x2), 7, 10, 3.5, 2 (+0.02/0), 0.5, 11.2 | text only, unassigned |
| AK70-10 flange / rear PCD, depth, protrusion | not readable | PLACEHOLDER |
| XM540 size, mass, torque | 33.5 x 58.5 x 44, 165 g, 10.6 N.m at 12 V | SOURCED, matches |
| XM540 horn set | WB M3x8 x1, WB M2.5x4 x10, thrust washer | SOURCED |
| XM540 horn PCD, frame holes, axis direction, offset, connectors | drawings section not retrievable | PLACEHOLDER |
| STS3215 case 45.2 x 24.7 x 35, ~55 g, 25T spline | converted from inches | DERIVED |
| STS3215 ear holes, shaft offset, horn height | not found | PLACEHOLDER |
Closes: nothing fully. Partly informs P1 (thread is M3 not M4: 6-M3 / 8-M3 callouts) and P2 (horn bolts M3x8 and frame bolts M2.5x4, not M2.5x3). Stays open: P1 PCD/count/protrusion (AK_OUT_N_UNVERIFIED, AK_OUT_PCD_UNVERIFIED), P2 horn pattern and axis direction (XM_HORN_OFFSET_UNVERIFIED), P3, P4, P5 connector positions, P6 (SHAFT_OFF, HORN_D, HORN_P), P8 ear holes.
Contradiction: the project's placeholder 6 x M4 on PCD 60 conflicts with the drawing's M3 callouts. All verified project dimensions are consistent with the vendor pages.
Next step: open the AK70 STEP zip and the ROBOTIS drawings page in a browser (both are images/CAD, not extractable here).
