# SPEC_internal_routing: can the cables and servo leads run inside the joints?  (2026-10-19)
Target: WIRING integration across SHOULDER/UPPER ARM/FOREARM/WRIST. Scripts: cad/axis_corridor.py, cad/axis_offset.py. Data: stl/electronics/axis_corridor.json, axis_offset.json.

## Result
(a) Solids on each joint axis (**W1 corrected below**) (park pose, then 76 legal poses, clearance >= 4 mm): only W1 has a free axis (124 mm). Others: S1 12, S2 16, S3 8, E 28, W2 24, W3 32 mm free runs, broken by AK70/XM540/STS3215 bodies and adapters.
(b) Smallest radius r from the axis at which a line parallel to the axis clears all solids by >= 3 mm:

| joint | hole through housing, park pose | clear over all legal poses |
|---|---|---|
| S1 | 56 | none <= 60 |
| S2 | 56 | 56 |
| S3 | 36 | 56 |
| E | 48 | 52 |
| W1 | 4 | 4 (on-axis) |
| W2 | 24 | 44 |
| W3 | 20 | 32 |

All distances mm. The earlier "10-15 mm off-axis housing holes" estimate was wrong: any channel at E/W2/W3/S1-S3 sits at the housing's outer surface, not near the axis.

## Consequence
- W1: on-axis twist-only routing works now (no loop).
- E, W2, W3, S1, S2, S3: through-routing needs either hollow-bore actuators or a cable race at r = 32-56 mm, i.e. the boot/skin. Actuators here (AK70-10, XM540, STS3215) have no through-bore.
- Decision for Forest: swap to hollow-bore actuators (changes interface contracts, torque margins, mass: reopens S/E/W) or keep actuators and accept boots (<= ~90 g/arm target).

## DESIGN_STATE addendum (paste)
- 2026-10-19: axis_corridor/axis_offset study. W1 axis free (on-axis routing OK). Other six joints blocked; minimum offset channel radius 32-56 mm. Open: hollow-bore actuator swap vs boots (user decision).

## CORRECTION (same day): W1 is NOT free
axis_corridor.py sampled W1 at z = 230 +/- 60 (the elbow/forearm-root region), not at the real W1 stage. `cad/w1_axis_check.py` samples the full forearm (z 230..410, y 57): sts_w1 sits ON the axis at z 302-334, w1_pinion at z 342, sts_w2 at z 378-410; legal-pose clearance goes negative (-10.7 mm at z 310, -13 at z 390). **No joint axis has a through corridor, including W1.** My earlier "run W1 on-axis" was wrong.

## FINAL routing decision (Claude, on Forest's "route the wires and finalize"; hollow actuators not chosen)
- Keep all 7 DOF and all actuators. Cables cross each joint as a free loop (arm_chain results), covered by a soft boot at each of the 7 joints: S1, S2, S3, E, W1, W2, W3.
- Boots: TPU/textile bellows, NOT rigid shells (cannot be collision-checked as rigid; they are soft and ride the loop envelope). Study mass 141 g/arm at the SPEC_fairing_study thickness (S1 33, S2 34, S3 18, E 22, W1 13, W2 11, W3 10). Top-pad pressure +2.49 kPa/kg, margin ~0.3 kPa -> 141 g gives +0.35 kPa = FAIL; at 0.7x wall (~99 g) +0.25 kPa = pass with ~0.05 kPa left. ESTIMATE, not run through harness_loads (wearer mesh stage not rebuilt).
- Gates still open: harness_loads re-run with the boot mass; boot clearance to keep-out (soft, so check loop envelope only); printability/TPU fatigue.
- Option kept open: hollow-bore actuators remove the loops entirely (reopens S/E/W torque, mass, interfaces).

## DESIGN_STATE addendum (paste; supersedes the earlier line above)
- 2026-10-19: axis_corridor / axis_offset / w1_axis_check: no joint has a free through-axis (W1 blocked by sts_w1, w1_pinion, sts_w2); offset channels need r = 32-56 mm. Decision (Claude, after Forest's "route the wires and finalize"): free loops + soft boots at all seven joints, ~99 g/arm at 0.7x wall; top-pad margin ~0.05 kPa (unverified); harness_loads re-run required. Hollow-bore actuators remain an option (reopens S/E/W).

## Wire physicality audit (2026-10-19, cad/wire_audit.py -> stl/electronics/wire_audit.json)
Independent of the original stage A (single joint swept, other joints at 4 base poses). Here ALL joints move together: 16 random poses inside the soft range, firmware-legal (>= 3 mm part-to-part), 12-step continuation from park, rod relaxation (0.8 mm clearance target).
- Anchors sit on real printed links: 4.0-19.0 mm from the host surface, tangents leave into free space (W2 A and W3 A: 9 mm along the tangent is 6.4 / 5.0 mm, i.e. close to the host: check clamp posts).
- Reach: chord |A-B| < cable length at every pose; minimum slack 7.9 % (E), 26-42 % elsewhere.
- In-link runs join consecutive anchors: routed length / straight distance 1.03-1.23 (S1->S2 16.9/16.4, S2->S3 123.1/99.9, E->W1 22.3/20.6, W1->W2 13.0/11.4, W2->W3 43.6/35.1 mm).
- Electrical (arm_electrical.json, original session): 0.32 V drop to the gripper at hold, all wire J <= limits, S2 SF 2.76.
- Combined-pose loop result: S1, S2, W1 pass (clearance 5.1 / 9.6 / 0.8 mm, stretch < 1 %). S3: 2 of 16 poses at 0.8 mm clearance, 7 under the bend-radius target (min 21.8 vs 28.4). **E, W2, W3 fail in some combined poses**: E 1/16 (clearance -1.5 mm), W2 7/16, W3 11/16 (clearance -1.8 mm, bend 2.2 mm, relaxation cannot keep length). These loops were only verified with one joint swept at a time; the wrist and elbow loops need re-search against combined poses (or the poses excluded by a firmware tier).
- Not modelled: the actuator-side connector pigtails (STS3215 / XM540 ports) and the claw slot (PLACEHOLDER position).

## Combined-pose loop re-search (2026-10-19, cad/wire_search_combined.py, wire_fail_poses.py)
Search = arm_wire_search machinery with the joint swept over its HARD range at 12 bases (4 original + 8 random firmware-legal). Found: E 2 candidates, W2 5, W3 7 (stl/electronics/wire_search_{E,W2,W3}_CB.json). Best of each swapped into arm_chain_loops_v2.json and audited on 16 FRESH legal poses (wire_audit.py, LOOPS env):
| joint | loop | result on fresh poses |
|---|---|---|
| E | L 191.6 (was 108.6), A (42.1,-13.7,168) M3, B (55.3,-33.7,268.3) ME | PASS: clearance >= 10.0 mm, bend 28.5 vs 23.9 needed, stretch 0.21 %. Cable +83 mm, boot grows |
| W2 | L 97.5 | FAIL 6/16: clearance down to -2.2 mm. Fails cluster at W1 <= -33 deg with small E (e.g. E 40 / W1 -67, E 8 / W1 -75, E 3 / W1 -33 / W2 39) and one at W1 +31 / W2 47 / E 96 |
| W3 | L 111.3 | FAIL 4/16 (clearance 0.09 mm min, bend 5.9 mm) |
The search overfits its 12 bases; a candidate that passes them does not generalise to W2/W3. Not done: in-link run re-routing for the new anchors (E->W1, W1->W2 runs no longer match: straight distance 118.9 mm vs the old 22.3 mm run), arm_chain DP, hub link, clips, boot re-sizing. S3 loop still marginal (2/16 at 0.8 mm, 7/16 under bend target).
Options for W2/W3: (a) raise the search base set (30+ poses) and k; (b) firmware tier: limit W1 to >= -30 when E < 60 (cuts a standing ROM target, needs Forest); (c) let the sleeve/boot absorb contact (abrasion protected cable; accept clearance to -2 mm with a rubbing sleeve). Recommend (a) then (c); (b) only if both fail.
