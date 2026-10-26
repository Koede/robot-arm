# Connector concepts: A dual-pin dovetail splice, B bayonet-dovetail quick-connect, C cross-spring hinge

Date 2026-10-26. Files: `cad/connector_concepts.py` (coupons, checks), `cad/connector_calc.py` (hand calc + scikit-fem), `stl/connectors/` (STL + `connector_concepts.json` + `connector_calc.json`). mm, N, MPa. No existing file was edited.

**Read this first.** There is no PA12-CF / PA-CF / TPU 95A datasheet in the repo and none was retrieved. Every material allowable in `connector_calc.py` (dict `PH`) is an **UNVERIFIED PLACEHOLDER**: PA-CF UTS 60 MPa in-plane / 25 MPa across layers, bearing 45, flexural modulus 4000, S-N rule `UTS (1 - 0.09 log10 N)`, TPU E 26 MPa, TPU-on-PA-CF mu 0.5, dowel shear 300, M4 bolt shear 270. They rank the concepts; they are not design values. The cycle counts for C in particular have no source. The first bench step (section Tests) replaces them.

## 1. Coupons and geometry checks

All coupons are parametric in `CLR` (nominal 0.25, variants 0.15 / 0.35 written as `_c025`, `_c015`, `_c035`). Result (`all_checks_pass: true`):

| Check | A | B | C |
|---|---|---|---|
| Each printed part = 1 watertight solid (3 clearances) | yes | yes | yes (PA-CF and TPU versions) |
| Halves interference at 0.15 / 0.25 / 0.35 | 0.0 mm3 | 0.0 mm3 at 0, 30, 60, 90 deg of turn | leaves 0.0, blocks 0.0 |
| Interference at CLR = -0.1 | 48.6 mm3 | 91 mm3 (insert), 191 mm3 (locked) | 0.14 mm3 (leaf vs leaf, slot only 0.1 deep) |
| Other | steel dowels: 5.1 mm3 press overlap in the arm cheeks (designed d2.4 on d2.5), 0 in the tongue | locked male cannot be pulled 1 mm out (27.5 mm3 collision) | |

Caveats: interference is a CadQuery boolean volume of the two solids, not a tolerance-stack; the coupons are straight lug / sector shapes with no chamfers, fillets or teardrop holes. The TPU detent pin is a separate body (not in the interference test).

## 2. Concept A: dual-pin parallel-load splice

**Geometry (coupon, `build_A`).** Strut section 18 x 9 (the pauldron `STRUT`). Core member carries a dovetail tongue (16 long, root 8 wide, 10 deg flanks, tip 13.6) extruded through the 9 mm thickness; the arm member has the matching groove and slides on along z' (the bolt axis). Two steel dowels d2.5 run along w through the seam at 3.5 and 12.5 mm (pitch 9): **press** (d2.4) in the arm cheeks, **run** (d2.6 at 0.25 mm) in the tongue, the same rule as `claw_hand.py` PIN RETENTION. Cheeks are 2.8 mm (outer pin) to 4.4 mm thick. The dovetail locks t and w; the two dowels lock z' and stop the slide-out. A pin that works loose does not release the joint.

**Load case (stated, per the brief).** Limit moment 24.8 N m (`plate_pocket.py`, AK70 peak). Lever = distance from P (robot-frame origin) to the splice frame `at` in `pauldron.json`: **arm_lat 125.8 mm, arm_med 147.2 mm**. Governing splice arm_lat: **V = 24.8 / 0.1258 = 197 N** transverse, shared by two fasteners. This assumes the whole peak torque goes through one splice, which is an upper bound; the plate struts carry the real path. Bound case 2: V acting at the clasp pad F4 (147.4 mm from the splice) puts 29.1 N m into the splice.

| | existing 2 x M4 x 16 half-lap | A, 2 x d2.5 dowels + dovetail |
|---|---|---|
| Fastener shear (V = 197 N) | 11.2 MPa (thread area, single shear), SF 24 | 10.0 MPa (double shear), SF 30 |
| Bearing on PA-CF | 5.5 MPa (4 x 4.5 mm), SF 8.2 | 3.2-4.5 MPa in the tongue, up to 7.0 MPa in the 2.8 mm cheek, SF 6.4 |
| One fastener lost | bearing 10.9 MPa, SF 4.1; the other bolt alone holds a half-lap, but the splice can rotate | pin shear 20 MPa, bearing 8.5 MPa, SF 5.3; dovetail still locks t and w |
| Bound case (29.1 N m) couple per fastener | 2.4 kN on 12 mm pitch, bearing 134 MPa, SF 0.33 | 3.2 kN on 9 mm pitch, bearing 140 MPa, SF 0.32 |
| Section modulus at the seam | 121-243 mm3 (two 4.5 halves, non-composite..composite) | 108 mm3 (8 x 9 tongue neck) .. 243 |
| Strut reference, 29.1 N m | strut itself 120 MPa = 2.0 x placeholder UTS | same |

FE (scikit-fem, P2 plane strain, in-plane tear-out slice, force toward the free edge; peak sampled 1.2-2 r from the hole so it is a lower bound): cheek hole at 3.5 mm edge distance peak/nominal 0.80, at 12.5 mm 0.70; M4 half-lap hole 0.79. No edge-distance problem in either; the slice says nothing about the out-of-plane (z') pin loads, which are the real ones.

**Reading.** At the transverse shear load the existing joint is already 8x over on bearing, so A buys no strength. What A buys: redundancy (no single fastener drops the joint), no nuts or bolt heads on the shell surface, and the dovetail takes t and w loads without the pins. What it costs: pitch 9 vs 12 mm and a tongue neck at 44 % of the strut modulus, so under a moment (bound case) A is slightly worse than the half-lap and both are far over the placeholder allowables, which says that case is not what the splice is for (the strut itself would be at 2x UTS). The other costs: 2.8 mm cheeks at 18 mm strut width (thin for a pressed pin: expect cracking at d2.4 press without reaming), 16 mm lap vs the 24 mm half-lap, and a tolerance-sensitive dovetail that must be assembled along a direction (z') that the bolted shells may not leave free. The 0.5 mm total play at 0.25 clearance is not preloaded by anything, so the joint will rattle unless a shim or glue is added.

**Print.** Stand each member on its 9 x 36 edge (w up). The dovetail extrusion and both pin holes are then vertical, no overhangs, no supports, holes keep roundness; layers lie in the t-z' plane, which is the plane of the splice bending, so layer shear is not loaded. Ream the d2.4 press holes after printing (as in the claw). Fit 256 mm: no issue (36 x 18 x 9 each).

**Risks.** Pressed dowels in 2.8 mm cheeks (split); dovetail flank clearance varies +-0.1 between printers (check with the 0.15 / 0.25 / 0.35 set); creep of the pressed fit in PA; assembly direction; the numbers rest on placeholders.

**Recommendation: CHANGE, not build.** Do not replace the half-lap. Keep 2 x M4 and add the dovetail key (or one dowel) only if a rattle or a lost-bolt scenario is a real requirement. If built, widen the strut at the splice to 24 mm so cheeks are >= 4 mm.

## 3. Concept B: printed bayonet-dovetail quick-connect with TPU detent

**Geometry (coupon, `build_B`).** Male: 30 x 30 x 4 base, d18 stem 10 long, two diametric lugs (annular sectors r 8.5-12, 36 deg = 7.5 mm arc, 3 mm thick) at 4 mm. Female 30 x 30 x 14: bore d18 + 2 CLR, two axial entry slots, an L-channel 90 deg long at lug height (3 + 2 CLR high), closed by a stop wall. Insert at 0/180, turn 90, lugs sit against the stop. A TPU 95A detent pin (d3, domed nose, 0.6 mm protrusion) in a radial hole at 62 deg per side (two pins) is crossed by each lug on the way in and out. The "parallel tongues" are the two lugs, which also balance the load.

**Numbers (placeholders for TPU).** TPU pin E 26 MPa, unconfined axial stiffness k = E A / l = 28 N/mm over 6.5 mm free length, normal force N = 17 N at 0.6 mm (9 % strain). Ramp friction mu 0.5 (placeholder; `claw_hand.py` also lists TPU pad mu as unverified).
- Lock-in (30 deg ramp): 25.7 N per detent, **0.62 N m** to turn in.
- **Unlock (45 deg ramp): 50.9 N per detent, 102 N tangential at r 12 mm, 1.22 N m torsion to release** (hand-turnable, not accidental).
- Axial pull-out of the lugs: root bending of a 3 x 3 x 6.6 mm cantilever with the load across the layers, 0.076 MPa per N; at 12.5 MPa allowed (placeholder, 50 % of the across-layer UTS) the limit is **165 N** (330 N at the placeholder UTS); with the stem axis in the bed plane the limit rises to about 400 N (XY strength).
- Play: 0.5 mm axial at CLR 0.25. A 0.5 mm TPU wave washer under the base would preload the lugs; not modelled.

**Print.** Print the male on its back (stem up) only if 165 N is enough; otherwise stem horizontal with the lug cantilevers in the layer plane and a small tree support under the base. Female: face down on the bed, bore and channel vertical, no supports; channel is a closed cavity so print entry side down. TPU pin separately (0.12 mm layers, 100 % infill, 2 pins per coupon). 

**Risks.** TPU stress relaxation and creep of the detent (the click force falls with time and temperature, UNVERIFIED); lug root across layers; 0.25 clearance gives a loose quarter-turn; chamfers on lug and nose are assumed in the detent numbers but absent in the CAD; heat-set inserts remain better for tiny shell halves, where the quick-connect only adds height (14 mm).

**Recommendation: BUILD for the non-structural shell halves (cowl, boot covers, electronics lids) as a coupon to measure unlock torque; DROP for the clasp / pauldron interface**, where 165 N across-layer lugs and 0.5 mm of play are inadequate for the load path and a loosened clasp is a safety risk.

## 4. Concept C: cross-spring flexure hinge for the claw finger

**Geometry (coupon, `build_C`).** Two blocks (16 x 10 x 6) 12 mm apart, joined by two crossed leaves, each 10 wide, length 14.4 mm, leaf crossing angle 67 deg between them; leaf 1 full width in the middle, leaf 2 split into two 1.5 mm outer strips with a CLR slot either side so the leaves pass without fusing. Monolithic print; PA-CF leaf t = 0.8 (two 0.4 perimeters) and TPU leaf t = 1.2.

**Bending at 30 deg.** Strain eps = t theta / (2 L), stress = E/(1-nu^2) eps (uniform-curvature S bend; leaf stretch ignored). FE check, 3 deg linear: FE 5.9 vs closed form 6.6 MPa (0.89; centroid sampling of 6 elements through the thickness accounts for most of the 11 %).

| Leaf | eps at 30 deg | sigma | Result |
|---|---|---|---|
| PA-CF t 0.8, L 14.4 | 1.45 % | 66 MPa = 1.1 x placeholder UTS | fails on the first stroke |
| PA-CF t 0.5 (not reliably printable) | 0.91 % | 41 MPa = 0.69 x UTS | about 10^3.4 cycles by the placeholder S-N: UNVERIFIED PLACEHOLDER |
| PA-CF t 0.8 | max angle for 1e5 cycles | 15 deg | need L = 29 mm for 30 deg at 1e5 cycles |
| TPU 95A t 1.2 | 2.2 % | 0.7 MPa | strain below the 5 % working limit assumed: UNVERIFIED PLACEHOLDER |
| Pinned (d3 steel, d3.1 hole) | n/a | n/a | no fatigue in this range; 0.1 mm play |

The cycle numbers come from a rule-of-thumb S-N curve, `UTS (1 - 0.09 log10 N)`, which I state as a placeholder with no source; vendor fatigue data for the exact filament and orientation is needed, and printed fatigue scatters by an order of magnitude with road direction, porosity and humidity (PA absorbs water, which lowers E and strength). Do not read "10^3.4" as a prediction, only as "this is in the low-cycle regime".

**Stiffness and stability.** PA-CF crossed pair: hinge stiffness 270 N mm/rad, leaf buckling load 324 N each (fine for the claw). TPU: 6.5 N mm/rad and **7 N** buckling load per leaf, so a TPU flexure cannot carry a finger link load and wobbles sideways; it only works as a return spring or in a hybrid with a rigid stop.

**Print.** Print the hinge axis (y) vertical so the leaves are vertical walls one road thick and the roads run along the leaf length (bending along the roads). Blocks 16 x 10 x 6; no supports if the leaf crossing is printed in a 45 deg orientation (leaf angle 34 deg from vertical, ok). Small bridge across the leaf 2 slot needs 0.2 mm layers.

**Risks.** Stress at the leaf-to-block root (no fillet in the coupon; add r >= 1 mm); parasitic centre shift of a cross spring (the instantaneous axis moves, so the finger path changes); no hard-stop positioning; creep under hold in a gripping posture.

**Recommendation: DROP for the claw finger hinges** as specified (+-30 deg). PA-CF only reaches about +-15 deg at 1e5 cycles with leaves this size, and TPU is too weak. Keep the pins. Consider a cross-spring only for a small-angle, light-load joint (e.g. a +-8 deg compliant tip pad).

## 5. Comparison with the existing joints

| Joint | Where | What carries load | Load capacity (this study) | Redundancy | Print / assembly | Verdict |
|---|---|---|---|---|---|---|
| Half-lap + 2 x M4 x 16 + nuts | pauldron splices | bolt shear and bearing on the 4.5 mm halves | 197 N transverse: bearing SF 8.2 (placeholder) | one bolt lost: SF 4.1, joint can rotate | flat, supports none; needs nuts and hex access | keep |
| Pressed/running dowel (claw) | claw hinges, P/Q/J/R/lug pins | pin shear and bearing | not recomputed here; pin d2.5 shear SF 30 at 197 N | pin walks only if both press and capture fail | reamed holes; captured by the next layer | keep |
| Heat-set M3/M4 inserts | shell halves, pads | thread pull-out in the boss | not evaluated (no vendor pull-out data in repo) | many screws | simple; insert depth 1.5 d | keep |
| A dual-pin dovetail splice | pauldron | dovetail t and w, dowel shear z' | same shear SF 6.4-30; worse under moment | one pin loose: still held, SF 5.3 | on edge, no supports, ream; 2.8 mm cheeks | change (not build) |
| B bayonet + TPU detent | shell halves, lids | lug root across layers, detent | 165 N pull-out, 1.2 N m to unlock | two lugs, two detents | needs support or on-side print; TPU pin | build (shells), drop (clasp) |
| C cross-spring hinge | claw fingers | leaf bending | PA-CF: first-stroke failure at 30 deg; TPU: 7 N buckling | none: single leaf break = hinge lost | monolithic, vertical leaves | drop |

## 6. Tests to replace the placeholders

1. Print 3 sets of A and B at CLR 0.15 / 0.25 / 0.35 in the final filament; record the fit (slide force, rattle).
2. Tensile-test 5 XY and 5 Z coupons to get UTS XY, Z, bearing and flexural E; replaces `PH` and rerun `connector_calc.py`.
3. B: spring-scale pull of the locked coupon (axial), torque-wrench unlock; log vs age for the TPU relaxation.
4. C: cycle a PA-CF leaf coupon to +-15 deg for 1e5 cycles; stop at first crack.

Nothing here edits `DESIGN_STATE.md`; if the connector decisions are adopted, record them there (Forest's call, per `CLAUDE.md`).
