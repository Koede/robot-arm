# SPEC: MJ11 electrode leads, liner shear, WING interface update (SENSING / HARNESS / WING, rev 2026-11-03)
Target: SENSING + HARNESS + WING, cross-subsystem integration on the adopted J1 = MJ11 blade-seated pancake (`specs/SPEC_j1r2_blade_seat.md`, DESIGN_STATE 2026-11-02d).
Status: IN DESIGN. Scripts ran and the numbers below come from them; bench items are not run. Unattended run (Forest asleep): decisions that change geometry or limits are laid out in section 5, not taken, except the liner geometry edit in 1.4 (slots and jack plate), which is the requested "modify the liner channels if needed" and is reversible (`mj11_leads.slot_polys`).
Units: mm, kPa, N, J. Robot frame (+x away from the back, +y right, +z up); plan view = (y, z). Pad centre (-60, -190), liner footprint R66 cut at y -96 (rail edge -100), skin-side = -x face of the liner.
Code: `cad/j1r2/mj11_leads.py` (routing model + slot geometry), `mj_concepts3.py` (liner, jack plate), `mj11_shear.py`, `mj11_jack_scan.py`, `mj11_leads_render.py`, `mj11_leads_plan.py`, `cad/interfaces.py`. Data: `stl/j1r2_mounts/MJ11/{leads,shear,jack_scan,metrics}.json`. Run order: `mj11_leads.py` -> `mj_eval.py MJ11` -> `mj11_shear.py` -> renders (J1R2_ROOT work root, docs/ENV_SETUP.md).

## 0. Result in six lines
1. Only two atlas pairs lie under the liner at park: infraspinatus and teres major. The rhomboid pair slides into the liner's medial edge near full elevation (25 mm2 at the atlas travel, 481 mm2 at the 62 mm design travel). Mid trap, lower trap and latissimus never reach the liner. Serratus is flank skin 85 to 100 mm deeper in x than the liner, not under it.
2. The three radial lead channels of 2026-11-02 do not work and are retired: the skin moves lateral-and-down (psi 297 to 327 at these sites), so psi 225 is 90 to 104 deg off the motion, psi 270 25 to 60 deg, and the pairs sit 33 mm off the psi 315 axis. Replaced by three skin-side relief slots aligned with the skin motion (infraspinatus, teres major, rhomboid entry), 5.5 mm deep, which also keep the electrodes out from under foam load while they slide.
3. The slots remove 36 % of the liner footprint (4063 of 11347 mm2): same-load liner pressure x1.56, and the frame model now gives liner increments +0.98 / -0.59 kPa at park and +2.80 / -1.95 kPa at reach (were +0.66 / -0.60 and +2.21 / -1.98). The 2.2 kPa figure no longer holds; 2.8 kPa does.
4. The lateral half of the dish rim (psi 290 to 80) is swept by the S1' / S2 stage (0.0 to 0.3 mm), so the jack cannot sit where the leads would naturally leave (+d side). The jack plate is at psi 240 (lower-medial rim; stage 8.9 mm, rail 4.3, spine bridge 19.6, skin 18.7, all gates met). Leads leave along -d and run along the liner edge: 119 mm (teres major) and 210 mm (infraspinatus) to the jack.
5. Shear: with an assumed PTFE-fabric / base-layer pair of mu 0.1 to 0.2 and a 4.3 kPa preload, skin shear is 0.7 to 2.0 kPa (7 to 14 N total over 7284 mm2), heat is negligible (0.17 K worst). The real risks are not the shear number: (a) the dish is rigid and bolted, so liner pressure is displacement-controlled at 19 to 33 kPa per mm of misfit, and the model already sees a 1.9 mm skin-mesh mismatch; (b) sweat can raise the slip pair's friction to the skin's own (0.3 to 0.7), moving the slip onto the skin.
6. WING contract and S1' flange are recorded in `cad/interfaces.py` (new file; it was missing) and DESIGN_STATE section 11b. The worst M4 bolt (313 N static, 626 N with x2 dynamic) fails SF 2 if the typical 1 kN insert pull-out holds: bench test in section 3.3.

## 1. Electrode leads

### 1.1 Which sites are under the dish (atlas rev c, `stl/shoulder/emg_atlas.json`)
Skin motion: rigid-blade bound of `muscle_atlas.py` (pivot y 14.75, z -116.22), used for direction; magnitude from the atlas (`travel_mm`, blended) and from the SPEC_flex_electrode_leads s2.3 design table. Discs R6 plus 1.5 mm margin.

| Pair | centre (y,z) | carrier | motion dir (psi) | travel atlas / design (mm) | under liner at park | swept area under liner, atlas / design |
|---|---|---|---|---|---|---|
| infraspinatus | (-36.8, -167) | base layer | 314.6 | 31.8 / 70 | yes, both discs (24.6 and 41.3 mm inside the edge) | 1475 of 1475 / 2034 of 2670 mm2 |
| teres major | (-55.4, -223) | base layer | 326.7 | 56.3 / 96 | yes (28.9 and 33.5 mm inside) | 1425 of 2199 / 1425 of 3459 mm2 |
| rhomboid | (-121, -186) | base layer under the rail | 297.1 | 26 / 62 | no; footprint touches the medial edge after 22.5 mm | 25 / 481 mm2 |
| trap mid | (-126, -136) | base layer under the rail | 278 | 18 / 56 | no | 0 / 0 |
| trap low | (-130, -233) | base layer under the rail | 308.9 | 16.7 / 76 | no | 0 / 0 |
| lat dorsi | (-84, -282) | base layer | 329 | 44.5 | no (moves away) | 0 |
| serratus | (-10, -239) | flank panel | n/a | none (flank) | no: skin x -214 vs liner skin x -110 to -131, only the (y,z) projection reaches the rim | n/a |

Not under the dish by construction: trap up, delt ant, pec clav (clasp pads on stable zones), delt mid / post (sleeve).
Design travel (70 / 96 mm) is the SPEC table with unmeasured follow fractions (T14-3b); the slots are cut for atlas x 1.25 and the 481 mm2 rhomboid overlap at 62 mm is not covered. Re-cut `slot_polys(mult=...)` when T14-3b gives numbers.

### 1.2 Standing-rule check: electrodes in the compression base layer, nothing rigid on them
- The dish is rigid PA-CF but its inner face is skin + 8 mm. Each slot is 5.5 mm deep from the skin face: electrode stack 3.5 mm (PLACEHOLDER: no electrode part selected, 2 mm snap on a textile disc) + 2.0 mm margin for the fit error (mj_eval skin-mesh mismatch -1.9 mm) leaves a 2.5 mm foam roof. Liner load on any electrode = 0 while the margin holds; contact pressure is the garment's (SPEC_flex_electrode_leads s5: collar <= 1.5 kPa on the two medial discs, <= 2 kPa elsewhere).
- The electrode slides, so the relief is a swept slot along its own motion, not a pocket (a 20 mm pocket would be left behind after 30 to 56 mm of travel).
- Still contradicts the 2026-10-23 decision (rail removed over the electrode zone): MJ11 keeps the paraspinal rail (y -138 to -100, rigid, bolted to the pad) and the atlas flags trap mid, trap low and rhomboid as under the rail. That decision was never built; MJ11 needs the rail for its four bosses. Decision D1 in section 5.

### 1.3 Routing (`mj11_leads.lead_path`, `leads.json`)
- The lead is stitched or laminated to the base-layer panel from the pair centre and slides with it. The liner can only be cleared by the lead if the lead lies along the skin motion, so each corridor (8 mm wide, same relief depth) is the line through the pair centre along the motion, drawn on the -d side (the +d side exits into the lateral half of the rim, which the arm stage sweeps).
- Exit and path to the jack plate (psi 240, rim R66, jack point 63 mm from the pad centre):

| Pair | in-liner run to the liner boundary | boundary run to the jack | path incl. 15 mm header offset | lead slack, atlas x 1.25 / design x 1.25 | S-loop half-turns (R18) |
|---|---|---|---|---|---|
| infraspinatus | 57.2 mm, exit at (-77, -126) psi 105 | 135.0 mm (top edge to the corner (-96,-135), then down the 4 mm dish-rail gap) | 210 mm | 39.8 / 87.5 mm | 2 / 5 |
| teres major | 48.5 mm, exit at (-96, -196.5) on the medial cut | 52.3 mm down the gap | 119 mm | 70.4 / 120 mm | 4 / 6 (216 mm axial) |

- Bend radius: lead OD 1.6 mm (assumption), >= 10 x OD = 16 mm. The route follows the liner edge (R66) or the straight gap; the corner at (-96, -135) needs a 16 mm fillet (turn 66 deg). Serpentine half-turn R18 absorbs (pi - 2) R = 20.5 mm each (SPEC s2.3).
- Channel size: 8 mm wide x 5.5 mm tall corridor carries one 1.6 mm lead; two leads (infraspinatus, teres major) never share a corridor, so width is not binding. Height is the slot depth (5.5 mm) against a 1.6 mm lead plus its 0.5 mm laminate.
- Strain relief: the lead is clamped at the jack plate (2 mm bead of silicone over the last 10 mm, header recess) and again 20 mm before it; at the panel end it is stitched with a 3 mm zig-zag over the last 30 mm before the electrode snap, so the stitched section moves with the skin and the free span carries the slack.
- Service loop: the 2026-11-02 assumption that loops sit "beside the channel" does not hold: an 8 mm corridor cannot hold an R18 serpentine, and the 4 mm dish-rail gap cannot either. At the atlas travel the slack fits as a stretch-lead segment inside the 119 / 210 mm paths. At the design travel (teres major 120 mm) it does not fit without a loop of 216 mm x 36 mm on free skin, and the medial side has none (rail zone, electrodes). OPEN, decision D3.
- The electrode moves away from the jack (+d), so the free span is shortest at full elevation: the lead is installed at park with 1.25 x travel of excess.
- Separation from motor and power wiring (>= 50 mm, SPEC_flex_electrode_leads s2.2): the arm cable run is on the other side of the dish plane (x > -91.5); the jack plate is at x -103.5 to -98.5. The 50 mm rule vs the AK70 phase leads and the 24 V bus is NOT checked (cable run not designed).

### 1.4 Geometry changed (`mj_concepts3.py`)
- `liner_mesh`: the three radial through-channels are replaced by `mj11_leads.slot_polys()` (3 slots, 5.5 mm from the skin face, height-field cut, watertight). Liner mass 9.9 -> 8.3 g (foam 0.12 g/cm3 assumed), jack plate 2.0 g (PETG-CF), mount mass 186.7 -> 187.1 g.
- `jack_plate_mesh`: new part `jack_plate`, PETG-CF, 26 x 12 x 5 mm at psi 240, x -103.5 to -98.5, PLACEHOLDER 16-way 1.27 mm-pitch header footprint (part and pinout not selected).
- `mj11_frame`: lattice nodes inside slots are skipped (the foam is not there).
- Jack plate position scan (`jack_scan.json`, `mj11_jack_scan.py`, plate 5 mm thick, R62 to 74, width 26 and 38, psi 0 to 360): stage clearance 0.0 to 0.3 mm for psi 290 to 70 (3.3 at 80), 6 to 8 mm for psi 90 to 140, 10 to 20 mm for psi 150 to 230; skin gap >= 12 mm except psi 330 and 350 where the wearer's arm is in the way (-34 mm); rail and spine bridge >= 3 mm only for psi 235 to 285 (width 26; width 38 from 240). All four gates pass for psi 235 to 285 (stage 7.3 to 11.8 mm); psi 240 is the first of that sector and the shortest lead path.
- Gates (mj_eval MJ11, after this change): skin rule violations none; no wearer ROM first contact; stage clearance (hard ranges): S1' group 7.5 mm overall (clasp bracket, unchanged), jack plate 8.9; arm vs dish 5.7, vs liner 7.1, vs jack plate 20.0; clasp bracket vs arm 0.0 mm in this sampling (the 2.0 mm of 2026-11-02 inside the passive box) remains the open item; tip sag park / reach 0.84 / 1.92 mm (was 0.85 / 1.81).

## 2. Liner shear, friction, heat, abrasion (`mj11_shear.py` -> `shear.json`)

### 2.1 Model
The slip face (PTFE fabric on the liner) slides on the outer face of the compression base layer, not on the skin. The base layer stays on the skin as long as mu_slip < mu_skin; skin shear is then tau = mu_slip x p, carried through the base layer. If mu_slip rises to the skin's own (sweat), the slip moves to the skin: blister / abrasion regime.
Normal load: p = p0 + dp. dp from `run_frame` (liner springs, E_FOAM 0.15 MPa lumped foam + tissue): max +2.80 kPa (reach), min -1.95 kPa; x2 dynamic factor (as the bolt loads). Footprint 7284 mm2. p0 (strap preload) is not computed anywhere in the project: cases 2, 4.3 (the within-17 mm band budget of SPEC_flex_electrode_leads s6) and 8 kPa.
Lift-off: dp min is -1.95 kPa, -3.9 kPa with x2: p0 below 3.9 kPa lets the liner leave the skin in the dynamic case.

### 2.2 Friction coefficients (searched this session; sources below)
- PTFE: no value found for PTFE fabric against a knit base layer. Veijgen 2013: friction on skin was lowest for PTFE among stainless steel, aluminium, PE and PTFE (median skin static mu 0.52, dynamic 0.36 over 450 measurements). NICE HTG355: a low-friction fabric (Parafricta) mu 0.2 against standard textiles 0.3 to 0.7. Jayawardana 2017: overhydration of the stratum corneum triples skin-fabric friction.
- Assumptions used (flagged, to be replaced by T-L1): mu_slip 0.05 / 0.10 / 0.20 / 0.30; the project's PTFE ~0.1 is the optimistic point, 0.2 the benchmark of a commercial low-friction fabric. mu_skin (base layer on skin) 0.3 to 0.7 dry.

### 2.3 Results (liner footprint 7284 mm2; p0 = 4.3 kPa; full table in `shear.json`)
| case | p (kPa) | N (N) | mu_slip | shear force (N) | skin shear (kPa) | energy per stroke 56 / 96 mm (J) |
|---|---|---|---|---|---|---|
| static (p0 + 2.8) | 7.1 | 51.7 | 0.05 / 0.10 / 0.20 / 0.30 | 2.6 / 5.2 / 10.3 / 15.5 | 0.35 / 0.71 / 1.42 / 2.13 | 0.15 to 0.87 / 0.25 to 1.49 |
| dynamic peak (p0 + 2 x 2.8) | 9.9 | 72.1 | 0.05 / 0.10 / 0.20 / 0.30 | 3.6 / 7.2 / 14.4 / 21.6 | 0.49 / 0.99 / 1.98 / 2.97 | 0.20 to 1.21 / 0.35 to 2.08 |
Other p0: 2 kPa -> p 4.8 static; 8 kPa -> 10.8 static (tau 0.5 to 3.2 kPa).
- The shear force on the dish is <= 21.6 N: about 5 N shear per bolt and +9 N on the worst bolt from the moment (lever 25 mm over a 60 mm bolt pair): +3 % on 313 N. Not binding.
- Heat: P = F v with v 10 mm/s typical and 37 mm/s worst (96 mm in 2.6 s at the 0.5 m/s tip cap); worst case (mu 0.3, dynamic peak) 0.8 W, 110 W/m2, skin temperature rise 0.17 K by semi-infinite conduction (k 0.37 W/m/K, rho 1100, c 3400: textbook values, assumption). Negligible against the foam's insulation and sweat accumulation under 7284 mm2 of closed cell foam.
- Abrasion: 500 strokes per day (assumption) x 56 to 96 mm = 28 to 48 m per day, 10 to 18 km per year of sliding on the slip face and the base layer.
- Skin-shear risk: no numeric blister threshold was found (the blister-prevention source says "we have no measure of it" and cites Naylor 1955 for the stiction peak). So there is no pass / fail number here. Relative scale: 1 to 2 kPa is the shear of an ordinary compression garment (collar 1.5 to 2 kPa x mu_skin 0.5 to 0.7; same order, not a sourced tolerance). The drivers that matter: static / dynamic ratio (skin median 0.52 / 0.36 = 1.44: the first stroke after rest is the peak), sweat (x3 on skin-fabric), and a fit-driven pressure rise.
- Fit sensitivity (the finding that matters): the dish is rigid and bolted to the rail, so liner pressure = k x misfit, k = 33 kPa per mm for Plastazote LD33 (66 kPa at 25 % over 8 mm, Zotefoams datasheet) or 19 kPa per mm for the model's 0.15 MPa. 4.3 kPa is 0.13 to 0.23 mm of compression; the mj_eval skin-mesh mismatch is -1.9 mm (63 or 36 kPa). The window between no-lift-off (3.9 kPa) and a 10 kPa peak is 0.12 to 0.5 mm of misfit. A printed dish cannot hold that against a body that moves 56 to 96 mm and breathes. Pressure has to be force-controlled (D2).

### 2.4 Recommended grades (PLACEHOLDER: not verified, nothing ordered)
- Foam: softer than Plastazote LD33. LD33 (33 kg/m3, 66 kPa at 25 %, 133 kPa at 50 %, compression set 9 % then 4 %, water absorption < 1 %: Zotefoams datasheet) is stiff for a 8 mm displacement-controlled layer; the project's 0.12 g/cm3 is stiffer still. Target secant <= 10 kPa per 25 % (k <= 5 kPa per mm): cellular urethane or very soft PE (no datasheet retrieved). Keep closed cell, < 1 % water, skin-contact grade, 8 mm.
- Slip fabric: PTFE-coated woven (or ePTFE laminate) 0.1 to 0.3 mm, bonded to the foam on the base-layer side, edges sealed against fraying; benchmark and fallback: a commercial low-friction medical fabric (mu 0.2 per NICE HTG355).
- Base-layer outer face: smooth nylon / elastane tricot (no loop pile). Pair to be chosen on T-L1.

### 2.5 Bench test plan (all NOT RUN)
- T-L1 slip pair: sled friction, PTFE fabric on the base-layer fabric, dry and artificial-sweat wet, p 2 / 5 / 10 kPa, v 10 and 40 mm/s, 100 cycles over 96 mm. Accept mu_k <= 0.2 dry, <= 0.3 wet, mu_s / mu_k <= 1.2.
- T-L2 skin side (Forest as subject, stop on redness): force-gauge sled with the base layer on the back skin at 5 and 10 kPa dry and after 20 min of sweating; accept: the base layer does not slip on skin (mu_skin >= 1.5 x mu_slip measured in T-L1).
- T-L3 foam: compression-deflection 0 to 30 % and creep 4 h at 5 kPa; fixes k and the preload mechanism.
- T-L4 wear: 20,000 strokes over 96 mm at 10 kPa; mass loss and mu drift; accept mu +/- 25 %, no fabric breakthrough.
- T-L5 leads: 5000 strokes (T14-3d) over the design travel with the chosen loop; resistance change < 10 %, no open circuit.
- T-L6 climate: 1 h walk and arm raises, thermistor + RH under the liner; report dT and RH; sweat pooling in the slots.
- T-L7 electrode in slot: electrode impedance and artefact in the slot against a non-slot site; pressure film at the slot edges.

## 3. Interfaces (WING contract change; recorded in `cad/interfaces.py`, DESIGN_STATE 11b)

### 3.1 WING <-> SHOULDER J1 (MJ11)
Retired: bridge, tongue, chassis plate (and J1 rev 2 riser, standoffs, fairing lap). Kept: paraspinal rail, spine bridge.
Four new M4 heat-set insert bosses (5.6 mm OD x 4 mm) on the rail rear face: (y, z) = (-113, -160), (-105, -160), (-113, -220), (-105, -220). Rail rear face x -102.32 (z -160) and -105.76 (z -220); pad plane x -97.5; spacers D10 4.82 and 8.26 mm (spec: 4.8 / 8.3); M4 x 20 SHCS. Boss wall, hole diameter and height are PLACEHOLDER (not designed). Loads: worst bolt 231.5 N park, 313.2 N reach (x2 dynamic 626 N); typical insert pull-out 1 kN UNVERIFIED -> SF 3.2 static, 1.6 dynamic.

### 3.2 S1' AK70 flange pad pattern: PLACEHOLDER list (do not print)
Position only: 6 x M4 on 60 PCD on the pad rear face (x -91.5) concentric with (-60, -190), motor R44.5 inside the pad R66. Unverified: AK70-10 output flange pattern (thread, depth, phase), pilot diameter and depth, housing and body length behind the pad, cable exit side and connector, bend radius of the phase and encoder cables, fastener length and dowel, S1' stop collar and posts on the new pad.

### 3.3 Bench test, M4 insert pull-out in PA-CF (`interfaces.BENCH_M4_INSERT_PULLOUT`, NOT RUN)
Real boss geometry, n >= 10 per condition; variables: pull direction vs layers, hole dia nominal and +/- 0.2, boss wall 2 / 3 / 4 mm, dry and conditioned (23 C 50 % RH two weeks; 24 h soak), 23 C and 50 C. Tests: pull-out 5 mm/min, torque-out (M4 2.5 N.m target), cyclic 0 to 313 N 100,000 cycles then pull-out, creep 313 N 24 h at 50 C, combined 313 N axial + 40 N shear, 5 install / remove cycles. Accept: mean - 3 sigma >= 1.25 kN (SF 2 on 626 N), torque-out >= 5 N.m, creep < 0.1 mm, residual >= 80 %. Fallback: 2 inserts per position, thicker boss, through-bolt with nut plate.

## 4. Gates run (this change)
| Gate | Result |
|---|---|
| skin rule per zone (>= 5 mm rail, 20 blade for rigid parts) | none violated; dish 5.2 mm, jack plate 18.7 / 19.0, liner 0 by design (-2.0 fit mismatch) |
| arm stage vs new parts (S1' -95..35, S2 -30..80) | jack plate 8.9 mm (M1) / 20.0 (M2); dish 5.7, liner 7.1; clasp bracket 0.0 (open since 2026-11-02, unchanged by this work) |
| wing rail / spine bridge vs jack plate | 4.3 / 19.6 mm (>= 3) |
| wearer ROM first contact | none |
| liner pressure increments | +0.98 / -0.59 (park), +2.80 / -1.95 kPa (reach); the 2.2 kPa figure is superseded |
| bolt loads | 231.5 / 313.2 N, unchanged |
| meshes | liner (8.3 g) and jack plate (2.0 g) built by mj_eval, mount 187.1 g |
| lead bend radius >= 16 mm | met on the edge arc and straight gap; the corner at (-96, -135) needs a 16 mm fillet in the lead |
| lead separation >= 50 mm from motor wiring | not run (cable run not designed) |
| harness_loads / body_loads | not run (scripts not in the repo) |
| renders looked at | yes: `renders_j1r2/mounts/MJ11_leads_{skin_front,skin_up,skin_low,plan}.png`, `MJ11_rear34.png` |

## 5. Decisions for Forest (options, not taken)
- D1 Rail over the electrodes. The 2026-10-23 decision removes the rail over trap mid / low / rhomboid; MJ11 bolts into that rail. Options: (a) keep the rail as is (rigid over three electrode pairs, contradicts the standing rule for those sites); (b) shorten the rail to z -140..-240 and bolt only there, windows over the pairs (needs a re-run of harness_loads / body_loads, the top pad has 2 % margin); (c) move the four bosses to a rail tab on the still strip medially (y <= -121).
- D2 Liner preload mechanism. Options: (a) soft foam plus spring or elastomer standoffs in the four bolts (force-controlled, 0.5 to 1 mm stroke); (b) a strap-loaded dish floating on the rail (adds one DOF, needs a stop); (c) keep rigid and size by a fitting shim set per wearer (cheapest, fragile). Mass and clearance not computed.
- D3 Service loop. Options: (a) accept the atlas travel and carry slack as a stretch lead inside the 119 / 210 mm paths; (b) add a second jack plate at psi 105 to 120 beside the clasp bracket (stage 6 to 8 mm), shortening the infraspinatus path to 70 mm; (c) place the loop on the lateral skin outside the liner (needs the lateral half free of the arm: it is not).
- D4 Liner area. The slots remove 36 % of the liner for electrode relief. Option: narrow slots to the disc width plus 3 mm (area 25 % smaller), or drop the rhomboid notch; trade is electrode freedom vs liner pressure (x1.56).
- D5 Whether 1 kN insert pull-out is acceptable before T-M4 runs (SF 1.6 dynamic < 2).

## 6. Open items
1. T14-3b skin follow fractions (they set slot length, slack, rhomboid overlap).
2. Electrode part and snap height, header part, lead product, pinout (all PLACEHOLDER).
3. Foam and fabric grades, T-L1 to T-L7.
4. Boss design and T-M4.
5. 50 mm separation from the AK70 phase leads; cable run; fairing re-fit; full sweep + tier tables + firmware keep-out; harness_loads / body_loads; balancer / torque-gate decision (unchanged from 2026-11-02).

## Sources (retrieved 2026-11-03)
- Veijgen NK, Masen MA, van der Heide E. Relating friction on the human skin to the hydration and temperature of the skin. Tribology Letters 49:251-262 (2013). https://link.springer.com/article/10.1007/s11249-012-0062-1
- NICE HTG355, Parafricta bootees and undergarments, the technology (friction 0.2 vs 0.3 to 0.7). https://www.nice.org.uk/guidance/HTG355/chapter/2-the-technology
- Jayawardana K, Ovenden NC, Cottenden A. Quantifying the frictional forces between skin and nonwoven fabrics. Frontiers in Physiology 8:107 (2017); skin-fabric friction triples on overhydration. https://www.frontiersin.org/journals/physiology/articles/10.3389/fphys.2017.00107/pdf
- Zotefoams Plastazote LD33 datasheet (December 2017). https://zotefoams.com/wp-content/uploads/2016/03/LD33-December-2017.pdf
- Blister-prevention.com, static vs dynamic friction (no numeric threshold; cites Naylor 1955). https://www.blister-prevention.com/blogs/treatment/blog-static-friction-versus-dynamic-friction
- Not found: a numeric skin-shear blister threshold; a friction coefficient for PTFE fabric on knit textile (CS Hyde page gives none).
