# SPEC: MJ11 electrode leads, liner shear, interface contracts (SENSING / HARNESS / WING, 2026-11-03)
Target: MJ11 blade-seated pancake (adopted 2026-11-02, `SPEC_j1r2_blade_seat.md`). Status: IN DESIGN. Units mm, kPa, N, K. Robot frame, right arm.
Code: `cad/j1r2/mj11_liner_loads.py`, `mj11_leads.py`, `mj11_shear.py`, `mj11_leads_render.py`, liner trenches in `mj_concepts3.py` (patched), `cad/interfaces.py` (new). Data: `stl/j1r2_mounts/MJ11/{liner_pressure,leads,shear}.json`. Renders: `renders_j1r2/mounts/MJ11_leads_*.png`.
Workspace: all numbers rerun on branch `claude/mj11-leads-interfaces-2026-11-03` (mj_eval MJ11 refreshed after the trench change).

## 0. Result in six lines
1. Only **infraspinatus** and **teres major** electrodes lie under the dish/liner (4 of 36 electrodes with a back-of-blade position). Rhomboid, mid and lower trapezius stay under the wing rail (y < -96), lat dorsi is below the dish, serratus and the front sites are off the footprint.
2. **Infraspinatus can be routed**: channel psi 225, inlet r 18, lead-length variation 19 mm (worst sign), slack 24 mm = 1..2 accordion half-turns; bend radius and channel width fit.
3. **Teres major cannot**: it travels 56 mm through the footprint and crosses every candidate inlet; no inlet leaves the minimum electrode-to-inlet distance above the folded accordion length. Forest decides (section 2.5).
4. The lead trenches must all end in **one fixed exit at the rail edge**; psi 270 / 315 exited onto moving skin. A rim collector trench (r 56, psi 315 to 218) was added; the liner loses 13.8 % of its area (mass 9.4 g, was 9.9 g).
5. **Electrode contact pressure does not stay in the 1-2 kPa band** with the present liner: infraspinatus sees 0.9 to 2.4 kPa at preload 1.5 kPa (swing 1.4 kPa from the mount's compliance). Softer foam (about half) or preload 1.0-1.25 fixes it on paper. Lift-off of the liner at some nodes at reach needs preload >= 1.93 kPa, which conflicts. Needs Forest's choice (section 4).
6. **Shear on the skin is small if the slip face really has mu <= 0.18**: peak 0.63 kPa shear, 2.4 N total drag, 0.2 J per stroke, skin temperature rise < 0.04 K. If the slip face degrades to mu 0.3 (wet, worn) the base layer is only 1.2x from slipping on the skin. Friction numbers for PTFE fabric are assumptions (no source found): bench test T-L1 decides.

## 1. Which electrodes lie under the MJ11 dish (`mj11_leads.py`)
Footprint: disc R62 about (y -60, z -190) (liner lattice), cut at y >= -94 (dish cut y = -96). Skin-travel model = `muscle_atlas.travel()`: rigid-blade rotation about (y 14.75, z -116.22) by the site angle (chord = travel_mm); **the direction sign is unknown, both signs are evaluated** and the worse one is kept. Front sites (x < -170) are excluded.

| Electrode | Position (y, z) | Distance to pad centre | Travel (full elevation) | Under liner at park | Fraction of the stroke under the liner (+ / - sign) |
|---|---|---|---|---|---|
| infraspin a | (-46.3, -169.4) | 25 | 31.8 | yes | 1.00 / 1.00 |
| infraspin b | (-27.3, -164.6) | 41 | 31.8 | yes | 0.95 / 1.00 |
| teres_maj a | (-64.6, -226.8) | 37 | 56.3 | yes | 0.67 / 0.71 |
| teres_maj b | (-46.3, -219.5) | 32 | 56.3 | yes | 0.62 / 1.00 |
| rhomboid a, b | (-129.7, -180.8), (-112.2, -190.5) | 70, 52 | 26 | no (y < -96, under the rail) | the + sign path reaches y -97: touches the cut |
| trap_mid, trap_low | y -136..-116 / -135..-125 | 78-93 | 18 / 16.7 | no (under the rail) | 0 |
| lat_dorsi | (-91, -289), (-77, -275) | 87-104 | 44.5 | no | 0 |
| serratus, delt_*, pec_*, trap_up | off the back of the blade | > 95 or front | 0-3 | no | 0 |

Teres major leaves the footprint over the lateral-lower rim in the + direction (the electrode slides out from under the foam edge: the liner edge needs a feathered ramp, section 3.4).
The 56 mm / 31.8 mm figures are the atlas's per-site values (rigid-blade bound, follow fractions assumed, DESIGN_STATE s4); the blade-wide 56-96 mm refers to the lateral border.

**Rigid material over electrodes.** The dish never touches an electrode: the rigid PA-CF face is the foam thickness (8 mm normal, 5.3 mm minimum normal gap where the x-offset liner thins on slopes) above the skin. The base layer, the electrode and 8 mm foam are between the skin and rigid. This is the standing-rule override Forest accepted on 2026-11-02 (a). It does break the flex-lead spec rule "no rigid pad over the electrode zone or within 20 mm of an electrode" (SPEC_flex_electrode_leads section 1) for infraspin and teres major; the electrode is under foam, not under a pad, and the foam pressure is what section 4 quantifies. Under the rail: the new rail bosses (z -160, -220; y -113, -105) are 29.5 mm from the rhomboid pair (OK) and 12.9 mm from trap_low b (inside 20 mm; those electrodes were already under the rail foam stack).

## 2. Leads
### 2.1 Model
Lead = flat 3-conductor shielded flex tail, about 3 x 0.1 mm (requirement; product NOT selected; PLACEHOLDER). Leads are laminated to the base-layer face and move with the skin; the trench, the exit and the dock are fixed to the dish/rail. The skin-to-trench distance changes by dL = max_t D - min_t D, with D(t) = |electrode(t) - inlet|, t over the stroke, both electrodes of the pair, both travel signs. Slack = 1.25 x dL (SPEC_flex_electrode_leads 2.3). Slack is stored as a serpentine between the electrode and the inlet: bend radius R_b = 5 mm (ASSUMPTION: 100 x thickness for a dynamic flex, IPC-type rule of thumb, not retrieved; bench T-L3), pitch 12, amplitude 32 (width 36 for two nested tails): stored 25.7 mm per half-turn.
Fit rule: the closest approach D_min must be at least the folded length (n+1) x 12 mm, else the serpentine has nowhere to fold.
### 2.2 Results per channel (inlet r chosen to minimise D_max where it fits; values rounded)
| Pair | Channel psi | r_in | D_min | D_max | dL | Slack 1.25 dL | Half-turns | Folded length | Fits |
|---|---|---|---|---|---|---|---|---|---|
| infraspin | 225 (recommended) | 18 | 42.1 | 77.6 | 19.3 | 24.2 | 1 (use 2) | 24 | yes |
| infraspin | 270 | 18 | 37.4 | 71.1 | 25.3 | 31.6 | 2 | 36 | yes |
| infraspin | 315 | 50 | 36.7 | 91.6 | 31.6 | 39.5 | 2 | 36 | yes |
| teres_maj | 225 | 32 | 0.9 | 93.1 | 56.2 | 70.2 | 3 | 48 | **no** |
| teres_maj | 270 | 40 | 0.1 | 67.8 | 56.2 | 70.2 | 3 | 48 | **no** |
| teres_maj | 315 | 60 | 0.7 | 97.2 | 55.1 | 68.9 | 3 | 48 | **no** |
(The full table over psi 120..330 is in `leads.json`; the best any single inlet does for teres is dL 37 mm at psi 330, which exits on the lateral rim, far from the rail.)
### 2.3 Layout (renders `MJ11_leads_rear/oblique/low.png`)
Infraspin a/b: electrode -> accordion (2 half-turns, 12 mm amplitude drawn; the real 32 mm amplitude fits in the 36 mm free span) -> inlet at psi 225, r 18 -> trench radially out to the exit at the y = -96 cut (r 50.9, z -226) -> dock block on the rail lateral edge (PLACEHOLDER, 8 x 14 x 26) -> along the base layer under the rail foam to the spine-bridge jack (y -146, about 54 mm further, skin travel there is blended to 0 by the atlas model: no extra slack). Total lead length per electrode: D_max 78 + trench 33 + 54 = about 165 mm, plus slack folded inside D_min.
Channel/trench: 8 wide x 8 deep (through the liner); two 3 mm tails + 2 mm clearance. Strain relief: clip at the exit (tail bonded in a 6 x 10 TPU boot, fixed to the dock), plus a stitched anchor tab on the base layer at the accordion start. The jack end uses the 16-way keyed header of SPEC_flex_electrode_leads section 3 (part not selected).
### 2.4 Checks
- Bend radius: R_b 5 mm >= assumed dynamic limit; the trench turn at the rim collector has R 56 (no constraint); the exit turn is a 90 deg in-plane bend with R >= 10 mm (dock geometry, to do). A round shielded 28 AWG silicone lead (OD about 2 mm) would need R >= 10 x OD = 20 mm, which does NOT fit the 8 mm trench turns or the 36 mm serpentine: flat flex is required.
- Trench height: 8 mm through the liner; a 0.1 mm tail leaves 7.9 mm. Pressure ridge from a tail lying under the foam: 0.1 / 8 = 1.3 % strain over a 3 mm strip, about 1.9 kPa for 0.15 MPa foam, only within the trench-free path between electrode and inlet (no electrode under it).
- Electrodes stay in the compression base layer; none is under rigid material (section 1).
- Distance to power wiring: 50 mm rule (SPEC_electronics 7.1) NOT recomputed; the S1' AK70 cable exit is PLACEHOLDER.
### 2.5 Teres major: Forest's decision
Options: (A) drop teres major from MJ11 (its atlas rule is flagged unsourced and 'crosstalk lat dorsi / infraspin'; nearest-site distance 59 mm to infraspin); (B) move the pair to the lateral border or the lower blade outside the footprint; (C) a spring-return reel (micro retractor) for the 70 mm slack, mechanism not designed; (D) stretch lead at 70 % strain, rejected (fatigue). Recommended: A. Channels psi 270 and 315 stay as spares (rhomboid / trap_low if the rail is ever removed: dL 7-21 mm there).

## 3. Liner changes made (`mj_concepts3.py`)
3.1 Rim collector trench r 56 from psi 315 to 218 (8 deg segments) joins the three radial channels in the single rail-side exit. 3.2 `trench_segments()` / `in_trench()` helpers; the frame model's liner lattice no longer carries load over trenches (it did before: a model error, 40 nodes -> 27). 3.3 Result: liner area 9,771 mm2 (was 11,340), 9.4 g at 0.12 g/cm3. 3.4 NOT done: feathered liner edge (1:4 skive over 32 mm) so that no foam edge crosses an electrode when teres/infraspin slide out; PLACEHOLDER geometry.
3.5 Foam density inconsistency: the mass uses 0.12 g/cm3 but the stiffness E_f = 0.15 MPa is that of a 24-30 kg/m3 closed-cell PE foam (Zotefoams ECOZOTE data: > 41 kPa at 25 % strain at 24 kg/m3, > 45 at 30, > 65 at 45 kg/m3). At 30 kg/m3 the liner is about 2.3 g, not 9.4 g. Not changed in code; pick the foam first (section 5.4).
Regression (mj_eval MJ11 rerun): mount 186.2 g (was 186.7), tip sag park / reach 0.84 / 1.92 mm (0.85 / 1.81), bolt worst 231.5 / 312.9 N (231 / 312), dish skin gap 5.3 mm, pad 10.6, rule violations none, stage clearance unchanged (clasp bracket 0.0 mm at the hard range; 2.0 inside the soft box), wearer ROM no contact.

## 4. Pressure on the electrodes and the liner (`mj11_liner_loads.py`, `mj11_leads.py`)
Increment (frame model, 27 liner nodes): park -0.60..+0.92, reach -1.93..+2.73 kPa (before the trench change -0.60..0.66 and -1.98..2.21: the smaller area costs 0.5 kPa peak). Absolute pressure = strap/garment preload p0 + increment; **p0 is not modelled; it is the strap tension.** Sweep 0.5..2.0 kPa. Electrode window from the sensors literature: 1.0-2.0 kPa optimum, above 2.0 uncomfortable, a 10 mm foam layer left too little pressure (Wearable EMG polyurethane foam paper, PMC11124951; its numbers are for an arm cuff, not for this site).
| Electrode | p0 = 1.0 | p0 = 1.5 | p0 = 2.0 |
|---|---|---|---|
| infraspin a (kPa, min-max over park/reach, both signs) | 0.43-1.8 | 0.93-2.3 | 1.43-2.8 |
| infraspin b | 0.65-1.9 | 1.15-2.4 | 1.65-2.9 |
Liner lift-off at the worst node needs p0 >= 1.93 kPa (reach); infraspin alone only needs p0 >= 0.57. A single p0 cannot meet both 1-2 kPa at the electrode and no lift-off everywhere. First-order estimate: increments scale with foam stiffness at a fixed pad displacement (0.15 deg tilt x 66 mm), so a foam at half the stiffness (E_f 0.07 MPa) gives infraspin about -0.3..+0.45 kPa increments and p0 1.25 -> 0.96-1.7 kPa. Lift-off at other nodes then disappears too (increment -1.0). **Decision for Forest / bench: foam stiffness and p0 (T-L4).** Under the rail: the rail foam increments from the same run are park -1.58..+0.76, reach -3.36..+2.29 kPa; the under-rail electrodes (rhomboid, trap_mid, trap_low) need the rail preload to exceed 3.4 kPa or accept lift-off at reach: `harness_loads.py` / `body_loads.py` rerun required (missing from the repo).

## 5. Liner shear (`mj11_shear.py`)
### 5.1 Setup and assumptions
Stack: skin | base layer (compression tricot, holds to the skin by skin-textile friction) | PTFE-fabric slip face (bonded to the foam) | 8 mm foam | dish. Sliding happens at the slip face. Normal load N = p0 x A + sum of increments = 15.4 N (park) to 16.3 N (reach) at p0 1.5 kPa, A = 9,771 mm2. Skin travel 56-96 mm per stroke (DESIGN_STATE s4, user-accepted). Stroke time 1-3 s (assumption; wearer-led, tip cap 0.5 m/s), 400 strokes per day (assumption), 90-day horizon. Skin thermal effusivity 1,190 W s^0.5 / m2 K (k 0.37, rho 1,100, c 3,500: typical values, assumption).
### 5.2 Friction coefficients (sources and assumptions)
| Pair | mu | Status |
|---|---|---|
| dry skin against materials, mean over sites | 0.46 +/- 0.15, range 0.24-1.26 (nylon 0.37 +/- 0.09, silicone liner 0.61 +/- 0.21; load 25 -> 100 g lowers mu by 9.5 %; speed 25 -> 62.5 rpm raises it 7 %) | Zhang and Mak, Prosthet Orthot Int 1999, page `oandplibrary.org/poi/1999_02_135.asp` (authors from memory) |
| dry skin: > 0.2 and typically about 0.5, nearly pressure-independent; moist/wet skin > 1 and rising as pressure falls; hydration is the main source of scatter | | Derler and Gerhardt, Tribology of Skin review, `doc.rero.ch/record/315296` |
| PTFE fabric against base-layer tricot, kinetic | 0.10-0.20 assumed (design 0.15); 0.05 best case (film); 0.30 degraded or wet | **ASSUMPTION, no source retrieved this session** (searches returned patents and product pages, no table) |
Static friction is higher than kinetic except PTFE on PTFE (tribonet.org summary); stick-slip is a risk as the foam preload relaxes.
### 5.3 Results (design mu 0.15)
| Quantity | p0 1.0 | p0 1.5 | Note |
|---|---|---|---|
| total normal load, reach | 11.4 N | 16.3 N | park 10.5 / 15.4 N |
| drag force F = mu N | 1.7 N | 2.4 N | mu 0.30: 3.4 / 4.9 N; mu 0.46 (no slip face, skin direct): 5.2 / 7.5 N |
| peak shear on the skin, mu p_peak | 0.56 kPa | 0.63 kPa | p_peak 3.7 / 4.2 kPa at reach |
| work per stroke (56-96 mm) | | 0.14-0.23 J | mu 0.3: 0.27-0.47 J |
| interface power / heat flux (1 s stroke, 96 mm) | | 0.23 W, 24 W/m2 | 3 s, 56 mm: 0.05 W, 4.7 W/m2 |
| skin temperature rise per stroke | | 0.004-0.011 K | worst case (mu 0.46, 96 mm, 1 s): 0.035 K; blister studies quote 3.5-5 K rises (Akers 1977, Hashmi 2013 via blister-prevention.com, secondary) |
| sliding distance | 22-38 m per day | 2.0-3.5 km in 90 days | 400 strokes per day |
| skin-attach margin mu_skin-textile (0.37, dry nylon) / mu_slip | 2.5 at 0.15 | 1.2 at 0.30 | needs >= 2 |
### 5.4 Risk assessment
- **Skin shear:** shear stress at 0.15 is 0.2-0.6 kPa, below the pressure itself and small beside a waistband; the risk is not the slip face when it works, it is the slip face failing: at mu >= 0.3 the base layer's hold on the skin is within 1.2x and the slip moves to the skin, which is the blister mechanism (Naylor 1955 via blister-prevention.com: a 30 % cut in friction coefficient about triples the cycles to trauma; doubling friction load makes damage occur 3 x as fast; blisters need long static friction at a high coefficient). Requirement: mu_slip <= 0.18 dry **and** <= 0.18 wet (sweat) over the life, or the skin attach margin falls below 2.
- **Heat:** negligible (section 5.3).
- **Abrasion / wear:** 2-3.5 km of sliding per quarter at 1.5 kPa; PTFE fabric wears and fibres pill; the base-layer fabric loses its finish; lint and sweat salts raise mu. Life limit unknown: T-L2.
- **Stiction at strap re-tension / long static holds** (arm parked for hours): mu_s about 1.3 x mu_k, then a jerk shifts the electrodes: motion artefact, not injury.
- **Electrodes:** drag moves leads and the accordion every stroke (fatigue T14-3d, T-L3).
### 5.5 Foam and fabric (recommended grades; PLACEHOLDER until bench)
Foam: closed-cell crosslinked PE or EVA, 24-30 kg/m3, 8 mm skin-contact (orthotic) grade, tangent stiffness about 0.07-0.15 MPa below 5 % strain (compression stress at 25 % strain around 40-45 kPa), skin-safety certificate (ISO 10993 irritation and sensitisation), low compression set, no plasticiser bloom. Search targets: Plastazote / Evazote class orthotic foams, Zotefoams LD24 / LD29 (not verified this session). The 0.12 g/cm3 mass assumption is 4-5 x too heavy for this class (section 3.5).
Slip face: candidates in order: (1) PTFE-fibre woven or knit fabric, etched-back laminated to the foam; (2) PTFE-coated nylon or polyester fabric (the coating is the sliding surface, adhesive bond on the uncoated side); (3) UHMWPE woven fabric (bond problem: needs plasma or hot-melt). Mate: the base-layer outer face = smooth nylon-elastane tricot (specify flat, tight knit; avoid open mesh). Edge: heat-sealed or taped, no raw PTFE fibres toward the skin.
### 5.6 Bench test plan
T-L1 friction: sled tribometer, slip fabric against the base-layer fabric, 1, 2, 4 kPa, 20-100 mm/s, 100 mm stroke, 50 cycles, dry and with artificial sweat; report static and kinetic mu, PASS mu_k <= 0.18 and mu_s <= 0.25 in both states. T-L2 wear: 5,000 strokes at 96 mm and 2 kPa (= 12 days), mu drift <= 20 %, no pilling, no fibre shed; then 36,000 strokes (= 90 days). T-L3 lead fatigue: accordion tail through 5,000 strokes at the 25-70 mm excursion, resistance change < 10 %, no open circuit (T14-3d), R_b 5 mm vs 8 mm. T-L4 pressure: pressure-mapping film or cell array under the liner on a back phantom and on Forest, arm at park and at 90 deg abduction, p0 swept by strap tension; record the 27-node field against the model (increments -1.9..+2.7 kPa) and the electrode-site pressure; PASS infraspin 1-2 kPa with no lift-off. T-L5 skin-side friction: sled with the base layer on the volar forearm and on the blade, load cell, confirm mu_skin-textile and that slip never occurs there (PASS slip-face-to-skin ratio >= 2). T-L6 thermal: thermistor on the skin under the liner, 200 strokes at 1 s, rise < 1 K. T-L7 wear trial (self, stop at any erythema, bounded by 1 h then 4 h sessions): erythema and blister check after each; abort criteria written before the first wear.

## 6. Interface contracts (`cad/interfaces.py`, new)
Recorded: (1) WING <-> SHOULDER: chassis plate, wing tongue, bridge-as-load-path, S1 base 4 x M5 and the pauldron / spaulder feet are RETIRED; 4 new M4 heat-set insert bosses (5.6 OD x 4) on the rail rear face at z -160 / -220, y -113 / -105 with spacers 4.82 / 8.26 mm (D10) and bolts M4 x 16 / x 20 (the code note said x 20 for all: tip 5 mm past the insert, corrected). (2) AK70 S1' flange pad pattern: PLACEHOLDER list of 8 items (sourced body data separated). (3) SENSING <-> SHOULDER liner trenches and exit.
New risks found while writing the contract: bolt pitch 8 mm (2.4 mm wall between inserts, rule of thumb 11.2), insert 5 mm from the rail edge (2.2 mm wall), pull axis = layer normal if printed flat. Pull-out need: 2 x 2 x 312.9 = 1,250 N per insert versus an UNVERIFIED 1 kN. Test list T-I1..T-I8 in `interfaces.BENCH_INSERT_PULLOUT` (installation, static and conditioned pull-out, geometry, torque-out, cyclic, bolt tip, creep).
The retired plate leaves the armour (pauldron + spaulder) with no mounting: the fairing / armour re-fit is the next mechanical task.

## 7. Gates and status
| Gate | Result |
|---|---|
| Rigid over electrodes | none touching; 8 mm foam above all under-liner electrodes (accepted override) |
| Lead length vs travel | infraspin OK (slack 24 mm); teres major FAILS (needs 70 mm, no inlet) |
| Bend radius / channel | flat flex OK; round lead not possible |
| Electrode pressure 1-2 kPa | FAIL at p0 1.5 (0.9-2.4); passes with softer foam + p0 1.25 (estimate) |
| Liner lift-off | FAIL for p0 < 1.93 kPa at reach (worst node), not at the infraspin site |
| Skin shear | pass if mu_slip <= 0.18 (assumed); bench T-L1 |
| Meshes watertight / mass | liner watertight, 9.4 g at 0.12 g/cm3 (inconsistent with the stiffness: about 2.3 g at 30 kg/m3) |
| mj_eval MJ11 regression | no new violations |
| Not run | harness_loads / body_loads (scripts missing), full sweep and firmware tables, distance to power wiring, fairing re-fit, thermal of the main box |

## 8. Decisions for Forest
1. Teres major: drop (recommended), move, or fund a retractor reel. 2. Foam stiffness and preload target: soft foam (about half) and p0 1.0-1.25 kPa, versus the present liner. 3. Accept a flat-flex lead (round lead is impossible), choose the product (PLACEHOLDER). 4. Rail bosses: widen the pitch / add a shared boss and test inserts before the rail is re-cut. 5. The flex-lead spec (2026-10-23) removed the rail over the electrodes, MJ11 bolts into the rail: these two decisions conflict at z -160..-220; the rail stays at least there unless the pad moves.
Sources: Zhang and Mak 1999 (oandplibrary.org/poi/1999_02_135.asp); Derler and Gerhardt (doc.rero.ch/record/315296); Wearable EMG Measurement Device Using Polyurethane Foam (pmc.ncbi.nlm.nih.gov/articles/PMC11124951); blister-prevention.com research timeline (secondary, Naylor 1955, Akers 1977, Hashmi 2013); tribonet.org "Does friction cause friction blisters?"; Zotefoams ECOZOTE PER datasheet (zotefoams.com/wp-content/uploads/2025/03/PIS-E007-ECOZOTE-PER.pdf, provisional). PMC pages other than PMC11124951 returned a captcha and were not read.
