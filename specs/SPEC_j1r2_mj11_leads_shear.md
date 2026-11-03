# SPEC: MJ11 electrode leads, liner shear, WING interface contract (SENSING / HARNESS / WING), 2026-11-03
Unattended run for Forest. Nothing here changes geometry, limits or firmware tables by default; every geometry change below is an OPTION for Forest.
Code: `cad/j1r2/mj11_leads.py`, `mj11_shear.py`, `mj11_leads_render.py`, `mj11_efoam_sweep.py`; `cad/interfaces.py` (new). Results: `stl/j1r2_mounts/MJ11/leads.json`, `shear.json`, `efoam_sweep.json`, `MJ11_leadmodel_*_R.stl`. Renders: `renders_j1r2/mj11_leads/{A_assembled,B_open}_sheet.png` (+ single views).
Run: `J1R2_ROOT=<root> python3 cad/j1r2/mj11_leads.py && python3 cad/j1r2/mj11_shear.py && python3 cad/j1r2/mj11_leads_render.py`.

## 1. Which electrodes sit under what (atlas rev c, 12 mm discs, plane projection on the back; edge-to-edge distances)
| pair | under liner/dish | under pad | under rail (rigid) | nearest bolt spacer (edge) | follow bound / spec travel |
|---|---|---|---|---|---|
| infraspinatus | **yes** (both discs) | yes | no | 48.5 mm | 31.8 / 70 mm |
| teres major | **yes** (both) | yes | no | 30.0 mm | 56.3 / 96 mm |
| rhomboid | no (27.7 mm medial of the liner edge) | 1 disc at the trim | **yes** (both) | 15.7 mm (< 20) | 26.0 / 62 |
| trap_mid | no | no (5.9 mm) | **yes** (both) | 13.4 mm (< 20) | 18.0 / 56 |
| trap_low | no | no (1.7 mm) | **yes** (both) | **1.9 mm** at (y -113, z -220) | 16.7 / 76 |
| lat_dorsi | no | no | no (3.2 mm from the rail) | 51 mm | 44.5 / n/a |
Serratus (x -214, flank) and the clasp/anterior sites are not on the back plane and are unaffected. Liner/dish footprint = disc R66 at (-60, -190) cut at y -96 (10 315 mm2 net of channels).

**Finding 1 (conflicts with a standing decision).** Three pairs (trap_mid, rhomboid, trap_low; 6 discs) lie under the 38 x 7 mm PA-CF rail, which MJ11 keeps and now bolts to. `SPEC_flex_electrode_leads` (Forest, 2026-10-23: "remove the 3D printed skeleton over the electrodes") is DECIDED but was never built, and MJ11 makes the rail structural. Both cannot hold. Also trap_low's lateral disc is 1.9 mm from the z -220 spacer (rule: nothing rigid within 20 mm of an electrode); rhomboid and trap_mid are 15.7 / 13.4 mm. Options, Forest to choose (not decided here):
- **A. Windowed rail:** cut windows (about 32 x 24 mm trap_mid, 30 x 22 rhomboid, 22 x 30 trap_low) in the rail between the bolt bosses; rungs at z -160 and below -250 only. Bosses move off z -220 (trap_low) to about z -255 or the lateral half of the rail. Costs rail stiffness: re-run `mj_eval` (rail 1.3 to 2.3 MPa now, much margin).
- **B. Move the three pairs** off the rail: lateral strip y -100..-96 is only 4 mm wide, so this means the blade (liner zone) or the lower back; changes atlas sites (SENSING decision).
- **C. Keep the rail and the old stack** (electrodes under 7.5 mm rail foam, SPEC_rail_facing): violates the 2026-10-23 decision, which then has to be withdrawn explicitly.
Recommendation: A (keeps the decision and MJ11). Cheapest to test: bosses move first.

## 2. Leads for the two liner-covered pairs
Assumptions (inputs, not measurements): lead = 3-conductor shielded 28 AWG silicone, OD 2.2 mm; bend radius >= 10 x OD = **22 mm** (SPEC_flex_electrode_leads 2.2); slack = 1.25 x design travel (its section 2.3 table: infraspinatus 70, "lateral border" 96 for teres major); jack = ASSUMED 24 x 12 x 8 mm 16-way keyed header (PLACEHOLDER) on the rail rear face at (y -108, z -272), 52 mm below the lowest boss, with a lead wrap round the rail's lateral edge (about 15 mm in x).

- Routes (shortest, `leads.json`): infraspinatus via the 225 deg channel exit (-96, -226): 83.6 mm couched in the base layer + 47.5 mm free span; teres major via the 270 deg exit (-60, -256): 33.1 + 50.6 mm. The 315 deg channel leads away from the jack and serves no electrode: spare or delete.
- Service loop: slack 88 / 120 mm needs loop radius 13.9 / 19.1 mm (circumference = slack) but the bend rule sets R >= 22 mm: **one full turn of R22 (138 mm of lead) per lead, 52 mm OD loop bay**, shared bay for two leads about 60 x 110 mm. The spec's "4 half-turn serpentine +/-18 mm" is **not allowed** (R18 < 22; each half-turn at R22 gives only 25 mm slack, 4 need 176 mm of axis).
- Strain relief: lead clamped on a 10 mm tangent at the jack (cable tie to a 2-boss plate), second anchor on the base layer at the liner exit; loop between them. Never anchor across the 4 mm gap between dish edge and rail.
- Channel size as drawn: 8 wide x 8 high (through the liner). Lead 2.2 mm leaves (8 - 2.2)/2 = **2.9 mm** lateral allowance each side; height fine.

**Finding 2 (the 8 mm radial channels cannot carry a lead that rides on the base layer).** The base layer slides under the fixed liner 32..56 mm (rigid-blade bound) to 70..96 mm (spec). The component of that motion across a channel axis (`leads.json: channels`) is 22..47 mm for the 270 deg channel and 32..55 mm for the 225 deg channel (the two channels the leads would use), versus 2.9 mm allowed; only the 315 deg channel is aligned with the infraspinatus follow (across 0.2 mm) and it still misses teres major by 11.5 mm. The across-axis components are independent of the (uncertain) sign of the blade rotation. A lead laminated to the base layer therefore sweeps out of a fixed slot almost at once. Meanwhile a lead lying in the liner interface without a slot is a ridge: foam Winkler model (E 0.15 MPa, 8 mm, project model) gives extra local pressure E t / 8 = **6.6 kPa (0.35 mm flat), 18.8 (1.0), 41.2 kPa (2.2 mm round)** on top of 2..4 kPa, over the 4.3 kPa budget. So a round lead cannot go under the liner at all and even a flat one is marginal; this needs bench T-L-RIDGE.
Options (Forest decides; the default CAD is unchanged, env `MJ11_LEAD_PSI`, `MJ11_LEAD_W` in `mj_concepts3.py` allow trying them):
- **L1 (recommended to test first):** flat printed / embroidered trace (<= 0.35 mm) inside the base-layer panel, no channel under the liner. Couched length per lead as above. Channels shrink to a single 12 mm exit relief at the 225 or 270 rim, and the anchor + loop sit just outside the rim. Pass requires the ridge bench test (<= about 2 kPa extra).
- **L2:** motion-aligned channels: one channel along each site's follow direction (infraspinatus 315 deg is already aligned; teres major needs about 326 deg), width 8 -> 12 mm, length >= travel + 12. Lead slides along the slot with its slack buckled in the slot (flat ribbon accordion, bench). Direction depends on the assumed rotation sign (confirm with T14-3b skin-follow measurement).
- **L3:** coiled (helical) lead, relaxed 35 mm, extending to ~130 mm, OD 6 mm in an 8 mm channel: fits width and height, but runs the coil strain rule instead of the bend rule (conductor strain about 0.6/2.4); bench T14-3d (5 000 strokes) decides.
- Not an option: any rigid element over the electrode, any round lead pressed between liner and base layer.

## 3. Channel changes made
None by default. `mj_concepts3.py` now reads `MJ11_LEAD_PSI` (comma list, default `225,270,315`) and `MJ11_LEAD_W` (default 8) so L2 variants can be generated without editing code.

## 4. Liner shear (`shear.json`)
Normal loads: `mj_eval run_frame` (E_FOAM 0.15 MPa): liner pressure increment park -0.60..+0.66 kPa, reach -1.98..+2.21 kPa. Sweep over E_FOAM 0.05..0.30 MPa (`efoam_sweep.json`): reach max 1.82..2.42, min -1.44..-2.48 kPa: softening the foam does not close the problem.
- **Preload window:** to avoid lift-off the strap/garment preload must be >= 1.98 kPa; to keep peak <= the 4.3 kPa band budget it must be <= 2.09 kPa. Window 0.11 kPa wide: not controllable by strap tension alone. Either accept brief lift-off at the lateral-reach extreme (the arm leaves the liner at its upper edge, not a hazard; the arm load is on the bolts, 312 N) or raise the budget with bench perfusion data. Recommend lift-off be accepted and preload set at 2.0 kPa (above, 3.0 kPa gives 5.2 kPa peak, over budget).
- Interface: the PTFE-fabric slip face (liner side) slides on the base-layer tricot; the tricot does not slip on the skin as long as mu_tricot-skin > mu_slip (about 0.5 vs 0.1..0.4). Skin shear therefore = mu_slip x p. Stick-mode check: if the foam gripped the base layer, the 76 mm travel over 8 mm would be a shear strain of 9.5 (0.48 MPa at G 0.05 MPa, assumed): impossible, so a slip face is mandatory and a failed slip face means the foam drags the tissue.
- Friction coefficients (ASSUMED, no direct PTFE-fabric / tricot / skin measurement was found): slip face dry 0.10..0.25, wet (sweat) 0.20..0.40, foam on tricot with no slip face 0.6..1.0. Support: Veijgen et al., Tribology Letters 49 (2013): skin friction against stainless steel, aluminium, PE and PTFE, overall median 0.52 static / 0.36 dynamic, **lowest for PTFE** (the page gave no per-material numbers); Naylor 1955 and Sulzberger 1966 (as summarised by blister-prevention.com): friction is lowest dry/greasy/very wet and **highest when moist**; Carlson (as cited): damage rate scales with the friction load (about 30 % lower mu gives about 3x the cycles; doubling friction triples the damage rate).
- Result at the peak pressure (reach, p0 2.0 kPa, 4.2 kPa total, area 10.3 cm2; uniform p is conservative): shear 0.42..1.05 kPa dry slip face, 0.84..1.68 kPa wet, 2.5..4.2 kPa with no slip face; whole-liner drag 4..11 N dry, 9..17 N wet, 26..43 N without slip fabric (carried by the pad/rail, small versus the 312 N bolt load).
- Heat: strokes of 56..96 mm in 1 s (assumed): 40..160 W/m2 with the slip face, flash rise **0.04..0.16 K per stroke** (skin effusivity 1180 W s^0.5/m2K), versus blister thresholds of +3 K (Kirkham 2014) and a median +5.1 K at blister (Hashmi 2013, both via blister-prevention.com). Heat is not the risk; accumulated rubbing in sweat (cycles) is. Sliding distance at an assumed 2000 strokes per hour: 224..384 m per hour.
- Risk verdict: skin shear is acceptable only with a working slip face (<= 1.7 kPa wet); it fails (2.5..4.2 kPa, 9.5 shear strain demand) if the slip fabric wets out, bunches or is lost. The channel/slot edges are catch lines for the base layer: chamfer and cover with the slip fabric.
- Recommended grades (PLACEHOLDERS, no purchase links retrieved): liner 6..8 mm closed-cell PE/EVA, skin-safe, E about 0.1..0.15 MPa, 0.10..0.14 g/cm3, perforated or skived for moisture; slip face: PTFE-coated or PTFE-laminated woven fabric, 0.1..0.15 mm, bonded across the whole liner face (adhesive edge sealed); base layer: smooth nylon tricot (not elastane-rich fuzzy knit); optional dry PTFE film instead of fabric if the weave catches.

## 5. Bench plan
T-L-SHEAR sled friction (dry and sweat, p 2 and 4 kPa, 0.1 m/s, 100 strokes, slip face vs. none); T-L-RIDGE lead ridge pressure map; T14-3b skin-follow measurement (direction and magnitude per site, needed for L2); T14-3d lead fatigue (5 000 strokes, < 10 % R change); 20 min worn trial with blister check; wet-out and wash cycles of the slip fabric.

## 6. M4 insert pull-out in PA-CF (bench list, `interfaces.BENCH_TESTS`)
T-W-INS: M4 heat-set 5.6 x 4 insert, printed PA-CF coupons (wall 7 mm and thinner 4 mm bosses), >= 10 each, install at the vendor temperature, measure pull-out and torque-out (target: mean >= 1.25 kN = 312 N reach bolt x SF 2 x 2 dynamic; min >= 1.0 kN), 10^5 cycles at 150 N, hot (50 C) and wet. Also: insert depth 4 mm in a 7 mm wall leaves 3 mm of floor: test a 5 mm boss lip.

## 7. WING contract change (details in `cad/interfaces.py` and DESIGN_STATE section 13)
Retired: bridge over the blade, tongue, 8 mm chassis plate, J1 rev 2 riser. Added: 4 x M4 heat-set (5.6 x 4) bosses on the rail rear face at z -160 / -220, y -113 / -105 (PLACEHOLDER), spacers D10 x 4.8 / 8.3 mm. S1' AK70 flange pattern: PLACEHOLDER list (6 x M4 on 60 PCD position-only; others unverified). Renders: `renders_j1r2/mj11_leads/`.

## 8. Open / decisions for Forest
1. Rail over trap_mid / rhomboid / trap_low vs the 2026-10-23 flex-lead decision (Finding 1: options A/B/C).
2. Lead architecture under the liner (Finding 2: L1/L2/L3); channels left unchanged meanwhile.
3. Preload window (0.11 kPa) and whether lift-off at the reach extreme is acceptable.
4. Jack location and header part (ASSUMED), loop bay on the lower back.
