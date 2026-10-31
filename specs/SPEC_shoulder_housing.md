# SPEC: Shoulder housing — socket cowl + humeral-head dome + armoured pauldron

Current revision: **2026-10-30b (flush armour)**, §0. It keeps the rev 2026-10-30 three-part architecture (crown + keel + spaulder, §1) and rebuilds every part to sit flush. The socket cowl and humeral-head dome are unchanged from rev 2026-10-13 (§2).

## 0. Rev 2026-10-30b: flush armour

**Forest (2026-10-30):** "Wayyyyyyy too bulky, this thing should sit flush against the shoulder as much as possible."

| | Rev a (§1) | **Rev b** |
|---|---|---|
| Armour mass (PA-CF 1.15) | 171.4 g | **110.6 g** (crown 28.7, keel 41.9, spaulder 40.0; walls 1.8) |
| Armour volume | 149.0 cm³ | 96.2 cm³ |
| Assembly envelope (x × y × z) | 287 × 186 × 120 mm | **277 × 149 × 77 mm** |
| Bottom of the armour | z −163 | **z −115** (the keel cups the AK70 S1, which ends at z −106.8, instead of hanging 56 mm below it) |
| Spaulder distance to skin, median | 52.3 mm | **36.9 mm** |
| Chassis plate (6061, 8 mm) | 154 × 114, 337 g | **158 × 104, 278 g** |

Rev b per arm: −61 g armour, −59 g plate (−120 g).

**How the parts became flush.**
- **Crown.** It now follows the stage contents at G_IN 3.5 instead of a decorative superellipse.
  - The outline is the convex hull of the stage contents (z −112..−50) plus the r 66.5 bore.
  - Bulge 0, flare 1.2 (was 2 / 3), cop 3.0, bead 0.9, wall 1.8 (was 1.6: the flush shells fell below the 1.4 mm p5 gate at 1.6).
  - No ears: the spaulder no longer bolts to the crown.
  - 147 × 143 × 46, 28.7 g.
- **Keel.** It is a shallow dish under the S1 motor (keel depth 14, was a two-lame 72 mm cascade).
  - Flange ≤ 6 mm wide, with two webs down to two plate feet.
  - The lip follows the crown flare with a 0.7 mm nesting gap. Rev b found the old fixed lip offset 0.9 mm interfering (8.6 mm³) once the flare dropped to 1.2. The new interference gate (exact manifold boolean) holds it at 0.
  - 148 × 137 × 24, 41.9 g.
- **Spaulder.** It is a draped, conformal plate.
  - The design surface is the obstacle floor: skin + 10.5 mm, clasp/wing/plate + 3.5, crown + 1. It is max-filtered over 10 mm, grey-closed with a 36 mm disc, then Gaussian-smoothed at 7 mm.
  - It sits T/2 + 0.6 mm above that floor, with 1.5 mm lame steps and a 2 mm edge curl.
  - The closing removes the crumpled "shrink-wrap" look the first rev-b pass had (bumps traced over every clasp node).
  - Two rear feet bolt down onto the plate top edge. The four ear posts are gone.
  - 127 × 141 × 69, 40.0 g.

**The floor that sets how flush the spaulder can get.** The spaulder over the shoulder top is held 10 mm off skin by the rule in §1.2 item 4 (gate `static/spaulder-skin away from the seats`, need 10). It is also held 3.5 mm off the clasp frame, which itself rides on pads. Going lower means relaxing that 10 mm rule, which is Forest's call. Every 1 mm of rule is about 1 mm of height over the trapezius. The skin travel figure over the blade (56–96 mm) does not apply here: the spaulder sits over the upper trapezius and acromion, which are stable zones. The 10 mm covers soft-tissue bulge under load and the clasp pad stack.

**Interface changes (WING: chassis plate, `cad/chassis_axilla.py` rev b; contract in `stl/shoulder/plate.json`):**
- Pauldron feet are 2 × M4 at (y, z) = (±50, −109) (was 6 at three positions): Ø4.5 clearance + 90° countersink Ø9, ISO 10642 M4 × 12 from the plate front into M4 heat-set inserts (5.6 × 4) in the keel's Ø13 × 6 bosses.
- New: 2 × M3 tapped holes (Ø2.5 drill, 10 deep) vertically into the plate top edge at y ±40, x −84, for the spaulder's rear feet (ISO 7380 M3 × 10, Ø6.6 counterbore in the foot, 4 mm clamp).
- Plate outline trimmed to the tongue that carries the S1 base (corner seeds at y 72/104 inboard), 158 × 104. The S1 base bolt pattern (4 × M5) is unchanged.
- Unchanged: clasp seats F1/F4 (M4 × 12, Ø24 round pads), socket rim z −54 / bore r 66, the wing/plate bolt pattern.

**Fasteners (rev b; replaces the §1.3 table):**
| Joint | Fastener | Insert / hole |
|---|---|---|
| Keel → plate (2) | ISO 10642 M4 × 12 from the plate front | M4 heat-set 5.6 × 4 in Ø13 × 6 bosses |
| Crown hem → keel lip (4) | ISO 7380 M3 × 8 | M3 heat-set 4.6 × 5.7 in radial bosses at az ±35°, ±100°, z −96; lip gap 0.7 |
| Spaulder rear feet → plate top edge (2) | ISO 7380 M3 × 10 | Tapped M3 in the 6061 plate (Ø2.5 × 10); foot Ø3.4 + counterbore Ø6.6 |
| Spaulder → clasp seats F1/F4 (2) | M4 × 12 (unchanged) | Insert in the clasp boss; Ø8.5 counterbore |

**Assembly (rev b):**
1. Slide the crown along −x onto the bare S1 base.
2. AK70 S1, clamp and stop posts as before.
3. Keel up from below into the crown hem; fit 4 × M3 × 8.
4. Bolt the base (4 × M5) and the keel (2 × M4 × 12 csk) to the plate from the front.
5. Yoke, S2, cowl, dome and arm as before.
6. Lower the spaulder (−z) onto the plate top edge and the two clasp seats; fit 2 × M3 × 10 and 2 × M4 × 12 from above.

**Harness.** The harness model carries a 182 g pauldron, and p_top scales with total mass (N_top = W, §1.2 item 5). On the same basis as rev a, −120 g per arm on 5.875 kg gives ≈ −0.30 kPa on the top pad: ~14.31 kPa estimated with HAND rev B, against the 14.7 limit. `harness_loads.py` is still missing from the repo (DESIGN_STATE open item 4), so this is an estimate.

**Scripts:**
- `cad/pauldron.py` rev 2026-10-30b.
- `cad/chassis_axilla.py` rev b.
- `cad/verify_armor.py`: rev-b masks, plus exact interference gates between every armour part pair and between each part and the plate.
- `cad/scene_armor_only.py`, new: armour-only close-up renders.

**Renders:** `armor_flush_close_sheet.png`, `armor_flush_parts_sheet.png`, `armor_flush_before_after.png` (rev a vs rev b).

### 0.1 Gates (rev b)
| Gate | Result | Need | Pass | Note |
|---|---|---|---|---|
| static/crown-stage contents | 3.08 | 3.0 | yes |  |
| static/keel-stage contents | 3.15 | 3.0 | yes |  |
| static/spaulder-stage contents | 12.09 | 3.0 | yes |  |
| static/crown-plate | 3.51 | 3.0 | yes |  |
| static/spaulder-plate outside the 2 rear feet | 4.12 | 3.0 | yes | feet M3x10 into the tapped plate top edge (designed contact) |
| static/keel-plate outside the bolted flange | 5.51 | 3.0 | yes | flange face on the plate rear face is the bolted seat |
| static/crown-clasp | 15.18 | 3.0 | yes |  |
| static/keel-clasp | 8.2 | 3.0 | yes |  |
| static/spaulder-clasp outside the 2 seats | 4.43 | 3.0 | yes |  |
| static/crown-wing | 21.3 | 3.0 | yes |  |
| static/keel-wing | 8.65 | 3.0 | yes |  |
| static/spaulder-wing | 5.79 | 3.0 | yes |  |
| static/crown-spine bridge | 60.0 | 3.0 | yes |  |
| static/keel-spine bridge | 60.0 | 3.0 | yes |  |
| static/spaulder-spine bridge | 60.0 | 3.0 | yes |  |
| static/crown-skin | 60.0 | 2.0 | yes |  |
| static/keel-skin | 45.33 | 2.0 | yes |  |
| static/spaulder-skin away from the seats | 15.47 | 10.0 | yes | rule 10 mm over the shoulder top |
| static/spaulder-skin within 35 mm of a seat | 17.35 | 2.5 | yes | seats sit on stable zones (F1 / F4) |
| interference/crown-keel volume (mm3) | 0 | 0 | yes | must be 0 |
| interference/crown-spaulder volume (mm3) | 0 | 0 | yes | must be 0 |
| interference/keel-spaulder volume (mm3) | 0 | 0 | yes | must be 0 |
| interference/crown-plate volume (mm3) | 0 | 0 | yes | must be 0 |
| interference/keel-plate volume (mm3) | 0 | 0 | yes | must be 0 |
| interference/spaulder-plate volume (mm3) | 0 | 0 | yes | must be 0 |
| joint/keel lip inside crown hem (gap) | 0.46 | 0.3 | yes | designed nesting gap 0.7 mm (rev b: follows the crown flare); screws M3x8 at the hem |
| static/keel-crown away from the lip | 1.2 | 1.0 | yes |  |
| static/spaulder-crown (rev b: no ear posts, scalloped over the crown) | 7.68 | 1.0 | yes |  |
| S1 sweep/crown (hard 5..150) | 4.35 | 3.0 | yes | worst at S1 87 |
| S1 sweep/keel (hard 5..150) | 37.05 | 3.0 | yes | worst at S1 52 |
| S1 sweep/spaulder (hard 5..150) | 4.18 | 3.0 | yes | worst at S1 13 |
| arm sweep/crown (soft box) | 18.03 | 3.0 | yes | worst at S1/S2/S3 (45.0, 87.0, -70.0) |
| arm sweep/keel (soft box) | 40.0 | 3.0 | yes | worst at S1/S2/S3 (10.0, -30.0, -70.0) |
| arm sweep/spaulder (soft box) | 4.18 | 3.0 | yes | worst at S1/S2/S3 (45.0, 87.0, -70.0) |
| ROM/arm abduction | no contact | none | yes | no contact in the tested range; old pauldron None; clasp seat boss 100.0; clasp 5.0; normal 110 GH (171 total) deg |
| ROM/arm scaption (30 deg) | 107.5 | none | yes | first contact; old pauldron None; clasp seat boss 95.0; clasp 5.0; normal 110 GH deg |
| ROM/arm flexion | no contact | none | yes | no contact in the tested range; old pauldron None; clasp seat boss 102.5; clasp 10.0; normal 110 GH (165 total) deg |
| ROM/neck lateral flexion to the right | 50.0 | 50.0 | yes | first contact; old pauldron 50.0; clasp seat boss 55.0; clasp 42.5; normal 42 deg |
| ROM/neck rotation to the right | no contact | none | yes | no contact in the tested range; old pauldron None; clasp seat boss None; clasp None; normal 71 deg |
| ROM/neck flexion | no contact | none | yes | no contact in the tested range; old pauldron None; clasp seat boss None; clasp 55.0; normal 58 deg |
| ROM/neck extension | no contact | 67.5 | yes | no contact in the tested range; old pauldron 67.5; clasp seat boss None; clasp None; normal 59 deg |
| print/crown watertight + 256 cube | 1.0 | 1.0 | yes |  |
| print/crown wall p5 | 1.53 | 1.4 | yes | nominal 1.8 rev b (keel flange 3-6) |
| print/keel watertight + 256 cube | 1.0 | 1.0 | yes |  |
| print/keel wall p5 | 1.54 | 1.4 | yes | nominal 1.8 rev b (keel flange 3-6) |
| print/spaulder watertight + 256 cube | 1.0 | 1.0 | yes |  |
| print/spaulder wall p5 | 1.4 | 1.4 | yes | nominal 1.8 rev b (keel flange 3-6) |
| mass/armour total <= 182 g (harness model's pauldron) | 71.4 | 0 | yes | 110.6 g |
| assembly/crown slides on along -x over the bare S1 base | 3.15 | 0.5 | yes |  |
| assembly/keel rises +z (stage, motor, crown) | 0.57 | 0.3 | yes | flange slides parallel to the plate face |
| assembly/spaulder drops -z onto the plate top and the 2 clasp seats | 4.42 | 0.3 | yes |  |

**Print per part:** crown: best orientation -z up, support 18.5 cm², wall p1/p5/median 1.24/1.53/1.82 mm; keel: best orientation +x up, support 12.0 cm², wall p1/p5/median 1.53/1.54/1.61 mm; spaulder: best orientation +y up, support 14.5 cm², wall p1/p5/median 0.95/1.4/1.81 mm.

Spaulder rear-foot seat gap on the plate top edge: 4.08 mm. Arm sweep against the 2026-10-23 pauldron for comparison: 19.36 mm. Overall: **ALL PASS**, mass 110.6 g.

The spaulder wall p5 sits exactly at the 1.4 mm gate. The thin 5 % are the rolled outer bead and the ROM-carved edges, not the lames (median 1.8+). Print them with 4 perimeters; the bead is a cosmetic roll.

## 1. Rev 2026-10-30: armoured pauldron (crown + keel + spaulder) — rev a, SUPERSEDED in geometry by §0

**Forest (2026-10-30):** "Let's focus on refining the shoulder mount. It shouldn't be a shelf, it should be more like a shoulder piece of a knight's armor." Then: "Rethink with opus, really make smart ergonomic choices that are also aesthetically cool."

**Why the old part read as a shelf.** The S1 base is a 136 mm block cantilevered off the flat 6061 plate. The 2026-10-13/23 pauldron was a housing round that block with struts under it: a bracketed shelf in silhouette and a flat puck from the side, with an open floor showing from below (`armor_before_after.png`).

**What changed.**
- Only the pauldron (group F, fixed) and the six pauldron-foot holes in the plate.
- Joints, actuators, limits, stops, cowl, dome, S1 stage, clasp and wing are untouched.
- Every interface the old part used is kept:
  - plate feet: 3 × 2 M4 at the same positions;
  - clasp seats F1/F4: same centres and normals, M4 × 12;
  - socket rim at z −54, bore r 66;
  - assembly order.

**Scripts.**
- `cad/pauldron.py`: rev 2026-10-30. Signed-distance-field CAD; every surface is placed from distance fields computed from the meshes, so a re-fit to Forest's mesh is a re-run.
- `cad/verify_armor.py`: all gates, including the wearer range of motion. `ROM_ONLY=1` runs only the range-of-motion study.
- `cad/scene_shoulder.py`: three.js renders (`CLOSE=1` for the shoulder close-up).
- `cad/chassis_axilla.py`: plate holes changed.

**Outputs.**
- `stl/shoulder/pauldron/pauldron_{crown,keel,spaulder}_{R,L}.stl`
- `pauldron_R.stl` (combined, for sweeps)
- `pauldron.json`, `verify_armor.json`
- Render-only fastener heads: `stl/shoulder/proxies/pauldron_rivets_R.stl`.
- The 2026-10-13/23 parts are archived in `stl/archive/pauldron_rev2026-10-23/`.
- Renders at the repo root: `armor_close_*.png`, `armor_full_sheet.png`, `armor_before_after.png`.

### 1.1 Parts (PA-CF annealed, 1.15 g/cm³)
| Part | Description | Wall | Volume / mass | Envelope (mm) |
|---|---|---|---|---|
| **Crown** (the cop) | Domed brim round the turning socket. A rolled bezel (R1.5) frames the bore. The brim falls 7 mm on a quadratic from r 66 to its edge. Below it are two lames (z −57..−79, −77..−100), each flaring 3 mm and bulging 2 mm at mid-height, with rolled hems (R1.2) and 1.6 mm risers. The plan is a superellipse (rear 76, front 120, half-width 83, n 3), U-shaped and open toward the plate. Two ears beside the plate side edges carry the spaulder. | 1.6, normal thickness held on the sloped bulge | 53.5 cm³ / 61.5 g | 176 × 186 × 46 |
| **Keel** | Two stepped lames continuing the cascade from z −100 down into the plate. Each slice is the crown outline scaled about the plate centre line: the rear tapers fast (exponent 1.2) while the width holds (exponent 2.6), so the plate feet stay inside. The upper lame flares 3 mm to the step. A centre crest and ±45° flutes run along the meridians. Plate flange 9 wide × 3 thick, with six Ø13 × 6 insert bosses on the old foot holes. The lip nests 0.6–1.4 mm inside the crown hem. | 1.65 | 64.6 cm³ / 74.3 g | 157 × 171 × 72 |
| **Spaulder** | Three sculpted lames from the crown over the shoulder top onto clasp seats F1 and F4. It domes fore-aft (30 mm fall to the front), with cross camber growing from the plate seam, a soft medial crest, and 3 mm steps (the rear lame overlaps the next) with rolled hems. The outer edges are rolled outward and curled down. The rear edge is scalloped concentric with the socket at r 88.5. The front is cut back along Forest's own arm and neck envelope (§1.2.1), so it forks into two lobes that land on the seats. Seat pads are Ø24 × 5, round, on the Ø24 clasp bosses. | 1.6 | 31.0 cm³ / 35.6 g | 127 × 170 × 45 |
| **Total** | | | **149.1 cm³ / 171.4 g** (budget: the 182 g the harness model already carries) | |

### 1.2 Ergonomic decisions (built into the generator, checked in §1.4)
1. **The armour follows Forest's body, not the other way round.** The generator sweeps his own arm and head/neck from the MakeHuman mesh and cuts the spaulder outline along that envelope with 5 mm clearance. The edges stay rolled because the cut is applied to the plan outline before the shell is built; a 3-D carve after it is only a safety net.
   - **Arm:** abduction, scaption (30°) and flexion, rigid about the glenohumeral centre, to **110° GH**. The spaulder sits on the acromion and upper trapezius, which ride with the scapula, so relative to them the humerus turns only the GH share of elevation. With GH:ST = 1.7:1 (McClure et al. 2001, bone pins; DESIGN_STATE sources), full elevation (171° abduction, 165° flexion; normative values, Acta Med Iran 2005) is ~110° at the GH joint. A rigid GH rotation past that drives the deltoid skin through the acromion: the first run with 180° severed the F4 seat, which is how this was found.
   - **Neck:** right lateral flexion to **50°**, right rotation 90°, flexion 70°, extension 70°. The rotation is blended over the cervical span (0 at the neck joint, 1 at +50 mm). 50° is the old pauldron's first-contact angle and above normal (42 ± 8°, Apti et al. 2023, n = 300).
2. **Nothing over Forest's own joint.** The lateral edge stops at the acromion line (the F4 seat) and the deltoid is not covered.
3. **Nothing standing beside the neck.** The medial edge curls down and away. There is deliberately no haute-piece (the raised neck guard on real pauldrons): it would stop lateral neck flexion.
4. **Body contact only on stable zones.** The spaulder touches Forest only through the two existing clasp seats (F1 upper trapezius, F4 acromion). Skin gap is ≥ 16.7 mm everywhere else (rule 10 mm).
5. **Mass below budget.** The harness top pad is the binding gate: 14.5 vs 14.7 kPa at rev f3 (the project DESIGN_STATE), ~14.61 kPa estimated with HAND rev B. The armour is 171.4 g against the 182 g pauldron in the harness model, so pressures do not rise. In that model N_top = W for every pose, so p_top scales with the total mass: −10.6 g on 5.875 kg gives ≈ −0.03 kPa.
6. **Sculpted, not articulated.** Nothing under the lames moves relative to anything else; articulation would only add rattle and pinch points.
7. **Serviceable.** The keel comes off with 4 × M3 at the hem plus the 6 plate screws, exposing the S1 motor and its cable without touching the arm. The spaulder lifts off with 4 × M3 + 2 × M4 from above.
8. **Structural role (unchanged in kind).** The stage is carried by the plate (4 × M5 through the S1 base). The armour is a cover and a parallel tie (plate feet → keel → crown → ears → spaulder → clasp seats), as the old pauldron was.
   - **Check:** 100 N pressed down on the crown's rear rim (x +80) gives ~16 N·m about the plate. The upper keel insert pair takes ~300 N, inside M4 heat-set pull-out in PA-CF (≥ 1 kN typical; vendor figure to confirm).
   - The keel shell section near the flange (thin U, r ≈ 60, t 1.65) has S ≈ 9 × 10³ mm³, so ≈ 1.8 MPa.

### 1.3 Fasteners, fits, inserts, print
| Joint | Fastener | Insert / hole | Notes |
|---|---|---|---|
| Keel → plate (6) | ISO 10642 M4 × 12 countersunk, from the plate **front** | M4 heat-set 5.6 × 4 in the keel bosses, pressed from the plate face (hole Ø5.6 × 4.5). Plate holes now Ø4.5 + 90° csk Ø9.0 (`chassis_axilla.py`; they were tapped 3.3, which cannot clamp a screw driven from the plate front) | Head flush with the plate front. Fitted when the stage + armour go onto the plate, before the plate goes onto the harness. |
| Crown hem → keel lip (4) | ISO 7380 M3 × 8 button head | M3 heat-set 4.6 × 5.7 in radial Ø8.5 bosses inside the lip at az ±35°, ±100°, z −96; crown clearance Ø3.4 | Slip fit, gap 0.6–1.4 mm |
| Spaulder → crown ears (4) | ISO 7380 M3 × 10 | M3 heat-set 4.6 × 5.7 from the ear top at x −81.5 / −90.5, y ±87.25; Ø6.8 counterbore in the post, 4 mm clamp | Ears 3.5 mm off the plate side edges |
| Spaulder → clasp seats F1/F4 (2) | M4 × 12 (unchanged) | Insert in the clasp boss (unchanged); Ø8.5 counterbore through lame and boss | Contract unchanged; pad is Ø24 round (was 24 × 24) on the same Ø24 boss |

**Print.** PA-CF annealed, all wall (4 perimeters at 0.4 mm), 100 %. Print the M3 / M4 insert holes at nominal and ream to the insert vendor's figure. Best orientation and support area per part come from the overhang analysis in `verify_armor.json` (§1.4).

**PLACEHOLDER.** The S1 bundle port in the keel (22 × 12 slot on the medial side) waits for the measured AK70-10 cable exit. Do not rely on its position.

### 1.4 Gates (`verify_armor.py` → `stl/shoulder/pauldron/verify_armor.json`)
| Gate | Result | Need | Pass | Note |
|---|---|---|---|---|
| static/crown-stage contents | 3.12 | 3.0 | yes |  |
| static/keel-stage contents | 3.2 | 3.0 | yes |  |
| static/spaulder-stage contents | 13.38 | 3.0 | yes |  |
| static/crown-plate | 3.5 | 3.0 | yes |  |
| static/spaulder-plate | 3.83 | 3.0 | yes |  |
| static/keel-plate outside the bolted flange | 5.51 | 3.0 | yes | flange face on the plate rear face is the bolted seat |
| static/crown-clasp | 14.92 | 3.0 | yes |  |
| static/keel-clasp | 8.2 | 3.0 | yes |  |
| static/spaulder-clasp outside the 2 seats | 6.89 | 3.0 | yes |  |
| static/crown-wing | 12.22 | 3.0 | yes |  |
| static/keel-wing | 8.73 | 3.0 | yes |  |
| static/spaulder-wing | 10.62 | 3.0 | yes |  |
| static/crown-spine bridge | 54.31 | 3.0 | yes |  |
| static/keel-spine bridge | 51.95 | 3.0 | yes |  |
| static/spaulder-spine bridge | 60.0 | 3.0 | yes |  |
| static/crown-skin | 50.8 | 2.0 | yes |  |
| static/keel-skin | 34.96 | 2.0 | yes |  |
| static/spaulder-skin away from the seats | 16.66 | 10.0 | yes | rule 10 mm over the shoulder top |
| static/spaulder-skin within 35 mm of a seat | 18.55 | 2.5 | yes | seats sit on stable zones (F1 / F4) |
| joint/keel lip inside crown hem (gap) | 0.79 | 0.3 | yes | designed nesting gap 0.6-1.4 mm; screws M3x8 at the hem |
| static/keel-crown away from the lip | 1.82 | 1.0 | yes |  |
| static/spaulder-crown away from the ear posts | 2.03 | 1.0 | yes |  |
| S1 sweep/crown (hard 5..150) | 4.33 | 3.0 | yes | worst at S1 60 |
| S1 sweep/keel (hard 5..150) | 37.01 | 3.0 | yes | worst at S1 5 |
| S1 sweep/spaulder (hard 5..150) | 5.58 | 3.0 | yes | worst at S1 31 |
| arm sweep/crown (soft box) | 18.03 | 3.0 | yes | worst at S1/S2/S3 (np.float64(20.0), np.float64(87.0), -70.0) |
| arm sweep/keel (soft box) | 40.0 | 3.0 | yes | worst at S1/S2/S3 (np.float64(10.0), np.float64(-30.0), -70.0) |
| arm sweep/spaulder (soft box) | 8.89 | 3.0 | yes | worst at S1/S2/S3 (np.float64(20.0), np.float64(87.0), -70.0) |
| ROM/arm abduction | no contact | none (old: no contact) | yes | no contact in the tested range; old pauldron None; clasp seat boss 102.5; clasp 5.0; normal 110 GH (171 total) deg |
| ROM/arm scaption (30 deg) | 107.5 | none (old: no contact) | yes | first contact; old pauldron None; clasp seat boss 95.0; clasp 5.0; normal 110 GH deg |
| ROM/arm flexion | no contact | none (old: no contact) | yes | no contact in the tested range; old pauldron None; clasp seat boss 102.5; clasp 10.0; normal 110 GH (165 total) deg |
| ROM/neck lateral flexion to the right | 50.0 | 50.0 | yes | first contact; old pauldron 50.0; clasp seat boss 55.0; clasp 42.5; normal 42 deg |
| ROM/neck rotation to the right | no contact | none (old: no contact) | yes | no contact in the tested range; old pauldron None; clasp seat boss None; clasp None; normal 71 deg |
| ROM/neck flexion | no contact | none (old: no contact) | yes | no contact in the tested range; old pauldron None; clasp seat boss None; clasp 55.0; normal 58 deg |
| ROM/neck extension | no contact | 67.5 | yes | no contact in the tested range; old pauldron 67.5; clasp seat boss None; clasp None; normal 59 deg |
| print/crown watertight + 256 cube | 1.0 | 1.0 | yes |  |
| print/crown wall p5 | 1.41 | 1.4 | yes | nominal 1.6 (keel flange 3-6) |
| print/keel watertight + 256 cube | 1.0 | 1.0 | yes |  |
| print/keel wall p5 | 1.43 | 1.4 | yes | nominal 1.6 (keel flange 3-6) |
| print/spaulder watertight + 256 cube | 1.0 | 1.0 | yes |  |
| print/spaulder wall p5 | 1.56 | 1.4 | yes | nominal 1.6 (keel flange 3-6) |
| mass/armour total <= 182 g (harness model's pauldron) | 10.6 | 0.0 | yes | 171.4 g |
| assembly/crown slides on along -x over the bare S1 base | 3.1 | 0.5 | yes |  |
| assembly/keel rises +z (stage, motor, crown) | 0.49 | 0.3 | yes | flange slides parallel to the plate face |
| assembly/spaulder drops -z onto the ears and seats | 2.2 | 0.3 | yes |  |

**Print per part:** crown: best orientation -x up, support 55.3 cm², wall p1/p5/median 1.04/1.41/1.65 mm; keel: best orientation +x up, support 12.2 cm², wall p1/p5/median 1.37/1.43/1.53 mm; spaulder: best orientation +x up, support 15.9 cm², wall p1/p5/median 1.27/1.56/1.61 mm.

Wearer ROM first-contact angles (deg, 2 mm): {"arm abduction": {"armour": none, "old_pauldron": none, "clasp": 5.0, "clasp_seat_bosses": 102.5}, "arm scaption (30 deg)": {"armour": 107.5, "old_pauldron": none, "clasp": 5.0, "clasp_seat_bosses": 95.0}, "arm flexion": {"armour": none, "old_pauldron": none, "clasp": 10.0, "clasp_seat_bosses": 102.5}, "neck lateral flexion to the right": {"armour": 50.0, "old_pauldron": 50.0, "clasp": 42.5, "clasp_seat_bosses": 55.0}, "neck rotation to the right": {"armour": none, "old_pauldron": none, "clasp": none, "clasp_seat_bosses": none}, "neck flexion": {"armour": none, "old_pauldron": none, "clasp": 55.0, "clasp_seat_bosses": none}, "neck extension": {"armour": none, "old_pauldron": 67.5, "clasp": none, "clasp_seat_bosses": none}}. The clasp row is in contact by design (pads on skin); `clasp_seat_bosses` is the D24 boss each armour seat sits on.

Arm sweep against the old pauldron for comparison: [19.36, [40.0, 87.0, -70.0]]. Overall: **ALL PASS**.

### 1.5 Assembly (replaces the 2026-10-13 sequence, steps 1, 5 and 9)
1. Slide the **crown** along −x onto the bare S1 base (verified free: ≥ 3.1 mm along the slide). It hangs on the base until step 4.
2. Drop the AK70 S1 in through the bore (Ø132 > Ø89), slide the S1 clamp up from below, fit the stop posts (unchanged).
3. Offer the **keel** up from below (+z) so its lip enters the crown hem (verified free: ≥ 0.49 mm). Fit 4 × M3 × 8 through the hem.
4. Bolt the base (4 × M5) and the keel (6 × ISO 10642 M4 × 12) to the plate from the plate front.
5. Yoke, S2, cowl, dome and arm as before.
6. Lower the **spaulder** (−z) onto the ear posts and the two clasp seats (verified free: ≥ 2.2 mm). Fit 4 × M3 × 10 and 2 × M4 × 12 from above.

### 1.6 Open items
- **Clasp F4 node vs Forest's elevation (existing, not changed).** With the GH-share arm model, the clasp's own F4 boss is reached at ~95–102° GH (~150–160° total elevation). The armour adds nothing beyond it, but the clasp node itself limits overhead reach in this model. Check on the first fit (I7).
- `sweep_full.py` (both modes) and the firmware keep-out tables must be re-run with the new pauldron for the elbow, wrist and hand groups. The S1 / M2 / M3 checks pass here.
- Bench: heat-set insert pull-out in annealed PA-CF; lip slip fit on the first print; S1 cable-port position once the AK70 exit is measured.

---

## 2. Previous revision (2026-10-13), kept for history: compact faceted shoulder

**Forest (2026-10-13):** "Make the shoulders not giant spheres." Then: "facet it to my shoulder better, rather than a hinge, add a complex connection". From the options offered, he picked the faceted pauldron.

**Replaced** the 190 mm ball (`ball_shell.py`, retired). **Kept** joints, actuators, limits and stops; still ball-and-socket styled.

### 2.1 Size
| | Old ball | Cowl ∪ dome |
|---|---|---|
| Envelope | 186 × 188 × 149 mm | 150 × 108 × 110 mm |
| Turning-shell mass | 146 g ball + 9.6 g lip | 117 g cowl + 22 g dome |

### 2.2 Socket cowl (M1, turns with S1), PETG — unchanged, current
- **Construction:** offset shell 4 mm round the S2 AK70, clamp, yoke saddle and S2 stop pads. Wall 2.4 mm, ≈ 260 true-plane facets, wall p5 3.2 mm.
- **+y end wall:** bore R 50 for the S2 output / adapter / dome (gap 3.2–3.8 mm, constant over S2).
- **Arm slot:** swept volume of M2 (+ M3 at S3 −75/0/95) over S2 −40..95, grown 3.5 mm; ≥ 3.0 after faceting; < 4 mm (ISO 13857 fingertip).
- **Bottom:** cut at z −49.6, 3.4 mm above the pauldron rim. 2 tabs on the yoke disc (M3 × 8 into inserts at r 56, az 250° / 275°; inserts still to be added to `shoulder_stage.py`).
- **Split** at x = 0, slides on along x with the arm at S2 0; two seam laps (M3 × 10, TPU washer). 117 g.

### 2.3 Humeral-head dome (M2, turns with S2), PETG — unchanged, current
Geodesic icosphere (320 facets) on R 46.4 about P, inner R 43.8. It slides on along −y over the bracket stem and is held by 3 radial M3 × 8 screws into adapter-rim inserts (still to be added to `s2_adapter`). 22 g.

### 2.4 Pauldron rev 2026-10-13 / 2026-10-23 — SUPERSEDED by §1
- Faceted socket housing round the S1 stage.
- 3 struts (18 × 9) to plate feet at y ±48 / z −140 and y 0 / z −150 (2 × M4 each).
- 2 wrap-arms to the clasp seats F4 / F1.
- Split core / lateral / medial with half-lap splices.
- Rev 2026-10-23 replaced the lateral side with a curved deltoid shell and the lateral arm shrank to 3.5 cm³ (it no longer reached F4).
- Meshes in `stl/archive/pauldron_rev2026-10-23/`.
