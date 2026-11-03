# SPEC: MJ11 loads + balancer / torque-gate options (SHOULDER / HARNESS / CONTROL), 2026-11-03
Unattended run. Status: ANALYSIS, nothing adopted, no geometry changed. Decisions marked **FORREST** are Forest's call; this file lays out numbers only.
Units mm, N, N.m, kPa. Pad (-60,-190), standoff 12. Frame model `cad/j1r2/mj_frame.py` + `mj_concepts3.mj11_frame`; scripts `mj11_loads.py`, `mj11_loads_stall_layouts.py`, `mj11_free_region.py`, `mj11_balancer.py`, `mj11_bal_pack.py`, `mj11_bal_cordfix.py`, `mj11_bal_home*.py`, `mj11_bal_render.py`. Data `stl/j1r2_mounts/MJ11/{loads,loads_partA,loads_stall_layouts,free_region(.npz),balancer_options,balancer_pack,balancer_cordfix,balancer_home,balancer_home2}.json`. Render `renders_j1r2/mj11_balancer_a_sheet.png`.

## 0. What could not be run
`harness_loads.py`, `body_loads.py`, `sensor_pods.py`, `electronics_fit.py` are not in the repo (DESIGN_STATE 12.7). Their load cases were **ported onto the MJ11 frame model**: arm + payload + fixed module at the pad, rail/liner/clasp foam springs, bolt groups. The frame has NO pods / battery / main box mass and is linear about an unknown preload, so absolute rail/front pressures of the 2-D statics (rev e: rail 1.7-12.5, front 1.6-11.6 kPa) are not reproduced; only arm-induced increments. Top-pad margin is done with the harness_loads 2-D formula. Baseline reproduction: bolt worst 228 / 310 N park / reach (MJ11 spec 231 / 312).

## 1. Loads (right arm, 0.5 kg, other arm at park; increments about preload)
| case | rail kPa (max / min) | liner | top pads | front | axilla | bolt tension N (worst bolt) |
|---|---|---|---|---|---|---|
| park | +0.67 / -1.43 | +0.63 / -0.57 | 1.39 | 0.02 | 0.45 | 228 |
| reach (S1' -90, S2 0) | +1.91 / -3.06 | +2.17 / -1.95 | 1.59 | 0.29 | 0.57 | 310 |
| 32-pose scan (S1' -95..5, S2 -15..35) | +3.34 / -4.31 | +3.98 / -3.04 | 1.84 | -0.51 min | - | **327** (S1' -65, S2 0) |
| x1.3 dynamic (pressures) | -5.6 | -3.96 | | | | x2 dynamic: 456 / 619 / 653 |
| stall S1'/S2/S3 at 8.3/8.3/4.8 (8 sign combos x 4 poses) | +14.1 / -15.3 | +24.2 / -22.4 | 2.9 | 1.7 | 1.3 | **707** (S1' -50, S2 17.4) |
| stall at 8.0/8.0/4.8 (proposal) | +13.7 / -14.9 | +23.5 / -21.8 | 2.8 | 1.7 | 1.2 | 695 |
All pressures are far below 14.7 kPa sustained / 44 kPa transient (stall liner 24.2 is transient, 55 % of 44). Rail beam stress 1.3 / 2.3 MPa (park / reach), clasp arms <= 2.0, spine bridge <= 1.7 MPa. Hand-tip sag 0.85 / 1.83 mm.
Frame note: the frame routes 31-33 N of the ~37 N module weight into the rail foam SHEAR springs (no friction cap) and only 4.6-5.4 N into the clasp top pads. Not credited in section 3.

## 2. Strap preload (no contact lifts off?)
- Front strap as modelled (internal pair cradle <-> F3): 0.000 kPa/N on rail, liner, top. It is closed through the clasp and cannot press the rail or liner onto the back.
- A pull that does press the rig onto the back (-x force at the clasp hubs, hypothetical back strap): +0.050 kPa/N on the weakest rail cell, +0.052 on the weakest liner cell. Needed for no lift-off at x1.3: rail 5.6 kPa -> **112 N per side**, liner 3.96 -> **76 N**. The 32 mm front strap is 2.8 kPa mean at 10 N, limit 4.3 -> about 15 N. **No strap preload closes this** (7x short).
- Consequence: the rail and liner are unilateral and WILL unload by 4.3 / 3.0 kPa (x1.3: 5.6 / 4.0) in the worst poses. Lift-off size: 5.6 kPa x 7.5 mm / 150 kPa = **0.28 mm** (rail), 4.0 x 8 / 150 = **0.21 mm** (liner). Positive increments never exceed 4 kPa, so redistribution is harmless for pressure. What matters is the rig weight / pods moment preload (rev e 2-D, rail 1.7-12.5 kPa), which this frame does not contain: **open, needs the pod masses (harness_loads re-run)**. Donning squeeze of >= 0.3 mm on rail and liner (8-10 % foam strain at 3 mm is not the issue; a 0.3 mm interference is) is the cheap fix; the strap does not provide it.

## 3. Clasp top-pad margin (2-D statics, N_top = W, dyn 1.3, pad area 5124 mm2, target 14.7 kPa)
p_top = 1.3 m g / 5124 mm2 = 0.249 kPa per 100 g. J1 rev 2 reference 2.0 % margin = 14.4 kPa = **5.79 kg per side** (inferred from the 2.0 %; mass limit 5.91 kg).
| case | mass per side | p_top | margin |
|---|---|---|---|
| J1 rev 2 as recorded | 5.79 kg | 14.4 | 2.0 % |
| MJ11, no balancer (mount 186.7 g vs REF mount 98.2 + pad 94.4 g: -6 g) | 5.78 kg | 14.4 | 2.0 % (unchanged) |
| MJ11 + option (a) balancer (fixed ~335 g incl. 88 g spring + 52 g arm side, tube lower on the back) | 6.13 kg | 15.2 | **-3.4 % FAIL** |
Fix for (a): top pad area >= 1.3 x 6.13 x 9.81 / 14.7 kPa = **5318 mm2 (+3.8 %, +194 mm2)**, or shed 0.28 kg. Boot mass (~99 g, open item 4) would take the no-balancer case to 14.7 (0 %); it needs the same area. The rail shear credit of section 1 could remove the issue but depends on foam friction at 0.3-5 kPa normal and is not claimed.

## 4. Pad bolts, PA-CF
Per-pair method of mj_eval; bolt axial load is dominated by the shear couple through the spacer lever on the 8 mm spaced bolt pair (Mz), not by the arm moment. Rail 10 mm node grid for layouts. Pull-out 1 kN UNVERIFIED. Stall columns: reach pose only / worst of 32 cases.
| layout | park | reach | scan max | x2 scan | SF (1 kN) at x2 | stall reach | stall worst |
|---|---|---|---|---|---|---|---|
| L0 baseline 2 pairs z -160/-220, y -113/-105 | 227 | 307 | 320 | 640 | 1.6 | 257 | 697 (SF 1.4) |
| L1 z -150/-230 (80 mm) | 202 | 270 | 278 | 556 | 1.8 | 215 | |
| L2 3 pairs z -160/-190/-220 | 134 | 172 | 182 | 364 | 2.7 | 322 | |
| **L3 3 pairs z -150/-190/-230 (fits the R66 pad: r <= 63)** | 136 | 167 | **184** | **368** | **2.7** | 306 | **413 (SF 2.4)** |
| L4 y spread 12 mm + rail ear, z -160/-220 | 182 | 237 | 246 | 492 | 2.0 | 215 | |
| L5 y 12 mm + ear, z -150/-230 | 176 | 224 | 239 | 478 | 2.1 | 174 | |
| L6 3 pairs z -150/-190/-230, y 12 mm + ear | 115 | 136 | 156 | 312 | 3.2 | 283 | 412 |
Proposal: **L3** (a 3rd bolt pair, spread to z -150/-190/-230). Needs no rail ear: y spread beyond +-4 mm does not fit (rail edge y -100, pad edge y -121, insert 5.6 OD needs 2.5 mm wall: +-6 mm needs a local rail ear, L4-L6). Rail bosses and inserts stay PLACEHOLDER (WING contract); M4 x 20 SHCS, 3 more heat-set inserts. Bolt shear is not limiting: rigid-group shear <= 67 N gravity, <= 138 N stall vs M4 shank (thousands of N). Torque path: the S1' reaction 7.3-8.3 N.m goes through the bolt group as 32-35 N shear per bolt, plus spacer prying above.
PA-CF stress at the pad bolt zone (M4 SHCS, 6 mm plate): with an M4 washer OD 9 under the head, bearing 49 mm2 -> 13.3 MPa at 653 N (x2 scan), 14.4 MPa at the 707 N stall; pull-through shear on 170 mm2 -> 3.8 / 4.2 MPa. Without washer (head OD 7): bearing 28.6 MPa, shear 4.9 MPa. Allowables (PA-CF printed: compressive ~100, interlayer shear ~25 MPa) are assumptions, UNVERIFIED: SF >= 3.5 on washer bearing, >= 6 on shear. Use the washer. Dish: backed by the flat pad face over its whole area, stress = liner pressure (<= 4 kPa gravity, 24 kPa stall); an unsupported 16 mm span at the 3.1 mm ridge gives 0.5 MPa at stall. Not a limit.

## 5. Gravity torque over the REAL MJ11 workspace (new finding)
The 7.31 / 6.59 N.m worst cases were taken over S1' -95..35 x S2 -30..88 (12960 poses). MJ11 can only reach part of that (free region from the real-parts sweep, 5 deg grid, `free_region.npz`: S2 -25..+40 at the lateral S1', -25..0 at S1' +35). Same pose set (E 0/45/90/145, S3 -70/0/90, payload 0/0.5), same 8.3 N.m rating:
| envelope | S1' max | SF | S2 max | SF |
|---|---|---|---|---|
| full old domain | 7.24 | 1.15 | 6.53 | 1.27 |
| reach-free (hard) | 7.24 | 1.15 | 5.00 | 1.66 |
| soft (1 cell inside) | 7.24 | 1.15 | **4.61** | **1.80** |
| passive box S1' -95..5, S2 -15..17.4 | 7.24 | 1.15 | 3.60 | 2.30 |
S1' worst pose S1' -55, S2 0, E 45, S3 -70, payload 0.5. **S2 is not the problem: SF 2 on S2 needs S2 <= ~4.15 N.m, i.e. cap S2 at about +30 deg (or use the box) at no balancer.** S1' is binding everywhere.

## 6. Decision table (S1' is the binding axis; gate SF >= 2 vs 8.3 -> <= 4.15 N.m)
| option | S1' / S2 N.m (SF) | cost | status |
|---|---|---|---|
| 0 none (today) | 7.24 / 4.61 (1.15 / 1.80) | 0 g | fails gate, needs waiver |
| **a** world-anchored zero-length spring, refit; anchor ABOVE the S1' axis only (psi 100-130 deg; psi 150-330 cannot balance: 6.5-7.4 residual) | psi 120: **3.27 / 3.27 (SF 2.54)**; psi 110: 2.94 / 2.87 (2.82); psi 100: 2.75 / 2.76 (3.01) | +335 g/side fixed (spring 88-152 g), top pad +3.8 % area | feasible, see 6a |
| b firmware cap at 4.15 | holds SF 2 by refusing poses | workspace: payload 0 -> 95.7 % of poses; payload 0.5 -> **48.6 %** of poses (5.4 % of (S1', S2) cells allowed in every E/S3). Cap 5.0 (SF 1.66): 63 % at 0.5 kg; 6.0 (SF 1.38): 79 %; 7.0 (SF 1.19): 98 % | does not reach SF 2 at 0.5 kg without halving the pose space. Hold heat ~ (tau/8.3)^2: 25 % of rated at 4.15, 76 % at 7.24 (vendor thermal data PLACEHOLDER) |
| c payload / links, no balancer | arm alone (0 kg) 4.34 / 2.37 (SF 1.91 / 3.5): **no payload gives SF 2 at the current links**. Links x0.9: 4.05 at 0 kg, 6.70 at 0.5; x0.7: 3.50 / 5.64; x0.6: 3.25 / 5.13 (0.25 kg: 4.18 at x0.6). SF 2 at 0.5 kg needs links < x0.5 | links/mass redesign | not viable for the 0.5 kg spec |
| d1 ideal th1-only cam / cassette inside the pad (S1') + phi-only S2 cassette | S1' 2.83 (SF 2.93), S2 2.17 (3.8) lower bounds; a K sin(th1+d) spiral gives 2.9 (K 4.5, d -36); K sin(phi) for S2 does nothing (4.61 stays) | fits in the 50 mm pad volume only if a cam is designed | NEW mechanism, not designed (PLACEHOLDER); S2 needs a phi cam. Pose-aware spring (a) does better on S2 |
| d2 gas / constant-force strut at the arm | not evaluated | | open |
6a. Option (a) numbers (`balancer_cordfix.json`, 261 soft cells x 24 E/S3/payload variants, 52 g arm-side hardware included): psi 120, A = world (-45, -85, -124), B arm-local (-0.1, -30, 112), k_cord 0.96 N/mm, F_cord max 130 N (spring 260 N at 2:1, A228 d 3.5 D 36, 8.4 coils, free length 179 mm, tau 0.34 Sut, 88 g). Cord to the S1' group 2.0 mm (**gate 3: short by 1 mm, needs a cord guide / stand-off sheave**), to the arm 17.9 mm. No friction hysteresis, spring failure, T20 test or firmware gravity-model change done (SPEC_j1r2_balancer.md sections 4-6 carry over, numbers to be redone).
Packaging (`balancer_pack.json`, `balancer_home2.json`): the old radial tube from A **fails**: psi 100/110 sweep through the arm (0.1-0.4 mm); psi 120/130 clear the arm (24-42 mm), S1' group (24 mm), skin (53), head (67-77 mm, no longer a problem) but the 191 mm tube crosses the spine and hits the mirror tube. **Home found:** vertical tube D44.5 x 191 hanging below a redirect sheave on the lower back: axis x -84 (62 mm behind the back), y -122, z -300..-491; skin 20 mm, mirror tube 11.5, wing rail 12, S1' group 15, arm sweep 3.8, fixed MJ11 parts 52, head 225 mm. z_top -270 (y -126) passes with 3.5 mm mirror gap; higher homes collide with the spine bridge / AK70. Cost: second sheave and a cord run of ~180 mm that has to go around the AK70 (R44.5), not designed (cord guide PLACEHOLDER), +5 % friction per sheave (SF 2.3 at 5 % hysteresis estimated), tube 62 mm behind the back (S1' motor is at 67 mm already).
Render: `renders_j1r2/mj11_balancer_a_sheet.png` (tube drawn as a proxy; cropped torso ends above the tube end).

## 7. Recommendation to weigh (not a decision)
Combine: bolt layout L3 (cheap, +3 inserts, SF 2.4-2.7); donning squeeze >= 0.3 mm instead of a strap preload; for the torque gate (a) is the only option that reaches SF >= 2 at 0.5 kg over the full workspace (2.5-3.0) but costs 335 g, a top-pad +3.8 % area change, a cord-routing design and a mirror/spine-tight home; (b) alone keeps 49 % of the pose space at 0.5 kg. A middle path is (a) with a smaller spring tuned to S1' only plus the S2 +30 deg cap. **Forest decides** between (a), (a)+cap, (b), waiver at SF 1.15 / 1.80, and the AK70 stall cap 8.3 -> 8.0 (changes bolt stall tension by -1.7 %, pad stall unchanged in practice).

## 8. Open
1. harness_loads / body_loads re-run with pods, battery, boots (rail/front absolute pressures and true preload). 2. Insert pull-out coupon (1 kN assumed). 3. Bolt layout L3 into `mj11_geo` / WING contract (bosses z -150/-190/-230). 4. Balancer: cord guide, friction, firmware gravity model, free fall. 5. Skin sign of the cord census was unreliable near the hip/arm mesh and was dropped; only part clearances are reported.
