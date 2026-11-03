# SPEC: MJ11 loads re-run and torque-gate options (SHOULDER / HARNESS), 2026-11-03
Unattended run. Branch `claude/mj11-loads-balancer-2026-11-03`. Nothing here changes geometry, limits or firmware; every choice marked **Forest** is his call.
Scripts: `cad/j1r2/mj11_loads.py`, `mj11_torque_options.py`, `mj11_balancer_refit.py`, `mj11_balancer_alt.py`, `mj11_balancer_render.py`, `mj11_figs.py`. Data: `stl/j1r2_mounts/MJ11/loads.json`, `loads_with_balancer.json`, `stl/j1r2_bal/mj11_*.json`. Charts / render: `renders_j1r2/mounts/MJ11_loads_balancer_charts.png`, `MJ11_balancer_*.png`.
Run: `J1R2_ROOT=/home/claude/ts python3 cad/j1r2/<script>.py` (work root per docs/ENV_SETUP.md).

## 0. Method and what could not run
`harness_loads.py`, `body_loads.py`, `sensor_pods.py`, `electronics_fit.py` are not in the repo (DESIGN_STATE 12.7). Their load cases were **ported onto the MJ11 frame model** (`mj_frame.py` + `mj_concepts3.mj11_frame`, linear about the strap preload). It reproduces the adoption numbers (tip 0.85 / 1.81 mm, bolt 231 / 312 N, rail unload -3.1 kPa, liner -2.0 kPa). Consequence: pressures are **increments**; absolute top-pad pressure (old 14.4 vs 14.7 kPa) needs the old solver, so the top pad is reported as a preload budget (1.2). Boot mass (99 g per arm) is not in the frame model; it is included in the torque study (+0.48 N.m on S1').
Added over `run_frame`: pose scan over the MJ11 box (S1' -95..5 step 10, S2 -15/0/17.4, 0.5 kg), x2 dynamic (increments scale, preload does not), stall (8.3 N.m about S1' and about S2) and S1' bumper-plateau (20 N.m) reactions on the pad, strap preload (lift-off and friction), per-bolt tension (beam-group model with prying) and a rigid-plate/dowel statics bound, bolt layouts, PA-CF hand checks. Assumptions (PLACEHOLDER, coupons needed): foam-on-base-layer mu 0.5, PA-CF XY UTS 70 MPa x 0.5 creep derate (bearing allowable 17.5, shear 15 MPa), M4 insert pull-out 1 kN (UNVERIFIED), M4 8.8 steel.

## 1. LOADS
### 1.1 Rail bolts (worst bolt, N, beam-group model)
| layout | reach x1 | reach x2 | S1' stall 8.3 (-) | S1' stop 20 N.m (-) | pose-scan worst x1 | pad edge ligament |
|---|---|---|---|---|---|---|
| L0 baseline: 2 rows (z -160/-220), pair y -113/-105 | 312 | 625 | 613 | **1038** | 329 | **2.85 mm (e/d 1.27)** |
| L1 edge-corrected pair (y -111/-103) | 315 | 630 | 628 | 1069 | 333 | 4.6 |
| L2 + 3rd row (z -160/-200/-220) | 190 | 380 | 376 | 639 | 201 | 4.6 |
| **L3 4 rows (z -160/-180/-200/-220), y -112/-103** | **121** | **242** | **234** | **407** | 132 | 3.7 (e/d 1.5) |
| L4 3 rows near centre (z -180/-200/-220) | 192 | 385 | 373 | 627 | 203 | 1.1 (rejected: breaks out) |
Insert pull-out SF (1 kN UNVERIFIED), L0: 3.2 static, **1.6 at x2**, 1.6 stall, **0.96 at the S1' stop plateau**. L3: 8.3, 4.1, 4.3, 2.5. Dowel/rigid-plate bound (shear on dowels, bolts axial only): 22 N axial, shear 65 N (x1) to 209 N (stop) per dowel: the 231 to 312 N is prying across the 4.8 / 8.3 mm spacers, an 8 mm bolt pair resisting a couple. Adding rows and spreading the pattern in z cuts the couple; the pair spacing cannot grow (pad trimmed at y -121, rail edge y -100, bolt radius <= 60 from the axis).
**Finding:** baseline bolts at (y -113, z -160/-220) sit 60.9 mm from the pad axis: 5.1 mm to the R66 rim, 2.85 mm ligament. Move to y <= -111 or use the 4-row pattern.
Fixes in order: (1) L3 (4 rows on the 20 mm rail-node pitch, two M4 per row; insert web between pair 3.4 mm: confirm boss), (2) two dowels (4 h7 pins in the spacers) so the bolts only clamp, (3) re-measure insert pull-out in PA-CF (coupons) before relying on 1 kN. Mass of the extra rows about 4 g.

### 1.2 Pressures, strap preload, top pad
Worst unload over the box (pose th1 -65, S2 -15): rail -4.37 kPa, liner -3.07 kPa (x2: -8.75 / -6.15). Reach park/reach: rail -1.49 / -3.12, liner -0.60 / -1.98 (reproduced). Stall S2: rail -11.7, liner -19.4 kPa (transient, current-limited).
Vertical load goes into rail foam shear: 35 N (x2 70 N). Friction needs normal 70 N: **rail p0 >= 4.9 kPa (x2 9.7), liner p0 >= 3.1 (x2 6.2)** (mu 0.5). Uniform preload force per side: **102 N static, 203 N at x2**. If that is reacted on the clasp front pads (effective 5531 mm2) they carry **18.4 kPa (x2 36.7), over the 14.7 limit**. Preload that follows the unloading only (non-uniform; needs the strap to act near the pad) is 21 N (x2 42 N) = 3.8 kPa (7.5). So the binding constraint on MJ11 is strap preload distribution, not arm load. Needs a strap/pad layout decision (**Forest**): preload tab on the pad rim and the rail's lower end.
Top pads F1/F4: arm-load increment 1.5 kPa park, 1.9 kPa worst pose (x2 3.8), 4.8 / 5.6 N (REF 10.5 / 6.0 N). Margin stays >= 2 % while the strap puts <= 12.5 kPa (x1) / 10.6 kPa (x2) on them. Adding the 355 g balancer changes this by +0.15 / +0.3 kPa. Absolute margin vs 14.4 / 14.7 needs the missing harness solver: open.
### 1.3 PA-CF stress (L0 vs L3; x2 reach / S1' stop 20 N.m; allowables 17.5 bearing, 15 shear, 35 bending MPa)
| check | formula | L0 x2 | L0 stop | L3 x2 | L3 stop |
|---|---|---|---|---|---|
| washer (OD 9, hole 4.5) bearing, through thickness | T / (pi/4 (9^2-4.5^2)) | 13.1 | **21.7 > 17.5** | 5.1 | 8.5 |
| shear-out at the rim | V / (2 t (e - d_h/2)) | 3.8 | 6.1 | 2.5 | 4.1 |
| in-plane hole bearing | V / (d t) | 5.4 | 8.7 | 4.7 | 7.7 |
| pad strip bending (6 mm, 78 mm wide, between motor boss and bolt line) | M/Z | 7.1 | 12.9 | 7.2 | 12.9 |
| M4 von Mises (no preload) | sqrt(sigma^2+3 tau^2) | 76 | 125 | 35 | 59 (yield 640) |
Dish (2.4 mm shell wall, 30 mm rib span, foam p 17.8 kPa incl. x2 preload): 0.75 p (L/t)^2 = 2.1 MPa, no issue; dish is not load path for the bolts.
### 1.4 Loads verdict
Park/reach/x2 survive with the baseline only if the 1 kN insert holds (SF 1.6). The S1' stop plateau fails it (0.96) and the washer bearing, and the edge ligament is 2.85 mm. L3 passes everything. Stall S2 unloads the liner by 19 kPa transiently: the preload cannot cover it, the 300 ms dead-man and current limit do.

## 2. TORQUE GATE
Gate: |tau| <= 8.3/2 = 4.15 N.m. Gravity torque is translation invariant, so the pad move does not change it. **What MJ11 changes is the reachable box** (S1' -95..5, S2 -15..17.4).
| | S1' max (SF) | S2 max (SF) |
|---|---|---|
| J1 rev 2 soft range, no balancer (my grid; spec 7.31/6.59 included the 52 g collar) | 7.24 (1.15) | 6.53 (1.27) |
| **MJ11 box**, 0 / 0.25 / 0.5 kg | 4.34 / 5.78 / **7.23 (1.15)** | 1.65 / 2.62 / **3.60 (2.30)** |
| MJ11 box + 99 g boots | 7.71 (1.08) | 3.74 (2.22) |
**S2 already passes in the MJ11 box; only S1' fails**, worst pose th1 -55, phi -3, E 45, S3 -70, 0.5 kg. Empty arm alone is 4.34 (SF 1.91). Holding at 7.23 N.m is 87 % of rated (copper loss 0.76 of rated).

## 3. DECISION TABLE (not chosen; Forest)
| opt | what | S1' residual (SF) | S2 (SF) | mass / cost | workspace / payload | status and risks |
|---|---|---|---|---|---|---|
| 0 | do nothing, waive gate | 7.23 (1.15), boots 7.71 (1.08) | 3.60 (2.30) | 0 | full box | fails gate |
| **a1** | rev 2 pad-anchored zero-length spring, unchanged A/B/k/L0 | 2.93 (2.83) | 2.78 (2.98) | 406 g (355 g fixed + 52 g arm) per arm, loads +2 to 5 % (loads_with_balancer.json) | full box, payload 0..0.5 | **No home found.** See 3.1 |
| a2 | refit to the box (A, B, k, L0) | 2.62 (3.17) | 2.49 (3.33) | spring 124 g at 2:1 (tube 280 mm), 116 g at 4:1 (173 mm) | same | gain 0.3 N.m: not worth a change; same packaging and cord clash |
| b | firmware cap 4.15 N.m, no hardware | cap 4.15 (2.00) | cap | 0 | 0 kg: 94 % of box poses allowed; 0.25 kg: 58 %; **0.5 kg: 45 %**, and with an elbow floor still only 57 % of (th1, phi) cells can be held at any E (43 % none, even E 145). Needs keep-out table + derating; S2 unaffected | lowest cost, loses most of the loaded reach; unpowered fall unchanged |
| c1 | payload cut, links unchanged | empty arm already 4.34 | | 0 | **no payload meets SF 2** at 230/180/90 mm | not possible |
| c2 | shorter distal links (masses fixed, conservative) | 0.9: 4.03 / 0.8: 3.73 empty; max payload at SF 2: 0.023 kg (0.9), 0.089 (0.8), 0.17 (0.7), **0.27 (0.6)**, 0.39 (0.5) | | redesign | arm 40 % shorter for 0.27 kg | not credible as the only fix |
| d1 | S1'-only torsion spring (clock/negator), best linear fit 3.10 - 0.85 th (rad) = 4.5 N.m at -95 deg to 3.0 at +5 deg, 6.6 J | 3.40 (2.44); with boots 3.49 (2.38) | 3.60 (2.30) | est. 60 to 100 g (PLACEHOLDER) | full box | **passes SF 2 with no cord, tube or neck problem**; **packaging unresolved**: annulus around the AK70 is the 15 mm stop-collar wall (r 44.6-60), 5 mm to the R66 pad edge. A 4.5 N.m negator needs about 25 mm x 0.9 mm strip; needs the stop collar and posts redesigned |
| d2 | S1' cam/cassette, arbitrary tau(th1) | bound 2.65 (3.13) for any th1-only device; 2.66 sinusoid; 0.94 if payload fixed empty | 3.60 | | | lower bound on all cassette/cam alternatives; not designed |
| d3 | counterweight | needs about 7 kg at 100 mm | | | | rejected |
| d4 | decoupled baseline-style pair (rev 2 search) | 4.34 (1.91), full range | | | | still < 2 |
Free fall / stops / spring failure for a1 are unchanged from SPEC_j1r2_balancer (stop energy S1' +4.5 / -5.6 J, snap step 5.1 N.m S1'): MJ11 S1' stop loads in 1.1 matter more than before.

### 3.1 a1 packaging on MJ11 (real wearer mesh, MJ11 mount parts, rail, bridge, mirrored left tube, hard sweep S1' -100..10, S2 -20..22.4)
Anchor A = P1 + (-10.0, -19.7, +73.4): world (-45.3, -79.7, -116.6). Cap clears the S1' group by 16.2 mm, the post by 3.8 mm (gate 3). Tube direction psi (yz plane, from +y toward +z), 2:1 230 mm / 3:1 175 mm:
- psi 0-60 (lateral/up-lateral): the tube meets the arm sweep (0.0-0.1 mm); psi 0 also the S1' group (0.1 mm).
- psi 120-240 (medial / down-medial): the tube crosses the spine and meets the mirrored left tube (0 mm); psi 210-240 also the S1' group (4.6 / 0.0 mm).
- psi 270-330 (down / down-lateral): S1' group and arm sweep 0.0 mm (the S2 motor hangs there at th1 -90); psi 330 goes through the body.
- **psi 90-100 (up, beside the spine): clean**: S1' group 29 mm, arm 18-40 mm, skin 27 mm, head 66 mm (gate 60), left tube 15-75 mm. **But neck extension first contact is 22-45 deg** (tube end z +9 at 120 mm: 42.5 deg; 151 mm: 37.5; 175 mm: 32.5; 230 mm: 25-30), which restricts the wearer. The head keep-out is met; the neck-ROM problem replaces it. A tube short enough to be neck-free would be under about 100 mm: no spring option (2:1 280, 3:1 215, 4:1 173 mm for the refit; 230/175/151 for rev 2).
- **Cord:** the straight cord A-B penetrates the S1' group at th1 -100..-70 (73-85 of 258 poses < 3 mm, min -1.1 mm) for rev 2 and for the box refits (a random search near the fits found none clear). Needs a fairlead / second sheave on the yoke (not designed).
Render: `renders_j1r2/mounts/MJ11_balancer_rear34.png` (orange = candidate home at psi 95, 3:1).
**Conclusion:** the old radial-tube problem does not disappear. It changes from the head to the neck-extension range (and the cord clash stays). Option d1 avoids both but needs the stop collar re-packaged.

## 4. Recommendation to Forest (for the decision, not taken)
1. Loads: adopt L3 (4 rows) or dowels regardless of the balancer; it is cheap and removes the 0.96 insert SF and the 2.85 mm ligament. Decide the strap preload layout (18 kPa on the front pads at uniform preload).
2. Torque: S2 needs nothing in the MJ11 box. For S1' the real choices are d1 (spring on the S1' axis, if the stop collar can be moved), a1 with a neck-extension waiver and a cord fairlead, or b (cap, loses 55 % of loaded poses). c is not viable.
3. The pad stall cap (8.3 -> 8.0 N.m) is unaffected by this study.
Open: strap preload layout, absolute top-pad pressure (missing solver), insert and PA-CF coupons, foam mu, d1 spring packaging, a1 cord fairlead, firmware tables (cap b would need them).
