# SPEC_fairing_study (2026-10-18): hiding the seven joint cable loops (Forest: "minimize joints so a nice chassis can be overlaid without wires everywhere"; chose "keep all 7 DOF")
No geometry in the design was changed. Method: geometry rebuilt from the project scripts (shoulder, upper arm/elbow, forearm, stops, claw, S2 balancer, SDF fields); numbers below are from `fairing.py` diagnostics, `bay_distal.py`, `arm_clockspring.solve`, and hull-area estimates (marked EST).

## 1. Cable standoff (measured, 12 random legal poses, cable centreline to nearest solid)
S1 13.3 / 28.0, S2 20.3 / 28.8, S3 13.4 / 23.0, E 13.0 / 16.0, W1 15.0 / 22.0, W2 11.3 / 15.3, W3 9.7 / 15.3 mm (median / worst). A skin hugging the links (9 mm) leaves essentially all loops exposed (first fairing.py run; shells also failed the watertight check, no usable STLs).

## 2. Through-bore / clockspring bays (`bay_distal.py`: annulus about the joint axis free of every solid at every azimuth and joint angle)
| Joint | Free bay | Clockspring result (bend radius enforced, ribbon OD 1.4-1.8 mm) |
|---|---|---|
| E | R 6..60 mm, h -45..-33 mm (a 12 mm-thick side disc) | feasible only as a pancake: 3 ribbons, 4.7 turns, 1.05 m of cable, outer R 53 mm |
| W1 | R 40..50, h -40..40 | NO solution (needs >= 0.57 turn of unwind inside a 10 mm-deep annulus) |
| W2 | 2 mm tall only | NO bay |
| W3 | R 5..37, h 27..35 | feasible: 3 ribbons, 2.7 turns, 0.4 m, outer R 32 mm |
Conclusion: the "bores for the wrist and elbow" plan does not work as intended: W1 and W2 have no bay, E and W3 need side pancakes (more visible than the loops). S1/S2/S3 bays were studied earlier (`arm_bay.py`) and the free loop was kept.

## 3. Mass of covering the loops (EST, formulas in the run log)
- Closed 1.2 mm PETG/PA skin over all moving groups, hull area 2628 cm2 x 1.2 mm x 1.27 g/cm3 = up to ~400 g (upper bound). +1.0 kPa top pad.
- TPU boots (0.8 mm, 1.21 g/cm3) at all seven joints, D = link + 2 x worst standoff, L 40-70 mm: S1 33, S2 34, S3 18, E 22, W1 13, W2 11, W3 10 = **141 g/arm**, +0.35 kPa on the top pad (2.49 kPa/kg), +0.35 N m at S2 (SF 2.76 -> ~2.6).
- Top pad margin today: 14.4 vs 14.7 kPa (0.3 kPa). Boots alone use it up; they fit only with the pocketed-plate credit (-68 g, -0.17 kPa, unverified, see SPEC_minimization) or with fabric boots (~half the mass, EST).
- A rigid full skin does not fit the pad budget.

## 4. Recommendation
Keep all 7 DOF and the current loops (they verify). Cover with: textile or TPU accordion boots at the seven joints (target <= 90 g/arm), thin printed trim on M3/ME/MW1 only if mass is found elsewhere. Next work: sleeve geometry per joint from `arm_chain_loops.json` swept loops, a swept-envelope check (boot vs neighbours >= 3 mm), mass-budget re-run of `harness_loads`, and renders.
Open: pad-pocket credit verification; fabric boot mass is an estimate; no sweep of boot geometry done.
