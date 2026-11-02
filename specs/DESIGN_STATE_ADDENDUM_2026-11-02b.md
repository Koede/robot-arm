# DESIGN_STATE addendum 2026-11-02b: J1 mount round 2, spine-led, no balancer
- Request (Forest): "No balancer maybe" then "reinforce on the spine more or even extend the spine to cover more of the back, bridge from there".
- Built MJ7 (spine ladder), MJ8 (K-truss, long leg), MJ10 (K-truss to the spine head); MJ9 (straight CF mast) dropped (50 mm proud of the back). Spec: specs/SPEC_j1r2_spine_mounts.md. Code: cad/j1r2/mj_concepts2.py, mj_eval.py (balancer kind "none").
- Result: reach sag 4.7-5.2 mm (MJ4 5.3), but the 322 N bolt group of MJ4 becomes posts <= 20 N, rail stress 5.3 -> 0.6 MPa, rail unload -4.9 -> -1.8 kPa (MJ7, MJ10). Cost +20..55 g/side, +19..34 mm depth. Lever alone (MJ8) does not help the bolts (370 N).
- OPEN, Forest's call: (1) mount choice (MJ4 bare vs MJ10 vs MJ7); (2) no balancer means shoulder SF 1.14 vs rated at the worst pose (gate needs 2): waiver, firmware torque cap (reach envelope not yet computed), or later cassette balancer.
