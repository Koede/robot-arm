# MJ11 overnight integration summary (2026-11-03, SHOULDER / WING / CLASP / SENSING / CONTROL integration)

Replaces the earlier "nothing merged" stub (written when no overnight branch existed yet). Main now carries all five sessions.

## 1. What was merged
`git ls-remote` showed all five named branches present (clasp, fairing, leads-interfaces, loads-balancer, whole-arm-sweep). Each session was also run more than once in parallel and pushed variant branches (`-b`, `b`, `-w14`, `-sep-body-cover`, `-slots`, `-c`, `-run2`, `-post5`). Only the five exact names were merged; the variants are NOT merged and remain on origin for comparison (notably clasp `-w14`: 14x6 bar at psi 120, 4.7 / 6.2 mm; clasp `-b`: 16x6 at psi 119, 3.8 / 6.2 / 5.4; whole-arm `-run2`: 5 deg curves + proposed S1' tables).
Order: clasp bracket, fairing, leads/interfaces, loads/balancer, whole-arm sweep. Conflicts: CLAUDE.md item 7, DESIGN_STATE.md changelog (all lines kept, dated), SPEC_j1r2_blade_seat.md (both sections kept), `mj_concepts3.py` (bracket function from clasp + fairing functions; psi 119 default and LEAD_PSI/LEAD_W env from leads kept), `.gitignore`, generated renders / JSON (regenerated after the merges). The loads branch also tracked `_render_cache` / `_bal_render` STLs; those were un-tracked and ignored. No branch failed a gate, so none was withheld.
After merging: `mj_eval.py MJ11`, `mj11_sweep.py`, the whole-arm sweep chain (`mj11_whole.py`, `mj11_shoulder_s3.py`, report, tiers, plot) were re-run on the merged parts, because the whole-arm branch had been computed with the old bracket (psi 120, r 58) and no fairing.

## 2. What each session found
- **Clasp bracket:** psi 120 -> 119, anchor r 58 -> 66; bar 24x6 PA-CF. Bracket to S1' group 2.03 -> 4.82 mm (passive box +-5 deg), rail 5.1, bridge 20.1, skin blade 5.4. Finding: the pair M1|bracket had been silently excluded as "designed contact"; S1' now reaches the 3 mm gate on the bracket at +13.4 deg, so the S1' firmware keep-out is +13 deg, reach-free 0.786 -> 0.706. Bar-to-pad lap fastening PLACEHOLDER.
- **Fairing:** 1.8 mm PA-CF C-shell, 42.7 g (mount 186 -> 229 g), 81 x 94 x 112 mm, end plate down, no split; M3 x 5 tabs into 3 heat-set inserts in the pad (PLACEHOLDER). Clearance to moving stage >= 3.2 mm, skin 24.4, rail 9.7, bridge 5.8.
- **Leads / shear / interfaces:** only infraspinatus and teres major lie under the liner; trap_mid, rhomboid, trap_low (6 discs) lie under the now-structural rail, which conflicts with the 2026-10-23 "remove skeleton over electrodes" decision (options A windowed rail / B move pairs / C keep stack). The fixed 8 mm radial lead channels cannot carry a lead laminated to the base layer (22..55 mm across-axis slide vs 2.9 mm allowed): options L1 flat trace / L2 aligned channels / L3 coil. Liner shear 0.42..1.05 kPa dry / 0.84..1.68 wet with a slip face (2.5..4.2 kPa without); preload window 0.11 kPa wide, so accept brief lift-off. New `cad/interfaces.py` WING contract (4 x M4 rail bosses).
- **Loads / balancer:** bolt tension governs (park 228, reach 310, scan max 327, stall 707 N); layout L3 (3 pairs z -150/-190/-230) gives SF 2.4 at stall and 2.7 at x2 scan (pull-out 1 kN UNVERIFIED). No strap preload can close rail / liner lift-off (7x short): use >= 0.3 mm donning squeeze. Gravity torque over the real MJ11 workspace: S1' 7.24 N.m (SF 1.15), S2 4.61 soft / 3.60 box (SF 1.80 / 2.30). Balancer (a) psi 120: 3.27 / 3.27 N.m (SF 2.54) for +335 g/side, top-pad area +3.8 %; cap-only (b) keeps 49 % of poses at 0.5 kg.
- **Whole-arm sweep:** see section 3.

## 3. Merged-geometry re-run (this session)
Whole-arm sweep (S1' x S2 10 deg, S3 3, E 20, W1/W2/W3 3 each = 170,100 poses, 566 s): reach-free 51,151 (30.1 %; branch value on old parts 32.5 %), frontier 32,094 (18.9 %), deep dropped 86,855 (51.1 %). Passive box unchanged: S1' -95..5, S2 -15..17.4, coverage 0.466; shoulder-level reach-free coverage 0.706. Shoulder keep-outs: M1|M3 S2 -21.9 (1.5 mm); M3|clasp bracket S1' >= -6.9, S2 >= 28.2; M3|head first contact S1' 9.4..14.6; M3|skin S2 32.9..44.0; M2|skin S2 40.7..44.7; **new vs the branch run:** M1|bracket at S1' +13.5 (2.2 mm) and M1|fairing at S1' +11.9 (1.06 mm), both beyond the +10 deg hard stop (1.9 deg margin). Tier tables regenerated: park cell FULL 0 / RESTRICT 120 / NEUTRAL 0 deg; over the box RESTRICT 60..120 (never closed), NEUTRAL 0, FULL closed in 9 of 72 cells.

## 4. Gate table (merged parts)
| gate | result | status |
|---|---|---|
| torque SF >= 2 (no balancer) | S1' 1.15, S2 1.80 (soft) / 2.30 (box) at 8.3 N.m rating | **FAIL, waiver or balancer needed; Forest's call** |
| torque SF with balancer (a) psi 120 | 2.54 / 2.54 (cord to S1' group 2.0 mm, gate 3: short 1 mm; not in default geometry) | not adopted |
| collision, parts (3 mm) | passive box + 5 deg hard stops free; fairing to moving stage 3.2 mm (S2 clamp), stop collar 4.2, bracket vs fairing 3.18, bracket vs S1' group 4.82, rail spacers 5.8, bridge 5.8 | PASS |
| collision, hard-stop margins | M1|fairing 11.9 deg and M1|bracket 13.5 deg vs hard stop 10 | PASS (1.9 / 3.5 deg) |
| skin gap | pad 18.3 rail / 10.6 blade; seat dish 5.3 blade; bracket 5.3 blade / 21.9 rail; fairing 24.4 / 25.8; liner -1.7 on blade (accepted design contact, slip face); rule violations: none | PASS (liner exception per Forest 2026-11-02d) |
| head / neck 60 mm | moving groups: first contact S1' >= 9.4 (outside the box). Fixed-parts clearance to the head at neutral (mj_eval `head_neutral_mm`) is 47.8 mm (48.4 before the merge; the fairing costs 0.6 mm), BELOW the 60 mm convention; the sweep does not gate fixed parts against the head. Forest to say whether 60 mm applies to fixed hardware | OPEN |
| frame stiffness / stress | tip sag 0.84 / 1.81 mm; rail 1.3, clasp seat 1.8, bridge 1.0 MPa; liner +-2 kPa, <= 14.7 kPa sustained | PASS |
| bolts | reach 312 N, scan 327, stall 707 N at L0 (SF 1.4 at 1 kN UNVERIFIED); L3 proposal SF 2.4 | CONDITIONAL (L3 not in default geometry) |
| printability (256 mm cube) | fairing 81 x 94 x 112 mm one piece; bracket, pad, dish unchanged; supports only for tab undersides / gland bore (teardrop to do) | PASS |
| mass | mount 228.7 g per side (pad 93.0, dish 56.6 hollow fill, liner 9.9, spacers 2.7, bracket 23.8, fairing 42.7); +335 g if balancer (a) | reported |
| top-pad margin | 2.0 % no balancer; -3.4 % with balancer (a) unless pad area +3.8 % | PASS / FAIL by option |
| watertight / volume | fairing, bracket, pad meshes exported watertight by mj_eval (mass from volume) | PASS |

## 5. Decisions that remain Forest's
1. **Torque gate / balancer:** waiver at SF 1.15 / 1.80; (a) world-anchored spring (psi 120, +335 g, +3.8 % pad area, cord guide + redirect sheave, mirror-tight home); (a)+cap; (b) firmware cap 4.15 N.m (49 % of poses at 0.5 kg); AK70 stall cap 8.3 -> 8.0 N.m.
2. **Firmware table adoption:** shoulder keep-outs (S2 soft min -15, S1' soft max 5, S2 <= 26 for S1' >= -5, S2 <= 31 for S1' <= -85, S1' +13 deg bracket) and the candidate tier headers in `stl/j1r2_sweep/firmware_candidate/` (not drop-in: control.h / protocols.h need an S1' port; firmware lives in the Project, not the repo).
3. Rail over the trap/rhomboid electrodes vs the flex-lead decision (A / B / C); lead architecture L1 / L2 / L3.
4. Bolt layout L3 into the geometry and WING contract; donning squeeze >= 0.3 mm.
5. Clasp variant: merged psi 119 / 24x6 bar vs the alternates on origin (14x6 at psi 120; 16x6 at psi 119).

## 6. Next steps
Tighten the hand envelope (rev B mesh instead of the +-60 mm box) and re-sweep (~9 min); port control.h / protocols.h to S1'; harness_loads / body_loads re-run with pods and boots; insert pull-out coupon (T-W-INS); cord guide and friction for balancer (a) if chosen; L3 bosses in `mj11_geo`; bar-to-pad lap fastening; gland position vs cable run; hard-stop S1' -100 sweep (grid ended at -95).

## 7. Files
Renders: `renders_j1r2/mounts/MJ11_{rear34,rear,side,top,sheet}.png` (regenerated on the merged parts), STLs `stl/j1r2_mounts/MJ11/MJ11_*_{R,L}.stl`, sweeps `stl/j1r2_sweep/`.
