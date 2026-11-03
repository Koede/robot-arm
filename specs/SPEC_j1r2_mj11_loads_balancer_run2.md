# SPEC: MJ11 loads + torque gate / balancer options (2026-11-03)

Subsystems: SHOULDER / HARNESS / CONTROL. Unattended run. **No geometry, limit or firmware-table change was made.** Decisions below are Forest's.
Scripts (cad/j1r2/): `mj11_workspace.py`, `mj11_torque.py`, `mj11_balancer_fit.py`, `mj11_bal_options.py`, `mj11_bal_sizing.py`, `mj11_loads.py`, `mj11_bal_pack.py` (incomplete, see 3.5), `mj11_figs.py`. Data: `stl/j1r2_mounts/MJ11/{workspace_mask,torque_options,torque_options2,torque_options3,bal_fit_*,loads}.json`. Figure: `renders_j1r2/mj11_loads_balancer.png`.

## 0. Method / port statement
`harness_loads.py` and `body_loads.py` are not in the repo and were tied to the retired rig, so their load cases were **ported** onto the MJ11 frame model (`mj_frame.py`: rail beams on foam, spine bridge, clasp pads, liner foam, pad on 2 M4 pairs) plus the 3-group statics of `harness_loads.py` (`statics_port`). Dynamic factor 1.3 on loads, x2 on bolt tension. Pad (-60,-190,12). Rig-extras (electronics/boot masses) added at estimated positions: PLACEHOLDER positions, effect <= 2 kPa / 3 N bolt tension (rig_extras True vs False in loads.json).
Workspaces: FULL (J1 grid, 540 cells), REACH (collision-free, reachable: 329 cells, `mj11_workspace.py`), BOX (passive-stop box S1' -95..5, S2 -15..17.4, hard stops +-5).

## 1. Job 1: loads (park and reach th1 -90, 0.5 kg, x1.3)
| Item | Result | Limit | Verdict |
|---|---|---|---|
| Rail peak pressure | 0.8 park / 1.9 reach (kPa) | 14.7 sust | pass |
| Rail unload (negative) | -1.6 park, **-3.1 reach** (-3.4 both arms) | 0 | lifts off |
| Liner | +0.7 / -0.6 park; +2.2 / **-2.0 reach** | 0 | lifts off |
| Top pad (3D, with shear springs) | 1.5-2.4 kPa | 14.7 | pass |
| Axilla | 0.5-0.7 kPa | 4.3 | pass |
| Top-pad margin, sagittal statics (harness_loads port) | p_top 11.0-12.2 kPa vs 14.7 -> margin ~17 % (was 2 %) | >0 | pass; 3-D result depends on rail friction (strongest assumption: mu 0.2-0.6) |
| Bolt tension, V0 as adopted, x2 | 457 N park, **620 N reach**, 672 N both arms | insert ~1000 N pull-out (UNVERIFIED) | insert SF 2.19 / **1.61** / 1.49 |
| Bolt shear per bolt x2 | 56-70 N | | fine |
| S1' stall at cap 8.3 / 8.0 | 611 / 600 N (SF 1.64 / 1.67) | | below 2 |
| S1' stall at AK70 peak 24.8 | 1210 N (SF 0.83) | | **fails** unless current-limited |
| PA-CF pad ear bending (6 mm, 105 mm chord, 17 mm lever) | 16.7 MPa | 35-45 MPa (0.5 x 70-90, UNVERIFIED) | pass |
| Hole bearing / head bearing | 2.6 / 27.5 MPa (6.4 with M4 OD12 washer) | ~100 crush UNVERIFIED | pass; use washers |
| Seat dish | liner <= 8.9 kPa, plate < 0.1 MPa | | not the weak path |

**Finding: the pad bolt/insert, not the plate or dish, governs.** The upper pair (z -160) carries nearly all tension.

**Strap preload.** With no preload the rail and liner lift off (above). Uniform pre-pressure p0 needed to keep every contact >= 0 (liftoff-only): park rail 2.1 / liner 0.8 kPa; reach rail 4.2-4.5 / liner 2.5-2.6 kPa. With friction (mu 0.4, shear capacity bisection, slip not allowed) p0 = 6 kPa; 4 kPa if slip allowed at mu 0.6. This needs a **closure force 56 N (p0 3) to 112 N (p0 6) per side**, against ~5 N x-component of the front strap at the 10 N spec (6.8 kPa width-averaged, line limit 4.3 kPa). **The strap cannot supply it**; the 2-pair pad must carry the unload in tension instead, or the contacts are allowed to lift. Options: accept designed lift-off at the extreme reach (it is bolt-held, stays within bolt limits with a fix below); or add a compliance-neutral preload (spring-loaded rail pad), Forest's call.

**Bolt fix table (reach th1 -90, x2, insert 1 kN)**
| Variant | Layout | Tension N | Insert SF | Stall at cap 8.0: SF | Notes |
|---|---|---|---|---|---|
| V0 (adopted) | 2 pairs z -160/-220, 8 mm apart | 620 | 1.61 | 1.67 | fails 2 |
| V1 | 2 pairs, 12 mm apart | 476 | 2.10 | 2.18 | pad tab |
| V2 | 2 pairs z -140/-240 | 470 | 2.13 | 2.11 | pad tab |
| V3 | 3 pairs -160/-200/-240 | 397 | 2.52 | 2.61 | pad tab |
| **V4** | **3 pairs -140/-200/-260 (120 mm)** | **306** | **3.27** | **3.27** | pad tab; recommended |
| V5 | 3 pairs + 12 mm pairs | 342 | 2.93 | 3.09 | |
| V6 | wide z + 12 mm | 415 | 2.41 | 2.41 | |
V4 is also the only layout with margin at AK70 peak stall (SF 1.6 at 24.8 N.m), so **the S1' firmware cap is a load-bearing safety item**, not only a thermal one. Rail unload is unchanged by layout (-3.5 kPa V4): bolts do not cure lift-off, only carry it.
Pad-tab geometry and insert pull-out are **UNVERIFIED** (pull-test M4 x 5.6 heat-set in PA-CF before ordering).

## 2. Job 2: torque gate (AK70-10 rated 8.3 N.m; SF >= 2 needs <= 4.15)
Gravity torque, 0.5 kg payload (0 kg): worst S1' **7.24 (4.34) N.m, SF 1.15 (1.91)**, at th1 -55, phi 0, E 45, S3 -70. Worst S2: FULL 6.53 (SF 1.27); REACH 5.00 (1.66); **BOX 3.60 (SF 2.30; 3.85 at hard stops, SF 2.15)**. So **S2 passes inside the MJ11 box; S1' fails everywhere**, even empty (53-59 cells over cap with empty arm).
Thermal (lumped AK70, thermal.h): steady torque limit 7.17 N.m (S1'), 8.02 (S2); 0.5 kg holds are thermally fine, the gate is the SF rule.

## 3. Decision table (Forest decides)
| Option | What | S1' / S2 N.m | SF (S1'/S2) | Mass | Workspace / payload cost |
|---|---|---|---|---|---|
| 0 Baseline | none | 7.24 / 3.60 (BOX) | 1.15 / 2.30 | 0 | fails gate |
| a1 Balancer, J1 rev 2 geometry | zero-length-type spring, 2:1/3:1/4:1 tackle, A228, tube 230/175/151 mm | residual 2.91 / 2.76 | 2.86 / 3.01 | 403 / 385 / 394 g | none; needs home on lower back |
| a2 BOX refit (min residual) | A (-0.5,-18.1,67.6) on pad frame, B (-1.6,-50,134.9), k 0.584 N/mm, L0 32.8 | 2.65 / 2.54 | 3.13 / 3.27 | 443 / 421 / 386 g (2:1/3:1/4:1), cord force 98 N | none within box |
| a3 BOX light (residual <= 3.5) | k 0.40, L0 20 | 3.50 / 2.69 | 2.37 / 3.08 | 387 / 374 / 387 g, cord 73 N | none |
| a4/a5 REACH refit/light | wider workspace | 2.80 / 2.40 ; 3.50 / 3.10 | 2.96 / 3.46 ; 2.37 / 2.68 | 403-386 g | none; covers 329-cell REACH |
| b Firmware cap 4.15 | no hardware | capped | 2.0 by construction | 0 | BOX configs reachable at 0.5 kg: **44.5 %** (REACH 51 %); at 0.25 kg 57 %, empty 94 %. Cap 6.0: 75 % at 0.5 kg (SF 1.38) |
| c1 Payload cut | no balancer | | | | SF 2 needs payload **0.0 kg** (S1') ; empty arm already SF 1.91 |
| c2 Link/arm mass scale | | | | | arm mass x0.24 at 0.5 kg; x0.95 empty. Links x0.7 gives SF 1.49 at 0.5 kg, still < 2 |
| d1 Constant-torque negator / cam (th1 only) | floor 2.65 N.m residual | 2.65 | 3.13 | not sized (spiral fit degenerate, ignore) | none; needs vendor-sized negator, PLACEHOLDER |
| d2 Ideal pose-only floor | bound | 2.56 / 1.82 | 3.24 / 4.56 | | theoretical limit of any passive balancer on BOX |

Residuals include the AK70/yoke M1 masses on S1' only.

### 3.5 Packaging (INCOMPLETE)
Tube-direction scan on the lower back (`mj11_bal_pack.py`) was stopped at 230 mm tube, directions 0-135 deg: all fail. Cord-to-M1 clearance is 0.3 mm at every direction (the pad-anchored cord passes the S1' motor), M1 sweep 0.1-3.9 mm, head >= 62.6 mm OK, wing rail 14.7 mm, M2 sweep clear only for 90-135 deg (18-42 mm). The radial 105 deg case also failed neck extension (first contact 22.5 deg, needs 70). So **no packaging home is demonstrated**; the cord anchor A must move (a3 variants have A within 22 mm of the pad axis) or the cord must route around M1. 150-345 deg and 175/151 mm (3:1/4:1) tubes not scanned. Treat all a-options as **mass/torque feasible, packaging unproven**.

## 4. Recommendation (not a decision)
Do now regardless: V4 bolts (3 pairs, tab, washers), S1' firmware cap as safety item. Torque gate: a2/a3 with 4:1 tackle (tube 161-169 mm, ~390 g) is the only option reaching SF >= 2 at 0.5 kg without a workspace cut; option b alone gives SF 2 at 44 % of configs at 0.5 kg. Needs the packaging scan finished.

## 5. PLACEHOLDER / UNVERIFIED
Insert pull-out 1 kN; PA-CF allowables; rig-extra positions; mu 0.2-0.6; strap geometry; pad tab; negator/cam sizing; spring-sizing (tau/Sut ~0.3, A228); balancer mass of tube/cap/post.

## 6. Open decisions for Forest
1. Adopt V4 bolts? 2. S1' cap value (8.3 -> 4.15 for gate, or 8.0 proposal withdrawn as irrelevant to SF; AK70 peak is the real stall) and whether to derate payload. 3. Fund the balancer (a2/a3) and finish packaging, or accept cap/payload (b/c). 4. Accept rail/liner lift-off at extreme reach, or add spring preload.
