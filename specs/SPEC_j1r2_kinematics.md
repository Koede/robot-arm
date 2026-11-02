# SPEC: J1 rev 2 kinematics, limits, control integration (SHOULDER / CONTROL), 2026-11-01
Code: `cad/j1r2_kin.py` (+ `j1r2_params.py`). Report: `stl/j1r2_kin/j1r2_kin_report.json`, `passive_box_study.json`, proxies in `stl/j1r2_kin/proxies/`. Forest override (2026-11-01): shoulder is no longer ball-styled/concurrent; DOF, actuators (AK70-10 x2, XM540 S3), passive stops and the 5 deg rule are kept.
## 1. Kinematics (product of exponentials, robot frame, home th1 = phi = 0)
T(q) = e^{xi1 th1} e^{xi2 phi} e^{xi3 S3} e^{xiE E} e^{xiW1 W1} e^{xiW2 W2} e^{xiW3 W3} M; e^{xi q} = Tr(p) Rot(w,q) Tr(-p).
| joint | w (home) | point p | note |
|---|---|---|---|
| S1' | +x | P1 = (PX, y1, z1) | AK70 flat on the pad, back-normal axis |
| S2 | -y | P1 | in-plane axis s(th1) = (0, cos th1, sin th1); d = a cos(phi) - xhat sin(phi); + swings forward (-x) |
| S3 | +z | O0 = P1 + (0, T_ARM = 105.75, 0) | XM540 on the arm axis |
| E | -y | O0 + (0,0,230) | E>0 swings the forearm toward -x (as baseline) |
| W1 / W2 / W3 | +z / -y / +x | O0+(0,57,0) / (0,57,410) / (0,57,455) | hand rev B = rev C: tip local z 624 (wrist-to-tip 214 from W2) |
Groups: M1 (S1'), M2 (+S2: S3 bracket, XM540), M3 (+S3: upper arm, elbow ball), ME = M4 (+E: forearm), MW1, MW2, MW3 = M5 (hand). `Kin.group_transforms(q)` returns 4x4 relative to the HOME-built pose; proxies are built at home in the robot frame.
Tests (all pass): FK vs nested arm_kin-form chain 1.7e-13; arm frame vs `P.shoulder_frames` 2.8e-14; park direction = baseline S1 45/S2 25 (0.05 deg); soft-box direction coverage 0.963; det J = -cos(phi).
## 2. Limits
Soft S1' -95..35, S2 -30..88, S3 -70..90, E 0..145, W1 +-80, W2 -70..80, W3 -20..30; hard = soft +-5 (5 deg rule). Pole (arm along +-x) is at phi = +-90; J1's S2 max of 88 would let the arm run INTO the pole (2 deg). Coverage cost of pulling S2 max in (alpha = 0): 88: 0.963, 80: 0.963, 75: 0.961, 70: 0.958, 65: 0.949, 60: 0.941. **Recommend S2 soft max 75** (arm >= 15 deg from the pole, amplification <= 3.9x, hard stop 80, coverage -0.2 pt). Axis tilt alpha = 10 deg: coverage 0.928 (-3.5 pt), pole-to-set margin 10.2 -> 15.6 deg, depth cost 44.5 sin(10) = 7.7 mm: **not recommended** (S2 <= 75 already gives 15 deg of arm-to-pole margin for 0 mm).
Conditioning (sigma_min of the 3x3 task Jacobian, 20k poses): J1 with S2 <= 75: min 0.185, 9.1 % below 0.3; baseline soft box: min 0.000 (S2 = 0 is inside the baseline box), 41 % below 0.3. Amplification 1/cos(phi) = 1.05 at park (baseline park 2.4x).
**Head/neck (finding, needs a decision):** a straight arm (590 mm, r 32) from J1's arm base violates the 60 mm keep-out inside the soft box at y1 = -70, z1 = -35 (min -12.7 mm at S1' 35 / S2 50; 83 of 648 grid poses < 60). The excluded poses are NOT in the baseline direction set (mask coverage stays 0.963), so a software table (S1', S2) keeps full coverage, but **the baseline principle "passive stops alone keep a stretched arm out of the head" fails**: the best passive-only box at the current pad is S1' <= 5, S2 <= 55 (coverage 0.736). Moving the pad laterally fixes most of it (z1 = -35): y1 = -50: 0.863 (S1' <= 15, S2 <= 80); -30: 0.903; -10: 0.926 (S1' <= 25); **+10: 0.940 (S1' <= 30, S2 <= 80, full-box min clearance 52.9)**; x0 changes by < 3 mm. Lower pads are worse (z1 -70: -30 mm at y1 -70). Pad position is the lever for the local-geometry optimisation; y1 up to +10 needs the skin/blade gap re-checked by the sweep.
## 3. Speed caps (0.5 m/s tip, real levers, 6000 random poses in the soft box)
| joint | lever max (m) | cap deg/s | baseline lever, same hand (m) |
|---|---|---|---|
| S1' | 0.647 | 44.3 | 0.623 (S1) |
| S2 | 0.627 | 45.7 | 0.627 |
| S3 | 0.402 | 71.2 | 0.402 |
| E | 0.394 | 72.7 | 0.394 |
| W1 / W2 / W3 | 0.211 / 0.214 / 0.169 | 90 (actuator limit; 136/134/170 by tip speed) | same |
Combined limiter sum lever_j |qdot_j| <= 0.5 m/s as before. (Project figures 49.5 / 81.5 were for the 575 mm tip.)
## 4. Gravity / torque (project masses, payload 0.5 kg at 575 mm)
S1' unbalanced 6.94 N.m (SF 1.20), S2 6.45 (SF 1.29); S1' now needs a balancer (K 4.0 N.m, residual 3.19, SF 2.6). Caps: AK70 S1'/S2 8.3, S3 4.8 N.m. `gravity_torques(th, phi, payload)` is the model for the gravity-explained EMG subtraction.
## 5. Workspace vs baseline (same arm directions)
95.5 % of the 648 baseline directions are reachable (619). Same-direction extended-tip displacement vs baseline: mean (-88, +47, -87), mean |.| 153 mm, p95 221, max 233 mm (the arm base moves on a circle of 105.75 mm about the S1' axis: x = -45, y -61..36, z -140..18). Random-tip voxel overlap is meaningless (non-uniform joint-space sampling) and is not used.
## 6. Firmware / table changes
See `cad/j1r2_control.md`.
