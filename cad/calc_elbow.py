"""Phase-3-style hand calcs for the upper arm / elbow stage (SI units in comments; masses in kg, lengths in m)."""
import math
g = 9.81
# ---- masses beyond the elbow axis (forearm + wrist + hand), ASSUMED until those parts are designed
m_servo = 0.055            # STS3215 (x4: W1, W2, W3, G)  [vendor ~55 g, not re-verified here]
m_fore_struct = 0.150      # printed forearm tube + wrist brackets + gripper fingers
m_wiring = 0.030
m_fore = 4 * m_servo + m_fore_struct + m_wiring
l_fore_cg = 0.140          # CG of forearm+hand from the elbow axis (m)
payload, l_payload = 0.5, 0.270     # kg at the gripper tip: forearm 180 + wrist-to-tip 90 = 270 mm from the elbow axis
M_elbow_static = m_fore * g * l_fore_cg + payload * g * l_payload       # forearm horizontal (worst)
SF = 2.0
print("forearm+hand mass %.3f kg ; elbow static torque (horizontal, 0.5 kg tip) = %.2f N.m ; with SF %.1f -> %.2f N.m" % (m_fore, M_elbow_static, SF, SF * M_elbow_static))
tau_stall = 10.6
print("XM540-W270 stall 10.6 N.m @12 V -> design point is %.0f %% of stall (SF 2). Continuous rating not re-verified; thermal check belongs to Phase 6" % (100 * SF * M_elbow_static / tau_stall))
# ---- dynamic: accelerate forearm+payload 0.5 m/s tip cap, reach 0.27 m, 0.2 s ramp -> alpha
I = m_fore * 0.14 ** 2 + 0.5 * 0.27 ** 2 + 0.4 * m_fore * 0.1 ** 2 * 0   # point-mass approximation
alpha = (0.5 / 0.27) / 0.2
print("inertia about elbow %.4f kg m2 ; alpha for 0.2 s ramp to the 0.5 m/s tip cap = %.1f rad/s2 -> accel torque %.2f N.m" % (I, alpha, I * alpha))
print("tip speed cap 0.5 m/s at 0.27 m -> %.1f rad/s = %.0f rpm  (XM540-W270 no-load 30 rpm = %.1f rad/s)" % (0.5 / 0.27, 0.5 / 0.27 * 60 / 2 / math.pi, 30 * 2 * math.pi / 60))
# ---- S2 recheck with the real arm mass beyond S2 (CG distance from S2 axis, arm horizontal)
parts = {"S3 XM540": (0.165, 0.09), "S3 bracket+adapter": (0.12, 0.05), "upper-arm shells+plate": (0.16, 0.17), "elbow XM540": (0.165, 0.23),
         "elbow fork+hardware": (0.12, 0.23), "forearm+hand": (m_fore, 0.23 + 0.14)}
M2 = sum(m * l * g for m, l in parts.values()) + payload * g * (0.23 + 0.27)
mass_beyond = sum(m for m, l in parts.values()) + payload
print("beyond S2: %.2f kg (incl. 0.5 kg payload) ; static torque at S2 horizontal = %.2f N.m ; x2 = %.2f vs AK70-10 rated 8.3 / peak 24.8 N.m" % (mass_beyond, M2, 2 * M2))
# ---- shell bending check (hollow elliptical tube, a=32 b=20, wall 2.4 mm), load = elbow torque reacted as a couple on the housing + payload bending
a, b, t = 32.0, 20.0, 2.4
I_x = math.pi / 4 * (a * b ** 3 - (a - t) * (b - t) ** 3)       # bending in the flexion (x-z) plane about the y axis uses the x-radius: I_y = pi/4 (b a^3 - (b-t)(a-t)^3)
I_y = math.pi / 4 * (b * a ** 3 - (b - t) * (a - t) ** 3)
Mmax = SF * M_elbow_static * 1000 + (m_fore * 0.14 + payload * 0.27) * g * 0 + 0
Mb = (m_fore + payload) * g * 0.12 * 1000            # extra bending at the S3 plate: forearm+payload weight acting 0.12 m above the plate (N.mm) - the elbow torque is reacted inside the housing
Mtot = Mmax + Mb
print("shell section: I_y = %.0f mm^4 (x-radius %.0f), c = %.0f mm -> sigma = %.1f MPa at M = %.0f N.mm (SF 2 already in the torque); PETG layer-direction allowable ~ 25 MPa (assumed)" % (I_y, a, a, Mtot * a / I_y, Mtot))
