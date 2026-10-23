"""S2 reconciliation step 2 (in the pipeline since 2026-10-15): S2 gravity holding torque over the full soft-limit grid, from the mesh mass model (s2_massmodel.json).
Physics: with S1 = S2 = 0 the arm frame equals the M1 frame.  Let c = sum m_i p_i (kg m) of everything beyond S2 (groups M2..MW3 + payload at the
claw tip (0, D, Z_TIP) in the MW3 frame), evaluated in the S2-output frame for a given (S3, E, W1, W2, W3).  S2 rotates c by Ry(-S2):
    x' = cx cos S2 - cz sin S2.  Gravity moment about +y: tau_y = g x'.  Forward-falling moment (rev d sign, motor must supply it):
    Tg(S2, pose) = g (cz sin S2 - cx cos S2)          [N m]
S1 does not enter (vertical axis).  Output: stl/balancer/s2_grid.npz (+ s2_envelope.json) and printed summary."""
import os, sys, json, itertools, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import arm_kin as ak, rom_limits as rl
g = 9.81
OUTD = os.path.join(HERE, "..", "stl", "balancer"); os.makedirs(OUTD, exist_ok=True)
MM = json.load(open(os.path.join(OUTD, "s2_massmodel.json")))
CH = ["M2", "M3", "ME", "MW1", "MW2", "MW3"]
D, ZT = ak.LAY["D_OFF"], ak.LAY["Z_TIP"]
EXTRA_WIRING = float(os.environ.get("EXTRA_UPPER_G", 40.0))   # rev 2026-10-15: default 40 g, the same allowance harness_loads.py carries       # optional unmodelled M2/M3 hardware (g) at z 150 in M3 frame
def c_vec(q, pay_kg):
    T = ak.frames(q); c = np.zeros(3)
    for gname in CH:
        m = MM[gname]["mass_g"] * 1e-3; M = np.array(MM[gname]["first_moment_gmm"]) * 1e-6     # kg m
        R, t = T[gname][:3, :3], T[gname][:3, 3] * 1e-3
        c += R @ M + m * t
    if EXTRA_WIRING: c += EXTRA_WIRING * 1e-3 * (T["M3"][:3, :3] @ np.array([0, 0, 0.150]) + T["M3"][:3, 3] * 1e-3)
    if pay_kg: c += pay_kg * (T["MW3"] @ np.array([0, D, ZT, 1.0]))[:3] * 1e-3
    return c
S3s = np.arange(-70, 90.1, 20.0); Es = np.arange(0, 145.1, 5.0); W1s = (-80.0, 0.0, 80.0); W2s = np.arange(-70, 80.1, 15.0); W3s = (-20.0, 0.0, 30.0)
S2s = np.arange(-30, 88.01, 1.0)
rows = []
for s3, e, w1, w2, w3 in itertools.product(S3s, Es, W1s, W2s, W3s):
    q = dict(S1=0.0, S2=0.0, S3=s3, E=e, W1=w1, W2=w2, W3=w3, G=50.0)
    for pay in (0.0, 0.5):
        cx, cy, cz = c_vec(q, pay); rows.append((s3, e, w1, w2, w3, pay, cx, cz))
A = np.array(rows)
th = np.radians(S2s)
TG = g * (A[:, 7][:, None] * np.sin(th)[None] - A[:, 6][:, None] * np.cos(th)[None])      # (poses, S2)
np.savez(os.path.join(OUTD, "s2_grid%s.npz" % os.environ.get("TAG", "")), A=A, S2=S2s, TG=TG, EXTRA_UPPER_G=EXTRA_WIRING)
i, j = np.unravel_index(np.argmax(np.abs(TG)), TG.shape)
straight = [r for r in range(len(A)) if A[r, 0] == -70 + 0 and False]
q0 = dict(S1=0, S2=0, S3=0, E=0, W1=0, W2=0, W3=0, G=50); cs = c_vec(q0, 0.5); c0 = c_vec(q0, 0.0)
print("straight arm, S2 88:  own %.3f  + payload %.3f = %.3f N m" % (g * c0[2] * np.sin(np.radians(88)), g * (cs[2] - c0[2]) * np.sin(np.radians(88)), g * cs[2] * np.sin(np.radians(88))))
print("straight arm, S2 90 equivalent: %.3f N m; arm first moment %.4f kg m" % (g * cs[2], c0[2]))
print("grid poses %d x S2 %d; max |Tg| %.3f N m at S2 %g, S3 %g E %g W1 %g W2 %g W3 %g pay %g" % (len(A), len(S2s), abs(TG).max(), S2s[j], *A[i, :6]))
print("min Tg %.3f N m (negative = falls backward)" % TG.min())
for pay in (0.0, 0.5):
    k = A[:, 5] == pay; print("pay %.1f: max %.3f  min %.3f" % (pay, TG[k].max(), TG[k].min()))
# envelope per S2 and ideal minimax passive S2-only balancer: f*(S2) = (max+min)/2, residual = (max-min)/2
hi, lo = TG.max(0), TG.min(0); half = (hi - lo) / 2
print("ideal arbitrary f(S2) minimax residual: %.3f N m at S2 %g -> SF %.2f" % (half.max(), S2s[half.argmax()], 8.3 / half.max()))
json.dump(dict(S2=S2s.tolist(), hi=hi.round(3).tolist(), lo=lo.round(3).tolist()), open(os.path.join(OUTD, "s2_envelope%s.json" % os.environ.get("TAG", "")), "w"))
