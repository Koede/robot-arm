"""PROPOSED firmware tables from the MJ11 whole-arm sweep, in the exact output format of firmware/gen_tables.py (Project doc third_shoulder_project_knowledge/firmware/gen_tables.py):
elbow_limits.h (EL_F / EL_R) and keepout_table.h (KO_MM), on the (S1', S2) axes of the J1 rev 2 / MJ11 shoulder.  NOT dropped into firmware/: control.h, protocols.h and the S1 axis definition still
follow the baseline S1 (S1' carries gravity now), and gen_tables.py itself cannot run on this sweep (see specs/SPEC_j1r2_mj11_whole_arm.md section 6).
usage: python3 mj11_gen_tables.py  (reads stl/j1r2_sweep/mj11_elbow_limits*.json, mj11_sweep_full_axes.json, mj11_keepout_map.npz; writes stl/j1r2_sweep/firmware_proposed/)"""
import os, json, hashlib, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); R = os.path.abspath(os.path.join(HERE, "..", "..", "stl", "j1r2_sweep")); OUT = os.path.join(R, "firmware_proposed"); os.makedirs(OUT, exist_ok=True)
ax = json.load(open(os.path.join(R, "mj11_sweep_full_axes.json"))); s1, s2 = ax["S1"], ax["S2"]
rf = open(os.path.join(R, "mj11_elbow_limits.json"), "rb").read(); el = json.loads(rf); elr = json.load(open(os.path.join(R, "mj11_elbow_limits_restricted.json")))
key = lambda a, b: f"{a:.0f},{b:.0f}"
with open(os.path.join(OUT, "elbow_limits.h"), "w") as f:
    f.write("// PROPOSED (MJ11 whole-arm sweep 2026-11-03), same format as firmware/gen_tables.py -- do not edit; S1 axis here is S1' (J1 rev 2 / MJ11), NOT the baseline S1\n"
            "// Largest elbow angle with >= 3 mm to parts / harness, >= 2 mm to skin, >= 60 mm to head/neck (project gates, SPEC_collision_method), first-contact scan up from E = 0, per (S1', S2) grid point.\n"
            "// EL_F: any S3 / wrist.  EL_R: S3 +-30, W2 +-30, W3 +-15.  -1 = not allowed in that tier (use the NEUTRAL tier, see control.h).  Grid = sweep grid incl. the park values (S1' -18.2, S2 17.4).\n"
            "// Look-up = MIN of the 4 surrounding grid points.  source sha256 %s\n#pragma once\n" % hashlib.sha256(rf).hexdigest()[:16])
    f.write(f"const int EL_N1 = {len(s1)}, EL_N2 = {len(s2)};\nconst float EL_S1[EL_N1] = {{{','.join(f'{v:.2f}f' for v in s1)}}};\nconst float EL_S2[EL_N2] = {{{','.join(f'{v:.2f}f' for v in s2)}}};\n")
    for nm, tab in (("EL_F", el), ("EL_R", elr)):
        f.write(f"const float {nm}[EL_N1][EL_N2] = {{\n" + ",\n".join("{" + ",".join(f"{tab[key(a, b)]:.1f}f" for b in s2) + "}" for a in s1) + "};\n")
k = np.load(os.path.join(R, "mj11_keepout_map.npz")); S1, S2, C = k["S1"], k["S2"], k["C"]
i1 = [i for i, v in enumerate(S1) if abs((v + 95.0) / 2.5 - round((v + 95.0) / 2.5)) < 1e-6]; i2 = [i for i, v in enumerate(S2) if abs((v + 30.0) / 2.5 - round((v + 30.0) / 2.5)) < 1e-6 and v <= 85.0]     # uniform 2.5 deg lattice (drops the inserted park values and 88)
Cu = C[np.ix_(i1, i2)]; S1u, S2u = S1[i1], S2[i2]
with open(os.path.join(OUT, "keepout_table.h"), "w") as f:
    f.write("// PROPOSED (MJ11 whole-arm sweep 2026-11-03), same format as firmware/gen_tables.py -- do not edit; axes S1', S2 of J1 rev 2 / MJ11\n// KO_MM = straight-arm (E 0, S3 0, wrists 0, HAND rev B/C envelope) clearance to the wearer mesh, min(skin, head/neck), mm; 0 = touching.  Gates: skin 2 mm, head/neck 60 mm.\n#pragma once\n#include <stdint.h>\n")
    f.write(f"const float KO_S1_0 = {S1u[0]:.1f}f, KO_S2_0 = {S2u[0]:.1f}f, KO_STEP = {S1u[1]-S1u[0]:.2f}f;\nconst int KO_N1 = {len(S1u)}, KO_N2 = {len(S2u)};\n")
    f.write("const int16_t KO_MM[KO_N1][KO_N2] = {\n" + ",\n".join("{" + ",".join(str(int(round(max(-999, min(999, v))))) for v in row) + "}" for row in Cu) + "};\n")
print("elbow_limits.h", len(s1), "x", len(s2), "; keepout_table.h", Cu.shape)
