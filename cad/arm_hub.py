"""Arm hub board H1 (one per arm) in the upper-arm tube + the CAN termination plug at S2, as fit proxies in the M3 / M1 group frames (robot frame at the zero pose).
H1 = 12 V servo-bus distribution: trunk in (12 V, GND, XM data, STS data), SMCJ15A TVS + 2 x 470 uF bulk, two resettable fuses (elbow XM branch, STS chain branch),
elbow XM output (JST EH 3-pin), STS trunk output (3-pin 3.96 mm).  No MCU: data lines are passed straight through (the buses are driven by the Teensy in the main box).
Component sizes: R = retrieved (arm_vendor_data.json), T = typical / unverified (PLACEHOLDER).  Outputs: stl/electronics/arm_hub_R.stl, arm_hub.json.
Run:  python3 arm_hub.py [y_board z_board]"""
import os, sys, json, numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "..", "stl", "electronics")
BOARD = (54.0, 30.0, 1.6)                      # x (across the tube), z (along the arm), thickness t (along y) ; FR4 1.6 mm (C)
PARTS = {   # name: (size x, size z, height above the board (along y), mass g, provenance, note)
    "C1_470uF": (10.0, 10.0, 10.5, 1.4, "R", "Wurth 865080457015, 470 uF 25 V, D10 x H10.5 max"),
    "C2_470uF": (10.0, 10.0, 10.5, 1.4, "R", "same"),
    "D1_TVS": (8.1, 6.6, 2.7, 0.3, "R", "SMCJ15A DO-214AB, lead span 8.13, H 2.62 max (stand-off 15 V > 14.8 V XM max; clamp 24.4 V at 61.5 A)"),
    "F1_PTC": (7.4, 5.4, 2.0, 0.2, "T", "resettable fuse 3 A hold, elbow XM branch (1812-class)"),
    "F2_PTC": (7.4, 5.4, 2.0, 0.2, "T", "resettable fuse 3 A hold, STS chain branch"),
    "J1_trunk_in_12V": (8.0, 9.5, 9.5, 1.0, "T", "2-pin 3.96 mm (JST VH class, >= 7 A): 12 V + GND from the trunk"),
    "J2_data_in": (5.0, 9.5, 8.1, 0.6, "R", "JST EH 3-pin: GND, XM data, STS data (EHR-3 5.0 x 9.5, mated height 8.1)"),
    "J3_elbow_XM": (5.0, 9.5, 8.1, 0.6, "R", "JST EH 3-pin to the elbow XM540 (12 V, GND, data)"),
    "J4_STS_out": (11.9, 9.5, 9.5, 1.4, "T", "3-pin 3.96 mm (JST VH class): 12 V, GND, STS data down the forearm"),
}
HOLES = [(-22.0, -10.0), (22.0, -10.0), (-22.0, 10.0), (22.0, 10.0)]   # M2.5 mounting holes (x across, z from the board centre), 2.7 mm clearance; on 4 printed posts on ua_med (C)
FACE = {"J1_trunk_in_12V": -1, "J2_data_in": -1}          # parts on the -y face (inputs, towards the S3 window); everything else on the +y face
def build(y_b, z_b):
    """meshes in the M3 frame: board plane normal = y, board centred at (x 0, y y_b, z z_b); inputs on the -y face, outputs/protection on the +y face"""
    ms = {}; bx, bz, bt = BOARD
    ms["board"] = trimesh.creation.box((bx, bt, bz)); ms["board"].apply_translation((0, y_b, z_b))
    rows = {-1: [["J1_trunk_in_12V", "J2_data_in"]], +1: [["J3_elbow_XM", "J4_STS_out"], ["C1_470uF", "C2_470uF", "D1_TVS", "F1_PTC", "F2_PTC"]]}
    place = {}
    for face, rws in rows.items():
        zc = z_b - bz / 2 + 1.5
        for row in rws:
            depth = max(PARTS[n][1] for n in row); tot = sum(PARTS[n][0] for n in row) + 2.0 * (len(row) - 1); xs = -bx / 2 + 1.5 if tot > bx - 8 else -tot / 2
            for n in row:
                sx, sz, h, m, _, _ = PARTS[n]; place[n] = (xs + sx / 2, zc + depth / 2); xs += sx + 2.0
            if xs - 2.0 > bx / 2 - 1.0: raise RuntimeError("row does not fit: " + str(row))
            zc += depth + 2.0
        if zc - 2.0 > z_b + bz / 2 - 1.5: raise RuntimeError("rows do not fit in z")
    for n, (px, pz) in place.items():
        sx, sz, h, *_ = PARTS[n]; f = FACE.get(n, +1); b = trimesh.creation.box((sx, h, sz)); b.apply_translation((px, y_b + f * (bt / 2 + h / 2), pz)); ms[n] = b
    return ms, place

def fit_check(y_b, z_b):
    import arm_wiring_lib as L
    ms, place = build(y_b, z_b); sdf, lo, p = L.sdf_of("M3"); worst = 1e9; worst_name = ""
    for n, m in ms.items():
        P = m.sample(4000) if n != "board" else m.sample(8000); P = np.vstack([P, m.vertices]); v = L.sample_sdf("M3", P).min()
        if v < worst: worst, worst_name = float(v), n
    return worst, worst_name, ms, place

if __name__ == "__main__":
    best = None
    ys = [float(sys.argv[1])] if len(sys.argv) > 2 else [-4, -3, -2, -1, 0, 1, 2, 3, 4]; zs = [float(sys.argv[2])] if len(sys.argv) > 2 else [135, 140, 145, 150, 155, 160, 165, 170]
    for yb in ys:
        for zb in zs:
            try: w, nm, ms, place = fit_check(yb, zb)
            except RuntimeError as e: print("layout", e); continue
            if best is None or w > best[0]: best = (w, nm, yb, zb, ms, place)
    w, nm, yb, zb, ms, place = best; print("best y %.1f z %.1f  min clearance to the tube/halves/tabs %.2f mm (%s)" % (yb, zb, w, nm))
    allm = trimesh.util.concatenate(list(ms.values())); allm.export(os.path.join(OUT, "arm_hub_R.stl"))
    mass = 1.85e-3 * BOARD[0] * BOARD[1] * BOARD[2] / 1.0 + sum(v[3] for v in PARTS.values())
    json.dump(dict(y_board=yb, z_board=zb, board=BOARD, min_clearance_mm=round(w, 2), limiting=nm, mass_g=round(mass, 1), mass_note="board 1.85 g/cm3 + part masses (T)",
                   placement_xz={k: [round(a, 1) for a in v] for k, v in place.items()}, face={k: FACE.get(k, +1) for k in PARTS}, holes=HOLES, parts={k: dict(size=v[:3], mass=v[3], prov=v[4], note=v[5]) for k, v in PARTS.items()}),
              open(os.path.join(OUT, "arm_hub.json"), "w"), indent=1)
