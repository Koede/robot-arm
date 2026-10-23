"""Joint limits for the Third Shoulder (robot angles, deg). Single source for soft limits; hard (passive) stops sit
STOP_MARGIN beyond (standing decision), except S1 max which keeps its verified 10 deg margin.

Rev 2026-10-12c (I1):
  * S1 soft min -15 -> +10 and S2 soft min -55 -> -30. Reason: with the real MakeHuman head/neck mesh the straight arm
    (P -> 500 mm, r 30 mm; since 2026-10-13 P -> 590 mm, r 32 to cover the 575 mm tip) came within -10 mm of the head/neck at S1 -15..5 / S2 40..88 and S1 105..140 / S2 -55..-35
    (stl/shoulder/keepout_map.npz, built by cad/keepout_map.py). Inside the new box the straight-arm clearance is >= 64.9 mm (590 mm capsule),
    so the 60 mm software head keep-out can never be the only thing between the arm and the head: the passive stops
    alone keep a fully stretched arm out of it. S2 -30 was also the original target in SPEC_ball_stops.
  * Distal limits unchanged here; E target 145 (I6), W limits from human ROM.
Human references (AAOS/typical adult) kept for the distal joints."""
STOP_MARGIN = 5
SOFT = {          # joint: (min, max)
    "S1": (10, 140),
    "S2": (-30, 88),
    "S3": (-70, 90),
    "E":  (0, 145),
    "W1": (-80, 80),
    "W2": (-70, 80),
    "W3": (-20, 30),
    "G":  (0, 100),      # gripper opening as % of stroke
}
HARD = {k: (a - STOP_MARGIN, b + STOP_MARGIN) for k, (a, b) in SOFT.items()}
HARD["S1"] = (5, 150)    # S1 max keeps the verified +150 post
REF = {
    "S1": "arm-plane swing (horizontal abd/add-like); medial limit set by head keep-out",
    "S2": "elevation from the up-rest; back lean limited to 30 deg by head keep-out (double cover S1,S2 == S1-180,-S2)",
    "S3": "humeral rotation: int 70 / ext 90",
    "E": "elbow flexion 145, hyperextension 0",
    "W1": "pronation / supination 80 / 80",
    "W2": "wrist extension 70 / flexion 80",
    "W3": "radial 20 / ulnar 30",
    "G": "gripper stroke",
}
PARK = {"S1": 45.0, "S2": 25.0, "S3": 0.0, "E": 45.0, "W1": 0.0, "W2": 0.0, "W3": 0.0, "G": 50.0}   # off the S2 = 0 singularity
if __name__ == "__main__":
    for k in SOFT:
        print(f"{k:3s} soft {SOFT[k][0]:5d}..{SOFT[k][1]:4d}  hard {HARD[k][0]:5d}..{HARD[k][1]:4d}  park {PARK[k]:6.1f}  [{REF[k]}]")
