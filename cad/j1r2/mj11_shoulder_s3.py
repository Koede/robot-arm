"""MJ11 shoulder-level sweep at 5 deg with S3 in {-70,0,90} (joints_upto S3): passive-stop box + shoulder-level first contacts incl. S3.  usage: J1R2_ROOT=<work root> python3 mj11_shoulder_s3.py"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mj11_sweep as W, mj_concepts3 as C3, mj_frame as M
parts = C3.mj11_geo(None); r, out = W.run(parts, {"S1p": 5.0, "S2": 5.0, "S3": [-70.0, 0.0, 90.0]}, label="mj11_shoulder_s3")
rows = {k: {kk: v.get(kk) for kk in ("MM", "park_d", "status", "n_frontier_cells", "n_deep_cells", "d_frontier_min", "first_contact", "first_contact_example", "n_first_contact_points", "worst_contact_pose") if kk in v} for k, v in out["pairs"].items() if v.get("n_frontier_cells") or v["status"].startswith("FAIL") or v["status"].startswith("BELOW")}
json.dump(dict(result=r, frontier_pairs=rows), open(os.path.join(os.path.dirname(W.S.OUT + "/"), "mj11_shoulder_s3_summary.json"), "w"), indent=1, default=float)
print(json.dumps(r, indent=1, default=float))
