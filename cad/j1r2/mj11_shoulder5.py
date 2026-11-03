"""MJ11 shoulder-level sweep at 5 deg with S3 {-70,0,90} (M1/M2/M3 vs fixed parts + wearer): cross-check of the 10 deg whole-arm grid.  -> stl/j1r2_sweep/mj11_shoulder5.json"""
import os, sys, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mj11_sweep as W, mj_concepts3 as C3
parts = C3.mj11_geo(None); r, out = W.run(parts, {"S1p": 5.0, "S2": 5.0, "S3": [-70.0, 0.0, 90.0]}, label="mj11_sh5")
import j1r2_sweep as S
json.dump({k: v for k, v in r.items()}, open(os.path.join(S.OUT, "mj11_shoulder5.json"), "w"), indent=1, default=float); print(json.dumps(r, default=float)); print(S.table(out))
