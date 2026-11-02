import sys, os, json, itertools; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import j1r2_opt as O
res = []
for y1, z1 in itertools.product((-70.0, -55.0, -40.0, -25.0, -10.0, 10.0), (-70.0, -50.0, -30.0, -10.0)):
    try:
        r, _ = O.evaluate(y1, z1, 22.0, label="scan"); b = r["box"]; res.append(dict(y1=y1, z1=z1, cov=(b or {}).get("cov"), reach_cov=r["reach_cov"], ranges=(b or {}).get("ranges"), park_ok=r["park_ok"], t=r["t"]))
    except Exception as e: res.append(dict(y1=y1, z1=z1, err=str(e)[:120]))
    print(res[-1], flush=True)
json.dump(res, open("/home/claude/ts/stl/j1r2_sweep/scan_pad_xy.json", "w"), indent=1)
