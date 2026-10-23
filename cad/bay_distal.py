"""Bay existence study for E, W1, W2, W3 (arm_bay.py only had S1..S3). Same method: annulus about the joint axis free of every solid at every azimuth and every joint angle."""
import sys, json, os, numpy as np
import arm_bay as B, arm_wire_spec as S
B.AXIS.update({
 "E":  dict(p=(0,0,230), d=(0,1,0), e1=(1,0,0), sign=-1, hmin=-45.0, hmax=45.0, Rmin=6.0, Rmax=60.0),
 "W1": dict(p=(0,57,345), d=(0,0,1), e1=(1,0,0), sign=1, hmin=-40.0, hmax=40.0, Rmin=6.0, Rmax=50.0),
 "W2": dict(p=(0,57,410), d=(0,1,0), e1=(1,0,0), sign=-1, hmin=-35.0, hmax=35.0, Rmin=5.0, Rmax=50.0),
 "W3": dict(p=(0,57,455), d=(1,0,0), e1=(0,1,0), sign=1, hmin=-35.0, hmax=35.0, Rmin=5.0, Rmax=45.0)})
for j in sys.argv[1:]:
    Rs, hs, free, worst = B.bay_map(j, dphi=6.0, dR=2.0, dh=2.0, rstep=10.0)
    area, rect = B.largest_rect(free); print(j, "free cells", int(free.sum()), "of", free.size, flush=True)
    if rect is None: print(j, "NO free bay (max clearance anywhere %.1f mm)" % float(worst.max())); continue
    i0,i1,j0,j1 = rect; print(j, "largest free rect R %.0f..%.0f  h %.0f..%.0f" % (Rs[i0],Rs[i1],hs[j0],hs[j1]), flush=True)
