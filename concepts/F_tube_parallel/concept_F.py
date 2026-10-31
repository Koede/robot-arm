import sys, json, math, numpy as np; sys.path.insert(0, '/home/claude/robot-arm/cad')
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
PL1, MATE, R_C = 495.8, 515.6, 22.0           # plane A top, plane B (mating), leg circle radius
GAP = MATE - PL1; ANG = np.radians([90, 210, 330])
A = np.array([[R_C*np.cos(a), R_C*np.sin(a), 0.0] for a in ANG])           # base joints, rel to plane A centre (z=0 at plane A)
def Rx(t): c,s=math.cos(t),math.sin(t); return np.array([[1,0,0],[0,c,-s],[0,s,c]])
def Ry(t): c,s=math.cos(t),math.sin(t); return np.array([[c,0,s],[0,1,0],[-s,0,c]])
def Rz(t): c,s=math.cos(t),math.sin(t); return np.array([[c,-s,0],[s,c,0],[0,0,1]])
def legs(R, h=GAP):                              # spherical centre on the axis at plane B height h (central ball joint fixes translation)
    c = np.array([0, 0, h]); B = c + (R @ A.T).T; v = B - A; L = np.linalg.norm(v, axis=1)
    inc = np.degrees(np.arccos(np.clip(v[:,2]/L, -1, 1))); return L, inc, v/L[:,None]
def jac_cond(R, h=GAP):                          # leg-length rate vs platform angular rate about the centre (3x3)
    L, inc, u = legs(R, h); r = (R @ A.T).T; J = np.array([np.cross(r[i], u[i]) for i in range(3)]); sv = np.linalg.svd(J, compute_uv=False); return sv[0]/max(sv[-1],1e-9), sv[-1]
LMIN, LMAX, JANG = 8.0, 34.0, 25.0                # leg length window (crank stroke 26), printed ball joint cone half angle
def ok(R, h=GAP):
    L, inc, _ = legs(R, h); return bool(L.min() >= LMIN and L.max() <= LMAX and inc.max() <= JANG)
def rng(fn, vals, h=GAP):
    f = [v for v in vals if ok(fn(math.radians(v)), h)]; return (min(f), max(f)) if f else (0, 0)
vals = np.arange(-60, 60.5, 0.5); tw = np.arange(-180, 180.5, 1.0)
res = {}
for h, nm in ((GAP, "gap_19.8"), (GAP + 10, "gap_29.8_lift10"), (GAP + 16, "gap_35.8_lift16")):
    res[nm] = dict(tilt_x_deg=rng(Rx, vals, h), tilt_y_deg=rng(Ry, vals, h), twist_deg=rng(Rz, tw, h),
                   leg_len_at_0=[round(float(x),1) for x in legs(np.eye(3), h)[0]],
                   cond_at_0=round(jac_cond(np.eye(3), h)[0], 2))
# singularities: scan combined tilt_x 0..30 x twist 0..24 at nominal gap
sing = [(tx, t, round(jac_cond(Rx(math.radians(tx)) @ Rz(math.radians(t)))[0], 1)) for tx in (0, 15, 30) for t in (0, 12, 24)]
res["cond_grid_tx_twist"] = sing
# loads / stiffness: payload 0.5 kg at 200 mm, 2 g -> tilt moment; leg force ~ M/(1.5 r)
M = 0.5*9.81*0.2*2.0; Fleg = M/(1.5*R_C*1e-3); crank = 14.0
E_tube, A_tube = 6000.0, math.pi/4*(6**2-4**2); k_tube = E_tube*A_tube/GAP
back = math.radians(0.3)*crank; tilt_def = back/R_C; tip = tilt_def*200.0
res["load"] = dict(tilt_moment_Nm=round(M,2), leg_force_N=round(Fleg,1), crank_torque_Nm_14mm=round(Fleg*crank*1e-3,2), micro_servo_Nm=0.216, sts3215_Nm=2.94,
                   tube_axial_k_N_per_mm=round(k_tube), tube_axial_defl_mm=round(Fleg/k_tube,3), servo_backlash_tip_mm_0p3deg=round(tip,2))
res["actuators"] = dict(count=3, type="STS3215 (micro servos cannot: 0.216 vs 0.83 N.m)", mass_g=165, vs_now_g=110, delta_g=55,
                        note="3-SPS needs the central ball joint (spindle) or it has 3 uncontrolled DOF; servos cannot sit under plane A (W3 servo is there) so they sit outboard at r>=40 on cranks + push-pull cables or legs get rod ends")
res["verdict"] = "tilt +-30 about x needs leg inclination ~; twist limited by ball cone: see ranges"
json.dump(res, open("concept.json", "w"), indent=1); print(json.dumps(res, indent=1))
# section sketch: y-z cut with platform tilted 30 deg about x
fig, ax = plt.subplots(1, 2, figsize=(12, 5), dpi=110)
for k, (tx, t, ttl) in enumerate(((0, 0, "neutral"), (30, 0, "tilt x +30 deg"))):
    R = Rx(math.radians(tx)) @ Rz(math.radians(t)); L, inc, _ = legs(R); c = np.array([0, GAP]); a = ax[k]
    a.plot([-35, 35], [0, 0], "k", lw=4); pl = (R @ np.array([[-35,0,0],[35,0,0],[0,-35,0],[0,35,0]]).T).T; pb = pl[2:]
    a.plot(pb[:,1], pb[:,2]+GAP, "tab:green", lw=4)
    for i in range(3):
        b = np.array([0,GAP]) + (R @ A[i])[1:]*np.array([1,1]); a.plot([A[i][1], b[0]], [0, b[1]], "tab:orange", lw=3)
        a.plot(*[[A[i][1]], [0]], "ko"); a.plot([b[0]],[b[1]], "o", color="tab:green")
    a.plot([0],[GAP],"r*", ms=12); a.set_aspect("equal"); a.set_title(f"{ttl}: legs {np.round(L,1)} mm, max incl {inc.max():.0f} deg"); a.set_xlim(-40,40); a.set_ylim(-8,40); a.grid(alpha=.3)
fig.suptitle("Concept F: 3 tube legs (OD6/ID4) + central ball, y-z section sketch (not to scale of tube OD)"); fig.tight_layout(); fig.savefig("F_section.png")
# cadquery neutral sketch
import cadquery as cq
w = None
for a in ANG:
    x, y = R_C*math.cos(a), R_C*math.sin(a); t = cq.Workplane("XY").circle(3).circle(2).extrude(GAP).translate((x, y, PL1)); w = t if w is None else w.union(t)
w = w.union(cq.Workplane("XY").circle(35).extrude(2).translate((0,0,PL1-2))).union(cq.Workplane("XY").circle(35).extrude(2).translate((0,0,MATE)))
cq.exporters.export(w, "F_neutral.stl"); print("done")
