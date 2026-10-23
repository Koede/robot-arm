"""Electronics enclosures and sensor pods (I3/I4), right side + shared spine items. Robot frame, mm. CadQuery boxes with lids.
  afe_pod_R    : 3 x ADS1299 (daisy) + isolated 5 V DC-DC (2 x MOPP) + ISO7762 on the isolated board stack; on the wing rail outer face
                 (paraspinal, still skin), 2 x M4 into heat-set inserts (5.6 OD x 4) in the 7 mm rail. Inner 66 x 46 x 24.
  main_box     : Teensy 4.1 + 2 x SN65HVD230 + XM/STS half-duplex buffers + TCA9548A + 24->12 V and 24->5 V bucks + E-stop relay/contactor
                 driver; on the spine bridge outer face (shared, centred on the spine). Inner 120 x 74 x 32.
  battery_box  : 6S2P 18650 pack (3 x 4 cell grid, 54 x 72 x 65 cells) + BMS; on a cradle between the L and R rails below the AFE pods
                 (rev 2026-10-15: z -431..-365, was -378..-312), centred on the spine. Inner 62 x 80 x 72.
  imu_pod      : BNO085 breakout (Adafruit 4754, ~25.6 x 17.8 mm, verify) in a 32 x 25 x 10 pod with a 25 mm webbing slot (arm, forearm, hand
                 bands) - the scapula pod is the same body with a flat adhesive base (medical double-sided tape on the scapular spine).
  lra_cup      : 10 mm coin LRA in a 14 x 5 mm cup sewn into the sleeve cuffs.
All outputs in stl/pods/; placements (for renders and the harness mass model) in pods.json."""
import os, json, numpy as np, trimesh, cadquery as cq, sys
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
OUT = os.path.join(HERE, "..", "stl", "pods"); os.makedirs(OUT, exist_ok=True)
S = os.path.join(HERE, "..", "stl", "shoulder") + "/"
fr = json.load(open(S + "scapular/scapular_frame.json")); SP = fr["spine_y"]
# rev f2 (2026-10-17, electronics fit): inner dims come from stl/pods/pod_dims.json (written by electronics_fit.py); defaults = rev f values
_pd = os.path.join(OUT, "pod_dims.json")
DIMS = json.load(open(_pd)) if os.path.exists(_pd) else {}
MB_IN = tuple(DIMS.get("main_box", (74.0, 120.0, 32.0))); BAT_IN = tuple(DIMS.get("battery_box", (80.0, 62.0, 72.0)))
AFE_IN = tuple(DIMS.get("afe_pod_R", (46.0, 66.0, 24.0))); IMU_IN = tuple(DIMS.get("imu_pod", (26.6, 18.8, 6.0)))
TURRET = DIMS.get("estop_turret_mm", 0.0); ESTOP_XY = DIMS.get("estop_xy_local"); XT60_XY = DIMS.get("xt60_xy_local")
def enclosure(ix, iy, iz, wall=2.0, r=3.0, lid=2.0):
    body = cq.Workplane("XY").box(ix + 2 * wall, iy + 2 * wall, iz + wall, centered=(True, True, False)).edges("|Z").fillet(r)
    body = body.cut(cq.Workplane("XY").box(ix, iy, iz + 1, centered=(True, True, False)).translate((0, 0, wall)))
    cover = cq.Workplane("XY").box(ix + 2 * wall, iy + 2 * wall, lid, centered=(True, True, False)).edges("|Z").fillet(r).translate((0, 0, iz + wall))
    return body, cover
def export(w, name): cq.exporters.export(w, os.path.join(OUT, name + ".stl"), tolerance=0.05)
place = {}
# ---- AFE pod on the rail outer face (box base faces -x onto the rail, so local +z maps to robot +x)
wing = trimesh.load(S + "scapular/wing_R.stl")
R_POD = np.eye(4); R_POD[:3, :3] = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])     # local x->y, y->z, z->x (same as render_full.place_pod)
def rail_outer_x(y, z):
    loc, _, _ = wing.ray.intersects_location([[200.0, y, z]], [[-1.0, 0, 0]])
    return float(loc[:, 0].max()) if len(loc) else None
def rail_max_x(y0, y1, z0, z1, n=48):
    """highest point of the rail outer face under a footprint (robot y0..y1, z0..z1); rev 2026-10-15: the rail face is ridged
    (edges ~9 mm proud of the relief channel) and leans back ~13 deg along z, so a single ray under-seats a flat base."""
    Y, Z = np.meshgrid(np.linspace(y0, y1, n), np.linspace(z0, z1, n)); o = np.c_[np.full(Y.size, 200.0), Y.ravel(), Z.ravel()]
    loc, _, _ = wing.ray.intersects_location(o, np.tile([-1.0, 0.0, 0.0], (len(o), 1)), multiple_hits=False)
    return float(loc[:, 0].max())
def rail_min_x(y0, y1, z0, z1, n=12):
    Y, Z = np.meshgrid(np.linspace(y0, y1, n), np.linspace(z0, z1, n)); o = np.c_[np.full(Y.size, 200.0), Y.ravel(), Z.ravel()]
    loc, _, _ = wing.ray.intersects_location(o, np.tile([-1.0, 0.0, 0.0], (len(o), 1)), multiple_hits=False)
    return float(loc[:, 0].min())
BASE_GAP = 3.0                                    # rev 2026-10-15: flat floors stand >= 3 mm off the rail; only bosses / lead shroud touch it
def seat(body_robot, feats):
    """feats: list of (centre_yz, radius or (wy, hz), hole) in robot frame. Each feature is a prism along x from the floor
    (x_floor) down past the rail, then trimmed by the wing so it lands conformally on the rail face (mating, zero gap)."""
    x_floor = body_robot.bounds[0][0]; add, cut = [], []
    for (yc, zc), size, hole in feats:
        hy, hz = (size, size) if np.isscalar(size) else (size[0] / 2, size[1] / 2)
        x_lo = rail_min_x(yc - hy, yc + hy, zc - hz, zc + hz) - 1.0        # stop 1 mm inside the 7 mm rail: never reaches past it to the skin
        L = x_floor + 0.6 - x_lo                                           # 0.6 mm overlap into the floor for a clean union
        if np.isscalar(size): pr = trimesh.creation.cylinder(radius=size, height=L, sections=40); pr.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
        else: pr = trimesh.creation.box([L, size[0], size[1]])
        pr.apply_translation([x_lo + L / 2, yc, zc])
        add.append(trimesh.boolean.difference([pr, wing], engine="manifold"))
        if hole is not None:
            hp = trimesh.creation.cylinder(radius=hole, height=200, sections=32) if np.isscalar(hole) else trimesh.creation.box([200, hole[0], hole[1]])
            if np.isscalar(hole): hp.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
            hp.apply_translation([x_floor, yc, zc]); cut.append(hp)
    out = trimesh.boolean.union([body_robot] + add, engine="manifold")
    return trimesh.boolean.difference([out] + cut, engine="manifold") if cut else out
def boss_len(yc, zc, x_floor): return round(x_floor - rail_outer_x(yc, zc), 1)
# rev 2026-10-16 (gI fastener fix): every pod screw is sized from the stack it actually passes through, and its tip is checked against the
# part it threads into (>= 2 mm inside the rail/cradle back face) and the skin (> 2 mm, MakeHuman mesh, ray along x through the screw axis).
BODY = trimesh.load(S + "proxies/wearer_makehuman.stl")
def xhits(m, y, z):
    loc, _, _ = m.ray.intersects_location([[300.0, y, z]], [[-1.0, 0, 0]]); return sorted(loc[:, 0].tolist(), reverse=True)
FAST = {}
M4_LENGTHS = (6, 8, 10, 12, 16, 20, 25, 30)          # ISO 4762 / ISO 7380 M4 stock lengths
INSERT_L, ENG_MIN, TIP_IN_MIN, TIP_SKIN_MIN = 4.0, 2.5, 2.0, 2.0          # ENG_MIN 2.5 mm = 3.5 threads (P 0.7) in the brass insert
def size_rail_screw(y, z, head_x, part, standoffs=(0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0)):
    """screw from a head seat at head_x (robot x, head on +x side) into a heat-set insert (5.6 OD x 4) at the outer face of `part`.
    Returns the smallest internal standoff + the length giving engagement ENG_MIN..INSERT_L+0.5 with the tip >= 2 mm inside the part and > 2 mm off skin."""
    h = xhits(part, y, z); xo_, xi_ = h[0], h[1]; xsk = xhits(BODY, y, z)[0]
    for so in standoffs:
        best = None
        for L in M4_LENGTHS:
            tip = head_x + so - L; eng = xo_ - tip
            if ENG_MIN <= eng <= INSERT_L + 0.5 and tip - xi_ >= TIP_IN_MIN and tip - xsk > TIP_SKIN_MIN:
                if best is None or abs(eng - INSERT_L) < abs(best[1] - INSERT_L): best = (L, eng, tip)
        if best:
            L, eng, tip = best
            return dict(length_mm=L, standoff_mm=so, engagement_mm=round(eng, 2), tip_x=round(tip, 2), part_outer_x=round(xo_, 2), part_inner_x=round(xi_, 2),
                        part_thickness_mm=round(xo_ - xi_, 2), tip_inside_part_mm=round(tip - xi_, 2), skin_x=round(xsk, 2), tip_to_skin_mm=round(tip - xsk, 2))
    raise RuntimeError("no M4 length fits at y %.1f z %.1f" % (y, z))
y_afe, z_afe = SP + 27.0, -320.0                  # unchanged (atlas + lead slot at y -123, z -320 depend on it)
AFE_W, AFE_H, AFE_D = AFE_IN[0] + 4.0, AFE_IN[1] + 4.0, AFE_IN[2] + 4.0
xo = rail_max_x(y_afe - AFE_W / 2, y_afe + AFE_W / 2, z_afe - AFE_H / 2, z_afe + AFE_H / 2) + BASE_GAP   # rev 2026-10-15: was rail_outer_x(y_afe+10, z_afe) = -122.7 (cut 2.6 cm3 into the rail ridges)
b, c = enclosure(*AFE_IN)        # local x = robot y (46 across the rail), local y = robot z (66 along it)
cq.exporters.export(b, "/tmp/_afe_b.stl", tolerance=0.05)
T_afe = R_POD.copy(); T_afe[:3, 3] = [xo, y_afe, z_afe]
bm = trimesh.load("/tmp/_afe_b.stl"); bm.apply_transform(T_afe)
SLOT = fr.get("lead_slot", dict(y=y_afe, z=z_afe, wy=2.5, hz=14.0))
afe_feats = [((y_afe, z_afe + sz * 26.0), 5.0, 2.25) for sz in (-1, 1)] + \
            [((SLOT["y"], SLOT["z"]), (SLOT["wy"] + 1.5 + 4.0, SLOT["hz"] + 1.5 + 4.0), (SLOT["wy"] + 1.5, SLOT["hz"] + 1.5))]   # lead shroud: 2 mm wall round a 4 x 15.5 window over the rail lead slot
bm = seat(bm, afe_feats)
afe_boss = {f"z{int(z_afe + sz * 26)}": boss_len(y_afe, z_afe + sz * 26.0, xo) for sz in (-1, 1)}
FLOOR = 2.0
for sz in (-1, 1):                                  # SHCS head on the floor inside the pod (x = xo + 2), M4 insert in the rail outer face
    zc = z_afe + sz * 26.0; f_ = size_rail_screw(y_afe, zc, xo + FLOOR, wing); f_.update(item="ISO 4762 M4 x %d SHCS" % f_["length_mm"], head_seat="pod floor (inside)" + (" on a %.1f mm D10 standoff pad" % f_["standoff_mm"] if f_["standoff_mm"] else ""))
    FAST["afe_pod_R z%d" % int(zc)] = f_
    if f_["standoff_mm"] > 0:                      # D10 pad on the inside of the floor raises the head seat (keeps the tip 2 mm inside the thinner rail)
        pad = trimesh.creation.cylinder(radius=5.0, height=f_["standoff_mm"] + 0.2, sections=40); pad.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
        pad.apply_translation([xo + FLOOR - 0.2 + (f_["standoff_mm"] + 0.2) / 2, y_afe, zc]); hl_ = trimesh.creation.cylinder(radius=2.25, height=40, sections=32)
        hl_.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])); hl_.apply_translation([xo, y_afe, zc])
        bm = trimesh.boolean.difference([trimesh.boolean.union([bm, pad], engine="manifold"), hl_], engine="manifold")
        bm = max(bm.split(only_watertight=False), key=lambda q: abs(q.volume))      # drop the < 1 mm3 boolean slivers (keeps the pod a single volume)
bm.merge_vertices(); bm = trimesh.util.concatenate([q for q in bm.split(only_watertight=False) if abs(q.volume) > 1.0])   # rev 2026-10-16: drop zero-volume boolean sheets / < 1 mm3 slivers
bm.apply_transform(np.linalg.inv(T_afe)); bm.export(os.path.join(OUT, "afe_pod_R.stl")); export(c, "afe_pod_lid_R")
place["afe_pod_R"] = dict(origin=[round(xo, 2), y_afe, z_afe], local_z_to="+x", local_y_to="+z",
                          note=f"rev 2026-10-16: floor {BASE_GAP} mm above the highest rail point; 2 x 10 mm bosses (lengths {afe_boss} mm) land conformally on the rail: "
                               + "; ".join(f"{k.split()[-1]}: {v['item']} (tip {v['tip_inside_part_mm']} mm inside the {v['part_thickness_mm']} mm rail, {v['tip_to_skin_mm']} mm off skin{', head on a %.1f mm standoff pad' % v['standoff_mm'] if v['standoff_mm'] else ''})" for k, v in FAST.items() if k.startswith("afe"))
                               + " into heat-set inserts (5.6 OD x 4) in the rail at z +-26; lead shroud over the rail lead slot carries the EMG leads into the floor window")
# ---- main box on the spine bridge outer face
# rev 2026-10-15 (gI): the flat floor sat on a single ray hit of the bridge (x -99.4 at y SP+6) and cut 3.92 cm3 into the bridge crown
# (outer face reaches x -95.6) and came within 1.78 mm of the wing rails. Now the floor stands BASE_GAP above the highest point of the
# bridge AND both wings under the 78 x 124 footprint, and 4 bosses land conformally on the bridge outer face (mating faces only).
# The bridge shell is only 3-4 mm thick, so the M4 heat-set inserts (5.6 OD x 4) go in the BOX bosses (pressed from the mating face) and
# M4 x 10 button-head screws (ISO 7380, head 2.2 mm) come up through 4.5 mm holes in the bridge from its inner face.
sb = trimesh.load(S + "scapular/spine_bridge.stl")
MB_W, MB_H, MB_Z = MB_IN[0] + 4.0, MB_IN[1] + 4.0, -176.0
wing_pair = trimesh.util.concatenate([wing, wing.copy().apply_transform(np.array([[1, 0, 0, 0], [0, -1, 0, 2 * SP], [0, 0, 1, 0], [0, 0, 0, 1.0]]))])
def mesh_max_x(m, y0, y1, z0, z1, n=48):
    Y, Z = np.meshgrid(np.linspace(y0, y1, n), np.linspace(z0, z1, n)); o = np.c_[np.full(Y.size, 200.0), Y.ravel(), Z.ravel()]
    loc, _, _ = m.ray.intersects_location(o, np.tile([-1.0, 0.0, 0.0], (len(o), 1)), multiple_hits=False)
    return float(loc[:, 0].max()) if len(loc) else -1e9
xs = max(mesh_max_x(sb, SP - MB_W / 2, SP + MB_W / 2, MB_Z - MB_H / 2, MB_Z + MB_H / 2),
         mesh_max_x(wing_pair, SP - MB_W / 2 - 3, SP + MB_W / 2 + 3, MB_Z - MB_H / 2 - 3, MB_Z + MB_H / 2 + 3)) + BASE_GAP + 0.3   # +0.3: the 48 x 48 ray grid under-reads the crown (floor-to-bridge measured 2.97 without it)
b, c = enclosure(*MB_IN)
if TURRET > 0:       # rev f2: E-stop turret (26 x 26 inner, 3 mm walls, top 2 mm with a 16.2 mm E-stop hole) + XT60E-M panel cut (PLACEHOLDER 15.8 x 8.4) in the lid
    ex, ey = ESTOP_XY; tz0 = MB_IN[2] + 2.0 + 2.0
    tur = cq.Workplane("XY").box(32.0, 32.0, TURRET + 0.5, centered=(True, True, False)).edges("|Z").fillet(4.0).translate((ex, ey, tz0 - 0.5))
    tur = tur.cut(cq.Workplane("XY").box(26.0, 26.0, TURRET - 2.0 + 0.5, centered=(True, True, False)).translate((ex, ey, tz0 - 0.5)))
    c = c.cut(cq.Workplane("XY").box(26.0, 26.0, 6.0, centered=(True, True, False)).translate((ex, ey, tz0 - 3.0))).union(tur)
    c = c.cut(cq.Workplane("XY").circle(8.1).extrude(TURRET + 6.0).translate((ex, ey, tz0 + TURRET - 3.0)))
    if XT60_XY: c = c.cut(cq.Workplane("XY").box(15.8, 8.4, 6.0, centered=(True, True, False)).translate((XT60_XY[0], XT60_XY[1], tz0 - 3.0)))
export(c, "main_box_lid")
cq.exporters.export(b, "/tmp/_mb_b.stl", tolerance=0.05)
T_mb = R_POD.copy(); T_mb[:3, 3] = [xs, SP, MB_Z]
mb = trimesh.load("/tmp/_mb_b.stl"); mb.apply_transform(T_mb)
MB_BOLTS = [(SP + sy * 18.0, MB_Z + sz * 45.0) for sy in (-1, 1) for sz in (-1, 1)]
R_MB_BOSS = 5.5                                    # 5.6 OD insert -> 2.2 mm wall
add = []
for yc, zc in MB_BOLTS:
    loc, _, _ = sb.ray.intersects_location(np.c_[np.full(9, 200.0), yc + R_MB_BOSS * np.cos(np.linspace(0, 2 * np.pi, 9)), zc + R_MB_BOSS * np.sin(np.linspace(0, 2 * np.pi, 9))], np.tile([-1.0, 0, 0], (9, 1)), multiple_hits=False)
    x_lo = float(loc[:, 0].min()) - 1.0; L = xs + 0.6 - x_lo
    pr = trimesh.creation.cylinder(radius=R_MB_BOSS, height=L, sections=40); pr.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    pr.apply_translation([x_lo + L / 2, yc, zc]); bo = trimesh.boolean.difference([pr, sb], engine="manifold")
    add.append(max(bo.split(only_watertight=False), key=lambda q: q.bounds[1][0]))   # keep the piece above the shell; drop slivers that slip under the 3-4 mm bridge
mb = trimesh.boolean.union([mb] + add, engine="manifold")
holes = []
MB_BORE = 7.0                                      # rev 2026-10-16: 5 -> 7 mm deep so the M4 x 8 (4.7..5.5 mm past the bridge) never bottoms (boss + floor >= 10 mm above the bridge face)
for yc, zc in MB_BOLTS:                            # blind insert bore from the mating face: 5.3 mm (5.6 OD insert, PETG hot-press), MB_BORE deep
    h = trimesh.creation.cylinder(radius=2.65, height=2 * MB_BORE, sections=32); h.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    loc, _, _ = sb.ray.intersects_location([[200.0, yc, zc]], [[-1.0, 0, 0]]); xb = float(loc[:, 0].max())
    h.apply_translation([xb, yc, zc]); holes.append(h)   # spans xb-MB_BORE .. xb+MB_BORE: MB_BORE into the boss above the bridge face
    hb_ = xhits(sb, yc, zc); t_br = hb_[0] - hb_[1]; L_ = 8; eng = L_ - t_br
    head = trimesh.creation.cylinder(radius=3.8, height=2.2, sections=32); head.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0])); head.apply_translation([hb_[1] - 1.1, yc, zc])
    d_sk = float(np.min(np.abs(trimesh.proximity.signed_distance(BODY, head.vertices))))
    FAST["main_box y%+d z%d" % (int(yc - SP), int(zc))] = dict(item="ISO 7380 M4 x 8 button head (up from the bridge inner face)", length_mm=L_, bridge_thickness_mm=round(t_br, 2),
        engagement_past_bridge_mm=round(eng, 2), bore_depth_mm=MB_BORE, tip_clear_of_bore_bottom_mm=round(MB_BORE - eng, 2), head_to_skin_mesh_mm=round(d_sk, 2), head_to_skin_ray_mm=round(hb_[1] - 2.2 - xhits(BODY, yc, zc)[0], 2))
mb = trimesh.boolean.difference([mb] + holes, engine="manifold")
mb_boss = {f"y{int(y - SP):+d}_z{int(z)}": round(xs - mesh_max_x(sb, y - 0.1, y + 0.1, z - 0.1, z + 0.1, n=2), 1) for y, z in MB_BOLTS}
mb.apply_transform(np.linalg.inv(T_mb)); mb.export(os.path.join(OUT, "main_box.stl"))
place["main_box"] = dict(origin=[round(xs, 2), SP, MB_Z], local_z_to="+x", local_y_to="+z",
                         note=f"rev 2026-10-16 (gI): floor {BASE_GAP + 0.3:.1f} mm above the highest bridge/wing point (ray grid; measured >= 3.0 mesh-to-mesh) under the footprint; 4 x 11 mm bosses (lengths {mb_boss} mm) "
                              f"land conformally on the spine bridge at y SP +-18, z {MB_Z:.0f} +-45; M4 heat-set inserts (5.6 OD x 4) pressed into the boss faces, "
                              "ISO 7380 M4 x 8 button-head screws up through 4.5 mm holes in the bridge into 7 mm blind insert bores (was M4 x 10 into 5 mm: bottomed); "
                              f"head-to-skin min {min(v['head_to_skin_mesh_mm'] for k, v in FAST.items() if k.startswith('main')):.1f} mm (mesh; gate >= 10) (bridge holes not yet in scapular_frame.py: drill from a template). "
                              "E-stop button on the lid, XT60 battery input, 2 x CAN + 4 x TTL bus connectors (JST-XH / Molex Micro-Fit)")
# ---- battery box on a cradle between the rails
# rev 2026-10-15 (open item 0(h), Forest approved): box and cradle moved down from z -345 to Z_BAT so they clear the AFE pods (z -355..-285)
# by >= 3 mm; the rail runs to -440 (rev d). Cradle was box(112, 6, 90) with no rotation = 112 mm along x through the back (a
# modelling bug: the plate was meant to be 6 thick in x and span the rails in y). Now 6 (x) x 100 (y) x 74 (z), floor 3 mm off the
# highest rail point, 4 bosses conformal to the rail at y SP +-31 (insert zone |y| >= 26), z Z_BAT +-28.
Z_BAT = -398.0
b, c = enclosure(*BAT_IN); export(b, "battery_box"); export(c, "battery_box_lid")
BAT_W, BAT_H = BAT_IN[0] + 4.0, BAT_IN[1] + 4.0
CR_T, CR_W, CR_H = 6.0, 100.0, BAT_H + 8.0
xr = rail_max_x(SP + 12.0, SP + 50.0, Z_BAT - CR_H / 2, Z_BAT + CR_H / 2) + BASE_GAP     # cradle rear face (rail is symmetric about the spine)
cradle = trimesh.creation.box([CR_T, CR_W, CR_H]); cradle.apply_translation([xr + CR_T / 2, SP, Z_BAT])
CR_BOLTS = [(SP + sy * 31.0, Z_BAT + sz * 28.0) for sy in (-1, 1) for sz in (-1, 1)]
wing_R_only = wing; wing = trimesh.util.concatenate([wing_R_only, wing_R_only.copy().apply_transform(np.array([[1, 0, 0, 0], [0, -1, 0, 2 * SP], [0, 0, 1, 0], [0, 0, 0, 1.0]]))])
cradle = seat(cradle, [(p, 5.0, 2.25) for p in CR_BOLTS])
SPOT_D, SPOT_H = 8.0, 2.5                                            # rev 2026-10-16: D8 x 2.5 spot-faces in the cradle front face for the ISO 7380 M4 heads (dk 7.6, k 2.2): heads sit 0.3 below the battery-box seat
for yc, zc in CR_BOLTS:
    sf = trimesh.creation.cylinder(radius=SPOT_D / 2, height=SPOT_H + 1.0, sections=40); sf.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
    sf.apply_translation([xr + CR_T - SPOT_H + (SPOT_H + 1.0) / 2, yc, zc]); cradle = trimesh.boolean.difference([cradle, sf], engine="manifold")
wing_pair_c = wing
for yc, zc in CR_BOLTS:
    f_ = size_rail_screw(yc, zc, xr + CR_T - SPOT_H, wing_pair_c, standoffs=(0.0,)); f_.update(item="ISO 7380 M4 x %d button head in a D8 x 2.5 spot-face" % f_["length_mm"])
    FAST["cradle y%+d z%d" % (int(yc - SP), int(zc))] = f_
BF_Z = 14.0                                                          # rev 2026-10-16: box-floor inserts z +-20 -> +-14 (were 1.3 mm from the new spot-faces)
for yy in (-1, 1):                                                   # 4 x M4 heat-set inserts for the battery box floor (box bolts from inside)
    for zz in (-1, 1):
        h = trimesh.creation.cylinder(radius=2.8, height=CR_T + 1, sections=32); h.apply_transform(trimesh.transformations.rotation_matrix(np.pi / 2, [0, 1, 0]))
        h.apply_translation([xr + CR_T / 2 + 1.5, SP + yy * 30.0, Z_BAT + zz * BF_Z]); cradle = trimesh.boolean.difference([cradle, h], engine="manifold")
POCKET_BOTTOM = xr + 1.0; BOX_FLOOR = 2.0
for L_ in (6, 8, 10):
    tip = xr + CR_T + BOX_FLOOR - L_
    FAST.setdefault("battery_box_floor_options", {})["M4 x %d" % L_] = dict(tip_x=round(tip, 2), engagement_in_insert_mm=round(min(xr + CR_T - tip, INSERT_L), 2),
        tip_above_pocket_bottom_mm=round(tip - POCKET_BOTTOM, 2), tip_inside_cradle_back_mm=round(tip - xr, 2))
FAST["battery_box_floor"] = dict(item="ISO 7380 M4 x 6 button head (box floor 2 + 4 mm full insert engagement)", **FAST["battery_box_floor_options"]["M4 x 6"],
                                 spacing_to_spot_face_wall_mm=round(float(np.hypot(1.0, 28.0 - BF_Z)) - SPOT_D / 2 - 2.8, 2))
wing = wing_R_only
cr_boss = {f"y{int(y - SP):+d}_z{int(z)}": boss_len(abs(y - SP) + SP, z, xr) for y, z in CR_BOLTS}
cradle.apply_translation([-(xr + CR_T / 2), -SP, -Z_BAT]); cradle.export(os.path.join(OUT, "battery_cradle.stl"))
place["battery_box"] = dict(origin=[round(xr + CR_T, 2), SP, Z_BAT], local_z_to="+x", local_y_to="+z",
                            note="6S2P 18650 (e.g. Molicel P30B / Samsung 30Q class, 3.0 Ah) + 6S BMS 20 A; vented, fire-resistant liner. rev 2026-10-15: "
                                 "rev 2026-10-16: 4 x ISO 7380 M4 x 6 through the 2 mm box floor into heat-set inserts (5.6 OD x 4) in the cradle at y SP +-30, z -398 +-14 (M4 x 8 would reach the 1 mm pocket floor, M4 x 10 protruded 2 mm); 4 x 2.2 mm cradle-bolt heads sit in D8 x 2.5 spot-faces under the box floor; XT60 lead leaves the top wall at the lid edge and runs up a lid clip at x -30 to z -225, then in to an XT60 on the main box lid's lower end (>= 50 mm from both AFE pods, see pods_clearance.json)")
place["battery_cradle"] = dict(origin=[round(xr + CR_T / 2, 2), SP, Z_BAT],
                               note=f"PETG-CF plate 6 x 100 x {CR_H:.0f}, 4 conformal bosses (lengths {cr_boss} mm) on the rails: ISO 7380 M4 x {FAST['cradle y-31 z-426']['length_mm']} button heads in D8 x 2.5 spot-faces "
                                    f"into heat-set inserts (5.6 OD x 4) in the rails at y SP +-31, z {Z_BAT:.0f} +-28 (tip >= {min(v['tip_inside_part_mm'] for k, v in FAST.items() if k.startswith('cradle')):.1f} mm inside the rail)")
# ---- IMU pod and LRA cup
b, c = enclosure(*IMU_IN, wall=1.6, r=2.0, lid=1.6)
b = b.cut(cq.Workplane("XY").box(30.0, 3.0, 4.0).translate((0, 0, 1.0)))       # webbing slot across the floor
export(b, "imu_pod"); export(c, "imu_pod_lid")
cup = cq.Workplane("XY").circle(7.0).extrude(5.0).cut(cq.Workplane("XY").circle(5.2).extrude(5.0).translate((0, 0, 1.2)))
export(cup, "lra_cup")
masses = {}
for f in os.listdir(OUT):
    if f.endswith(".stl"): masses[f[:-4]] = round(abs(trimesh.load(os.path.join(OUT, f)).volume) * 1.27e-3, 1)
contents_g = DIMS.get("contents_mass_g", {"afe_pod_R": 45.0, "main_box": 180.0, "battery_box": 600.0, "imu_pod": 4.0})
json.dump(dict(place=place, printed_mass_g=masses, contents_mass_g=contents_g, fasteners=FAST), open(os.path.join(OUT, "pods.json"), "w"), indent=1)
print(json.dumps(FAST, indent=1))
print(json.dumps(dict(place=place, printed_mass_g=masses), indent=1))
