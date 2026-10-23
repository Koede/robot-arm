"""Electronics fit check (rev f, 2026-10-17): component library, box layouts, mass/budget and wall-connector checks.
Run from cad/:  python3 electronics_fit.py        (reads stl/pods/pods.json; writes stl/electronics/electronics_fit.json,
                                                    stl/pods/pod_dims.json (read by sensor_pods.py), renders/electronics_layout.png)
Dimension provenance flags:  R = retrieved from a vendor/retailer page this project (see SPEC_electronics_fit.md sources),
  F = family inference (same footprint family as a retrieved part), T = typical value from memory, NOT verified (PLACEHOLDER),
  C = custom board of ours (assumed outline, set by this script).  Nothing flagged T/F/C may be printed or ordered against without a datasheet check.
Frames: pod local x = robot y, local y = robot z, local z = robot x (outward from the back). Heights H are along local z (depth)."""
import os, sys, json, math, random, itertools
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
OUT = os.path.join(ROOT, "stl", "electronics"); os.makedirs(OUT, exist_ok=True)
RND = os.path.join(ROOT, "renders"); os.makedirs(RND, exist_ok=True)
PODS = json.load(open(os.path.join(ROOT, "stl", "pods", "pods.json")))
BASE = dict(main_box=(74.0, 120.0, 32.0), battery_box=(80.0, 62.0, 72.0), afe_pod_R=(46.0, 66.0, 24.0), imu_pod=(26.6, 18.8, 6.0))   # rev f inner dims (x=robot y, y=robot z, z=depth)
BUDGET = dict(afe_pod_R=45.0, main_box=180.0, battery_box=600.0, imu_pod=4.0)  # rev f record (the harness-model inputs before this fit)                                                                                                     # 45 / 180 / 600 / 4 g, the harness model inputs

# --------------------------------------------------------------------------------------------- component library
# (name, L, W, H, mass_g, flag, note)   L x W is the footprint, H the height in the mounting attitude given
LIB = {
 "teensy41":     (61.0, 17.8, 13.0, 10.0, "R/T", "L x W 2.4 x 0.7 in (retailer page, R); H ~ 4.2 board + SD socket plus 8.5 for female sockets (T)"),
 "tca9548a":     (30.6, 17.6, 5.2, 2.0, "R/T", "Adafruit 2717 L x W (R); H 2.7 board + ~2.5 header (T). Needs TWO (see below)"),
 "bno085":       (25.6, 22.7, 4.6, 3.0, "R", "Adafruit 4754 L x W x H (R); STEMMA QT plug adds ~3.5 mm beyond one short edge (T)"),
 "drv2605l":     (25.8, 17.8, 4.6, 2.5, "R", "Adafruit 2305 class breakout (R)"),
 "buck12_15A":   (43.2, 31.8, 11.0, 22.0, "F", "Pololu D24V150F12 assumed same footprint as the retrieved D24V150F6 (43.2 x 31.8 x 11); mass T"),
 "buck5_3A":     (40.6, 20.3, 7.6, 9.0, "R", "Pololu D24V90F5 footprint (R) used as the proxy for the 5 V 3 A module; mass T"),
 "buck12_ctl":   (17.8, 17.8, 7.9, 4.0, "R", "Pololu D24V22F5 footprint (R) as proxy for the small 12 V_ctl coil-chain buck"),
 "relay_TE_ref": (25.9, 29.9, 51.9, 38.0, "R/T", "TE plug-in automotive relay 25.9 x 29.9 x 51.9 (retailer page, R); 12 V coil variant and mass assumed (T). DC rating at 24 V NOT retrieved"),
 "relay_envelope":(30.0, 16.0, 28.0, 25.0, "T", "ENVELOPE REQUIREMENT, not a part: a sealed 12 V-coil power relay <= 30 x 16 x 28 mm, >= 30 A make / break at 28 VDC (datasheet check pending)"),
 "fuse_holder":  (44.0, 22.0, 17.0, 12.0, "T", "ATO/MINI inline blade-fuse holder, PLACEHOLDER"),
 "interlock_pcb":(55.0, 35.0, 12.0, 18.0, "C", "hard-chain comparators A,B,C,D, SR latch, Q1, off-delay timer (replaces the K2T DIN relay), 12 V_ctl rails"),
 "carrier_pcb":  (64.0, 42.0, 14.6, 24.0, "C", "Teensy 4.1 on sockets + 2 x TCA9548A + 2 x SN65HVD230 + 74LVC2G241 buffers on one custom board (rows along the long edge)"),
 "estop":        (26.0, 26.0, 40.0, 30.0, "R/T", "Omron A165E 16 mm cutout, 2 NC, overall depth 48.5 (R); behind-panel depth ~31.5 + 8.5 terminal/wire clearance assumed (T)"),
 "xt60_panel":   (16.0, 22.0, 30.0, 10.0, "T", "XT60E-M panel male body ~15.5 x 8 x 16 (T), solder cups + 14 AWG bend: keep-out"),
}
ESTOP_BEHIND_PANEL = 31.5; ESTOP_WIRE = 8.5
MAIN_CONNECTORS = [   # (name, qty, width along wall mm, flag)   width = shrouded housing incl. latch, T unless noted
 ("XT30 panel 24 V / 12 V arm power", 6, 10.5, "T"), ("JST-GH 4 CAN (R, L)", 2, 8.0, "T"), ("JST-EH 3 XM TTL + STS TTL", 4, 8.5, "T"),
 ("JST-GH 8 AFE (R, L)", 2, 12.0, "T"), ("JST-SH 4 STEMMA QT I2C ports (arm IMUs + haptics)", 8, 7.0, "T"), ("micro-USB (Teensy)", 1, 8.0, "T"),
 ("charge-detect / spare", 1, 8.0, "T")]
GAP_ITEM, GAP_WALL, GAP_STACK, LID_CLR = 3.0, 1.5, 2.5, 1.0     # wiring gap between items, from the wall, between stacked items, between the tallest item and the lid

# --------------------------------------------------------------------------------------------- packing
def pack(cols, W, H, seed, tries=300):
    """guillotine pack of columns [(w, h)] (each may rotate) into W x H with GAP_ITEM; returns placements or None."""
    Wf, Hf = W - 2 * GAP_WALL, H - 2 * GAP_WALL
    rng = random.Random(seed)
    best = None
    for t in range(tries):
        order = list(range(len(cols))); rng.shuffle(order) if t else order.sort(key=lambda i: -cols[i][0] * cols[i][1])
        free = [(0.0, 0.0, Wf, Hf)]; plc = {}
        ok = True
        for i in order:
            w, h = cols[i]; cand = None
            for rot in (0, 1):
                ww, hh = (w, h) if not rot else (h, w)
                for fi, (fx, fy, fw, fh) in enumerate(free):
                    if ww <= fw + 1e-9 and hh <= fh + 1e-9:
                        sc = min(fw - ww, fh - hh)
                        if cand is None or sc < cand[0]: cand = (sc, fi, rot, ww, hh)
            if cand is None: ok = False; break
            _, fi, rot, ww, hh = cand; fx, fy, fw, fh = free.pop(fi)
            plc[i] = (fx + GAP_WALL, fy + GAP_WALL, ww, hh, rot)
            rw, rh = fw - ww - GAP_ITEM, fh - hh - GAP_ITEM
            if rh > 0: free.append((fx, fy + hh + GAP_ITEM, ww if rw < rh else fw, rh))
            if rw > 0: free.append((fx + ww + GAP_ITEM, fy, rw, fh if rw < rh else hh))
            if rh > 0 and rw >= rh: free[-2] = (fx, fy + hh + GAP_ITEM, fw, rh)
        if ok: return plc
    return None

def stack_search(items, W, H, Dmax, n_iter=4000, seed=1):
    """items: dict name -> (L, Wd, Ht, stackable). Randomly group stackable items into columns (Sum Ht + gaps <= Dmax - LID_CLR) and pack columns.
    Returns (plan, utilisation) with the smallest total column footprint area that packs, or None."""
    rng = random.Random(seed); names = list(items); st = [n for n in names if items[n][3]]; fixed = [n for n in names if not items[n][3]]
    best = None
    for it in range(n_iter):
        k = rng.randint(1, max(1, len(st))); assign = {n: rng.randrange(k) for n in st}
        groups = {}
        for n, g in assign.items(): groups.setdefault(g, []).append(n)
        cols = []; ok = True
        for g, ns in groups.items():
            hsum = sum(items[n][2] for n in ns) + GAP_STACK * (len(ns) - 1)
            if hsum > Dmax - LID_CLR: ok = False; break
            # footprint: all members aligned (same orientation), take the max in each dimension
            w = max(items[n][0] for n in ns); h = max(items[n][1] for n in ns)
            cols.append((ns, w, h, hsum))
        if not ok: continue
        for n in fixed: cols.append(([n], items[n][0], items[n][1], items[n][2]))
        if any(c[3] > Dmax - LID_CLR for c in cols): continue
        area = sum(c[1] * c[2] for c in cols)
        if best is not None and area >= best[0]: continue
        plc = pack([(c[1], c[2]) for c in cols], W, H, seed=it)
        if plc: best = (area, cols, plc)
    return best

TURRET = 8.0      # lid turret over the E-stop (26 x 26 footprint, 32 x 32 outside): the E-stop body (31.5 behind the panel + 8.5 wire room) stands up into it
def main_box_items(D, relay="relay_envelope", fuse_in_box=False):
    L = LIB; it = {}
    it["carrier_pcb (Teensy + 2 mux + CAN + buffers)"] = (L["carrier_pcb"][0], L["carrier_pcb"][1], L["carrier_pcb"][2], True)
    it["buck 24->12 V 15 A"] = (L["buck12_15A"][0], L["buck12_15A"][1], L["buck12_15A"][2], True)
    it["buck 24->5 V 3 A"] = (L["buck5_3A"][0], L["buck5_3A"][1], L["buck5_3A"][2], True)
    it["buck 12 V_ctl"] = (L["buck12_ctl"][0], L["buck12_ctl"][1], L["buck12_ctl"][2], True)
    it["interlock PCB (hard chain + timer)"] = (L["interlock_pcb"][0], L["interlock_pcb"][1], L["interlock_pcb"][2], True)
    if fuse_in_box: it["fuse holder 20 A"] = (L["fuse_holder"][0], L["fuse_holder"][1], L["fuse_holder"][2], True)
    for k in ("K1 pole +", "K1 pole -", "K1 aux (phase-short drive)"):      # relays lie on the 29.9 mm side: footprint 51.9 x 25.9, H 29.9
        r = L[relay]
        it[k] = (r[2], r[0], r[1], False) if relay == "relay_TE_ref" else (r[0], r[1], r[2], False)    # TE ref lies on its 29.9 mm side: 51.9 x 25.9, H 29.9
    it["E-stop body (lid turret)"] = (L["estop"][0], L["estop"][1], ESTOP_BEHIND_PANEL + ESTOP_WIRE - TURRET - 2.0, False)   # 2.0 = lid thickness
    it["XT60E-M (lid, lower end)"] = (L["xt60_panel"][0], L["xt60_panel"][1], L["xt60_panel"][2], False)
    return it

MASS_MAIN = {"carrier_pcb (Teensy + 2 mux + CAN + buffers)": 24 + 2, "buck 24->12 V 15 A": 22, "buck 24->5 V 3 A": 9, "buck 12 V_ctl": 4, "interlock PCB (hard chain + timer)": 18,
             "fuse holder 20 A": 12, "K1 pole +": 38, "K1 pole -": 38, "K1 aux (phase-short drive)": 38, "E-stop body (lid turret)": 30, "XT60E-M (lid, lower end)": 10,
             "wall connectors (24 housings)": 24 * 3.5, "internal wiring (14/16/18 AWG, signal)": 40.0}

def search_case(relay, fuse_in_box, H, D, n_iter=6000, seed=5, W=74.0):
    items = main_box_items(D, relay, fuse_in_box)
    if any(v[2] > D - LID_CLR for v in items.values()): return dict(ok=False, why="a part is taller than the interior")
    best = stack_search(items, W, H, D, n_iter=n_iter, seed=seed)
    if best is None: return dict(ok=False, why="no packing found")
    area, cols, plc = best
    plan = []
    for i, c in enumerate(cols):
        x, y, pw, ph, rot = plc[i]; plan.append(dict(items=c[0], stack_h=round(c[3], 1), rot=int(rot), col_w0=c[1], col_h0=c[2], cx=round(x + pw / 2 - W / 2, 1), cy=round(y + ph / 2 - H / 2, 1), w=round(pw, 1), h=round(ph, 1)))
    # the battery lead enters at the lower end: mirror the layout in local y if the XT60 landed in the upper half
    xt = [p for p in plan if any("XT60" in n for n in p["items"])][0]
    if xt["cy"] > 0:
        for p in plan: p["cy"] = -p["cy"]
    return dict(ok=True, used_area_pct=round(100 * area / ((W - 3) * (H - 3)), 1), plan=plan)

CHOSEN = dict(H=132.0, D=34.0, relay="relay_envelope", fuse_in_box=False)
def main_box_fit(full=True):
    out = dict(cases={})
    if full:
        out["cases"]["A: rev f box 74x120x32, TE plug-in relay 51.9 long, fuse in box (E-stop on a turret)"] = {str(D): search_case("relay_TE_ref", True, 120.0, D, n_iter=1500) for D in (32.0, 44.0, 52.0)}
        out["cases"]["B: 74x120, compact relay envelope 30x16x28, fuse in box"] = {str(D): search_case("relay_envelope", True, 120.0, D, n_iter=3000) for D in (32.0, 34.0, 36.0, 44.0)}
        out["cases"]["C: 74x120, compact relay, fuse moved to the pack lead"] = {str(D): search_case("relay_envelope", False, 120.0, D, n_iter=3000) for D in (32.0, 34.0, 36.0)}
    out["chosen"] = dict(CHOSEN, **search_case(CHOSEN["relay"], CHOSEN["fuse_in_box"], CHOSEN["H"], CHOSEN["D"], n_iter=12000))
    return out

# --------------------------------------------------------------------------------------------- battery box
def battery_fit():
    CELL_D, CELL_L = 18.6, 65.2          # IEC 61960 18650 max (T)
    CELL_G = 0.4                          # gap between cells (fish paper / cell holder 0.2 each side)
    cols, rows = 4, 3
    cross_y, cross_z = cols * (CELL_D + CELL_G), rows * (CELL_D + CELL_G)
    LINER = 1.0                           # fire-resistant liner (PLACEHOLDER thickness) on every wall
    STRIP, INSUL = 0.15, 0.3              # nickel strip, kapton / fish paper under each end
    HEAD = 2.2                            # ISO 7380 M4 button head standing on the box floor inside (4 floor bolts)
    BMS = (50.0, 32.0, 4.0)               # 6S 20 A BMS board, 50 x 32 x 4 (retailer page, R)
    BMS_CLR = 1.0                         # bend room for balance leads / thermal
    need_y, need_z = cross_y + 2 * LINER, cross_z + 2 * LINER
    need_x = CELL_L + 2 * (STRIP + INSUL) + HEAD + LINER * 2 + BMS[2] + BMS_CLR
    mass = dict(cells_12x47g=12 * 47.0, strips=14.0, bms=10.0, balance_and_power_wire=18.0, liner=20.0, fish_paper_kapton=6.0)
    return dict(cell_d=CELL_D, cell_l=CELL_L, grid=f"{cols} x {rows} square", cross_needed_mm=[round(need_y, 1), round(need_z, 1)], inner_have_mm=[BASE["battery_box"][0], BASE["battery_box"][1]],
                depth_needed_mm=round(need_x, 1), depth_have_mm=BASE["battery_box"][2], mass_g=mass, mass_total_g=round(sum(mass.values()), 1),
                bms_fits_cross=BMS[0] <= BASE["battery_box"][0] - 2 * LINER and BMS[1] <= BASE["battery_box"][1] - 2 * LINER, notes=dict(cells="Molicel P30B / Samsung 30Q class, 47-48 g each (T)", liner=LINER, head=HEAD, bms=BMS))

# --------------------------------------------------------------------------------------------- AFE pod
def rand_place(items, W, H, obstacles=(), gap=2.0, wall=1.0, tries=20000, seed=0):
    """random sequential placement of rectangles (w, h, rot allowed) into W x H avoiding obstacles (x, y, w, h); returns placements or None"""
    rng = random.Random(seed); n = len(items)
    for t in range(tries):
        placed = []; ok = True
        order = sorted(range(n), key=lambda i: -items[i][0] * items[i][1])
        for i in order:
            w0, h0 = items[i][0], items[i][1]; done = False
            for k in range(60):
                rot = rng.random() < 0.5; w, h = (h0, w0) if rot else (w0, h0)
                if w + 2 * wall > W or h + 2 * wall > H: continue
                x = rng.uniform(wall, W - wall - w); y = rng.uniform(wall, H - wall - h)
                if all(x + w + gap <= px or px + pw + gap <= x or y + h + gap <= py or py + ph + gap <= y for (px, py, pw, ph) in [(p[1], p[2], p[3], p[4]) for p in placed] + list(obstacles)):
                    placed.append((i, x, y, w, h)); done = True; break
            if not done: ok = False; break
        if ok: return placed
    return None

def afe_fit():
    ix, iy, iz = BASE["afe_pod_R"]
    fasteners = PODS["fasteners"]
    pads = [v["standoff_mm"] for k, v in fasteners.items() if k.startswith("afe_pod_R")]
    HEAD_H, HEAD_D = 4.0, 7.0                                            # ISO 4762 M4 SHCS head (standard)
    post = HEAD_H + max(pads) + 0.5                                      # board posts stand over the screw heads (screws are driven first, board after)
    PCB_T = 1.6
    pcb = (ix - 4.0, iy - 4.0)                                           # 42 x 62 board, 2 mm to every wall
    iso_barrier, host_strip = 8.0, 11.0                                  # 2 x MOPP target 8 mm creepage; host strip holds the JST-GH 8 + host side of the ISO7762
    comp = dict(ads1299=(12.0, 12.0, 1.6), iso7762_dw=(10.3, 10.3, 2.65), dcdc_2xmopp=(19.0, 11.0, 10.0), ffc_hdr_16=(14.0, 8.0, 7.0), jst_gh8=(11.7, 6.0, 5.0), ldo_caps=(14.0, 10.0, 3.0))
    iso_len = pcb[1] - host_strip - iso_barrier; iso_wid = pcb[0]        # isolated area (both faces)
    # screw heads at board-local (iso_wid/2, 5) and (iso_wid/2, pcb_l - 5): keep-out squares on the UNDER face (heads stand under the board)
    # host strip at local +y (11 mm, head 1 under it), barrier 8 mm, isolated area from the far (-y) end
    obst_bot = [(iso_wid / 2 - 4.5, 5.0 - 4.5, 9.0, 9.0)]     # host end at local +y (toward the main box); isolated area spans local y -31..+12; head 2 is 5 mm from the far end
    top_items = [(comp["ffc_hdr_16"][0], comp["ffc_hdr_16"][1])] * 3 + [(comp["dcdc_2xmopp"][0] / 2 + 1.0, comp["dcdc_2xmopp"][1])] * 2 + [(comp["iso7762_dw"][0] / 2, comp["iso7762_dw"][1])]
    bot_items = [(comp["ads1299"][0], comp["ads1299"][1])] * 3 + [(comp["ldo_caps"][0], comp["ldo_caps"][1])]
    top = rand_place(top_items, iso_wid, iso_len, seed=1); bot = rand_place(bot_items, iso_wid, iso_len, obstacles=obst_bot, seed=2)
    # single-sided check for the record
    single = rand_place(top_items + bot_items, iso_wid, iso_len, seed=3, tries=3000)
    tall_top = max(comp["dcdc_2xmopp"][2], comp["ffc_hdr_16"][2]); tall_bot = comp["ads1299"][2]
    stack_h = post + PCB_T + tall_top
    under_room = post - max(comp["ads1299"][2], comp["ldo_caps"][2]) - 0.3
    n_din, din_pitch = 48, 8.0
    mass = dict(pcb=round(pcb[0] * pcb[1] * PCB_T * 1.85e-3, 1), parts=12.0, connectors=8.0, lead_service_loop_48=12.0)
    return dict(inner=[ix, iy, iz], board=pcb, post_mm=round(post, 1), top_stack_height_mm=round(stack_h, 1), head_room_mm=round(iz - stack_h, 1), under_board_room_mm=round(under_room, 1),
                isolated_area_mm=[iso_wid, round(iso_len, 1)], two_sided_ok=top is not None and bot is not None, single_sided_ok=single is not None,
                top_layout=top, bottom_layout=bot, creepage_barrier_mm=iso_barrier, host_strip_mm=host_strip,
                din_sockets=dict(n=n_din, pitch=din_pitch, area_needed_mm2=n_din * din_pitch ** 2, area_have_mm2=ix * iy, perimeter_slots=int((2 * (ix + iy) - 16) // din_pitch)),
                mass=mass, mass_total_g=round(sum(mass.values()), 1))

# --------------------------------------------------------------------------------------------- IMU pod
def imu_fit():
    ix, iy, iz = BASE["imu_pod"]; b = LIB["bno085"]; PLUG, CLR = 3.5, 0.5
    SLOT_TOP = 3.0; WALL = 1.6
    need_x = b[0] + PLUG + 2 * CLR; need_y = b[1] + 2 * CLR
    board_z0 = SLOT_TOP + 0.4 - WALL                                    # height of the board bottom above the cavity floor (webbing slot top at pod z 3.0)
    need_z = board_z0 + b[2] + CLR
    return dict(have=[ix, iy, iz], need=[round(need_x, 1), round(need_y, 1), round(need_z, 1)], ok=(ix >= need_x and iy >= need_y and iz >= need_z), board=b[:3])

# --------------------------------------------------------------------------------------------- wall connector budget
def connectors(dims):
    ix, iy, iz = dims; outer_perim = 2 * ((ix + 4) + (iy + 4))
    tot = sum(q * w for _, q, w, _ in MAIN_CONNECTORS) + 1.5 * sum(q for _, q, _, _ in MAIN_CONNECTORS)
    avail = outer_perim - 4 * 2 * 3.0 - 2 * 16.0 - 30.0     # minus corner fillets, XT60 end and the lid-edge battery lead exit
    return dict(count=sum(q for _, q, _, _ in MAIN_CONNECTORS), width_incl_gaps_mm=round(tot, 1), wall_length_available_mm=round(avail, 1), ok=tot <= avail)

# --------------------------------------------------------------------------------------------- arm cable bundles (copper + insulation OD)
def arm_cables():
    od = {"16 AWG silicone": 2.3, "18 AWG silicone": 1.9, "24 AWG shielded CAN pair": 3.0, "26 AWG 3-wire TTL": 1.2}
    s1 = [("16 AWG silicone", 4), ("24 AWG shielded CAN pair", 1), ("18 AWG silicone", 2), ("26 AWG 3-wire TTL", 2)]      # 24 V x2 AK, CAN, 12 V, XM TTL, STS TTL
    elbow = [("18 AWG silicone", 2), ("26 AWG 3-wire TTL", 2)]
    def bundle(lst, fill=0.6):
        a = sum(q * math.pi / 4 * od[n] ** 2 for n, q in lst); return dict(copper_area_mm2=round(a, 1), min_bore_d_mm=round(math.sqrt(a / fill * 4 / math.pi), 1), n_cables=sum(q for _, q in lst))
    return dict(across_S1_yoke_S2=bundle(s1), across_elbow_slot=dict(**bundle(elbow), slot_mm2=18 * 8, slot_have="18 x 8 harness slot (upper_arm_elbow.py)"))

# --------------------------------------------------------------------------------------------- thermal estimate of the main box
def thermal_main(dims):
    ix, iy, iz = dims; A = 2 * ((ix + 4) * (iy + 4) + (ix + 4) * (iz + 4 + TURRET * 0.2) + (iy + 4) * (iz + 4 + TURRET * 0.2)) * 1e-6      # outer surface, m2
    eta12, eta5 = 0.93, 0.90                                             # 93 % from the 15 A module listing (R); 5 V buck assumed (T)
    p12_avg, p12_worst = 34.0, 89.8                                      # W on the 12 V rail, power_budget.json duty mix (0.5/0.45/0.05) and worst-hold both arms
    loads = dict(buck12_loss_avg=p12_avg * (1 / eta12 - 1), buck5_loss=3.0 * (1 / eta5 - 1), relay_coils_3=3 * 1.0, teensy_can_logic=1.5, interlock=0.5, wiring_contacts=0.1)
    avg = sum(loads.values()); worst = avg - loads["buck12_loss_avg"] + p12_worst * (1 / eta12 - 1) + 0.4
    econ = avg - 3 * 1.0 * 0.5                                           # relay economizer: hold at ~50 % coil power
    res = {}
    for h in (6.0, 8.0, 10.0):                                           # total (convection + radiation) coefficient W/m2K, worn under a shirt: T
        res[f"h{h:g}"] = dict(dT_avg_K=round(avg / (h * A), 1), dT_worst_steady_K=round(worst / (h * A), 1), dT_avg_economizer_K=round(econ / (h * A), 1))
    return dict(area_m2=round(A, 4), loads_W={k: round(v, 2) for k, v in loads.items()}, avg_W=round(avg, 1), worst_W=round(worst, 1), economizer_avg_W=round(econ, 1), dT=res,
                gate="shell rise <= 20 K average (<= 55 C at 35 C ambient); PETG-CF HDT ~ 70+ C (T). Thermal time constant ~ 25 min (0.4 kg shell + contents / 0.3 W/K)")

# --------------------------------------------------------------------------------------------- run
def run(write=True, full=True):
    out = {}
    mf = main_box_fit(full)
    ch = mf["chosen"]; out["main_box"] = dict(cases={k: {d: dict(ok=v["ok"], used_area_pct=v.get("used_area_pct"), why=v.get("why")) for d, v in c.items()} for k, c in mf["cases"].items()}, chosen=ch)
    MASS_MAIN["K1 pole +"] = MASS_MAIN["K1 pole -"] = MASS_MAIN["K1 aux (phase-short drive)"] = LIB["relay_envelope"][3]; MASS_MAIN.pop("fuse holder 20 A", None)
    out["main_box"]["mass_g"] = MASS_MAIN; out["main_box"]["mass_total_g"] = round(sum(MASS_MAIN.values()), 1); out["main_box"]["budget_g_rev_f"] = BUDGET["main_box"]
    out["main_box"]["fuse"] = "20 A ATO inline holder moves to the pack lead, strapped to the battery box top at the lead exit (12 g + 0.3 m wire), NOT in the main box"
    out["battery_box"] = battery_fit(); out["afe_pod"] = afe_fit(); out["imu_pod"] = imu_fit(); out["connectors"] = connectors((BASE["main_box"][0], CHOSEN["H"], CHOSEN["D"])); out["arm_cables"] = arm_cables()
    out["i2c_ports"] = dict(tca9548a_ports_per_chip=8, imu_ports_used=5, drv2605l_count=8, drv2605l_address="0x5A fixed (one per port)", ports_left_on_one_chip=3,
                            chips_needed_for_8_drv=2, note="the 8 DRV2605L share address 0x5A, so each needs its own port; one TCA9548A leaves 3. A second TCA9548A at 0x71 gives 11 free ports; both sit on the carrier PCB (already in the carrier footprint)")
    # ---- new inner dims + positions for sensor_pods.py
    bat = out["battery_box"]; imu = out["imu_pod"]
    dims = dict(main_box=[BASE["main_box"][0], CHOSEN["H"], CHOSEN["D"]], battery_box=[BASE["battery_box"][0], BASE["battery_box"][1], math.ceil(bat["depth_needed_mm"] + 1.0)],
                afe_pod_R=list(BASE["afe_pod_R"]), imu_pod=[round(math.ceil(imu["need"][0] * 10) / 10 + 0.1, 1), round(math.ceil(imu["need"][1] * 10) / 10 + 0.1, 1), math.ceil(imu["need"][2] * 2) / 2],
                estop_turret_mm=TURRET)
    if ch["ok"]:
        es = [p for p in ch["plan"] if any("E-stop" in n for n in p["items"])][0]; xt = [p for p in ch["plan"] if any("XT60" in n for n in p["items"])][0]
        dims["estop_xy_local"] = [es["cx"], es["cy"]]; dims["xt60_xy_local"] = [xt["cx"], xt["cy"]]
    out["thermal_main_box"] = thermal_main(dims["main_box"])
    out["pod_dims"] = dims
    out["budgets"] = dict(rev_f=BUDGET)
    new_budget = dict(afe_pod_R=round(out["afe_pod"]["mass_total_g"] + 3.0), main_box=round(out["main_box"]["mass_total_g"] + 10.0), battery_box=round(bat["mass_total_g"] + 5.0), imu_pod=6.0)
    out["budgets"]["rev_f2_contents_g"] = new_budget; dims["contents_mass_g"] = new_budget
    if write:
        json.dump(out, open(os.path.join(OUT, "electronics_fit.json"), "w"), indent=1, default=str)
        json.dump(dims, open(os.path.join(ROOT, "stl", "pods", "pod_dims.json"), "w"), indent=1)
    return out

if __name__ == "__main__":
    full = "--quick" not in sys.argv
    out = run(full=full)
    print(json.dumps({k: v for k, v in out.items() if k not in ("main_box",)}, indent=1, default=str)[:5000])
    print(json.dumps({k: v for k, v in out["main_box"].items() if k != "chosen"}, indent=1, default=str))
    print("CHOSEN", json.dumps(out["main_box"]["chosen"], indent=1, default=str)[:3500])
