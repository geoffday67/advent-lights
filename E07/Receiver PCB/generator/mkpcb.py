import pcbnew, json, math, sys, os
from pcbnew import FromMM as MM, VECTOR2I
from router import Router, Shape, simplify, G
import numpy as np

PRJ = "/home/claude/rx"
OUT = f"{PRJ}/advent_rx.kicad_pcb"
W, H = 38.0, 26.0
KEEP_Y = 9.0                     # antenna strip: y 0..KEEP_Y
info = json.load(open(f"{PRJ}/schinfo.json"))
KL = "/usr/share/kicad/footprints"

def V(x, y): return VECTOR2I(MM(float(x)), MM(float(y)))

b = pcbnew.NewBoard(OUT)
ds = b.GetDesignSettings()
ds.m_MinClearance = MM(0.15); ds.m_TrackMinWidth = MM(0.15)
ds.m_ViasMinSize = MM(0.5); ds.m_MinThroughDrill = MM(0.3)
ds.m_CopperEdgeClearance = MM(0.3); ds.m_HoleClearance = MM(0.25); ds.m_HoleToHoleMin = MM(0.25); ds.m_MinResolvedSpokes = 1
dn = ds.m_NetSettings.m_DefaultNetClass
dn.SetClearance(MM(0.2)); dn.SetTrackWidth(MM(0.25)); dn.SetViaDiameter(MM(0.6)); dn.SetViaDrill(MM(0.3))

# ---------- outline
for (x1, y1, x2, y2) in ((0, 0, W, 0), (W, 0, W, H), (W, H, 0, H), (0, H, 0, 0)):
    s = pcbnew.PCB_SHAPE(b); s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(V(x1, y1)); s.SetEnd(V(x2, y2)); s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(MM(0.1)); b.Add(s)

# ---------- nets
netnames = sorted({n for n in info["nets"] if n not in ("None", "null")})
NET = {}
for n in netnames:
    ni = pcbnew.NETINFO_ITEM(b, n if n in ("GND", "VDD") else "/" + n); b.Add(ni); NET[n] = ni
pinnet = {}
for n, pins in info["nets"].items():
    for p in pins: pinnet[p] = None if n in ("None", "null") else n

# ---------- placement: ref -> (lib, fpname, x, y, rot, value)
R = (f"{KL}/Resistor_SMD.pretty", "R_0805_2012Metric_Pad1.20x1.40mm_HandSolder")
C = (f"{KL}/Capacitor_SMD.pretty", "C_0805_2012Metric_Pad1.18x1.45mm_HandSolder")
A = f"{PRJ}/AdventRX.pretty"
P = json.load(open(f"{PRJ}/place.json"))
LIB = {"R": R, "C": C,
       "U1": (f"{KL}/Package_SO.pretty", "SOIC-14_3.9x8.7mm_P1.27mm"),
       "U2": (A, "E07-400M10S"), "AE1": (A, "ANT_Helical_433_Placeholder"),
       "J1": (f"{KL}/Connector_PinHeader_2.54mm.pretty", "PinHeader_1x05_P2.54mm_Vertical"),
       "J2": (A, "Wire_StrainRelief_1x02_P2.6mm"), "J3": (A, "Wire_StrainRelief_1x02_P2.6mm"),
       "TP1": (f"{KL}/TestPoint.pretty", "TestPoint_THTPad_D1.5mm_Drill0.7mm")}
VAL = {"U1": "PIC16F15225-I/SL", "U2": "E07-400M10S", "AE1": "Helical 433MHz", "J1": "ICSP",
       "J2": "BATT", "J3": "LED", "TP1": "TX", "R1": "10k", "R3": "TBD", "C1": "100nF", "C2": "10uF", "C3": "100nF"}
FP = {}
for ref, (x, y, rot) in P["parts"].items():
    lib, name = LIB.get(ref, LIB.get(ref[0]))
    fp = pcbnew.FootprintLoad(lib, name)
    fp.SetReference(ref); fp.SetValue(VAL[ref])
    fp.SetFPID(pcbnew.LIB_ID("AdventRX" if lib == A else os.path.basename(lib)[:-7], name))
    fp.SetPath(pcbnew.KIID_PATH("/" + info["sym"][ref]))
    b.Add(fp)
    fp.SetPosition(V(x, y)); fp.SetOrientationDegrees(rot)
    for pad in fp.Pads():
        n = pinnet.get(f"{ref}.{pad.GetNumber()}")
        if n: pad.SetNet(NET[n])
    FP[ref] = fp
    if ref in P.get("refpos", {}):
        rx, ry, rs = P["refpos"][ref]
        fp.Reference().SetPosition(V(rx, ry)); fp.Reference().SetTextAngleDegrees(-rot)
        fp.Reference().SetTextSize(V(rs, rs)); fp.Reference().SetTextThickness(MM(rs * 0.15))
    fp.Value().SetVisible(False)
    if ref in P.get("hideref", []): fp.Reference().SetVisible(False)

# ---------- router setup
rt = Router(W, H)
def netof(item):
    n = item.GetNetname()
    return n.lstrip("/") if n else None
TOP, BOT = 0, 1
for fp in FP.values():
    for pad in fp.Pads():
        bb = pad.GetBoundingBox()
        c = pad.GetPosition(); cx, cy = pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)
        hw, hh = pcbnew.ToMM(bb.GetWidth()) / 2, pcbnew.ToMM(bb.GetHeight()) / 2
        layers = [L for L, lid in ((TOP, pcbnew.F_Cu), (BOT, pcbnew.B_Cu)) if pad.IsOnLayer(lid)]
        n = netof(pad)
        if pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
            rt.add(Shape('circ', (cx, cy, hw + 0.05), [TOP, BOT], "#hole")); continue
        if pad.GetShape() == pcbnew.PAD_SHAPE_CIRCLE or (pad.GetShape() == pcbnew.PAD_SHAPE_OVAL and abs(hw - hh) < 1e-3):
            rt.add(Shape('circ', (cx, cy, hw), layers, n))
        else:
            rt.add(Shape('rect', (cx, cy, hw, hh), layers, n))
        if not n:   # unconnected pins: give them a private net name so nothing connects to them
            rt.shapes[-1].net = f"#nc{id(pad)}"

# keep-outs
X, Y = rt.X, rt.Y
rt.noVia |= (Y < KEEP_Y + 0.35)                 # no vias in the antenna strip
ant_strip = (Y < KEEP_Y + P.get("strip_margin", 0.0))
mx, my = P["parts"]["U2"][0], P["parts"]["U2"][1]
mod = (X > mx - 10) & (X < mx + 10) & (Y > my - 7) & (Y < my + 7)
rt.noTop |= mod                                  # keep top copper under the module as ground
sx, sy = P["parts"]["U1"][0], P["parts"]["U1"][1]
for n in netnames:
    if n != "ANT": rt.noTrack[n] = ant_strip.copy()
for n in P.get("bottom_only_left_of", {}).get("nets", []):
    rt.noTopNet[n] = (X < P["bottom_only_left_of"]["x"])

tracks = []  # (L, x1,y1,x2,y2,w,net)
vias = []    # (x,y,net)
def commit(net, items, w):
    for it in items:
        if it[0] == 'seg':
            _, L, (x1, y1), (x2, y2) = it
            tracks.append((L, x1, y1, x2, y2, w, net)); rt.add(Shape('seg', (x1, y1, x2, y2, w / 2), [L], net))
        else:
            x, y = it[1]; vias.append((x, y, net)); rt.add(Shape('circ', (x, y, 0.3), [TOP, BOT], net))

def cells_of(shape_list, L):
    m = np.zeros_like(rt.noTop)
    for s in shape_list:
        if L in s.layers: m |= (s.dist(X, Y) <= 0)
    return m

def route_net(net, w, order=None, via_cost=40, allow_bottom=True):
    pads = [s for s in rt.shapes if s.net == net and s.kind in ('rect', 'circ') and s.p[2] > 0.31]
    if order: pads = [pads[i] for i in order]
    connected = [pads[0]]; rest = pads[1:]
    while rest:
        # nearest unconnected pad to the connected set
        def dmin(p): return min(math.hypot(p.p[0] - c.p[0], p.p[1] - c.p[1]) for c in connected)
        rest.sort(key=dmin); tgt = rest.pop(0)
        own = [s for s in rt.shapes if s.net == net and s not in rest and s is not tgt]
        src = []
        for L in (TOP, BOT):
            jj, ii = np.nonzero(cells_of(own, L)); src += [(L, j, i) for j, i in zip(jj, ii)]
        tm = [cells_of([tgt], TOP), cells_of([tgt], BOT)]
        path = rt.route(net, src, tm, w / 2, via_cost=via_cost, allow_bottom=allow_bottom)
        if path is None:
            print("FAILED", net, "to pad at", tgt.p[:2]); return False
        items = simplify(path)
        L0, j0, i0 = path[0]; L1, j1, i1 = path[-1]
        segs = [it for it in items if it[0] == 'seg']
        # pull the ends onto pad centres so the track is properly connected
        if L1 in tgt.layers:
            items.append(('seg', L1, (i1 * G, j1 * G), (tgt.p[0], tgt.p[1])))
        for s0 in own:
            if s0.kind in ('rect', 'circ') and L0 in s0.layers and s0.dist(np.array(i0 * G), np.array(j0 * G)) <= 0:
                items.insert(0, ('seg', L0, (s0.p[0], s0.p[1]), (i0 * G, j0 * G))); break
        items = [it for it in items if it[0] == 'via' or math.hypot(it[2][0]-it[3][0], it[2][1]-it[3][1]) > 1e-6]
        commit(net, items, w); connected.append(tgt)
    return True

# ---------- fixed route: antenna feed, straight up from pin 21 to the feed hole
p21 = [p for p in FP["U2"].Pads() if p.GetNumber() == "21"][0].GetPosition()
af = FP["AE1"].GetPosition()
tracks.append((TOP, pcbnew.ToMM(p21.x), pcbnew.ToMM(p21.y), pcbnew.ToMM(af.x), pcbnew.ToMM(af.y), 0.5, "ANT"))
rt.add(Shape('seg', (pcbnew.ToMM(p21.x), pcbnew.ToMM(p21.y), pcbnew.ToMM(af.x), pcbnew.ToMM(af.y), 0.25), [TOP], "ANT"))

# ---------- pre-placed stitching / GND vias (x,y)
for (x, y) in P.get("gndvias", []):
    vias.append((x, y, "GND")); rt.add(Shape('circ', (x, y, 0.3), [TOP, BOT], "GND"))
for (L, x1, y1, x2, y2) in P.get("gndtracks", []):
    tracks.append((L, x1, y1, x2, y2, 0.4, "GND")); rt.add(Shape('seg', (x1, y1, x2, y2, 0.2), [L], "GND"))
for net, x, y1, y2, w in P.get("stubs", []):          # pad -> via stubs (top), via at (x, y2)
    tracks.append((TOP, x, y1, x, y2, w, net)); rt.add(Shape('seg', (x, y1, x, y2, w / 2), [TOP], net))
    vias.append((x, y2, net)); rt.add(Shape('circ', (x, y2, 0.3), [TOP, BOT], net))

ok = True
for net, w in P["order"]:
    print("routing", net, flush=True)
    ok &= route_net(net, w)

# ---------- GND stitching vias on a grid wherever there is room
st = P.get("stitch", {"pitch": 2.0})
vb = rt.masks("GND", 0.125)[1]
def via_free(x, y):
    i, j = int(round(x / G)), int(round(y / G))
    r = int(0.05 / G) + 1
    return not vb[max(j-r,0):j+r+1, max(i-r,0):i+r+1].any()
xs = np.arange(1.2, W - 1.0, st["pitch"]); ys = np.arange(KEEP_Y + 1.2, H - 0.9, st["pitch"])
nst = 0
for y in ys:
    for x in xs:
        if any(x1 <= x <= x2 and y1 <= y <= y2 for (x1, y1, x2, y2) in st.get("avoid", [])): continue
        near_pad = any(sh.kind in ('rect', 'circ') and sh.p[2] > 0.31 and sh.dist(np.array(x), np.array(y)) < 0.55
                       for sh in rt.shapes)
        if not near_pad and via_free(x, y) and all(math.hypot(x - vx, y - vy) > 1.2 for (vx, vy, _) in vias):
            vias.append((float(x), float(y), "GND")); rt.add(Shape('circ', (x, y, 0.3), [TOP, BOT], "GND")); nst += 1
            vb = rt.masks("GND", 0.125)[1]
print("stitching vias:", nst)

# ---------- write tracks / vias
for (L, x1, y1, x2, y2, w, net) in tracks:
    t = pcbnew.PCB_TRACK(b); t.SetStart(V(x1, y1)); t.SetEnd(V(x2, y2)); t.SetWidth(MM(w))
    t.SetLayer(pcbnew.F_Cu if L == TOP else pcbnew.B_Cu); t.SetNet(NET[net]); b.Add(t)
for (x, y, net) in vias:
    v = pcbnew.PCB_VIA(b); v.SetPosition(V(x, y)); v.SetWidth(MM(0.6)); v.SetDrill(MM(0.3)); v.SetNet(NET[net]); b.Add(v)

# ---------- zones
def poly_zone(pts, layer=None, net=None, rule=False):
    z = pcbnew.ZONE(b)
    if rule:
        z.SetIsRuleArea(True); z.SetDoNotAllowCopperPour(True); z.SetDoNotAllowVias(True)
        z.SetDoNotAllowTracks(False); z.SetDoNotAllowPads(False); z.SetDoNotAllowFootprints(False)
        ls = pcbnew.LSET(); ls.AddLayer(pcbnew.F_Cu); ls.AddLayer(pcbnew.B_Cu); z.SetLayerSet(ls)
        z.SetZoneName("Antenna keep-out")
    else:
        z.SetLayer(layer); z.SetNet(net); z.SetLocalClearance(MM(0.3)); z.SetMinThickness(MM(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(MM(0.4)); z.SetThermalReliefSpokeWidth(MM(0.4))
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    o = z.Outline(); o.NewOutline()
    for x, y in pts: o.Append(MM(x), MM(y))
    b.Add(z); return z
poly_zone([(0, 0), (W, 0), (W, KEEP_Y), (0, KEEP_Y)], rule=True)
for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
    poly_zone([(0, KEEP_Y), (W, KEEP_Y), (W, H), (0, H)], layer, NET["GND"])

# ---------- silkscreen text
def silk(t, x, y, size=0.8, layer=pcbnew.F_SilkS, just=None):
    tx = pcbnew.PCB_TEXT(b); tx.SetText(t); tx.SetPosition(V(x, y)); tx.SetLayer(layer)
    tx.SetTextSize(V(size, size)); tx.SetTextThickness(MM(size * 0.15)); b.Add(tx)
    if layer == pcbnew.B_SilkS: tx.SetMirrored(True)
    return tx
for t, x, y, sz in P.get("silk", []): silk(t, x, y, sz)
for x1, y1, x2, y2 in P.get("silkrect", []):
    r = pcbnew.PCB_SHAPE(b); r.SetShape(pcbnew.SHAPE_T_RECT); r.SetStart(V(x1, y1)); r.SetEnd(V(x2, y2))
    r.SetLayer(pcbnew.F_SilkS); r.SetWidth(MM(0.15)); b.Add(r)
for t, x, y, sz in P.get("bsilk", []): silk(t, x, y, sz, pcbnew.B_SilkS)

filler = pcbnew.ZONE_FILLER(b); filler.Fill(b.Zones())
pcbnew.SaveBoard(OUT, b)
print("ROUTED_OK" if ok else "ROUTING_INCOMPLETE")

# ---------- debug plot
if "--plot" in sys.argv:
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle, Circle
    fig, ax = plt.subplots(figsize=(15, 10.5))
    for s in rt.shapes:
        col = "#c0392b" if s.layers == [TOP] else ("#2e86c1" if s.layers == [BOT] else "#7d3c98")
        if s.kind == 'rect':
            ax.add_patch(Rectangle((s.p[0]-s.p[2], s.p[1]-s.p[3]), 2*s.p[2], 2*s.p[3], color=col, alpha=.5))
        elif s.kind == 'circ':
            ax.add_patch(Circle((s.p[0], s.p[1]), s.p[2], color=col if s.net != "#hole" else "k", alpha=.5))
    for (L, x1, y1, x2, y2, w, net) in tracks:
        ax.plot([x1, x2], [y1, y2], color="#e74c3c" if L == TOP else "#3498db", lw=w*28, solid_capstyle="round", alpha=.8)
        ax.text((x1+x2)/2, (y1+y2)/2, net, fontsize=5)
    for (x, y, n) in vias: ax.add_patch(Circle((x, y), .3, color="#27ae60"))
    ax.add_patch(Rectangle((0, 0), W, KEEP_Y, fill=False, hatch="//", ec="#aaa"))
    for ref, fp in FP.items():
        p = fp.GetPosition(); ax.text(pcbnew.ToMM(p.x), pcbnew.ToMM(p.y), ref, fontsize=9, weight="bold")
    ax.set_xlim(-0.5, W+.5); ax.set_ylim(H+.5, -.5); ax.set_aspect(1); ax.grid(alpha=.2)
    ax.set_xticks(range(0, 39, 2)); ax.set_yticks(range(0, 27, 2))
    plt.savefig(f"{PRJ}/out/route.png", dpi=90, bbox_inches="tight")
