import uuid
def U(): return str(uuid.uuid4())
def hdr(name, descr):
    return f'''(footprint "{name}" (version 20221018) (generator pcbnew)
  (layer "F.Cu")
  (descr "{descr}")
  (attr through_hole)
'''
def txt(kind, val, x, y, layer, hide=False):
    h = " hide" if hide else ""
    return f'  (fp_text {kind} "{val}" (at {x} {y}) (layer "{layer}"){h}\n    (effects (font (size 1 1) (thickness 0.15))) (tstamp {U()}))\n'
def line(x1,y1,x2,y2,layer,w):
    return f'  (fp_line (start {x1} {y1}) (end {x2} {y2}) (stroke (width {w}) (type solid)) (layer "{layer}") (tstamp {U()}))\n'
def rect(x1,y1,x2,y2,layer,w):
    return f'  (fp_rect (start {x1} {y1}) (end {x2} {y2}) (stroke (width {w}) (type solid)) (fill none) (layer "{layer}") (tstamp {U()}))\n'
def circ(x,y,r,layer,w):
    return f'  (fp_circle (center {x} {y}) (end {x+r} {y}) (stroke (width {w}) (type solid)) (fill none) (layer "{layer}") (tstamp {U()}))\n'

# ---- E07-400M10S: native orientation, origin at module centre, 14 x 20 body
s = hdr("E07-400M10S", "Ebyte E07-400M10S CC1101 module, 14x20mm, 22 castellated pads 1.27mm pitch, hand-solder pads (1.1mm outside body)").replace("(attr through_hole)","(attr smd)")
s += txt("reference","REF**",0,-11.2,"F.SilkS")
s += txt("value","E07-400M10S",0,0,"F.Fab")
L = {}
for k,p in enumerate(range(12,20)): L[p]=-8.0+1.27*k
L[20],L[21],L[22]=6.46,7.73,9.0
R = {}
for k,p in enumerate(range(11,3,-1)): R[p]=-8.0+1.27*k
R[3],R[2],R[1]=6.46,7.73,9.0
pads=[]
for p,y in L.items(): pads.append((p,-7.1,round(y,3)))
for p,y in R.items(): pads.append((p,7.1,round(y,3)))
for p,x,y in sorted(pads):
    s += f'  (pad "{p}" smd rect (at {x} {y}) (size 2 0.8) (layers "F.Cu" "F.Paste" "F.Mask") (tstamp {U()}))\n'
s += rect(-7,-10,7,10,"F.Fab",0.1)
s += rect(-8.35,-10.25,8.35,10.25,"F.CrtYd",0.05)
s += line(-7.1,-10.1,7.1,-10.1,"F.SilkS",0.12)
s += line(-7.1,10.1,7.1,10.1,"F.SilkS",0.12)
s += circ(4.8,8.2,0.2,"F.SilkS",0.3)   # pin 1 dot (beyond the pin-1 end of the module)
s += txt("user","ANT",-4.6,7.73,"F.Fab")
s += ")\n"
open("AdventRX.pretty/E07-400M10S.kicad_mod","w").write(s)

# ---- Wire pair with strain-relief holes
s = hdr("Wire_StrainRelief_1x02_P2.6mm","Two solder holes for flying leads, each with an unplated strain-relief hole 2.4mm away: thread the wire up through the relief hole, back down through the plated hole, solder underneath")
s += txt("reference","REF**",1.3,-1.8,"F.SilkS")
s += txt("value","WIRE",1.3,4.2,"F.Fab")
for n,x in ((1,0),(2,2.6)):
    shape = "rect" if n==1 else "circle"
    s += f'  (pad "{n}" thru_hole {shape} (at {x} 0) (size 1.8 1.8) (drill 1.0) (layers "*.Cu" "*.Mask") (tstamp {U()}))\n'
    s += f'  (pad "" np_thru_hole circle (at {x} 2.4) (size 1.6 1.6) (drill 1.6) (layers "*.Cu" "*.Mask") (tstamp {U()}))\n'
    s += line(x,1.15,x,1.45,"F.SilkS",0.15)
s += rect(-1.15,-1.15,3.75,3.45,"F.CrtYd",0.05)
s += ")\n"
open("AdventRX.pretty/Wire_StrainRelief_1x02_P2.6mm.kicad_mod","w").write(s)

# ---- Helical antenna placeholder: feed hole + two tie holes, coil axis along +X
s = hdr("ANT_Helical_433_Placeholder","PLACEHOLDER for a 433MHz helical (spring) antenna laid flat: plated feed hole at origin, coil along +X, two unplated tie holes. Move holes to suit the chosen part.")
s += txt("reference","REF**",0,-3.6,"F.SilkS",hide=True)
s += txt("value","Helical 433MHz",11,0,"F.Fab")
s += f'  (pad "1" thru_hole circle (at 0 0) (size 2 2) (drill 1.0) (layers "*.Cu" "*.Mask") (tstamp {U()}))\n'
for y in (-3.2,3.2):
    s += f'  (pad "" np_thru_hole circle (at 12 {y}) (size 1.2 1.2) (drill 1.2) (layers "*.Cu" "*.Mask") (tstamp {U()}))\n'
s += rect(1.5,-2.5,22,2.5,"F.Fab",0.1)          # nominal coil envelope, 5 mm dia
s += line(1.5,-2.5,22,-2.5,"F.SilkS",0.12)
s += line(1.5,2.5,22,2.5,"F.SilkS",0.12)
s += rect(-1.25,-4.05,22.25,4.05,"F.CrtYd",0.05)
s += ")\n"
open("AdventRX.pretty/ANT_Helical_433_Placeholder.kicad_mod","w").write(s)
print("ok")
