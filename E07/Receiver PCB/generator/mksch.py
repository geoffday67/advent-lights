import uuid, re, json, math
def U(): return str(uuid.uuid4())
PROJ="advent_rx"
SYM="/usr/share/kicad/symbols"
def extract(path,name):
    t=open(path).read(); i=t.index(f'(symbol "{name}"'); d=0
    for j in range(i,len(t)):
        if t[j]=='(':d+=1
        elif t[j]==')':
            d-=1
            if d==0: return t[i:j+1]

def pin(num,name,x,y,ang,typ="bidirectional",length=2.54,hide=False):
    h=" hide" if hide else ""
    return f'''      (pin {typ} line (at {x} {y} {ang}) (length {length}){h}
        (name "{name}" (effects (font (size 1.27 1.27))))
        (number "{num}" (effects (font (size 1.27 1.27))))
      )\n'''
def custom(name,ref,value,fp,desc,body,pins,refy,valy):
    s=f'''  (symbol "{name}" (in_bom yes) (on_board yes)
    (property "Reference" "{ref}" (at 0 {refy} 0) (effects (font (size 1.27 1.27))))
    (property "Value" "{value}" (at 0 {valy} 0) (effects (font (size 1.27 1.27))))
    (property "Footprint" "{fp}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
    (property "Datasheet" "" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
    (property "ki_description" "{desc}" (at 0 0 0) (effects (font (size 1.27 1.27)) hide))
    (symbol "{name}_0_1"
      (rectangle (start {body[0]} {body[1]}) (end {body[2]} {body[3]}) (stroke (width 0.254) (type default)) (fill (type background)))
    )
    (symbol "{name}_1_1"
{"".join(pins)}    )
  )'''
    return s

# PIC16F15225 (SOIC-14)
pp=[pin(1,"VDD",0,15.24,270,"power_in"), pin(14,"VSS",0,-15.24,90,"power_in")]
for k,(n,nm) in enumerate([(13,"RA0/ICSPDAT"),(12,"RA1/ICSPCLK"),(11,"RA2"),(4,"RA3/~{MCLR}"),(3,"RA4"),(2,"RA5")]):
    pp.append(pin(n,nm,-15.24,7.62-2.54*k,0,"input" if n==4 else "bidirectional"))
for k,(n,nm) in enumerate([(10,"RC0"),(9,"RC1"),(8,"RC2"),(7,"RC3"),(6,"RC4"),(5,"RC5")]):
    pp.append(pin(n,nm,15.24,7.62-2.54*k,180))
PIC=custom("PIC16F15225","U","PIC16F15225-I/SL","Package_SO:SOIC-14_3.9x8.7mm_P1.27mm",
  "8-bit PIC MCU, 14 pins, SOIC-14",(-12.7,12.7,12.7,-12.7),pp,13.97,-13.97)

ep=[pin(9,"VCC",0,15.24,270,"power_in")]
for k,(n,nm) in enumerate([(19,"CSN"),(18,"SCK"),(17,"MOSI"),(16,"MISO/GDO1"),(15,"GDO0"),(14,"GDO2")]):
    ep.append(pin(n,nm,-15.24,7.62-2.54*k,0,"input" if n in (19,18,17) else "output"))
for k,n in enumerate([1,2,3,4,5,11,12,20,22]):
    ep.append(pin(n,"GND",-10.16+2.54*k,-15.24,90,"power_in"))
ep.append(pin(21,"ANT",15.24,7.62,180,"passive"))
for k,n in enumerate([6,7,8,10,13]):
    ep.append(pin(n,"NC",15.24,2.54-2.54*k,180,"no_connect"))
E07=custom("E07-400M10S","U","E07-400M10S","AdventRX:E07-400M10S",
  "Ebyte CC1101 433MHz module, 14x20mm castellated",(-12.7,12.7,12.7,-12.7),ep,13.97,-13.97)

open("AdventRX.kicad_sym","w").write("(kicad_symbol_lib (version 20220914) (generator kicad_symbol_editor)\n"+PIC+"\n"+E07+"\n)\n")

libsyms={}
def addlib(libid,text):
    nm=libid.split(":")[1]
    libsyms[libid]=text.replace(f'(symbol "{nm}"',f'(symbol "{libid}"',1)
for lib,n in [("Device","R"),("Device","C"),("Device","Antenna"),("Connector_Generic","Conn_01x02"),
              ("Connector_Generic","Conn_01x05"),("Connector","TestPoint"),("power","GND"),("power","VDD"),("power","PWR_FLAG")]:
    addlib(f"{lib}:{n}",extract(f"{SYM}/{lib}.kicad_sym",n))
addlib("AdventRX:PIC16F15225",PIC.strip())
addlib("AdventRX:E07-400M10S",E07.strip())

# pin table per lib symbol: num -> (x,y,ang)
pins={}
for lid,t in libsyms.items():
    pins[lid]={m.group(4):(float(m.group(1)),float(m.group(2)),float(m.group(3)))
               for m in re.finditer(r'\(pin \w+ \w+ \(at ([-\d.]+) ([-\d.]+) ([-\d.]+)\).*?\(number "([^"]+)"',t,re.S)}

ROOT=U()
items=[]; symuuid={}
pwrn=[0]
def place(lid,ref,val,x,y,fp="",rot=0,extra=None,rv=None):
    su=U(); symuuid[ref]=su
    (rx,ry),(vx,vy)=rv if rv else ((3.0,-1.5),(3.0,1.5))
    pr=f'''  (symbol (lib_id "{lid}") (at {x} {y} {rot}) (unit 1) (in_bom {"no" if ref.startswith("#") else "yes"}) (on_board {"no" if ref.startswith("#") else "yes"}) (dnp no)
    (uuid {su})
    (property "Reference" "{ref}" (at {x+rx} {y+ry} 0) (effects (font (size 1.27 1.27)) (justify left){" hide" if ref.startswith("#") else ""}))
    (property "Value" "{val}" (at {x+vx} {y+vy} 0) (effects (font (size 1.27 1.27)) (justify left)))
    (property "Footprint" "{fp}" (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))
    (property "Datasheet" "" (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))
'''
    for k,v in (extra or {}).items():
        pr+=f'    (property "{k}" "{v}" (at {x} {y} 0) (effects (font (size 1.27 1.27)) hide))\n'
    for n in pins[lid]: pr+=f'    (pin "{n}" (uuid {U()}))\n'
    pr+=f'    (instances (project "{PROJ}" (path "/{ROOT}" (reference "{ref}") (unit 1))))\n  )'
    items.append(pr)
    return su
def pinpos(lid,x,y,num,rot=0):
    px,py,a=pins[lid][num]
    r=math.radians(rot)
    rx=px*math.cos(r)-py*math.sin(r); ry=px*math.sin(r)+py*math.cos(r)
    a=(a+rot)%360
    ox,oy=-math.cos(math.radians(a)), math.sin(math.radians(a))   # outward, schematic coords
    return (round(x+rx,2), round(y-ry,2), round(ox), round(oy))
def wire(x1,y1,x2,y2):
    items.append(f'  (wire (pts (xy {x1} {y1}) (xy {x2} {y2})) (stroke (width 0) (type default)) (uuid {U()}))')
def label(name,x,y,ox,oy):
    ang={(1,0):0,(-1,0):180,(0,-1):90,(0,1):270}[(ox,oy)]
    just="left bottom" if ang in (0,90) else "right bottom"
    items.append(f'  (label "{name}" (at {x} {y} {ang}) (fields_autoplaced) (effects (font (size 1.27 1.27)) (justify {just})) (uuid {U()}))')
def power(kind,x,y,o=None):
    pwrn[0]+=1
    rot=0
    if o is not None:
        rot={"VDD":{(0,-1):0,(-1,0):90,(1,0):270,(0,1):180},"GND":{(0,1):0,(1,0):90,(-1,0):270,(0,-1):180}}[kind][o]
    place(f"power:{kind}",f"#PWR{pwrn[0]:02d}",kind,x,y,rot=rot)
def nc(x,y):
    items.append(f'  (no_connect (at {x} {y}) (uuid {U()}))')
def conn(lid,x,y,num,net,rot=0,stub=2.54):
    px,py,ox,oy=pinpos(lid,x,y,num,rot)
    if net is None: nc(px,py); return
    ex,ey=round(px+ox*stub,2),round(py+oy*stub,2)
    wire(px,py,ex,ey)
    if net in ("GND","VDD"):
        if net=="GND" and oy==-1 or net=="VDD" and oy==1:   # stub points the wrong way for the symbol: extend sideways
            pass
        power(net,ex,ey,(ox,oy))
    else: label(net,ex,ey,ox,oy)

nets={}
def part(lid,ref,val,x,y,fp,pinnets,rot=0,extra=None,rv=None,skip=()):
    place(lid,ref,val,x,y,fp,rot,extra,rv)
    for n,net in pinnets.items():
        if n not in skip: conn(lid,x,y,str(n),net,rot)
        nets.setdefault(net,[]).append(f"{ref}.{n}")

PICn={1:"VDD",14:"GND",13:"PGD",12:"PGC",11:None,4:"MCLR",3:"MISO",2:"GDO2",10:None,9:"UART_TX",8:"LED_DRV",7:"CSN",6:"SCK",5:"MOSI"}
part("AdventRX:PIC16F15225","U1","PIC16F15225-I/SL",101.6,96.52,"Package_SO:SOIC-14_3.9x8.7mm_P1.27mm",PICn,rv=((-12.7,-17.0),(3.81,17.5)))
E07n={9:"VDD",19:"CSN",18:"SCK",17:"MOSI",16:"MISO",15:None,14:"GDO2",21:"ANT",6:None,7:None,8:None,10:None,13:None}
for g in (1,2,3,4,5,11,12,20,22): E07n[g]="GND"
part("AdventRX:E07-400M10S","U2","E07-400M10S",182.88,96.52,"AdventRX:E07-400M10S",E07n,extra={"LCSC":"C2965513"},rv=((-12.7,-17.0),(-12.7,23.6)),skip=(1,2,3,4,5,11,12,20,22))
gx=[pinpos("AdventRX:E07-400M10S",182.88,96.52,str(n))[:2] for n in (1,2,3,4,5,11,12,20,22)]
for (px,py) in gx: wire(px,py,px,py+2.54)
for i in range(len(gx)-1): wire(gx[i][0],gx[i][1]+2.54,gx[i+1][0],gx[i+1][1]+2.54)
for (px,py) in gx[1:-1]: items.append(f'  (junction (at {px} {py+2.54}) (diameter 0) (color 0 0 0 0) (uuid {U()}))')
mx,my=gx[4][0],gx[4][1]+2.54
wire(mx,my,mx,my+2.54); power("GND",mx,my+2.54)
part("Device:Antenna","AE1","Helical 433MHz (placeholder)",215.9,83.82,"AdventRX:ANT_Helical_433_Placeholder",{1:"ANT"})

R="Resistor_SMD:R_0805_2012Metric_Pad1.20x1.40mm_HandSolder"
Cf="Capacitor_SMD:C_0805_2012Metric_Pad1.18x1.45mm_HandSolder"
part("Device:C","C1","100nF",76.2,63.5,Cf,{1:"VDD",2:"GND"})
part("Device:C","C2","10uF",160.02,63.5,Cf,{1:"VDD",2:"GND"})
part("Device:C","C3","100nF",172.72,63.5,Cf,{1:"VDD",2:"GND"})
part("Device:R","R1","10k",48.26,63.5,R,{1:"VDD",2:"MCLR"})
part("Connector_Generic:Conn_01x05","J1","ICSP (Snap pins 1-5)",43.18,96.52,
     "Connector_PinHeader_2.54mm:PinHeader_1x05_P2.54mm_Vertical",{1:"MCLR",2:"VDD",3:"GND",4:"PGD",5:"PGC"})
part("Connector_Generic:Conn_01x02","J2","Battery 2xAAA (+,-)",43.18,137.16,"AdventRX:Wire_StrainRelief_1x02_P2.6mm",{1:"VDD",2:"GND"})
part("Connector_Generic:Conn_01x02","J3","LED (A,K)",157.48,137.16,"AdventRX:Wire_StrainRelief_1x02_P2.6mm",{1:"LED_A",2:"GND"})
part("Device:R","R3","TBD (LED)",132.08,137.16,R,{1:"LED_DRV",2:"LED_A"})
part("Connector:TestPoint","TP1","UART TX",147.32,111.76,"TestPoint:TestPoint_THTPad_D1.5mm_Drill0.7mm",{1:"UART_TX"})
# power flags
for kind,x in (("VDD",30.48),("GND",30.48)):
    pass
place("power:PWR_FLAG","#FLG01","PWR_FLAG",25.4,137.16); wire(25.4,137.16,25.4,139.7); power("VDD",25.4,139.7)
place("power:PWR_FLAG","#FLG02","PWR_FLAG",25.4,149.86); wire(25.4,149.86,25.4,152.4); power("GND",25.4,152.4)

notes=[("Receiver for one advent tree: PIC16F15225 + Ebyte E07-400M10S (CC1101), 2xAAA, WOR polling",25.4,25.4),
       ("AE1 footprint and R3 value are placeholders until the helical antenna and LED are chosen",25.4,30.48),
       ("J2/J3: flying leads soldered into plated holes, threaded through unplated strain-relief holes",25.4,35.56)]
for t,x,y in notes:
    items.append(f'  (text "{t}" (at {x} {y} 0) (effects (font (size 1.778 1.778)) (justify left bottom)) (uuid {U()}))')

sch=f'''(kicad_sch (version 20230121) (generator eeschema)
  (uuid {ROOT})
  (paper "A4")
  (title_block (title "Advent tree RF receiver") (date "2026-11-01") (rev "1") (company "Geoff Day"))
  (lib_symbols
{chr(10).join(libsyms.values())}
  )
{chr(10).join(items)}
  (sheet_instances (path "/" (page "1")))
)
'''
open(f"{PROJ}.kicad_sch","w").write(sch)
json.dump({"root":ROOT,"sym":symuuid,"nets":nets},open("schinfo.json","w"),indent=1)
open("sym-lib-table","w").write('(sym_lib_table\n  (lib (name "AdventRX")(type "KiCad")(uri "${KIPRJMOD}/AdventRX.kicad_sym")(options "")(descr "Advent receiver parts"))\n)\n')
open("fp-lib-table","w").write('(fp_lib_table\n  (lib (name "AdventRX")(type "KiCad")(uri "${KIPRJMOD}/AdventRX.pretty")(options "")(descr "Advent receiver footprints"))\n)\n')
for k,v in nets.items(): print(k,v)
