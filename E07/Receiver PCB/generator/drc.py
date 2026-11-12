import pcbnew,sys,re,collections
b=pcbnew.LoadBoard("/home/claude/rx/advent_rx.kicad_pcb")
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
rep="/home/claude/rx/out/drc.txt"
pcbnew.WriteDRCReport(b,rep,pcbnew.EDA_UNITS_MILLIMETRES,True)
t=open(rep).read()
print(re.findall(r"\*\* Found \d+ [^\n]*",t))
c=collections.Counter(re.findall(r"^\[(\w+)\]",t,re.M)); print(c)
if "-v" in sys.argv: print(t[:6000])
