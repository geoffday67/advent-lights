"""Small 2-layer grid A* router (octilinear, turn + via penalties) for the advent receiver board.
Obstacles are kept as exact shapes; for each net, block masks are rebuilt from every shape
belonging to other nets, inflated by clearance + half track width (or via radius)."""
import numpy as np, heapq, math

G = 0.05          # grid pitch, mm
DIRS = [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]

class Shape:
    # kind: 'rect' (cx,cy,hw,hh) | 'circ' (cx,cy,r) | 'seg' (x1,y1,x2,y2,hw)
    def __init__(self, kind, params, layers, net, via_ok=True):
        self.kind, self.p, self.layers, self.net = kind, params, layers, net
    def bbox(self):
        k,p=self.kind,self.p
        if k=='rect': return p[0]-p[2],p[1]-p[3],p[0]+p[2],p[1]+p[3]
        if k=='circ': return p[0]-p[2],p[1]-p[2],p[0]+p[2],p[1]+p[2]
        return min(p[0],p[2])-p[4],min(p[1],p[3])-p[4],max(p[0],p[2])+p[4],max(p[1],p[3])+p[4]
    def dist(self, X, Y):
        k,p=self.kind,self.p
        if k=='rect':
            dx=np.maximum(np.abs(X-p[0])-p[2],0); dy=np.maximum(np.abs(Y-p[1])-p[3],0)
            return np.hypot(dx,dy)
        if k=='circ':
            return np.maximum(np.hypot(X-p[0],Y-p[1])-p[2],0)
        x1,y1,x2,y2,hw=p
        vx,vy=x2-x1,y2-y1; L2=vx*vx+vy*vy
        t=np.clip(((X-x1)*vx+(Y-y1)*vy)/L2,0,1) if L2>0 else 0*X
        return np.maximum(np.hypot(X-(x1+t*vx),Y-(y1+t*vy))-hw,0)

class Router:
    def __init__(self, W, H, clr=0.2, edge=0.35, via_d=0.6, margin=0.03):
        self.W,self.H,self.clr,self.edge,self.vr,self.m=W,H,clr,edge,via_d/2,margin
        self.nx,self.ny=int(round(W/G))+1,int(round(H/G))+1
        xs=np.arange(self.nx)*G; ys=np.arange(self.ny)*G
        self.X,self.Y=np.meshgrid(xs,ys)
        self.shapes=[]
        self.noTop=np.zeros((self.ny,self.nx),bool)     # regions where top-layer tracks are banned
        self.noVia=np.zeros((self.ny,self.nx),bool)
        self.noTrack={}                                  # net-specific bans: net -> mask (both layers)
        self.noTopNet={}                                 # net-specific top-layer bans
        self.cost_bottom=1.3
    def add(self, s): self.shapes.append(s)
    def ban_rect(self, mask, x1,y1,x2,y2):
        mask[(self.X>=x1)&(self.X<=x2)&(self.Y>=y1)&(self.Y<=y2)]=True
    def masks(self, net, hw):
        blk=[np.zeros((self.ny,self.nx),bool) for _ in range(2)]
        vblk=np.zeros((self.ny,self.nx),bool)
        own=[np.zeros((self.ny,self.nx),bool) for _ in range(2)]
        ti=hw+self.clr+self.m; vi=self.vr+self.clr+self.m
        for s in self.shapes:
            x1,y1,x2,y2=s.bbox(); r=max(ti,vi)+G
            i1=max(int((x1-r)/G),0); i2=min(int((x2+r)/G)+2,self.nx)
            j1=max(int((y1-r)/G),0); j2=min(int((y2+r)/G)+2,self.ny)
            d=s.dist(self.X[j1:j2,i1:i2],self.Y[j1:j2,i1:i2])
            if s.net==net and net is not None:
                for L in s.layers:
                    own[L][j1:j2,i1:i2]|=(d<=0)
                # own SMD pads: no via inside or touching
                if s.kind=='rect' and len(s.layers)==1: vblk[j1:j2,i1:i2]|=(d<self.vr+0.1)
                continue
            for L in s.layers: blk[L][j1:j2,i1:i2]|=(d<ti)
            vblk[j1:j2,i1:i2]|=(d<vi)
        e=self.edge
        for L in (0,1):
            blk[L]|=(self.X<e+hw)|(self.X>self.W-e-hw)|(self.Y<e+hw)|(self.Y>self.H-e-hw)
        vblk|=(self.X<e+self.vr)|(self.X>self.W-e-self.vr)|(self.Y<e+self.vr)|(self.Y>self.H-e-self.vr)
        vblk|=self.noVia
        blk[0]|=self.noTop
        if net in self.noTrack:
            blk[0]|=self.noTrack[net]; blk[1]|=self.noTrack[net]
        if net in self.noTopNet: blk[0]|=self.noTopNet[net]
        for L in (0,1): blk[L]&=~own[L]       # own copper always enterable
        return blk,vblk,own

    def route(self, net, src_cells, tgt_mask, hw, via_cost=40, turn=3, allow_bottom=True):
        """src_cells: list of (L,j,i). tgt_mask: [mask_top, mask_bot]. Returns list of (L,j,i)."""
        blk,vblk,_=self.masks(net,hw)
        if not allow_bottom: blk[1][:]=True
        ny,nx=self.ny,self.nx
        INF=1e18
        best=np.full((2,ny,nx,9),INF,np.float64)
        prev={}
        tj,ti_=np.nonzero(tgt_mask[0]|tgt_mask[1])
        if len(tj)==0: return None
        tx,ty=ti_.mean(),tj.mean()
        def h(j,i): # octile distance to target centroid (admissible-ish)
            dx,dy=abs(i-tx),abs(j-ty); return 10*(max(dx,dy)+0.414*min(dx,dy))*0.9
        pq=[]
        for (L,j,i) in src_cells:
            if blk[L][j,i] and not tgt_mask[L][j,i]: pass
            best[L,j,i,8]=0; heapq.heappush(pq,(h(j,i),0.0,L,j,i,8))
        done=None; n=0
        while pq:
            f,g,L,j,i,d=heapq.heappop(pq)
            if g>best[L,j,i,d]: continue
            if tgt_mask[L][j,i]: done=(L,j,i,d); break
            n+=1
            if n>3_000_000: break
            cl=self.cost_bottom if L==1 else 1.0
            for k,(di,dj) in enumerate(DIRS):
                i2,j2=i+di,j+dj
                if not(0<=i2<nx and 0<=j2<ny) or blk[L][j2,i2]: continue
                if di and dj and (blk[L][j,i2] or blk[L][j2,i]): continue
                c=g+(14.14 if di and dj else 10)*cl+(0 if d in (8,k) else turn*(1 if (abs(k-d)%8) in (1,7) else 3))
                if c<best[L,j2,i2,k]:
                    best[L,j2,i2,k]=c; prev[(L,j2,i2,k)]=(L,j,i,d)
                    heapq.heappush(pq,(c+h(j2,i2),c,L,j2,i2,k))
            if not vblk[j,i]:
                L2=1-L; c=g+via_cost
                if not blk[L2][j,i] and c<best[L2,j,i,8]:
                    best[L2,j,i,8]=c; prev[(L2,j,i,8)]=(L,j,i,d)
                    heapq.heappush(pq,(c+h(j,i),c,L2,j,i,8))
        if done is None: return None
        path=[done]
        while path[-1] in prev: path.append(prev[path[-1]])
        return [(L,j,i) for (L,j,i,d) in reversed(path)]

def simplify(path):
    """Convert cell path into ('seg',L,(x1,y1),(x2,y2)) and ('via',(x,y)) items in mm."""
    out=[]; pts=[(L,i*G,j*G) for (L,j,i) in path]
    k=0
    while k<len(pts)-1:
        L,x,y=pts[k]
        if pts[k+1][0]!=L:
            out.append(('via',(x,y))); k+=1; continue
        # extend straight run
        dx,dy=pts[k+1][1]-x,pts[k+1][2]-y; m=k+1
        while m+1<len(pts) and pts[m+1][0]==L and abs((pts[m+1][1]-pts[m][1])-dx)<1e-9 and abs((pts[m+1][2]-pts[m][2])-dy)<1e-9: m+=1
        out.append(('seg',L,(x,y),(pts[m][1],pts[m][2]))); k=m
    return out
