from hls import *
import random
from collections import deque

'''
constants, structures, and variables to be used by the phase{1,2,3}.py files

A few that will be of concern to you:

m - the global MockFPGA object. All components (Registers, Logic), should be added to this via m.add(<component>)

N_CELL - number of cells in the universe

{p,a,v}_caches - array of BRAMS containing particle, acceleration, and velocity data respectively for each cell. len({p,a,v}_caches) == N_CELL

class Position, Velocity, and Acceleration - I found when trying to implement this myself that it was useful to package particle origin (cell, addr) in the same variable as the actual data.
(note: do not write instances of these classes to the actual caches! Their origin is implicit in their actual address. Just write the floating point value)

particleFilter and forcePipeline - these are instances of ParticleFilter and ForcePipeline. Because they are shared between phase 1 and 2, I have implemented them here. We only use one of each for simplicity (we may expand to more in the future). particleFilter expects two instances of Position, and forcePipeline produces a tuple of Accelerations
'''


m = MockFPGA()

# Emulator constants
UNIVERSE_SIZE = 4 # size of one dimension of the universe. Means that we will have cells in the range (0:4, 0:4, 0:4)

EPSILON = 1.0 # LJ const
SIGMA = 1.0 # LJ const

# depth of computation pipelines. Your emulator should be robust to any value of these
FORCE_PIPELINE_STAGES = 70 
FILTER_PIPELINE_STAGES = 9  

# derrived from above
N_CELL = UNIVERSE_SIZE ** 3
CUTOFF = SIGMA * 2.5
N_FILTER = 0
for di in range(-1,2):
    for dj in range(-1,2):
        for dk in range(-1,2):
            if di < 0 or di == 0 and dj < 0 or di == 0 and dj == 0 and dk < 0:
                continue
            N_FILTER += 1

# structs to hold particle data while it's passing through pipelines
class Position:
    def __init__(self, r, addr, cell):
        self.cell = cell
        self.addr = addr
        self.r = r

class Velocity:
    def __init__(self, v, addr, cell):
        self.cell = cell
        self.addr = addr
        self.v = v

class Acceleration:
    def __init__(self, a, addr, cell):
        self.cell = cell
        self.addr = addr
        self.a = a

# converts (i,j,k) tuple to linear idx and back
def linear_idx(i,j,k):
    return (i%UNIVERSE_SIZE) + (j%UNIVERSE_SIZE)*UNIVERSE_SIZE + (k%UNIVERSE_SIZE)*UNIVERSE_SIZE**2

def cell_idx(i):
    return i%UNIVERSE_SIZE, i//UNIVERSE_SIZE%UNIVERSE_SIZE, i//UNIVERSE_SIZE**2%UNIVERSE_SIZE

class ParticleFilter(Logic):
    def __init__(self):
        super().__init__("Particle Filter")

        self.reference = Input(self,"PF_Reference")
        self.neighbor = Input(self,"PF_Neighbor")

        self.pair = Output(self,"PF_Pair")

        self.pipeline(FILTER_PIPELINE_STAGES)

    def logic(self):
        r = self.reference.get()
        n = self.neighbor.get()
        halt = self.halt.get()

        if r is NULL or n is NULL:
            self.pair.set(NULL)
            return

        r = math.sqrt(sum([
            (r1 - r2)**2 for r1, r2 in zip(r.r, n.r)
        ]))

        self.pair.set(
                (r, n) if r < CUTOFF else NULL
        )
 
class ForcePipeline(Logic):
    def __init__(self):
        super().__init__("Force Pipeline")
        
        self.i = Input(self,"FP_input")

        self.reference = Output(self,"FP_Reference")
        self.neighbor = Output(self,"FP_Neigbor")
        
        self.pipeline(FORCE_PIPELINE_STAGES)

    def logic(self):
        i = self.i.get()
        if i is NULL:
            self.reference.set(NULL)
            self.neighbor.set(NULL)
            return

        ref, neighbor = self.i.get()
        l2 = lambda v1, v2: math.sqrt(sum([(r1-r2)**2 for r1, r2 in zip(v1, v2)]))
        r = l2(ref.r, neighbor.r)
        f = 4*EPSILON*((SIGMA/r)**12-(SIGMA/r)**6)*(ref.r - neighbor.r)/r
        
        self.reference.set(
            Acceleration(cell = ref_cell, addr = ref_addr, a = f)
        )

        self.neighbor.set(
            Acceleration(cell = neighbor_cell, addr = neighbor_addr, a = -1.0*f)
        )


# Following in the pattern of trying to be as explicit as is practical
# we provide each cache mux type explicitly
class CacheMux(Logic):
    def __init__(self, name, idents, ctl_idents):
        super().__init__(name)

        assert len(idents) == len(ctl_idents)

        self.opts = []
        for ident, ctl_ident in zip(idents, ctl_idents):
            setattr(self,f"i_{ident}", Input(self,f"i-{ident}"))
            setattr(self,f"iaddr_{ident}", Input(self,f"iaddr-{ident}"))
            setattr(self,f"oaddr_{ident}", Input(self,f"oaddr-{ident}"))
            setattr(self, ctl_ident.replace("-","_"), Input(self, ctl_ident))
            self.opts.append(
                [
                    getattr(self,ctl_ident.replace("-","_")),
                    [getattr(self,f"{prefix}_{ident}") for prefix in ["i", "iaddr", "oaddr"]]
                ]
            )

        self.i = Output(self, "i")
        self.iaddr = Output(self, "iaddr")
        self.oaddr = Output(self, "oaddr")

    def logic(self):
        halt = True
        for opt in self.opts:
            if not opt[0].get():
                continue
            halt = False
            self.i.set(opt[1][0].get())
            self.iaddr.set(opt[1][1].get())
            self.oaddr.set(opt[1][2].get())
        if halt:
            self.i.set(NULL)
            self.iaddr.set(NULL)
            self.oaddr.set(NULL)

class NullConst(Logic):
    def __init__(self):
        super().__init__("null-const")

        self.o = Output(self,"o")

    def logic(self):
        self.o.set(NULL)

def init_bram(ident, mux_idents, mux_ctl_idents):
    caches = [m.add(BRAM(512,f"{ident}-cache-{i}")) for i in range(N_CELL)]
    muxes = [m.add(CacheMux(f"{ident}-mux-{i}", mux_idents, mux_ctl_idents)) for i in range(N_CELL)]
    for mux, cache in zip(muxes, caches):
        connect(mux.i, cache.i)
        connect(mux.iaddr, cache.iaddr)
        connect(mux.oaddr, cache.oaddr)
    return caches, muxes

p_caches, p_muxes = init_bram("p", ["p3","p1"], ["clt-position-update-ready","ctl-force-evaluation-ready"])
a_caches, a_muxes = init_bram("a", ["p1","p2"], ["ctl-force-evaluation-ready", "ctl-velocity-update-ready"])
v_caches, v_muxes = init_bram("v", ["p2","p3"], ["ctl-velocity-update-ready","ctl-position-update-ready"])

# reference these in the phase{1,2,3} files
filter_bank = [m.add(ParticleFilter()) for _ in range(N_FILTER)]
force_pipeline = m.add(ForcePipeline())
null_const = m.add(NullConst())
