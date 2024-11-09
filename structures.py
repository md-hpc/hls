from hls import *
import random
from collections import deque
from math import floor

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
T = 5 # number of timesteps to simulate
UNIVERSE_SIZE = 3 # size of one dimension of the universe. Means that we will have cells in the range (0:4, 0:4, 0:4)
EPSILON = 0.1 # LJ const
SIGMA = 0.8 # LJ const
DT = 0.1 # timestep length
DENSITY = 10 # particles per cell


# depth of computation pipelines. Your emulator should be robust to any value of these
FORCE_PIPELINE_STAGES = 0
FILTER_PIPELINE_STAGES = 0  

# converts (i,j,k) tuple to linear idx and back
def linear_idx(i,j,k):
    return (i%UNIVERSE_SIZE) + (j%UNIVERSE_SIZE)*UNIVERSE_SIZE + (k%UNIVERSE_SIZE)*UNIVERSE_SIZE**2

def cubic_idx(i):
    return i%UNIVERSE_SIZE, i//UNIVERSE_SIZE%UNIVERSE_SIZE, i//UNIVERSE_SIZE**2%UNIVERSE_SIZE

class neighborhood:
    def __init__(self, cell):
        self.cell = cell
    def __iter__(self):
        i, j, k = cubic_idx(self.cell)
        for di in range(-1,2):
            for dj in range(-1,2):
                for dk in range(-1,2):
                    if di < 0 or di == 0 and dj < 0 or di == 0 and dj == 0 and dk < 0:
                        continue
                    yield linear_idx(i+di, j+dj, k+dk)

# derrived from above
N_CELL = UNIVERSE_SIZE ** 3
CUTOFF = SIGMA * 2.5
N_FILTER = sum([1 for _ in neighborhood(0)])
N_PARTICLE = N_CELL * DENSITY
L = CUTOFF * UNIVERSE_SIZE


# structs to hold particle data while it's passing through pipelines
class Struct:
    def __init__(self, data, addr, cell, ident):
        self.cell = cell
        self.addr = addr
        self.ident = ident
        setattr(self, ident, data)

    def __eq__(self, obj):
        return self.ident == obj.ident and self.cell == obj.cell and self.addr == obj.addr
    
    def __str__(self):
        return f"({self.cell}, {self.addr}), {getattr(self,self.ident)}"

Position = lambda r, addr, cell: Struct(r, addr, cell, "r")
Velocity = lambda v, addr, cell: Struct(v, addr, cell, "v")
Acceleration = lambda a, addr, cell: Struct(a, addr, cell, "a")

cell_from_position = lambda r: linear_idx(*[floor(x/CUTOFF)%UNIVERSE_SIZE for x in r])

# if you're confused about what this does, ask me (Vance)
class CacheMux(Logic):
    def __init__(self, name, idents, prefixes):
        super().__init__(name)
        
        self.opts = []
        for ident in idents:
            ctl = Input(self, f"{ident}-ready")
            setattr(self, f"{ident}_ready", ctl)
            
            inputs = []
            for prefix in prefixes:
                i = Input(self,f"{prefix}-{ident}")
                setattr(self, f"{prefix}_{ident}", i)
                inputs.append(i)
            
            self.opts.append([ctl, inputs])

        self.o = []
        for prefix in prefixes:
            o = Output(self,prefix)
            setattr(self, prefix, o)
            self.o.append(o)

    def logic(self):
        halt = True
        for opt in self.opts:
            if opt[0].get() != True:
                continue
            halt = False
            for i, o in zip(opt[1], self.o):
                o.set(i.get())

        if halt:
            for o in self.o:
                o.set(NULL)

class NullConst(Logic):
    def __init__(self):
        super().__init__("null-const")

        self.o = Output(self,"o")

    def logic(self):
        self.o.set(NULL)

class ResetConst(Logic):
    def __init__(self):
        super().__init__("reset-const")
        self.o = Output(self,"o")

    def logic(self):
        self.o.set(RESET)

class And(Logic):
    def __init__(self,n,name):
        super().__init__(name)

        self.i = [Input(self,f"i{i}") for i in range(n)]
        self.o = Output(self,"o")

    def logic(self):
        self.o.set(
            all([i.get() for i in self.i])
        )

def init_bram(ident, mux_idents):
    caches = [m.add(BRAM(512,f"{ident}-cache-{i}")) for i in range(N_CELL)]
    imuxes = [m.add(CacheMux(f"{ident}-imux-{i}", mux_idents, ["i","iaddr"])) for i in range(N_CELL)]
    omuxes = [m.add(CacheMux(f"{ident}-omux-{i}", mux_idents, ["oaddr"])) for i in range(N_CELL)]
    for imux, omux, cache in zip(imuxes, omuxes, caches):
        connect(imux.i, cache.i)
        connect(imux.iaddr, cache.iaddr)
        connect(omux.oaddr, cache.oaddr)
    return caches, imuxes, omuxes

class concat:
    def __init__(self, *iters):
        self.iters = iters

    def __iter__(self):
        for it in self.iters:
            for x in it:
                yield x

p_caches, p_imuxes, p_omuxes = init_bram("p", ["phase3","phase1"])
a_caches, a_imuxes, a_omuxes = init_bram("a", ["phase1","phase2"])
v_caches, v_imuxes, v_omuxes = init_bram("v", ["phase2","phase3"])

# reference these in the phase{1,2,3} files
null_const = m.add(NullConst())
reset_const = m.add(ResetConst())
