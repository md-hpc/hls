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


p_caches = [m.add(BRAM(512)) for _ in range(N_CELL)] # position cache
a_caches = [m.add(BRAM(512)) for _ in range(N_CELL)] # acceleration cache
v_caches = [m.add(BRAM(512)) for _ in range(N_CELL)] # velocity cache

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
        super().__init__()

        self.reference = Input(self)
        self.neighbor = Input(self)

        self.pair = Output(self)

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
        super().__init__()

        self.i = Input(self)

        self.reference = Output(self)
        self.neighbor = Output(self)
        
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

# reference these in the phase{1,2} files
filterBank = [ParticleFilter() for _ in range(N_FILTER)]
forcePipeline = ForcePipeline()
