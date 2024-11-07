from hls import *
import random
from collections import deque

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


p_caches = [m.add(BRAM(512, f"pc{i}")) for i in range(N_CELL)] # position cache

class NullConst(Logic):
    def __init__(self):
        super().__init__("null-const")
        self.o = Output(self,"null")

    def logic(self):
        self.o.set(NULL)

null_const = m.add(NullConst())
for p in p_caches:
    connect(null_const.o, p.i)
    connect(null_const.o, p.iaddr)
    

# structs to hold particle data while it's passing through pipelines
class Position:
    def __init__(self, r, addr, cell):
        self.cell = cell
        self.addr = addr
        self.r = r
    def __str__(self):
        return f"({self.cell}, {self.addr} -> {self.r})"

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

# given a linear idx of a cell cache, gives the linear index of all neighbors, including self
class neighborhood:
    def __init__(self, cidx):
        self.cidx = cidx
    def __iter__(self):
        i, j, k = cell_idx(self.cidx)
        for di in range(-1,2):
            for dj in range(-1,2):
                for dk in range(-1,2):
                    if di < 0 or di == 0 and dj < 0 or di == 0 and dj == 0 and dk < 0:
                        continue
                    yield linear_idx(i+di, j+dj, k+dk)


filter_set = set()

class ParticleFilter(Logic):
    def __init__(self, name):
        super().__init__(name)

        self.reference = Input(self, "reference")
        self.neighbor = Input(self, "neighbor")

        self.pair = Output(self, "o")

        self.pipeline(FILTER_PIPELINE_STAGES)

        self.input_set = set()

    def logic(self):
        r = self.reference.get()
        n = self.neighbor.get()

        if r is NULL or n is NULL:
            self.pair.set(NULL)
            return

        p = pident(r,n)
        self.input_set.add(p)
        self.pair.set(p)
 
class ForcePipeline(Logic):
    def __init__(self, name):
        super().__init__(name)

        self.i = Input(self, "i")
        
        self.input_set = set()

    def logic(self):
        i = self.i.get()
        self.input_set.add(i)

# reference these in the phase{1,2} files
filter_bank = [ParticleFilter(f"filter{i}") for i in range(N_FILTER)]
force_pipeline = ForcePipeline("FE")

ident = 0
for p_cache in p_caches:
    for i in range(10):
        p_cache.contents[i] = ident
        ident += 1

pident = lambda r,n: r*ident + n

