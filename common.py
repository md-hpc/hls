from hls import *
import random
from collections import deque
from math import floor
import numpy
from numpy.linalg import norm

import os
from os.path import dirname, join
import shutil

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

# verify.py and emulator.py will store their computed position data here
# viz.py will look here for data to render. Don't want anything from old runs to be rendered
RECORDS_PATH = join(dirname(__file__),"records")
shutil.rmtree(RECORDS_PATH)
os.mkdir(RECORDS_PATH)


# Emulator parameters
T = 1000 # number of timesteps to simulate
DT = 1e-2 # timestep length
UNIVERSE_SIZE = 3 # size of one dimension of the universe
EPSILON = 5 # LJ const
SIGMA = 1 # LJ const
DENSITY = 20 # particles per cell
SEED = 1 # Random seed for particle initialization
FORCE_PIPELINE_STAGES = 0 # depth of computation pipeline
FILTER_PIPELINE_STAGES = 0  # depth of filter pipeline
N_PIPELINE = 7 # for particle-mapping and uniform-spread, number of compute units working in parallel
N_PPAR = 4 # particle parallelism
N_CPAR = 8 # cell parallelism

# constants
N_FILTER = 14 # number of filters per force pipeline
BSIZE = 512 # bram size
DBSIZE = BSIZE//2 # double buffer buffer size

# derrived from above
N_CELL = UNIVERSE_SIZE ** 3
CUTOFF = SIGMA * 2.5
N_PARTICLE = N_CELL * DENSITY
L = CUTOFF * UNIVERSE_SIZE
N_IDENT = N_CELL*BSIZE
LJ_MAX = None

# mod min, used to "center" our universe at our reference for N3L comparisons
def modm(n, m):
    if abs(n) < abs(n%m):
        return n
    else:
        return n%m

def n3l(reference,neighbor):
    if type(reference) is not type(neighbor):
        raise TypeError(f"Called n3l with mismatched {type(reference)} and {type(neighbor)}")
    if type(reference) is list:
        return _n3l([modm((n-r),UNIVERSE_SIZE) for r,n in zip(reference, neighbor)])
    if type(reference) is numpy.ndarray:
        return _n3l([modm(r, L) for r in (neighbor-reference).tolist()])
    if type(reference) is int:
        return _n3l([modm(n-r,UNIVERSE_SIZE) for r,n in zip(cubic_idx(reference), cubic_idx(neighbor))])
    raise TypeError(f"Called n3l with unsupported {type(reference)}")

def _n3l(pos):
    for p in pos:
        if p < 0:
            return False
        if p > 0:
            return True
    return True # want to include [0, 0, 0]

# converts (i,j,k) tuple to linear idx and back
def linear_idx(i,j,k):
    return (i%UNIVERSE_SIZE) + (j%UNIVERSE_SIZE)*UNIVERSE_SIZE + (k%UNIVERSE_SIZE)*UNIVERSE_SIZE**2

def cubic_idx(i):
    return [i%UNIVERSE_SIZE, i//UNIVERSE_SIZE%UNIVERSE_SIZE, i//UNIVERSE_SIZE**2%UNIVERSE_SIZE]

def neighborhood(cell, full=False):
    i, j, k = cubic_idx(cell)
    for di in range(-1,2):
        for dj in range(-1,2):
            for dk in range(-1,2):
                if not n3l([0,0,0],[di,dj,dk]) and not full:
                    continue
                yield linear_idx(i+di, j+dj, k+dk)
sz = sum([1 for _ in neighborhood(0)])

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
        return f"{self.origin()}, {getattr(self,self.ident)}"

    def origin(self):
        return f"({cubic_idx(self.cell)}, {self.addr})"

Position = lambda r, addr, cell: Struct(r, addr, cell, "r")
Velocity = lambda v, addr, cell: Struct(v, addr, cell, "v")
Acceleration = lambda a, addr, cell: Struct(a, addr, cell, "a")

cell_from_position = lambda r: linear_idx(*[floor(x/CUTOFF)%UNIVERSE_SIZE for x in r])

# identifiers so we can track whether or not the compute units are recieving all and only expected inputs
ident = lambda p: p.cell*BSIZE + p.addr
pair_ident = lambda p1, p2: N_IDENT*ident(p1) + ident(p2)

def ident_to_p(ident):
    cell = ident // BSIZE
    addr = ident % BSIZE
    return cubic_idx(cell), addr

def pi_to_p(pi):
    ident1 = pi // N_IDENT
    ident2 = pi % N_IDENT
    reference = ident_to_p(ident1)
    neighbor = ident_to_p(ident2)
    return reference, neighbor



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
            all([i.get() and i.get() is not NULL for i in self.i])
        )

def nul(out):
    if type(out) is list:
        for o in out:
            o.set(NULL)
    else:
        out.set(NULL)

def concat(*iters):
    for it in iters:
        for x in it:
            yield x


def _lj(reference, neighbor):
    r = norm(reference - neighbor)
    return 4.0*EPSILON*(6.0*SIGMA**6.0/r**7.0-12.0*SIGMA**12/r**13.0)*(neighbor - reference)/r

def lj(reference, neighbor):
    f = _lj(reference, neighbor)
    return numpy.minimum(numpy.abs(f), LJ_MAX) * numpy.sign(f)
    

_lj_max = _lj(numpy.array([0., 0., 0.]), numpy.array([(26/7)**(1/6)*SIGMA, 0., 0.]))[0]
LJ_MAX = 4*numpy.array([_lj_max, _lj_max, _lj_max])

def db(double_buffer):
    return DBSIZE if double_buffer else 0

def ndb(double_buffer):
    return 0 if double_buffer else DBSIZE

def bram_enum(cache, double_buffer):
    a0 = db(double_buffer)
    a1 = a0 + DBSIZE
    for addr, val in enumerate(cache[a0:a1]):
        yield addr + a0, val

def sort_contents(contents):
    s = lambda a: int.from_bytes(a[1][0])
    contents.sort(key=s)
    return [[i for i,x in c] for c in contents], [[x for x in c] for c in contents]


def init_bram(ident, mux_idents):
    caches = [m.add(BRAM(512,f"{ident}-cache-{i}")) for i in range(N_CELL)]
    imuxes = [m.add(CacheMux(f"{ident}-imux-{i}", mux_idents, ["i","iaddr"])) for i in range(N_CELL)]
    omuxes = [m.add(CacheMux(f"{ident}-omux-{i}", mux_idents, ["oaddr"])) for i in range(N_CELL)]
    for imux, omux, cache in zip(imuxes, omuxes, caches):
        connect(imux.i, cache.i)
        connect(imux.iaddr, cache.iaddr)
        connect(omux.oaddr, cache.oaddr)
    return caches, imuxes, omuxes

p_caches, p_imuxes, p_omuxes = init_bram("p", ["phase3","phase1"])
a_caches, a_imuxes, a_omuxes = init_bram("a", ["phase1","phase2"])
v_caches, v_imuxes, v_omuxes = init_bram("v", ["phase2","phase3"])

# reference these in the phase{1,2,3} files
null_const = m.add(NullConst())
reset_const = m.add(ResetConst())
