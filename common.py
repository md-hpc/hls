import argparse
from hls import *
import random
from collections import deque
from math import floor
import numpy
from numpy.linalg import norm

import os
from os.path import dirname, join
import shutil
from random import random, seed

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

parser = argparse.ArgumentParser()
parser.add_argument("-c","--cpar", type=int, default=9)
parser.add_argument("-p","--ppar", type=int, default=4)
parser.add_argument("-t","--time", type=int, default=2)
parser.add_argument("-n","--particles", type=int, default=300)
parser.add_argument("-u","--size", type=int, default=3)
args = parser.parse_args()


# Emulator parameters
T = args.time # number of timesteps to simulate
DT = 1e-7 # timestep length
UNIVERSE_SIZE = args.size # size of one dimension of the universe
N_PARTICLE = args.particles
EPSILON = 40 # LJ const
SIGMA = 1 # LJ const
SEED = 0 # Random seed for particle initialization
FORCE_PIPELINE_STAGES = 70 # depth of computation pipeline
FILTER_PIPELINE_STAGES = 13  # depth of filter pipeline
N_PIPELINE = 7 # for particle-mapping and uniform-spread, number of compute units working in parallel

N_CPAR = args.cpar # particle parallelism
N_PPAR = args.ppar # cell parallelism

VERIFY_COMPUTED = True # At every timestep, use verify.compute_targets to compare the emulator's computations with what they should be
ERR_TOLERANCE = 1e-2 # % error tolerance. The max permissable value of norm(target-computed)/norm(computed) for each computed acceleration, velocity, or position

# constants
N_FILTER = 18 # number of filters per force pipeline
BSIZE = 512 # bram size
DBSIZE = BSIZE//2 # double buffer buffer size

# derrived from above
N_CELL = UNIVERSE_SIZE ** 3 # total number of cells in universe
CUTOFF = SIGMA * 2.5 # cutoff radius
L = CUTOFF * UNIVERSE_SIZE # length of one side of the universe
N_IDENT = N_CELL*BSIZE # maximum number of unique particles. Used for verification

seed(SEED)


# gives an initial position and velocity vector
r0 = lambda: numpy.array([L*random() for _ in range(3)])
v0 = lambda: numpy.array([EPSILON*(random() - 0.5) for _ in range(3)])

# Muxes the inputs of a cache between phases
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

# always writes NULL to self.o
class NullConst(Logic):
    def __init__(self):
        super().__init__("null-const")

        self.o = Output(self,"o")

    def logic(self):
        self.o.set(NULL)

# always writes RESET to self.o
class ResetConst(Logic):
    def __init__(self):
        super().__init__("reset-const")
        self.o = Output(self,"o")

    def logic(self):
        self.o.set(RESET)


# creates a new array of BRAMs along with the muxes necessary to access it between phases
def init_bram(ident, mux_idents):
    caches = [m.add(BRAM(512,f"{ident}-cache-{i}")) for i in range(N_CELL)]
    imuxes = [m.add(CacheMux(f"{ident}-imux-{i}", mux_idents, ["i","iaddr"])) for i in range(N_CELL)]
    omuxes = [m.add(CacheMux(f"{ident}-omux-{i}", mux_idents, ["oaddr"])) for i in range(N_CELL)]
    for imux, omux, cache in zip(imuxes, omuxes, caches):
        connect(imux.i, cache.i)
        connect(imux.iaddr, cache.iaddr)
        connect(omux.oaddr, cache.oaddr)
    return caches, imuxes, omuxes


# one-dimensional modulo L distance from a to b
def modd(a,b):
    opts = [(b-L)-a, b-a, (b+L)-a]
    m = abs(opts[0])
    mi = 0
    for i, opt in enumerate(opts):
        if abs(opt) < m:
            m = abs(opt)
            mi = i
    return opts[mi]
        
# three-dimensional modulo L distance from reference to neighbor
def modr(reference, neighbor):
    r = numpy.zeros(3)
    for i in range(3):
        r[i] = modd(reference[i], neighbor[i])
    return r

# whether we should evaluate neighbor wrt reference under N3L half-shell method
def n3l(reference,neighbor):
    if type(reference) is not numpy.ndarray:
        raise TypeError(f"Called n3l with unsupported {type(reference)}")
    r = modr(reference, neighbor)
    for i in range(3):
        if r[i] < 0:
            return False
        if r[i] > 0:
            return True
    return False # should not evaluate self wrt self

# whether we should evaluate neighbor cell cell_n wrt reference cell cell_r under N3: half-shell method
def n3l_cell(cell_r, cell_n):
    return (cubic_idx(cell_n)[0] - cubic_idx(cell_r)[0]) % UNIVERSE_SIZE <= 1


# converts a cubic cell index [i,j,k] to a linear (integer) index
def linear_idx(i,j,k):
    return (i%UNIVERSE_SIZE) + (j%UNIVERSE_SIZE)*UNIVERSE_SIZE + (k%UNIVERSE_SIZE)*UNIVERSE_SIZE**2

# converts a linear (integer) cell index to a cubic index [i, j, k] 
def cubic_idx(i):
    return [i%UNIVERSE_SIZE, i//UNIVERSE_SIZE%UNIVERSE_SIZE, i//UNIVERSE_SIZE**2%UNIVERSE_SIZE]

# an iterator that gives all the neighbor cells we should evaluate wrt linear cell index cell
# full == True will just yield all neighboring cells instead.
def neighborhood(cell, full=False):
    i, j, k = cubic_idx(cell)
    for di in range(-1 if full else 0,2):
        for dj in range(-1,2):
            for dk in range(-1,2):
                yield linear_idx(i+di, j+dj, k+dk)
sz = sum([1 for _ in neighborhood(0)])
assert sz == N_FILTER, sz

# structs to hold particle data while it's passing through pipelines
class Struct:
    def __init__(self, data, addr, cell, ident):
        self.cell = cell # cell of origin
        self.addr = addr # addr of origin
        self.ident = ident # an identifier for the attribute that will hold the actual data
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

# given a [x,y,z] position vector, gives the linear cell index of the cell it should be placed in
cell_from_position = lambda r: linear_idx(*[floor(x/CUTOFF)%UNIVERSE_SIZE for x in r])

# identifiers so we can track whether or not the compute units are recieving all and only expected inputs
ident = lambda p: p.cell*BSIZE + p.addr
def pair_ident(p1, p2):
    return N_IDENT*ident(p1) + ident(p2)

# helper
def ident_to_p(ident):
    cell = ident // BSIZE
    addr = ident % BSIZE
    return (cell), addr

# converts a pair ident generated by the above function back into the (cell1, addr1) (cell2, addr2) tuple that generated it
def pi_to_p(pi):
    ident1 = pi // N_IDENT
    ident2 = pi % N_IDENT
    reference = ident_to_p(ident1)
    neighbor = ident_to_p(ident2)
    return reference, neighbor

# self.o is a logical AND of all inputs
class And(Logic):
    def __init__(self,n,name):
        super().__init__(name)

        self.i = [Input(self,f"i{i}") for i in range(n)]
        self.o = Output(self,"o")

    def logic(self):
        self.o.set(
            all([i.get() and i.get() is not NULL for i in self.i])
        )

# Sets list or nested list of Outputs to NULL
def nul(out):
    if isinstance(out,Output):
        out.set(NULL)
        return
    for o in out:
        nul(o)

# lazily concatenates an array of iterators
def concat(*iters):
    for it in iters:
        for x in it:
            yield x

# helper
def _lj(reference, neighbor):
    r = norm(modr(reference, neighbor))

    if r == 0:
        return 0.
    else:
        return 4.0*EPSILON*(6.0*SIGMA**6.0/r**7.0-12.0*SIGMA**12/r**13.0)*(neighbor - reference)/r

# compute LJ force between two particles given their position
def lj(reference, neighbor):
    f = _lj(reference, neighbor)
    return numpy.minimum(numpy.abs(f), LJ_MAX) * numpy.sign(f)

_lj_max = _lj(numpy.array([0., 0., 0.]), numpy.array([(26/7)**(1/6)*SIGMA, 0., 0.]))[0]
LJ_MAX = 4*numpy.array([_lj_max, _lj_max, _lj_max])

   
# given the double_buffer signal from control_unit, computes the address offset into the BRAMs for this cycle
def db(double_buffer):
    return DBSIZE if double_buffer else 0

# negation of db()
def ndb(double_buffer):
    return 0 if double_buffer else DBSIZE

# verify.py and emulator.py will store their computed position data here
# viz.py will look here for data to render. Don't want anything from old runs to be rendered
def clear_records():
    path = join(dirname(__file__),"records")
    shutil.rmtree(path)
    os.mkdir(path)


# globally accessible FPGA elements
m = MockFPGA()
null_const = m.add(NullConst())
reset_const = m.add(ResetConst())

p_caches, p_imuxes, p_omuxes = init_bram("p", ["phase3","phase1"])
v_caches, v_imuxes, v_omuxes = init_bram("v", ["phase1","phase3"])

