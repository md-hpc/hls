from hls import *
from structures import *
import sys
from random import random
from math import floor
import hls
import numpy

hls.CONFIG_VERBOSE = False

import phase1
import phase2
if "-t" in sys.argv:
    import faux_phase3 as phase3
else:
    import phase3



# making these constants so I won't make any typos
PHASE1 = "phase 1"
PHASE2 = "phase 2"
PHASE3 = "phase 3"

class ControlUnit(Logic):
    def __init__(self):
        super().__init__("control-unit")
        self.t = 0 # current timestep

        self.phase = PHASE1
        
        self.phase1_done = Input(self,"phase1-done")
        self.phase2_done = Input(self,"phase2-done")
        self.phase3_done = Input(self,"phase3-done")

        self.phase1_ready = Output(self,"phase1-ready")
        self.phase2_ready = Output(self,"phase2-ready")
        self.phase3_ready = Output(self,"phase3-ready")

        self._double_buffer = 0
        self.double_buffer = Output(self,"double-buffer")

    def logic(self):
        if self.phase == PHASE1 and self.phase1_done.get():
            self.phase = PHASE2
        elif self.phase == PHASE2 and self.phase2_done.get():
            self.phase = PHASE3
        elif self.phase == PHASE3 and self.phase3_done.get():
            self.phase = PHASE1
            self._double_buffer = 0 if self._double_buffer else 1
            self.t += 1

        self.double_buffer.set(self._double_buffer)
        self.phase1_ready.set(self.phase == PHASE1)
        self.phase2_ready.set(self.phase == PHASE2)
        self.phase3_ready.set(self.phase == PHASE3)

control_unit = m.add(ControlUnit())

# control_unit inputs need to be buffered in registers to prevent cycles
#
# we can equivalently buffer the outputs. The choice is aesthetic
phase1_done = m.add(Register("phase1-done"))
phase1_done.contents = False
phase2_done = m.add(Register("phase2-done"))
phase2_done.contents = False
phase3_done = m.add(Register("phase3-done"))
phase3_done.contents = False


# control_unit inputs
connect(phase1_done.o, control_unit.phase1_done)
connect(phase2_done.o, control_unit.phase2_done)
connect(phase3_done.o, control_unit.phase3_done)

# register inputs
connect(phase1.CTL_DONE, phase1_done.i)
connect(phase2.CTL_DONE, phase2_done.i)
connect(phase3.CTL_DONE, phase3_done.i)

# phase 1 inputs
connect(control_unit.double_buffer, phase1.CTL_DOUBLE_BUFFER)
connect(control_unit.phase1_ready, phase1.CTL_READY)


# phase 2 inputs
connect(control_unit.phase2_ready, phase2.CTL_READY)

# phase 3 connections
connect(control_unit.double_buffer, phase3.CTL_DOUBLE_BUFFER)
connect(control_unit.phase3_ready, phase3.CTL_READY)

# p_muxes inputs
for mux in concat(p_imuxes, p_omuxes):
    connect(control_unit.phase3_ready, mux.phase3_ready)
    connect(control_unit.phase1_ready, mux.phase1_ready)

# a_muxes inputs
for mux in concat(a_imuxes, a_omuxes):
    connect(control_unit.phase1_ready, mux.phase1_ready)
    connect(control_unit.phase2_ready, mux.phase2_ready)

# v_muxes inputs
for mux in concat(v_imuxes, v_omuxes):
    connect(control_unit.phase2_ready, mux.phase2_ready)
    connect(control_unit.phase3_ready, mux.phase3_ready)

# simulation initialization
T = 5 # number of timesteps

L = CUTOFF * UNIVERSE_SIZE # length of one dimension of the simulation box
N = 80 * N_CELL # number of particles in the simulation

cidx = [0 for _ in range(N_CELL)] # index into contents of each p_cache
for _ in range(N):
        r = numpy.array([L*random(), L*random(), L*random()])
        idx = linear_idx( # find which p_cache this particle must go into
            *[floor(x/CUTOFF) for x in r]
        )
        p_caches[idx].contents[cidx[idx]] = r
        cidx[idx] += 1

for cache in v_caches:
    cache.contents = [
        numpy.array([0.0, 0.0, 0.0])
        for _ in cache.contents
    ]

bram = p_caches[0]
def validate_bram(unit):
    assert bram.contents[0] is not NULL, unit.name
m._clock_validation_fn = validate_bram

phase1.force_pipeline.verbose = True
phase1.pair_queue.verbose = True

t = 0
while control_unit.t < 3:
    print("==== NEXT CYCLE ====")
    t += 1
    if t == 10:
        breakpoint()
    m.clock()
