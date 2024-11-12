from hls import *
from common import *
import sys
from random import random, seed
from math import floor
import hls
import numpy
import os

import compute_pipeline


if "--naive" in sys.argv:
    import naive as  phase1
if "--particle-mapping" in sys.argv:
    import particle_mapping as phase1
if "--uniform-spread" in sys.argv:
    import uniform_spread as phase1
else:
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

        self.phase1_handler = None
        self.phase2_handler = None
        self.phase3_handler = None

        self._init = False

    def logic(self):
        if not self._init:
            if self.phase1_handler:
                self.phase1_handler()
            self._init = True

        if self.phase == PHASE1 and self.phase1_done.get():
            self.phase = PHASE2
            if self.phase2_handler:
                self.phase2_handler()
        elif self.phase == PHASE2 and self.phase2_done.get():
            self.phase = PHASE3
            if self.phase3_handler:
                self.phase3_handler()
        elif self.phase == PHASE3 and self.phase3_done.get():
            self.phase = PHASE1
            self._double_buffer = 0 if self._double_buffer else 1
            self.t += 1
            if self.phase1_handler:
                self.phase1_handler()

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

phase3_signal = m.add(And(len(phase3.CTL_DONE),"phase3-signal"))
for signal, i in zip(phase3.CTL_DONE, phase3_signal.i):
    connect(signal, i)    
connect(phase3_signal.o, phase3_done.i)

# phase 1 inputs
connect(control_unit.double_buffer, phase1.CTL_DOUBLE_BUFFER)
connect(control_unit.phase1_ready, phase1.CTL_READY)


# phase 2 inputs
for i in phase2.CTL_READY:
    connect(control_unit.phase2_ready, i)

# phase 3 connections
for i in phase3.CTL_DOUBLE_BUFFER:
    connect(control_unit.double_buffer, i)
for i in phase3.CTL_READY:
    connect(control_unit.phase3_ready, i)

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

# some helper fns
def extract_contents(caches):
    return [cache.contents.copy() for cache in caches]

def save_pos(t):
    if not os.path.exists("records"):
        os.mkdir("records")
    with open(f"records/t{t}","wb") as fp:
        for cache in p_caches:
            for p in cache.contents:
                if p is not NULL:
                    assert fp.write(p.tobytes()) == 24, "uh oh"

# simulation initialization
seed(SEED)
cidx = [0 for _ in range(N_CELL)] # index into contents of each p_cache
for _ in range(N_PARTICLE):
        r = numpy.array([L*random(), L*random(), L*random()])
        idx = cell_from_position(r)
        p_caches[idx].contents[cidx[idx]] = r
        v_caches[idx].contents[cidx[idx]] = numpy.array([0., 0., 0.])
        cidx[idx] += 1

# compute_pipeline.VERBOSE = True

# compute_pipeline ensures the correctness of its own inputs using global state
filter_inputs = set()
filter_expect = set()
pipeline_inputs = set()

for cp in phase1.compute_pipelines:
    for f in cp.filter_bank:
        f.input_set = filter_inputs
        f.input_expect = filter_expect
    cp.force_pipeline.input_set = pipeline_inputs
    #cp._reference.verbose = True

compute_pipeline.next_timestep(p_caches, pipeline_inputs, filter_inputs, filter_expect, control_unit._double_buffer)

if False:
    phase1.position_read_controller.verbose = True
    phase1.position_reader.verbose = True

t = 0
cycles_total = 0
t0 = control_unit.t
with numpy.errstate(all="raise"):
    while control_unit.t < T:
        if control_unit.t != t0:
            save_pos(control_unit.t)
            t0 = control_unit.t
            compute_pipeline.next_timestep(p_caches, pipeline_inputs, filter_inputs, filter_expect, control_unit._double_buffer)
            cycles_total += t
            t = 0
        print(f"==== CYCLE {control_unit.t}-{t}: {control_unit.phase} ====")
        m.clock()
        print()
        t += 1

save_pos(control_unit.t)
print(f"Emulator took {cycles_total} clock cycles to simulate {T} timesteps")
