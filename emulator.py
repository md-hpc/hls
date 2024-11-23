from hls import *
from common import *
import sys
from random import random, seed, shuffle
from math import floor, inf
import hls
import numpy
import os

import compute_pipeline
import verify
from verify import verify_emulator

if "--naive" in sys.argv:
    import naive as  phase1
if "--particle-mapping" in sys.argv:
    import particle_mapping as phase1
if "--uniform-spread" in sys.argv:
    import uniform_spread as phase1
else:
    import phase1

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
        self.phase3_done = Input(self,"phase3-done")

        self.phase1_ready = Output(self,"phase1-ready")
        self.phase3_ready = Output(self,"phase3-ready")

        self._double_buffer = 0
        self.double_buffer = Output(self,"double-buffer")

        self._init = False

    def logic(self):
        if not self._init:
            self._init = True

        if self.phase == PHASE1 and self.phase1_done.get():
            self.phase = PHASE3
        elif self.phase == PHASE3 and self.phase3_done.get():
            self.phase = PHASE1
            self._double_buffer = 0 if self._double_buffer else 1
            self.t += 1

        self.double_buffer.set(self._double_buffer)
        self.phase1_ready.set(self.phase == PHASE1)
        self.phase3_ready.set(self.phase == PHASE3)

control_unit = m.add(ControlUnit())
verify.CONTROL_UNIT = control_unit

# control_unit inputs need to be buffered in registers to prevent cycles
#
# we can equivalently buffer the outputs. The choice is aesthetic
phase1_done = m.add(Register("phase1-done"))
phase1_done.contents = False
phase3_done = m.add(Register("phase3-done"))
phase3_done.contents = False


# control_unit inputs
connect(phase1_done.o, control_unit.phase1_done)
connect(phase3_done.o, control_unit.phase3_done)

# register inputs
connect(phase1.CTL_DONE, phase1_done.i)

phase3_signal = m.add(And(len(phase3.CTL_DONE),"phase3-signal"))
for signal, i in zip(phase3.CTL_DONE, phase3_signal.i):
    connect(signal, i)    
connect(phase3_signal.o, phase3_done.i)

# phase 1 inputs
connect(control_unit.double_buffer, phase1.CTL_DOUBLE_BUFFER)
connect(control_unit.phase1_ready, phase1.CTL_READY)

# phase 3 connections
for i in phase3.CTL_DOUBLE_BUFFER:
    connect(control_unit.double_buffer, i)
for i in phase3.CTL_READY:
    connect(control_unit.phase3_ready, i)

# p_muxes inputs
for mux in concat(p_imuxes, p_omuxes):
    connect(control_unit.phase3_ready, mux.phase3_ready)
    connect(control_unit.phase1_ready, mux.phase1_ready)

# v_muxes inputs
for mux in concat(v_imuxes, v_omuxes):
    connect(control_unit.phase1_ready, mux.phase1_ready)
    connect(control_unit.phase3_ready, mux.phase3_ready)

# simulation initialization
clear_records()
seed(SEED)
cidx = [0 for _ in range(N_CELL)] # index into contents of each p_cache
for _ in range(N_PARTICLE):
        r = r0()
        idx = cell_from_position(r)
        p_caches[idx].contents[cidx[idx]] = r
        v_caches[idx].contents[cidx[idx]] = v0()

        cidx[idx] += 1
verify_emulator() # initialized the filter_expect and pipeline_expect sets

t = 0
cycles_total = 0
t0 = control_unit.t
with numpy.errstate(all="raise"):
    while control_unit.t < T:
        print(f"CYCLE {control_unit.t}-{t} (p={N_PPAR}, c={N_CPAR}) {control_unit.phase}:", end=" ")
        m.clock()
        t += 1
        if control_unit.t != t0:
            verify_emulator()
            t0 = control_unit.t
            cycles_total += t
            t = 0



cycles_total += t
with open(f"records/t{control_unit.t-1}","wb") as fp:
    for cache in p_caches:
        offst = ndb(control_unit._double_buffer)
        for p in cache.contents[offst:offst+DBSIZE]:
            if p is not NULL:
                b = fp.write(p.tobytes())
                assert b == 24, "uh oh"

    
print(f"Emulator took {cycles_total} clock cycles to simulate {T} timesteps")

path = f"performance.csv"
if not os.path.exists(path):
    with open(path,"w") as fp:
        print(f"N_PARTICLE,N_CELL,T,N_CPAR,N_PPAR,cycles_total", file=fp)
with open(path,"a") as fp:
    print(f"{N_PARTICLE},{N_CELL},{T},{N_CPAR},{N_PPAR},{cycles_total}", file=fp)
