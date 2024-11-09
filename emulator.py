from hls import *
from structures import *
import sys
from random import random
from math import floor
import hls

hls.CONFIG_VERBOSE = False

import phase1
import phase2
if "-t" in sys.argv:
    import faux_phase3 as phase3
else:
    import phase3



# making these constants so I won't make any typos
FORCE_EVALUATION = "force evaluation"
VELOCITY_UPDATE = "velocity update"
POSITION_UPDATE = "position update"

class ControlUnit(Logic):
    def __init__(self):
        super().__init__("control-unit")
        self.t = 0 # current timestep

        self.phase = FORCE_EVALUATION
        
        self.force_evaluation_done = Input(self,"force-evaluation-done")
        self.velocity_update_done = Input(self,"velocity-update-done")
        self.position_update_done = Input(self,"position-update-done")

        self.force_evaluation_ready = Output(self,"force-evaluation-ready")
        self.velocity_update_ready = Output(self,"velocity-update-ready")
        self.position_update_ready = Output(self,"position-update-ready")

        self._double_buffer = 0
        self.double_buffer = Output(self,"double-buffer")

    def logic(self):
        if self.phase == FORCE_EVALUATION and self.force_evaluation_done.get():
            self.phase = VELOCITY_UPDATE
        elif self.phase == VELOCITY_UPDATE and self.velocity_update_done.get():
            self.phase = POSITION_UPDATE
        elif self.phase == POSITION_UPDATE and self.position_update_done.get():
            self.phase = FORCE_EVALUATION
            self._double_buffer = 0 if self._double_buffer else 1
            self.t += 1

        self.double_buffer.set(self._double_buffer)
        self.force_evaluation_ready.set(self.phase == FORCE_EVALUATION)
        self.velocity_update_ready.set(self.phase == VELOCITY_UPDATE)
        self.position_update_ready.set(self.phase == POSITION_UPDATE)

# control_unit outputs need to be buffered in registers so we don't form cycles
# 
# we could equivalently buffer control_unit inputs instead, the choice is aesthetic
double_buffer = m.add(Register("double-buffer"))
force_evaluation_ready = m.add(Register("force-evaluation"))
velocity_update_ready = m.add(Register("velocity-update"))
position_update_ready = m.add(Register("position-update"))

control_unit = m.add(ControlUnit())

# control_unit inputs
connect(phase1.CTL_FORCE_EVALUATION_DONE, control_unit.force_evaluation_done)
connect(phase2.CTL_VELOCITY_UPDATE_DONE, control_unit.velocity_update_done)
connect(phase3.CTL_POSITION_UPDATE_DONE, control_unit.position_update_done)

# register inputs
connect(control_unit.force_evaluation_ready, force_evaluation_ready.i)
connect(control_unit.velocity_update_ready, velocity_update_ready.i)
connect(control_unit.position_update_ready, position_update_ready.i)
connect(control_unit.double_buffer, double_buffer.i)

# phase 1 inputs
connect(double_buffer.o, phase1.CTL_DOUBLE_BUFFER)
connect(force_evaluation_ready.o, phase1.CTL_FORCE_EVALUATION_READY)


# phase 2 inputs
connect(velocity_update_ready.o, phase2.CTL_VELOCITY_UPDATE_READY)

# phase 3 connections
connect(double_buffer.o, phase3.CTL_DOUBLE_BUFFER)
connect(position_update_ready.o, phase3.CTL_POSITION_UPDATE_READY)

# p_muxes inputs
for mux in concat(p_imuxes, p_omuxes):
    connect(position_update_ready.o, mux.ctl_position_update_ready)
    connect(force_evaluation_ready.o, mux.ctl_force_evaluation_ready)

# a_muxes inputs
for mux in concat(a_imuxes, a_omuxes):
    connect(force_evaluation_ready.o, mux.ctl_force_evaluation_ready)
    connect(velocity_update_ready.o, mux.ctl_velocity_update_ready)

# v_muxes inputs
for mux in concat(v_imuxes, v_omuxes):
    connect(velocity_update_ready.o, mux.ctl_velocity_update_ready)
    connect(position_update_ready.o, mux.ctl_position_update_ready)

# simulation initialization
T = 5 # number of timesteps

L = CUTOFF * UNIVERSE_SIZE # length of one dimension of the simulation box
N = 80 * N_CELL # number of particles in the simulation

cidx = [0 for _ in range(N_CELL)] # index into contents of each p_cache
for _ in range(N):
        r = (L*random(), L*random(), L*random())
        idx = linear_idx( # find which p_cache this particle must go into
            *[floor(x/CUTOFF) for x in r]
        )
        p_caches[idx].contents[cidx[idx]] = r
        cidx[idx] += 1
breakpoint()

p_omuxes[0].verbose = True

t = 0
while control_unit.t < 3:
    print("==== NEXT CYCLE ====")
    t += 1
    if t == 10:
        breakpoint()
    m.clock()
