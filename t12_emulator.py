from hls import *

import phase1
import phase2

from structures import *

from random import random

# making these constants so I won't make any typos
force_evaluation = "force evaluation"
velocity_update = "velocity update"
position_update = "position update"

class ControlUnit(Logic):
    def __init__(self):
        super().__init__("control unit")
        self.t = 0 # current timestep

        self.phase = force_evaluation
        
        self.force_evaluation_done = Input(self,"force-evaluation-done")
        self.velocity_update_done = Input(self,"velocity-update-done")
        self.position_update_done = Input(self,"position-update-done")

        self.filter_empty = [Input(self, "filter-empty-{i}") for i in range(N_FILTER)]
        self.force_pipeline_empty = Input(self,"force-pipeline-empty")

        self.force_evaluation_ready = Output(self,"force-evaluation-ready")
        self.velocity_update_ready = Output(self,"velocity-update-ready")
        self.position_update_ready = Output(self,"position-update-ready")

        self.double_buffer_state = 0
        self.double_buffer = Output(self,"double-buffer")

    def logic(self):
        if all([
            self.phase == force_evaluation, 
            self.force_evaluation_done.get(), 
            self.filter_empty.get(),
            self.pipeline_empty.get()
            ]):
            
            self.phase = velocity_update
        elif all([
            self.phase == velocity_update,
            self.velocity_update_done.get(),
            ]):

            self.phase = position_update
        elif all([
            self.phase == position_update,
            self.position_update_done.get(),
            ]):
            
            self.phase = force_evaluation
            self.double_buffer_state = 0 if self.double_buffer_state else 1
            self.t += 1

        self.double_buffer.set(self.double_buffer_state)
        self.force_evaluation.set(int(self.phase == force_evaluation))
        self.velocity_update.set(int(self.phase == velocity_update))
        self.position_update.set(int(self.phase == position_update))

# control_unit outputs need to be buffered in registers so we don't form cycles
# 
# we could equivalently buffer control_unit inputs, but it's not as easy to manage
double_buffer = m.add(Register("double-buffer"))
force_evaluation = m.add(Register("force-evaluation"))
velocity_update = m.add(Register("velocity-update"))
position_update = m.add(Register("position-update"))

control_unit = m.add(ControlUnit())

# control_unit inputs
connect(phase1.CTL_FORCE_EVALUATION_DONE, control_unit.force_evaluation_done)
connect(phase2.CTL_VELOCITY_UPDATE_DONE, control_unit.velocity_update_done)
connect(phase3.CTL_POSITION_UPDATE_DONE, control_unit.position_update_done)
for i in range(N_FILTER):
    connect(filter_bank[i].empty, control_unit.filter_empty[i])
connect(force_pipeline.empty, control_unit.force_pipeline_empty)

# register inputs
connect(control_unit.force_evaluation, force_evaluation.i)
connect(control_unit.velocity_update, velocity_update.i)
connect(control_unit.position_update, position_update.i)
connect(control_unit.double_buffer, double_buffer.i)

# phase 1 inputs
connect(double_buffer.o, phase1.CTL_DOUBLE_BUFFER)
connect(force_evaluation.o, phase1.CTL_FORCE_EVALUATION)


# phase 2 inputs
connect(velocity_update.o, phase2.CTL_VELOCITY_UPDATE_DONE)

# phase 3 connections
connect(double_buffer.o, phase3.CTL_DOUBLE_BUFFER)
connect(position_update.o, phase3.CTL_POSITION_UPDATE_READY)

# simulation initialization
T = 5 # number of timesteps

L = CUTOFF * UNIVERSE_SIZE # length of one dimension of the simulation box
N = 80 * N_CELL # number of particles in the simulation

cidx = [0 for _ in range(N_CELL)] # index into contents of each p_cache
for _ in range(N):
        r = (l*random(), l*random(), l*random())
        idx = linear_idx( # find which p_cache this particle must go into
            [floor(x/CUTOFF) for x in r]
        )
        p_caches.contents[cidx[idx]] = r
        cidx[idx] += 1


