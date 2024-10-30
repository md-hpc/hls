from import *

import phase1
import phase2
import phase3

from structures import *

from random import random

# making these constants so I won't make any typos
force_evaluation = "force evaluation"
velocity_update = "velocity update"
position_update = "position update"

class ControlUnit(Logic):
    def __init__(self):
        self.t = 0 # current timestep

        self.phase = force_evaluation
        
        self.phase1_force_evaluation_done = Input(self)
        self.phase2_force_evaluation_done = Input(self)
        self.phase2_velocity_update_done = Input(self)
        self.phase3_position_update_done = Input(self)

        self.filter_empty = Input(self)
        self.pipeline_empty = Input(self)

        self.force_evaluation_ready = Output(self)
        self.velocity_update_ready = Output(self)
        self.position_update_ready = Output(self)

        self.double_buffer_state = 0
        self.double_buffer = Output(self)

    def logic(self):
        if all([
            self.phase == force_evaluation, 
            self.phase1_force_evaluation_done.get(), 
            self.phase2_force_evaluation_done.get(),
            self.filter_empty.get(),
            self.pipeline_empty.get()
            ]):
            
            self.phase = velocity_update
        elif all([
            self.phase == velocity_update,
            self.phase2_velocity_update_done.get(),
            ]):

            self.phase = position_update
        elif all([
            self.phase == position_update,
            self.phase3_position_update_done.get(),
            ]):
            
            self.phase = force_evaluation
            self.double_buffer_state = 0 if self.double_buffer_state else 1
            self.t += 1

        self.double_buffer.set(self.double_buffer_state)
        self.force_evaluation.set(int(self.phase == force_evaluation))
        self.velocity_update.set(int(self.phase == velocity_update))
        self.position_update.set(int(self.phase == position_update))

double_buffer = m.add(Register())
force_evaluation = m.add(Register())
velocity_update = m.add(Register())
position_update = m.add(Register())

controlUnit = m.add(ControlUnit())

connect(controlUnit.force_evaluation, force_evaluation.i)
connect(controlUnit.velocity_update, velocity_update.i)
connect(controlUnit.position_update, position_update.i)
connect(controlUnit.double_buffer, double_buffer.i)

# phase 1 connections
connect(double_buffer.o, phase1.CTL_DOUBLE_BUFFER)

connect(force_evaluation.o, phase1.CTL_FORCE_EVALUATION)
connect(phase1.CTL_FORCE_EVALUATION_DONE, controlUnit.phase1_force_evaluation_done)

# phase 2 connections
connect(double_buffer.o, phase2.CTL_DOUBLE_BUFFER)

connect(force_evaluation.o, phase2.CTL_FORCE_EVALUATION_READY)
connect(phase2.CTL_FORCE_EVALUATION_DONE, controlUnit.phase2_force_evaluation_done)

connect(velocity_update.o, phase2.CTL_VELOCITY_UPDATE_DONE)
connect(phase2.CTL_VELOCITY_UPDATE_DONE, controlUnit.phase2_velocity_update_done)

# phase 3 connections
connect(double_buffer.o, phase3.CTL_DOUBLE_BUFFER)

connect(position_update.o, phase3.CTL_POSITION_UPDATE_READY)
connect(phase3.CTL_POSITION_UPDATE_DONE, controlUnit.phase3_position_update_done)

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


