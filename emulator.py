from hls import *
from common import *
import sys
from random import random, seed, shuffle
from math import floor, inf
import hls
import numpy
import os

import compute_pipeline
from verify import compute_targets

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

# ==== VERIFICATION CODE ====
VERIFY_COMPUTED = False

# this is going to be computed in verify.py
expected_interactions = None 

# determine what inputs different logic units should expect to receive this timestep
def compute_expected():
    double_buffer = control_unit._double_buffer

    pipeline_inputs.clear()
    filter_inputs.clear()
    
    for pi in filter_expect:
        r, n = pi_to_p(pi)
        print(f"expected {r} {n}")
    if len(filter_expect):
        print(f"Filter banks from last timestep did not recieve all expected inputs. {len(filter_expect)} missing")
        exit()

    for pi in pipeline_expect:
        r, n = pi_to_p(pi)
        print(f"expected {r} {n}")
    if len(pipeline_expect):
        print(f"Pipelines from last timestep did not recieve all expected inputs. {len(pipeline_expect)} missing")
        exit()

    for cell_r, _ in enumerate(p_caches):
         for cell_n in neighborhood(cell_r):
            if not n3l(cell_r, cell_n):
                continue
            r_cache = p_caches[cell_r]
            n_cache = p_caches[cell_n]
            for addr_r, r in bram_enum(r_cache.contents, double_buffer):
                if r is NULL:
                    continue
                for addr_n, n in bram_enum(n_cache.contents, double_buffer):
                    if n is NULL:
                        continue
                    assert addr_n // DBSIZE == double_buffer
                    assert addr_r // DBSIZE == double_buffer
                    reference = Position(cell = cell_r, addr = addr_r, r = r)
                    neighbor = Position(cell = cell_n, addr = addr_n, r = n)
                    pi = pair_ident(reference,neighbor)
                    if pi in filter_expect:
                        print("duplicate in verify!!")
                        exit()
                    filter_expect.add(pi)
                    if norm(r-n) < CUTOFF and n3l(cell_r, cell_n) and n3l(r,n) and not (cell_r == cell_n and addr_r == addr_n):
                        pipeline_expect.add(pi)
    if False:
        passed = True
        for pi in pipeline_expect:
            if pi not in expected_interactions:
                print(f"unexpected by verify: {pi_to_p(pi)}")
                passed = False
            else:
                expected_interactions.remove(pi)
        for pi in expected_interactions:
            print(f"expected by verify: {pi_to_p(pi)}")
            passed = False
        if not passed:
            print("pipeline_expect does not expect same interactions as verify")
            exit(1)

    n_particle = sum([sum([r is not NULL for _, r in bram_enum(cache.contents, double_buffer)]) for cache in p_caches])
    if n_particle != N_PARTICLE:
        print(f"Particle count has changed from {N_PARTICLE} to {n_particle}")
        exit(1)
 
# compute what the positions, velocities, and accelerations should be based on a naive nested for-loop
target_positions = None
target_velocities = None
target_accelerations = None
VERIFY_COMPUTED = False

def extract_contents(caches, indicies = False, double_buffer = None):
    if double_buffer is None:
        double_buffer = control_unit._double_buffer
    return [[[i,x.copy()] if indicies else x.copy() for i,x in enumerate(cache.contents[db(double_buffer):db(double_buffer)+DBSIZE]) if x is not NULL] for cache in caches]

def compare_contents(caches, targets):
    computed = extract_contents(caches)
    targets = [t for t in concat(*targets)]

    passed = True
    for cell, C in enumerate(computed):
        for addr, c in enumerate(C):
            matched = False
            min_err = inf
            for t in targets:
                err = norm(c-t)/(norm(t)+1e-2)
                if err < min_err:
                    min_err = err
                if err < ERR_TOLERANCE:
                    matched = True
                    break
            if not matched:
                print(f"{cell}, {addr} could not be matched. Min err was {min_err}")
                passed = False        
    return passed

def phase1_handler():
    global target_positions, target_velocities, target_accelerations, expected_interactions
    if VERIFY_COMPUTED and not compare_contents(p_caches, target_positions):
        print("Positions are incorrect")
        exit(1)

    positions = extract_contents(p_caches)
    velocities = extract_contents(v_caches)
    
    target_positions, target_velocities, target_accelerations, expected_interactions = compute_targets(positions, velocities)

def phase2_handler():
    if VERIFY_COMPUTED and not compare_contents(a_caches, target_accelerations):
        print("Accelerations are incorrect")
        exit(1)

def phase3_handler():
    if VERIFY_COMPUTED and not compare_contents(v_caches, target_velocities):
        print("Velocities are incorrect")
        exit(1)

control_unit.phase1_handler = phase1_handler
control_unit.phase2_handler = phase2_handler
control_unit.phase3_handler = phase3_handler

# simulation initialization
clear_records()
seed(SEED)
cidx = [0 for _ in range(N_CELL)] # index into contents of each p_cache
for _ in range(N_PARTICLE):
        r = numpy.array([L*random(), L*random(), L*random()])
        idx = cell_from_position(r)
        p_caches[idx].contents[cidx[idx]] = r
        v_caches[idx].contents[cidx[idx]] = numpy.array([v0(), v0(), v0()])

        cidx[idx] += 1
target_positions = extract_contents(p_caches)
phase1_handler()
# compute_pipeline.VERBOSE = True

# compute_pipeline ensures the correctness of its own inputs using global state
filter_inputs = set()
filter_expect = set()
pipeline_inputs = set()
pipeline_expect = set()

for cp in phase1.compute_pipelines:
    for f in cp.filter_bank:
        f.input_set = filter_inputs
        f.input_expect = filter_expect
    cp.force_pipeline.input_set = pipeline_inputs
    cp.force_pipeline.input_expect = pipeline_expect
    #cp._reference.verbose = True
compute_expected()

t = 0
cycles_total = 0
t0 = control_unit.t
with numpy.errstate(all="raise"):
    while control_unit.t < T:
        if control_unit.t != t0:
            with open(f"records/t{control_unit.t-1}","wb") as fp:
                for cache in p_caches:
                    offst = ndb(control_unit._double_buffer)
                    for p in cache.contents[offst:offst+DBSIZE]:
                        if p is not NULL:
                            b = fp.write(p.tobytes())
                            assert b == 24, "uh oh"

            t0 = control_unit.t
            compute_expected()
            cycles_total += t
            t = 0
        print(f"CYCLE {control_unit.t}-{t}, {control_unit.phase}:", end=" ")
        m.clock()
        t += 1
       
save_pos(control_unit.t)
print(f"Emulator took {cycles_total} clock cycles to simulate {T} timesteps")
