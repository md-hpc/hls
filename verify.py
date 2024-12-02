from random import random, seed
from math import floor
import math
from math import inf

from common import *
import phase1

CONTROL_UNIT = None

filter_inputs = set()
filter_expect = set()

pipeline_inputs = set()
pipeline_expect = set()

target_positions = None

def offst():
    return CONTROL_UNIT._double_buffer * DBSIZE

def compute_timestep(positions, velocities):
    accelerations = [[numpy.zeros_like(r) for r in cell] for cell in positions]
    for cell_r in range(N_CELL):
        print(f"Computing direct {cell_r}")
        for addr_r, reference in enumerate(positions[cell_r]):
            for cell_n in neighborhood(cell_r, full=True):
                for addr_n, neighbor in enumerate(positions[cell_n]):
                    f = lj(reference,neighbor)
                    accelerations[cell_r][addr_r] = accelerations[cell_r][addr_r] + f * DT
                    
                    r = Acceleration(cell = cell_r, addr = addr_r + offst(), a = reference)
                    n = Acceleration(cell = cell_n, addr = addr_n + offst(), a = neighbor)
                    pi = pair_ident(r,n)
                    
                    if n3l_cell(cell_r, cell_n):
                        filter_expect.add(pi)    
                    
                    if norm(modr(reference, neighbor)) >= CUTOFF or cell_r == cell_n and addr_r == addr_n:
                        continue
                    pipeline_expect.add(pi)

                    
    new_positions = [[] for _ in range(N_CELL)]
    new_velocities = [[] for _ in range(N_CELL)]

    for cell in range(N_CELL):
        for addr, _ in enumerate(accelerations[cell]):
            velocities[cell][addr] += accelerations[cell][addr] * DT
            new_position = (positions[cell][addr] + velocities[cell][addr] * DT) % L
            new_cell = cell_from_position(new_position)
            new_positions[new_cell].append(new_position)
            new_velocities[new_cell].append(velocities[cell][addr])

    positions = new_positions
    velocities = new_velocities
    return positions, velocities

def extract_contents(caches, indicies = False, double_buffer = None):
    return [[[i,x.copy()] if indicies else x.copy() for i,x in enumerate(cache.contents[offst():offst()+DBSIZE]) if x is not NULL] for cache in caches]

def verify_emulator():
    global target_positions
    
    positions = extract_contents(p_caches)
    velocities = extract_contents(v_caches)
  
    for pi in filter_expect:
        r, n = pi_to_p(pi)
        print(f"expected {r} {n}")
    if len(filter_expect):
        print(f"Filter banks from last timestep did not recieve all expected inputs. {len(filter_expect)} missing")
        exit(1)

    for pi in pipeline_expect:
        r, n = pi_to_p(pi)
        print(f"expected {r} {n}")
    if len(pipeline_expect):
        print(f"Force pipelines from last timestep did not recieve all expected inputs. {len(pipeline_expect)} missing")
        exit(1)

    n_particle = sum([sum([r is not NULL for r in cache.contents[offst():offst()+DBSIZE]]) for cache in p_caches])
    if n_particle != N_PARTICLE:
        print(f"Particle count has changed from {N_PARTICLE} to {n_particle}")
        exit(1)
    
    max_err = -inf
    if target_positions is not None:
        passed = True
        for cell, P in enumerate(positions):
            for addr_r, r in enumerate(P):
                matched = False
                min_err = inf
                for T in target_positions:
                    for addr_t, t in enumerate(T):
                        err = norm(modr(r, t))/(norm(t)+ERR_TOLERANCE)
                        if err < min_err:
                            min_err = err
                        if err < ERR_TOLERANCE:
                            matched = True
                            T.pop(addr_t)
                            break
                    if matched:
                        break
                if not matched:
                    print(f"{cell}, {addr_r} could not be matched. Min err was {min_err}")
                    passed = False
                if min_err > max_err:
                    max_err = min_err
        if not passed:
            print("Positions are incorrect")
            exit(1)

    pipeline_inputs.clear()
    filter_inputs.clear()

    target_positions, _ = compute_timestep(positions, velocities)

    return max_err

for cp in phase1.compute_pipelines:
    for f in cp.filter_bank:
        f.input_set = filter_inputs
        f.input_expect = filter_expect
    cp.force_pipeline.input_set = pipeline_inputs
    cp.force_pipeline.input_expect = pipeline_expect
