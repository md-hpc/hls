from common import *
from random import random, seed
from math import floor
import math

def compute_targets(positions, velocities):
    interactions = set()

    accelerations = [[numpy.zeros_like(r) for r in cell] for cell in positions]
    for cell_r in range(N_CELL):
        for addr_r, reference in enumerate(positions[cell_r]):
            for cell_n in neighborhood(cell_r):
                for addr_n, neighbor in enumerate(positions[cell_n]):
                    if norm(reference-neighbor) >= CUTOFF:
                        continue
                    if cell_r == cell_n and addr_r == addr_n:
                        continue
                    if not n3l(reference,neighbor):
                        continue

                    f = lj(reference,neighbor)
                    accelerations[cell_r][addr_r] += f * DT
                    accelerations[cell_n][addr_n] -= f * DT 
                    r = Acceleration(cell = cell_r, addr = addr_r, a = None)
                    n = Acceleration(cell = cell_n, addr = addr_n, a = None)
                    interactions.add(pair_ident(r,n))

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
    return positions, velocities, accelerations, interactions

if __name__ == "__main__":
    T = 100
    DT = 1e-2

    P25 = floor(N_PARTICLE*.25)
    P50 = floor(N_PARTICLE*.50)
    P75 = floor(N_PARTICLE*.75)

    clear_records()
    seed(SEED)
    positions = [[] for _ in range(N_CELL)]
    velocities = [[] for _ in range(N_CELL)]
    for _ in range(N_PARTICLE):
        r = numpy.array((random() * L, random() * L, random() * L))
        v = numpy.array((v0(), v0(), v0())) 
        cell = cell_from_position(r)
        positions[cell].append(r)
        velocities[cell].append(v)

    for t in range(T):
        P = sum([sum(_v) for _v in velocities])/N_PARTICLE
        KE = sum([sum([norm(v)**2/2 for v in _v]) for _v in velocities])/N_PARTICLE
        print(f"Computing timestep {t}, KE={KE}, P={P}")
        positions, velocities, _, __ = compute_targets(positions, velocities)
        
        with open(f'records/t{t}','wb') as fp:
            for contents in positions:
                for particle in contents:
                    fp.write(particle.tobytes())

        
