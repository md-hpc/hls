from common import *
from random import random, seed
from math import floor
import math

N_PARTICLE = 1
T = 100
DT = 1e-2

P25 = floor(N_PARTICLE*.25)
P50 = floor(N_PARTICLE*.50)
P75 = floor(N_PARTICLE*.75)

def compute_targets(positions, velocities):
    accelerations = [[numpy.zeros_like(r) for r in cell] for cell in positions]
    for cell_r in range(N_CELL):
        for addr_r, reference in enumerate(positions[cell_r]):
           for cell_n in neighborhood(cell_r):
                for addr_n, neighbor in enumerate(positions[cell_n]):
                    if not n3l(reference,neighbor) or cell_n == cell_r and addr_n == addr_r:
                        continue
                    if norm(reference-neighbor) >= CUTOFF:
                        continue
                    f = lj(reference,neighbor)
                    accelerations[cell_r][addr_r] += f * DT
                    accelerations[cell_n][addr_n] += -1.0 * f * DT
    
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
    return positions, velocities, accelerations

if __name__ == "__main__":
    seed(SEED)
    positions = [[] for _ in range(N_CELL)]
    velocities = [[] for _ in range(N_CELL)]
    for _ in range(1):
        r = numpy.array((random() * L, random() * L, random() * L))
        cell = cell_from_position(r)
        positions[cell].append(r)
        velocities[cell].append(numpy.array([1., 0., 0.]))

    for t in range(T):
        positions, velocities, _ = compute_targets(positions, velocities)

        with open(f'records/t{t}','wb') as fp:
            for contents in positions:
                for particle in contents:
                    fp.write(particle.tobytes())

        
