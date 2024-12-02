from common import *
from verify import compute_timestep
import verify

class dummy:
    _double_buffer = 0

verify.CONTROL_UNIT = dummy()

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
    positions, velocities = compute_timestep(positions, velocities)
    
    with open(f'records/t{t}','wb') as fp:
        for contents in positions:
            for particle in contents:
                fp.write(particle.tobytes())
