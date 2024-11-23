import matplotlib.pyplot as plt
import numpy as np
import csv

import sys

with open(sys.argv[1],newline='') as fp:
    measurements = [d for d in csv.DictReader(fp)]

fig = plt.figure()
ax = fig.add_subplot(projection="3d")
x = np.array([int(m['N_PPAR']) for m in measurements])
y = np.array([int(m['N_CPAR']) for m in measurements])
z = np.array([m['cycles_total'] for m in measurements],dtype=float)

dz = z / 1000
bottom = np.zeros_like(z)
dx = np.ones_like(x)
dy = np.ones_like(y)
ax.bar3d(x,y,bottom,dx,dy,dz,shade=True)
ax.set_xlabel("Particle parallelism")
ax.set_ylabel("Cell parallelism")
ax.set_zlabel("Thousands of cycles per timestep")
plt.savefig('scaling.png')

n = np.multiply(x,y)
print(np.corrcoef(1/n,z))
