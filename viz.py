from common import *

from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from math import floor
import os

T = 1
while os.path.exists(f"records/t{T+1}"):
    T += 1
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

def gen(t, *fargs):
    fp = open(f"records/t{t+1}","rb")
    p = np.zeros((3,N_PARTICLE))
    for i in range(N_PARTICLE):
        buf = fp.read(24)
        p[:,i] = np.frombuffer(buf)
    return np.array_split(p,3)    
        

def update(t, ax):
    ax.clear()
    xs, ys, zs = gen(t)
    ax.scatter(xs, ys, zs, c="r")

# Setting the axes properties
ax.set_xlim3d([0.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim3d([0.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, 1.0])
ax.set_zlabel('Z')

ani = animation.FuncAnimation(fig, update, T, fargs=(ax,), interval=floor(DT*1000), blit=False)
ani.save('md.gif', writer='imagemagick')
plt.show()
