from common import *

from matplotlib import pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from math import floor
import os

N_PARTICLE = os.path.getsize("records/t1")

T = 1
while os.path.exists(f"records/t{T+1}"):
    T += 1
fig = plt.figure()
ax = fig.add_subplot(projection='3d')

def gen(t, *fargs):
    fp = open(f"records/t{t+1}","rb")
    p = np.zeros((3,N_PARTICLE))
    i = 0
    while True:
        buf = fp.read(24)
        if len(buf) < 24:
            break 
        if i % 100 == 0:
            print(f"Drawing frame {t}, particle {i}")
        p[:,i] = np.frombuffer(buf)
        i += 1
    return np.array_split(p,3)    
        

def update(t, ax):
    ax.clear()
    xs, ys, zs = gen(t)
    ax.scatter(xs, ys, zs, c="b")

# Setting the axes properties
ax.set_xlim3d([0.0, 1.0])
ax.set_xlabel('X')

ax.set_ylim3d([0.0, 1.0])
ax.set_ylabel('Y')

ax.set_zlim3d([0.0, 1.0])
ax.set_zlabel('Z')

ani = animation.FuncAnimation(fig, update, T, fargs=(ax,), interval=floor(DT*1000), repeat_delay=5000, blit=False)
ani.save('md.gif', writer='imagemagick')
plt.show()
