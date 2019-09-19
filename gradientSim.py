# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import

import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import numpy as np
import random
import math

fig = plt.figure()
ax = fig.gca(projection='3d')

# Make data.
x = np.arange(-1, 1, 0.01)
y = np.arange(-1, 1, 0.01)

X, Y = np.meshgrid(x, y)

pos = np.array((0, 0))

radius = 0.5

mag = (4*3.1415/3) * radius*radius*radius * np.array((3, -11))


def getPotential(x, y):
    r = pos - np.array([x, y])
    return np.linalg.norm((r*mag)/(r*r*r))

potential = []
for i in range(len(X)):
    potential.append([])
    for j in range(len(Y)):
        potential[i].append(math.log(getPotential(x[i], y[j]), 10))

potential = np.array(potential)
# Plot the surface.
surf = ax.plot_surface(X, Y, potential, cmap=cm.coolwarm, linewidth=0, antialiased=False,)

plt.show()