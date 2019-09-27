# This import registers the 3D projection, but is otherwise unused.
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import 

import matplotlib.pyplot as plt
import numpy as np
from Magnet import Magnet
from StabilitySimulator2 import MagnetSimulator
# Fixing random state for reproducibility
np.random.seed(19680801)

m = MagnetSimulator([])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_zscale("log")

# xs = []
# ys = []
# zs = []
# for i in range(-100, 100, 5):
#     for j in range(-100, 100, 5):
#         xs.append(i)
#         ys.append(j)
#         magnet1 = Magnet(np.array([i, j, 0]), 0.003175, np.array([0, 0, 0]))
#         zs.append(np.linalg.norm(magnet1.magnetization))

# print(zs)
# ax.scatter(xs, ys, zs)

# ax.set_xlabel('Base Magnetization X')
# ax.set_ylabel('Base Magnetization Y')
# ax.set_zlabel('Magnetization')

# xs = []
# ys = []
# zs = []
# magnet1 = Magnet(np.array([0, 100, 0]), 0.003175, np.array([0, 0, 0]))

# for i in range(-10, 10, 1):
#     for j in range(-10, 10, 1):
#         xs.append(i/2)
#         ys.append(j/2)
#         magnet2 = Magnet(np.array([100, 100, 0]), 0.003175, np.array([i/2, j/2, 0]))
#         zs.append(m.getPotentialEnergy([magnet1, magnet2]))

# print(zs)
# ax.scatter(xs, ys, zs)


# ax.set_xlabel('Magnet 2 X Position')
# ax.set_ylabel('Magnet 2 Y Position')
# ax.set_zlabel('System Potential Energy')

# plt.show()

t = np.linspace(-20,20,100)
s = np.zeros(len(t))

magnet1 = Magnet(np.array([1, 0, 0]), 0.003175, np.array([0, 0, 0]))

print(s.shape)
for i in range(len(t)):
    magnet2 = Magnet(np.array([1, 0, 0]), 0.003175, np.array([0, 0, t[i]]))
    s[i] = m.getPotentialEnergy([magnet1, magnet2])

fig, ax = plt.subplots()
ax.plot(t, s)

t2 = np.linspace(-20,20,100)
s2 = np.zeros(len(t))

magnet1 = Magnet(np.array([1, 0, 0]), 0.003175, np.array([0, 0, 0]))

print(s.shape)
for i in range(len(t2)):
    magnet2 = Magnet(np.array([-1, 0, 0]), 0.003175, np.array([0, 0, t2[i]]))
    s2[i] = m.getPotentialEnergy([magnet1, magnet2])

ax.plot(t2, s2)

ax.set(xlabel='Pos', ylabel='Potential',
       title='')


print(t)
plt.show()

print(s)

magnet1 = Magnet(np.array([100, 100, 100]), 0.003175, np.array([0, 0, 0]))