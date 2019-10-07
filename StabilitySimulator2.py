from matplotlib.widgets import Slider, Button, RadioButtons
import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from Magnet import Magnet
import math
import itertools
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from itertools import product, combinations
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d


def partialX(m0, m1):
    r = m0.position - m1.position
    m0 = m0.moment
    m1 = m1.moment
    p1 = -m1[0] * (1/(4 * np.pi)) * (3*(-2*m0[0] * math.pow(r[0], 3) - 4 * math.pow(r[0], 2) * m0[1] * r[1] - 4 * math.pow(r[0], 2) * m0[2] * r[2] + 3*m0[0] * r[0] + math.pow(r[1], 2) + 3 * m0[0] * r[0] * math.pow(r[2], 2) + m0[1] * math.pow(r[1], 3) + m0[1] * r[1] * math.pow(r[2], 2) + math.pow(r[2], 2)*m0[2] * r[2] + m0[2] * math.pow(r[2], 3))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2))
    p2 = -m1[1] * (1/(4 * np.pi)) * (3 * (r[0]*r[0]*r[0] *m0[1] - 4*m0[0]*r[0]*r[0]*r[1] + r[0]*m0[1]*r[2]*r[2] - 4*r[0]*m0[1]*r[1]*r[1] - 5*r[0]*r[1]*m0[2]*r[2] + m0[0]*r[1]*r[1]*r[1] + m0[0]*r[1]*r[2]*r[2]) / (math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)))
    p3 = -m1[2] * (1/(4 * np.pi)) * ((3*(r[0]*r[0]*r[0]*m0[2] - 4*m0[0]*r[0]*r[0]*r[2] + r[0]*r[0]*r[0]*m0[2] - 4*r[0]*m0[2]*r[2]*r[2] - 5*r[0]*m0[1]*r[1]*r[2] + m0[0]*r[2]*r[2]*r[2] + m0[0]*r[1]*r[1]*r[2])) / (math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)))
    return p1 + p2 + p3


def partialY(m0, m1):
    r = m0.position - m1.position
    m0 = m0.moment
    m1 = m1.moment
    p1 = -m1[0] * (1/(4 * np.pi)) * (3 * (r[0]*r[0]*r[0] + m0[1] - 4*m0[0]*r[0]*r[0]*r[1] + r[0]*m0[1]*r[2]*r[2] - 4*r[0]*m0[1]*r[1]*r[1] - 5*r[0]*r[1]*m0[2]*r[2] + m0[0] * r[1]*r[1]*r[1] + m0[0]*r[1]*r[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)
    p2 = -m1[1] * (1/(4 * np.pi)) * (3 * (m0[0]*r[0]*r[0]*r[0] + 3*r[0]*r[0]*m0[1]*r[1] + r[0]*r[0]*m0[2]*r[2]+m0[0]*r[0]*r[2]*r[2] - 4*m0[0]*r[0]*r[1]*r[1] + 3*m0[1]*r[1]*r[2]*r[2] + m0[2]*r[2]*r[2]*r[2] - 2*m0[1]*r[1]*r[1]*r[1] - 4*r[1]*r[1]*m0[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)
    p3 = -m1[2] * (1/(4 * np.pi)) * (3 * (r[1]*r[1]*r[1]*m0[2] - 4*m0[1]*r[1]*r[1]*r[2] + r[0]*r[0]*r[1]*m0[2] - 4*r[1]*m0[2]*r[2]*r[2] - 5*m0[0]*r[0]*r[1]*r[2] + r[0]*r[0]*m0[1]*r[2] + m0[1]*r[2]*r[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)
    return p1 + p2 + p3


def partialZ(m0, m1):
    r = m0.position - m1.position
    m0 = m0.moment
    m1 = m1.moment
    p1 = -m1[0] * (1/(4 * np.pi)) * (3 * (r[0]*r[0]*r[0]*m0[2] - 4*m0[0]*r[0]*r[0]*r[2] + r[0]*r[1]*r[1]*m0[2] - 4*r[0]*m0[2]*r[2]*r[2] - 5*r[0]*m0[1]*r[1]*r[2] + m0[0]*r[2]*r[2]*r[2] + m0[0]*r[1]*r[1]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)
    p2 = -m1[1] * (1/(4 * np.pi)) * (3 * (r[1]*r[1]*r[1]*m0[2] - 4*m0[1]*r[1]*r[1]*r[2] + r[0]*r[0]*r[1]*m0[2] - 4*r[1]*m0[2]*r[2]*r[2] - 5*m0[0]*r[0]*r[1]*r[2] + r[0]*r[0]*m0[1]*r[2] + m0[1]*r[2]*r[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)
    p3 = -m1[2] * (1/(4 * np.pi)) * (3 * (m0[0]*r[0]*r[0]*r[0] + 3*r[0]*r[0]*m0[2]*r[2] + r[0]*r[0]*m0[1]*r[1] + m0[0]*r[0]*r[1]*r[1] - 4*m0[0]*r[0]*r[2]*r[2] + m0[1]*r[1]*r[1]*r[1] + 3*r[1]*r[1]*m0[2]*r[2] - 4*m0[1]*r[1]*r[2]*r[2] - 2*m0[2]*r[2]*r[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)
    return p1 + p2 + p3


def partial1X(mag1, mag2):
    return 5

def partial1Y(mag1, mag2):
    return 5

def partial1Z(mag1, mag2):
    return 5

def partial2X(mag1, mag2):
    return 5

def partial2Y(mag1, mag2):
    return 5

def partial2Z(mag1, mag2):
    return 5

class Arrow3D(FancyArrowPatch):

    def __init__(self, xs, ys, zs, *args, **kwargs):
        FancyArrowPatch.__init__(self, (0, 0), (0, 0), *args, **kwargs)
        self._verts3d = xs, ys, zs

    def draw(self, renderer):
        xs3d, ys3d, zs3d = self._verts3d
        xs, ys, zs = proj3d.proj_transform(xs3d, ys3d, zs3d, renderer.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        FancyArrowPatch.draw(self, renderer)

class MagnetSimulator:

    posPartials = [partialX, partialY, partialZ]
    MomentPartials = [partial1X, partial1Y, partial1Z, partial2X, partial2Y, partial2Z]

    threshold = 0.1
    distThreshold = 100

    def __init__(self, magnets):
        self.magnets = magnets

    def getPotentialEnergy(self, magnets):
        totalPotential = 0
        for mag1 in magnets:
            for mag2 in magnets:
                if not mag1 == mag2:
                    moment1 = mag1.moment
                    x = Symbol('x')
                    moment2 = mag2.moment
                    r = np.linalg.norm(mag1.position - mag2.position)

                    totalPotential += np.dot(-1*moment2, 1/(4*np.pi)*((3*np.dot(moment1, r)*r)/math.pow(r, 5) - (moment1/math.pow(r, 3))))
        return totalPotential

    def getPotentialEnergy(self, magnets):
        totalPotential = 0
        for mag1 in magnets:
            for mag2 in magnets:
                if not mag1 == mag2:
                    moment1 = mag1.moment
                    x = Symbol('x')
                    moment2 = mag2.moment
                    r = np.linalg.norm(mag1.position - mag2.position)

                    totalPotential += np.dot(-1*moment2, 1/(4*np.pi)*((3*np.dot(moment1, r)*r)/math.pow(r, 5) - (moment1/math.pow(r, 3))))
        return totalPotential

    def pointsTowardsMagnet(self, partialVector, magnet):
        avgVec = np.array([0.0, 0.0, 0.0]) #Center of mass of connected magnets
        for magnet2 in magnets:
            if (not magnet == magnet2) and np.linalg.norm(magnet.position - magnet2.position) < self.distThreshold:
                vec = magnet2.position - magnet.position
                v_hat = vec / (vec**2).sum()**0.5
                avgVec += v_hat
                p_hat = partialVector / (partialVector**2).sum()**0.5
                if np.linalg.norm(p_hat - v_hat) < self.threshold:
                    return true
        
        print('Average vector of colliding magnets: ' + str(avgVec))
        avgVec = np.true_divide(avgVec, len(magnets))
        p_hat = partialVector / (partialVector**2).sum()**0.5
        if np.linalg.norm(p_hat - avgVec) < self.threshold:
            return true
                
                # for magnet3 in magnets:
                #     if (not (magnet3 == magnet or magnet3 == magnet2)) and np.linalg.norm(magnet.position - magnet3.position) < self.distThreshold:
                #         vec1 = magnet2.position - magnet.position
                #         vec2 = magnet3.position - magnet.position
                #         avgVec = (vec1 + vec2) / 2
                #         avgVec_hat = avgVec / (avgVec**2).sum()**0.5
                #         p_hat = partialVector / (partialVector**2).sum()**0.5
                #         if np.linalg.norm(p_hat - avgVec_hat) < self.threshold:
                #             return true
        return false


    def draw(self, partials):

        fig = plt.figure()
        ax = fig.gca(projection='3d')
        # ax.set_aspect("equal")

         # draw sphere
        for i in range(len(magnets)):
            u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
            x = np.cos(u)*np.sin(v)*self.magnets[i].radius + self.magnets[i].position[0]
            y = np.sin(u)*np.sin(v)*self.magnets[i].radius + self.magnets[i].position[1]
            z = np.cos(v)*self.magnets[i].radius + self.magnets[i].position[2]
            ax.plot_wireframe(x, y, z, color=self.magnets[i].color)

            # draw a vector
            partials[i] = (partials[i] / (partials[i]**2).sum()**0.5)/50
            gx = partials[i][0]
            gy = partials[i][1]
            gz = partials[i][2]

            px = self.magnets[i].position[0]
            py = self.magnets[i].position[1]
            pz = self.magnets[i].position[2]

            a = Arrow3D([px, px+gx], [py, py+gy], [pz, pz+gz], mutation_scale=20, lw=2, arrowstyle="-|>", color="k")
            ax.add_artist(a)
            print("Displaying a Gradient Vector")

            # draw a vector
            moment = (self.magnets[i].moment / (self.magnets[i].moment**2).sum()**0.5)/50
            gx = moment[0]
            gy = moment[1]
            gz = moment[2]
            a = Arrow3D([px, px+gx], [py, py+gy], [pz, pz+gz], mutation_scale=20, lw=2, arrowstyle="-|>", color=self.magnets[i].color)
            ax.add_artist(a)

            # if np.linalg.norm(partialRot) < self.threshold:
            #     print("Unstable magnet")
        # draw cube
        r = [-0.025, 0.025]
        for s, e in combinations(np.array(list(product(r, r, r))), 2):
            if np.sum(np.abs(s-e)) == r[1]-r[0]:
                ax.plot3D(*zip(s, e), color="k")
        plt.show()

    def run(self):
        # energy = self.getPotentialEnergy(self.magnets)
        partials = []

        for mag1 in magnets:
            partialPos = np.array([0.0, 0.0, 0.0])
            partialRot = np.array([0, 0, 0])
            for mag2 in magnets:
                if not mag1 == mag2:
                    partialPos += np.array([partialX(mag1, mag2), partialY(mag1, mag2), partialZ(mag1, mag2)])
                    # partialPos += np.array([partial(mag1, mag2) for partial in self.posPartials])
                    # partialRot += np.array([partial(mag1, mag2) for partial in self.rotPartials])
                    
            
            print("Magnet Partial: " + str(partialPos))
            partials.append(partialPos)
            # if not self.pointsTowardsMagnet(partialPos, magnet1):
            #     print("Unstable magnet")

        self.draw(partials)


if __name__ == "__main__":
    magnet1 = Magnet(np.array([-1, 1, 0]), 0.003175, np.array([0, 0, 0]), 'r')
    magnet2 = Magnet(np.array([-1, -1, 0]), 0.003175, np.array([0.003175*2, 0, 0]), 'g')
    magnet3 = Magnet(np.array([1, 0, 0]), 0.003175, np.array([0.003175, 0.005499261314, 0]), 'b')

    # magnet1 = Magnet(np.array([1, 0, 0]), 0.003175, np.array([0, 0, 0]), 'r')
    # magnet2 = Magnet(np.array([1, 0, 0]), 0.003175, np.array([0.003175*2, 0, 0]), 'g')

    magnets = [magnet1, magnet2, magnet3]
    sim = MagnetSimulator(magnets)
    sim.run()

# 0.003175*2 ^2 - 