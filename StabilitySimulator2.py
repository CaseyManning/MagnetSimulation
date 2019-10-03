import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from Magnet import Magnet
import math
import itertools

def partialX(mag1, mag2):
    return 5

def partialY(mag1, mag2):
    return 5

def partialZ(mag1, mag2):
    return 5

def partial1X(m0, m1):
    r = np.linalg.norm(m0.position, m1.position)
    m0 = m0.moment
    m1 = m1.moment
    p1 = -m1[0] * (1/(4 * np.pi)) * (3*(-2*m0[0] * math.pow(r[0], 3) - 4 * math.pow(r[0], 2) * m0[1] * r[1] - 4 * math.pow(r[0], 2) * m0[2] * r[2] + 3*m0[0] * r[0] + math.pow(r[1], 2) + 3 * m0[0] * r[0] * math.pow(r[2], 2) + m0[1] * math.pow(r[1], 3) + m0[1] * r[1] * math.pow(r[2], 2) + math.pow(r[2], 2)*m0[2] * r[2] + m0[2] * math.pow(r[2], 3))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2))
    p2 = -m1[1] * (1/(4 * np.pi)) * (3 * (r[0]*r[0]*r[0] *m0[1] - 4*m0[0]*r[0]*r[0]*r[1] + r[0]*m0[1]*r[2]*r[2] - 4*r[0]*m0[1]*r[1]*r[1] - 5*r[0]*r[1]*m0[2]*r[2] + m0[0]*r[1]*r[1]*r[1] + m0[0]*r[1]*r[2]*r[2]) / (math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)))
    p3 = -m1[2] * (1/(4 * np.pi)) * ((3*(r[0]*r[0]*r[0]*m0[2] - 4*m0[0]*r[0]*r[0]*r[2] + r[0]*r[0]*r[0]*m0[2] - 4*r[0]*m0[2]*r[2]*r[2] - 5*r[0]*m0[1]*r[1]*r[2] + m0[0]*r[2]*r[2]*r[2] + m0[0]*r[1]*r[1]*r[2])) / (math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7.2)))
    return p1 + p2 + p3

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

class MagnetSimulator:

    posPartials = [partialX, partialY, partialZ]
    MomentPartials = [partial1X, partial1Y, partial1Z, partial2X, partial2Y, partial2Z]

    threshold = 0.1
    distThreshold = 0.1

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

    def pointsTowardsMagnet(self, partialVector, magnet):
        avgVec = np.array([0, 0, 0])
        for magnet2 in magnets:
            if (not magnet == magnet2) and np.linalg.norm(magnet.position - magnet2.position) < self.distThreshold:
                vec = magnet2.position - magnet.position
                v_hat = vec / (vec**2).sum()**0.5
                avgVec += v_hat
                p_hat = partialVector / (partialVector**2).sum()**0.5
                if np.linalg.norm(p_hat - v_hat) < self.threshold:
                    return true
        
        avgVec /= len(magnets)
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

    def run(self):
        energy = self.getPotentialEnergy(self.magnets)

        for mag1 in magnets:
            partialPos = np.array([0, 0, 0])
            partialRot = np.array([0, 0, 0])
            for mag2 in magnets:
                if not mag1 == mag2:
                    partialPos += np.array([partial(mag1, mag2) for partial in self.posPartials])
                    partialRot += np.array([partial(mag1, mag2) for partial in self.rotPartials])
            
            if not self.pointsTowardsMagnet(partialPos, magnet1):
                print("Unstable magnet")

            if np.linalg.norm(partialRot) < self.threshold:
                print("Unstable magnet")

        
        print(energy)

#378.94 * the orientation


if __name__ == "__main__":
    magnet1 = Magnet(np.array([100, 0, 0]), 0.003175, np.array([0, 0, 0]))
    magnet2 = Magnet(np.array([100, 0, 0]), 0.003175, np.array([-1, 0, 0]))
    magnets = [magnet1, magnet2]
    sim = MagnetSimulator(magnets)
    sim.run()