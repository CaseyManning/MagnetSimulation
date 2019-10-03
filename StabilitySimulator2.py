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

def partial1X(m1, m0):
    r = (5,5,5)
    return -m1.x * (1/(4 * np.pi)) * (3*(-2*m0.x * math.pow(r.x, 3) - 4 * math.pow(r.x, 2) * m0.y * r.y - 4 * math.pow(r.x, 2) * m0.z * r.z + 3*m0.x * r.x + math.pow(r.y, 2) + 3 * m0.x * r.x * math.pow(r.z, 2) + m0.y * math.pow(r.y, 3) + m0.y * r.y * math.pow(r.z, 2) + math.pow(r.z, 2)*m0.z * r.z + m0.z * math.pow(r.z, 3))/math.pow(math.pow(r.x, 2) + math.pow(r.y, 2) + math.pow(r.z, 2), 7.2))

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