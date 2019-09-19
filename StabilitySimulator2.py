import numpy as np
import matplotlib.pyplot as plt
from Magnet import Magnet
import math

class MagnetSimulator:

    def __init__(self, magnets):
        self.magnets = magnets

    def getPotentialEnergy(self, magnets):
        totalPotential = 0
        for mag1 in magnets:
            for mag2 in magnets:
                if not mag1 == mag2:
                    moment1 = mag1.moment
                    moment2 = mag2.moment
                    r = np.linalg.norm(mag1.position - mag2.position)

                    totalPotential += np.dot(-1*moment2, 1/(4*np.pi)*((3*np.dot(moment1, r)*r)/math.pow(r, 5) - (moment1/math.pow(r, 3))))
        return totalPotential

    def run(self):
        energy = self.getPotentialEnergy(self.magnets)

        print(energy)


        
if __name__ == "__main__":
    magnet1 = Magnet(np.array([100, 100, 100]), 0.003175, np.array([0, 0, 0]))
    magnet2 = Magnet(np.array([-200, 200, 200]), 0.003175, np.array([-1, 1, 1]))
    magnets = [magnet1, magnet2]
    sim = MagnetSimulator(magnets)
    sim.run()