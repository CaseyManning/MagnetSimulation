import numpy as np
from sympy import *
import matplotlib.pyplot as plt
from Magnet import Magnet
import math

class MagnetSimulator:

    def partialM0x(self, mag1, mag2):
        return 5
    
    def partialM0y(self, mag1, mag2):
        return 5
    
    def partialM0z(self, mag1, mag2):
        return 5
    
    def partialM1x(self, mag1, mag2):
        return 5

    def partialM1x(self, mag1, mag2):
        return 5

    def partialM1x(self, mag1, mag2):
        return 5


    rotPartials = [partialM0x]
    posPartials = [partialM0x]

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

    def run(self):
        energy = self.getPotentialEnergy(self.magnets)

        for mag1 in magnets:
            for mag2 in magnets:
                if not mag1 == mag2:
                    for partial in self.rotPartials:
                        if not partial(mag1, mag2) == 0:
                            print('Not stable :\'o(')
                    posPartialDerVec = [partial(mag1, mag2) for partial in self.posPartials]
        
        print(energy)

#378.94 * the orientation


if __name__ == "__main__":
    magnet1 = Magnet(np.array([100, 0, 0]), 0.003175, np.array([0, 0, 0]))
    magnet2 = Magnet(np.array([100, 0, 0]), 0.003175, np.array([-1, 0, 0]))
    magnets = [magnet1, magnet2]
    sim = MagnetSimulator(magnets)
    sim.run()