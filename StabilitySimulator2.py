import magpylib as magpy
import numpy as np
import matplotlib.pyplot as plt
from Magnet import Magnet

class MagnetSimulator:

    def __init__(self, magnets):
        self.magnets = magnets

    def getPotentialEnergy(self, magnets):
        return 0

    def run(self):
        self.output = output
        self.increment = increment

        energy = self.getPotentialEnergy(self.magnets)

        print(energy)
        

    def getMagnetData(self):
        return ""

    def outputFrame(self):
        if self.output == 'csv':
            print(self.getMagnetData())
        elif self.output == 'print':
            print(self.getMagnetData())

        
if __name__ == "__main__":
    magnet1 = Magnet((1, 0, 0), 1/8, (0, 0, 0))
    magnet2 = Magnet((0, 1, 0), 1/8, (1, 1, -1))
    magnets = [magnet1, magnet2]
    sim = MagnetSimulator(magnets)
    sim.run()