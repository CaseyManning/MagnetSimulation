import numpy as np
import math

mu0 = 4*np.pi*math.pow(10, -7) # Magnetic constant (H/m)

mui = 1.05 #permiability of the SM in the medium, (Steirman et al., 2019)

mue = 1.257 * math.pow(10, -6) #permiability of the medium, approximated to the permiability of free space

H0 = np.array([0, 0, 0]) #Uniform external magnetic field

Br = 1.1 # (1.5?) Residual Flux Density of neodymium (Wikipedia et al., 2019)

class Magnet:

    radius = 0.003175
    
    def getMomentOld(self, magnetization):
        return (4*np.pi / 3) * np.power(self.radius, 3) * magnetization

    def getMagnetization(self, baseMagnetization):
        return ((3 * (mui - mue))/(mui + 2*mue)) * H0 + (3*mu0*baseMagnetization)/(mui + 2*mue) #Equation 4

    def getMomentRemanence(self):
        return (1/mue) * Br * ((4/3) * np.pi * math.pow(self.radius, 3)) * (self.magnetization / (self.magnetization**2).sum()**0.5)

    def __init__(self, baseMagnetization, radius, position, color='r'):
        self.magnetization = self.getMagnetization(baseMagnetization)
        self.radius = radius
        self.position = position
        self.moment = self.getMomentRemanence()#self.getMoment(self.magnetization)
        self.color = color

    def __eq__(self, other):
        return self.position[0] == other.position[0] and self.position[1] == other.position[1] and self.position[2] == other.position[2]

    def __hash__(self):
        return hash(str(self.position[0]) + "aaa" + str(self.position[1]) + str(self.position[2]))