import numpy as np
import math

mu0 = 4*np.pi*math.pow(10, -7) # Magnetic constant (H/m)

mui = 1.05 #permiability of the SM in the medium, (Steirman et al., 2019)

mue = 1.257 * math.pow(10, -6) #permiability of the medium, approximated to the permiability of free space

H0 = np.array([0, 0, 0]) #Uniform external magnetic field

class Magnet:
    
    def getMoment(self, magnetization):
        return (4*np.pi / 3) * np.power(self.radius, 3) * magnetization

    def getMagnetization(self, baseMagnetization):
        return ((3 * (mui - mue))/(mui + 2*mue)) * H0 + (3*mu0*baseMagnetization)/(mui + 2*mue) #Equation 4

    def __init__(self, baseMagnetization, radius, position):
        self.magnetization = self.getMagnetization(baseMagnetization)
        self.radius = radius
        self.position = position
        self.moment = self.getMoment(self.magnetization)
