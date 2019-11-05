import numpy as np
from . Magnet import Magnet
import math
import itertools

def partialX(m0, m1):
    r = m0.position - m1.position
    m0 = m0.moment
    m1 = m1.moment
    p1 = -m1[0] * (1/(4 * np.pi)) * (3*(-2*m0[0] * math.pow(r[0], 3) - 4 * math.pow(r[0], 2) * m0[1] * r[1] - 4 * math.pow(r[0], 2) * m0[2] * r[2] + 3*m0[0] * r[0] * math.pow(r[1], 2) + 3 * m0[0] * r[0] * math.pow(r[2], 2) + m0[1] * math.pow(r[1], 3) + m0[1] * r[1] * math.pow(r[2], 2) + math.pow(r[1], 2)*m0[2] * r[2] + m0[2] * math.pow(r[2], 3))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7/2))
    p2 = -m1[1] * (1/(4 * np.pi)) * (3 * (math.pow(r[0],3)*m0[1] - 4*m0[0]*math.pow(r[0],2)*r[1] + r[0]*m0[1]*math.pow(r[2],2) - 4*r[0]*m0[1]*math.pow(r[1],2) - 5*r[0]*r[1]*m0[2]*r[2] + m0[0]*math.pow(r[1],3) + m0[0]*r[1]*math.pow(r[2],2)) / (math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7/2)))
    p3 = -m1[2] * (1/(4 * np.pi)) * (3*(math.pow(r[0],3)*m0[2] - 4*m0[0]*math.pow(r[0],2)*r[2] + math.pow(r[0],3)*m0[2] - 4*r[0]*m0[2]*math.pow(r[2],2) - 5*r[0]*m0[1]*r[1]*r[2] + m0[0]*math.pow(r[2],3) + m0[0]*math.pow(r[1],2)*r[2])) / (math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7/2))
    return p1 + p2 + p3

def partialY(m0, m1):
    r = m0.position - m1.position
    m0 = m0.moment
    m1 = m1.moment
    p1 = -m1[0] * (1/(4 * np.pi)) * (3 * (math.pow(r[0],3) * m0[1] - 4*m0[0]*math.pow(r[0],2)*r[1] + r[0]*m0[1]*math.pow(r[2],2) - 4*r[0]*m0[1]*math.pow(r[1],2) - 5*r[0]*r[1]*m0[2]*r[2] + m0[0] * math.pow(r[1],3) + m0[0]*r[1]*math.pow(r[2],2)))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7/2)
    p2 = -m1[1] * (1/(4 * np.pi)) * (3 * (m0[0]*math.pow(r[0],3) + 3*math.pow(r[0],2)*m0[1]*r[1] + math.pow(r[0],2)*m0[2]*r[2]+m0[0]*r[0]*math.pow(r[2],2) - 4*m0[0]*r[0]*math.pow(r[1],2) + 3*m0[1]*r[1]*math.pow(r[2],2) + m0[2]*math.pow(r[2],3) - 2*m0[1]*math.pow(r[1],3) - 4*math.pow(r[1],2)*m0[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1],2) + math.pow(r[2], 2), 7/2)
    p3 = -m1[2] * (1/(4 * np.pi)) * (3 * (r[1]*r[1]*r[1]*m0[2] - 4*m0[1]*r[1]*r[1]*r[2] + r[0]*r[0]*r[1]*m0[2] - 4*r[1]*m0[2]*r[2]*r[2] - 5*m0[0]*r[0]*r[1]*r[2] + r[0]*r[0]*m0[1]*r[2] + m0[1]*r[2]*r[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1],2) + math.pow(r[2], 2), 7/2)
    return p1 + p2 + p3

def partialZ(m0, m1):
    r = m0.position - m1.position
    m0 = m0.moment
    m1 = m1.moment
    p1 = -m1[0] * (1/(4 * np.pi)) * (3 * (r[0]*r[0]*r[0]*m0[2] - 4*m0[0]*r[0]*r[0]*r[2] + r[0]*r[1]*r[1]*m0[2] - 4*r[0]*m0[2]*r[2]*r[2] - 5*r[0]*m0[1]*r[1]*r[2] + m0[0]*r[2]*r[2]*r[2] + m0[0]*r[1]*r[1]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7/2)
    p2 = -m1[1] * (1/(4 * np.pi)) * (3 * (r[1]*r[1]*r[1]*m0[2] - 4*m0[1]*r[1]*r[1]*r[2] + r[0]*r[0]*r[1]*m0[2] - 4*r[1]*m0[2]*r[2]*r[2] - 5*m0[0]*r[0]*r[1]*r[2] + r[0]*r[0]*m0[1]*r[2] + m0[1]*r[2]*r[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7/2)
    p3 = -m1[2] * (1/(4 * np.pi)) * (3 * (m0[0]*r[0]*r[0]*r[0] + 3*r[0]*r[0]*m0[2]*r[2] + r[0]*r[0]*m0[1]*r[1] + m0[0]*r[0]*r[1]*r[1] - 4*m0[0]*r[0]*r[2]*r[2] + m0[1]*r[1]*r[1]*r[1] + 3*r[1]*r[1]*m0[2]*r[2] - 4*m0[1]*r[1]*r[2]*r[2] - 2*m0[2]*r[2]*r[2]*r[2]))/math.pow(math.pow(r[0], 2) + math.pow(r[1], 2) + math.pow(r[2], 2), 7/2)
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

class MagnetSimulator:

    posPartials = [partialX, partialY, partialZ]
    rotPartials1 = [partial1X, partial1Y, partial1Z]
    rotPartials1 = [partial2X, partial2Y, partial2Z]

    threshold = 0.1
    distThreshold = 100

    def __init__(self, magnets):
        self.magnets = magnets

    def getPotentialEnergy(self, magnets):
        totalPotential = 0
        for mag1 in self.magnets:
            for mag2 in self.magnets:
                if not mag1 == mag2:
                    moment1 = mag1.moment
                    moment2 = mag2.moment
                    r = np.linalg.norm(mag1.position - mag2.position)

                    totalPotential += np.dot(-1*moment2, 1/(4*np.pi)*((3*np.dot(moment1, r)*r)/math.pow(r, 5) - (moment1/math.pow(r, 3))))
        return totalPotential

    def pointsTowardsMagnet(self, partialVector, magnet):
        avgVec = np.array([0.0, 0.0, 0.0]) #Center of mass of connected magnets
        for magnet2 in self.magnets:
            if (not magnet == magnet2) and np.linalg.norm(magnet.position - magnet2.position) < self.distThreshold:
                vec = magnet2.position - magnet.position
                v_hat = vec / (vec**2).sum()**0.5
                avgVec += v_hat
                p_hat = partialVector / (partialVector**2).sum()**0.5
                if np.linalg.norm(p_hat - v_hat) < self.threshold:
                    return True
        
        print('Average vector of colliding magnets: ' + str(avgVec))
        avgVec = np.true_divide(avgVec, len(self.magnets))
        p_hat = partialVector / (partialVector**2).sum()**0.5
        if np.linalg.norm(p_hat - avgVec) < self.threshold:
            return True
    
                # for magnet3 in magnets:
                #     if (not (magnet3 == magnet or magnet3 == magnet2)) and np.linalg.norm(magnet.position - magnet3.position) < self.distThreshold:
                #         vec1 = magnet2.position - magnet.position
                #         vec2 = magnet3.position - magnet.position
                #         avgVec = (vec1 + vec2) / 2
                #         avgVec_hat = avgVec / (avgVec**2).sum()**0.5
                #         p_hat = partialVector / (partialVector**2).sum()**0.5
                #         if np.linalg.norm(p_hat - avgVec_hat) < self.threshold:
                #             return true
        return False

    def run(self):
        # energy = self.getPotentialEnergy(self.magnets)
        partials = []
        rotPartials = {}

        for mag1 in self.magnets:
            partialPos = np.array([0.0, 0.0, 0.0])
            partialRot = np.array([0, 0, 0])
            for mag2 in self.magnets:
                if not mag1 == mag2:
                    partialPos += np.array([partial(mag1, mag2) for partial in self.posPartials])

                    rotPartials[mag1] = np.array([partial(mag1, mag2) for partial in self.rotPartials1])
                    rotPartials[mag2] = np.array([partial(mag1, mag2) for partial in self.rotPartials1])
                    
            partialPos = -partialPos
            print("Magnet Partial: " + str(partialPos))
            partials.append(partialPos)
            if not self.pointsTowardsMagnet(partialPos, mag1):
                mag1.color = 'r'
                print("Unstable magnet")

        # self.draw(partials)
        return partials

    def line(num, ldir, momentDir):
        colors = ['g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b']
        ret = []
        for i in range(num):
        
            ret.append(Magnet(np.array([1 if 'x' in momentDir else 0,  1 if 'y' in momentDir else 0,  1 if 'z' in momentDir else 0]), Magnet.radius, np.array([Magnet.radius*2*i if 'x' in ldir else 0, Magnet.radius*2*i if 'y' in ldir else 0, Magnet.radius*2*i if 'z' in ldir else 0]), colors[i]))

        return ret

    def grid(x, y, z, momentDir):
        colors = ['g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b']
        ret = []
        for i in range(x):
            for j in range(y):
                for k in range(z):
                    ret.append(Magnet(np.array([1 if 'x' in momentDir else 0,  1 if 'y' in momentDir else 0,  1 if 'z' in momentDir else 0]), Magnet.radius, np.array([Magnet.radius*2*i, Magnet.radius*2*j, Magnet.radius*2*k]), colors[i]))

        return ret


    def loop(num, counterclockwise):
        colors = ['g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b']
        loop = []
        for i in range(num): #OFF BY ONE ERROR HERE
            theta = (i*2*np.pi)/(num)
            print(theta)
            magnetDirection = np.array([-np.sin(theta), np.cos(theta),0])
            if counterclockwise == False:
                magnetDirection *= -1
            # posVecMag = Magnet.radius/(np.cos(theta/2))
            posVecMag = Magnet.radius/(np.sin(np.pi/num))
            magnetPosition = np.array([posVecMag*np.cos(theta), posVecMag*np.sin(theta),0])
            loop.append(Magnet(magnetDirection, Magnet.radius, magnetPosition, colors[i]))
        return loop
        

    def saddle():
        colors = ['g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b', 'g', 'b']
        saddle = [
            Magnet(np.array([1, 1, 0]), Magnet.radius, np.array([0, 0, 0]), colors[0]),
            Magnet(np.array([1, -1, 0]), Magnet.radius, np.array([Magnet.radius*2, 0, 0]), colors[1]),
            Magnet(np.array([-1, -1, 0]), Magnet.radius, np.array([Magnet.radius*2, Magnet.radius*2, 0]), colors[2]),
            Magnet(np.array([-1, 1, 0]), Magnet.radius, np.array([0, Magnet.radius*2, 0]), colors[3])
        ]
        return saddle

if __name__ == "__main__":
    
    mag1 = Magnet(np.array([-1, 0, 0]), Magnet.radius, np.array([0, 0, Magnet.radius*2]), 'b')
    mag2 = Magnet(np.array([1, 0, 0]), Magnet.radius, np.array([0, 0, 0]), 'b')

    # magnets = line(5, ldir='z', momentDir='x')
    # magnets = line(5,'x','y')
    # magnets = saddle()
    magnets = loop(6, True)
    sim = MagnetSimulator(magnets)
    sim.run()
