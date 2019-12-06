MATPLOT = False

if __name__ == "__main__":
    MATPLOT = True

if MATPLOT:
    from matplotlib.widgets import Slider, Button, RadioButtons
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import proj3d
    from matplotlib.patches import FancyArrowPatch
    from Magnet import Magnet
else:
    from . Magnet import Magnet
import numpy as np
import math
import itertools
from itertools import product, combinations

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
    r = mag1.position - mag2.position
    m1 = mag1.moment
    squares = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    # 1/4pi factored out
    p1x = -m1[0] * ((3 * r[0] * r[0] - squares) / math.pow(squares, 5/2)) - m1[1] * ((3 * r[0] * r[1]) / math.pow(squares, 5/2)) - m1[2] * ((3 * r[0] * r[2]) / math.pow(squares, 5/2))
    return (1/(4 * np.pi)) * p1x

def partial1Y(mag1, mag2):
    r = mag1.position - mag2.position
    m1 = mag1.moment
    squares = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    # 1/4pi factored out
    p1y = -m1[0] * ((3 * r[0] * r[1]) / math.pow(squares, 5/2)) - m1[1] * ((3 * r[1] * r[1] - squares) / math.pow(squares, 5/2)) - m1[2] * ((3 * r[1] * r[2]) / math.pow(squares, 5/2))
    return (1/(4 * np.pi)) * p1y

def partial1Z(mag1, mag2):
    r = mag1.position - mag2.position
    m1 = mag1.moment
    squares = r[0] * r[0] + r[1] * r[1] + r[2] * r[2]
    # 1/4pi factored out
    p1z = -m1[0] * ((3 * r[0] * r[2]) / math.pow(squares, 5/2)) - m1[1] * ((3 * r[1] * r[2]) / math.pow(squares, 5/2)) - m1[2] * ((3 * r[2] * r[2] - squares) / math.pow(squares, 5/2))
    return (1/(4 * np.pi)) * p1z

#We don't need to implement the m1 partials, because we'll end up doing the above calculations for that m1 anyways.
def partial2X(mag1, mag2):
    return 5

def partial2Y(mag1, mag2):
    return 5

def partial2Z(mag1, mag2):
    return 5

def dotproduct(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))

def length(v):
  return math.sqrt(dotproduct(v, v))

def angle(v1, v2):
  return math.acos(dotproduct(v1, v2) / (length(v1) * length(v2)))

if MATPLOT:
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
    rotPartials1 = [partial1X, partial1Y, partial1Z]
    # rotPartials1 = [partial2X, partial2Y, partial2Z]

    threshold = 0.01
    distThreshold = Magnet.radius*2.01

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

    def calculateNormals(self, magnet, force, contactPoints):

        k = 1 #spring constant
        m = 2 #magnitude of magnetic force

        numVariables = len(contactPoints) * 2

        angles = []
        for i in range(len(contactPoints)):
            vec = contactPoints[i].position - magnet.position
            angles.append(angle(vec, force))

        equations = []  # [n1, n2, n3, ..., d1, d2, d3, ... ]
        values = []

        for i in range(len(contactPoints)):
            eq = [0 for z in range(numVariables)]
            eq[i] = k
            eq[i+int(numVariables/2)] = -1

            equations.append(eq)
            values.append(0)

        eq2 = []
        for i in range(len(contactPoints)):       #TODO: Change it so we replace the variables for the normal forces from past magnets with the known normal force
            eq2.append(math.cos(angles[i]))

        for i in range(len(contactPoints)):
            eq2.append(0)
        
        equations.append(eq2)
        values.append(m)

        for i in range(len(contactPoints) - 1):
            eq3 = [0 for z in range(numVariables)]
            eq3[i + int(numVariables/2)] = math.cos(angles[i])
            eq3[(i+1) + int(numVariables/2)] = - math.cos(angles[i+1])                
            equations.append(eq3)
            values.append(0)
            
        print(equations)
        print(values)
        print(np.array(equations).shape)
        print(np.array(values).shape)
        solution = np.linalg.solve(equations, values)
        return solution


    class ContactPoint:

        def __init__(self, position, magnets):
               self.position = position
               self.magnets = magnets

        def forceOn(self, magnet):
            return self.normalForces[magnet]

        def forceIs(self, magnet, force):
            self.normalForces = {magnet : force, [mag for mag in self.magnets if not mag == magnet][0] : -force}

    #Go through each magnet, calculate normal forces, then just check that the normal forces for each contact point add up to zero

    #Start at one magnet, get using the gradient as a spring force displacement vector, calculate the forces for each contact point.
    # Then, for each of those normal forces, solve the same system of equations for the next magnet over with the added opposite force
    # from the contact point. If there is ever an unsolvable system, the configuration is unstable.

    def checkStability(self, partials):

        contactPoints = []

        try:

            for i in range(len(self.magnets)):
                for j in range(len(self.magnets)):
                    m1 = self.magnets[i]
                    m2 = self.magnets[j]
                    if (not m1 == m2) and np.linalg.norm(m1.position - m2.position) < self.distThreshold:
                        cp = self.ContactPoint((m1.position + m2.position)/2, [m1, m2])
                        contactPoints.append(cp)

            for i in range(len(self.magnets)):
                magnet = self.magnets[i]

                cps = []
                for c in contactPoints:
                    if magnet in c.magnets:
                        cps.append(c)
                normalForces = self.calculateNormals(magnet, partials[i], cps)
                for i in range(contactPoints):
                    vec = contactPoints[i].position - magnet.position
                    vec /= np.linalg.norm(vec)
                    vec *= normalForces[i]

                    contactPoints[i].forceIs(self.magnets[i], vec)

        except:
            for i in range(len(normalForces)):
                vec = contactPoints[i].position
            return False, contactPoints

        return True, contactPoints

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

            a = Arrow3D([px, px+gx], [py, py+gy], [pz, pz+gz], mutation_scale=20, lw=4, arrowstyle="-|>", color="k")
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
        rotPartials = []

        normalForces = []

        for i in range(len(self.magnets)):
            rotPartials.append(np.array([0.0, 0.0, 0.0]))

        for mag1 in self.magnets:
            normalForces.append(0)
            partialPos = np.array([0.0, 0.0, 0.0])
            partialRot = np.array([0, 0, 0])
            for mag2 in self.magnets:
                if not mag1 == mag2:
                    partialPos += np.array([partial(mag1, mag2) for partial in self.posPartials])

                    rotPartials[self.magnets.index(mag1)] += np.array([partial1X(mag1, mag2), partial1Y(mag1, mag2), partial1Z(mag1, mag2)])

                vec = mag2.position - mag1.position
                if np.linalg.norm(vec) < Magnet.radius*2.01:
                    normalForces.append((vec) / np.linalg.norm(vec))

            partialPos = -partialPos
            print("Magnet Partial: " + str(partialPos))
            partials.append(partialPos)

        stability, contactPoints = self.checkStability(partials)

        if not stability:
            for mag in magnets:
                mag.color = 'r'
            print("Unstable magnet")
        
        # for i in range(len(rotPartials)):
        #     if not int(np.dot(self.magnets[i].moment, rotPartials[i])) in [int(np.linalg.norm(self.magnets[i].moment) * np.linalg.norm(rotPartials[i])), int(-1*(np.linalg.norm(self.magnets[i].moment) * np.linalg.norm(rotPartials[i])))]:
        #         print(np.dot(self.magnets[i].moment, rotPartials[i]))
        #         print(np.linalg.norm(self.magnets[i].moment) * np.linalg.norm(rotPartials[i]))
        #         print("Rotationally unstable: " + str(i))
        #         self.magnets[i].color = 'r'

        if MATPLOT:
            self.draw(partials)
            return partials
        else:
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


    def loop(num, counterclockwise=False):
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
    
    mag1 = Magnet(np.array([1, 0, 0]), Magnet.radius, np.array([Magnet.radius*2, 0, 0]), 'b')
    mag2 = Magnet(np.array([1, 1, 0]), Magnet.radius, np.array([0, 0, 0]), 'b')

    # magnets = line(3, ldir='z', momentDir='x')
    # magnets = line(2,'x','x')
    # magnets[0].moment = -magnets[0].moment
    # magnets[1].moment = -magnets[1].moment
    # magnets = saddle()

    # magnets = MagnetSimulator.loop(15, True)
    # magnets[4].position -= np.array([0, 0.001, 0])

    magnets = MagnetSimulator.loop(5)


    # magnets = [mag1, mag2]
    # magnets = [mag1, mag2]

    sim = MagnetSimulator(magnets)
    partials = sim.run()
    print(partials[0])
    print((partials[1] + partials[2] + partials[3] + partials[4]))
