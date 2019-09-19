import magpylib as magpy
import numpy as np
import matplotlib.pyplot as plt

class MagnetSimulator:

    def __init__(self, magnets):
        self.magnets = magnets

    def run(self, time, output, increment):
        self.output = output
        self.increment = increment

        for i in range(int(time/increment)):
            self.tickMagnets()
            self.outputFrame()
            break

    def getMagnetData(self):
        return ""

    def outputFrame(self):
        if self.output == 'csv':
            print(self.getMagnetData())
        elif self.output == 'print':
            print(self.getMagnetData())

    def drawGraph(self):
                #Define the space in which to calculate the b-field
        xs = np.linspace(-10,10,20)
        ys = np.linspace(-10,10,20)
        zs = np.linspace(-10,10,20)
        
        # #put all the magnets together into a collection
        pmc = pmc = magpy.Collection(*self.magnets)

        # #Calculate the overall 3d vector field from all the magnets
        Bs = np.array([[[pmc.getB([x,y,z]) for x in xs] for y in ys] for z in zs])

        # x,y = np.meshgrid(xs,zs)
        u,v,w = Bs[:,:,:,0], Bs[:,:,:,1], Bs[:,:,:,2]
        # fig, ax = plt.subplots()
        # ax.quiver(x, y, z, u, v, w, length=0.1)

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        x, y, z = np.meshgrid(xs, ys, zs)

        print(u.shape)

        u = np.sin(np.pi * x) * np.cos(np.pi * y) * np.cos(np.pi * z)
        # v = -np.cos(np.pi * x) * np.sin(np.pi * y) * np.cos(np.pi * z)
        # w = (np.sqrt(2.0 / 3.0) * np.cos(np.pi * x) * np.cos(np.pi * y) * np.sin(np.pi * z))

        print(u.shape)

        ax.quiver(x, y, z, u, v, v, length=0.1)

        plt.show()

    def tickMagnets(self):
        
        pmc = pmc = magpy.Collection(*self.magnets)
        
        

time = 10
increment = 0.1

if __name__ == "__main__":
    magnet1 = magpy.source.magnet.Sphere(mag=[0,0,600],dim=3,pos=[-4,0,3], angle=0)
    magnet2 = magpy.source.magnet.Sphere(mag=[0,0,1000],dim=3,pos=[2,-5,1], angle=0)
    magnets = [magnet1, magnet2]
    sim = MagnetSimulator(magnets)
    sim.run(time, 'print', increment)
