class MagnetSimulator:

    def __init__(self, magnets):
        magnets = magnets

    def run(self, time, output, increment):
        self.output = output
        self.increment = increment

        for i in range(time/increment):
            self.tickMagnets()
            self.outputFrame()

    def getMagnetData(self):
        return ""

    def outputFrame(self):
        if self.output == 'csv':
            print(self.getMagnetData())
        elif self.output == 'print':
            print(self.getMagnetData())


    def tickMagnets(self):
        pass


time = 10
increment = 0.1

if __name__ == "__main__":
    magnets = []
    sim = MagnetSimulator(magnets)
    sim.run(time, 'print', increment)
