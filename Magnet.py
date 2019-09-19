class Magnet:
    
    def getMoment(self, magnetization):
        return 0

    def __init__(self, magnetization, radius, position):
        self.magnetization = magnetization
        self.radius = radius
        self.position = position
        self.moment = self.getMoment(magnetization)
