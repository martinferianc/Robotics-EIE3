import random

class Obstacle:

    def __init__(self, x,y, std_x,std_y,radius=6):
        self.x = x
        self.y = y
        self.std_x = std_x
        self.std_y = std_y
        self.radius = radius

    def is_in_obstacle(self,x,y):
        proposed_centres = {}
        proposed_centres["True"]=0
        proposed_centres["False"]=0
        for i in range(100):
            x_c = random.gauss(self.x, self.std_x)
            y_c = random.gauss(self.y, self.std_y)
            if (x-self.x)**2+(y-self.y)**2 < r**2:
                proposed_centres["True"]+=1
            else:
                proposed_centres["False"]+=1
        return proposed_centres["True"]>=30
