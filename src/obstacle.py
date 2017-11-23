import random
import numpy as np

class Obstacle:

    def __init__(self, x,y, std_x,std_y,radius=6):
        self.x = x
        self.y = y
        self.std_x = std_x
        self.std_y = std_y
        self.radius = radius
        self.__update_state()

    def is_in_obstacle(self,x,y,buff = 0):
        return (x-self.x_c)**2+(y-self.y_c)**2 < (self.radius+buff)**2:

    def __update_state(self):
        X = []
        Y = []
        for i in range(100):
            X.append(random.gauss(self.x, self.std_x))
            Y.append(random.gauss(self.y, self.std_y))
        self.x_c = np.mean(np.array(X))
        self.y_c = np.mean(np.array(Y))


    def set_std(self, std_x, std_y):
        self.std_x = std_x
        self.std_y = std_y

    def set_coordinates(self, x, y):
        self.x = x
        self.y = y
        self.__update_state()

    def get_x(self):
        return self.x_c

    def get_y(self):
        return self.y_c

    def get_std(self):
        return self.std_x, self.std_y
    def get_radius(self):
        return self.radius
