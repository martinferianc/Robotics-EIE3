import random
import math
import numpy as np

# Takes an angle and moves it within specified range, default is between -pi and pi
def move_angle_within_range(angle, lower = -math.pi, upper = math.pi, k = 2*math.pi):
    while (angle < lower):
        angle += k
    while (angle > upper):
        angle -= k
    return angle

class ParticleState():
    def __init__(self, standard_deviation, n_particles=100, x = None, y = None, theta=None, mode="continuous", map = None):
        if mode == "continuous":
            if x is not None and y is not None and theta is not None:
                self.state = [[[x,y,theta],1/n_particles] for x in xrange(n_particles)]

        self.number_of_particles = n_particles
	self.state = [[[0,0,0],1/n_particles] for x in xrange(n_particles)]
        self.standard_deviation = standard_deviation
    # Movement is distance for
    def update_state(self, action, movement, mcl=False):
        if action == "straight":
            # movement is the distance travelled
            for point in self.state:
                e_x=random.gauss(0,self.standard_deviation["x"])
                e_y=random.gauss(0,self.standard_deviation["y"])
                e_theta=random.gauss(0,self.standard_deviation["theta"])
                point[0][0]+=(movement + e_x)*math.cos(point[0][2])
                point[0][1]+=(movement + e_y)*math.sin(point[0][2])
                point[0][2]+=e_theta
        elif action == "rotation":
            # movement is the amount of rotation
            for point in self.state:
                point[0][2] += math.radians(movement) + random.gauss(0,self.standard_deviation["theta"])

        else:
            raise Exception("Not a valid action!")
        return True
    def get_coordinates(self):
        mean_x = np.mean(np.array([point[0][0] for point in self.state]))
        mean_y = np.mean(np.array([point[0][1] for point in self.state]))
        mean_theta = np.mean(np.array([point[0][2] for point in self.state]))
        return (mean_x, mean_y, mean_theta)
    def reset(self):
        self.state = [([0,0,0],1/self.number_of_particles) for x in xrange(self.number_of_particles)]
    def get_state(self):
        return self.state

    def __normalise_weights(self):
        # Normalisation of weights
        # Returns new state with all weights normalized

    def __calculate_liklihood(self, x, y, theta, z):
        # Calculate the probability of a single point at a given location
        # Takes the coordinate, ultra_sound reading
        # Changes the weight of that given point

        # Find out which wall the sonar beam will hit

        self.__calculate_distance_to_nearest_wall()

    def __resample(self):
        #Resamples our probability distribution according to new weights

    def __calculate_incidence_angle(self,point):
        #Calculates the incidence angle of the ultrasound reading


    def __calculate_distance_to_nearest_wall(self,point):
        #Calculates the distance to nearest wall
