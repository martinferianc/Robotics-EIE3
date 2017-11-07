import random
import math
import numpy as np

class ParticleState():
    def __init__(self, standard_deviation, n_particles=100, x = None, y = None, theta=None, mode="continuous", map = None):
        #Patata
        if mode == "continuous":
            if x is not None and y is not None and theta is not None:
                self.state = [[[x,y,theta],1/n_particles] for x in xrange(n_particles)]
            else:
                raise Exception("X,Y or theta has not been initialized!")
        else:

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
	mean_y = np.mean(np.array([point[0][1] for point in self.state]))
        mean_x = np.mean(np.array([point[0][0] for point in self.state]))
        mean_theta = np.mean(np.array([point[0][2] for point in self.state]))
        return (mean_x, mean_y, mean_theta)
    def reset(self):
	self.state = [([0,0,0],1/self.number_of_particles) for x in xrange(self.number_of_particles)]
    def get_state(self):
        return self.state

    def __normalise_weights(self):
        #Mike
        # Normalisation of weights
        # Returns new state with all weights normalized
    def __calculate_proability(self, point, ultrasound_reading):
        #George
        #Calculate the probabilities of a single point at a given location
        #Takes the coordinate, ultra_sound reading
        #Changes the weight of that given point

    def __resample(self):
        #Mike
        #Resamples our probability distribution according to new weights

    def __calculate_incidence_angle(self,point):
        #Owen
        #Calculates the incidence angle of the ultrasound reading


    def __calculate_distance_to_nearest_wall(self,point):
        #Owen
        #Calculates the distance to nearest wall
