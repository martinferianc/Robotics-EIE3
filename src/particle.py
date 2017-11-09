from __future__ import division
import random
import math
import numpy as np
import copy

# Takes an angle and moves it within specified range, default is between -pi and pi
def move_angle_within_range(angle, lower = -math.pi, upper = math.pi, k = 2*math.pi):
    while (angle < lower):
        angle += k
    while (angle > upper):
        angle -= k
    return angle

class ParticleState():
    def __init__(self,
                 standard_deviation,
                 n_particles=100,
                 x = None,
                 y = None,
                 theta=None,
                 mode="continuous",
                 mcl = False,
                 Map = None):
        self.state = []
        self.mcl = mcl
        if mode == "continuous":
            if x is not None and y is not None and theta is not None:
                self.state = [[[x,y,math.radians(theta)],1/n_particles] for x in xrange(n_particles)]
            else:
	            self.state = [[[0,0,0],1/n_particles] for x in xrange(n_particles)]
        elif mode == "global":
            for i in range(n_particles):
                x = random.randint(1,210)
                y = random.randint(1,210)
                valid = True
                while valid is False:
                    if (y>=168 and x<=84) or (y>=84 and x>=168):
                        valid = False
                        x = random.randint(1,210)
                        y = random.randint(1,210)
                    else:
                        valid = True
                self.state.append([[x,y,0],1/n_particles])
        else:
            self.state = [[[0,0,0],1/n_particles] for x in xrange(n_particles)]
        self.number_of_particles = n_particles
        self.standard_deviation = standard_deviation
    # Movement is distance for
    def update_state(self, action, movement, ultrasound=None):
        if action == "straight":
            # movement is the distance travelled
            for point in self.state:
                e_x=random.gauss(0,self.standard_deviation["x"])
                e_y=random.gauss(0,self.standard_deviation["y"])
                e_theta=random.gauss(0,self.standard_deviation["theta_straight"])
                point[0][0]+=(movement + e_x)*math.cos(point[0][2])
                point[0][1]+=(movement + e_y)*math.sin(point[0][2])
                point[0][2]+=e_theta
                point[0][2] = move_angle_within_range(point[0][2])
        elif action == "rotation":
            # movement is the amount of rotation
            for point in self.state:
                point[0][2] += math.radians(movement) + random.gauss(0,self.standard_deviation["theta_rotate"])
                point[0][2] = move_angle_within_range(point[0][2])
        else:
            raise Exception("Not a valid action!")

        if self.mcl:
            # Step 1 - Motion prediction based on odometry
            for point in self.state:
                likelihood = __calculate_likelihood(point[0], ultrasound)
                point[1] *= likelihood
            self.__normalise_weights()
            self.__resample()
        return True

    def get_coordinates(self):
        if self.mcl:
            self.__normalise_weights()
            mean_x = sum([point[0][0]*point[1] for point in self.state])
            mean_y = sum([point[0][1]*point[1]  for point in self.state])
            mean_theta = sum([point[0][2]*point[1]  for point in self.state])
        else:
            mean_x = np.mean(np.array([point[0][0] for point in self.state]))
            mean_y = np.mean(np.array([point[0][1] for point in self.state]))
            mean_theta = np.mean(np.array([point[0][2] for point in self.state]))
        print("Coordinates - x: {0}, y: {1}, theta {2}".format(mean_x, mean_y, mean_theta))
        mean_theta = move_angle_within_range(mean_theta)
        return (mean_x, mean_y, mean_theta)
    def reset(self):
	    self.state = [([0,0,0],1/self.number_of_particles) for x in xrange(self.number_of_particles)]
    def get_state(self):
        return self.state

    def __normalise_weights(self):
        #Mike
        # Normalisation of weights
        # Returns new state with all weights normalized
        # initialize weight_sum
        weight_sum = 0
        # sumarize weight
        for index in xrange(len(self.state)):
            weight_sum += self.state[index][1]
        # divide individual weight by weight_sum
        for index in xrange(len(self.state)):
            self.state[index][1] = self.state[index][1] / weight_sum

        return True

    def __calculate_likelihood(self, point, ultrasound_measurement):
        nearest_wall = self.__predict_distance_to_nearest_wall(point)
        predicted_distance = nearest_wall["distance"]
        if nearest_wall["angle"] > 15:
            return k
        diff = ultrasound_measurement - predicted_distance
        likelihood = k + math.exp(-math.pow(diff,2)/(2*math.pow(standard_deviation["ultrasound"],2)))
        return likelihood

    def __resample(self):
        #Mike
        #Resamples our probability distribution according to new weights
        """ Takes a normalised particle distribution array
            1. generate an array of the particle distribution probability histogram
            2. generate an array of particle coordinates corresponding to the histogram
            3. use Cumulative Porbability Distribution to pick particles
        """
        new_state = []
        # generate histogram and particle coordinates array
        histo = []
        particle_coord = []
        for index in xrange(len(self.state)):
            particle_coord.append(self.state[index][0])
            histo.append(self.state[index][1])
        # generate cumulative sum array
        cum_histo = np.cumsum(histo)
        # pick particles and form a new set of particles
        for loop_no in xrange(len(self.state)):
            random_no = random.uniform(0, 1)
            chosen_idx = 0
            while (cum_histo[chosen_idx] < random_no):
                chosen_idx += 1
                if (chosen_idx >= len(cum_histo)-1):
                    break
            new_state.append([particle_coord[chosen_idx], 1/float(self.no_of_particles)])
        # put new state into self
        self.state = copy.deepcopy(new_state)

    def __predict_distance_to_nearest_wall(self,point):
        #Calculates the distance to nearest wall, returning M
        #For each wall in the area which a line from the robot would pass thorough, we need to calculate M, the distance.
        #We will then take the smallest positive value of M which will be the nearest wall.
        m = []
        for i in range(len(Map)-1):
            m.append(__calculate_m(point, Map[i], Map[i+1]))
        smallest_m = min(j for j in m if j > 0)
        position = m.index(smallest_m);
        B = __predict_incidence_angle(point,Map[position], Map[position+1])
        return {"distance": smallest_m, "angle": B, "wall":str(Map[position][2])+str(Map[position+1][2]) }

    def __predict_incidence_angle(self,point, wallPointA, wallPointB):
        #Owen
        #Calculates the incidence angle of the ultrasound reading
        diff_y_AB = wallPointA[1]-wallPointB[1]
        diff_x_AB = wallPointB[0]-wallPointA[0]
        c = math.cos(point[2])*diff_y_AB
        s = math.sin(point[2])*diff_x_AB
        angle = (c + s) / math.sqrt(math.pow(diff_y_AB,2)+math.pow(diff_x_AB,2))
        return math.degrees(math.acos(angle))

    def __calculate_m(self,point,wallPointA,wallPointB):
        m = (((wallPointB[1]-wallPointA[1])(wallPointA[0]-point[0])-(wallPointB[0]-wallPointA[0])(wallPointA[1]-point[1]))/
        (((wallPointB[1]-wallPointA[1])*math.cos(point[2])) - (wallPointB[0]-wallPointA[0])*math.sin(point[2])))
        intersect_x = point[0] + m*math.cos(point[2])
        intersect_y = point[1] + m*math.sin(point[2])
        if ((intersect_x >= min(wallPointA[0], wallPointB[0])) and (intersect_x <= max(wallPointA[0], wallPointB[0]))):
            if ((intersect_y >= min(wallPointA[1], wallPointB[1])) and (intersect_y <= max(wallPointA[1], wallPointB[1]))):
                return m
        return -1;
