from __future__ import division
import random
import math
import numpy as np

class ParticleState():
    def __init__(self,
                 standard_deviation,
                 n_particles=100,
                 x = None,
                 y = None,
                 theta=None,
                 mode="continuous",
                 mcl = False,
                 map = None):
        self.state = []
        if mode == "continuous":
            if x is not None and y is not None and theta is not None:
                self.state = [[[x,y,theta],1/n_particles] for x in xrange(n_particles)]
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
    def update_state(self, action, movement, mcl=False, ultra_sound=None):
        if mcl is False:
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
        else:
            pass

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
        # initialize weight_sum
        weight_sum = 0
        # sumarize weight
        for index in xrange(len(self.state)):
            weight_sum += self.state[index][1]
        # divide individual weight by weight_sum
        for index in xrange(len(self.state)):
            self.state[index][1] = self.state[index][1] / weight_sum

        return True

    def __calculate_proability(self, point, ultrasound_reading):
        #George
        #Calculate the probabilities of a single point at a given location
        #Takes the coordinate, ultra_sound reading
        #Changes the weight of that given point

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
        self.state = new_state


    def __calculate_incidence_angle(self,point):
        #Owen
        #Calculates the incidence angle of the ultrasound reading


    def __calculate_distance_to_nearest_wall(self,point):
        #Owen
        #Calculates the distance to nearest wall
