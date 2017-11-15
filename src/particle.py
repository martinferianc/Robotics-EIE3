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
                 Map = None,
                 points = None):
        self.state = []
        self.mcl = mcl
        self.Map = Map
        if mode == "continuous":
            if x is not None and y is not None and theta is not None:
                print("Initializing particle state with x,y,theta = ({0},{1},{2})".format(x,y,math.radians(theta)))
                self.state = [[[x,y,math.radians(theta)],1/n_particles] for i in xrange(n_particles)]
            else:
	            self.state = [[[0,0,0],1/n_particles] for i in xrange(n_particles)]
        elif mode == "global":
            for i in range(n_particles):
                x = random.randint(1,210)
                y = random.randint(1,210)
                theta = random.uniform(-math.pi, math.pi)
                valid = True
                while valid is False:
                    if (y>=168 and x<=84) or (y>=84 and x>=168):
                        valid = False
                        x = random.randint(1,210)
                        y = random.randint(1,210)
                    else:
                        valid = True
                self.state.append([[x,y,theta],1/n_particles])
        elif mode == "localised":
            X = [point[0] for point in points]
            Y = [point[1] for point in points]
            for i in range(n_particles):
                x = random.choice(X)
                y = random.choice(Y)
                theta = random.uniform(-math.pi, math.pi)
                self.state.append([[x,y,theta],1/n_particles])

        else:
            self.state = [[[0,0,0],1/n_particles] for i in xrange(n_particles)]
        self.number_of_particles = n_particles
        self.standard_deviation = standard_deviation

    # Movement is distance for
    def update_state(self, action, movement, ultrasound={'0', 255}):
        if action == "straight":
            # movement is the distance travelled
            for point in self.state:
                #print "Point before movement: {0}, {1}, {2}".format(point[0][0], point[0][1], point[0][2])
                e_x=random.gauss(0,self.standard_deviation["x"])
                e_y=random.gauss(0,self.standard_deviation["y"])
                e_theta=random.gauss(0,self.standard_deviation["theta_straight"])
                point[0][0]+=(movement + e_x)*math.cos(point[0][2])

                if(point[0][0] < 0) and self.mcl:
                    point[0][0] = 0
                point[0][1]+=(movement + e_y)*math.sin(point[0][2])
                if(point[0][1] < 0) and self.mcl:
                    point[0][1] = 0
                point[0][2]+=e_theta
                point[0][2] = move_angle_within_range(point[0][2])
        elif action == "rotation":
            # movement is the amount of rotation
            for point in self.state:
                point[0][2] += math.radians(movement) + random.gauss(0,self.standard_deviation["theta_rotate"])
                point[0][2] = move_angle_within_range(point[0][2])
        #Account for the uncertainity in ultra sound rotation
        elif action == "refinement":
            for point in self.state:
                point[0][2] += random.gauss(0,self.standard_deviation["theta_top_rotate"])
                point[0][2] = move_angle_within_range(point[0][2])
        else:
            raise Exception("Not a valid action!")

        if self.mcl is True:
            # Step 1 - Motion prediction based on odometry
            for point in self.state:
                likelihood = self.__calculate_likelihood(point[0], ultrasound)
                #print "Likelihood: {0} Ultrasound: {1}".format(likelihood, ultrasound)
                point[1] *= likelihood
            self.__normalise_weights()
            self.__resample()
            #for point in self.state:
                #print "Point after resampling x:{} y:{} theta:{} w:{}".format(point[0][0], point[0][1], point[0][2],point[1])
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
        # print("Coordinates - x: {0}, y: {1}, theta {2}".format(mean_x, mean_y, mean_theta))
        mean_theta = move_angle_within_range(mean_theta)
        return (mean_x, mean_y, mean_theta)

    def reset(self):
	    self.state = [([0,0,0],1/self.number_of_particles) for i in xrange(self.number_of_particles)]

    def get_error(self):
        std_x = np.std(np.array([point[0][0] for point in self.state]))
        std_y = np.std(np.array([point[0][1] for point in self.state]))
        std_theta = np.std(np.array([point[0][2] for point in self.state]))
        return std_x, std_y, std_theta

    def get_state(self):
        return self.state

    def __normalise_weights(self):
        #Mike
        # Normalisation of weights
        # Returns new state with all weights normalized
        # initialize weight_sum
        # sumarize weight
        weight_sum = sum([point[1] for point in self.state])
        # divide individual weight by weight_sum
        for point in self.state:
            point[1] = point[1] / weight_sum

        return True

    def __calculate_likelihood(self, point, ultrasound):
        k = 0.05
        out = []
        for key,value in ultrasound.items():
            try:
                nearest_wall = self.__predict_distance_to_nearest_wall(point, int(key))
                predicted_distance = nearest_wall["distance"]
            except ValueError as e:
                out.append(k)
                continue

            if nearest_wall["angle"] > 15:
                out.append(k)
                continue
            print("Point position: x - {0}, y - {1}, theta - {2}".format(point[0], point[1], math.degrees(point[2]+math.radians(int(key)))))

            diff = value - predicted_distance
            numerator = -math.pow(diff,2)
            denominator = 2*math.pow(self.standard_deviation["ultrasound"],2)
            exponent = math.exp(numerator/denominator)
            likelihood = k + (math.exp(numerator/denominator))
            out.append(likelihood)

        avg = sum(out)/len(out)
        print("Predicted distance: {0}, Diff: {1}, Likelihood:{2}".format(predicted_distance, diff, avg))
        return avg

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
        particle_coord = copy.deepcopy([point[0] for point in self.state])
        histo = copy.deepcopy([point[1] for point in self.state])
        # generate cumulative sum array
        cum_histo = np.cumsum(histo)
        # pick particles and form a new set of particles
        for loop_no in xrange(self.number_of_particles):
            random_no = random.uniform(0, 1)
            chosen_idx = 0
            while (cum_histo[chosen_idx] < random_no):
                chosen_idx += 1
                if (chosen_idx >= len(cum_histo)-1):
                    break
            new_state.append([copy.deepcopy(particle_coord[chosen_idx]), 1/float(self.number_of_particles)])
        # put new state into self
        self.state = new_state

    def __predict_distance_to_nearest_wall(self,point, ultrasound_pose=None):
        #Calculates the distance to nearest wall, returning M
        #For each wall in the area which a line from the robot would pass thorough, we need to calculate M, the distance.
        #We will then take the smallest positive value of M which will be the nearest wall.
        #print "Testing for particle {0},{1},{2}".format(point[0],point[1],point[2])
        m = []
        number_walls = len(self.Map)
        for i in range(len(self.Map)):
            m.append(self.__calculate_m(point, self.Map[i], self.Map[(i+1)%number_walls], ultrasound_pose))
        #print("M:")
        #print(m)
        try:
            smallest_m = min(j for j in m if j >= 0)
            position = m.index(smallest_m);
            next_wall = (position+1)%number_walls
            B = self.__predict_incidence_angle(point,self.Map[position], self.Map[next_wall],ultrasound_pose)
            return {"distance": smallest_m, "angle": B, "wall":str(self.Map[position][2])+str(self.Map[next_wall][2]) }
        except ValueError as e:
            print "Error: No wall values satisfy criteria"
            raise

    def __predict_incidence_angle(self,point, wallPointA, wallPointB, ultrasound_pose=None):
        #Owen
        #Calculates the incidence angle of the ultrasound reading
        diff_y_AB = wallPointA[1]-wallPointB[1]
        diff_x_BA = wallPointB[0]-wallPointA[0]
        if ultrasound_pose:
            c = math.cos(point[2]+math.radians(ultrasound_pose))*diff_y_AB
            s = math.sin(point[2]+math.radians(ultrasound_pose))*diff_x_BA
        else:
            c = math.cos(point[2])*diff_y_AB
            s = math.sin(point[2])*diff_x_BA
        angle = (c + s) / math.sqrt(math.pow(diff_y_AB,2)+math.pow(diff_x_BA,2))
        #if(math.degrees(math.acos(angle)) > 50):
            #print "Robot @: {0}, {1}, {2}".format(point[0], point[1], point[2])
            #print "Wall Point A: {0}, {1}".format(wallPointA[0], wallPointA[1])
            #print "Wall Point B: {0}, {1}".format(wallPointB[0], wallPointB[1])
        return math.degrees(math.acos(angle))

    def __calculate_m(self,point,wallPointA,wallPointB, ultrasound_pose=None):
        if ultrasound_pose:
            cos_t = math.cos(point[2]+math.radians(ultrasound_pose))
            sin_t = math.sin(point[2]+math.radians(ultrasound_pose))
        else:
            cos_t = math.cos(point[2])
            sin_t = math.sin(point[2])
        diff_y_BA = wallPointB[1]-wallPointA[1]
        diff_x_AO = wallPointA[0]-point[0]
        diff_x_BA = wallPointB[0]-wallPointA[0]
        diff_y_AO = wallPointA[1]-point[1]
        m = ((diff_y_BA*diff_x_AO)-(diff_x_BA*diff_y_AO))/((diff_y_BA*cos_t) - (diff_x_BA*sin_t))
        intersect_x = point[0] + m*cos_t
        intersect_y = point[1] + m*sin_t
        # If the wall is vertical (parallel to y) check to see if the predicted y is within range.
        if(wallPointA[0] == wallPointB[0]):
            if(float(min(wallPointA[1], wallPointB[1])) <= intersect_y <= float(max(wallPointA[1], wallPointB[1]))):
                return m
        # If the wall is horizontal (parallel to x) check to see if the predicted x is within range.
        if(wallPointA[1] == wallPointB[1]):
            if (float(min(wallPointA[0], wallPointB[0])) <= intersect_x <= float(max(wallPointA[0], wallPointB[0]))):
                return m
        return -1;
