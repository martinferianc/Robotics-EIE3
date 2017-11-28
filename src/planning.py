# Planning
# Dynamic Window Approach (Local Planning)
# Andrew Davison 2017
from __future__ import division
import os, math, time, random


class Planner:
    def __init__(self, y,x=0,theta=0,radius = 0.06, size = 0.2, safe_dist=0.2, playfield=(0, 0, 3.0, 2.0), target=(3.1,0), canvas = None):
        # Constants and variables
        # Units here are in metres and radians using our standard coordinate frame
        self.obstacleRADIUS = radius
        self.ROBOTRADIUS = size
        self.W = 2 * self.ROBOTRADIUS # width of robot
        self.SAFEDIST = safe_dist      # used in the cost function for avoiding obstacles

        self.MAXVELOCITY = 10     #ms^(-1) max speed of each wheel
        self.MAXACCELERATION = 6 #ms^(-2) max rate we can change speed of each wheel

        # The region we will fill with obstacles
        self.PLAYFIELDCORNERS = playfield

        # Set an initial target location which is beyond the obstacles
        self.target = target

        # Starting pose of robot

        # Timestep delta to run control and simulation at
        self.dt = 0.1

        # obstacle (obstacle) locations
        self.obstacles = []
        # obstacle contents are (bx, by, visibilitymask)
        # Generate some initial random obstacles
        # Simulation of forward looking depth sensor; assume cone beam
        # Which obstacles can it see? If a obstacle has been seen at least once it becomes known to the planner
        self.SENSORRANGE = 1.5
        self.SENSORHALFANGLE = 15.0 * math.pi / 180.0
        self.canvas = canvas

    def append_obstacle(self, obstacle, threshold = 7):
        for obs in self.obstacles:
            x = obs[0].get_x()
            y = obs[0].get_y()
            diff_x = abs(x-obstacle.get_x())
            diff_y = abs(y-obstacle.get_y())
            if diff_x<threshold and diff_y<threshold:
                return False
        self.obstacles.append([obstacle,False])
        x = obstacle.get_x()
        y = obstacle.get_y()
        radius = obstacle.get_radius()
        std_x,std_y = obstacle.get_std()
        self.canvas.drawCross((x,y), size_x = std_x+radius,size_y = std_y+radius,offset_y=110)
        return True

    # Function to predict new robot position based on current pose and velocity controls
    # Uses time deltat in future
    # Returns xnew, ynew, thetanew
    def __predict_position(self,vL, vR, x, y, theta, TAU):
        # Simple special cases
        # Straight line motion
        if (vL == vR):
            xnew = x + vL * TAU * math.cos(theta)
            ynew = y + vL * TAU * math.sin(theta)
            thetanew = theta
        # Pure rotation motion
        elif (vL == -vR):
            xnew = x
            ynew = y
            thetanew = theta + ((vR - vL) * TAU / self.W)
        else:
            # Rotation and arc angle of general circular motion
            # Using equations given in Lecture 2
            R = self.W / 2.0 * (vR + vL) / (vR - vL)
            deltatheta = (vR - vL) * TAU / self.W
            xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
            ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
            thetanew = theta + deltatheta
        return (xnew, ynew, thetanew)

    # Function to calculate the closest obstacle at a position (x, y)
    # Used during planning
    def __calculate_closest_obstacle_distance(self,x, y):
        closestdist = 100000.0
        # Calculate distance to closest obstacle
        for obstacle in self.obstacles:
            dx = obstacle[0].get_x()/100 - x
            dy = obstacle[0].get_y()/100 - y
            d = math.sqrt(dx**2 + dy**2)
            # Distance between closest touching point of circular robot and circular obstacle
            dist = d - self.obstacleRADIUS - self.ROBOTRADIUS
            print "Your dist is: " + str(dist)
            if (dist < closestdist):
                closestdist = dist
                print "Closest obstacle: " + str(dist)
        return closestdist

    def get_plan(self,x,y,theta,vL,vR,interval):
        # Planning
        # We want to find the best benefit where we have a positive component for closeness to target,
        # and a negative component for closeness to obstacles, for each of a choice of possible actions
        bestBenefit = -100000
        FORWARDWEIGHT = 12
        OBSTACLEWEIGHT = 16

        # Range of possible motions: each of vL and vR could go up or down a bit
        vLpossiblearray = (vL - self.MAXACCELERATION * interval, vL, vL + self.MAXACCELERATION * interval)
        vRpossiblearray = (vR - self.MAXACCELERATION * interval, vR, vR + self.MAXACCELERATION * interval)

        vLchosen = 1
        vRchosen = 1

        print "Getting a new plan"

        for vLpossible in vLpossiblearray:
            for vRpossible in vRpossiblearray:
                # We can only choose an action if it's within velocity limits
                print "Velocity Check"
                print self.MAXVELOCITY
                print vLpossible
                print vRpossible
                if (vLpossible <= self.MAXVELOCITY and vRpossible <= self.MAXVELOCITY and vLpossible >= -self.MAXVELOCITY and vRpossible >= -self.MAXVELOCITY):
                    # Predict new position in TAU seconds
                    TAU = interval
                    (xpredict, ypredict, thetapredict)= self.__predict_position(vLpossible, vRpossible, x, y, theta, TAU)

                    # What is the distance to the closest obstacle from this possible position?
                    distanceToObstacle = self.__calculate_closest_obstacle_distance(xpredict, ypredict)
                    print("Distance to nearest obstacle: {}".format(distanceToObstacle))
                    # Calculate how much close we've moved to target location
                    previousTargetDistance = math.sqrt((x - self.target[0])**2 + (y - self.target[1])**2)
                    newTargetDistance = math.sqrt((xpredict - self.target[0])**2 + (ypredict - self.target[1])**2)
                    distanceForward = previousTargetDistance - newTargetDistance

                    # Alternative: how far have I moved forwards?
                    # distanceForward = xpredict - x
                    # Positive benefit
                    distanceBenefit = FORWARDWEIGHT * distanceForward
                    # Negative cost: once we are less than SAFEDIST from collision, linearly increasing cost
                    if (distanceToObstacle < self.SAFEDIST):
                        obstacleCost = OBSTACLEWEIGHT * (self.SAFEDIST - distanceToObstacle)
                    else:
                        obstacleCost = 0.0
                    # Total benefit function to optimise
                    benefit = distanceBenefit - obstacleCost
                    if (benefit > bestBenefit):
                        vLchosen = vLpossible
                        vRchosen = vRpossible
                        bestBenefit = benefit
        vL = vLchosen
        vR = vRchosen

        # Actually now move robot based on chosen vL and vR
        (x, y, theta) = self.__predict_position(abs(vL), abs(vR), x, y, theta, interval)

        #Convert back to CM
        return (abs(vL), abs(vR), x*100,y*100,theta)
