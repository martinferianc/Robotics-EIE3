# Planning
# Dynamic Window Approach (Local Planning)
# Andrew Davison 2017
import os, math, time, random

class Planner:
    def __init__(self, y,x=0,theta=0,radius = 0.06, size = 0.2, safe_dist=0.2, playefield=(0, 0, 3.0, 2.0), target=(4,0)):
        # Constants and variables
        # Units here are in metres and radians using our standard coordinate frame
        self.BARRIERRADIUS = radius
        self.ROBOTRADIUS = size
        self.W = 2 * self.ROBOTRADIUS # width of robot
        self.SAFEDIST = safe_dist      # used in the cost function for avoiding obstacles

        self.MAXVELOCITY = 0.5     #ms^(-1) max speed of each wheel
        self.MAXACCELERATION = 0.5 #ms^(-2) max rate we can change speed of each wheel

        # The region we will fill with obstacles
        self.PLAYFIELDCORNERS = playfield

        # Set an initial target location which is beyond the obstacles
        self.target = target

        # Starting pose of robot
        self.x = x
        self.y = y
        self.theta = theta

        # Starting wheel velocities
        self.vL = 0.00
        self.vR = 0.00

        # Timestep delta to run control and simulation at
        self.dt = 0.1

        # Barrier (obstacle) locations
        self.barriers = []
        # barrier contents are (bx, by, visibilitymask)
        # Generate some initial random barriers
        # Simulation of forward looking depth sensor; assume cone beam
        # Which barriers can it see? If a barrier has been seen at least once it becomes known to the planner
        self.SENSORRANGE = 1.5
        self.SENSORHALFANGLE = 15.0 * math.pi / 180.0

    def append_barrier(self, barrier):
        self.barriers.append([barrier,0])

    # Function to predict new robot position based on current pose and velocity controls
    # Uses time deltat in future
    # Returns xnew, ynew, thetanew
    def __predict_position(self,vL, vR, x, y, theta):
    	# Simple special cases
    	# Straight line motion
    	if (vL == vR):
    		xnew = x + vL * self.dt * math.cos(theta)
    		ynew = y + vL * self.dt * math.sin(theta)
    		thetanew = theta
    	# Pure rotation motion
    	elif (vL == -vR):
    		xnew = x
    		ynew = y
    		thetanew = theta + ((vR - vL) * self.dt / self.W)
    	else:
    		# Rotation and arc angle of general circular motion
    		# Using equations given in Lecture 2
    		R = self.W / 2.0 * (vR + vL) / (vR - vL)
    		deltatheta = (vR - vL) * self.dt / self.W
    		xnew = x + R * (math.sin(deltatheta + theta) - math.sin(theta))
    		ynew = y - R * (math.cos(deltatheta + theta) - math.cos(theta))
    		thetanew = theta + deltatheta
    		# To calculate parameters for arc drawing (complicated Pygame stuff, don't worry)
    		# We need centre of circle
    		(cx, cy) = (x - R * math.sin(theta), y + R * math.cos (theta))
    		# Turn this into Rect
    		Rabs = abs(R)
    		((tlx, tly), (Rx, Ry)) = ((int(u0 + k * (cx - Rabs)), int(v0 - k * (cy + Rabs))), (int(k * (2 * Rabs)), int(k * (2 * Rabs))))
    		if (R > 0):
    			start_angle = theta - math.pi/2.0
    		else:
    			start_angle = theta + math.pi/2.0
    		stop_angle = start_angle + deltatheta

    	return (xnew, ynew, thetanew)

    # Function to calculate the closest obstacle at a position (x, y)
    # Used during planning
    def __calculate_closest_obstacle_distance(self,x, y):
    	closestdist = 100000.0
    	# Calculate distance to closest obstacle
    	for barrier in self.barriers:
    		# Is this a barrier we know about? barrier[2] flag is set when sonar observes it
    		if barrier[1] is True:
    			dx = barrier[0].get_x() - x
    			dy = barrier[0].get_y() - y
    			d = math.sqrt(dx**2 + dy**2)
    			# Distance between closest touching point of circular robot and circular barrier
    			dist = d - self.BARRIERRADIUS - self.ROBOTRADIUS
    			if (dist < closestdist):
    				closestdist = dist
    	return closestdist

    def __observe_barriers(self,x, y, theta):
    	for i, barrier in enumerate(barriers):
    		vector = (barrier[0].get_x() - x, barrier[0].get_y() - y)
    		vectorangle = math.atan2(vector[1], vector[0])
    		vectorlength = math.sqrt(vector[0]**2 + vector[1]**2)
    		anglediff = vectorangle - theta
    		while(anglediff < -math.pi):
    			anglediff = anglediff + 2 * math.pi
    		while(anglediff > math.pi):
    			anglediff = anglediff - 2 * math.pi
    		# compare anglediff with arc length barrier presents at the distance it is away
    		barrierangularhalfwidth = self.BARRIERRADIUS / vectorlength
    		if(abs(anglediff) - self.SENSORHALFANGLE < barrierangularhalfwidth and vectorlength < self.SENSORRANGE):
    			self.barriers[i][1] = True


    def get_plan(self,x,y,theta,vL,vR):
    	# Check if any new barriers are visible from current pose
    	self.__observe_barriers(x, y, theta)

    	# Planning
    	# We want to find the best benefit where we have a positive component for closeness to target,
    	# and a negative component for closeness to obstacles, for each of a choice of possible actions
    	bestBenefit = -100000
    	FORWARDWEIGHT = 12
    	OBSTACLEWEIGHT = 16

    	# Range of possible motions: each of vL and vR could go up or down a bit
    	vLpossiblearray = (vL - self.MAXACCELERATION * self.dt, vL, vL + self.MAXACCELERATION * self.dt)
    	vRpossiblearray = (vR - self.MAXACCELERATION * self.dt, vR, vR + self.MAXACCELERATION * self.dt)

    	for vLpossible in vLpossiblearray:
    		for vRpossible in vRpossiblearray:
    			# We can only choose an action if it's within velocity limits
    			if (vLpossible <= self.MAXVELOCITY and vRpossible <= self.MAXVELOCITY and vLpossible >= -self.MAXVELOCITY and vRpossible >= -self.MAXVELOCITY):
    				# Predict new position in TAU seconds
    				TAU = 1.5
    				(xpredict, ypredict, thetapredict, path) = self.__predict_position(vLpossible, vRpossible, x, y, theta, TAU)

    				# What is the distance to the closest obstacle from this possible position?
    				distanceToObstacle = self.__calculate_closest_obstacle_distance(xpredict, ypredict)

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
    	(x, y, theta, tmppath) = self.__predict_position(vL, vR, x, y, theta)

        return (vL, vR, x,y,theta)
