# Planning
# Dynamic Window Approach (Local Planning)
# Andrew Davison 2017
import os, math, time, random

class Planner:
    def __init__(self, y,x=0,theta=0,radius = 0.06, size = 0.2, safe_dist=0.2, playfield=(0, 0, 3.0, 2.0), target=(3.1,0), canvas = None):
        # Constants and variables
        # Units here are in metres and radians using our standard coordinate frame
        self.obstacleRADIUS = radius
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

        # obstacle (obstacle) locations
        self.obstacles = []
        # obstacle contents are (bx, by, visibilitymask)
        # Generate some initial random obstacles
        # Simulation of forward looking depth sensor; assume cone beam
        # Which obstacles can it see? If a obstacle has been seen at least once it becomes known to the planner
        self.SENSORRANGE = 1.5
        self.SENSORHALFANGLE = 15.0 * math.pi / 180.0
        self.canvas = canvas

    def append_obstacle(self, obstacle, threshold = 2):
        for obs in self.obstacles:
            x = obs.get_x()
            y = obs.get_y()
            diff_x = abs(x-obstacle.get_x())
            diff_y = abs(y-obstacle.get_y())
            if diff_x<threshold and diff_y<threshold:
                return False
        self.obstacles.append([obstacle,0])
        x = obstacle.get_x()
        y = obstacle.get_y()
        radius = obstacle.get_radius()
        std_x,std_y = obstacle.get_std()
        self.canvas.drawCross((x,y), size_x = std_x+radius,size_y = std_y+radius)
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
    		# Is this a obstacle we know about? obstacle[2] flag is set when sonar observes it
    		if obstacle[1] is True:
    			dx = obstacle[0].get_x() - x
    			dy = obstacle[0].get_y() - y
    			d = math.sqrt(dx**2 + dy**2)
    			# Distance between closest touching point of circular robot and circular obstacle
    			dist = d - self.obstacleRADIUS - self.ROBOTRADIUS
    			if (dist < closestdist):
    				closestdist = dist
    	return closestdist

    def __observe_obstacles(self,x, y, theta):
    	for i, obstacle in enumerate(self.obstacles):
    		vector = (obstacle[0].get_x() - x, obstacle[0].get_y() - y)
    		vectorangle = math.atan2(vector[1], vector[0])
    		vectorlength = math.sqrt(vector[0]**2 + vector[1]**2)
    		anglediff = vectorangle - theta
    		while(anglediff < -math.pi):
    			anglediff = anglediff + 2 * math.pi
    		while(anglediff > math.pi):
    			anglediff = anglediff - 2 * math.pi
    		# compare anglediff with arc length obstacle presents at the distance it is away
    		obstacleangularhalfwidth = self.obstacleRADIUS / vectorlength
    		if(abs(anglediff) - self.SENSORHALFANGLE < obstacleangularhalfwidth and vectorlength < self.SENSORRANGE):
    			self.obstacles[i][1] = True


    def get_plan(self,x,y,theta,vL,vR):
    	# Check if any new obstacles are visible from current pose
    	self.__observe_obstacles(x, y, theta)

    	# Planning
    	# We want to find the best benefit where we have a positive component for closeness to target,
    	# and a negative component for closeness to obstacles, for each of a choice of possible actions
    	bestBenefit = -100000
    	FORWARDWEIGHT = 12
    	OBSTACLEWEIGHT = 16

    	# Range of possible motions: each of vL and vR could go up or down a bit
    	vLpossiblearray = (vL - self.MAXACCELERATION * self.dt, vL, vL + self.MAXACCELERATION * self.dt)
    	vRpossiblearray = (vR - self.MAXACCELERATION * self.dt, vR, vR + self.MAXACCELERATION * self.dt)

        vLchosen = 0
        vRchosen = 0

    	for vLpossible in vLpossiblearray:
    		for vRpossible in vRpossiblearray:
    			# We can only choose an action if it's within velocity limits
    			if (vLpossible <= self.MAXVELOCITY and vRpossible <= self.MAXVELOCITY and vLpossible >= -self.MAXVELOCITY and vRpossible >= -self.MAXVELOCITY):
    				# Predict new position in TAU seconds
    				TAU = 1.5
    				(xpredict, ypredict, thetapredict)= self.__predict_position(vLpossible, vRpossible, x, y, theta, TAU)

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
    	(x, y, theta) = self.__predict_position(vL, vR, x, y, theta, self.dt)

        return (vL, vR, x,y,theta)
