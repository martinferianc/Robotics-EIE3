# Planning
# Dynamic Window Approach (Local Planning)
# Andrew Davison 2017
import pygame, os, math, time, random
from pygame.locals import *


pygame.init()



# Constants and variables
# Units here are in metres and radians using our standard coordinate frame
BARRIERRADIUS = 0.06
ROBOTRADIUS = 0.15
W = 2 * ROBOTRADIUS # width of robot
SAFEDIST = 0.2      # used in the cost function for avoiding obstacles

MAXVELOCITY = 0.5     #ms^(-1) max speed of each wheel
MAXACCELERATION = 0.5 #ms^(-2) max rate we can change speed of each wheel


# The region we will fill with obstacles
PLAYFIELDCORNERS = (-3.0, -3.0, 3.0, 3.0)

# Set an initial target location which is beyond the obstacles
target = (PLAYFIELDCORNERS[2] + 1.0, 0)

# Starting pose of robot
x = PLAYFIELDCORNERS[0] - 0.5
y = 0.0
theta = 0.0

# Starting wheel velocities
vL = 0.00
vR = 0.00

# Timestep delta to run control and simulation at
dt = 0.1


# Barrier (obstacle) locations
barriers = []
# barrier contents are (bx, by, visibilitymask)
# Generate some initial random barriers
for i in range(2):
	(bx, by) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]))
	barrier = [bx, by, 0]
	barriers.append(barrier)



# Constants for graphics display
# Transformation from metric world frame to graphics frame
# k pixels per metre
# Horizontal screen coordinate:     u = u0 + k * x
# Vertical screen coordinate:       v = v0 - k * y

# set the width and height of the screen (pixels)
WIDTH = 1500
HEIGHT = 1000

size = [WIDTH, HEIGHT]
black = (20,20,40)
lightblue = (0,120,255)
darkblue = (0,40,160)
red = (255,100,0)
white = (255,255,255)
blue = (0,0,255)
k = 160 # pixels per metre for graphics

# Screen centre will correspond to (x, y) = (0, 0)
u0 = WIDTH / 2
v0 = HEIGHT / 2




# Initialise Pygame display screen
screen = pygame.display.set_mode(size)
# This makes the normal mouse pointer invisible in graphics window
pygame.mouse.set_visible(0)


# Array for path choices use for graphics
pathstodraw = []






# Function to predict new robot position based on current pose and velocity controls
# Uses time deltat in future
# Returns xnew, ynew, thetanew
# Also returns path. This is just used for graphics, and returns some complicated stuff
# used to draw the possible paths during planning. Don't worry about the details of that.
def predictPosition(vL, vR, x, y, theta, deltat):
	# Simple special cases
	# Straight line motion
	if (vL == vR):
		xnew = x + vL * deltat * math.cos(theta)
		ynew = y + vL * deltat * math.sin(theta)
		thetanew = theta
		path = (0, vL * deltat)   # 0 indicates pure translation
	# Pure rotation motion
	elif (vL == -vR):
		xnew = x
		ynew = y
		thetanew = theta + ((vR - vL) * deltat / W)
		path = (1, 0) # 1 indicates pure rotation
	else:
		# Rotation and arc angle of general circular motion
		# Using equations given in Lecture 2
		R = W / 2.0 * (vR + vL) / (vR - vL)
		deltatheta = (vR - vL) * deltat / W
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
		path = (2, ((tlx, tly), (Rx, Ry)), start_angle, stop_angle) # 2 indicates general motion

	return (xnew, ynew, thetanew, path)

# Function to calculate the closest obstacle at a position (x, y)
# Used during planning
def calculateClosestObstacleDistance(x, y):
	closestdist = 100000.0
	# Calculate distance to closest obstacle
	for barrier in barriers:
		# Is this a barrier we know about? barrier[2] flag is set when sonar observes it
		if(barrier[2] == 1):
			dx = barrier[0] - x
			dy = barrier[1] - y
			d = math.sqrt(dx**2 + dy**2)
			# Distance between closest touching point of circular robot and circular barrier
			dist = d - BARRIERRADIUS - 	ROBOTRADIUS
			if (dist < closestdist):
				closestdist = dist
	return closestdist

# Draw the barriers on the screen
def drawBarriers(barriers):
	for barrier in barriers:
		# Dark barriers we haven't seen
		if(barrier[2] == 0):
			pygame.draw.circle(screen, darkblue, (int(u0 + k * barrier[0]), int(v0 - k * barrier[1])), int(k * BARRIERRADIUS), 0)
		# Bright barriers we have seen
		else:
			pygame.draw.circle(screen, lightblue, (int(u0 + k * barrier[0]), int(v0 - k * barrier[1])), int(k * BARRIERRADIUS), 0)
	return

# Simulation of forward looking depth sensor; assume cone beam
# Which barriers can it see? If a barrier has been seen at least once it becomes known to the planner
SENSORRANGE = 1.5
SENSORHALFANGLE = 15.0 * math.pi / 180.0
def observeBarriers(x, y, theta):
	for i, barrier in enumerate(barriers):
		vector = (barrier[0] - x, barrier[1] - y)
		vectorangle = math.atan2(vector[1], vector[0])
		vectorlength = math.sqrt(vector[0]**2 + vector[1]**2)
		anglediff = vectorangle - theta
		while(anglediff < -math.pi):
			anglediff = anglediff + 2 * math.pi
		while(anglediff > math.pi):
			anglediff = anglediff - 2 * math.pi
		# compare anglediff with arc length barrier presents at the distance it is away
		barrierangularhalfwidth = BARRIERRADIUS / vectorlength
		if(abs(anglediff) - SENSORHALFANGLE < barrierangularhalfwidth and vectorlength < SENSORRANGE):
			barriers[i][2] = 1


# Main loop
while(1):
	# Check if any new barriers are visible from current pose
	observeBarriers(x, y, theta)

	# Planning
	# We want to find the best benefit where we have a positive component for closeness to target,
	# and a negative component for closeness to obstacles, for each of a choice of possible actions
	bestBenefit = -100000
	FORWARDWEIGHT = 12
	OBSTACLEWEIGHT = 16

	# Range of possible motions: each of vL and vR could go up or down a bit
	vLpossiblearray = (vL - MAXACCELERATION * dt, vL, vL + MAXACCELERATION * dt)
	vRpossiblearray = (vR - MAXACCELERATION * dt, vR, vR + MAXACCELERATION * dt)
	print "New action"
	pathstodraw = [] # We will store path details here for plotting later
	newpositionstodraw = [] # Also for possible plotting of robot end positions
	for vLpossible in vLpossiblearray:
		for vRpossible in vRpossiblearray:
			# We can only choose an action if it's within velocity limits
			if (vLpossible <= MAXVELOCITY and vRpossible <= MAXVELOCITY and vLpossible >= -MAXVELOCITY and vRpossible >= -MAXVELOCITY):
				# Predict new position in TAU seconds
				TAU = 1.5
				(xpredict, ypredict, thetapredict, path) = predictPosition(vLpossible, vRpossible, x, y, theta, TAU)
				pathstodraw.append(path)
				newpositionstodraw.append((xpredict, ypredict))
				# What is the distance to the closest obstacle from this possible position?
				distanceToObstacle = calculateClosestObstacleDistance(xpredict, ypredict)
				# Calculate how much close we've moved to target location
				previousTargetDistance = math.sqrt((x - target[0])**2 + (y - target[1])**2)
				newTargetDistance = math.sqrt((xpredict - target[0])**2 + (ypredict - target[1])**2)
				distanceForward = previousTargetDistance - newTargetDistance
				# Alternative: how far have I moved forwards?
				# distanceForward = xpredict - x
				# Positive benefit
				distanceBenefit = FORWARDWEIGHT * distanceForward
				# Negative cost: once we are less than SAFEDIST from collision, linearly increasing cost
				if (distanceToObstacle < SAFEDIST):
					obstacleCost = OBSTACLEWEIGHT * (SAFEDIST - distanceToObstacle)
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



    # Planning is finished; now do graphics
	screen.fill(black)
	drawBarriers(barriers)
	# Target location
	pygame.draw.circle(screen, red, (int(u0 + k * target[0]), int(v0 - k * target[1])), int(k * ROBOTRADIUS), 0)

	# Draw robot
	u = u0 + k * x
	v = v0 - k * y
	pygame.draw.circle(screen, white, (int(u), int(v)), int(k * ROBOTRADIUS), 3)
	# Draw wheels as little blobs so you can see robot orientation
	# left wheel centre
	wlx = x - (W/2.0) * math.sin(theta)
	wly = y + (W/2.0) * math.cos(theta)
	ulx = u0 + k * wlx
	vlx = v0 - k * wly
	WHEELBLOB = 0.04
	pygame.draw.circle(screen, blue, (int(ulx), int(vlx)), int(k * WHEELBLOB))
	# right wheel centre
	wrx = x + (W/2.0) * math.sin(theta)
	wry = y - (W/2.0) * math.cos(theta)
	urx = u0 + k * wrx
	vrx = v0 - k * wry
	pygame.draw.circle(screen, blue, (int(urx), int(vrx)), int(k * WHEELBLOB))

	# Draw paths: little arcs which show the different paths the robot is selecting between
	# A bit complicated so don't worry about the details!
	for path in pathstodraw:
		#if path[0] = 1:    # Pure rotation: nothing to draw
		if path[0] == 0:    # Straight line
			straightpath = path[1]
			linestart = (u0 + k * x, v0 - k * y)
			lineend = (u0 + k * (x + straightpath * math.cos(theta)), v0 - k * (y + straightpath * math.sin(theta)))
			pygame.draw.line(screen, (0, 200, 0), linestart, lineend, 1)
		if path[0] == 2:    # General case: circular arc
			# path[2] and path[3] are start and stop angles for arc but they need to be in the right order to pass
			if (path[3] > path[2]):
				startangle = path[2]
				stopangle = path[3]
			else:
				startangle = path[3]
				stopangle = path[2]
			# Pygame arc doesn't draw properly unless angles are positive
			if (startangle < 0):
				startangle += 2*math.pi
				stopangle += 2*math.pi
			if (path[1][1][0] != 0):
				pygame.draw.arc(screen, (0, 200, 0), path[1], startangle, stopangle, 1)

	# Uncomment to also draw circles for predicted end positions of robot
	#for newposition in newpositionstodraw:
		#u = u0 + k * newposition[0]
		#v = v0 - k * newposition[1]
		#pygame.draw.circle(screen, (0,100,0), (int(u), int(v)), int(k * ROBOTRADIUS), 3)
		#time.sleep(1.0)
		#pygame.display.flip()

	# Update display
	pygame.display.flip()


	# Actually now move robot based on chosen vL and vR
	(x, y, theta, tmppath) = predictPosition(vL, vR, x, y, theta, dt)


	# Wraparound: check if robot has reached target; if so reset it to the other side, randomise
	# target position and add some more barriers to go again
	disttotarget = math.sqrt((x - target[0])**2 + (y - target[1])**2)
	if (disttotarget < 0.05):
		# Reset robot pose
		x = PLAYFIELDCORNERS[0] - 0.5
		y = random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3])
		theta = 0
		# Reset barrier visibility
		for i, barrier in enumerate(barriers):
			barriers[i][2] = 0
		# Add new barriers
		for i in range(2):
			(bx, by) = (random.uniform(PLAYFIELDCORNERS[0], PLAYFIELDCORNERS[2]), random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]))
			barrier = [bx, by, 0]
			barriers.append(barrier)
		# Randomise target
		target = (target[0], random.uniform(PLAYFIELDCORNERS[1], PLAYFIELDCORNERS[3]))

	# Sleeping dt here runs simulation in real-time
	time.sleep(dt / 5)
