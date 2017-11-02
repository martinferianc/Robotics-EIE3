import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

NUMBER_OF_PARTICLES = None

OFFSET = 100
DISTANCE_TO_PIXEL = 15

#Draw the Grid
for j in range(4):
    print "drawLine:" + str((OFFSET,OFFSET*10*i,40*DISTANCE_TO_PIXEL+OFFSET,OFFSET*10*i))
    print "drawLine:" + str((OFFSET*i*10,40*DISTANCE_TO_PIXEL+OFFSET,OFFSET*i*10,OFFSET))

Robot = Robot(interface, pid_config_file="carpet_config.json")
while True:
    X = int(input("Type in the X coordinate: "))
    Y = int(input("Type in the Y coordinate: "))
    NUMBER_OF_PARTICLES = Robot.navigate_to_waypoint(X,Y)
    NUMBER_OF_PARTICLES = [point[0] for point in NUMBER_OF_PARTICLES]
    print "drawParticles:" + str(NUMBER_OF_PARTICLES)

interface.terminate()
