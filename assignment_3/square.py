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

#Draw the square
print "drawLine:" + str((OFFSET,OFFSET,40*DISTANCE_TO_PIXEL+OFFSET,OFFSET))
print "drawLine:" + str((40*DISTANCE_TO_PIXEL+OFFSET,OFFSET,40*DISTANCE_TO_PIXEL+OFFSET,OFFSET+40*DISTANCE_TO_PIXEL))
print "drawLine:" + str((40*DISTANCE_TO_PIXEL+OFFSET,40*DISTANCE_TO_PIXEL+OFFSET,OFFSET,40*DISTANCE_TO_PIXEL+OFFSET))
print "drawLine:" + str((OFFSET,40*DISTANCE_TO_PIXEL+OFFSET,OFFSET,OFFSET))

Robot = Robot(interface, pid_config_file="carpet_config.json")
for i in range(4):
    for j in range(4):
        NUMBER_OF_PARTICLES = Robot.travel_straight(10,update_particles=True)
        NUMBER_OF_PARTICLES = [(OFFSET+point[0][0]*DISTANCE_TO_PIXEL,OFFSET+point[0][1]*DISTANCE_TO_PIXEL,point[0][2]) for point in NUMBER_OF_PARTICLES]
        print "drawParticles:" + str(NUMBER_OF_PARTICLES)
        time.sleep(5)
    NUMBER_OF_PARTICLES = Robot.rotate_right(90,update_particles=True)
    NUMBER_OF_PARTICLES = [(OFFSET+point[0][0]*DISTANCE_TO_PIXEL,OFFSET+point[0][1]*DISTANCE_TO_PIXEL,point[0][2]) for point in NUMBER_OF_PARTICLES]
    print "drawParticles:" + str(NUMBER_OF_PARTICLES)
    time.sleep(5)

interface.terminate()
