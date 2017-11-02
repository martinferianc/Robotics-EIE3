import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

NUMBER_OF_PARTICLES = None

Robot = Robot(interface, pid_config_file="carpet_config.json")
for i in range(4):
    for j in range(4):
        NUMBER_OF_PARTICLES = Robot.travel_straight(10,update_particles=False)
        #NUMBER_OF_PARTICLES = [point[0] for point in NUMBER_OF_PARTICLES]
        #print "drawParticles:" + str(NUMBER_OF_PARTICLES)
        time.sleep(5)
    NUMBER_OF_PARTICLES = Robot.rotate_left(90,update_particles=False)
    #NUMBER_OF_PARTICLES = [point[0] for point in NUMBER_OF_PARTICLES]
    #print "drawParticles:" + str(NUMBER_OF_PARTICLES)
    time.sleep(5)

interface.terminate()
