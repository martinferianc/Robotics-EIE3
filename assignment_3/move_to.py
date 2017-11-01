import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

NUMBER_OF_PARTICLES = None

Robot = Robot(interface, pid_config_file="carper_config.json")
while True:
    X = int(input("Type in the X coordinate: "))
    Y = int(input("Type in the Y coordinate: "))
    NUMBER_OF_PARTICLES = Robot.move_to(X,Y,update_particles=True)
    NUMBER_OF_PARTICLES = [point[0] for point in NUMBER_OF_PARTICLES]
    print "drawParticles:" + str(NUMBER_OF_PARTICLES)

interface.terminate()
