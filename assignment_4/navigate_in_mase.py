import threading
import time
from src.robot import Robot
from src.drawing import Map, Canvas
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

PARTICLES = None

MAP = [[0,0,"O"],
       [0,168,"A"],
       [84,168,"B"],
       [84,126,"C"],
       [84,210,"D"],
       [168,210,"E"],
       [168,84,"F"],
       [210,84,"G"],
       [210,0,"H"]]

Robot = Robot(interface, pid_config_file="carpet_config.json")
while True:
    X = float(input("Type in the X coordinate: "))
    Y = float(input("Type in the Y coordinate: "))
    Robot.navigate_to_waypoint(X,Y)
    NUMBER_OF_PARTICLES = Robot.get_state()
    NUMBER_OF_PARTICLES = [point[0] for point in NUMBER_OF_PARTICLES]
    print "drawParticles:" + str(NUMBER_OF_PARTICLES)

interface.terminate()
