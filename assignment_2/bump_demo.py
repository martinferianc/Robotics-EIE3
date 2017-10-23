import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

bumper_1_port = 0
bumper_2_port = 1

Robot = Robot(interface)
bumpers = [None,None]

while True:
    Robot.travel_straight(1)
    bumpers[0] = Robot.read_touch_sensor(bumper_1_port)
    bumpers[1] = Robot.read_touch_sensor(bumper_2_port)

    if bumpers[0] and bumpers[1]:
        Robot.travel_straight(-3)
        Robot.rotate_right(30)

    if !bumpers[0] and bumpers[1]:
        Robot.rotate_right(30)

    if !bumpers[0] and !bumpers[1]:
        Robot.rotate_left(30)

interface.terminate()
