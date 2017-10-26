import threading
import time
from src.robot import Robot
import brickpi

#Initialize the interface
interface=brickpi.Interface()
interface.initialize()

left_bumper_port = 2
right_bumper_port = 0

Robot = Robot(interface, pid_config_file="carpet_config.json")
bumpers = [None,None]

while True:
    Robot.travel_straight(1)
    bumpers[0] = Robot.read_touch_sensor(left_bumper_port)
    bumpers[1] = Robot.read_touch_sensor(right_bumper_port)

    if bumpers[0] and bumpers[1]:
        Robot.stop()
        Robot.travel_straight(-3)
        Robot.rotate_right(30)

    if !bumpers[0] and bumpers[1]:
        Robot.stop()
        Robot.rotate_right(30)

    if !bumpers[0] and !bumpers[1]:
        Robot.stop()
        Robot.rotate_left(30)

interface.terminate()
