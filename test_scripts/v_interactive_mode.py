import threading
import time
import sys
from src.robot import Robot
from src.virtual_interface import Interface

#Initialize the interface
interface=Interface()
interface.initialize()
interface.startLogging("motor_position_1.log")

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

POINTS = [(84,30),
          (180,30),
          (180,53),
          (138,54),
          (138,168),
          (114,168),
          (114,84),
          (84,84),
          (84,30),
          (0,0)]

Robot = Robot(interface,
              pid_config_file="carpet_config.json",
              Map=MAP,
              mcl=True,
              x=0.2,
              y=0.2,
              mode="continuous",
              theta=90,
              threading=True
              )

Robot.interactive_mode()

interface.stopLogging()
interface.terminate()
