from __future__ import division
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

POINTS = [(86,30),
          (180,30),
          (180,53),
          (138,54),
          (138,168),
          (114,168),
          (114,84),
          (86,84),
          (86,30)]

Canvas = Canvas()
Map = Map(Canvas)

Robot = Robot(interface,
              pid_config_file="carpet_config.json",
              Map=MAP,
              mcl=True,
              x=86,
              y=30,
              mode="continuous",
              theta=0,
              threading=True,
              canvas=Canvas
              )
Map.add_wall((0,0,0,168))        # a
Map.add_wall((0,168,84,168))     # b
Map.add_wall((84,126,84,210))    # c
Map.add_wall((84,210,168,210))   # d
Map.add_wall((168,210,168,84))   # e
Map.add_wall((168,84,210,84))    # f
Map.add_wall((210,84,210,0))     # g
Map.add_wall((210,0,0,0))        # h
Map.draw()


for x,y in POINTS:
    Robot.step_to_waypoint(x,y, maxdistance=20)

Robot.stop_threading()
interface.terminate()
