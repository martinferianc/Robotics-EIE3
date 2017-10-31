import time
from src.robot import Robot
import brickpi
import sys

interface = brickpi.Interface()
interface.initialize()
config = str(sys.argv[1])
Robot = Robot(interface, pid_config_file = config)
Robot.set_speed([0,0])

NUMBER_OF_SAMPLES = 100
samples = [NUMBER_OF_SAMPLES]

Robot.draw_square()
samples = Robot.initialize_uncertainty_points(NUMBER_OF_SAMPLES)
samples = Robot.update_projected_points("x", samples, 10, 0.5, NUMBER_OF_SAMPLES)
Robot.plot_points(samples)
samples = Robot.update_projected_points("x", samples, 10, 0.5, NUMBER_OF_SAMPLES)
Robot.plot_points(samples)

interface.terminate()
