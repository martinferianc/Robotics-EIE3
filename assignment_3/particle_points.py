import time
from src.robot import Robot
import brickpi
import sys

interface = brickpi.Interface()
interface.initialize()
config = str(sys.argv[1])
Robot = Robot(interface, pid_config_file = config)

NUMBER_OF_SAMPLES = 100
samples[NUMBER_OF_SAMPLES]

samples = Robot.generate_uncertainty_dict(NUMBER_OF_SAMPLES)
samples = Robot.update_projected_points("x", 10, 0.5, NUMBER_OF_SAMPLES, samples)
