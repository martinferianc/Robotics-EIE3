import threading
import time

import brickpi

#Initialize the interface
interface = brickpi.Interface()
interface.initialize()
interface.startLogging("motor_position_first_test.log")
motors = [0,1]
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParamsLeft = interface.MotorAngleControllerParameters()
motorParamsLeft.maxRotationAcceleration = 6.0
motorParamsLeft.maxRotationSpeed = 12.0
motorParamsLeft.feedForwardGain = 255/20.0
motorParamsLeft.minPWM = 18.0
motorParamsLeft.pidParameters.minOutput = -255
motorParamsLeft.pidParameters.maxOutput = 255
motorParamsLeft.pidParameters.k_p = 100.0
motorParamsLeft.pidParameters.k_i = 0.0
motorParamsLeft.pidParameters.k_d = 0.0

motorParamsRight = interface.MotorAngleControllerParameters()
motorParamsRight.maxRotationAcceleration = 6.0
motorParamsRight.maxRotationSpeed = 12.0
motorParamsRight.feedForwardGain = 255/20.0
motorParamsRight.minPWM = 18.0
motorParamsRight.pidParameters.minOutput = -255
motorParamsRight.pidParameters.maxOutput = 255
motorParamsRight.pidParameters.k_p = 100.0
motorParamsRight.pidParameters.k_i = 0.0
motorParamsRight.pidParameters.k_d = 0.0

interface.setMotorAngleControllerParameters(motors[0],motorParamsLeft)
interface.setMotorAngleControllerParameters(motors[1],motorParamsRight)

def forward():


def rotateLeft():
    pass

def rotateRight():
    pass

interface.terminate()
