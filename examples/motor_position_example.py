import time


import brickpi
import motor_params


interface = motor_params.interface
motors = motor_params.motors

interface.startLogging("motor_position_example.log")

while True:
    angle = float(input("Enter a angle to rotate (in radians): "))

    interface.increaseMotorAngleReferences(motors,[angle,angle])

    while not interface.motorAngleReferencesReached(motors) :
	motorAngles = interface.getMotorAngles(motors)
	if motorAngles :
	    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
	time.sleep(0.1)

    print "Destination reached!"


interface.stopLogging()
interface.terminate()
