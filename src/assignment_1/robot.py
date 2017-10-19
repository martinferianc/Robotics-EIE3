import brickpi
import time
import json
import math

class Robot:
	def __init__(self, interface, config_file="config.json"):
		self.motors = [0,1]
		self.motorParams = {}
		self.interface = interface
		self.left_speed = 0
		self.right_speed = 0
		self.wheel_diameter = 5.3 #cm
		self.circumference = self.wheel_diameter * math.pi

		#Enabling the motors
		self.interface.motorEnable(self.motors[0])
		self.interface.motorEnable(self.motors[1])

		#Open the config file
		data = None
	    with open("config.json") as data_file:
    		data = json.load(data_file)

		#Configuring the left motor
		self.motorParams["left"] = self.interface.MotorAngleControllerParameters()
		self.motorParams["left"].maxRotationAcceleration = data["left"]["maxRotationAcceleration"]
		self.motorParams["left"].maxRotationSpeed = data["left"]["maxRotationSpeed"]
		self.motorParams["left"].feedForwardGain = data["left"]["feedForwardGain"]
		self.motorParams["left"].minPWM = data["left"]["minPWM"]
		self.motorParams["left"].pidParameters.minOutput = data["left"]["minOutput"]
		self.motorParams["left"].pidParameters.maxOutput = data["left"]["maxOutput"]
		self.motorParams["left"].pidParameters.k_p = data["left"]["k_p"]
		self.motorParams["left"].pidParameters.k_i = data["left"]["k_i"]
		self.motorParams["left"].pidParameters.k_d = data["left"]["k_d"]

		#Configure the right motor
		self.motorParams["right"] = self.interface.MotorAngleControllerParameters()
		self.motorParams["right"].maxRotationAcceleration = data["right"]["maxRotationAcceleration"]
		self.motorParams["right"].maxRotationSpeed = data["right"]["maxRotationSpeed"]
		self.motorParams["right"].feedForwardGain = data["right"]["feedForwardGain"]
		self.motorParams["right"].minPWM = data["right"]["minPWM"]
		self.motorParams["right"].pidParameters.minOutput = data["right"]["minOutput"]
		self.motorParams["right"].pidParameters.maxOutput = data["right"]["maxOutput"]
		self.motorParams["right"].pidParameters.k_p = data["right"]["k_p"]
		self.motorParams["right"].pidParameters.k_i = data["right"]["k_i"]
		self.motorParams["right"].pidParameters.k_d = data["right"]["k_d"]

		self.interface.setMotorAngleControllerParameters(self.motors[0],self.motorParams["left"])
		self.interface.setMotorAngleControllerParameters(self.motors[1],self.motorParams["right"])

		self.interface.setMotorRotationSpeedReferences(self.motors,[self.left_speed,self.right_speed])

	#Takes the distance in centimeters and moves it forward

	#Takes the distance in centimeters and moves it backwards
	def travel_straight(self,distance):
		motorAngles_start = self.interface.getMotorAngles(self.motors)
		revolutions_radians = (self.circumference/distance)*2*math.pi
		motorAngles_end = (-round(motorAngles_start[0][0] + revolutions_radians, 2), -round(motorAngles_start[1][0] + revolutions_radians, 2))
		self.interface.increaseMotorAngleReferences(self.motors,[motorAngles_end[0],motorAngles_end[1]])
		while not self.interface.motorAngleReferencesReached(self.motors):
			motorAngles = self.interface.getMotorAngles(self.motors)

	# Move specified wheel a certain distance
	def move_wheels(self, distance=1, wheels=[0,1], speed=[2,2]):
		# Set speed reference
		#self.interface.setMotorRotationSpeedReferences(wheels)

		# Retrieve start angle of motors
		motorAngles_start = self.interface.getMotorAngles(wheels)

		# Angle to rotate the wheels in radians
		revolutions_radians = (self.circumference/distance)*2*math.pi

		# Set the reference angles to reach
		motorAngles_end = [round(x+revolutions_radians,2) for x in motorAngles_start]

		self.interface.increaseMotorAngleReferences(wheels, motorAngles_end)



	#Takes the angle in degrees and rotates the robot right
	def rotateRight(self, angle):
		pass

	#Takes the angle in degrees and rotates the robot left
	def rotateLeft(self, angle):
		pass

	def calibrate(self, radians,angle):
		#So that we always start calibrating approximately at zero
		motorAngles = self.interface.getMotorAngles(self.motors)
		motorAngles_zero = (round(0-motorAngles[0][0],2), round(0-motorAngles[1][0],2))
		self.interface.increaseMotorAngleReferences(self.motors,[motorAngles_zero[0],motorAngles_zero[1]])
		while not self.interface.motorAngleReferencesReached(self.motors):
			motorAngles = self.interface.getMotorAngles(self.motors)
			if motorAngles:
				print "Motor angles calibrating to 0: ", motorAngles[0][0], ", ", motorAngles[1][0]
			time.sleep(0.1)

		self.interface.startLogging("motor_position_6_"+str(int(angle))+".log")
		self.interface.increaseMotorAngleReferences(self.motors,[radians,radians])
         	while not self.interface.motorAngleReferencesReached(self.motors):
			motorAngles = self.interface.getMotorAngles(self.motors)
			if motorAngles:
		    	    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
			time.sleep(0.1)
		self.interface.stopLogging()
