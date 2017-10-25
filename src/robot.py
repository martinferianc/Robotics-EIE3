import brickpi
import time
import json
import math

class Robot:
	def __init__(self, interface, config_file="paper_config.json", touch_ports = None, ultrasonic_port = None):
		# Robot initilization
		self.interface = interface
		# self.left_speed = 0
		# self.right_speed = 0
		# self.top_speed = 0
		self.wheel_diameter = 5.3 #cm
		self.circumference = self.wheel_diameter * math.pi

		self.ultra_calibration = {
			-180 : -1.49,
			-90 : 0.035,
			0 : 1.68,
			90 : 3.25,
			180 : 4.78
		}
		# Robot state
		self.state = {}
		with open("robot_state.json","r") as f:
			self.state = json.load(f)

		#Motor initialization
		self.motors = [0,1,2]
		self.motorParams = {}

		#Enabling the motors
		self.interface.motorEnable(self.motors[0])
		self.interface.motorEnable(self.motors[1])
		self.interface.motorEnable(self.motors[2])

		#Open the config file
		data = None
	    	with open(config_file) as data_file:
    			data = json.load(data_file)
		if data is None:
			raise Exception("Could not load configuration file!")
		# Configure motor calibration constants
		self.distance_calibration = data.get("distance_calibration", 3.05)
		self.angle_calibration = data.get("angle_calibration", 0.13)
		self.ultra_angle_calibration = data.get("ultra_angle_calibration", 0.15)

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

		#Configure the top motor
		self.motorParams["top"] = self.interface.MotorAngleControllerParameters()
		self.motorParams["top"].maxRotationAcceleration = data["top"]["maxRotationAcceleration"]
		self.motorParams["top"].maxRotationSpeed = data["top"]["maxRotationSpeed"]
		self.motorParams["top"].feedForwardGain = data["top"]["feedForwardGain"]
		self.motorParams["top"].minPWM = data["top"]["minPWM"]
		self.motorParams["top"].pidParameters.minOutput = data["top"]["minOutput"]
		self.motorParams["top"].pidParameters.maxOutput = data["top"]["maxOutput"]
		self.motorParams["top"].pidParameters.k_p = data["top"]["k_p"]
		self.motorParams["top"].pidParameters.k_i = data["top"]["k_i"]
		self.motorParams["top"].pidParameters.k_d = data["top"]["k_d"]

		self.interface.setMotorAngleControllerParameters(self.motors[0],self.motorParams["left"])
		self.interface.setMotorAngleControllerParameters(self.motors[1],self.motorParams["right"])
		self.interface.setMotorAngleControllerParameters(self.motors[2],self.motorParams["top"])

		# Set the angle of the ultra motor to zero
		self.calibrate_ultra_position()

		# self.interface.setMotorRotationSpeedReferences(self.motors,[self.left_speed,self.right_speed, self.top_speed])

		#Initialize the touch sensors
		self.touch_ports = touch_ports
		if self.touch_ports is not None:
			for i in touch_ports:
				self.interface.sensorEnable(i, brickpi.SensorType.SENSOR_TOUCH)

		self.ultrasonic_port = ultrasonic_port
		if self.ultrasonic_port is not None:
				self.interface.sensorEnable(i, brickpi.SensorType.SENSOR_ULTRASONIC)

	def calibrate_ultra_position(self):
		# Get current motor angle
		motor_angle = self.interface.getMotorAngles([2])[0][0]
		print("Calibration ultra position, motor angle = {}".format(motor_angle))

		# Could calibrate to the nearest angle instead of always to zero
		# Right now, calculate difference between current and zero and rotate to there
		rotation = round(self.ultra_calibration.get(0) - motor_angle,2)

		print("Rotation required: {}".format(rotation))
		self.interface.increaseMotorAngleReferences([2],[rotation])
		while not self.interface.motorAngleReferencesReached([2]):
			pass
		self.state["ultra_pose"]=0
		return True

	#Read input from the touch sensors
	def read_touch_sensor(self,port):
		if self.touch_ports is not None:
			result = self.interface.getSensorValue(port)
			if result:
  				return result[0]
		else:
			raise Exception("Touch sensors not initialized!")

	def read_ultrasonic_sensor(self,port):
		if self.ultrasonic_port is not None:
			result = self.interface.getSensorValue(port)
			if result:
	  			return result
		else:
			raise Exception("Ultrasonic sensor not initialized!")

	def save_state(self, state_file="robot_state.json"):
		with open("robot_state.json","w") as f:
			json.dump(self.state, f)

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

		self.interface.startLogging("motor_position_0_"+str(int(angle))+".log")
		self.interface.increaseMotorAngleReferences(self.motors,[radians,radians])
         	while not self.interface.motorAngleReferencesReached(self.motors):
			motorAngles = self.interface.getMotorAngles(self.motors)
			if motorAngles:
		    	    print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
			time.sleep(0.1)
		self.interface.stopLogging()

	# Move specified wheel a certain distance
	def move_wheels(self, distances=[1,1], wheels=[0,1]):
		print("Distance to move wheels: {}".format(distances))

		# Retrieve start angle of motors
		motorAngles_start = self.interface.getMotorAngles(wheels)
		print("Start Angles: {}".format(motorAngles_start))

		# Set the reference angles to reach
		circular_distances = [-round((2*x*self.distance_calibration)/self.circumference,2) for x in distances]
		print("Distance in radians: {}".format(circular_distances))
		# Angles to end at
		motorAngles_end = []
		motorAngles_end.append(round(motorAngles_start[0][0] + circular_distances[0],2))
		motorAngles_end.append(round(motorAngles_start[1][0] + circular_distances[1],2))
		print("Angles to end at: {}".format(motorAngles_end))

		self.interface.increaseMotorAngleReferences(wheels, circular_distances)

		# This function does PID control until angle references are reached
		while not self.interface.motorAngleReferencesReached(wheels):
			#time.sleep(0.1)
			#print(self.interface.getMotorAngles(wheels))
			if (round(self.interface.getMotorAngles(wheels)[0][0],2)==motorAngles_end[0] or round(self.interface.getMotorAngles(wheels)[1][0],2)==motorAngles_end[1]):
				return True
		return True

	# Rotate a motor by angle degrees (mainly for ultrasound motor)
	def rotate_motor(self, angles=[0], motors=[2]):
		print("Starting reference angles: {}".format(self.interface.getMotorAngles(motors)))
		self.interface.increaseMotorAngleReferences(motors, [x*self.ultra_angle_calibration for x in angles])
		# This function does PID control until angle references are reached
		while not self.interface.motorAngleReferencesReached(motors):
			pass
		print("Ending reference angles: {}".format(self.interface.getMotorAngles(motors)))
		return True

	#Takes the angle in degrees and rotates the robot right
	def rotate_right(self, angle):
		#print("Starting pose: {}".format(self.state.get("pose")))
		dist = self.angle_calibration*angle
		self.state["pose"] = self.state.get("pose", 0) + angle
		#print("New pose: {}".format(self.state.get("pose")))
		# Maybe only save state when the robot is shutting down?
		self.save_state()

		return self.move_wheels([dist,-dist])

	#Takes the angle in degrees and rotates the robot left
	def rotate_left(self, angle):
		return self.rotate_right(-angle)

	#Does the immediate stop if it runs into an obstacle
	def stop(self):
		self.interface.setMotorPwm(self.motors[0],0)
		self.interface.setMotorPwm(self.motors[1],0)

	#Takes the distance in centimeters and moves it forward
	def travel_straight(self, distance):
		return self.move_wheels([distance,distance], [0,1])

	# Move the top camera to specified pose
	def set_ultra_pose(self, pose):
		print("Current ultra pose: {}".format(self.state.get("ultra_pose", -1)))
		# Limits on pose settings so that it doesn't overrotate and stretch the cable
		while pose > 360:
			pose -= 360
		while pose < -360:
			pose += 360
		if pose == 360:
			pose = 0

		if pose > 180:
			# If greater than 180 e.g. 270, turn it into -90
			pose -= 360
		if pose < -180:
			# If less than -180, e.g. -270, turn it into +90
			pose += 360

		rotation = pose - self.state.get("ultra_pose", 0)
		if rotation:
			self.rotate_motor([rotation])
			self.state["ultra_pose"] = pose
			self.save_state()
		else:
			print("No rotation required.")
		return True

	# Move the robot to the specified pose
	def set_robot_pose(self, s_pose):
		print("Starting pose: {}".format(self.state.get("pose",-1)))
		while s_pose > 360:
			s_pose-=360

		rotation = s_pose-self.state.get("pose", 0)
		if rotation==0:
			print("No rotation required.")
			return True
		if rotation > 180:
			self.rotate_right(rotation-360)
		else:
			self.rotate_right(rotation)
		self.state["pose"] = s_pose
		print("Ending pose: {}".format(s_pose))
		self.save_state()
		return True

	def interactive_mode(self):
		command = 0
		while command!=-1:
			print("Available commands:\n-1: End session.\n1: Travel straight.\n2: Set pose.\n3: Move wheels.\n4: Set ultra pose.")
			command = int(input())
			if command == 1:
				print("Enter distance to move straight: ")
				distance = float(input())
				self.travel_straight
			elif command==2:
				print("Enter pose to rotate to:")
				s_pose = float(input())
				self.set_robot_pose(s_pose)

			elif command == 3:
				distances = []
				print("Enter left wheel distance:")
				distances.append(float(input()))
				print("Enter right wheel distance:")
				distances.append(float(input()))
				self.move_wheels(distances)
			elif command == 4:
				print("Enter desired camera pose:")
				s_pose = float(input())
				self.set_ultra_pose(s_pose)
			else:
				command = -1
				self.stop()
		return True
