from __future__ import division
import time
import json
import math
from obstacle import Obstacle
from thread import Poller
from particle import ParticleState
from collections import deque
from planning import Planner
import numpy as np
import random
import os
import brickpi

class Robot:
    ## INTITIALIZATION FUNCTIONS
    def __init__(self,
                 interface,
                 pid_config_file="paper_config.json",
                 config_file="base_config.json",
                 threading=False,
                 x = 0,
                 y = 0,
                 theta = 0,
                 mode = "continuous",
                 mcl = False,
                 Map = None,
                 canvas = None,
                 planning = False):
        # Robot initilization
        self.interface = interface
        self.mcl = mcl
        self.Map = Map
        self.canvas = canvas
        self.print_thread = None
        self.wheel_diameter = 5.3 #cm
        self.circumference = self.wheel_diameter * math.pi
        self.distance = 0
        self.distance_stack = deque(maxlen=15)
        if planning:
            self.planner = Planner(0,canvas = self.canvas)

        self.distances = {
            -90:255,
            -45:255,
            0:255,
            45:255,
            90:255
        }

        self.motor_speeds = [0,0]
        self.threads = []

        self.max_sd_error = 3

        # Robot state
        self.state = {'pose':{'x':x, 'y': y, 'theta': theta}, 'ultra_pose': 0}
        if(os.path.isfile("robot_state.json")):
            try:
                with open("robot_state.json","r") as f:
                    self.state = json.load(f)
            except Exception as e:
                print "Error reading from the JSON file."
        self.config_file = config_file
        self.pid_config_file = pid_config_file

        self.load_base_config()
        self.load_pid_config()
        self.particle_state = ParticleState(standard_deviation = self.standard_deviation,n_particles=300,x = x,y = y,theta=theta,mode=mode,mcl = self.mcl,Map = self.Map)
        if threading:
            self.start_threading()


    def load_base_config(self):
        # configure main settings
        with open(self.config_file) as config_file:
            data = json.load(config_file)
        if data is None:
            raise Exception("Could not load main config file!")

        self.ultra_angle_calibration = data.get("ultra_angle_calibration", 0.15)

        self.touch_ports = data["touch_ports"]
        self.ultrasonic_port = data["ultrasonic_port"]
        self.motor_ports = data["motor_ports"]
        self.distance_offset = data["ultra_sound_offset"]
        self.distance_proportional_offset = data["ultra_sound_proportional_offset"]

        #Motor initialization
        # self.wheels IS JUST THE WHEEL MOTORS
        self.wheels = [self.motor_ports.get("left"), self.motor_ports.get("right")]
        # self.motors IS ALL OF THE MOTORS (BOTH WHEELS AND TOP MOTOR)
        self.motors = self.wheels + [self.motor_ports["top"]]
        self.motorParams = {}

        #Enabling the motors
        # wheels[0] and wheels[1] are the left and right wheels
        self.interface.motorEnable(self.wheels[0])
        self.interface.motorEnable(self.wheels[1])
        # motor_ports["top"] is the top motor
        self.interface.motorEnable(self.motor_ports["top"])

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

        self.interface.setMotorAngleControllerParameters(self.motor_ports["top"],self.motorParams["top"])

        #Initialize the touch sensors
        print("Ultrasound sensor at port: {0}\nTouch sensors at ports: {1}".format(self.ultrasonic_port,self.touch_ports))
        if self.touch_ports is not None:
            self.bumpers = data["bumpers"]
            for i in self.touch_ports:
                self.interface.sensorEnable(i,brickpi.SensorType.SENSOR_TOUCH)

        if self.ultrasonic_port is not None:
                self.interface.sensorEnable(self.ultrasonic_port, brickpi.SensorType.SENSOR_ULTRASONIC)

        # load proportional control param
        self.proportional_control = {}
        self.proportional_control["k_p"] = data["prop_ctl"]["k_p"]

                # load particle state
        self.particle_state = None
        self.standard_deviation = {}
        self.standard_deviation["x"] = data["standard_deviation"]["x"]
        self.standard_deviation["y"] = data["standard_deviation"]["y"]
        self.standard_deviation["theta_straight"] = data["standard_deviation"]["theta_straight"]
        self.standard_deviation["theta_rotate"] = data["standard_deviation"]["theta_rotate"]
        self.standard_deviation["theta_top_rotate"] = data["standard_deviation"]["theta_top_rotate"]
        self.standard_deviation["ultrasound"] = data["standard_deviation"]["ultrasound"]

    #Load the PID config file
    def load_pid_config(self):
        PID = None
        with open(self.pid_config_file) as PID_file:
            PID = json.load(PID_file)
        if PID is None:
            raise Exception("Could not load PID configuration file!")

        # Configure motor calibration constants
        self.distance_calibration = PID.get("distance_calibration", 3.05)
        self.angle_calibration = PID.get("angle_calibration", 0.13)

        #Configuring the left motor
        self.motorParams["left"] = self.interface.MotorAngleControllerParameters()
        self.motorParams["left"].maxRotationAcceleration = PID["left"]["maxRotationAcceleration"]
        self.motorParams["left"].maxRotationSpeed = PID["left"]["maxRotationSpeed"]
        self.motorParams["left"].feedForwardGain = PID["left"]["feedForwardGain"]
        self.motorParams["left"].minPWM = PID["left"]["minPWM"]
        self.motorParams["left"].pidParameters.minOutput = PID["left"]["minOutput"]
        self.motorParams["left"].pidParameters.maxOutput = PID["left"]["maxOutput"]
        self.motorParams["left"].pidParameters.k_p = PID["left"]["k_p"]
        self.motorParams["left"].pidParameters.k_i = PID["left"]["k_i"]
        self.motorParams["left"].pidParameters.k_d = PID["left"]["k_d"]

        #Configure the right motor
        self.motorParams["right"] = self.interface.MotorAngleControllerParameters()
        self.motorParams["right"].maxRotationAcceleration = PID["right"]["maxRotationAcceleration"]
        self.motorParams["right"].maxRotationSpeed = PID["right"]["maxRotationSpeed"]
        self.motorParams["right"].feedForwardGain = PID["right"]["feedForwardGain"]
        self.motorParams["right"].minPWM = PID["right"]["minPWM"]
        self.motorParams["right"].pidParameters.minOutput = PID["right"]["minOutput"]
        self.motorParams["right"].pidParameters.maxOutput = PID["right"]["maxOutput"]
        self.motorParams["right"].pidParameters.k_p = PID["right"]["k_p"]
        self.motorParams["right"].pidParameters.k_i = PID["right"]["k_i"]
        self.motorParams["right"].pidParameters.k_d = PID["right"]["k_d"]

        self.interface.setMotorAngleControllerParameters(self.wheels[0], self.motorParams["left"])
        self.interface.setMotorAngleControllerParameters(self.wheels[1], self.motorParams["right"])
        self.interface.setMotorRotationSpeedReferences(self.motors,[0,0,0])

    ## END OF INITIALIZATION FUNCTIONS

    ## PRIVATE FUNCTIONS
    ## SENSORS
    #Read input from the touch sensors
    def __update_touch_sensors(self):
        if self.touch_ports is not None:
            self.bumpers["left"]["value"] = self.interface.getSensorValue(self.bumpers["left"]["port"])[0]
            self.bumpers["right"]["value"] = self.interface.getSensorValue(self.bumpers["right"]["port"])[0]
            return True
        else:
            raise Exception("Touch sensors not initialized!")

    def __read_ultrasonic_sensor(self):
        if self.ultrasonic_port is not None:
            try:
                result = self.interface.getSensorValue(self.ultrasonic_port)
                return result[0]
            except IndexError:
                return 255
        else:
            raise Exception("Ultrasonic sensor not initialized!")

    # Update self.distance to self.__median_filtered_ultrasonic()
    def update_distance(self):
        for i in range(15):
                    raw_ultra_reading = self.__read_ultrasonic_sensor()
                    calibrated_ultra_reading = raw_ultra_reading + self.distance_offset + (raw_ultra_reading*self.distance_proportional_offset)
                    self.distance_stack.append(calibrated_ultra_reading)
        q_copy = self.distance_stack
        d = sorted(q_copy)[int((len(q_copy)-1)/2)]
        self.distance = d
        return d

    def detect_obstacles(self, maxdist=110):
        # Get sonar reading
        d = self.update_distance()
        # If reading within maxdist
        if d < maxdist:
            # Get robot position
            robot_x, robot_y, robot_p = self.particle_state.get_coordinates()
            ultra_rad = math.radians(robot_p)
            print("Robot at x:{}. y:{}, theta:{}, ultra_angle:{}".format(robot_x, robot_y, robot_p,0))
            # Create object in position calculated from robot's position
            obstacle_x = robot_x + d*math.cos(robot_p+ultra_rad)
            obstacle_y = robot_y + d*math.sin(robot_p+ultra_rad)
            err = self.particle_state.get_error()
            self.planner.append_obstacle(Obstacle(obstacle_x, obstacle_y, err[0], err[1]))
            print("Obstacle detected {0}cm away at angle of {1} from robot. Obstacle coordinates - x:{2}. y:{3}".format(d, 0, obstacle_x, obstacle_y))
        return True


    # Move specified wheel a certain distance
    def __move_wheels(self, distances=[1,1],wheels=None):
        if wheels is None:
            wheels = self.wheels
        #print("Distance to move wheels: {}".format(distances))

        # Retrieve start angle of motors
        motorAngles_start = self.interface.getMotorAngles(wheels)
        #print("Start Angles: {}".format(motorAngles_start))

        # Set the reference angles to reach
        circular_distances = [-round((2*x*self.distance_calibration)/self.circumference,2) for x in distances]
        #print("Distance in radians: {}".format(circular_distances))
        # Angles to end at
        motorAngles_end = []
        motorAngles_end.append(round(motorAngles_start[0][0] + circular_distances[0],2))
        motorAngles_end.append(round(motorAngles_start[1][0] + circular_distances[1],2))

        #print("Angles to end at: {}".format(motorAngles_end))
        self.interface.increaseMotorAngleReferences(wheels, circular_distances)

        # This function does PID control until angle references are reached
        while not self.interface.motorAngleReferencesReached(wheels):
            if (round(self.interface.getMotorAngles(wheels)[0][0],2)==motorAngles_end[0] or round(self.interface.getMotorAngles(wheels)[1][0],2)==motorAngles_end[1]):
                break
        return True

    #Takes the angle in degrees and rotates the robot right
    def rotate_right(self, angle, update_particles=False):
        #print("Starting pose: {}".format(self.state["pose"].get("theta")))
        dist = self.angle_calibration*angle
        self.state["pose"]["theta"] = self.state["pose"].get("theta", 0) + angle
        #print("New pose: {}".format(self.state["pose"].get("theta")))
        # Maybe only save state when the robot is shutting down?
        if update_particles:
            self.particle_state.update_state("rotation", angle, self.distance)
        return self.__move_wheels([dist,-dist])

    #Takes the angle in degrees and rotates the robot left
    def rotate_left(self, angle,update_particles=False):
        return self.rotate_right(-angle,update_particles=update_particles)

    # Rotate a motor by angle degrees (mainly for ultrasound motor)
    def __rotate_top_motor(self, angles=[0], motors=None):
        if motors is None:
            motors = [self.motor_ports["top"]]
        # print("Starting reference angles: {}".format(self.interface.getMotorAngles(motors)))
        self.interface.increaseMotorAngleReferences(motors, [x*self.ultra_angle_calibration for x in angles])
        # This function does PID control until angle references are reached
        while not self.interface.motorAngleReferencesReached(motors):
            pass
        # print("Ending reference angles: {}".format(self.interface.getMotorAngles(motors)))
        return True

    ### END OF PRIVATE FUNCTIONS
    ### PUBLIC FUNCTIONS
    def start_obstacle_detection(self, interval = 0.05):
        if self.ultrasonic_port is not None:
            detection_thread = Poller(t=interval,target=self.detect_obstacles)
            self.threads.append(detection_thread)
            detection_thread.start()
        else:
            raise Exception("Ultrasonic sensor not initialized!")
        return True

    ### PUBLIC FUNCTIONS
    def start_threading(self, touch=True, ultrasonic=False, interval = 0.05):
        # If threads already exist, stop them and delete them.
        if self.threads:
            for i in self.threads:
                i.stop()
            self.threads = []

        if touch:
            if self.touch_ports is not None:
                touch_thread = Poller(t=interval,target=self.__update_touch_sensors)
                self.threads.append(touch_thread)
                touch_thread.start()
            else:
                raise Exception("Touch sensors not initialized!")
        if ultrasonic:
            if self.ultrasonic_port is not None:
                distance_thread = Poller(t=interval,target=self.update_distance)
                self.threads.append(distance_thread)
                distance_thread.start()
            else:
                raise Exception("Ultrasonic sensor not initialized!")
        return True

    def get_state(self):
        return self.particle_state.get_state()

    def stop_threading(self):
        for i in self.threads:
            i.stop()
        return True

    def get_bumper(self, bumper):
        return self.bumpers[bumper]["value"]

    def get_distance(self):
        return self.distance

    def start_debugging(self):
        self.print_thread = Poller(t=5, target=self.print_state)
        self.print_thread.start()
        return True

    def stop_debugging(self):
        self.print_thread.stop()
        return True

    def print_state(self):
        print("---WALL-E STATE---")
        print("MOTORS")
        print("Angles: {}".format([x[0] for x in self.interface.getMotorAngles(self.motors)]))

        print("SENSORS")
        if self.touch_ports is not None:
            print("Bumpers: Left - {0}, Right - {1}".format(self.get_bumper("left"), self.get_bumper("right")))
        if self.ultrasonic_port is not None:
            print("Distance: {}".format(self.distance))

        print("POSITIONING")
        print("Robot theta: {}".format(self.state["pose"]["theta"]))
        print("Robot x,y: {0},{1}".format(self.state["pose"]["x"],self.state["pose"]["y"]))
        print("Camera pose: {}".format(self.state["ultra_pose"]))
        current_x, current_y, current_theta = self.particle_state.get_coordinates()

        print("Particle state: x: {0}, y: {1}, theta: {2}".format(current_x, current_y, current_theta))

    # Set ultra_pose variable to pose without moving the motor.
    def calibrate_ultra_position(self, pose = 0):
        self.state["ultra_pose"] = pose
        return True
    def save_state(self, state_file="robot_state.json"):
        with open("robot_state.json","w") as f:
            json.dump(self.state, f)
    def reset_state(self):
        self.state = {'pose':{'x':0, 'y': 0, 'theta': 0}, 'ultra_pose': 0}
        self.particle_state.reset()
        return True

    def step_to_waypoint(self,X,Y,maxdistance=20):
        success = False
        while not success:
            success = self.navigate_to_waypoint(X,Y,maxdistance)
            if self.canvas:
                particles = self.particle_state.get_state()
                self.canvas.drawParticles(particles)
        return success

    def navigate_to_waypoint(self,X,Y, maxdistance = None):
        success = True
        current_x, current_y, current_theta = self.particle_state.get_coordinates()
        diff_X = X-current_x
        diff_Y = Y-current_y
        if abs(diff_X)<0.5:
            diff_X = 0
        if abs(diff_Y)<0.5:
            diff_Y = 0
        distance = math.sqrt(math.pow(diff_X,2)+math.pow(diff_Y,2))
        angle = math.degrees(math.atan2(diff_Y, diff_X))
        if maxdistance:
            if distance > maxdistance:
                distance = maxdistance
                success = False
        print("\nNavigating to point ({0},{1}) from point ({2},{3},{4})".format(X, Y,current_x,current_y,current_theta))
        print("diff x: {0}, diff y: {1} arctan2 result: {2}".format(diff_X, diff_Y, angle))
        self.set_robot_pose(angle, update_particles=True)
        print "Rotation Finished"
        self.travel_straight(distance, update_particles=True)

        # Check if S.D of particles is very large (they should be updated again)
        current_err = self.particle_state.get_error()
        print "Current Error - X:{0}, Y:{1}, Theta: {2}".format(current_err[0], current_err[1], current_err[2])
        if ((current_err[0] > self.max_sd_error) or (current_err[1] > self.max_sd_error)):
            d = self.update_distance()
            time.sleep(0.1)
            self.particle_state.update_state(action="refinement",movement=None,ultrasound={'0':d})
            self.set_ultra_pose(90)
            d = self.update_distance()
            time.sleep(0.1)
            self.particle_state.update_state(action="refinement",movement=None,ultrasound={'90':d})
            self.set_ultra_pose(-90)
            d = self.update_distance()
            time.sleep(0.1)
            self.particle_state.update_state(action="refinement",movement=None,ultrasound={'-90':d})
            self.set_ultra_pose(0)
        return success

    #Sets a constant speed for specified motors
    def set_speed(self, speeds=[2,2], wheels=None, k = 1):
        if wheels is None:
            wheels = self.wheels
        for index,i in enumerate(speeds):
            if abs(i)>10:
                raise Exception("Speed set too high, abort.")
            speeds[index]=-i
        speeds = [k*x for x in speeds]
        self.interface.setMotorRotationSpeedReferences(wheels,speeds)
        self.motor_speeds = speeds
        return True

    #Does the immediate stop if it runs into an obstacle
    def stop(self):
        self.interface.setMotorPwm(self.wheels[0],0)
        self.interface.setMotorPwm(self.wheels[1],0)
        return True

    #Takes the distance in centimeters and moves it forward
    def travel_straight(self, distance, update_particles=False):
        success = self.__move_wheels(distances=[distance,distance])
        if update_particles:
            self.particle_state.update_state("straight", distance, ultrasound = {'0':self.update_distance()})
        return success

    # Move the top camera to specified pose
    def set_ultra_pose(self, pose):
        success = True
        # print("Current ultra pose: {}".format(self.state.get("ultra_pose", -1)))
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
            success = self.__rotate_top_motor([rotation])
            if success:
                self.state["ultra_pose"] = pose
        else:
            print("No rotation required.")
        return success

    # Move the robot to the specified pose
    def set_robot_pose(self, s_pose, update_particles = False):
        success = True
        print("Starting pose: {}".format(self.state["pose"].get("theta",-1)))

        while s_pose >= 360:
            s_pose-=360
        while s_pose <= -360:
            s_pose+=360

        rotation = (s_pose-self.state["pose"].get("theta", 0))

        if rotation==0:
            print("No rotation required.")
            return True

        if rotation > 180:
            rotation-=360
        elif rotation < -180:
            rotation +=360
        success = self.rotate_left(rotation)
        self.state["pose"]["theta"] = s_pose
        print("Rotation required: {}".format(rotation))
        print("Ending pose: {}".format(s_pose))

        if update_particles:
            self.particle_state.update_state("rotation", rotation, {'0':self.distance})
        return success

    # Interactive mode for the robot to control without writing a program each time
    def interactive_mode(self):
        command = 0
        while command!=-1:
            print("Available commands:\n-1: End session.\n1: Travel straight.\n2: Set pose.\n3: Move wheels.\n4: Set ultra pose.\n5: Recalibrate ultra pose.\n6: Reload config files.\n7: Print sensor values.\n8: Navigate to (X,Y)(cm)\n9: Rotate right\n10: Save state\n11: Reset state\n12: Print state\n13: Start threading\n14: Stop threading")
            command = int(input())
            if command == 1:
                print("Enter distance to move straight: ")
                distance = float(input())
                self.travel_straight(distance)
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
                self.__move_wheels(distances)
            elif command == 4:
                print("Enter desired camera pose:")
                s_pose = float(input())
                self.set_ultra_pose(s_pose)
            elif command == 5:
                print("Enter recalibration value for ultrasound pose: ")
                s_pose = float(input())
                self.calibrate_ultra_position(s_pose)
            elif command == 6:
                print("Reloading config files")
                self.load_pid_config()
                self.load_base_config()
            elif command == 7:
                print("Left bumper: {0}, Right bumper: {1}, Ultrasound: {2}".format(self.get_bumper("left"),self.get_bumper("right"), self.distance))
            elif command == 8:
                x = float(input("Enter X value:"))
                y = float(input("Enter Y value:"))
                self.navigate_to_waypoint(x,y)
            elif command==9:
                angle = float(input("Enter angle:"))
                self.rotate_right(angle)
            elif command==10:
                self.save_state()
            elif command==11:
                self.reset_state()
            elif command==12:
                self.print_state()
            elif command==13:
                self.start_threading()
            elif command==14:
                self.stop_threading()
            elif command==15:
                print(self.update_distance())
            else:
                command = -1
                self.stop_threading()
                self.stop()

        return True


    def approach_object(self, d=30, s_pose=0):
        """ approach_object
        Takes a distance and the direction in terms of s_pose for the camera to look in
        Output: Approaches the object smoothly and stops at a distance of d
        """
        self.set_ultra_pose(s_pose)
        distance_to_travel = self.get_distance()-d-1
        print "Distance: " + str(self.get_distance())
        while (distance_to_travel != 0):
            motor_speed = int(round(distance_to_travel*0.4))
            if(motor_speed > 8):
                motor_speed = 8
            elif(motor_speed < -8):
                motor_speed = -8
            self.set_speed([motor_speed,motor_speed])
            distance_to_travel = self.get_distance()-d-1
        self.set_speed([0,0])

    def keep_distance(self, distance_to_keep, average_speed, wall_location):
        """ using ultrasonic sensor to keep a contant distance between the object and the robot
        args:
            distance_to_keep: int
            average_speed    : int
            wall_location    : int, 1 for Left side, 2 for Right side
        """
        # proportional control
        speed_compensation = - self.proportional_control["k_p"] * (distance_to_keep - self.distance)
        if (wall_location == 1):
            pass
        elif (wall_location == 2):
            speed_compensation = -speed_compensation
        else:
            raise Exception("Not a valid wall location!")
        # calculate motor speeds
        leftMotor_speed = average_speed - speed_compensation
        rightMotor_speed = average_speed + speed_compensation
        # limit motor speeds
        if(abs(leftMotor_speed) > 10):
            leftMotor_speed = leftMotor_speed/abs(leftMotor_speed) * 9
            rightMotor_speed = 2 * average_speed - leftMotor_speed
        if(abs(rightMotor_speed) > 10):
            rightMotor_speed = rightMotor_speed/abs(rightMotor_speed) * 9
            leftMotor_speed = 2 * average_speed - rightMotor_speed
        # print info
        print("speed compensation: {}".format(speed_compensation))
        print("\tcurrent distance: {}".format(self.distance))
        print("\tmotor speed set to: {}, {}".format(leftMotor_speed, rightMotor_speed))
        try:
            self.set_speed([leftMotor_speed, rightMotor_speed], self.wheels)
        except Exception, e:
            print("There is some problem setting motor speed, {}".format(str(e)))

    def challenge(self):
        if not self.planner:
            raise Exception("Planner has not been initialized!")
        x,y,theta = self.particle_state.get_coordinates()
        print("Robot state: x:{},y:{},theta:{}".format(x,y,theta))
        print(self.motor_speeds[0])
        v_l, v_r, x_new, y_new, theta_new = self.planner.get_plan(x/100,y/100,theta,self.motor_speeds[0],self.motor_speeds[1],0.1)
        print("Robot new state: x:{},y:{},theta:{}".format(x_new,y_new,theta_new))
        diff_x = math.pow(x-x_new,2)
        diff_y = math.pow(y-y_new,2)
        d = math.sqrt(diff_x+diff_y)
        print("Distance travelled:{}".format(d))
        print("New plan calculated: vL:{},vR:{}".format(v_l,v_r))
        self.set_speed([v_l*10, v_r*10], self.wheels)
        self.particle_state.update_state("mixed", movement=d,theta=theta-theta_new)
        particles = self.particle_state.get_state()
        self.canvas.drawParticles(particles, offset_y=110)
        return True

    def start_challenge(self, interval = 0.05):
        challenge_thread = Poller(t=interval,target=self.challenge)
        self.threads.append(challenge_thread)
        challenge_thread.start()
        return True

    def check_finished(self, finishLine=320):
        """ Check if we are at the end
            If self.state["x"] > 320, return True
            else return False
        """
        if self.state["x"] > finishLine:
            return True
        else:
            return False
