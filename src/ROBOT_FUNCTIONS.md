# Functions Implemented in the Robot Class

## Setup

### load_base_config()
**Arguments:** None

**Returns:** None

**Action:** Called when the robot is first powered on. Sets all the motor parameters to the values defined in the relevant json file. Also sets up all of the relevant sensors.

### load_pid_config()
**Arguments:** None

**Returns:** None

**Action:** Called when the robot is first powered on. Sets all the motor parameters to the values defined in the PID json file.


## Sensors

### calibrate_ultra_position(pose)
**Arguments:** pose

**Returns:** None

**Action:** Sets the current ultrasonic sensor position state to the value passed in and saves this state.

### read_touch_sensor(port) -> result
**Arguments:** port - The port on the BrickPi the sensor is attached to (int).

**Returns:** The value of the sensor (bool).

**Action:** If a sensor is attached to the reference port, it gets the current value of the sensor and returns it. As it is a bump sensor this will be a 1 or 0.

### read_ultrasonic_sensor() -> result
**Arguments:** None

**Returns:** The value of the ultrasonic sensor (int).

**Action:** If a sensor is attached to the reference port, it gets the current value of the sensor and returns it. As it is an ultrasonic sensor this will be an integer.

### median_filtered_ultrasonic(size) -> result
**Arguments:** Size of the filter to be used (int).

**Returns:** The median value of the ultrasonic sensor (int).

**Action:** Further from the function read_ultrasonic_sensor, this function filters the value by collecting a determined number of samples (size) and then taking the median value from the sample. Should help with noise filtering.

### save_state(state_file)
**Arguments:** Name of the file to be used (string).

**Returns:** None

**Action:** Saves the current robot state to a json file, determined by the string passed to the function.

### set_ultra_pose(pose)
**Arguments:** Position to move the ultrasonic sensor to (int).

**Returns:** None

**Action:** Rotates the ultrasonic sensor to the specified position in degrees.


## Movement

### calibrate(radians, angle)
Depricated.

### move_wheels(distances, wheels)
**Arguments:** Distance to move each wheel by (int array).
           Wheels to be moved (int array).

**Returns:** None

**Action:**

### rotate_motor(angles, motors)
**Arguments:** Angle to move each wheel by (int).
           Motor to be moved (int).

**Returns:** None

**Action:**  

### rotate_right(angle)
**Arguments:** Angle to rotate the robot to the right by (int).

**Returns:** None

**Action:** Rotates the robot right the number of degrees specified by the argument passed.

### rotate_left(angle)
**Arguments:** Angle to rotate the robot to the left by (int).

**Returns:** None

**Action:** Rotates the robot left the number of degrees specified by the argument passed.

### set_speed(speeds, motors)

### stop()
**Arguments:** None

**Returns:** None

**Action:** Stops the robot motors.

### travel_straight(distance)
**Arguments:** Distance for the robot to move (int).

**Returns:** None

**Action:** Moves the robot forward a set distance in centimeters.

### set_robot_pose(s_pose)
**Arguments:** Pose to move the robot to (int).

**Returns:** None

**Action:** Moves the robot to a certain pose. Differs from rotate as robot can be told to rotate to a certain number of degrees from it's starting post (i.e. return to the direction it began facing) rather than blindly turning.

### interactive_mode()
**Arguments:** None

**Returns:** None

**Action:** Allows the user to issue commands to the robot on the fly.
Available commands:\n-1: End session.\n1: Travel straight.\n2: Set pose.\n3: Move wheels.\n4: Set ultra pose.\n5: Recalibrate ultra pose.\n6: Reload config files.

### approach_object(d, s_post)
**Arguments:** Distance for the robot to be from the object (int).
               Coordinate sensor should be turned to so it's facing the object (int).

**Returns:** None

**Action:** Allows the robot to approach an object at a known angle. Robot will approach the object smoothly and stop when it reaches the set distance.
