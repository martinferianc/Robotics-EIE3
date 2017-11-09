# Robotics-EIE3
## Robotics Assignment 2:
[The booklet](../Resources/questions3.pdf)

### 1 Introduction
In this practical we will use some of the outward-looking sensors which come with the Lego Mind-
storms NXT kits, in particular the simple touch sensor and the more sophisticated sonar (ultrasonic)
sensor, and implement servoing behaviours.

### 2 Objectives

#### 2.1 Simple On/Off Forward/Backward Control with Touch Sensors
The touch sensor is a simple binary switch whose state is either 0 or 1 depending on whether it is
pressed. Mount two touch sensors onto the front of your robot with a simple bumper mechanism which
might be similar to the one in the picture above. Adding the long blocks to the front of the touch sensors
creates a bumper which should effectively be able to detect when the robot collides with an obstacle
when driving forwards, causing either or both of the touch sensors to be activated.

> Write a program which will drive the robot forward until it hits an obstacle and the bumper detects a crash. It should react in a sensible way and continue its journey.

* If the bump is on the left side, it reverse a short distance and turn to the right.
* If the bump is on the right, it reverses and turn to the left.
* If the bump is straight ahead and triggers both touch sensors the turn can be either left or right.

[bump_demo.py](./bump_demo.py) will operate the robot.

Here is a [demo video](https://youtube.com)


#### 2.2 Forward/Backward Proportional Servoing with the Sonar Sensor
The Lego NXT sonar sensor is able to measure the distance in centimetres to an object or obstacle
placed in front of it by timing the interval taken by an ultrasonic pulse to bounce off the object and
return.

##### Getting distance values
The sonar sensor will occasionally report a reading which is substantially wrong, usually when
it is not able to get a good strong sonar reflection from the surface it is looking at.

We used the median filtering of size `15` to get smooth distance reading from the sonar sensor.

##### Approach Object Demo
Mount the sonar sensor on the front of the robot facing straight forwards and **parallel to the ground**.
We used the ultrasonic sensor to approach the wall and keep a distance of 30cm.

**Proportional Control** with a single proportional gain value `k_p=0.4` was used to adjust the approach speed.
The equation for motor speed control looks like `motor_speed = k_p * (current_distance - distance_to_keep)`

The operational code is [approach_object_demo.py](./approach_object_demo.py)

[Demo video](https://youtube.com) was recorded.

#### 2.3 Wall Following
The robot can rotate the sonar sensor pointing perpendicular to the left or right side. We used **Proportional Control** to make it follow a wall at a fixed distance of 30cm.
When the robot is closer to the wall, it will steer away from the wall. Vice versa.
The robot should be able to cope with walls which are somewhat curved as well as straight ones; though
probably not sharp turns such as right angles.

The equations for left and right motor speed control look like _(wall on the left side of the robot)_:

`speed_compensation = k_p * (distance_to_keep - current_distance)`

`leftMotor_speed = average_speed - speed_compensation`

`rightMotor_speed = average_speed + speed_compensation`

Although the left and right motor have different individual speed, the `average_speed` is set to `6` we are always 
increasing the left wheel speed by the same amount that we decrease the right wheel speed and vice versa.
The gain value `k_p` is chosen to be `0.07` as that the robot will react to the distance change quick enough not to crash into the wall, and it will not over-react.

Code can be found here [wall_follow_demo.py](./wall_follow_demo.py)

Demo video via the [link](https://www.youtube.com/watch?v=33kcq_z78lE&feature=youtu.be)