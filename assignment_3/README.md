# Robotics-EIE3
## Robotics Assignment 3:

### 2.1 Representing and Displaying Uncertain Miton with a Particle Set.

### 2.2 Waypoint Navigation

### 2.3 Sonar investigation

#### _1:_ When placed facing and perpendicular to a smooth surface such as a wall, what are the minimum
and maximum depths that the sensor can reliably measure?

The sensor is a digital sensor. It returns the distance to an object (or objects as we will discuss later) as a byte value expressed in cm. The value of 255 has a special meaning, it indicates there is no object within measuring range. The resolution of the sensor is 1 cm.
So In theory its minimum range is 0 cm and its maximum range is 254 cm. In reality the minimum range is about 7 cm. The maximum range depends on the object to be detected, large and hard objects can be detected over a longer range than small and soft objects. False echoes can limit the practical range even further. A wall (large and hard) can often be detected if it is within a range of two meters, but for a reliable signal it has to be within a range of 160 centimeter.

#### _2:_ Move the sonar so that it faces the wall at a non-orthogonal incidence angle. What is the maximum
angular deviation from perpendicular to the wall at which it will still give sensible readings?

As a rule of thumb one can assume that objects that are within an angle 15 degrees to the left or right are detected. The total width of the detection area is about 30 degrees. When using the sensor for obstacle avoidance this wide beam is an advantage. When the sensor is used for mapping it is a drawback as objects seem wider then they really are. They also seem arc shaped, the arc having an angle of at least 30 degrees for small objects and more for wider objects.

#### _3:_ Do your sonar depth measurements have any systematic (non-zero mean) errors? To test this, set
up the sensor at a range of hand-measured depths (20cm, 40cm, 60cm, 80cm, 100cm) from a wall
and record depth readings. Are they consistently above or below what they should be?

We have observed a deviation of approximately +/- 3cm which agrees with the datasheet of the US sensor.

#### _4:_ What is the the accuracy of the sonar sensor and does it depend on depth? At each of two chosen
hand-measured depths (40cm and 100cm), make 10 separate depth measurements (each time
picking up and replacing the sensor) and record the values. Do you observe the same level of
scatter in each case?

#### _5:_ In a range of general conditions for robot navigation, what fraction of the time do you think your
sonar gives garbage readings very far from ground truth?
