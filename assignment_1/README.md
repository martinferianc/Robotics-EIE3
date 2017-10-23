# Robotics-EIE3
## Robotics Assignment 1:

### 5.2 Controller Tuning.

We used the script provided which gave us the difference between the projected motor position and actual motor position to give us an idea of how far off the motor was when rotating through a set number of degrees. For the purposes of the calibration, we ran a test of 60, 180 and 360 degrees. Once we had run these tests, we then uploaded the files to Github where we could then run them through MATLAB to generate the graphs.

In order to tune the controller, we first attempted to implement **Ziegler-Nichols** to provide a starting point for the tuning of the controller. 

> Set k<sub>i</sub> and k<sub>d</sub> to zero. Increase k<sub>p</sub> until the system starts oscillating with period P<sub>u</sub> (in seconds) — remember this gain as k<sub>u</sub>

> Set k<sub>p</sub> = 0.6 * k<sub>u</sub> , k<sub>i</sub> = 2k<sub>p</sub> / P<sub>u</sub> , and k<sub>d</sub> = k<sub>p</sub> * P<sub>u</sub> / 8


We increased the value of k<sub>u</sub> until we noticed that oscillations began to occur. We found this value to be around 750. That gave us values of 450 for k<sub>p</sub>. From the oscillation period we calculated k<sub>d</sub> to be 229 and k<sub>i</sub> to be 120.

When we then looked at the oscillations for these new values we found them to be better than the original but still requiring further refinement. We did this manually by changing the value of k<sub>i</sub> until we felt it best matched the actual curve. 

Measurement 1: k<sub>p</sub> = 750, k<sub>d</sub> = 0, k<sub>i</sub> = 0
![Measurement 1 60 degrees](./graphs/motor_position_1_60.0.log.png)
![Measurement 1 180 degrees](./graphs/motor_position_1_180.0.log.png)
![Measurement 1 360 degrees](./graphs/motor_position_1_360.0.log.png)

Measurement 2: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 0
![Measurement 2 60 degrees](./graphs/motor_position_2_60.0.log.png)
![Measurement 2 180 degrees](./graphs/motor_position_2_180.0.log.png)

Measurement 3: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 120
![Measurement 3 60 degrees](./graphs/motor_position_3_60.0.log.png)
![Measurement 3 180 degrees](./graphs/motor_position_3_180.0.log.png)
![Measurement 3 360 degrees](./graphs/motor_position_3_360.0.log.png)

Measurement 4: Left: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 170 Right: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 200
![Measurement 4 60 degrees](./graphs/motor_position_4_60.0.log.png)
![Measurement 4 180 degrees](./graphs/motor_position_4_180.0.log.png)
![Measurement 4 360 degrees](./graphs/motor_position_4_360.0.log.png)

Measurement 5: Left: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 200 Right: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 225
![Measurement 5 60 degrees](./graphs/motor_position_5_60.0.log.png)
![Measurement 5 180 degrees](./graphs/motor_position_5_180.0.log.png)
![Measurement 5 360 degrees](./graphs/motor_position_5_360.0.log.png)

Measurement 6: Left: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 160 Right: k<sub>p</sub> = 450, k<sub>d</sub> = 229 k<sub>i</sub> = 190
![Measurement 6 60 degrees](./graphs/motor_position_6_60.0.log.png)
![Measurement 6 180 degrees](./graphs/motor_position_6_180.0.log.png)
![Measurement 6 360 degrees](./graphs/motor_position_6_360.0.log.png)

As you can see from the images, we managed to from large oscillations to an almost perfect curve.

### 5.3 Distance and Rotation Calibration


### 5.4 Driving in a Square and Observing Drift

#### Driving recording
[https://youtu.be/rqx-CMewXA0](https://youtu.be/rqx-CMewXA0)

#### Drift logs
|  X-axis | Y-axis  |
|:-------:|:-------:|
|4.5      |-2.5     |
|-6.5     |1        |
|-6.25    |0.5      |
|-3.5     |-5.5     |
|-3.5     |-4       |
|-5.5     |0.25     |
|-8       |-0.75    |
|-4.5     |-0.5     |
|-1       |0        |
|-5.5     |0        |


### 5.5 Covariance Matrix
Once we were happy with how our robot was performing we then went about calculating the covariance matrix based upon the finishing position of the robot over 10 squares. We obtained the following matrix:

|    |X           |Y           |
|----|------------|------------|
|X   |12.64513889 |-2.94027778 |
|Y   |-2.94027778 |4.6         |

A Covariance Matrix allows us to clearly see the variance we observed in our x and y values along with the covariance between x and y. In our case the variance of `X` is `12.64`, the variance of `Y` is `4.6` and the covariance is `-2.94`.

### Further Thought:

#### What causes a larger effect in your robot, Uncertainty in drive distance or rotation angle?

For our robot, we believe that the rotation angle introduces a larger uncertainty as on measuring the lines we found them to all be roughly the same length. However if the robot was a few degrees off in it’s turn these quickly added up to create very distorted squares.

#### Can you think of any robot designs which will be able to move more precisely?

A number of things could be done to improve precision. Firstly we could reverse the robot so that the driving wheels were “dragging” the caster rather than “pushing” it. This would reduce the amount of drift in a line as the caster would help to keep the wheels in check when it is being dragged whilst it has the temptation to wander whilst being pushed. This is similar to how your car is much more stable driving forward but has a larger turning circle than when you try and reverse it.

We could also reduce the size of the wheels. By fitting smaller wheels we would slow the motion of the robot as it would travel a shorter distance in one revolution, but we would be able to make it travel a far more precise distance.

#### How should we go about equipping a robot to recover from the motion drift we have observed in this experiment?

Firstly we could ensure that the paper is as flat as possible so this had no effect on the steering of the robot. (Though this would be very difficult to achieve in a real world scenario). Another option would be to fit a sensor which would be able to lock onto a fixed point in the room and measure the distance to that item. By comparing this distance as the robot moves with the value the robot would expect it to be, the robot can observe if it is drifting off-course. This would require a complex sensor however such as a LIDAR with intensive behind-the-scenes processing to enable this feature. 

