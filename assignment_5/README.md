# Robotics-EIE3
## Robotics Assignment 5: Continuous localisation
[**The handout**](../Resources/questions6.pdf)

An alternative relocalisation technique to monte carlo localisation involves making a lot of measurements at a number of chosen locations and learning their characteristics.

• This can be done without a prior map but needs training
• The robot can only recognise the locations it has learned

First the robot must be placed in each target location to learn its
appearance

The raw measurements are stored to describe the location: a place descriptor, or signature. Hence after each rotation of e.g.: an ultra sound sensor it is possible to take it's measurement and record it, later it is possible to compare it with another measurement to determine the current position. To save computation power it is possible to record just the raw distances in a histogram but a disadvantage it the lack of identification of the curent pose.
