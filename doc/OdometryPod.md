# The Odometry Pods

## Wheel Distance 

Calculating the distance each pod travels per encoder tick:

``` Java
private final double podWheelRadius = 1.0;
private final double ticksPerRotation = 8192;
private final double inchesPerRotation = 2 * Math.PI * podWheelRadius;
private final double inchesPerTick = inchesPerRotation / ticksPerRotation;
```
## Robot Localization From Encoders

With three encoder values, arranged as shown in Figure 1 below, it should be 
possible to estimate changes in robot position and angle based on wheel 
rotation amounts. 

![Wheel Diagram](WheelDiagram.drawio.svg)

**Figure 1:** Layout of odometry pod wheels on the robot

We define the state of the robot as follows: 

$$ \vec{x} = \left [ \begin{array}{cc} r_x & r_y & \theta \end{array} \right ] $$

Where $$ r_x $$ is the x field position, $$ r_y $$ is the y field position, and 
$$ \theta $$ is the angle with respect to the field. These exact values cannot be
measured directly from the odometry pods, but their relative changes can be 
estimated. To make progress on this, we consider angles and distance traveled 
separately. 

## Robot Rotation

$$ d\theta/{dt} = \frac{d(lp - rp) / r_{lr} }{dt} $$

where $$ lp $$ and $$ rp $$ are the left pod distance traveled and the right pod 
distance traveled, respectively and $$ r_{lr} $$ is the distance between the left 
and right pod, respectively. 

## Estimating Distance Traveled 

Since the pods are all at a distance r from the center, the relation between
distance traveled and the rotation of the robot are coupled. The distance traveled
in the y (forward) direction is the measured forward motion on a pod (left or right) 
minus the distance it would be expected to travel based on the robot's rotation:

$$ \frac{dy}{dt} = lp - r_l \frac{d\theta}{dt} $$
