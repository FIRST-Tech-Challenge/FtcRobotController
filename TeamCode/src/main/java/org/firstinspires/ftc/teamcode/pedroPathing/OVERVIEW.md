## Basic Ideas
Pedro Pathing is a reactive vector based follower. What this means is that the robot dynamically
calculates a set of vectors that are required to correct error as well as to move forward and applies them.

The robot calculates:

* centripetal force correction
* translational correction
* heading correction
* drive vector

These are then applied to the robot in this order until either the robot's power is maxed out or all
the vectors are applied.

## Why Pedro Pathing?
Why use Pedro Pathing? Why not something else like Road Runner or Pure Pursuit?

* Why not Pure Pursuit?
  * Pure Pursuit searches for the farthest point on the path that's within a certain radius from the robot. Pure Pursuit will then go in a straight line to that point. This poses several problems, as a small search radius will cause some oscillations on corners, and a large search radius will cut corners on paths, which makes the paths inaccurate to real life.
  * Pedro Pathing instead corrects to the closest point on the path while still following the path. This ensures that the follower will stay on the path while still being able to move forward along the path without cutting corners or encountering oscillation issues.
* Why not Road Runner?
  * Road Runner is a motion profile based follower, which means that a set of instructions for motor powers are calculated for each path beforehand and then run. After reaching the end of this motion profile, Road Runner corrects. This can be sufficient for most situations, but if the robot encounters an obstacle or wheel slippage, it may be unable to correct in time.
  * Pedro Pathing instead dynamically corrects throughout the path. The movement vectors are calculated at every point along the path, and because of this, the path can even be changed midway through and Pedro Pathing will still be able to correct. Since correction occurs throughout the path, the error correction isn't concentrated on the end of the path and therefore the robot is able to better minimize error.

## How Does Pedro Path?
As mentioned in the *Basic Ideas* section, Pedro Pathing calculates a set of vectors to move the
robot along a path, which is defined with Bezier curves. Here, we'll go more in-depth on how these
vectors are calculated and used in path following.

### The Hierarchy
While following paths, sometimes all these motion vectors are demanding more power from the robot
than it actually has. How do we deal with this?

Our motion vectors are applied in order of importance, which is the order they appear in within the
list in *Basic Ideas*. The centripetal force vector is the highest importance, since it ensures the
robot sticks to the path. If the robot is far off the path, then the robot will not drive along the
path, and so the centripetal force vector will be reduced in magnitude. This is why ranking the
centripetal force correction above the translational correction doesn't produce issues. The next
highest, of course, is the translational correction. This corrects strictly the robot's position to
the closest point on the path. The reasoning behind this is that it is usually much more important
that the robot be on the path, and so avoid potential obstacles, rather than facing the correct
direction. The third highest important vector is the heading correction, which turns the robot to
the correct angle. This is higher than the drive vector since we don't want to drive forward if the
robot isn't facing the correct direction. Finally, the drive vector is applied. This ensures that
the robot only continues on the path when there aren't any major issues with the following.

As each vector is applied, the robot checks to see if the sum of the applied vectors is greater than
the power that the drivetrain can produce, which would be 1 motor power. If the magnitude of the
combined vectors is greater than 1, then the most recently added vector is scaled down until the
combined vectors' magnitude is equal to 1. If all vectors are able to be applied without exceeding
the power limit, then all the vectors can just be applied without issue.

### Centripetal Force Correction
Have you ever noticed that your robot seems to want to swing outwards when taking corners? This is
due to a lack of centripetal force correction. In order to take curves effectively, your robot must
accelerate towards the inside of the curve. If we can approximate the region of the path the robot
is at with a circle, then we can use the formula for centripetal force to calculate how much power
we need to allocate to approximate a centripetal force. 

Because paths are defined with Bezier curves, we can easily take the first and second derivative of
the path, expressed as vectors. We can use that to calculate the curvature of the path, which is the
inverse of the length of the radius of the circle we can use to approximate the path. The actual
formula for the calculations of the curvature is the cross product of the first derivative and
second derivative, divided by the magnitude of the first derivative raised to the power of 3.

With this, along with the weight of the robot and the velocity of the robot along the path, we can
calculate the force necessary to keep the robot on the path, and then tune a scaling factor to turn
that force into a corresponding power for the robot.

### Translational Correction
This is as simple as it sounds: this corrects error in the robot's position only. The robot's translational
error is corrected with a PID control. The translational correction does not act along the path the
robot takes, but instead moves the robot back to the closest point on the path.

### Heading Correction
The heading correction operates very similarly to the translational correction, except this corrects
the direction the robot is facing. The heading correction will turn in the closest direction from the
robot's current heading to the target heading.

### Drive Vector
The drive vector points in the direction of the tangent of the path and it is responsible for moving
the robot along the path. Using basic kinematics equations, we can use the velocity of the robot
along the path, the length of path left, and a specified target rate of deceleration to calculate
the velocity we should be moving at. Additionally, after finding out the rate of deceleration of the
robot under 0 power, we can compensate for that with another kinematics equation. Combining these
two lets us control our velocity to both move along the path quickly and brake without overshooting.

## Additional Capabilities
In addition to following paths, Pedro Pathing can be used for a few other purposes.

### Holding Points
Pedro Pathing is also capable of holding a specified position and direction. This can be useful for
improving on the end accuracy of paths, providing additional correction time if possible. It can
also be useful in cases where contact with other robots might occur. For instance, in the 2022-2023
FTC season challenge, Power Play, robots might come into contact when scoring on a contested middle
junction. Pedro Pathing would be able to recover and correct from a robot collision, allowing for
more consistent scoring.

### TeleOp Enhancements
Finally, Pedro Pathing can be used in TeleOp to enhance driving. With regular mecanum drive, robots
will tend to swing out when taking corners. Pedro Pathing can account for that, allowing the robot
to take corners more smoothly and efficiently. Using the same localizer as is used in autonomous, a
first and second derivative can be estimated from previous positions. Then, with a modified version
of the curvature formula, we can estimate a centripetal force correction and apply it under driver
control.

## Questions?
If you still have more questions, feel free to contact us at `scottsbots10158@gmail.com`