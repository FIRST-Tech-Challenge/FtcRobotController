## idk what to put here

### This is a WIP

#### My notes on MeepMeep (and Roadrunner in general) and its behavior. Note that these notes do not detail how Roadrunner generates the paths, but rather how certain parameters affect the shape of Roadrunner splines.

Misc:
 - `PathContinuityException` will be thrown when chaining multiple `splineToLinearHeading()` together. Instead, `splineToSplineHeading()` should be used.

endTangent Behavior:
 - endTangent behavior defines the angle of the line (in relation to the x-axis)
 - With a given endTangent angle, the robot will also generate a quintic spline that approaches the endpoint in the direction of the specified angle
   (e.g. with endTangent of 90 degrees, it will approach "from below" and go in the direction that 90 degrees points in the MeepMeep coordinate plane (towards the positive end of the y axis))

TrajectorySequence `PathContinuityException`:
 - TrajectorySequences will create new path segments if a PathContinuityException is encountered. However,
   this will still cause the robot to decelerate in order to complete a path segment, which slows down the robot.
 - For example, chaining multiple `splineToLinearHeading()` functions together will result in a `PathContinuityException`, so `splineToSplineHeading()` should be used instead in such a scenario.