## Prerequisites
Obviously, you have to have a robot to use Pedro Pathing. Also, Pedro Pathing is only able to work
with omnidirectional drives, like mecanum drive. There is currently no support for swerve drives.
You must also have a localizer of some sort. Pedro Pathing has a drive encoder, a two tracking wheel,
and a three tracking wheel localizer. You will need to have your localizer tuned before starting to
tune PedroPathing. Check out the tuning guide under the localization tab if you're planning on using one of the
localizers available in Pedro Pathing. Additionally, using [FTC Dashboard](http://192.168.43.1:8080/dash)
will help a lot in tuning. Team 16166 Watt'S Up made a path visualizer linked [here](https://pedro-path-generator.vercel.app).
The old Desmos visualizer is [here](https://www.desmos.com/calculator/3so1zx0hcd), but the one by
Watt'S Up is honestly a lot better.
One last thing to note is that Pedro Pathing operates in inches and radians. You can use centimeters
instead of inches, but you'll have to input all your measurement in centimeters, and any distances
that the tuners require you to push the robot or the tuners output will say "inches" when the actual
measurements will be in centimeters.

## Tuning
* First, make sure that your motor names and directions, located at the top of `FollowerConstants`,
  are correct.

* After that, we need the mass of the robot in kg. This is used for the centripetal force correction,
  and the mass, with the variable name `mass`, should be put on line `92` in the `FollowerConstants`
  class under the `tuning` package.

* Next, we need to find the preferred mecanum drive vectors. The rollers on mecanum wheels point at a
  45 degree angle from the forward direction, but the actual direction the force is output is actually
  closer to forward. Before running any OpModes, make sure your motors are reversed properly in the
  `Follower` class constructor. To find the direction your wheels will go, you will need to run the
  `Forward Velocity Tuner` and `Strafe Velocity Tuner` OpModes. These will run your robot at full
  power for 40 inches forward and to the right, respectively. The distance can be changed through FTC
  Dashboard under the dropdown for each respective class, but higher distances work better. After the
  distance has finished running, the end velocity will be output to telemetry. The robot may continue
  to drift a little bit after the robot has finished running the distance, so make sure you have
  plenty of room. Once you're done, put the velocity for the `Forward Velocity Tuner` on line `39` in
  the `FollowerConstants` class, and the velocity for the `Strafe Velocity Tuner` on line `40` in the
  `FollowerConstants` class. The variable names should be `xMovement` and `yMovement`, respectively.

* The last set of automatic tuners you'll need to run are the zero power acceleration tuners. These
  find the rate at which your robot decelerates when power is cut from the drivetrain. This is used to
  get a more accurate estimation of the drive vector. To find this, you will need to run the
  `Forward Zero Power Acceleration Tuner` and the `Lateral Zero Power Acceleration Tuner` OpModes.
  These will run your robot until it hits a velocity of 30 inches/second forward and to the right,
  respectively. The velocity can be changed through FTC Dashboard under the dropdown for each
  respective class, but higher velocities work better. After the velocity has been reached, power will
  be cut from the drivetrain and the robot's deceleration will be tracked until the robot stops, at
  which point it will display the deceleration in telemetry. This robot will need to drift to a stop
  to properly work, and the higher the velocity the greater the drift distance, so make sure you have
  enough room. Once you're done, put the zero power acceleration for the
  `Forward Zero Power Acceleration Tuner` on line `100` in the `FollowerConstants` class and the zero
  power acceleration for the `Lateral Zero Power Acceleration Tuner` on line `104` in the
  `FollowerConstants` class. The variable names should be `forwardZeroPowerAcceleration` and
  `lateralZeroPowerAcceleration`, respectively.

* After this, we will want to tune the translational PID. Go to FTC Dashboard and disable all but
  the `useTranslational` checkboxes under the `Follower` tab. Then, run `StraightBackAndForth`.
  Make sure you disable the timer on autonomous OpModes. You will notice in telemetry a message saying
  that the robot will travel a distance forward and backward, this will not happen until later, so for
  now you can ignore this message. The robot should not move when you run the opmode initally. Instead,
  it should correct when you push it away from its   starting position. Note that this correction should
  happen regardless of the robot's rotation, and   that the robot should not rotate itself (if it does,
  disable `useHeading` as mentioned prior). Also note that the robot will only correct to an imaginary line
  that runs straight forward from the robot's starting position, meaning that it will not correct in the
  (original) forward direction. The PID for the translational error is called `translationalPIDF`.
  If you need to add a feedforward value, use the `translationalPIDFFeedForward` since that will add
  the feedforward in the direction the robot is trying to move, rather than the feedforward in the
  PIDF itself, since those will only add the feedforward one way. You can change   the PIDF constants
  and feedforward values, under the `FollowerConstants` tab in FTC Dashboard.
  To tune the PID, push the robot off the path and see how corrects. You will want to alternate sides
  you push to reduce field wear and tear as well as push with varying power and distance. I would 
  recommend tuning the PID so that it is capable of correcting while minimizing oscillations and still
  achieving a satisfactory level of accuracy. Overall, try to tune for fewer oscillations rather than
  higher speeds or perfect accuracy, since this will make the robot run more smoothly under actual
  pathing conditions.

* Next, we will tune the heading PID. The process is essentially the same as above, except you will
  want to only enable `useHeading` under `Follower` on FTC Dashboard, as well as turn the robot from
  opposing corners instead of pushing the robot. Naturally, instead of changing the stuff with
  "translational" in the name, you will instead want to look for stuff with "heading" in the name.
  Otherwise, these two PIDs are functionally very similar. The same tips from above will apply to this.

* Afterwards, we will tune the drive PID. Before we continue, we will need to set the
  `zeroPowerAccelerationMultiplier`. This determines how fast your robot will decelerate as a factor
  of how fast your robot will coast to a stop. Honestly, this is up to you. I personally used 4, but
  what works best for you is most important. Higher numbers will cause a faster brake, but increase
  oscillations at the end. Lower numbers will do the opposite. This can be found on line `113` in
  `FollowerConstants`, named `zeroPowerAccelerationMultiplier`. The drive PID is much, much more sensitive than the others. For reference,
  my P values were in the hundredths and thousandths place values, and my D values were in the hundred
  thousandths and millionths place values. To tune this, enable `useDrive`, `useHeading`, and
  `useTranslational` in the `Follower` dropdown in FTC Dashboard. Next, run `StraightBackAndForth`
  and don't forget to turn off the timer on the OpMode. Then, tune the PID following the tips from
  earlier. For this, it is very important to try to reduce oscillations. Additionally, I would
  absolutely not recommend using the I, or integral, part of the PID for this. Using integral in
  drivetrain PIDs is already not ideal, but it will continuously build up error in this PID, causing
  major issues when it gets too strong. Don't use I; P and D are enough. In the versions of Pedro Pathing
  from after late July 2024, there is a Kalman filter on the drive error and the drive PID has a
  filter as well. These smooth out the drive error and PID response so that there is not as much
  oscillation during the braking portion of each path. The Kalman filter is tuned to have 6 for the
  model covariance and 1 for the data covariance. These values should work, but if you feel inclined
  to tune the Kalman filter yourself, a higher ratio of model covariance to data covariance means that
  the filter will rely more on its previous output rather than the data, and the opposite ratio will
  mean that the filter will rely more so on the data input (the raw drive error) rather than the model.
  The filtered PID functions like a normal PID, except the derivative term is a weighted average of the
  current derivative and the previous derivative. Currently, the weighting, or time constant for the
  drive filtered PID is 0.6, so the derivative output is 0.6 times the previous derivative plus 0.4
  times the current derivative. Feel free to mess around with these values and find what works best
  for your robot!

* Finally, we will want to tune the centripetal force correction. This is a pretty simple tune. Open
  up FTC Dashboard and enable everything under the `Follower` tab. Then, run `CurvedBackAndForth`
  and turn off its timer. If you notice the robot is correcting towards the inside of the curve
  as/after running a path, then increase `centripetalScaling`, which can be found on line `95` of
  `FollowerConstants`. If the robot is correcting towards the outside of the curve, then decrease
  `centripetalScaling`.

* Once you've found satisfactory tunings for everything, run the robot around in
  `StraightBackAndForth`, `CurvedBackAndForth`, or some paths of your own making. There's also
  `Circle`, but that's more so for fun than anything else. If you notice something could be improved,
  feel free to mess around more with your PIDs. That should be all! If you have any more questions,
  refer to the OVERVIEW readme file or the README readme file. Best of luck to your team this season! :)

## Note About the PIDs
In versions of Pedro Pathing before early August 2024, there were 2 PIDs used in the translational,
heading, and drive control. However, now there is only one main PID. The old system can still be used.
Scroll down to the bottom of `FollowerConstants` and set all the booleans from lines `163` to `165`
to true. They should be named `useSecondaryTranslationalPID`, `useSecondaryHeadingPID`, and `useSecondaryDrivePID`.
This will enable the two PID system that Pedro Pathing originally used. From there, scroll
down and all the values pertaining to the secondary PIDs will be there. The two PID system works with
a PID that handles larger errors (the main PID) and a second PID to handle smaller errors (the
secondary PID). The main PID should be tuned to move the error within the secondary PID's range
without providing too much momentum that could cause an overshoot. The secondary PID should be tuned
to correct within its range quickly and accurately while minimizing oscillations.
