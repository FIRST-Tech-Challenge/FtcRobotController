## Prerequisites
Obviously, you have to have a robot to use Pedro Pathing. Also, Pedro Pathing is only able to work
with omnidirectional drives, like mecanum drive. There is currently no support for swerve drives.
You must also have a localizer of some sort. Pedro Pathing has a drive encoder, a two tracking wheel,
and a three tracking wheel localizer. You will need to have your localizer tuned before starting to
tune PedroPathing. Check out the tuning guide under the localization tab if you're planning on using one of the
localizers available in Pedro Pathing. Additionally, using [FTC Dashboard](http://192.168.43.1:8080/dash)
will help a lot in tuning, and we have a slightly scuffed Desmos path visualizer [here](https://www.desmos.com/calculator/3so1zx0hcd).
One last thing to note is that Pedro Pathing operates in inches and radians.

## Tuning
* To start with, we need the mass of the robot in kg. This is used for the centripetal force
correction, and the mass should be put on line `114` in the `FollowerConstants` class under the
`tuning` package.

* Next, we need to find the preferred mecanum drive vectors. The rollers on mecanum wheels point at a
45 degree angle from the forward direction, but the actual direction the force is output is actually
closer to forward. To find the direction your wheels will go, you will need to run the
`Forward Velocity Tuner` and `Strafe Velocity Tuner` OpModes. These will run your robot at full
power for 40 inches forward and to the right, respectively. The distance can be changed through FTC
Dashboard under the dropdown for each respective class, but higher distances work better. After the
distance has finished running, the end velocity will be output to telemetry. The robot may continue
to drift a little bit after the robot has finished running the distance, so make sure you have
plenty of room. Once you're done, put the velocity for the `Forward Velocity Tuner` on line `25` in
the `FollowerConstants` class, and the velocity for the `Strafe Velocity Tuner` on line `26` in the
`FollowerConstants` class.

* The last set of automatic tuners you'll need to run are the zero power acceleration tuners. These
find the rate at which your robot decelerates when power is cut from the drivetrain. This is used to
get a more accurate estimation of the drive vector. To find this, you will need to run the
`Forward Zero Power Acceleration Tuner` and the `Lateral Zero Power Acceleration Tuner` OpModes.
These will run your robot until it hits a velocity of 10 inches/second forward and to the right,
respectively. The velocity can be changed through FTC Dashboard under the dropdown for each
respective class, but higher velocities work better. After the velocity has been reached, power will
be cut from the drivetrain and the robot's deceleration will be tracked until the robot stops, at
which point it will display the deceleration in telemetry. This robot will need to drift to a stop 
to properly work, and the higher the velocity the greater the drift distance, so make sure you have
enough room. Once you're done, put the zero power acceleration for the
`Forward Zero Power Acceleration Tuner` on line `122` in the `FollowerConstants` class and the zero
power acceleration for the `Lateral Zero Power Acceleration Tuner` on line `127` in the
`FollowerConstants` class.

* After this, we will want to tune the translational PIDs. Go to FTC Dashboard and disable all but
the `useTranslational` checkboxes under the `Follower` tab. Then, run `StraightBackAndForth`. Make
sure you disable the timer on autonomous OpModes. There are two different PIDs for translational
error, the `smallTranslationalPIDF` and `largeTranslationalPIDF`. If error is larger than a certain
amount, the `largeTranslationalPIDF` will be used, and if error is smaller than that amount the
`smallTranslationalPIDF` will be used. If you need to add a feedforward value, use the
`smallTranslationalPIDFFeedForward` and `largeTranslationalPIDFFeedForward` since those will add the
feedforward in the direction the robot is trying to move, rather than the feedforward in the PIDFs
themselves, since those will only add the feedforward one way. You can change the amount of error
required to switch PIDFs, as well as the PIDF constants and feedforward values, under the
`FollowerConstants` tab in FTC Dashboard. To tune the PIDs, push the robot off the path and see how
corrects. You will want to alternate sides you push to reduce field wear and tear as well as push 
with varying power and distance. I would recommend tuning the large PID first, and tuning it so that
the PID is capable of correcting to the point where the PIDs switch with little momentum. Then, tune
the small PID to minimize oscillations while still achieving a satisfactory level of accuracy.
Overall, try to tune for fewer oscillations rather than higher speeds or perfect accuracy, since
this will make the robot run more smoothly under actual pathing conditions.

* Next, we will tune the heading PIDs. The process is essentially the same as above, except you will
want to only enable `useHeading` under `Follower` on FTC Dashboard, as well as turn the robot from
opposing corners instead of pushing the robot. Naturally, instead of changing the stuff with
"translational" in the name, you will instead want to look for stuff with "heading" in the name.
Otherwise, these two PIDs are functionally very similar. The same tips from above will apply to this.

* Afterwards, we will tune the drive PIDs. Before we continue, we will need to set the
`zeroPowerAccelerationMultiplier`. This determines how fast your robot will decelerate as a factor
of how fast your robot will coast to a stop. Honestly, this is up to you. I personally used 4, but
what works best for you is most important. Higher numbers will cause a faster brake, but increase
oscillations at the end. Lower numbers will do the opposite. This can be found on line `136` in
`FollowerConstants`. There are once again two PIDs for the drive vector, but these PIDs are much,
much more sensitive than the others. For reference, my P values were in the hundredths and
thousandths place values, and my D values were in the hundred thousandths and millionths place
values. To tune this, enable `useDrive`, `useHeading`, and `useTranslational` in the `Follower`
dropdown in FTC Dashboard. Next, run `StraightBackAndForth`and don't forget to turn off the timer on
the OpMode. Then, tune the large PID and then the small PID following the tips from earlier. For
this, it is very important to try to reduce oscillations. Additionally, I would absolutely not
recommend using the I, or integral, part of the PID for this. Using integral in drivetrain PIDs is
already not ideal, but it will continuously build up error in this PID, causing major issues when
it gets too strong. So, just don't use it. P and D are enough.

* Finally, we will want to tune the centripetal force correction. This is a pretty simple tune. Open
up FTC Dashboard and enable everything under the `Follower` tab. Then, run `CurvedBackAndForth`
and turn off its timer. If you notice the robot is correcting towards the inside of the curve
as/after running a path, then increase `centripetalScaling`, which can be found on line `117` of
`FollowerConstants`. If the robot is correcting towards the outside of the curve, then decrease
`centripetalScaling`.

* Once you've found satisfactory tunings for everything, run the robot around in
`StraightBackAndForth`, `CurvedBackAndForth`, or some paths of your own making. There's also
`Circle`, but that's more so for fun than anything else. If you notice something could be improved,
feel free to mess around more with your PIDs. That should be all! If you have any more questions,
refer to the OVERVIEW readme file or the README readme file. Best of luck to your team this season! :)