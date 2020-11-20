What is the backend folder?
----------------------------------------------------------------------------------------------------
The backend folder is designed to hold all of the code that involves some of the 'lower-level'
functions. In other words, here's where the knitty-gritty code lies. Much of the code in this
folder deals with the design of the PID controller as well as some hardware abstractions that
should make it easier to actually 'program' solutions instead of getting bogged down with
boiler plate code. The goal being that most of the code here won't need to be accessed for the
average programmer.

How do I use it?
----------------------------------------------------------------------------------------------------
Unless you are trying to build on some of the features, I don't foresee a case where you'd have
to use this part of the library. If there are errors in your code, I would suggest looking at the
question below and attempting to dissect the problem. In any other scenario, most interactions
between the frontend (the part the programmer sees) and the backend (the 'gears' of it all) happen
through the midle-end folder. This is where the code is implemented for readability and further
abstracted for functionality.

What is actually in each folder?
----------------------------------------------------------------------------------------------------
I have included a description of all the folders and their purpose below to help dissect problems
when they occur:

"control" - the control folder includes the lower level code for the PID controllers. As of when
this is being written, the "PIDV2" class is the most up-to-date class for PID controllers and has
proved to be functional for our current needs. Possible revisions may be made, but this has worked
to handle a wide case of uses such as IMU angle correction, encoder distance estimation, as well as
other functions not listed here. For the "mathy" part of the code look at the "low_level" folder.
You will also find the "higher_level" folder within the "control" section of the backend. The
"higher_level" folder deals with actually implementing the "PIDV2" class. Currently, there are
better examples as to how to implement the "PIDV2" class in the "DeadReckoning" class found inside
of the "localisation" folder.

"hardware_extensions" - when programming some of the features, I found that the motor libraries
didn't have a lot of the small functions I frequently had to reprogram such as encoder to distance
conversions. In that vein, I decided to expand on certain hardware classes in order to keep the
programs manageable. A lot of the things these classes implement can be done with the standard
motor, imu, or servo classes but these simply serve the purpose of making code more readable.

"localisation" - this folder holds all the code necessary for dealing with our localisation
needs. This folder is really a combination of "PIDV2" implementations and a lot of simple
trigonometry math used to fuse the imu and sensor functions.

"robot_abstractions" - this folder houses a lot of the code that deals with creating abstractions
of common robot features. An example is the robot's drivetrain. It is common for the only 'real'
change in the drivetrain to be the 4 motors, wheel diameter, and distance between wheels and
center of mass. The rest is always the same, but each year we always reprogram it. I have decided
to end this tradition and create a drivetrain class that combines a lot of the aspects held across
drivetrains.

There is an error in my code, should I look here?
----------------------------------------------------------------------------------------------------
If there is an error in your code, chances are it comes from here. Before you go digging and
changing code, however, let me make one thing clear. These classes are the foundation of the
entire robotics code. Let's say you are working on a teleop and you get an error and you think it
is coming from here. Be aware that changes made to 'fix' your teleop might cause errors in
literally all other programs because many of the main classes implement these as base classes.

With that out of the way, your problem most likely comes from here. In order to test and make sure,
however, you can do a series of things. First of all, has it always been an issue?

Did you just implement a new feature that caused your program to break? Try removing the feature
from the program you are working on and test another teleop/auto that has that same feature. If
the issue persists across two different programs, it is very likely that (a) your are implementing
something wrong or (b) the issue is actually coming from here.

An example being, I want to add a PID controller for a servo. I program something that uses the base
PIDV2 class and add it to my TeleOp. When I run it, it doesn't do what I expect. First of all, it
could be a tuning issue (make sure you try many PID values). Second of all, test with another
program that also uses the PIDV2 class. If both programs fail to operate in the correct manner or
simply throw an error, this is becuase the base PIDV2 class most likely needs debugging. If the
error only occurs in one program, the issue is NOT with the PIDV2 class. The issue most likely
comes from how you implemented your logic.

Furthermore, keep in mind that I implemented these classes with my own purposes in mind. Classes
that operate using methods derived from the FTC SDK are bound to become depreciated at some point.
When this does happen, I wish you the best of luck and I hope that this document serves you well.
Likewise, if you believe that the issue is coming from one of these files, I bestow upon you
the gift of patience because you will most likely break some of my implementations and you'll have
to figure out how to solve that.

That isn't to say you shouldn't tinker around with these files. As a matter of fact, I think you
should. My code is by no means perfect and can 100% be optimized. Just be conscious of what you
are doing and for the love of god make sure you commit frequently!