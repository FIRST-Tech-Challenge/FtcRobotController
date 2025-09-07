## TeamCode Module

Welcome!

TeamCode is where you will put all related code files for controlling the robot. This will be the 
only place to store these types of code files.

## Committing to the remote repository
The remote repository consists of two main branches (independent lines of development):
- **master**: This syncs with the base FTCRobotController and is only used to receive necessary
  package upgrades from the base repository. **This should not be committed to.**
- **competition-code**: **This is where you will commit new code.** We will use a system of *pull 
  requests* and approvals to update and merge code. Make sure your pull requests go towards this branch.

## Creating your own OpModes

There are two main types of OpModes and movement programs: RobotAuto and RobotTeleOp: one is for the
autonomous period and one is for the tele-op period. Both files have special attributes that are
needed for functionality:
- RobotAuto files uses a @Autonomous(name, group) attribute.
- RobotTeleOp files use a @TeleOp(name, group) attribute.

Enabling and disabling OpModes is done with the @Disabled attribute. If this appears, this file
will not be shown as an available OpMode. Comment out or delete this attribute to enable.

### Naming of Samples

(Taken from FtcRobotController)
To gain a better understanding of how the samples are organized, and how to interpret the
naming system, it will help to understand the conventions that were used during their creation.

These conventions are described (in detail) in the sample_conventions.md file in this folder.

To summarize: A range of different samples classes will reside in the java/external/samples.
The class names will follow a naming convention which indicates the purpose of each class.
The prefix of the name will be one of the following:

Basic:  	This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Robot:	    This is a Sample OpMode that assumes a simple two-motor (differential) drive base.
            It may be used to provide a common baseline driving OpMode, or
            to demonstrate how a particular sensor or concept can be used to navigate.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.  These OpModes may not produce a drivable robot.

After the prefix, other conventions will apply:

* Sensor class names are constructed as:    Sensor - Company - Type
* Robot class names are constructed as:     Robot - Mode - Action - OpModetype
* Concept class names are constructed as:   Concept - Topic - OpModetype

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the TeamCode/java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

## That's it!
Check out the existing files here and understand how everything works, or start programming!