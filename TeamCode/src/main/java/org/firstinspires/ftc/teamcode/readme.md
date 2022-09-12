

## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController / java / org.firstinspires.ftc.robotcontroller / external / samples

A range of different samples classes can be seen in this folder.
The class names follow a naming convention which indicates the purpose of each class.
The full description of this convention is found in the samples/sample_convention.md file.

A brief synopsis of the naming convention is given here:
The prefix of the name will be one of the following:

* Basic:    This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.
* Sensor:   This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended as a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.
* Hardware: This is not an actual OpMode, but a helper class that is used to describe
            one particular robot's hardware devices: eg: for a Pushbot.  Look at any
            Pushbot sample to see how this can be used in an OpMode.
            Teams can copy one of these to create their own robot definition.
* Pushbot:  This is a Sample OpMode that uses the Pushbot robot structure as a base.
* Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the header should reference an external doc, guide or tutorial.
* Library:  This is a class, or set of classes used to implement some strategy.
            These will typically NOT implement a full OpMode.  Instead they will be included
            by an OpMode to provide some stand-alone capability.

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode / java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



