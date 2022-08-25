## Sample Class/Opmode conventions
#### V 1.1.0  8/9/2017

This document defines the FTC Sample OpMode and Class conventions.

### OpMode Name

A range of different samples classes will reside in the java/external/samples folder.

For ease of understanding, the class names will follow a naming convention which indicates
the purpose of each class.  The prefix of the name will be one of the following:

Basic:	    This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones Tank Drive examples.

Sensor:    	This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended to drive a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.

Hardware:	This is not an actual OpMode, but a helper class that is used to describe
            one particular robot's hardware configuration:   eg: For the K9 or Pushbot.
            Look at any Pushbot sample to see how this can be used in an OpMode.
            Teams can copy one of these to create their own robot definition.

Pushbot:	This is a Sample OpMode that uses the Pushbot robot hardware as a base.
            It may be used to provide some standard baseline Pushbot opmodes, or
            to demonstrate how a particular sensor or concept can be used directly on the
            Pushbot chassis.

Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the comments should reference an external doc, guide or tutorial.
            Each OpMode should try to only demonstrate a single concept so they are easy to
            locate based on their name.

Library:    This is a class, or set of classes used to implement some strategy.
            These will typically NOT implement a full opmode.  Instead they will be included
            by an OpMode to provide some stand-alone capability.

After the prefix, other conventions will apply:

* Sensor class names should constructed as:       Sensor - Company - Type
* Hardware class names should be constructed as:  Hardware - Robot type
* Pushbot class names should be constructed as:   Pushbot - Mode - Action - OpModetype
* Concept class names should be constructed as:   Concept - Topic - OpModetype
* Library class names should be constructed as:   Library - Topic - OpModetype

### Sample OpMode Content/Style

Code is formatted as per the Google Style Guide:

https://google.github.io/styleguide/javaguide.html

With “Sensor” and “Hardware” samples, the code should demonstrate the essential function,
and not be embellished with too much additional “clever” code.  If a sensor has special
addressing needs, or has a variety of modes or outputs, these should be demonstrated as
simply as possible.

Special programming methods, or robot control techniques should be reserved for “Concept” Samples,
and where possible, Samples should strive to only demonstrate a single concept,
eg: State machine coding, or a User Menu system, and not combine them into a single “all inclusive”
sample.  This will prevent an “all inclusive” Sample being deleted just because one part of it
becomes obsolete.

### Device Configuration Names

The following device names are used in the external samples
 
** Motors:
left_drive
right_drive
left_arm

** Servos:
left_hand
right_hand
arm
claw

** Sensors:
sensor_color
sensor_ir
sensor_light
sensor_ods
sensor_range
sensor_touch
sensor_color_distance
sensor_digital
digin
digout

** Localization:
compass
gyro
imu 
navx

### Device Object Names

Device Object names should use the same words as the device’s configuration name, but they
should be re-structured to be a suitable Java variable name.  This should keep the same word order,
but adopt the style of beginning with a lower case letter, and then each subsequent word
starting with an upper case letter.

Eg: from the examples above:  tool, leftMotor, rightClawServo, rearLightSensor.

Note:  Sometimes it’s helpful to put the device type first, followed by the variant.
eg:  motorLeft and motorRight, but this should only be done if the same word order
is used on the device configuration name.

### OpMode code Comments

Sample comments should read like normal code comments, that is, as an explanation of what the
sample code is doing.  They should NOT be directives to the user,
like: “insert your joystick code here” as these comments typically aren’t
detailed enough to be useful.  They also often get left in the code and become garbage.

Instead, an example of the joystick code should be shown with a comment describing what it is doing.
