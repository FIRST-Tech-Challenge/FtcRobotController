# [CoyotesRobot](./CoyotesRobot.java)

It is a class to instatiate the robot and perform basic tasks(e.g. drive the robot forward).
Both Autonomous and TeleOp instantiate a CoyotesRobot.
The CoyotesRobot uses objects instantiated from the classes in [hardware](./hardware).
The hardware variables(motors, servos, sensors, etc.) should be defined in the current season's CoyotesRobot.

> [!Important]
> DO NOT PUT THE HARDWARE NAMES IN THIS REPOSITORY. THIS IS THE TEMPLATE.
> THEY SHOULD BE PUT IN THE CURRENT SEASON'S REPOSITORY.

# [FileManager.java](./FileManager.java)

Reads and writes text files in external storage.

# [TeamColor.java](./TeamColor.java)
An enum that states whether the robot is on red or blue side.

# [TeamSide.java](./TeamSide.java)
An enum that states whether the robot is on far or near side. 

# [Hardware](./hardware/)

This directory contains helper classes used by CoyotesRobot.
The classes are meant to separate and organize the various systems of the robot(e.g. arms, wheels, etc.).
Some of the classes are abstract and are meant to be used as superclasses.
Being abstract classes rather than interfaces prevents multiple implementing.

## [ArmSystem](./hardware/ArmSystem.java)

An abstract class to control the robot's arm system.

## [CraneSystem](./hardware/CraneSystem.java)

A subclass of ArmSystem that controls a jointed arm system with a claw.
Not really a crane in the conventional sense.
If you can think of a better name for it, then rename it.

## [DriveSystem](./hardware/DriveSystem.java)

A abstract class for the robot's drive system(i.e. the wheels and motors).

## [MeccanumDrive](./hardware/MeccanumDrive.java)

A subclass of DriveSystem for controlling the driving of a four-wheel meccanum wheel system.
