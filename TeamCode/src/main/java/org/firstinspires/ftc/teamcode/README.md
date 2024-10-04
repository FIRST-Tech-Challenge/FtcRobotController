# [Hardware](./Hardware.java)

It is a class to instantiate the robot and perform basic tasks(e.g. drive the robot forward).
Both `Autonomous` and `TeleOp` instantiate a `Hardware` object.
The `Hardware` class uses objects instantiated from the classes in [hardware](./hardware).
The hardware variables(motors, servos, sensors, etc.) should be defined in the current season's `Hardware`.

> [!Important]
> DO NOT PUT THE HARDWARE NAMES IN THIS REPOSITORY. THIS IS THE TEMPLATE.
> THEY SHOULD BE PUT IN THE CURRENT SEASON'S REPOSITORY.

# [FileManager.java](./FileManager.java)

Reads and writes text files in external storage.

# Enums

## [TeamColor.java](./TeamColor.java)

An enum that states whether the robot is on red or blue side.

## [TeamSide.java](./TeamSide.java)

An enum that states whether the robot is on far or near side.

# [Hardware](./hardware/)

This directory contains helper classes used by `Hardware`.
The classes are meant to separate and organize the various systems of the robot(e.g. arms, wheels, etc.).
Some of the classes are abstract and are meant to be used as superclasses.
Being abstract classes rather than interfaces prevents multiple implementing.

## [Arm](./hardware/Arm.java)

An abstract class to control the robot's arm system.

## [Crane](./hardware/Crane.java)

A subclass of `Arm` that controls a jointed arm with a claw and lazy susan.
Not really a crane in the conventional sense.
If you can think of a better name for it, then rename it.

## [Wheels](./hardware/Wheels.java)

A abstract class for the robot's wheels,

## [Mecanum](./hardware/Mecanum.java)

A subclass of the `Wheels` class for controlling the driving of a four-meccanum wheel system.
