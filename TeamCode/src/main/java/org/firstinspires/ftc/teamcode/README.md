# [Auto.java](./Auto.java)

The autonomous class, which runs without driver input.

# [FileManager.java](./FileManager.java)

Reads and writes text files in external storage.

# [Hardware](./Hardware.java)

It is a class to instantiate the robot and perform basic tasks(e.g. drive the robot forward).
Both `Auto` and `TeleOp` instantiate a `Hardware` object.
The `Hardware` class uses objects instantiated from the classes in [hardware](./hardware).
The hardware variables(motors, servos, sensors, etc.) should be defined in the current season's `Hardware`.

# [PositionInput.java](./PositionInput.java)

A TeleOp that writes the TeamColor and TeamSide of the robot into external storage. 

> [!Note]
> This class is deprecated. 

# [TeamColor.java](./TeamColor.java)

An enum that states whether the robot is on red or blue side.

## [TeamSide.java](./TeamSide.java)

An enum that states whether the robot is on far or near side.


# [TeleOp.java](./DriverMode.java)

The TeleOp class which runs using driver input.


# [Hardware](./hardware/)

This directory contains helper classes used by `Hardware`.
The classes are meant to separate and organize the various systems of the robot(e.g. arms, wheels, etc.).
Some of the classes are abstract and are meant to be used as superclasses.
Being abstract classes rather than interfaces prevents multiple implementing.

## [Arm](./hardware/Arm.java)

An abstract class to control the robot's arm system.

## [ExtendableArm](./hardware/ExtendableArm.java)

A subclass of `Arm` that controls a rotating, extendable arm.

## [Wheels](./hardware/Wheels.java)

A abstract class for the robot's wheels,

## [MecanumWheels](./hardware/MecanumWheels.java)

A subclass of the `Wheels` class for controlling the driving of a four-meccanum wheel system.
