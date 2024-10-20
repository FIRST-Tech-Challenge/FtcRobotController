# Table of Contents

- [teamcode/](#teamcode)
    - [`Hardware`](#Hardware)
    - [`Auto`](#Auto)
    - [`DriverMode`](#DriverMode)
    - [`FileManager`](#Filemanager)
    - [`PositionInput`](#PositionInput)
    - [`TeamColor`](#TeamColor)
    - [`TeamSide`](#Teamside)
- [hardwareSystems/](#hardwareSystems)
    - [`Arm`](#Arm)
    - [`ExtendableArm`](#Extendablearm)
    - [`Wheels`](#Wheels)
    - [`MecanumWheels`](#MecanumWheels)

# [teamcode/](./)

## [`Hardware`](./Hardware.java)

A class to instantiate the robot's hardware.
It was created to reduce code redundancy between the [`Auto`](#Auto)
and [`DriverMode`](#DriverMode).
Both [`Auto`](#Auto) and [`DriverMode`](#DriverMode) instantiate a `Hardware` object.
The `Hardware` class uses objects instantiated from the classes
in [hardwareSystems](#hardwareSystems) to separate hardware devices by system.
Methods that require multiple different systems should be declared in `Hardware`.
The hardware variables(motors, servos, sensors, etc.) should be defined in the current
season's `Hardware` class.
Replace any instances of [`Arm`](#Arm) or [`Wheels`](#Wheels) with the class appropriate for this season.

## [`Auto`](./Auto.java)

The Autonomous class, which runs without driver input.
Instantiates a [`Hardware`](#Hardware) object.
The annotation `@Autonomous(name = "Auto")` means that the class will be considered
an Autonomous program with the name of "Auto."
The `runOpMode()` method runs automatically without the need to do anything.

## [`DriverMode`](./DriverMode.java)

The TeleOp class which runs using driver input.
Instantiates a [`Hardware`](#Hardware) object.
The annotation `@TeleOp(name = "DriverMode")` means that the class will be considered
a TeleOp program with the name of "DriverMode".
The `init()` and `loop()` methods both run automatically,
with `init()` running once when the program is started,
and `loop()` running at set intervals after `init()`.
In the `init()` function, a [`Hardware`](#Hardware) object is instantiated. 

## [`FileManager`](./FileManager.java)

> [!Warning]
> This class relies on java.nio.file.Paths, which is only available from SDK version 26 and onward.

Reads and writes text files in external storage.

## [`PositionInput`](./PositionInput.java)

> [!Warning]
> This class relies on java.nio.file.Paths, which is only available from SDK version 26 and onward.

> [!Note]
> You probably will not need this class.

A TeleOp that writes the [`TeamColor`](#TeamColor) and [`TeamSide`](#TeamSide) of the robot into
external storage.
Uses [`FileManager`](#FileManager) to write to external storage files.

## [`TeamColor`](./TeamColor.java)

An enum that states whether the robot is on red or blue side.

## [`TeamSide`](./TeamSide.java)

An enum that states whether the robot is on far or near side.

# [hardwareSystems/](./hardwareSystems/)

This subdirectory contains helper classes used by [`Hardware`](#Hardware).
The classes are meant to separate and organize the various systems of the robot(e.g. arms, wheels,
etc.).
Contain methods for basic tasks such as driving and lifting the arm.
Some of the classes(e.g. [`Arm`](#Arm) and [`Wheels`](#Wheels)) are abstract and are meant to be
used as superclasses.
Being abstract classes rather than interfaces prevents multiple implementing.

## [`MotorType`](./hardwareSystems/MotorType.java)

An enum that stores the type of motor(e.g. Tetrix Torquenado) and its number of ticks per
revolution.

## [`Arm`](./hardwareSystems/Arm.java)

An abstract class to control the robot's arm system.

## [`ExtendableArm`](./hardwareSystems/ExtendableArm.java)

A subclass of [`Arm`](#Arm) that controls a rotating, extendable arm with an continuous intake
servo.
Contains four inner classes(i.e. `MotorParams`, `ServoParams`, `RotationParams`,
and `ExtensionParams`) that group together parameters for the constructor.
More specific details can be found in [`Arm`](#Arm).
The current system is admittedly clunky.
If it becomes cumbersome, please do change it.

## [`Wheels`](./hardwareSystems/Wheels.java)

A abstract class for the robot's wheels.

## [`MecanumWheels`](./hardwareSystems/MecanumWheels.java)

A subclass of the [`Wheels`](#Wheels) class for controlling the driving of a four-mecanum wheel
system.
Contains an inner class(`MotorParams`) to pass in the motors and motor types to the `MecanumWheels`
constructor. 

## [`Webcam`](./hardwareSystems/Webcam.java)

A class for vision and color detection.
Currently still in progress. 