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

It is a class to instantiate the robot and perform basic tasks(e.g. drive the robot forward).
Both [`Auto`](#Auto) and [`DriverMode`](#DriverMode) instantiate a `Hardware` object.
The `Hardware` class uses objects instantiated from the classes in [hardwareSystems](#hardwareSystems).
The hardware variables(motors, servos, sensors, etc.) should be defined in the current season's `Hardware` class.

## [`Auto`](./Auto.java)

The Autonomous class, which runs without driver input.
Instantiates a [`Hardware`](#Hardware) class.

## [`DriverMode`](./DriverMode.java)

The TeleOp class which runs using driver input.
Instantiates a [`Hardware`](#Hardware) class.

## [`FileManager`](./FileManager.java)

Reads and writes text files in external storage.

> [!Warning]
> This class relies on java.nio.file.Paths, which is only available from SDK version 26 and onward.

## [`PositionInput`](./PositionInput.java)

A TeleOp that writes the [`TeamColor`](#TeamColor) and [`TeamSide`](#TeamSide) of the robot into external storage.
Uses [`FileManager`](#FileManager) to write to external storage files.

> [!Note]
> This class is deprecated. 

## [`TeamColor`](./TeamColor.java)

An enum that states whether the robot is on red or blue side.

## [`TeamSide`](./TeamSide.java)

An enum that states whether the robot is on far or near side.


# [hardwareSystems/](./hardwareSystems/)

This subdirectory contains helper classes used by [`Hardware`](#Hardware).
The classes are meant to separate and organize the various systems of the robot(e.g. arms, wheels, etc.).
Some of the classes(e.g. [`Arm`](#Arm) and [`Wheels`](#Wheels)) are abstract and are meant to be used as superclasses.
Being abstract classes rather than interfaces prevents multiple implementing.

## [`Arm`](./hardwareSystems/Arm.java)

An abstract class to control the robot's arm system.

## [`ExtendableArm`](./hardwareSystems/ExtendableArm.java)

A subclass of [`Arm`](#Arm) that controls a rotating, extendable arm.

## [`Wheels`](./hardwareSystems/Wheels.java)

A abstract class for the robot's wheels,

## [`MecanumWheels`](./hardwareSystems/MecanumWheels.java)

A subclass of the [`Wheels`](#Wheels) class for controlling the driving of a four-mecanum wheel system.
