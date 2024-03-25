# Documentation

## Overview
This codebase is for a FIRST Tech Challenge (FTC) robot. It includes several autonomous and teleoperated (TeleOp) modes that the robot can run during a match. The robot's movements are controlled by four motors.

## Autonomous Modes
There are three autonomous modes: `AutoRed`, `AutoBlue`, and `AutoBackstage`. Each of these modes is designed for a specific starting position on the field. The robot uses encoders on its drive motors to move a specific distance.

### AutoRed
In `AutoRed` mode, the robot turns 60 inches to the right and then moves forward 79 inches.

### AutoBlue
In `AutoBlue` mode, the robot turns 58 inches to the left and then moves forward 79 inches.

### AutoBackstage
In `AutoBackstage` mode, the robot moves forward 28 inches.

## Teleoperated Modes
There are two TeleOp modes: `BlueTeleop` and `RedTeleop`. In these modes, the robot's movements are controlled by a gamepad. The left stick controls the robot's forward and strafe movements, and the right stick controls the robot's turning.

### BlueTeleop
In `BlueTeleop` mode, the robot's forward and strafe movements are reversed compared to `RedTeleop` mode.

### RedTeleop
In `RedTeleop` mode, the robot's forward and strafe movements are as described above.

## Hardware
The robot uses four motors for movement: `BackLeft`, `BackRight`, `FrontLeft`, and `FrontRight`. The `FrontRight` and `FrontLeft` motors are reversed so that a positive power command moves the robot forward.