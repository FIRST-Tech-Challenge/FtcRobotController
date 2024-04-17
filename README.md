# Overview
This repository is a template for our FtcRobotControllers.
It is a fork of FTC's official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git).
Please feel free to modify this template as necessary. 

# Creating templates using this template
To create new repositories using this one, create a fork. That way when you update the template, you can also pull changes into the forks.
> [!Warning]
> Before you pull changes from this template into a fork, ensure that the new changes would not break the current code.

It is not recommended to push commits from the fork to the upstream. 
It is suggested to name the new repo in the format of"{starting year}-{ending year}{season name}."

# Updating with FTC's FtcRobotController
Keep this fork up to date with FTC's official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git).
To do that, go to the GitHub page for [this repository](https://github.com/chsRobotix/TemplateRobotController.git) and click on "Sync fork."
Alternatively, you can type `git remote add-url upstream https://github.com/FIRST-Tech-Challenge/FtcRobotController.git ` into the terminal to save the FtcRobotController as
"upstream." Then type `git pull upstream` to pull from the upstream.


# Contents
Our team's code is in [./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/)

## Autonomous
Runs the robot's autonomous program. It declares and and instantiates a CoyotesRobot.

## TeleOp
Runs the robot's teleop program. It declares and and instantiates a CoyotesRobot.

## CoyotesRobot
CoyotesRobot is a class of methods and properties for initializing and controlling the robot.

## MeccanumDrive
It contains code for controlling a system of four meccanum wheels. 
