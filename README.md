# Overview
This repository is a template for our FtcRobotControllers.
It is a fork of FTC's official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git).
Please feel free to modify this template as necessary. 

# Creating repositories wth this template
The repository is configured to be a repository template, so to use it, simply create a new repository under chsRobotix,
click under "Repository template," and select "chsRobotix/TemplateRobotController." It is suggested to name it in the format of
"{starting year}-{ending year}{season name}."

# Syncing with FTC's FtcRobotController
Keep this fork up to date with FTC's official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git).
To do that, go to the GitHub page for [this repository](https://github.com/chsRobotix/TemplateRobotController.git) and click on "Sync fork."
Alternatively, you can type `git remote add-url upstream https://github.com/FIRST-Tech-Challenge/FtcRobotController.git ` into the terminal to save the FtcRobotController as
"upstream." Then type `git pull upstream` to pull from the upstream.

# Contents
In [our team's code](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/), there is an [Autonomous](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Autonomous.java) and a [TeleOp](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/TeleOp.java). The directory also contains a submodule([CoyotesRobot](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/CoyotesRobot)), which allows for the template and library to be modified independently of each other. 

## Autonomous
Runs the robot's autonomous program. It declares and and instantiates a CoyotesRobot.

## TeleOp
Runs the robot's teleop program. It declares and and instantiates a CoyotesRobot.

## CoyotesRobot
CoyotesRobot contains classes and methods for controlling the robot.