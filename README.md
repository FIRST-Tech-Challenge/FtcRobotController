# Overview

This repository is a template for our FtcRobotControllers.
It is a fork of FTC's official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git).
Please feel free to modify this template as necessary.
However, this template is meant for general code to be reused across seasons,
so please refrain from putting anything year specific into this template
(e.g. motor names or autonomous commands).

# Creating templates using this template

To create new repositories using this one, go to GitHub and and create a new repository with chsRobotix as the owner. Then,
under "Repository template" select "chsRobotix/TemplateRobotController." By default, GitHub does not allow users to pull from a template,
which complicates updating the new repository if the template changes. However, it also is safer since it prevents pushing to the template.
To pull from the template, type

```
git remote add template https://github.com/chsRobotix/TemplateRobotController.git
git pull template main
```

> [!Warning]
> Before you pull changes from this template into a fork, ensure that the new changes would not break the current code.

It is suggested to name the new repo in the format of "{starting year}-{ending year}{season name}." After creating the new repository,
update the README.md accordingly.

# Updating with FTC's FtcRobotController

Keep this fork up to date with FTC's official [FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController.git).
To do that, go to the GitHub page for [this repository](https://github.com/chsRobotix/TemplateRobotController.git) and click on "Sync fork."
Alternatively, you can type

```
git remote add upstream https://github.com/FIRST-Tech-Challenge/FtcRobotController.git
git pull upstream master
```

# Contents

Our team's code is in [./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/).
It contains an autonomous, teleop, and various helper classes to ease the process of programming the robot.
For more details, look at the [README.md](./TeamCode/src/main/java/org/firstinspires/ftc/teamcode/README.md)
