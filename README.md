# Animatores Romani (4965) Official Freight Frenzy Repo!

# NOTICE

This repository contains Latin School's fork of the public FTC SDK for the Freight Frenzy (2021-2022) competition season. **If you are looking for the official SDK/if you are not a Latin School of Chicago student, you're in the wrong place!** click [here :)](https://github.com/FIRST-Tech-Challenge/FtcRobotController)


## Welcome! ＼(≧▽≦)／
This GitHub repository contains the source code that is used to build an Android app to control a *FIRST* Tech Challenge competition robot.  To use this SDK, download/clone the entire project to your local computer.

## Why GitHub? ┐('～`;)┌

1. We've had code randomly vanish off the phone, thats not good. Github prevents "Disney Pixar's Finding auto.java"

2. Version control, lets us separate previous code, working code, and in progress code

3. easy to access in the future

4. forces us to be organized

5. if you like to code, u will probably use it in the future

6. sounds cool to judges (i think?)

# Getting Started With GIT


## Cloning the Repository

1. Touch base with [Charlie Gray](https://github.com/charlie-gray) in order to be **added to the GitHub group**, especially if you are an underclassman. This allows us to keep all of the code from each year in one place so we can look back and steal our own code. You can email Charlie at `cgray@lsoc.org` if you need to be added to the group <strike>or if you want to complain about the obnoxious emoticions in this readme document.</strike>
2. Once we've taken care of that housekeeping item, the next step is to **install git**. You can learn how to do that [here](https://git-scm.com/downloads)
3. After you've installed git, you'll need to clone the repository to your computer. In order to do this, you'll need some experience using your computer's command line prompt. If you don't know how to do that, I'd *highly* reccomend checking out [this codeacademy course](https://www.codecademy.com/learn/learn-the-command-line). Refreshed on command line work? Great! Change directories to where you want to clone (or "download") this repository. Then make a directory using `mkdir`. `cd` into that directory you just made, then run `git clone https://github.com/Latin-School-robotics/LSOC-Freight-Frenzy.git`. Horray! We just cloned the repository! \(^ヮ^)/

## Pushing to the Repository

### ✋ STOP 🛑
**READ THIS SECTION IN IT'S ENTIERTY BEFORE PUSHING ANY CODE TO THE REPOSITORY**

If you've never used git before it is **highly** reccomended that you go through this [codeacademy course](https://www.codecademy.com/learn/learn-git) before proceeding. 

It is essential that you understand the [GitHub Flow](https://guides.github.com/introduction/flow/) before proceeding.

**ALWAYS ALWAYS ALWAYS PUSH CODE TO A SEPRATE BRANCH**
NEVER push new code to the `main` branch. Ever. 

1. First [create your branch](https://docs.github.com/en/github/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-and-deleting-branches-within-your-repository).
2. Next, run `git checkout -b [branch-name]` filling in the name of the branch you just created
    **tip** to see a list of all of the branches, run `git branch -a`

## Commiting

This is how we send your code to your new branch.

1. Run `git status` to double check that you're staging the right branch
2. Run `git add .`
3. Run `git commit -m "message"`
    **NOTE:** ALWAYS fill out your message with a basic summary of what you changed.
4. Finally, push your code by running `git push origin [your-new-branch]`

You've succesfully submitted your first commit! Congrats! (ﾉ^ヮ^)ﾉ*:・ﾟ✧

## Pull Requests

We merge code from your branch to the `main` branch using Pull Requests. To learn how pull requests work [click here](https://docs.github.com/en/github/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/creating-a-pull-request). 

# Getting Started With FIRST Code
If you are new to robotics or new to the *FIRST* Tech Challenge, then you should consider reviewing the [FTC Blocks Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial) to get familiar with how to use the control system:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Blocks Online Tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial)

Even if you are an advanced Java programmer, it is helpful to start with the [FTC Blocks tutorial](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Blocks-Tutorial), and then migrate to the [OnBot Java Tool](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/OnBot-Java-Tutorial) or to [Android Studio](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki/Android-Studio-Tutorial) afterwards.

## Getting Help
### User Documentation and Tutorials
*FIRST* maintains online documentation with information and tutorials on how to use the *FIRST* Tech Challenge software and robot control system.  You can access this documentation using the following link:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FtcRobotController Online Documentation](https://github.com/FIRST-Tech-Challenge/FtcRobotController/wiki)

Note that the online documentation is an "evergreen" document that is constantly being updated and edited.  It contains the most current information about the *FIRST* Tech Challenge software and control system.

### Javadoc Reference Material
The Javadoc reference documentation for the FTC SDK is now available online.  Click on the following link to view the FTC SDK Javadoc documentation as a live website:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Javadoc Documentation](https://javadoc.io/org.firstinspires.ftc)

### Online User Forum
For technical questions regarding the Control System or the FTC SDK, please visit the FTC Technology forum:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[FTC Technology Forum](https://ftcforum.firstinspires.org/forum/ftc-technology)

### Sample OpModes
This project contains a large selection of Sample OpModes (robot code examples) which can be cut and pasted into your /teamcode folder to be used as-is, or modified to suit your team's needs.

Samples Folder: &nbsp;&nbsp; [/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples](FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples)

The readme.md file located in the [/TeamCode/src/main/java/org/firstinspires/ftc/teamcode](TeamCode/src/main/java/org/firstinspires/ftc/teamcode) folder contains an explanation of the sample naming convention, and instructions on how to copy them to your own project space.
