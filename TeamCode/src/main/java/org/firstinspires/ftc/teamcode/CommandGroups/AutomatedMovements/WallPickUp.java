package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class WallPickUp extends SequentialCommandGroup {

    // constructor
    public WallPickUp() {

        addCommands (
        // What this position should do is give the camera a good vantage point as well as keep the arm out of the way

                // Spline created w. start angle of 90 will ensure that the robot pulls away from submersible on wall
                // cycling from last drop accounting for submersible legs / structure inset
//                new FollowPath(
//                        1.0,
//                        0.4,
//                        0.0,
//                        0.0,
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(135.0))),
//                        new ArrayList<Translation2d>() {{ }},//1.0y
//                        AutoFunctions.redVsBlue(new Pose2d(-1.2, 1.2, new Rotation2d(Math.toRadians(90.0)))),
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),


                new MoveToPose(
                        1.0,
                        0.5,
                        AutoFunctions.redVsBlue(new Pose2d(-1.2, 1.2, new Rotation2d(Math.toRadians(-90.0))))
                ),

                // lifts the shoulder up 90+-60 degrees
                // lifts the shoulder up to 135 degrees
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(25)),

                // folds the elbow in 270
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(270)),

                // folds the wrist in 0
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(106)),

                // powers the wrist and moves it to straight position
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(0)),

                new Pause(0.25),

                new OpenClaw(),

                new Pause(0.25),

                // use side sensor to determine/set current robot x-Pose. Leave y and rotation as-is.
                new InstantCommand(()-> {
                    Pose2d position = RobotContainer.odometry.getCurrentPos();
                    Pose2d correctedPosition;
                    if(RobotContainer.isRedAlliance)
                        correctedPosition = new Pose2d(1.78435-0.19+0.0-0.01*RobotContainer.rightDistance.getDistance(), position.getY(), position.getRotation());
                    else
                        correctedPosition = new Pose2d(-1.78435+0.19-0.0+0.01*RobotContainer.rightDistance.getDistance(), position.getY(), position.getRotation());

                    RobotContainer.odometry.setCurrentPos(correctedPosition);
                }),

                new FollowPath(
                        0.3,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(90.0))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(-1.2, 1.45, new Rotation2d(Math.toRadians(90.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),

//                // Zoe: This could likely be a good case for a simple MoveToPose()
//                // This might allow Lonan to have an easier time to predict the robot's approach on pickup.
//                new MoveToPose(
//                        0.3,
//                        1.0,
//                        AutoFunctions.redVsBlue(new Pose2d(-1.2, 1.45, new Rotation2d(Math.toRadians(-90.0))))
//                ),

//               // Zoe: Lets play with these pauses to see if the grab response improves
                new Pause(0.35),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                new Pause(0.25)


        );
    }

}