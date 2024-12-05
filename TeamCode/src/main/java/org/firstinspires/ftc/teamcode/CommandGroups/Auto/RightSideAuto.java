package org.firstinspires.ftc.teamcode.CommandGroups.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.SpecimenPlacePos;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.PlaceSpecimenAddOffset;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.WallPickUp;
import org.firstinspires.ftc.teamcode.Commands.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.SlideTargetHeight;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class RightSideAuto extends SequentialCommandGroup {

    // constructor
    public RightSideAuto() {
        // start pos (0.25, 1.6, -90) on field
        addCommands (
                // sets the starting position
                new InstantCommand(() -> RobotContainer.odometry.setCurrentPos(new Pose2d(-0.22, 1.57, new Rotation2d(Math.toRadians(-90))))),
                //makes sure the claw is closed
                new CloseClaw(),

                new Pause(0.25),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW)),

                new SpecimenPlacePos(),
//                // folds the elbow in 270
//                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(255)),
//                //powers shoulder
//                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(60)),
//
//                new Pause(1.49),
//
//
//                // lifts the shoulder up 90+-60 degrees
//                // lifts the shoulder up to 135 degrees
//                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(135)),
//
//                // folds the wrist in 0
//                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(15)),
//
//                // powers the wrist and moves it to straight position
//                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180)),


//                new FollowPath(
//                        2.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        new Rotation2d(Math.toRadians(-90.0)),
//                        new ArrayList<Translation2d>() {{ }},
//                        new Pose2d(-0.25, 1.0, new Rotation2d(Math.toRadians(-90.0))),
//                        new Rotation2d(Math.toRadians(-90))),

                //Place specimen
                new PlaceSpecimenAddOffset(),

                // back up
                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(-0.25, 1.15, new Rotation2d(Math.toRadians(90.0))),
                        new Rotation2d(Math.toRadians(-90))),


                new ArmStowHigh(),

                new Pause(0.5),

                //sweep 1
                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(-0.85, 1.1, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90.0))
                ),

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(-0.85, 0.3, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90.0))
                ),
                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(180.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(-1.15, 0.3, new Rotation2d(Math.toRadians(180.0))),
                        new Rotation2d(Math.toRadians(-90.0))
                ),
//                new FollowPath(
//                        1.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        new Rotation2d(Math.toRadians(90.0)),
//                        new ArrayList<Translation2d>() {{ }},
//                        new Pose2d(-1.15, 1.35, new Rotation2d(Math.toRadians(90.0))),
//                        new Rotation2d(Math.toRadians(-90.0))
//                ),

//                // Sweep 2
//                new FollowPath(
//                        1.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        new Rotation2d(Math.toRadians(-90.0)),
//                        new ArrayList<Translation2d>() {{ }},
//                        new Pose2d(-1.15, 0.3, new Rotation2d(Math.toRadians(-90.0))),
//                        new Rotation2d(Math.toRadians(-90.0))
//                ),
//
//                new FollowPath(
//                        1.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        new Rotation2d(Math.toRadians(180.0)),
//                        new ArrayList<Translation2d>() {{ }},
//                        new Pose2d(-1.45, 0.3, new Rotation2d(Math.toRadians(180.0))),
//                        new Rotation2d(Math.toRadians(-90.0))
//
//                ),
//
//                new FollowPath(
//                        1.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        new Rotation2d(Math.toRadians(90.0)),
//                        new ArrayList<Translation2d>() {{ }},
//                        new Pose2d(-1.45, 1.35, new Rotation2d(Math.toRadians(90.0))),
//                        new Rotation2d(Math.toRadians(-90.0))
//                ),

                new WallPickUp(),

                new PlaceSpecimenAddOffset(),

                new WallPickUp(),

                new PlaceSpecimenAddOffset()





        );


    }

}

