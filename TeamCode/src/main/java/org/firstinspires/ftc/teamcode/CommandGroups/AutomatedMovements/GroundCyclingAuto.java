package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.DropToGrab;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.HuntingPos;
import org.firstinspires.ftc.teamcode.Commands.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class GroundCyclingAuto extends SequentialCommandGroup {

    // constructor
    public GroundCyclingAuto() {

        addCommands (
                // pick up middle
                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-135))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(1.51, 1.05, new Rotation2d(Math.toRadians(-90)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),

                new Pause(0.5),

                new HuntingPos(),

                new DropToGrab(),

                new Pause(0.5),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new HighBucketDeposit(),

                // pick up right
                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-135))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(1.28, 1.06, new Rotation2d(Math.toRadians(-90)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),

                new HuntingPos(),

                new DropToGrab(),

                new Pause(0.5),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new HighBucketDeposit(),


                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.25,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-135))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(1.3, 0.5, new Rotation2d(Math.toRadians(-45)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(50)))),

                // pick up left
                new FollowPath(
                        0.5,
                        0.75,
                        0.25,
                        0.0,
                      AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-10))),
                      new ArrayList<Translation2d>() {{ }},
                      AutoFunctions.redVsBlue(new Pose2d(1.66, 0.28, new Rotation2d(Math.toRadians(0)))),
                      AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(60)))),


                // drops the elbow to 175 degrees for pick up
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(175)),

                // moves shoulder to 165 degrees so slightly down from hunting pos
                new InstantCommand(() -> RobotContainer.shoulderJoint.RotateTo(170)),

                // same as in hunting pos moving wrist 45 degrees
                new Pause(0.25),

                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(45)),

                // sets wrist rotate to 150
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(170)),

                new Pause(1.45),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                new Pause(1.0),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new HighBucketDeposit()

        );
    }

}

