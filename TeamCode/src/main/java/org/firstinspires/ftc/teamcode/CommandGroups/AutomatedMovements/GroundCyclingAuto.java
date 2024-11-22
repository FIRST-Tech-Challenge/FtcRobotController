package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

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
                        new Rotation2d(Math.toRadians(-135)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(1.56, 1.05, new Rotation2d(Math.toRadians(-90))),
                        new Rotation2d(Math.toRadians(-90.0))),

                new Pause(0.5),

                new HuntingPos(),

                new Pause(0.5),

                new DropToGrab(),

                new Pause(0.5),

                new CloseClaw(),

                new Pause(0.5),

                new ArmStowHigh(),

                new BlueSideHighBucketDeposit(),

                // pick up right
                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-135)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(1.31, 1.05, new Rotation2d(Math.toRadians(-90))),
                        new Rotation2d(Math.toRadians(-90.0))),

                new Pause(0.5),

                new HuntingPos(),

                new Pause(0.5),

                new DropToGrab(),

                new Pause(0.5),

                new CloseClaw(),

                new Pause(0.5),

                new ArmStowHigh(),

                new BlueSideHighBucketDeposit()

//                //pick up left
//                new FollowPath(
//                        1.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        new Rotation2d(Math.toRadians(-135)),
//                        new ArrayList<Translation2d>() {{ }},
//                        new Pose2d(1.51, 0.97, new Rotation2d(Math.toRadians(-45))),
//                        new Rotation2d(Math.toRadians(-45))),
//
//                new Pause(2),
//
//                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180)),
//
//                new HuntingPos(),
//
//                new Pause(1),
//
//                new DropToGrab(),
//
//                new Pause(1),
//
//                new CloseClaw(),
//
//                new Pause(1),
//
//                new ArmStowHigh(),
//
//                new BlueSideHighBucketDeposit()

                // x - 1.209
                // y - 1.3655
                //


        );
    }

}

// Example #1: Lily's 2023 FRC super cube auto
/*          // enable arm, and lift to stow position
            new InstantCommand(() -> RobotContainer.arm.SetEnableArm(true)),

            // move arm back to drop off cube
            new InstantCommand(() -> RobotContainer.arm.SetArmPosition(RobotContainer.arm.HIGH_DEG)),

            // delay until arm gets back
            new DelayCommand(1.0),

            // place cube
            new InstantCommand(() -> RobotContainer.grabber.setClose()),

            // delay for gripper to close
            new DelayCommand(0.7),

            // move arm to 'forward position' but inside robot bumper)
            // move to 135deg
            new InstantCommand(() -> RobotContainer.arm.SetArmPosition(135.0)),

            // delay for arm to get to stow
            new DelayCommand(1.5),

            // ensure arm is stowed before it is allow to begin moving over charge station
            new SafetyCheckStowPosition(),

            // drive right
            // new DrivetoRelativePose(new Pose2d(0,-2.0, new Rotation2d(0.0)), 1.0, 0.1, 5.0),

            // drive straight
            new DrivetoRelativePose(new Pose2d(5.0, 0, new Rotation2d(0.0)),1.8,0.1, 7.0),

            // pick up cube from floor :)
            new AutoFloorCubePickup(),

            // delay
            new DelayCommand(0.5),

            // drive back
            //new DrivetoRelativePose(new Pose2d(1.0,0, new Rotation2d(0.0)), 1.0, 0.1, 2.0),

            // drive left to center
            new DrivetoRelativePose(new Pose2d(-1.0,2.0, new Rotation2d(0.0)), 1.8, 0.1, 5.0),

            // drive straight onto charge station
            new DrivetoRelativePose(new Pose2d(-1.5, 0, new Rotation2d(0.0)),1.0,0.1, 30.0),

            // balance
            new AutoBalance()

// Example #2: Matthew's 2024 shoot donut sequence.
This sequence contains parallel and parallelrace subgroups within an overall series command

      addCommands(
      new ParallelRaceGroup(
        new AimToSpeaker(),
        new SpinupSpeaker()
      ),
      new ParallelCommandGroup(
        new WaitForEffectorAngle(),
        new WaitForShooterSpinup()
      ),

      new ShootSpeaker()
    );

 */