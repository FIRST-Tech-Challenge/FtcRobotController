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
                        new Pose2d(1.53, 1.05, new Rotation2d(Math.toRadians(-90))),
                        new Rotation2d(Math.toRadians(-90.0))),

                new Pause(0.5),

                new HuntingPos(),

               // new Pause(0.5),

                new DropToGrab(),

                //new Pause(0.5),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new BlueSideHighBucketDeposit(),

                // pick up right
                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-135)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(1.27, 1.05, new Rotation2d(Math.toRadians(-90))),
                        new Rotation2d(Math.toRadians(-90.0))),

                //new Pause(0.5),

                new HuntingPos(),

                //new Pause(0.5),

                new DropToGrab(),

                //new Pause(0.5),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new BlueSideHighBucketDeposit(),

              //pick up left
              new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-135)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(1.58, 0.27, new Rotation2d(Math.toRadians(0))),
                        new Rotation2d(Math.toRadians(70))),

                new DropToGrab(),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                new Pause(1.0),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new BlueSideHighBucketDeposit()


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

