package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.DropToGrab;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.HuntingPos;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Drive.MoveToPose;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.SlideTargetHeight;
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
                new MoveToPose(
                        2.0,
                        0.7,
                        AutoFunctions.redVsBlue(new Pose2d(1.49, 1.02, new Rotation2d(Math.toRadians(-90))))
                ),

                new HuntingPos(),

                new DropToGrab(),
                // this timer is for debounce
                new Pause(0.5),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new HighBucketDeposit(),

                // pick up right
                new MoveToPose(
                        2.0,
                        0.7,
                        AutoFunctions.redVsBlue(new Pose2d(1.25, 1.02, new Rotation2d(Math.toRadians(-90))))
                ),

                new HuntingPos(),

                new DropToGrab(),

                new Pause(0.5),

                new CloseClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                // raise elevator to be ready to drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                new HighBucketDeposit(),

                // pick up left
                new FollowPath(
                        2.0,
                        0.7,
                        0.0,
                        0.0,
                      AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90))),
                      new ArrayList<Translation2d>() {{ }},
                      AutoFunctions.redVsBlue(new Pose2d(1.55, 0.99, new Rotation2d(Math.toRadians(-64)))),
                      AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-64)))),

                new HuntingPos(),

                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(5)),

                new DropToGrab(),

                new Pause(0.5),

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