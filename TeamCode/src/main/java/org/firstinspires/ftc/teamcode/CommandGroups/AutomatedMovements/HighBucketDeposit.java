package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.BackDepositPose;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
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

public class HighBucketDeposit extends SequentialCommandGroup {

    // constructor
    public HighBucketDeposit() {

        addCommands (

                //new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

//                new FollowPath(
//                        2.0,
//                        1.0,
//                        0.0,
//                        0.0,
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-135))),
//                        new ArrayList<Translation2d>() {{ }},
//                        AutoFunctions.redVsBlue(new Pose2d(1.43, 1.39, new Rotation2d(Math.toRadians(45)))),
//                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-135)))),

                new MoveToPose(
                        2.0,
                        1.0,
                        AutoFunctions.redVsBlue(new Pose2d(1.43, 1.39, new Rotation2d(Math.toRadians(-135))))
                ),

                //new Pause(3),

                //new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

               // new Pause(0.3),

                new BackDepositPose(),

                new Pause(1.0),

                new OpenClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO))
        );
    }

}