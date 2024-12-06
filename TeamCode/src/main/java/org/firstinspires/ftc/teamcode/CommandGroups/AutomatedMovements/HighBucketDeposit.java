package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.ArmStowHigh;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.BackDepositePose;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.OpenClaw;
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

public class HighBucketDeposit extends SequentialCommandGroup {

    // constructor
    public HighBucketDeposit() {

        addCommands (

                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(1.46, 1.40, new Rotation2d(Math.toRadians(45)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-135)))),

                //new Pause(1),

                //new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),

                //new Pause(1),

                new BackDepositePose(),

                new Pause(0.85),

                new OpenClaw(),

                new Pause(0.25),

                new ArmStowHigh(),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO))
        );
    }

}
