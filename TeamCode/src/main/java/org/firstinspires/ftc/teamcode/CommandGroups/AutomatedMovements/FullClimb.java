package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.ClimbTargetHeight;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;
import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class FullClimb extends SequentialCommandGroup {

    // constructor
    public FullClimb() {

        addCommands (

                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(180))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(0.45, 0.2, new Rotation2d(Math.toRadians(180.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(0.0)))),

                new Pause(1),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_LOW, true)),

                new Pause(1),

                new InstantCommand(()-> RobotContainer.climb.moveTo(ClimbTargetHeight.SAMPLE_LIFT)),

                new Pause(2),

                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_ZERO, true)),

                new Pause(1),

                new InstantCommand(()-> RobotContainer.climb.moveTo(ClimbTargetHeight.SAMPLE_CLIMB_ZERO)),

                new Pause(2)

        );
    }

}