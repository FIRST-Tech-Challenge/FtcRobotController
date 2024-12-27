package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class Sweep1 extends SequentialCommandGroup {

    // constructor
    public Sweep1() {

        addCommands (
                //new InstantCommand(()-> RobotContainer.odometry.setCurrentPos(new Pose2d(-0.40,1.6,new Rotation2d(Math.toRadians(-90))))),

                //sweep 1

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(-0.85, 1.1, new Rotation2d(Math.toRadians(-90.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),


                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(-0.85, 0.3, new Rotation2d(Math.toRadians(-90.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),


                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(180.0))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(-1.15, 0.3, new Rotation2d(Math.toRadians(180.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0)))),

                new FollowPath(
                        1.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(90.0))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(-1.15, 1.35, new Rotation2d(Math.toRadians(90.0)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))))

        );
    }

}