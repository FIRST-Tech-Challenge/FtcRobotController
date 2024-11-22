package org.firstinspires.ftc.teamcode.CommandGroups;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Commands.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Subsystems.SlideTargetHeight;

import java.util.ArrayList;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class LeftSideAuto87Pts extends SequentialCommandGroup {

    // constructor
    public LeftSideAuto87Pts() {
        // start pos (0.25, 1.6, -90) on field
        addCommands (
                // sets the starting position
                new InstantCommand(() -> RobotContainer.odometry.setCurrentPos(new Pose2d(0.25, 1.6, new Rotation2d(Math.toRadians(-90))))),
                //makes sure the claw is closed
                new CloseClaw(),

                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 1.0, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                //Place specimen
                new PlaceSpecimenAddOffset(),

                // pickup from submersibule
                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 1.15, new Rotation2d(Math.toRadians(90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                new HuntingPos(),

                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(-90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 0.8, new Rotation2d(Math.toRadians(-90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                // use camera to orient to sample
                // claw will close using touch sensor when it touches a sample

                new DropToGrab(),

                new Pause(0.5),

                new CloseClaw(),

                new Pause(0.5),

                new HuntingPos(),

                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        new Rotation2d(Math.toRadians(90.0)),
                        new ArrayList<Translation2d>() {{ }},
                        new Pose2d(0.25, 1.15, new Rotation2d(Math.toRadians(90.0))),
                        new Rotation2d(Math.toRadians(-90))),

                new ArmStowHigh(),

                new Pause(0.5),
                //  place in high bucket
                new BlueSideHighBucketDeposit(),

                // pickup all three and cycling in high bucket
                new GroundCyclingAuto()



        );


    }

}

