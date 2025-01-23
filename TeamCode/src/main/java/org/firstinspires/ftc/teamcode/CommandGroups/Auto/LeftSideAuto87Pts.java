package org.firstinspires.ftc.teamcode.CommandGroups.Auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.HighBucketDeposit;
import org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements.GroundCyclingAuto;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Drive.FollowPath;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.Subsystems.LinearSlide.SlideTargetHeight;
import org.firstinspires.ftc.teamcode.utility.AutoFunctions;
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
                new InstantCommand(() -> RobotContainer.odometry.setCurrentPos(AutoFunctions.redVsBlue(new Pose2d(0.8, 1.57, new Rotation2d(Math.toRadians(-90)))))),

                // makes sure the claw is closed
                new CloseClaw(),

                // put arm into stow position
                // powers shoulder
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(122)),
                // folds the elbow in 225
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(270)),
                // folds the wrist in 45
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(0)),

                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180)),

                // move slide up to get ready for drop off
                new InstantCommand(()-> RobotContainer.linearSlide.moveTo(SlideTargetHeight.SAMPLE_HIGH)),


                // drop off first sample into bucket
                new HighBucketDeposit(),

                // pickup all three and cycling in high bucket
                new GroundCyclingAuto(),


                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(132)),

                // folds the elbow in 270
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(250)),

                // folds the wrist in 0
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(0)),

                // powers the wrist and moves it to straight position
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180)),

                new OpenClaw(),
                // move to park position

                new FollowPath(
                        2.0,
                        1.0,
                        0.0,
                        0.0,
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-90.0))),
                        new ArrayList<Translation2d>() {{ }},
                        AutoFunctions.redVsBlue(new Pose2d(0.56, 0.25, new Rotation2d(Math.toRadians(-180)))),
                        AutoFunctions.redVsBlue(new Rotation2d(Math.toRadians(-180))))

        );

    }

}