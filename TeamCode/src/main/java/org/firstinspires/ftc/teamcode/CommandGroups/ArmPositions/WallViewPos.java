package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class WallViewPos extends SequentialCommandGroup {

    // constructor
    public WallViewPos() {

        addCommands (
                // lifts the shoulder up 90+-60 degrees
                // lifts the shoulder up to 135 degrees
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(43)),

                // folds the elbow in 270
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(266)),

                // folds the wrist in 0
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(100)),

                // powers the wrist and moves it to straight position
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(0))

        // new command1
        // new command2
        // new command3
        );
    }

}