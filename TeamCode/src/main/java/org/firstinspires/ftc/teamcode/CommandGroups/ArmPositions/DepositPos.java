package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class DepositPos extends SequentialCommandGroup {

    // constructor
    public DepositPos() {

        addCommands (

                // lifts the shoulder up 90+-60 degrees
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(120)),

                // folds the elbow in 10
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(200)),

                // folds the wrist in 10
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(70)),

                // folds the wrist in 10
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180))


        );
    }

}