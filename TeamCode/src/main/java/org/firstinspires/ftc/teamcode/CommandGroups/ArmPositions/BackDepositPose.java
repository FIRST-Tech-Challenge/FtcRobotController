package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class BackDepositPose extends SequentialCommandGroup {

    // constructor
    public BackDepositPose() {

        addCommands (

                // lifts the shoulder up 45 degrees
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(45)),

                // folds the elbow in 60 degrees
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(198)),

                // folds the wrist in 135 degrees
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(180)),

                // folds the wrist in 135 degrees
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180))


        );
    }

}