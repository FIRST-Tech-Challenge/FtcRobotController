package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class StartingArmStowHigh extends SequentialCommandGroup {

    // constructor
    public StartingArmStowHigh() {

        addCommands (
        // What this position should do is give the camera a good vantage point as well as keep the arm out of the way

                //powers shoulder
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(62)),
                // folds the elbow in 270
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(270)),

                new Pause(1.5),
                // lifts the shoulder up 90+-60 degrees
                // lifts the shoulder up to 135 degrees
                new InstantCommand(() ->RobotContainer.shoulderJoint.RotateTo(130)),

                // folds the wrist in 0
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(0)),

                // powers the wrist and moves it to straight position
                new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180)),

                new CloseClaw()



        );
    }

}