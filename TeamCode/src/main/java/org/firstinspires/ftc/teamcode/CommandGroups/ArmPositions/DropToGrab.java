package org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.RobotContainer;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class DropToGrab extends SequentialCommandGroup {

    // constructor
    public DropToGrab() {
        addCommands(

                // same as in hunting pos moving wrist 45 degrees
                new InstantCommand(() -> RobotContainer.flappyFlappyWrist.RotateTo(45)),

                new Pause(0.3),

                // drops the elbow to 175 degrees for pick up
                new InstantCommand(() ->RobotContainer.elbowJoint.RotateTo(175)),

                // moves shoulder to 165 degrees so slightly down from hunting pos
                new InstantCommand(() -> RobotContainer.shoulderJoint.RotateTo(158))

                // same as in hunting pos going to 135 degrees straight
                //new InstantCommand(() -> RobotContainer.wristRotateServo.RotateTo(180))



        );


        // new command1
        // new command2
        // new command3

    }

}