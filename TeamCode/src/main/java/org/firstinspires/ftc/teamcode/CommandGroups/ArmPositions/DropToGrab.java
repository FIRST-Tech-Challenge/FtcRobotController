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

                // intermediate position to ease impact on ground (and on the piece)
                new InstantCommand(() -> RobotContainer.elbowJoint.RotateTo(150)),

                // moves shoulder to 158 degrees so slightly down from hunting pos
                new InstantCommand(() -> RobotContainer.shoulderJoint.RotateTo(127)),

                // Was pause of 0.3.  Raised it with the higher drop to grab.
                new Pause(0.3),

                // drops the elbow to 175 degrees for pick up
                new InstantCommand(() -> RobotContainer.shoulderJoint.RotateTo(144)),

                // intermediate position to ease impact on ground (and on the piece)
                new InstantCommand(() -> RobotContainer.elbowJoint.RotateTo(145))

        );


        // new command1
        // new command2
        // new command3

    }

}