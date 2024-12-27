package org.firstinspires.ftc.teamcode.CommandGroups.AutomatedMovements;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.DropToGrab;
import org.firstinspires.ftc.teamcode.CommandGroups.ArmPositions.HuntingPos;
import org.firstinspires.ftc.teamcode.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Commands.Pause;
import org.firstinspires.ftc.teamcode.Commands.Claw.WaitForClawButton;

// Example Sequential Command Group
// There are also:
// ParallelCommandGroup
// ParallelRaceGroup
// ParallelDeadlineGroup

public class PickupFromSubmersible extends SequentialCommandGroup {

    // constructor
    public PickupFromSubmersible() {

        addCommands (

                new OpenClaw(),

                new Pause(0.25),

                new DropToGrab(),

                new WaitForClawButton(),

                new CloseClaw(),

                new Pause(0.25),

                new HuntingPos()

        );
    }

}