package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

//@Autonomous(name = "Test Command auton")
public class CommandAutoOpModeTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                commandFactory.driveToTargetAlternate(100, 0, 0 ),
                commandFactory.driveToTargetAlternate(100, 350, 45 ),
                commandFactory.driveToTargetAlternate(50, 350, 45 ),
                commandFactory.driveToTargetAlternate(90, 500, 0 ),
                commandFactory.driveToTargetAlternate(50, 350, 45 ),
                commandFactory.driveToTargetAlternate(90, 570, 0 )
        );
    }
}
