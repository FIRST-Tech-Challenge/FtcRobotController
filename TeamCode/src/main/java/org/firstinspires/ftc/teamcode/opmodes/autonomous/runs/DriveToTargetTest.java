package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

public class DriveToTargetTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                commandFactory.driveToTarget(500, 0, 0, .2),
                commandFactory.driveToTarget(500, 500, 0, .2)
        );
    }
}
