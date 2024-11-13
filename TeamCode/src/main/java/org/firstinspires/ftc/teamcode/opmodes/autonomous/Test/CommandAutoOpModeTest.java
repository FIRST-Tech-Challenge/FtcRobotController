package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Test Command auton")
public class CommandAutoOpModeTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                commandFactory.driveToTarget(0, -400),
                commandFactory.turnAngleRelative(90)
        );
    }
}
