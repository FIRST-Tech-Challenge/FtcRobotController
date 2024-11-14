package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous(name = "Test Command auton")
public class CommandAutoOpModeTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                commandFactory.turnAngleAbsolute(90),
                commandFactory.turnAngleRelative(45),
                commandFactory.turnAngleAbsolute(0),
                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.extandSlider()
                )
        );
    }
}
