package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

//@Autonomou
public class RobotServoTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new ParallelCommandGroup(
                //commandFactory.WriteTelemetry(),
                new SequentialCommandGroup(
                    commandFactory.alignToSample()
                )
        );
    }
}
