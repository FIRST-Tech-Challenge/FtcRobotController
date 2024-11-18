package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
public class OdoTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new ParallelCommandGroup(
                //commandFactory.WriteTelemetry(),
                new SequentialCommandGroup(
                        commandFactory.driveToTarget(200, 0, 0, 0.1),
                        //commandFactory.driveToTarget(400, 0, -90),
                        //commandFactory.driveToTarget(400, 400, -90, 0.1),
                        commandFactory.driveToTarget(400, 400, -45, 0.2)
                        //commandFactory.MotorTest()
                )
        );
    }
}
