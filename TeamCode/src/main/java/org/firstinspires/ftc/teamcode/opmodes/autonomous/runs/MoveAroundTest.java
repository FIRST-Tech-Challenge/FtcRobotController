package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
@Config
public class MoveAroundTest extends CommandAutoOpMode {

    public double forward_distance = 400;
    public double side_distance = 400;
    public double turn1 =-90;

    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                commandFactory.driveToTarget(forward_distance, 0, 0, .2),
                commandFactory.driveToTarget(forward_distance, side_distance, 0, .2),
                commandFactory.driveToTarget(forward_distance, side_distance, turn1, 0, .2),
                commandFactory.driveToTarget(forward_distance, 0, turn1, 0, .2),
                commandFactory.driveToTarget(forward_distance, 0, 0, 0, .2),
                commandFactory.driveToTarget(0, 0, 0, 0, .2)
        );
    }
}
