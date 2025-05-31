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
    public double tolerance = 0.2;

    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                //Make an 8
                commandFactory.driveToTarget(forward_distance, 0, 0, 0.05, 1,30),
                commandFactory.driveToTarget(0, side_distance, 0, 0.05, 1,30),
                commandFactory.driveToTarget(forward_distance, side_distance, 0, 0.05, 1,30),
                commandFactory.driveToTarget(0, 0, 0, 0.05, 1,tolerance),
                //Long Vertical Strafe
                commandFactory.driveToTarget(forward_distance*2,side_distance,0,0.05,1,30),
                commandFactory.driveToTarget(0,0,0,0.05,1,tolerance)

        );
    }
}

