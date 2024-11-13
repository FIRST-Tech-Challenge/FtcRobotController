package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;
import org.firstinspires.ftc.teamcode.util.Units;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Test Command auton")
public class CommandAutoOpModeTest extends CommandAutoOpMode {
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        commandFactory.extandSlider(),
                        commandFactory.pivotToInTake()
                ),
//                commandFactory.driveToTarget(0, -400),
                commandFactory.waitFor(2000),
                new ParallelCommandGroup(
                        commandFactory.collapseSlider(),
                        commandFactory.pivotToDelivery()
                )
//                commandFactory.turnAngleRelative(90)
//                commandFactory.followTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
//                                new Pose2d(0, Units.inchesToMeters(10), Rotation2d.fromDegrees(90)))
//                        .andThen(commandFactory.stopDriveTrain())
        );
    }
}
