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
                        commandFactory.driveToTarget(450, 250, -45, 0.18),
                        commandFactory.pivotToDelivery(),
                        commandFactory.elbowToSpecimenPosition(),
                        commandFactory.extandSlider(),
                        commandFactory.driveToTarget(280, 420, -45, 0.1),
                        commandFactory.waitFor(300),
                        commandFactory.extandSlider(),
                        commandFactory.outtake(),
                        commandFactory.driveToTarget(450, 250, -45, 0.1),

                        commandFactory.collapseSlider(),
                        commandFactory.pivotToInTake(),

                        commandFactory.driveToTarget(450, 250, 0, 0.1),

                        commandFactory.pivotToStart()

//                        commandFactory.elbowToSpecimenPosition(),
//                        commandFactory.waitFor(500),
//                        commandFactory.extandSlider(),
//                        commandFactory.outtake(),
//                        commandFactory.collapseSlider()
                )
        );
    }
}
