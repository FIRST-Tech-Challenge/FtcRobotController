package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
public class OdoTest extends CommandAutoOpMode {

    boolean hold1End = false;
    boolean hold2End = false;
    boolean hold3End = false;
    @Override
    protected Command createCommand() {
        return new ParallelCommandGroup(
                //commandFactory.WriteTelemetry(),
                new SequentialCommandGroup(
                        //commandFactory.driveToTarget(200, 0, 0, 0.11),
                        new ParallelCommandGroup(
                             commandFactory.driveToTarget(300, 400, -45, .23),
                             commandFactory.pivotToDelivery(),
                             commandFactory.elbowToSpecimenPosition()
                        ),
                        commandFactory.extendSlider(),
                        commandFactory.driveToTarget(180, 520, -45, 0.11),
                        commandFactory.extendSlider(() -> hold1End),

                        commandFactory.outtake().andThen(new InstantCommand(() -> hold1End = true)),

                        new ParallelCommandGroup(
                        commandFactory.driveToTarget(500, 300, 0, 0.15),
                                        commandFactory.collapseSlider()
                        ),

                        new ParallelCommandGroup(
                            commandFactory.pivotToGroundInTakeBegin(),
                            commandFactory.elbowToIntakePosition()
                        ),

                        commandFactory.intakeFromGround(),

                        new ParallelCommandGroup(
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.pivotToDelivery()
                                ),

                        commandFactory.extendSlider(),
                        commandFactory.driveToTarget(180, 520, -45, 0.15),
                        commandFactory.extendSlider(() -> hold2End),

                        commandFactory.outtake().andThen(new InstantCommand(() -> hold2End = true)),

                        new ParallelCommandGroup(
                        commandFactory.driveToTarget(500, 590, 0, 0.12),
                                        commandFactory.collapseSlider()
                        ),

                        new ParallelCommandGroup(
                                commandFactory.pivotToGroundInTakeBegin(),
                                commandFactory.elbowToIntakePosition()
                        ),

                        commandFactory.intakeFromGround(),

                        new ParallelCommandGroup(
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.pivotToDelivery()
                        ),

                        commandFactory.extendSlider(),
                        commandFactory.driveToTarget(180, 520, -45, 0.11),
                        commandFactory.extendSlider(() -> hold3End),

                        commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),

                        new ParallelCommandGroup(
                            commandFactory.driveToTarget(600, 500, 45, 0.12),
                                commandFactory.collapseSlider()
                        ),

                        new ParallelCommandGroup(
                                commandFactory.pivotToGroundInTakeBegin(),
                                commandFactory.elbowToIntakePosition()
                        ),

                        commandFactory.intakeFromGround(),

                        new ParallelCommandGroup(
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.pivotToDelivery()
                        ),

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
