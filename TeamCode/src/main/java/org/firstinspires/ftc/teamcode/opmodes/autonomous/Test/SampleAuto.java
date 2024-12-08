package org.firstinspires.ftc.teamcode.opmodes.autonomous.Test;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
public class SampleAuto extends CommandAutoOpMode {

    boolean hold1End = false;
    boolean hold2End = false;
    boolean hold3End = false;
    @Override
    protected Command createCommand() {
        return new ParallelCommandGroup(


                //commandFactory.WriteTelemetry(),
                new SequentialCommandGroup(


                        new ParallelCommandGroup(
                             commandFactory.driveToTarget(300, 400, -45, .1, 100),
                             commandFactory.pivotToDelivery(),
                             commandFactory.elbowToSpecimenPosition(),
                                commandFactory.extendSlider()


                        ),

                        commandFactory.driveToTarget(160, 520, -45, 0.1, 1, 50),
                        commandFactory.extendSlider(),

                        // Sample #1
                        commandFactory.outtake().andThen(new InstantCommand(() -> hold1End = true)),

                        commandFactory.driveToTarget(430, 320, 0, 0.13, 1, 20),

                        new ParallelCommandGroup(
                                commandFactory.collapseSlider(),

                            commandFactory.pivotToGroundInTakeBegin(),
                            commandFactory.elbowToIntakePosition()
                        ),

                        commandFactory.intakeFromGround(),

                        new ParallelCommandGroup(
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.pivotToDelivery()

                                ),

                        commandFactory.extendSlider(),


                        commandFactory.driveToTarget(180, 520, -45, 0.1, 1, 100),
                        commandFactory.extendSlider(),

                        // Sample #2
                        commandFactory.outtake().andThen(new InstantCommand(() -> hold2End = true),

                        commandFactory.driveToTarget(400, 610, 0, 0.1, 1, 30)),


                        new ParallelCommandGroup(
                                commandFactory.collapseSlider(),
                                commandFactory.pivotToGroundInTakeBegin(),
                                commandFactory.elbowToIntakePosition()
                        ),

                        commandFactory.intakeFromGround(),

                        new ParallelCommandGroup(
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.pivotToDelivery()
                        ),

                        commandFactory.extendSlider(),
                        commandFactory.driveToTarget(180, 520, -45, 0.1, 1, 100),
                        commandFactory.extendSlider(),


                        // Sample #3
                        commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),

                        commandFactory.driveToTarget(440, 540, 35, 0.1, 1, 30),

                        commandFactory.collapseSlider(),

                        new ParallelCommandGroup(
                                commandFactory.pivotToGroundInTakeBegin(),
                                commandFactory.elbowToIntakePosition()
                        ),

                        commandFactory.intakeFromGround(),

                        commandFactory.pivotToStart()

//                        new ParallelCommandGroup(
//                                commandFactory.elbowToSpecimenPosition(),
//                                commandFactory.pivotToDelivery()
//
//                        ),
//
//                        commandFactory.extendSlider(),
//
//                        commandFactory.driveToTarget(180, 520, -45, 0.1, 1, 150),
//                        commandFactory.extendSlider(),
//
//                        // Sample #3
//                        commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
//
//
//                        new ParallelCommandGroup(
//                                commandFactory.pivotToStart()
//                    )
                )
        );
    }
}
