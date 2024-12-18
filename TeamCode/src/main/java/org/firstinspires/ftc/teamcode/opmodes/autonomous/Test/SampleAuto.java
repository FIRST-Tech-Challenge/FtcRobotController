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
        return new SequentialCommandGroup(

                new ParallelCommandGroup(
                commandFactory.driveToTarget(300, 400, -45, .13, .8, 100),
                commandFactory.pivotToDelivery(),
                commandFactory.elbowToSpecimenPosition(),
                        commandFactory.extendSlider()

                ),



                commandFactory.driveToTarget(30, 400, -45, 0.13, .8, 50),

                        // Sample #1
                        commandFactory.outtake().andThen(new InstantCommand(() -> hold1End = true)),

                        commandFactory.driveToTarget(470, 250, 0, 0.13, .7, 20),

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

                        commandFactory.driveToTarget(80, 400, -45, 0.13, .7, 100),
//
//                        // Sample #2
                        commandFactory.outtake().andThen(new InstantCommand(() -> hold2End = true),

                        commandFactory.driveToTarget(470, 500, 0, 0.13, .8, 30)),


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

                        commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 100),
                        commandFactory.extendSlider(),
                        commandFactory.driveToTarget(80, 440, -45, 0.13, .8, 50),


                        // Sample #3
                        commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),

                        commandFactory.driveToTarget(530, 490, 27, 0.13, .8, 30),

                        commandFactory.collapseSlider(),


                        new ParallelCommandGroup(
                                commandFactory.pivotToGroundInTakeBegin(),
                                commandFactory.elbowToIntakePosition()
                        ),

                        commandFactory.intakeFromGround(),

                        new ParallelCommandGroup(
                                commandFactory.elbowToSpecimenPosition(),
                                commandFactory.pivotToDelivery()

                        ),

                        commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 100),
                commandFactory.extendSlider(),
                commandFactory.driveToTarget(80, 440, -45, 0.13, .8, 50),


                // Sample #3
                        commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
                new ParallelCommandGroup(
                                commandFactory.pivotToStart(),
                        commandFactory.collapseSlider()
                )

        );
    }
}
