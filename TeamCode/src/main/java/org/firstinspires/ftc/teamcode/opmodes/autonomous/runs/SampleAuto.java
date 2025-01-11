package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.LogCatCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PrintCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.command.CommandFactory;

@Autonomous
public class SampleAuto extends CommandAutoOpMode {

    boolean hold1End = false;
    boolean hold2End = false;
    boolean hold3End = false;
    @Override
    protected Command createCommand() {
        return new SequentialCommandGroup(


                //region preloaded sample
                // drive to bucket, pivot the arm and extend slider
                new ParallelCommandGroup(
                    commandFactory.driveToTarget(300, 400, -45, .13, .7, 100),
                    commandFactory.pivotToDelivery(),
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.extendSlider()
                ),

                // ?
                commandFactory.driveToTarget(10, 420, -45, 0.13, .8, 50),

                // deliver preloaded sample
                commandFactory.outtake().andThen(new InstantCommand(() -> hold1End = true)),
                //endregion

                //region sample #1
                commandFactory.driveToTarget(480, 270, 0, 0.13, .7, 5),

                new ParallelCommandGroup(
                    commandFactory.collapseSlider(),
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
                ),

                commandFactory.intakeFromGround2(1500),
                commandFactory.inCaseSampleIntakeFailed("Sample 1", new SequentialCommandGroup(
                    commandFactory.pivotToIntakeRetry(),
                        commandFactory.pivotToGroundInTakeBegin(),
                        commandFactory.intakeFromGround2(2000)
                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()
                ),

                commandFactory.extendSlider(),

                commandFactory.driveToTarget(20, 440, -45, 0.13, .5, 10),

                // Sample #1
                commandFactory.outtake().andThen(new InstantCommand(() -> hold2End = true),
                //endregion for sample #1

                //region sample #2
                commandFactory.driveToTarget(480, 520, 0, 0.13, .5, 5)),

                new ParallelCommandGroup(
                    commandFactory.collapseSlider(),
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
                ),

                commandFactory.intakeFromGround2(1500),

                commandFactory.inCaseSampleIntakeFailed("Sample 2", new SequentialCommandGroup(
                        commandFactory.pivotToIntakeRetry(),
                        commandFactory.pivotToGroundInTakeBegin(),
                        commandFactory.intakeFromGround2(2000)
                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()
                ),

                commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 10),
                commandFactory.extendSlider(),
                commandFactory.driveToTarget(30, 460, -45, 0.13, .8, 10),


                // Sample #2
                commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
                //endregion sample #2

                //region sample #3
//                commandFactory.driveToTarget(230, 550, 15, 0.13, .5, 5),
//                commandFactory.collapseSlider(),
//                new ParallelCommandGroup(
//                    commandFactory.pivotToGroundInTakeBegin()
//
//                ),
//                commandFactory.elbowToIntakePositionForSample3(),
//                commandFactory.extendSliderToIntakeSample3(),
//                commandFactory.sleep(300),
//                commandFactory.intakeFromGroundForSample3(4000),

//                commandFactory.inCaseSampleIntakeFailed("Sample 3", new SequentialCommandGroup(
//                        commandFactory.pivotToIntakeRetry(),
//                        commandFactory.pivotToGroundInTakeBegin(),
//                        commandFactory.intakeFromGroundForSample3(6000)
//                )),

//                new ParallelCommandGroup(
//                    commandFactory.elbowToSpecimenPosition(),
//                    commandFactory.pivotToDelivery()
//
//                ),
//
//                commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 10),
//                commandFactory.extendSlider(),
//                commandFactory.driveToTarget(80, 440, -45, 0.13, .8, 10),
//
//
//                // Sample #3
//                commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
//                commandFactory.driveToTarget(300, 420, 0, 0.13, .8, 10),
                //endregion sample #3

//                new ParallelCommandGroup(
//                    commandFactory.pivotToStart(),
//                    commandFactory.collapseSlider(),
//                    commandFactory.driveToTarget(2600, -290, -120, .13, 1, 10),
//                    commandFactory.elbowToStartPosition()
//                )

                new ParallelCommandGroup(
                        commandFactory.pivotToStart(),
                        commandFactory.sleep(300).andThen(commandFactory.collapseSlider()),
                        commandFactory.driveToTarget(2300, 0, -90, .5, 1, 10),
                        commandFactory.elbowToStartPosition()
                ),

                commandFactory.driveToTarget(2100, -350, -90, .2, .5, 10)
                        .alongWith(commandFactory.elbowToIntakePosition()).andThen(commandFactory.pivotToInTake()
//                commandFactory.driveToTarget(2000, -120, -90, .13, 1, 10)

        ));
    }
}
