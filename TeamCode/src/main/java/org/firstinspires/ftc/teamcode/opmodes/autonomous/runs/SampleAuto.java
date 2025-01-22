package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;

@Autonomous
public class SampleAuto extends CommandAutoOpMode {

    boolean hold1End = false;
    boolean hold2End = false;
    boolean hold3End = false;

    boolean inMatch = false;

    @Config
    public static class Sample3Config {
        public static int beforeExtendSlider_tragetX = 230;
        public static int beforeExtendSlider_tragetY = 550;
        public static int beforeExtendSlider_heading = 15;
        public static double beforeExtendSlider_minPower = 0.13;
        public static double beforeExtendSlider_maxPower = .5;
        public static int beforeExtendSlider_distanceTolerance = 5;
        public static double elbowIntakePosition = .85;
        public static int sliderIntakePosition = 450;
        public static int pivotIntakePosition = -1300;
    }

    private boolean tryGetDevBoolean(boolean val) {
        return inMatch ? false : val;
    }

    @Override
    protected Command createCommand() {
        boolean skipOuttakePreloaded = tryGetDevBoolean(true);
        boolean skipIntakeSample1 = tryGetDevBoolean(true);
        boolean skipIntakeSample2 = tryGetDevBoolean(true);

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
                commandFactory.driveToTarget(-10, 430, -45, 0.15, .8, 50),


                // deliver preloaded sample
                skipOuttakePreloaded ? commandFactory.doNothing() : commandFactory.outtake().andThen(new InstantCommand(() -> hold1End = true)),
                //endregion

                //region sample #1
                commandFactory.driveToTarget(510, 280, 0, 0.13, .7, 5),

                new ParallelCommandGroup(
                    commandFactory.collapseSlider(),
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
                ),

                skipIntakeSample1 ? commandFactory.doNothing() : commandFactory.intakeFromGround2(1500),
                skipIntakeSample1 ? commandFactory.doNothing() : commandFactory.inCaseSampleIntakeFailed("Sample 1", new SequentialCommandGroup(
                    commandFactory.pivotToIntakeRetry(),
                        commandFactory.pivotToGroundInTakeBegin(),
                        commandFactory.intakeFromGround2(2000)
                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()
                ),

                commandFactory.extendSlider(),

                commandFactory.driveToTarget(10, 450, -45, 0.13, .5, 10),

                // Sample #1
                skipIntakeSample1 ? commandFactory.doNothing() : commandFactory.outtake().andThen(new InstantCommand(() -> hold2End = true),
                //endregion for sample #1

                //region sample #2
                commandFactory.driveToTarget(490, 520, 0, 0.13, .5, 5)),

                new ParallelCommandGroup(
                    commandFactory.collapseSlider(),
                    commandFactory.pivotToGroundInTakeBegin(),
                    commandFactory.elbowToIntakePosition()
                ),

                skipIntakeSample2 ? commandFactory.doNothing() : commandFactory.intakeFromGround2(1500),

                skipIntakeSample2 ? commandFactory.doNothing() : commandFactory.inCaseSampleIntakeFailed("Sample 2", new SequentialCommandGroup(
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
                commandFactory.driveToTarget(5, 475, -45, 0.13, .8, 10),


                // Sample #2
                skipIntakeSample2 ? commandFactory.doNothing() : commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
                //endregion sample #2

                //region sample #3
                commandFactory.driveToTarget(Sample3Config.beforeExtendSlider_tragetX, Sample3Config.beforeExtendSlider_tragetY, Sample3Config.beforeExtendSlider_heading, Sample3Config.beforeExtendSlider_minPower, Sample3Config.beforeExtendSlider_maxPower, Sample3Config.beforeExtendSlider_distanceTolerance),
                commandFactory.collapseSlider(),
                new ParallelCommandGroup(
                    commandFactory.pivotTo(Sample3Config.pivotIntakePosition)

                ),
                commandFactory.elbowToPosition(Sample3Config.elbowIntakePosition),
                commandFactory.extendSlider(Sample3Config.sliderIntakePosition),
                commandFactory.sleep(300),
                commandFactory.intakeFromGroundForSample3(4000),

//                commandFactory.inCaseSampleIntakeFailed("Sample 3", new SequentialCommandGroup(
//                        commandFactory.pivotToIntakeRetry(),
//                        commandFactory.pivotToGroundInTakeBegin(),
//                        commandFactory.intakeFromGroundForSample3(6000)
//                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()

                ),

                commandFactory.driveToTarget(300, 400, 0, 0.13, .8, 10),
                commandFactory.extendSlider(),
                commandFactory.driveToTarget(80, 440, -45, 0.13, .8, 10),


                // Sample #3
                commandFactory.outtake().andThen(new InstantCommand(() -> hold3End = true)),
                commandFactory.driveToTarget(300, 420, 0, 0.13, .8, 10),
                //endregion sample #3

//                new ParallelCommandGroup(
//                    commandFactory.pivotToStart(),
//                    commandFactory.collapseSlider(),
//                    commandFactory.driveToTarget(2600, -290, -120, .13, 1, 10),
//                    commandFactory.elbowToStartPosition()
//                )

                new ParallelCommandGroup(
                        commandFactory.sleep(300).andThen(commandFactory.collapseSlider()).andThen(commandFactory.pivotToStart()),
                        commandFactory.driveToTarget(2300, 0, -90, .5, 1, 10),
                        commandFactory.elbowToStartPosition()
                ),

                commandFactory.driveToTarget(2100, -350, -90, .2, .5, 10)
                        //.alongWith(commandFactory.elbowToIntakePosition()).andThen(commandFactory.pivotToInTake()
//                commandFactory.driveToTarget(2000, -120, -90, .13, 1, 10)

        );
    }
}
