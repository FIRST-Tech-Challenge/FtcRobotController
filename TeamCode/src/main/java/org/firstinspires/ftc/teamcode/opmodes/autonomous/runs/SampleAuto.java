package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.acmerobotics.dashboard.config.Config;
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

    boolean inMatch = true;

    @Config
    public static class Sample3Config {

        public static int beforeIntakeDrive_targetX = 230;
        public static int beforeIntakeDrive_targetY = 550;
        public static int beforeIntakeDrive_heading = 15;
        public static double beforeIntakeDrive_minPower = 0.13;
        public static double beforeIntakeDrive_maxPower = .5;
        public static int beforeIntakeDrive_distanceTolerance = 5;
        public static double elbowIntakePosition = .75;
        public static int sliderIntakePosition = 300;
        public static int pivotIntakePosition = -1400;
        public static int pivotToGroundPosition = -2200;
        public static int IntakeFromGroundTimeout = 5000;


        public static int ToDeliveryDrive_TargetX = 300;
        public static int ToDeliveryDrive_TargetY = 400;
        public static int ToDeliveryDrive_TargetHeading = 0;
        public static double ToDeliveryDrive_MinPower = 0.13;
        public static double ToDeliveryDrive_MaxPower = .8;
        public static int ToDeliveryDrive_DistanceTolerance = 10;
        public static int AfterDeliveryExtendDrive_TargetX = 80;
        public static int AfterDeliveryExtendDrive_TargetY = 440;
        public static int AfterDeliveryExtendDrive_TargetHeading = -45;
        public static double AfterDeliveryExtendDrive_MinPower = 0.13;
        public static double AfterDeliveryExtendDrive_MaxPower = .8;
        public static int AfterDeliveryExtendDrive_DistanceTolerance = 10;
    }

    private boolean tryGetDevBoolean(boolean val) {
        return inMatch ? false : val;
    }

    @Override
    protected Command createCommand() {
        boolean skipOuttakePreloaded = tryGetDevBoolean(true);
        boolean skipIntakeSample1 = tryGetDevBoolean(true);
        boolean skipIntakeSample2 = tryGetDevBoolean(true);

        telemetry.addData("skip out take preloaded", skipOuttakePreloaded);
        telemetry.addData("skip intake sample 1", skipIntakeSample1);
        telemetry.addData("skip intake sample 2", skipIntakeSample2);

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
                commandFactory.driveToTarget(Sample3Config.beforeIntakeDrive_targetX, Sample3Config.beforeIntakeDrive_targetY, Sample3Config.beforeIntakeDrive_heading, Sample3Config.beforeIntakeDrive_minPower, Sample3Config.beforeIntakeDrive_maxPower, Sample3Config.beforeIntakeDrive_distanceTolerance),
                commandFactory.collapseSlider(),
                commandFactory.pivotTo(Sample3Config.pivotIntakePosition),
                commandFactory.elbowToPosition(Sample3Config.elbowIntakePosition),
                commandFactory.extendSlider(Sample3Config.sliderIntakePosition),
                commandFactory.sleep(300),
                commandFactory.intakeFromGround(Sample3Config.pivotToGroundPosition, Sample3Config.IntakeFromGroundTimeout),

//                commandFactory.inCaseSampleIntakeFailed("Sample 3", new SequentialCommandGroup(
//                        commandFactory.pivotToIntakeRetry(),
//                        commandFactory.pivotToGroundInTakeBegin(),
//                        commandFactory.intakeFromGroundForSample3(6000)
//                )),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()

                ),

                commandFactory.driveToTarget(Sample3Config.ToDeliveryDrive_TargetX, Sample3Config.ToDeliveryDrive_TargetY, Sample3Config.ToDeliveryDrive_TargetHeading, Sample3Config.ToDeliveryDrive_MinPower, Sample3Config.ToDeliveryDrive_MaxPower, Sample3Config.ToDeliveryDrive_DistanceTolerance),
                commandFactory.extendSlider(),
                commandFactory.driveToTarget(Sample3Config.AfterDeliveryExtendDrive_TargetX, Sample3Config.AfterDeliveryExtendDrive_TargetY, Sample3Config.AfterDeliveryExtendDrive_TargetHeading, Sample3Config.AfterDeliveryExtendDrive_MinPower, Sample3Config.AfterDeliveryExtendDrive_MaxPower, Sample3Config.AfterDeliveryExtendDrive_DistanceTolerance),


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
