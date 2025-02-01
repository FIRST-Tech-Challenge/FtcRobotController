package org.firstinspires.ftc.teamcode.opmodes.autonomous.runs;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.CommandAutoOpMode;

@Autonomous
@SuppressWarnings("unused")
public class SampleAuto extends CommandAutoOpMode {

    public static boolean inMatch = true;

    @Config
    public static class PreloadConfig {
        public static boolean skipOuttake = false;

        public static String someTextField = "placeholder";
        public static double toBucket_x = 200;
        public static double toBucket_y = 440;
        public static double toBucket_heading = -45;
        public static double toBucket_tolerance = 100;
        public static long toBucket_timeout = 1500;
    }

    @Config
    public static class Sample1Config {
        public static boolean skipIntake = false;

        public static int toSample_x = 530;
        public static int toSample_y = 250;
        public static int toSample_tolerance = 5;
        public static long toSample_timeout = 1000;
        public static int toBucket_x = -20;
        public static int toBucket_y = 450;
        public static int toBucket_heading = -45;
        public static int toBucket_tolerance = 10;
        public static long toBucket_timeout = 1500;
    }

    @Config
    public static class Sample2Config {
        public static boolean skipIntake = false;

        public static int toSample_x = 500;
        public static int toSample_y = 520;
        public static int toSample_heading = 0;
        public static int toSample_tolerance = 5;
        public static int toSample_timeout = 1500;

        public static int toBucket_x = -30;
        public static int toBucket_y = 450;
        public static int toBucket_heading = -45;
        public static int toBucket_tolerance = 10;
        public static int toBucket_timeout = 1500;
    }
    @Config
    public static class Sample3Config {
        public static boolean skipIntake = false;

        public static int beforeIntakeDrive_targetX =330;
        public static int beforeIntakeDrive_targetY = 530;
        public static int beforeIntakeDrive_heading = 15;
        public static int beforeIntakeDrive_distanceTolerance = 5;
        public static double elbowIntakePosition = .75;
        public static int sliderIntakePosition = 337;
        public static int pivotIntakePosition = -1400;
        public static int pivotToGroundPosition = -2200;
        public static int IntakeFromGroundTimeout = 5000;


        public static int ToDeliveryDrive_TargetX = -20;
        public static int ToDeliveryDrive_TargetY = 450;
        public static int ToDeliveryDrive_TargetHeading = -45;
        public static int ToDeliveryDrive_DistanceTolerance = 10;
    }

    @Config
    public static class ToEndPositionConfig {

        public static int stage1_x = 2700;
        public static int stage1_y = 300;
        public static int stage1_heading = -90;
        public static int stage1_tolerance = 100;
        public static int stage1_timeout = 2000;

        public static int stage2_x = 2100;
        public static int stage2_y = -650;
        public static int stage2_heading = -90;
        public static int stage2_tolerance = 10;
        public static int stage2_timeout = 1000;
    }

    private boolean tryGetDevBoolean(boolean val) {
        return inMatch ? false : val;
    }

    @Override
    protected Command createCommand() {
        boolean skipOuttakePreloaded = tryGetDevBoolean(PreloadConfig.skipOuttake);
        boolean skipIntakeSample1 = tryGetDevBoolean(Sample1Config.skipIntake);
        boolean skipIntakeSample2 = tryGetDevBoolean(Sample2Config.skipIntake);
        boolean skipIntakeSample3 = tryGetDevBoolean(Sample3Config.skipIntake);

        telemetry.addData("skip out take preloaded", skipOuttakePreloaded);
        telemetry.addData("skip intake sample 1", skipIntakeSample1);
        telemetry.addData("skip intake sample 2", skipIntakeSample2);
        telemetry.addData("skip intake sample 3", skipIntakeSample3);

        return new SequentialCommandGroup(

                //region preloaded sample
                // drive to bucket, pivot the arm and extend slider
                new ParallelCommandGroup(
                    commandFactory.logDebug("preload drive to basket").andThen(commandFactory.driveToTarget(PreloadConfig.toBucket_x, PreloadConfig.toBucket_y, PreloadConfig.toBucket_heading, .13, .7, PreloadConfig.toBucket_tolerance, PreloadConfig.toBucket_timeout)),
                    commandFactory.pivotToDelivery(),
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.logDebug("preload extend slider").andThen(commandFactory.extendSliderForOuttake())
                ),

                // deliver preloaded sample
                skipOuttakePreloaded ? commandFactory.doNothing() : commandFactory.outtake(),//.andThen(new InstantCommand(() -> hold1End = true)),
                //endregion

                //region sample #1
                commandFactory.elbowToIntakePosition()
                        .andThen(commandFactory.sleep(400))
                        .andThen(commandFactory.collapseSlider()),

                commandFactory.pivotToGroundIntakeReady()
                        .alongWith(commandFactory.logDebug("Sample 1 drive to sample")
                                .andThen(commandFactory.driveToTarget(Sample1Config.toSample_x, Sample1Config.toSample_y, 0, 0.13, .7, Sample1Config.toSample_tolerance, Sample1Config.toSample_timeout))),

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

                // Simplified drop with slider holding power
                new ParallelDeadlineGroup(
                        commandFactory.whileSamplePresent(3000),
                        commandFactory.extendSliderForOuttake2(5000),
                        new SequentialCommandGroup(
                            commandFactory.driveToTarget(Sample1Config.toBucket_x, Sample1Config.toBucket_y, Sample1Config.toBucket_heading, 0.13, .5, Sample1Config.toBucket_tolerance, Sample1Config.toBucket_timeout),
                            skipIntakeSample1 ? commandFactory.doNothing() : commandFactory.outtake()//.andThen(new InstantCommand(() -> hold2End = true),
                        )
                ),

                // Sample #1
                //skipIntakeSample1 ? commandFactory.doNothing() : commandFactory.outtake(),//.andThen(new InstantCommand(() -> hold2End = true),
                //endregion for sample #1

                //region sample #2
                commandFactory.elbowToIntakePosition().andThen(commandFactory.sleep(400)).andThen(commandFactory.collapseSlider()).andThen(commandFactory.pivotToStart()),
                commandFactory.logDebug("before sample 2 drive to sample"),
                commandFactory.logDebug("sample 2 drive to sample").andThen(commandFactory.driveToTarget(Sample2Config.toSample_x, Sample2Config.toSample_y, Sample2Config.toSample_heading, 0.13, .5, Sample2Config.toSample_tolerance, Sample2Config.toSample_timeout))
                        .alongWith(commandFactory.pivotToGroundInTakeBegin()),

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

                new ParallelDeadlineGroup(
                        commandFactory.whileSamplePresent(3000),
                        commandFactory.extendSliderForOuttake2(5000),
                        new SequentialCommandGroup(
                                commandFactory.driveToTarget(Sample2Config.toBucket_x, Sample2Config.toBucket_y, Sample2Config.toBucket_heading, 0.13, .8, Sample2Config.toBucket_tolerance, Sample2Config.toBucket_timeout),
                                commandFactory.outtake()
                        )
                ),

//                commandFactory.logDebug("sample 2 drive to basket")
//                        .andThen(commandFactory.driveToTarget(Sample2Config.toBucket_x, Sample2Config.toBucket_y, Sample2Config.toBucket_heading, 0.13, .8, Sample2Config.toBucket_tolerance, Sample2Config.toBucket_timeout))
//                        .alongWith(commandFactory.sleep(300)
//                                .andThen(commandFactory.logDebug("sample 2 extend for delivery"))
//                                .andThen(commandFactory.extendSliderForOuttake())),
////                commandFactory.logDebug("sample 2 extend for delivery").andThen(commandFactory.extendSlider()),
//
//                // Sample #2
//                skipIntakeSample2 ? commandFactory.doNothing() : commandFactory.outtake(),//.andThen(new InstantCommand(() -> hold3End = true)),
//                //endregion sample #2

                //region sample #3
                new ParallelCommandGroup(
                        new ParallelRaceGroup(
                                commandFactory.sleep(300),
                                commandFactory.extendSlider()),
                        commandFactory.logDebug("sample 3 drive to sample")
                                .andThen(commandFactory.driveToTarget(Sample3Config.beforeIntakeDrive_targetX, Sample3Config.beforeIntakeDrive_targetY, Sample3Config.beforeIntakeDrive_heading, 0.13, .5, Sample3Config.beforeIntakeDrive_distanceTolerance, 1000))
                ),
                commandFactory.collapseSlider(),
                commandFactory.pivotTo(Sample3Config.pivotIntakePosition),
                commandFactory.elbowToPosition(Sample3Config.elbowIntakePosition),
                commandFactory.logDebug("sample 3 extend for sample").andThen(commandFactory.extendSlider(Sample3Config.sliderIntakePosition)),
                commandFactory.sleep(300),
                skipIntakeSample3 ? commandFactory.doNothing() : commandFactory.intakeFromGround(Sample3Config.pivotToGroundPosition, Sample3Config.IntakeFromGroundTimeout),

                new ParallelCommandGroup(
                    commandFactory.elbowToSpecimenPosition(),
                    commandFactory.pivotToDelivery()

                ),

                new ParallelDeadlineGroup(
                        commandFactory.whileSamplePresent(7000),
                        commandFactory.extendSliderForOuttake2(7000),
                        new SequentialCommandGroup(
                                commandFactory.driveToTarget(Sample3Config.ToDeliveryDrive_TargetX, Sample3Config.ToDeliveryDrive_TargetY, Sample3Config.ToDeliveryDrive_TargetHeading, 0.13, .8, Sample3Config.ToDeliveryDrive_DistanceTolerance, 1200),
                                commandFactory.outtake()
                        )
                ),

//                commandFactory.logDebug("sample 3 extend for delivery")
//                        .andThen(commandFactory.extendSliderForOuttake()),
//                commandFactory.logDebug("sample 3 drive to basket")
//                        .andThen(commandFactory.driveToTarget(Sample3Config.ToDeliveryDrive_TargetX, Sample3Config.ToDeliveryDrive_TargetY, Sample3Config.ToDeliveryDrive_TargetHeading, 0.13, .8, Sample3Config.ToDeliveryDrive_DistanceTolerance, 1200)),
//
//                // Sample #3
//                skipIntakeSample3 ? commandFactory.doNothing() : commandFactory.outtake(),//.andThen(new InstantCommand(() -> hold3End = true)),
//                //endregion sample #3

                new ParallelCommandGroup(
                        commandFactory.elbowToIntakePosition().andThen(commandFactory.sleep(400)).andThen(commandFactory.collapseSlider()).andThen(commandFactory.pivotToStart()),
                        commandFactory.logDebug("sample 3 drive to sample pile step 1").andThen(commandFactory.driveToTarget(ToEndPositionConfig.stage1_x, ToEndPositionConfig.stage1_y, ToEndPositionConfig.stage1_heading, 0.13, .8, ToEndPositionConfig.stage1_tolerance, ToEndPositionConfig.stage1_timeout))
                ),

                commandFactory.logDebug("sample 3 drive to sample pile step 2").andThen(commandFactory.driveToTarget(ToEndPositionConfig.stage2_x, ToEndPositionConfig.stage2_y, ToEndPositionConfig.stage2_heading, .2, .5, ToEndPositionConfig.stage2_tolerance, ToEndPositionConfig.stage2_timeout))
        );
    }
}
