package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

/**
 * The Autonomous ran on Blue side near spinner for Qualifier
 */
@Autonomous(name = "Blue Carousel Autonomous")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    static final double[] initialPos = {133, 101, 0};

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        odometry.setPosition(initialPos[0], initialPos[1], initialPos[2]);

        telemetry.addData("GC", "Started");
        telemetry.update();
        System.gc();
        telemetry.addData("GC", "Finished");
        telemetry.update();

        BarcodePositions Pos;
        do {
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        } while (!isStarted() && !isStopRequested());

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
            switch (Pos) {
                case NotSeen:
                    telemetry.addData("position", " is far right");
                    telemetry.update();

                    driveSystem.moveToPosition(120, 84, 1);


                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                    intake.setServoClosed();

                    Thread.sleep(2000);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 81, 1);

                    intake.setServoClosed();
                    Thread.sleep(500);
                    break;
                case Right:
                    telemetry.addData("position", " is center");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);


                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);
                    Thread.sleep(1500);
                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);
                    break;

                case Left:
                    telemetry.addData("position", "is left");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);
                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);
                    break;
            }

            intake.setIntakeOn();
            Thread.sleep(1000);
            intake.setIntakeOff();
            driveSystem.moveToPosition(117, 84, 1);
            intake.setServoOpen();
            slide.setTargetLevel(LinearSlide.HeightLevel.Down);
            Thread.sleep(500);

            driveSystem.moveToPosition(126, 112, 1);
            driveSystem.turnTo(185, .5);
            driveSystem.moveToPosition(116, 130, 1);

            driveSystem.strafeAtAngle(180, .5);
            Thread.sleep(400);
            driveSystem.stopAll();
            driveSystem.strafeAtAngle(270, .55);
            Thread.sleep(700);
            driveSystem.stopAll();
            spinner.spinOffBlueDuck();

            boolean warehousePark = false;

            if (warehousePark) {
                //TODO: Make this work
                driveSystem.strafeAtAngle(180, .5);
                Thread.sleep(400);
                odometry.setPosition(144 - 17, 144 - 7, 180);
                driveSystem.moveToPosition(144 - 24, 64, 1);
                driveSystem.turnTo(180, 0.5);
                podServos.raise();
                driveSystem.stopAll();
                Thread.sleep(500);
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(1750);

            } else {
                driveSystem.moveToPosition(102, 130, 1);
                driveSystem.strafeAtAngle(180, .5);
                Thread.sleep(700);
                driveSystem.stopAll();
            }


        }
        slide.end();
        odometry.end();

    }
}


