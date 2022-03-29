package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceTimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near spinner for Qualifier
 */
@Disabled
@Autonomous(name = "Red Carousel Autonomous")
public class RedCarouselAutonomous extends AutoObjDetectionTemplate {
    static final boolean wareHousePark = true;
    static final BlinkinPattern def = BlinkinPattern.RED;
    private final boolean overBarrier = false;
    public DistanceSensor distanceSensor;

    //static final double[] initialPos = {7, 101, 90};

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6, 111, 180);
        distanceSensor = (DistanceSensor) hardwareMap.get("distance_sensor");

        telemetry.addData("GC", "Started");
        telemetry.update();
        System.gc();
        telemetry.addData("GC", "Finished");
        telemetry.update();

        BarcodePositions Pos;
        do {
            Pos = this.findPositionOfMarker();
            telemetry.addData("Position", Pos);
            telemetry.update();
            Thread.sleep(200);
        } while (!isStarted() && !isStopRequested());

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
            driveSystem.debugOn();

            driveSystem.strafeAtAngle(270, .6);
            Thread.sleep(1000);
            driveSystem.turnTo(260, .5);

            try {
                driveSystem.moveToPosition(22, 82.5, 1, .5, new DistanceTimeoutException(500));
            } catch (MovementException ignored) {
            }
            driveSystem.turnTo(272, .3);


            // driveSystem.turnTo(260, .4);


            switch (Pos) {
                case NotSeen:
                case Right:
                    // got to the top level when right
                    slide.setTargetLevel(HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(1000);
                    driveSystem.halt();
                    outtake.setServoOpen();
                    Thread.sleep(750);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    driveSystem.halt();
                    slide.setTargetLevel(HeightLevel.Down);
                    break;
                case Center:
                    slide.setTargetLevel(HeightLevel.MiddleLevel);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .25);
                    Thread.sleep(725);
                    driveSystem.halt();
                    outtake.setServoOpen();
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    driveSystem.halt();
                    slide.setTargetLevel(HeightLevel.Down);
                    Thread.sleep(500);
                    break;
                case Left:
                    // go to bottom when left
                    slide.setTargetLevel(HeightLevel.BottomLevel);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(1000);
                    driveSystem.halt();
                    outtake.setServoOpen();
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);

                    driveSystem.halt();

                    slide.setTargetLevel(HeightLevel.Down);
                    Thread.sleep(500);
                    break;
            }


            try {
                driveSystem.moveToPosition(20, 150, 1, 1, new DistanceTimeoutException(500));
            } catch (MovementException ignored) {
            }

            // this moves into the wall before spinning off the duck

            driveSystem.strafeAtAngle(15, .5);

            Thread.sleep(1350);
            driveSystem.halt();
            spinner.spinOffRedDuck();
            driveSystem.strafeAtAngle(270, 1);
            Thread.sleep(1000);
            if (!overBarrier) {
                try {
                    driveSystem.moveToPosition(17, 80, 1, 2, new DistanceTimeoutException(1000));
                } catch (MovementException stopOnStuck) {
                    this.stop();
                }
                driveSystem.turnTo(180, .5);
                try {
                    driveSystem.moveToPosition(0, 80, 0, 2, new DistanceTimeoutException(750));
                } catch (MovementException ignored) {
                }

                intake.turnIntakeOn();
                try {

                    driveSystem.moveToPosition(0, 10, 1, 1, new MovementException[]{new DistanceSensorException(distanceSensor, 8), new DistanceTimeoutException(500)});

                } catch (MovementException ignored) {
                } finally {
                    intake.turnIntakeOff();
                }
            } else if (overBarrier) {
                try {
                    driveSystem.moveToPosition(33, 77, 1, 2, new DistanceTimeoutException(1000));
                } catch (MovementException ignored) {
                }
                driveSystem.turnTo(180, .8);
                driveSystem.turnTo(180, .2);
                podServos.raise();
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(2500);
                this.stop();
            }
        }
    }
}
