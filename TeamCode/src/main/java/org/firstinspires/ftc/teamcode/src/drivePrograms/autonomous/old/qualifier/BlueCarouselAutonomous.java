package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceTimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Blue side near spinner for Qualifier
 */
@Disabled
@Autonomous(name = "Blue Carousel Autonomous")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    static final double[] initialPos = {133.5, 103, 0};
    final boolean overBarrier = true;
    public DistanceSensor distanceSensor;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(initialPos[0], initialPos[1], initialPos[2]);
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
            telemetry.addData("Executing Auto", Pos);
            telemetry.update();
            driveSystem.strafeAtAngle(270, .8);
            Thread.sleep(500);
            driveSystem.turnTo(70, .8);
            driveSystem.moveToPosition(117, 82.5, 1);
            driveSystem.turnTo(90, .5);

            switch (Pos) {
                case NotSeen:
                case Right:
                    // got to the top level when right
                    slide.setTargetLevel(HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(1000);
                    driveSystem.stopAll();
                    intake.setServoOpen();
                    Thread.sleep(750);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    driveSystem.stopAll();
                    slide.setTargetLevel(HeightLevel.Down);
                    break;
                case Center:
                    slide.setTargetLevel(HeightLevel.MiddleLevel);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .25);
                    Thread.sleep(725);
                    driveSystem.stopAll();
                    intake.setServoOpen();
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    driveSystem.stopAll();
                    slide.setTargetLevel(HeightLevel.Down);
                    Thread.sleep(500);
                    break;
                case Left:
                    // go to bottom when left
                    slide.setTargetLevel(HeightLevel.BottomLevel);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(1000);
                    driveSystem.stopAll();
                    intake.setServoOpen();
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);

                    driveSystem.stopAll();

                    slide.setTargetLevel(HeightLevel.Down);
                    Thread.sleep(500);
                    break;
            }

            try {
                driveSystem.moveToPosition(118, 144, 1, 1, new DistanceTimeoutException(500));
            } catch (MovementException ignored) {
            }
            driveSystem.turnTo(90, .8);
            driveSystem.turnTo(90, .3);
            driveSystem.strafeAtAngle(355, .5);
            Thread.sleep(800);
            driveSystem.stopAll();
            spinner.spinOffBlueDuck();
            try {
                driveSystem.moveToPosition(120, 70, 1, 1, new DistanceTimeoutException(1000));
            } catch (MovementException stop) {
                this.stop();
            }
            if (!overBarrier) {
                driveSystem.turnTo(160, 1);
                driveSystem.strafeAtAngle(270, .8);
                Thread.sleep(750);
                driveSystem.turnTo(190, .5);
                intake.setIntakeOn();


                try {
                    driveSystem.moveToPosition(135, 20, 1, 1, new MovementException[]{new DistanceSensorException(distanceSensor, 8), new DistanceTimeoutException(500)});

                } catch (MovementException ignored) {
                } finally {
                    intake.setIntakeOff();
                }
                while (!isStopRequested() && opModeIsActive() && (distanceSensor.getDistance(DistanceUnit.CM) > 8)) {
                    driveSystem.strafeAtAngle(0, 0.5);
                }
                if (gps.getY() < 20) {
                    try {
                        driveSystem.moveToPosition(gps.getX(), 15, 1, 1, new DistanceTimeoutException(500));
                    } catch (MovementException ignored) {
                    }
                }


            } else if (overBarrier) {
                try {
                    driveSystem.moveToPosition(115, 70, 1, 1, new DistanceTimeoutException(500));
                } catch (MovementException ignored) {
                }
                driveSystem.turnTo(180, 1);
                driveSystem.turnTo(180, .2);
                podServos.raise();
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(2500);
                driveSystem.stopAll();
                this.stop();

            }
        }


    }
}


