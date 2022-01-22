package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

/**
 * The Autonomous ran on Blue side near spinner for Qualifier
 */
@Autonomous(name = "Blue Carousel Autonomous")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    static final double[] initialPos = {133.5, 103, 0};
    public DistanceSensor distanceSensor;
    boolean overBarrier = true;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        odometry.setPosition(initialPos[0], initialPos[1], initialPos[2]);
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
        slide.start();

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
                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(1000);
                    driveSystem.stopAll();
                    intake.setServoOpen();
                    Thread.sleep(750);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    driveSystem.stopAll();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    break;
                case Center:
                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .25);
                    Thread.sleep(725);
                    driveSystem.stopAll();
                    intake.setServoOpen();
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    driveSystem.stopAll();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    break;
                case Left:
                    // go to bottom when left
                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(1000);
                    driveSystem.stopAll();
                    intake.setServoOpen();
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);

                    driveSystem.stopAll();

                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    break;
            }

            try {
                driveSystem.moveToPositionWithDistanceTimeOut(118, 144, 1, 1, 500);
            } catch (OdometryMovementException ignored) {
            }
            driveSystem.turnTo(90, .8);
            driveSystem.turnTo(90, .3);
            driveSystem.strafeAtAngle(355, .5);
            Thread.sleep(800);
            driveSystem.stopAll();
            spinner.spinOffBlueDuck();
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(120, 70, 1, 1, 1000);
            } catch (OdometryMovementException stop) {
                this.stop();
            }
            if (!overBarrier) {
                driveSystem.turnTo(160, 1);
                driveSystem.strafeAtAngle(270, .8);
                Thread.sleep(750);
                driveSystem.turnTo(190, .5);
                intake.setIntakeOn();

                Executable<Boolean> q;

                double millis = 500;
                final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                final double[] positionBeforeTimeLoop = {0}; //These are arrays to make the compiler happy. Treat them as a normal double
                final double[] positionAfterTimeLoop = {Double.MAX_VALUE}; //These are arrays to make the compiler happy. Treat them as a normal double
                final double tooSmallOfDistance = millis / 500.0; // this travels ~2 inches for every 1000 millis

                Executable<Boolean> t = () -> {

                    if (timer.milliseconds() >= millis) {
                        positionBeforeTimeLoop[0] = positionAfterTimeLoop[0];
                        positionAfterTimeLoop[0] = MiscUtils.distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), 135, 7);
                        double traveledDistance = Math.abs(positionBeforeTimeLoop[0] - positionAfterTimeLoop[0]);
                        if (traveledDistance < tooSmallOfDistance) {
                            return true;
                        }
                        timer.reset();
                    }
                    return false;
                };


                Executable<Boolean> e = () -> {
                    return !(distanceSensor.getDistance(DistanceUnit.CM) > 8 && !isStopRequested());
                };

                q = () -> {
                    return (t.call() || e.call());
                };

                try {


                    driveSystem.moveToPosition(135, 20, 1, 1, q);

                } catch (OdometryMovementException ignored) {
                } finally {
                    intake.setIntakeOff();
                }
                while (!isStopRequested() && opModeIsActive() && (!q.call())) {
                    driveSystem.strafeAtAngle(0, 0.5);
                }
                if (odometry.returnRelativeYPosition() < 20) {
                    try {
                        driveSystem.moveToPositionWithDistanceTimeOut(odometry.returnRelativeXPosition(), 15, 1, 1, 500);
                    } catch (OdometryMovementException ignored) {
                    }
                }


            } else if (overBarrier) {
                try {
                    driveSystem.moveToPositionWithDistanceTimeOut(115, 70, 1, 1, 1000);
                } catch (OdometryMovementException ignored) {
                }
                driveSystem.turnTo(180, 1);
                driveSystem.turnTo(180, .2);
                podServos.raise();
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(1900);
                driveSystem.stopAll();
                this.stop();

            }
        }
        slide.end();
        odometry.end();

    }
}


