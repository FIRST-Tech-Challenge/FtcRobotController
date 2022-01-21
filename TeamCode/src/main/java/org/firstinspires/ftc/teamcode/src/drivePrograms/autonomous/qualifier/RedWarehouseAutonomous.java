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
 * The Autonomous ran on Red side near Warehouse for Qualifier
 */
@Autonomous(name = "Red Warehouse Autonomous")
public class RedWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final double[] initialPos = {7, 63, 90};
    static final BlinkinPattern def = BlinkinPattern.RED;
    public DistanceSensor distanceSensor;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        odometry.setPosition(6.5, 64, 180);
        distanceSensor = (DistanceSensor) hardwareMap.get("distance_sensor");

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
        slide.start();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();

            driveSystem.debugOn();

            driveSystem.strafeAtAngle(270, .6);
            Thread.sleep(1000);
            driveSystem.turnTo(260, .5);
            driveSystem.moveToPosition(23, 85, 1);

            //TODO Add Raises and lowers
            switch (Pos) {
                case NotSeen:
                case Right:
                    // got to the top level when right
                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(180, .15);
                    Thread.sleep(500);
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
                    Thread.sleep(400);
                    intake.setServoOpen();
                    Thread.sleep(750);
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
                    Thread.sleep(1500);
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
                driveSystem.moveToPositionWithDistanceTimeOut(12, 70, 1, 1, 500);
            } catch (OdometryMovementException ignored) {
            }
            driveSystem.turnTo(200, .8);
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(4, 70, 1, 2, 1000);
            } catch (OdometryMovementException ignored) {
            }


            intake.setIntakeOn();
            try {

                double millis = 250;
                final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                final double[] positionBeforeTimeLoop = {0}; //These are arrays to make the compiler happy. Treat them as a normal double
                final double[] positionAfterTimeLoop = {Double.MAX_VALUE}; //These are arrays to make the compiler happy. Treat them as a normal double
                final double tooSmallOfDistance = millis / 500.0; // this travels ~2 inches for every 1000 millis

                Executable<Boolean> t = () -> {

                    if (timer.milliseconds() >= millis) {
                        positionBeforeTimeLoop[0] = positionAfterTimeLoop[0];
                        positionAfterTimeLoop[0] = MiscUtils.distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), 7, 7);
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

                Executable<Boolean> q = () -> {
                    return (t.call() || e.call());
                };
                driveSystem.moveToPosition(7, 7, 1, 1, q);

            } catch (OdometryMovementException ignored) {
            } finally {
                intake.setIntakeOff();
            }

            intake.setIntakeOff();


        }
        slide.end();
        odometry.end();

    }
}


