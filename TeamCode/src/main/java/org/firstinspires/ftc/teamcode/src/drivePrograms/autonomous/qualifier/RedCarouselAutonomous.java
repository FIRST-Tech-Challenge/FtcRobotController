package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.Executable;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

/**
 * The Autonomous ran on Red side near spinner for Qualifier
 */
@Autonomous(name = "Red Carousel Autonomous")
public class RedCarouselAutonomous extends AutoObjDetectionTemplate {
    static final boolean wareHousePark = true;
    static final BlinkinPattern def = BlinkinPattern.RED;
    public DistanceSensor distanceSensor;

    //static final double[] initialPos = {7, 101, 90};

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        odometry.setPosition(6, 111, 180);
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

            driveSystem.strafeAtAngle(270, .6);
            Thread.sleep(1000);
            driveSystem.turnTo(260, .5);

            try {
                driveSystem.moveToPositionWithDistanceTimeOut(22, 84, 1, 2, 500);
            } catch (OdometryMovementException ignored) {
            }

            driveSystem.turnTo(260, .4);


            //TODO add movements


            try {
                driveSystem.moveToPositionWithDistanceTimeOut(16, 150, 1, 1, 500);
            } catch (OdometryMovementException ignored) {
            }

            // this moves into the wall before spinning off the duck
            driveSystem.strafeAtAngle(5, .5);

            Thread.sleep(1000);
            driveSystem.stopAll();
            spinner.spinOffRedDuck();
            driveSystem.strafeAtAngle(270, 1);
            Thread.sleep(1000);
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(17, 80, 1, 2, 1000);
            } catch (OdometryMovementException stopOnStuck) {
                this.stop();
            }
            driveSystem.turnTo(180, .5);
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(0, 80, 0, 2, 750);
            } catch (OdometryMovementException ignored) {
            }

            intake.setIntakeOn();
            try {

                double millis = 500;
                final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
                final double[] positionBeforeTimeLoop = {0}; //These are arrays to make the compiler happy. Treat them as a normal double
                final double[] positionAfterTimeLoop = {Double.MAX_VALUE}; //These are arrays to make the compiler happy. Treat them as a normal double
                final double tooSmallOfDistance = millis / 500.0; // this travels ~2 inches for every 1000 millis

                Executable<Boolean> t = () -> {

                    if (timer.milliseconds() >= millis) {
                        positionBeforeTimeLoop[0] = positionAfterTimeLoop[0];
                        positionAfterTimeLoop[0] = MiscUtils.distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), 5, 7);
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
                driveSystem.moveToPosition(5, 7, 1, 1, q);

            } catch (OdometryMovementException ignored) {
            } finally {
                intake.setIntakeOff();
            }
        }
        slide.end();
        odometry.end();
    }
}
