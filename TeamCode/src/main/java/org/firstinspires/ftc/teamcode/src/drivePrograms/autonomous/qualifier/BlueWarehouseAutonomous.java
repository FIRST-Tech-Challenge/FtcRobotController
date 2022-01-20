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
import org.firstinspires.ftc.teamcode.src.utills.MiscUtills;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;

/**
 * The Autonomous ran on Blue side near warehouse for Qualifier
 */
@Autonomous(name = "Blue Warehouse Autonomous")
public class BlueWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    static final double[] initialPos = {133.5, 54.5, 0};
    DistanceSensor distanceSensor;

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
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        } while (!isStarted() && !isStopRequested());

        waitForStart();
        slide.start();
        driveSystem.strafeAtAngle(0, 1);
        Thread.sleep(500);
        driveSystem.strafeAtAngle(270, .8);
        Thread.sleep(500);
        driveSystem.turnTo(80, .8);
        driveSystem.moveToPosition(117, 83, 1);

        switch (Pos) {
            case NotSeen:
            case Right:
                // got to the top level when right
                slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                Thread.sleep(1000);
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
                driveSystem.strafeAtAngle(180, .3);
                Thread.sleep(250);
                driveSystem.stopAll();
                intake.setServoOpen();
                Thread.sleep(750);
                driveSystem.strafeAtAngle(0, .5);
                Thread.sleep(500);

                driveSystem.stopAll();

                slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                Thread.sleep(500);
                break;
        }

        try {
            driveSystem.moveToPositionWithDistanceTimeOut(131, 65, 1, 1, 1000);
        } catch (OdometryMovementException stop) {
            this.stop();
        }
        driveSystem.turnTo(160, 1);
        try {
            driveSystem.moveToPositionWithDistanceTimeOut(137, 65, 1, 1, 500);
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
                    positionAfterTimeLoop[0] = MiscUtills.distance(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition(), 135, 7);
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
            driveSystem.moveToPosition(135, 7, 1, 1, q);

        } catch (OdometryMovementException ignored) {
        } finally {
            intake.setIntakeOff();
        }


        slide.end();
        odometry.end();

    }
}


