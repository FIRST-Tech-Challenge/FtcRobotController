package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
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
        // this is where the first mineral drop-off happens
        Thread.sleep(3000);
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




        /*
        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();

            switch (Pos) {
                case Right:
                    telemetry.addData("position", " is far right");
                    telemetry.update();

                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                    Thread.sleep(1500);


                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 81, 1);

                    intake.setServoClosed();
                    Thread.sleep(500);


                    intake.setIntakeOn();
                    Thread.sleep(1000);
                    intake.setIntakeOff();
                    driveSystem.moveToPosition(120, 84, 1);
                    intake.setServoOpen();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);


                    intake.setServoOpen();
                    driveSystem.turnTo(190, .5);
                    //Thread.sleep(500);
                    driveSystem.moveToPosition(131, 63, 2);

                    driveSystem.moveToPosition(132, 24, 1);
                    break;

                case NotSeen:
                    telemetry.addData("position", " is Far Left");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);

                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    intake.setIntakeOn();
                    Thread.sleep(1000);
                    intake.setIntakeOff();
                    driveSystem.moveToPosition(117, 84, 1);
                    intake.setServoOpen();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse

                    driveSystem.turnTo(190, .5);
                    //Thread.sleep(500);
                    driveSystem.moveToPosition(131, 63, 2);

                    driveSystem.moveToPosition(132, 24, 1);
                    break;

                case Left:
                    telemetry.addData("position", "is center");
                    telemetry.update();
                    driveSystem.moveToPosition(120, 84, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);

                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(115, 84, 1);


                    intake.setIntakeOn();
                    Thread.sleep(1000);
                    intake.setIntakeOff();
                    driveSystem.moveToPosition(117, 84, 1);
                    intake.setServoOpen();
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    //following this is unique to carousel and warehouse

                    driveSystem.turnTo(190, .5);
                    //Thread.sleep(500);
                    driveSystem.moveToPosition(131, 63, 2);

                    driveSystem.moveToPosition(132, 24, 1);
                    break;


            }
        }

         */
        slide.end();
        odometry.end();

    }
}


