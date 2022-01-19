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
 * The Autonomous ran on Blue side near spinner for Qualifier
 */
@Autonomous(name = "Blue Carousel Autonomous")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    static final double[] initialPos = {};
    public DistanceSensor distanceSensor;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        odometry.setPosition(133.5, 103, 0);
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
            driveSystem.strafeAtAngle(270, .8);
            Thread.sleep(500);
            driveSystem.turnTo(80, .8);
            driveSystem.moveToPosition(117, 85, 1);
            // this is the first mineral load-off
            Thread.sleep(3000);
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(120, 139, 1, 1, 500);
            } catch (OdometryMovementException ignored) {
            }
            driveSystem.strafeAtAngle(355, .5);
            Thread.sleep(800);
            driveSystem.stopAll();
            spinner.spinOffBlueDuck();
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(130, 70, 1, 1, 1000);
            } catch (OdometryMovementException stop) {
                this.stop();
            }
            driveSystem.turnTo(160, 1);
            driveSystem.strafeAtAngle(270, .8);
            Thread.sleep(750);
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

             */


        }
        slide.end();
        odometry.end();

    }
}


