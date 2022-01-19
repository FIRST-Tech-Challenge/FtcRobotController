package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;
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
                driveSystem.moveToPositionWithDistanceTimeOut(23, 86, 1, 1, 1000);
            } catch (OdometryMovementException ignored) {
            }
            driveSystem.turnTo(255, .3);


            switch (Pos) {
                case Right:
                case NotSeen:
                    // got to the top level when right
                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(180, .3);
                    Thread.sleep(500);
                    intake.setServoOpen();
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    break;
                case Center:
                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);
                    Thread.sleep(750);
                    driveSystem.strafeAtAngle(180, .3);
                    Thread.sleep(500);
                    intake.setServoOpen();
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    break;
                case Left:
                    // go to bottom when left
                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(180, .3);
                    Thread.sleep(500);
                    intake.setServoOpen();
                    Thread.sleep(500);
                    driveSystem.strafeAtAngle(0, .5);
                    Thread.sleep(500);
                    slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                    Thread.sleep(500);
                    break;
            }

            //this is where the loadoff of first freight goes
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(16, 150, 1, 1, 500);
            } catch (OdometryMovementException ignored) {
            }

            // this moves into the wall before spinning off the duck
            driveSystem.strafeAtAngle(5, .5);

            Thread.sleep(700);
            driveSystem.stopAll();
            spinner.spinOffRedDuck();
            driveSystem.strafeAtAngle(270, 1);
            Thread.sleep(1000);
            driveSystem.moveToPosition(10, 80, 1);
            driveSystem.turnTo(180, .5);
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(5, 80, 0, 1, 500);
            } catch (OdometryMovementException ignored) {
            }
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(7, 45, 1, 1, 1000);
            } catch (OdometryMovementException stuck) {
                //TODO: create backup program
                driveSystem.stopAll();
            }

            while (distanceSensor.getDistance(DistanceUnit.CM) > 8 && !isStopRequested()) {
                intake.setIntakeOn();
                driveSystem.strafeAtAngle(0, .7);
            }
            intake.setIntakeOff();
            this.stop();









            /*switch (Pos) {
                case Right:
                    telemetry.addData("position", " is right");
                    telemetry.update();
                    driveSystem.moveToPosition(20, 84, 1);
                    slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);

                    Thread.sleep(1500);


                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(22, 81, 1);

                    intake.setServoClosed();
                    Thread.sleep(500);


                    break;
                case NotSeen:
                    telemetry.addData("position", " far left");
                    telemetry.update();
                    driveSystem.moveToPosition(18, 85, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);

                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 84, 1);


                    break;
                case Left:
                    telemetry.addData("position", "is center");
                    telemetry.update();
                    driveSystem.moveToPosition(19, 85, 1);

                    slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);

                    Thread.sleep(1500);

                    intake.setServoClosed();
                    Thread.sleep(500);

                    //this position will vary for different heights on the goal
                    driveSystem.moveToPosition(25, 84, 1);


                    break;


            }
            //Shared Code
            intake.setIntakeOn();
            Thread.sleep(1000);
            intake.setIntakeOff();
            driveSystem.moveToPosition(23, 84, 1);
            intake.setServoOpen();
            slide.setTargetLevel(LinearSlide.HeightLevel.Down);
            Thread.sleep(500);
            //following this is unique to carousel and warehouse

            driveSystem.moveToPosition(20, 130, 1);

            //strafe at angle into the wall and then the carousel
            driveSystem.strafeAtAngle(270, .5);
            Thread.sleep(500);
            driveSystem.strafeAtAngle(180, .5);
            Thread.sleep(650);
            driveSystem.stopAll();

            //spin off the duck
            spinner.spinOffRedDuck();

            if (wareHousePark) {
                //Parks in the Warehouse
                driveSystem.strafeAtAngle(270, .5);
                Thread.sleep(500);
                odometry.setPosition(17, 134, 90); //This recalibrates the Odometry to the spot it should be at this point
                driveSystem.moveToPosition(24, 75, 1);
                driveSystem.turnTo(180, 0.5);
                podServos.raise();
                driveSystem.stopAll();
                Thread.sleep(500);
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(1750);

            } else {
                // parks in depot
                driveSystem.moveToPosition(31.5, 130, 1);
                driveSystem.strafeAtAngle(270, .5);
                Thread.sleep(300);
                driveSystem.stopAll();
            }


             */
        }
        slide.end();
        odometry.end();
    }
}
