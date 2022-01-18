package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
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
        odometry.setPosition(7, 112, 180);
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

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();

            driveSystem.debugOn();
            driveSystem.strafeAtAngle(270, .8);
            Thread.sleep(500);
            driveSystem.moveToPosition(20, 86, 270, 1);
            Thread.sleep(3000);
            //this is where the loadoff of first freight goes
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(16, 138, 1, 1, 500);
            } catch (OdometryMovementException ignored) {
            }
            // this moves into the wall before spinning off the duck
            driveSystem.strafeAtAngle(10, 1);

            Thread.sleep(1000);
            driveSystem.stopAll();
            spinner.spinOffRedDuck();
            driveSystem.strafeAtAngle(270, 1);
            Thread.sleep(1000);
            driveSystem.moveToPosition(7, 80, 180, 1);
            driveSystem.strafeAtAngle(90, 1);
            Thread.sleep(500);
            driveSystem.moveToPosition(7, 60, 1);
            //Thread.sleep(1000);
            /*
            try {
                driveSystem.moveToPositionWithDistanceTimeOut(63,7,1,1000);
            }catch(OdometryMovementException stuck){
              //TODO: create backup program
            }

             */
            while (distanceSensor.getDistance(DistanceUnit.CM) > 8 && !isStopRequested()) {
                intake.setIntakeOn();
                driveSystem.strafeAtAngle(0, .7);
            }
            intake.setIntakeOff();
            this.stop();

            /*driveSystem.strafeAtAngle(270, .6);
            Thread.sleep(1000);
            driveSystem.turnTo(248.5, .5);
            driveSystem.moveToPosition(FieldPoints.RedWestLoadingPoint, 1);
            driveSystem.strafeAt

             */





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
