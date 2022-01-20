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

            driveSystem.moveToPosition(22, 84, 1);

            driveSystem.turnTo(260, .3);


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
        }
        slide.end();
        odometry.end();
    }
}
