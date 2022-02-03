package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceTimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near Warehouse for Qualifier
 */
@Disabled
@Autonomous(name = "Red Warehouse Autonomous")
public class RedWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;
    public DistanceSensor distanceSensor;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6.5, 64, 180);
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

            driveSystem.strafeAtAngle(270, .6);
            Thread.sleep(1000);
            driveSystem.turnTo(260, .5);

            try {
                driveSystem.moveToPosition(25, 85, 1, 1, new DistanceTimeoutException(1000));
            } catch (MovementException ignored) {
            }
            driveSystem.turnTo(270, .2);

            switch (Pos) {
                case NotSeen:
                case Right:
                    // got to the top level when right
                    slide.setTargetLevel(HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(500);
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
                driveSystem.moveToPosition(12, 70, 1, 1, new DistanceTimeoutException(500));
            } catch (MovementException ignored) {
            }
            driveSystem.turnTo(200, .8);
            try {
                driveSystem.moveToPosition(4, 70, 1, 2, new DistanceTimeoutException(1000));
                driveSystem.strafeAtAngle(90, 1);
                Thread.sleep(250);
            } catch (MovementException ignored) {
            }


            intake.setIntakeOn();
            try {
                driveSystem.moveToPosition(0, 8, 1, 1, new MovementException[]{new DistanceSensorException(distanceSensor, 8), new DistanceTimeoutException(500)});

            } catch (MovementException ignored) {
            } finally {
                intake.setIntakeOff();
            }

            intake.setIntakeOff();


        }

    }
}


