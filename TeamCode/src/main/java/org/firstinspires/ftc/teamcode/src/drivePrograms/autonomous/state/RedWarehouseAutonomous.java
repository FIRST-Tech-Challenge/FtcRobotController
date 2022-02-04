package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.TimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.IntakeColorSensorWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near Warehouse for State
 */
@Autonomous(name = "Red State Warehouse Autonomous")
public class RedWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;
    public DistanceSensor distanceSensor;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6.5, 64, 180);
        distanceSensor = (DistanceSensor) hardwareMap.get("distance_sensor");

        BarcodePositions Pos;
        do {
            Pos = this.findPositionOfMarker();
            telemetry.addData("Position", Pos);
            telemetry.update();
            Thread.sleep(200);
            if (opModeIsActive()) {
                break;
            }
        } while (!isStarted() && !isStopRequested());

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();

            driveSystem.setTurnWhileStrafe(true);
            driveSystem.debugOn();

            driveSystem.moveToPosition(26, 82.5, 272, .5, new DistanceTimeoutWarning(500));

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

            while (opModeIsActive() && !isStopRequested()) {

                driveSystem.moveTowardsPosition(12, 70, 180, 2, 1, new DistanceTimeoutWarning(100));

                driveSystem.turnTo(180, 0.5);

                driveSystem.moveToPosition(-1, 70, 180, 2, new DistanceTimeoutWarning(100));

                intake.setIntakeOn();

                if (this.getRuntime() > 25) {
                    continue;
                }

                try {
                    driveSystem.moveToPosition(0, 8, 180, 1, new MovementException[]{new DistanceSensorException(distanceSensor, 8), new DistanceTimeoutWarning(500)});
                } catch (DistanceSensorException e) {
                    intake.setIntakeReverse();
                    Thread.sleep(250);
                    intake.setIntakeOff();
                    intake.setServoClosed();
                    driveSystem.moveToPosition(-1, gps.getY(), 180, 2, new DistanceTimeoutWarning(100));

                    driveSystem.moveToPosition(-1, 70, 180, 2, new DistanceTimeoutWarning(100));
                    intake.setIntakeOn();
                    try {
                        driveSystem.moveToPosition(26, 82.5, 272, 0, new MovementException[]{new IntakeColorSensorWarning(intake), new TimeoutException(5000)});
                    } catch (TimeoutException w) {
                        //If we timed out, we don't have a item. We go back through into the warehouse
                        continue;
                    } catch (MovementException ignored) {
                    }

                    driveSystem.moveToPosition(26, 82.5, 272, 0, new DistanceTimeoutWarning(100));

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

                } catch (MovementException ignored) {
                }
                if (this.getRuntime() > 25) {
                    break;
                }


            }
            intake.setIntakeOff();


        }

    }
}


