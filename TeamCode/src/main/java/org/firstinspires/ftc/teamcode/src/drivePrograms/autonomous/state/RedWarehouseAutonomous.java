package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.TimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.IntakeColorSensorWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near Warehouse for State
 */
@Autonomous(name = "Red State Warehouse Autonomous")
public class RedWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;
    public DistanceSensor intakeDistanceSensor;
    public DistanceSensor frontDistanceSensor;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6.5, 64, 180);
        intakeDistanceSensor = (DistanceSensor) hardwareMap.get("distance_sensor");
        frontDistanceSensor = (DistanceSensor) hardwareMap.get("front_distance_sensor");

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

        Pos = BarcodePositions.Left;
        // get rid of this once camera position working

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();

            driveSystem.setTurnWhileStrafe(true);
            driveSystem.debugOn();

            driveSystem.moveToPosition(26, 82.5, 272, .5, new DistanceTimeoutWarning(500));

            dropOffFreight(Pos);


            while (opModeIsActive() && !isStopRequested()) {

                //Move against the wall
                {
                    driveSystem.moveTowardsPosition(12, 70, 180, 2, 1, new DistanceTimeoutWarning(100));

                    driveSystem.turnTo(180, 0.5);

                    driveSystem.moveToPosition(-1, 70, 180, 2, new DistanceTimeoutWarning(100));
                }


                intake.setIntakeOn();


                DistanceSensorException intakeEx = new DistanceSensorException(intakeDistanceSensor, 8);
                DistanceSensorException frontEx = new DistanceSensorException(frontDistanceSensor, 20);

                try {
                    driveSystem.moveToPosition(5, 15, 180, 1, new MovementException[]{intakeEx, new DistanceTimeoutWarning(100), frontEx});
                } catch (DistanceSensorException e) {
                    if (e == frontEx) {
                        double tmp = frontDistanceSensor.getDistance(DistanceUnit.INCH);
                        double power;

                        while (intakeDistanceSensor.getDistance(DistanceUnit.INCH) > 8 && opModeIsActive() && !isStopRequested()) {
                            // this will loop as long as an item is not in the intake
                            power = driveSystem.shortMovementPowerCalculation(tmp, frontDistanceSensor.getDistance(DistanceUnit.INCH), .5, .2);
                            driveSystem.strafeAtAngle(0, power);
                        }

                    }
                    intake.setIntakeReverse();
                    Thread.sleep(250);
                    intake.setIntakeOff();
                    //intake.setServoClosed();
                    driveSystem.moveToPosition(-1, gps.getY() + 5, 180, 2, new DistanceTimeoutWarning(100));
                    intake.setIntakeOn();


                    //driveSystem.moveTowardsPosition(0, 30, 180, .5,1, new DistanceTimeoutWarning(100));


                    try {
                        driveSystem.moveToPosition(-1, 30, 180, 2, new MovementException[]{new IntakeColorSensorWarning(intake), new TimeoutException(5000)});
                        //driveSystem.moveToPosition(26, 82.5, 272, 0, new MovementException[]{new IntakeColorSensorException(intake), new TimeoutException(5000)});
                    } catch (TimeoutException w) {
                        //If we timed out, we don't have a item. We go back through into the warehouse
                        telemetry.addData("Timeout exception", "reached");
                        telemetry.update();
                        //continue;


                    } catch (MovementException ignored) {
                    }
                    driveSystem.moveToPosition(0, 70, 180, 3, new DistanceTimeoutWarning(100));
                    driveSystem.moveToPosition(26, 82.5, 272, 0, new DistanceTimeoutWarning(100));


                    dropOffFreight();

                } catch (MovementException ignored) {
                    if (this.getRuntime() > 25) {
                        //break;
                    }


                }
                intake.setIntakeOff();


            }

        }
    }
}


