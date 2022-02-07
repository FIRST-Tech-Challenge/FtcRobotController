package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
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


                //Through Barrier
                driveSystem.moveToPosition(1, 30, gps.getRot(), 1, new DistanceTimeoutWarning(500));


                //Move away from the wall
                driveSystem.moveToPosition(3, gps.getY(), 180, 1, new DistanceTimeoutWarning(100));


                //Runtime Check
                if (this.getRuntime() > 25) {
                    return;
                }


                //Turn to 180
                driveSystem.turnTo(180, .4);


                pickupLoop:
                //See Java Labels
                //Loops while it has not successfully picked an item up
                while (opModeIsActive() && !isStopRequested()) {
                    driveSystem.moveToPosition(12, 26, 180, 1, new DistanceTimeoutWarning(500));

                    while (opModeIsActive() && !isStopRequested()) {
                        //strafe at angle of 0 degrees
                        driveSystem.strafeAtAngle(0, 0.5);

                        //If the distance sensor sees something, move on to shutting the intake off
                        if (intakeDistanceSensor.getDistance(DistanceUnit.CM) < 8) {
                            break;
                        }

                        //turn right
                        driveSystem.turnTo(gps.getRot() + 1, 0.5);
                    }

                    intake.setIntakeOff();

                    // Move backwards
                    driveSystem.move(180, 2, 1);


                    ElapsedTime intakeTimer = new ElapsedTime();
                    while (opModeIsActive() && !isStopRequested()) {
                        if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
                            break pickupLoop;
                        }
                        if (intakeTimer.seconds() > 5) {
                            break;
                        }
                    }
                }

                //Move to white line
                driveSystem.moveToPosition(-1, 30, gps.getRot(), 1, new DistanceTimeoutWarning(100));


                //Move out of warehouse
                driveSystem.moveToPosition(1, 70, 180, 2, new DistanceTimeoutWarning(100));

                //Move to drop off location
                driveSystem.moveToPosition(26, 82.5, 272, .5, new DistanceTimeoutWarning(100));

                dropOffFreight();

            }

        }
    }
}


