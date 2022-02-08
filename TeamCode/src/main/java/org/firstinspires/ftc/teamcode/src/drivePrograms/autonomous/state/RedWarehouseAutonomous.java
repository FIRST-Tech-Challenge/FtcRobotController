package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.MovementWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.enums.FreightFrenzyGameObject;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near Warehouse for State
 */
@Autonomous(name = "Red State Warehouse Autonomous")
public class RedWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6.5, 64, 180);


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
        double yOffset = 0;
        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();

            driveSystem.setTurnWhileStrafe(true);
            driveSystem.debugOn();

            while (opModeIsActive() && !isStopRequested()) {

                driveSystem.moveTowardsPosition(20, 82.5 + yOffset, 270, 1, 5, new MovementWarning());
                driveSystem.moveToPosition(20, 82.5 + yOffset, 270, 1, new DistanceTimeoutWarning(500));

                driveSystem.stopAll();

                {
                    double xDistance = (frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6) * Math.cos(Math.toRadians(gps.getRot() - 270));
                    gps.setPos(xDistance, gps.getY(), gps.getRot());
                }

                dropOffFreight(Pos);

                Pos = BarcodePositions.Right;

                //Move against the wall
                driveSystem.moveTowardsPosition(12, 70, 180, 2, 5, new DistanceTimeoutWarning(100));

                //Through Barrier
                driveSystem.moveToPosition(-1, 30, gps.getRot(), 1, new DistanceTimeoutWarning(500));

                //Update position with known coordinates 6 in is the distance from the distance sensor to the center of the robot
                gps.setPos(6.5, frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6, gps.getRot());

                //Move away from the wall
                driveSystem.strafeAtAngle(270, 0.5);
                Thread.sleep(50);
                driveSystem.stopAll();

                intake.setIntakeOn();


                //Continue strafing while a item is not in the bucket
                double startingDistanceFromWall = frontDistanceSensor.getDistance(DistanceUnit.INCH);
                double distanceDriven = 0;

                outer:
                while (opModeIsActive() && !isStopRequested()) {
                    distanceDriven += 4;
                    driveSystem.strafeAtAngle(0, 0.3);
                    while (opModeIsActive() && !isStopRequested()) {
                        if (frontDistanceSensor.getDistance(DistanceUnit.INCH) < (startingDistanceFromWall - distanceDriven)) {
                            break;
                        }
                        if (intake.identifyContents() != FreightFrenzyGameObject.EMPTY) {
                            break outer;
                        }
                    }
                    driveSystem.stopAll();
                    ElapsedTime time = new ElapsedTime();
                    while ((time.seconds() < 1.5) && (opModeIsActive() && !isStopRequested())) {
                        if ((intake.identifyContents() != FreightFrenzyGameObject.EMPTY)) {
                            break outer;
                        }
                    }
                }

                driveSystem.stopAll();

                intake.setIntakeReverse();

                driveSystem.moveToPosition(gps.getX(), startingDistanceFromWall, 180, 1, new DistanceTimeoutWarning(1000));

                driveSystem.turnTo(180, 0.5);


                //Move to white line and against the wall
                driveSystem.moveToPosition(-1, 36, gps.getRot(), 1, new DistanceTimeoutWarning(1000));

                //Update position with known coordinates
                gps.setPos(6.5, frontDistanceSensor.getDistance(DistanceUnit.INCH), gps.getRot());

                intake.setIntakeOff();

                //Runtime Check
                if (this.getRuntime() > 25) {
                    driveSystem.strafeAtAngle(0, 1);
                    Thread.sleep(50);
                    return;
                }

                //Move out of warehouse
                driveSystem.moveToPosition(gps.getX() + 1, 70, 180, 2, new DistanceTimeoutWarning(500));
                yOffset = -2;
            }

        }
    }
}


