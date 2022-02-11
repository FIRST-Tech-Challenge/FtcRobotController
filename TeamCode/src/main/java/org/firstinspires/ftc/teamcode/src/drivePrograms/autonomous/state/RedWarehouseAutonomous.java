package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.MovementWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
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

        tfod.shutdown();
        vuforia.close();

        driveSystem.setTurnWhileStrafe(true);
        driveSystem.debugOn();

        double distanceDriven = 0;

        while (opModeIsActive() && !isStopRequested()) {

            driveSystem.moveTowardsPosition(18, 80.5 + yOffset, 270, 1, 5, new MovementWarning());

            driveSystem.moveToPosition(20, 82.5 + yOffset, 270, 1, new DistanceTimeoutWarning(100));
            driveSystem.turnTo(270, 0.1);
            {
                double xDistance = (frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6) * Math.cos(Math.toRadians(gps.getRot() - 270));
                gps.setPos(xDistance, gps.getY(), gps.getRot());
            }


            dropOffFreight(Pos);

            Pos = BarcodePositions.Right;

            //Move against the wall
            driveSystem.moveTowardsPosition(12, 70, 180, 2, 5, new DistanceTimeoutWarning(100));

            //Through Barrier
            driveSystem.moveToPosition(8, 30, 180, 1, new DistanceTimeoutWarning(500));

            //To the last place it was grabbing from
            driveSystem.moveToPosition(gps.getX(), 30 - distanceDriven, 180, 1, new DistanceTimeoutWarning(500));

            //Update position with known coordinates 6 in is the distance from the distance sensor to the center of the robot
            gps.setPos(gps.getX(), frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6, gps.getRot());

            intake.setIntakeOn();


            //Continue strafing while a item is not in the bucket
            double startingDistanceFromWall = frontDistanceSensor.getDistance(DistanceUnit.INCH);


            distanceDriven = pickupBlock(distanceDriven, startingDistanceFromWall);

            driveSystem.stopAll();

            intake.setIntakeReverse();

            //Strafes away from pile
            {
                driveSystem.strafeAtAngle(180, 0.5);
                double distanceFromWall;
                do {
                    distanceFromWall = (frontDistanceSensor.getDistance(DistanceUnit.INCH)) * Math.cos(Math.toRadians(gps.getRot()));
                    distanceFromWall = Math.abs(distanceFromWall);
                } while (distanceFromWall < 25 && (opModeIsActive() && !isStopRequested()));
            }

            driveSystem.stopAll();

            //Move to white line and against the wall
            driveSystem.moveToPosition(-1, 36, gps.getRot(), 1, new DistanceTimeoutWarning(500));
            //Runtime Check
            if (this.getRuntime() > 25) {
                return;
            }


            //Update position with known coordinates
            gps.setPos(6.5, frontDistanceSensor.getDistance(DistanceUnit.INCH), gps.getRot());

            intake.setIntakeOff();

            //Runtime Check

            if (this.getRuntime() > 25) {
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(500);
                return;
            }


            //Move out of warehouse
            driveSystem.moveToPosition(gps.getX() + 1, 70, 180, 2, new DistanceTimeoutWarning(500));
            yOffset = -6;
        }


    }
}


