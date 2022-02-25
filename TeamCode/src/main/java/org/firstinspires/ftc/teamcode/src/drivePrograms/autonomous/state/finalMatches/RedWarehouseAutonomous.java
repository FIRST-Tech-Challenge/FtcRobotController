package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state.finalMatches;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.MovementWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near Warehouse for State
 */
@Autonomous(name = "Red State Warehouse Autonomous Finals Match")
public class RedWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;
    final ElapsedTime OpModeTimer = new ElapsedTime();

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

        // get rid of this once camera position working
        double yOffset = 0;

        tfod.shutdown();
        vuforia.close();

        driveSystem.setTurnWhileStrafe(true);
        driveSystem.debugOn();

        double distanceDriven = 0;

        OpModeTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {

            driveSystem.moveTowardsPosition(17, 80.5 + yOffset, 270, 1, 5, new MovementWarning());

            driveSystem.moveToPosition(19, 82.5 + yOffset, 270, 1, new DistanceTimeoutWarning(100));
            driveSystem.turnTo(270, 0.1);
            {
                double xDistance = (frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6) * Math.cos(Math.toRadians(gps.getRot() - 270));
                gps.setPos(xDistance, gps.getY(), gps.getRot());
            }


            dropOffFreight(Pos, -2);

            Pos = BarcodePositions.Right;

            //Move against the wall
            driveSystem.moveTowardsPosition(10, 65, 180, 2, 5, new DistanceTimeoutWarning(100));

            //Through Barrier
            driveSystem.moveTowardsPosition(4, 40, 180, 2, 5, new DistanceTimeoutWarning(100));

            driveSystem.moveToPosition(gps.getX(), 30, 180, 1, new DistanceTimeoutWarning(500));

            //Update position with known coordinates 6 in is the distance from the distance sensor to the center of the robot
            gps.setPos(gps.getX(), frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6, gps.getRot());

            intake.setIntakeOn();


            //Continue strafing while a item is not in the bucket
            double startingDistanceFromWall = frontDistanceSensor.getDistance(DistanceUnit.INCH);


            // distanceDriven = pickUpBlock(distanceDriven, startingDistanceFromWall);
            distanceDriven = pickUpBlock(distanceDriven, startingDistanceFromWall, false);

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
            driveSystem.moveToPosition(gps.getX(), 36, gps.getRot(), 1, new DistanceTimeoutWarning(500));

            //Update position with known coordinates
            gps.setPos(6.5, frontDistanceSensor.getDistance(DistanceUnit.INCH), gps.getRot());

            intake.setIntakeOff();

            //Runtime Check

            if (OpModeTimer.seconds() > 25) {
                driveSystem.strafeAtAngle(0, 0.5);
                Thread.sleep(500);
                driveSystem.stopAll();
                intake.setIntakeReverse();
                Thread.sleep(6000);
                return;
            }


            //Move out of warehouse
            driveSystem.moveToPosition(gps.getX() + 1, 70, 180, 2, new DistanceTimeoutWarning(500));
            yOffset = -6;
        }


    }
}


