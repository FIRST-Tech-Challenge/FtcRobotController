package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.MovementWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

@Autonomous(name = "Blue State Warehouse Autonomous")
public class BlueWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    final ElapsedTime OpModeTimer = new ElapsedTime();

    @Override
    public void opModeMain() throws InterruptedException {
        this.CameraNameToUse = GenericOpModeTemplate.RightWebcamName;
        this.initAll();
        leds.setPattern(def);
        gps.setPos(144 - 6.5, 64, 180);


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

        double yOffset = 0;

        tfod.shutdown();
        vuforia.close();

        driveSystem.setTurnWhileStrafe(true);
        driveSystem.debugOn();

        double distanceDriven = 0;

        OpModeTimer.reset();
        while (opModeIsActive() && !isStopRequested()) {

            driveSystem.moveTowardsPosition(144 - 16, 80.5 - 4 + yOffset, 90, 1, 5, new MovementWarning());

            driveSystem.moveToPosition(144 - 20, 81.5 + yOffset, 90, 1, new DistanceTimeoutWarning(100));
            driveSystem.newTurnToPrototype(90, .2, .1, false);
            {
                double xDistance = (frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6) * Math.cos(Math.toRadians(gps.getRot() - 90));
                gps.setPos(144 - xDistance, gps.getY(), gps.getRot());
            }


            dropOffFreight(Pos);

            Pos = BarcodePositions.Right;


            //driveSystem.newTurnToPrototype(180, 1, .2, false);

            //Move against the wall
            driveSystem.moveTowardsPosition(144 - 10, 65, 180, 1, 5, new DistanceTimeoutWarning(100));

            //driveSystem.moveTowardsPosition(144 -7, 60, 180, 1, 5, new DistanceTimeoutWarning(100));

            //Through Barrier
            driveSystem.moveTowardsPosition(144 - 4, 40, 180, 1, 4, new DistanceTimeoutWarning(100));


            //To the last place it was grabbing from
            driveSystem.moveToPosition(gps.getX(), 30 - distanceDriven, 180, 1, new DistanceTimeoutWarning(500));

            //Update position with known coordinates 6 in is the distance from the distance sensor to the center of the robot
            // gps.setPos(gps.getX(), frontDistanceSensor.getDistance(DistanceUnit.INCH) + 6, gps.getRot());

            intake.setIntakeOn();


            //Continue strafing while a item is not in the bucket
            double startingDistanceFromWall = frontDistanceSensor.getDistance(DistanceUnit.INCH); //first time 22


            distanceDriven = pickUpBlock(distanceDriven, startingDistanceFromWall, true);

            driveSystem.stopAll();

            intake.setIntakeReverse();
            Thread.sleep(500);

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

            //Update position with known coordinates
            gps.setPos(144 - 6.5, frontDistanceSensor.getDistance(DistanceUnit.INCH), gps.getRot());

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
            driveSystem.moveToPosition(gps.getX(), 64, 180, 2, new DistanceTimeoutWarning(500));

            yOffset = -6;
        }
    }
}
