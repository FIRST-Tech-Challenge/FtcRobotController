package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

/**
 * The Autonomous ran on Blue side near spinner for State
 */
@Autonomous(name = "Blue State Carousel Autonomous")
public class BlueCarouselAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    private final boolean overBarrier = true;


    @Override
    public void opModeMain() throws InterruptedException {
        this.CameraNameToUse = GenericOpModeTemplate.RightWebcamName;
        this.initAll();
        leds.setPattern(def);
        gps.setPos(144 - 6, 111, 180);

        BarcodePositions Pos;
        do {
            Pos = this.findPositionOfMarker();
            telemetry.addData("Position", Pos);
            telemetry.update();
            Thread.sleep(200);
        } while (!isStarted() && !isStopRequested());

        driveSystem.setTurnWhileStrafe(true);
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            tfod.shutdown();
            vuforia.close();
            driveSystem.debugOn();

            driveSystem.moveToPosition(144 - 24, 82.5 - 4, 90, .5, new DistanceTimeoutWarning(500));

            dropOffFreight(Pos);

            driveSystem.moveToPosition(144 - 20, 144, gps.getRot(), 1, new DistanceTimeoutWarning(100));
            //This moves into the wall for duck spinning
            driveSystem.moveToPosition(140, gps.getY(), gps.getRot(), 1, new DistanceTimeoutWarning(100));

            driveSystem.stopAll();
            spinner.spinOffBlueDuck();

            driveSystem.moveTowardsPosition(gps.getX() - 4, gps.getY() - 20, 180, 1, 5, new DistanceTimeoutWarning(500));

            if (!overBarrier) {
                //Through crack
                driveSystem.moveTowardsPosition(144 - 0, 80, 180, 1, 1, new DistanceTimeoutWarning(100));
                driveSystem.moveToPosition(144 - 0, 30, 180, 1, new DistanceTimeoutWarning(100));

               /*
                intake.setIntakeOn();


                try {
                    driveSystem.moveToPosition(144 - 0, 10, 180, 1, new MovementException[]{new DistanceSensorException(intakeDistanceSensor, 8), new DistanceTimeoutException(100)});
                } catch (MovementException e) {
                    if (gps.getY() > 20) {
                        driveSystem.moveToPosition(144 - 0, 10, gps.getRot(), 1, new DistanceTimeoutWarning(100));
                    }
                }

                */


                intake.setIntakeOff();


            } else {
                // Over Barrier
                //driveSystem.moveTowardsPosition(32, 77, 180, 1, 3, new DistanceTimeoutWarning(100));
                driveSystem.moveToPosition(144 - 23.5, 65, 180, 2.5, new DistanceTimeoutWarning(500));
                driveSystem.newTurnToPrototype(180, .2, 0, false);
                podServos.raise();
                double tmp = frontDistanceSensor.getDistance(DistanceUnit.INCH);
                double power;

                while (frontDistanceSensor.getDistance(DistanceUnit.INCH) > 6 && opModeIsActive() && !isStopRequested()) {
                    // power is calculated
                    power = driveSystem.shortMovementPowerCalculation(tmp, frontDistanceSensor.getDistance(DistanceUnit.INCH), 1.75, .5);// the max power is set to 2 to steepen the power curve at the end of the movement
                    driveSystem.strafeAtAngle(15, power);
                    telemetry.addData("power: ", power);
                    telemetry.update();
                }
                //Thread.sleep(2000);
                driveSystem.stopAll();
                Thread.sleep(200);// this is to keep the robot stopped and from possibly drifting into a wall or another robot
            }
        }
    }
}
