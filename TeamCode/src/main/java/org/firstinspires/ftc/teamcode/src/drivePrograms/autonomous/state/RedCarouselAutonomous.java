package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceTimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near spinner for Qualifier
 */
@Autonomous(name = "Red State Carousel Autonomous")
public class RedCarouselAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;
    private final boolean overBarrier = false;
    public DistanceSensor distanceSensor;


    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6, 111, 180);
        distanceSensor = (DistanceSensor) hardwareMap.get("distance_sensor");

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

            driveSystem.moveToPosition(26, 82.5, 272, .5, new DistanceTimeoutWarning(500));

            switch (Pos) {
                case NotSeen:
                case Right:
                    // got to the top level when right
                    slide.setTargetLevel(HeightLevel.TopLevel);
                    Thread.sleep(1000);
                    driveSystem.strafeAtAngle(180, .2);
                    Thread.sleep(1000);
                    driveSystem.stopAll();
                    intake.setServoOpen();
                    Thread.sleep(750);
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

            driveSystem.moveToPosition(10, 144, gps.getRot(), 1, new DistanceTimeoutWarning(100));
            //This moves into the wall for duck spinning
            driveSystem.moveToPosition(0, gps.getY() + 5, gps.getRot(), 1, new DistanceTimeoutWarning(100));

            driveSystem.stopAll();
            spinner.spinOffRedDuck();

            driveSystem.moveTowardsPosition(gps.getX() + 10, gps.getY() - 20, gps.getRot(), 1, 5, new DistanceTimeoutWarning(500));

            if (!overBarrier) {
                //Through crack
                driveSystem.moveTowardsPosition(0, 80, 180, 1, 1, new DistanceTimeoutWarning(100));

                intake.setIntakeOn();

                try {
                    driveSystem.moveToPosition(0, 10, 180, 1, new MovementException[]{new DistanceSensorException(distanceSensor), new DistanceTimeoutException(500)});
                } catch (MovementException e) {
                    if (gps.getY() > 20) {
                        driveSystem.moveToPosition(0, 10, gps.getRot(), 1, new DistanceTimeoutWarning(100));
                    }
                }

                intake.setIntakeOff();


            } else {
                // Over Barrier
                driveSystem.moveToPosition(33, 77, 180, 2, new DistanceTimeoutWarning(100));
                podServos.raise();
                driveSystem.strafeAtAngle(0, 1);
                Thread.sleep(2500);
                driveSystem.stopAll();
            }
        }
    }
}
