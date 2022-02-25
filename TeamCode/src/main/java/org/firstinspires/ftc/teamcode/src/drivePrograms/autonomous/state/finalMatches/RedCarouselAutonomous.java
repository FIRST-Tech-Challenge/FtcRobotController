package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state.finalMatches;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Red side near spinner for State
 */
@Autonomous(name = "Red State Carousel Autonomous Finals Match")
public class RedCarouselAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;
    private final boolean overBarrier = true;


    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6, 111, 180);

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
            Thread.sleep(4000);

            driveSystem.moveToPosition(24, 82.5, 272, .5, new DistanceTimeoutWarning(500));

            dropOffFreight(Pos);

            driveSystem.moveToPosition(10, 144, gps.getRot(), 1, new DistanceTimeoutWarning(100));
            //This moves into the wall for duck spinning
            driveSystem.moveToPosition(0, gps.getY() + 5, gps.getRot(), 1, new DistanceTimeoutWarning(100));

            driveSystem.stopAll();
            spinner.spinOffRedDuck();

            driveSystem.strafeAtAngle(180, 0.5);
            double[] initialPos = gps.getPos();
            double currentWallDistance;
            do {
                if (MiscUtils.distance(initialPos[0], initialPos[1], gps.getX(), gps.getY()) > 24) {
                    break;
                }
                currentWallDistance = Math.abs((frontDistanceSensor.getDistance(DistanceUnit.INCH)) * Math.cos(Math.toRadians(gps.getRot() - 270)));
            } while (currentWallDistance < (24) && opModeIsActive() && !isStopRequested());
            driveSystem.stopAll();
        }
    }
}
