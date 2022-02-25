package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.state.finalMatches;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;

/**
 * The Autonomous ran on Blue side near spinner for State
 */
@Autonomous(name = "Blue State Carousel Autonomous Finals Match")
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
            Thread.sleep(4000);

            driveSystem.moveToPosition(144 - 24, 82.5 - 4, 90, .5, new DistanceTimeoutWarning(500));

            dropOffFreight(Pos);

            driveSystem.moveToPosition(144 - 20, 144, gps.getRot(), 1, new DistanceTimeoutWarning(100));
            //This moves into the wall for duck spinning
            driveSystem.moveToPosition(140, gps.getY(), gps.getRot(), 1, new DistanceTimeoutWarning(100));

            driveSystem.stopAll();
            spinner.spinOffBlueDuck();

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
