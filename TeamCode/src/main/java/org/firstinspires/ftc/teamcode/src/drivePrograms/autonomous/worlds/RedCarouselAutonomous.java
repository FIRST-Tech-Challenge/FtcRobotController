package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtils;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

/**
 * The Autonomous ran on Red side near spinner for State
 */
@SuppressWarnings("unused")
@Autonomous(name = "🟥🦆Red Worlds Carousel Autonomous🦆🟥")
public class RedCarouselAutonomous extends AutonomousTemplate {
    static final BlinkinPattern def = BlinkinPattern.RED;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(6, 111, 180);


        driveSystem.setTurnWhileStrafe(true);
        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {
            driveSystem.debugOn();

            driveSystem.moveToPosition(24, 82.5, 272, .5, new DistanceTimeoutWarning(500));

            dropOffFreight(BarcodePositions.Right);

            driveSystem.moveToPosition(10, 144, gps.getRot(), 1, new DistanceTimeoutWarning(100));

            //This moves into the wall for duck spinning
            driveSystem.moveToPosition(0, gps.getY() + 5, gps.getRot(), 1, new DistanceTimeoutWarning(100));

            driveSystem.halt();

            spinner.spinOffRedDuck();

            driveSystem.strafeAtAngle(180, 0.5);
            double[] initialPos = gps.getPos();
            double currentWallDistance;
            do {
                double[] currentPos = gps.getPos();
                if (MiscUtils.distance(initialPos[0], initialPos[1], currentPos[0], currentPos[1]) > 24) {
                    break;
                }
                currentWallDistance = Math.abs((frontDistanceSensor.getDistance(DistanceUnit.INCH)) * Math.cos(Math.toRadians(gps.getRot() - 90)));
                telemetry.addData("currentWallDistance",currentWallDistance);
                telemetry.update();
                currentWallDistance = 0;
                Thread.yield();
            } while (currentWallDistance < (24) && opModeIsActive() && !isStopRequested());

            driveSystem.halt();
        }
    }
}
