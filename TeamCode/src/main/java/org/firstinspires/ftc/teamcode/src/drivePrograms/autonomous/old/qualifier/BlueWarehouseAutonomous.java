package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.old.qualifier;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceSensorException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceTimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * The Autonomous ran on Blue side near warehouse for Qualifier
 */
@Disabled
@Autonomous(name = "Blue Warehouse Autonomous")
public class BlueWarehouseAutonomous extends AutoObjDetectionTemplate {
    static final BlinkinPattern def = BlinkinPattern.BLUE;
    static final double[] initialPos = {133.5, 55, 0};
    DistanceSensor distanceSensor;

    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        leds.setPattern(def);
        gps.setPos(initialPos[0], initialPos[1], initialPos[2]);
        distanceSensor = (DistanceSensor) hardwareMap.get("distance_sensor");

        telemetry.addData("GC", "Started");
        telemetry.update();
        System.gc();
        telemetry.addData("GC", "Finished");
        telemetry.update();

        BarcodePositions Pos;
        do {
            Pos = this.findPositionOfMarker();
            telemetry.addData("Position", Pos);
            telemetry.update();
            Thread.sleep(200);
        } while (!isStarted() && !isStopRequested());

        waitForStart();
        driveSystem.strafeAtAngle(0, .8);
        Thread.sleep(500);
        driveSystem.strafeAtAngle(270, .8);
        Thread.sleep(500);
        driveSystem.turnTo(90, .5);
        driveSystem.moveToPosition(116.5, 80, 1);
        driveSystem.turnTo(90, .4);

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
                driveSystem.strafeAtAngle(0, .5);
                Thread.sleep(500);
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

        try {
            driveSystem.moveToPosition(131, 65, 1, 1, new DistanceTimeoutException(1000));
        } catch (MovementException stop) {
            this.stop();
        }


        driveSystem.turnTo(160, 1);
        try {
            driveSystem.moveToPosition(137, 65, 1, 1, new DistanceTimeoutException(500));
        } catch (MovementException ignored) {
        }
        intake.setIntakeOn();
        try {

            driveSystem.moveToPosition(135, 7, 1, 1, new MovementException[]{new DistanceSensorException(distanceSensor, 8), new DistanceTimeoutException(500)});

        } catch (MovementException ignored) {
        } finally {
            intake.setIntakeOff();
        }



    }
}


