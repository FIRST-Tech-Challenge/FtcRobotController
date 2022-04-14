package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.worlds;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplateCV;

public abstract class WorldsAutonomousProgram extends AutoObjDetectionTemplateCV {

    protected final RevBlinkinLedDriver.BlinkinPattern defaultColor;

    protected WorldsAutonomousProgram(RevBlinkinLedDriver.BlinkinPattern defaultColor) {
        this.defaultColor = defaultColor;
    }

    @Override
    public void initAll() throws InterruptedException {
        super.initAll();
        leds.setPattern(defaultColor);
    }

    protected BarcodePositions monitorMarkerWhileWaitForStart() {

        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        BarcodePositions pos;
        do {
            pos = this.findPositionOfMarker();
            telemetry.addData("Pos", pos);
            telemetry.update();

        } while (!opModeIsActive() && !isStarted());
        waitForStart();
        leds.setPattern(defaultColor);
        return pos;
    }

    protected void dropOffItem(BarcodePositions pos) throws InterruptedException {
        switch (pos) {
            case NotSeen:
            case Right:
                slide.setTargetLevel(HeightLevel.TopLevel);
                break;

            case Center:
                slide.setTargetLevel(HeightLevel.MiddleLevel);
                break;

            case Left:
                slide.setTargetLevel(HeightLevel.BottomLevel);
                break;
        }

        //Wait for the slide to reach position
        slide.waitOn();

        outtake.open();
        Thread.sleep(2000);
        outtake.close();
    }

    protected void driveOverBarriers() throws InterruptedException {
        podServos.raise();
        Thread.sleep(1000);
        drive.setMotorPowers(1, 1, 1, 1);
        Thread.sleep(1000);
        drive.setMotorPowers(0, 0, 0, 0);
    }


}
