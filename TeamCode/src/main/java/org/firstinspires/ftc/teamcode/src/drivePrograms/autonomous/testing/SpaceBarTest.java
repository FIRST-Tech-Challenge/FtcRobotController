package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.GenericOpModeTemplate;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.TeleOpTemplate;

@TeleOp(name = "SpaceBarTest")
public class SpaceBarTest extends TeleOpTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initLEDS();
        this.initSpaceBar();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (spaceBar.isPressed()) {
                leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
            } else {
                leds.setPattern(GenericOpModeTemplate.LEDErrorColor);
            }
            Thread.yield();
        }
    }
}
