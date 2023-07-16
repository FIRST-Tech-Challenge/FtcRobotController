package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Core.HWMap;


public class Blinkin extends HWMap {

    public Blinkin(Telemetry telemetry, HardwareMap hardwareMap){
        super(telemetry, hardwareMap);
    }

    public RevBlinkinLedDriver blinkin;
    public boolean slideDisplay = false;

    public void setColor() {
        if (gripper.getPosition() == 0.75 && !slideDisplay) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        } else if (!slideDisplay) {
            blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE);
        } else {
            if (LinearSlides.target == LinearSlides.Ls.NORM.level|| LinearSlides.target == LinearSlides.Ls.IN_CONE.level) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE);
            } else if (LinearSlides.target == LinearSlides.Ls.LOW.level) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else if (LinearSlides.target == LinearSlides.Ls.MEDIUM.level) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIME);
            } else if (LinearSlides.target == LinearSlides.Ls.HIGH.level) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
            } else if (!LinearSlides.automation) {
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
            }
        }
    }
}
