package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Subsystem;

public class Lights extends Subsystem {
    private final RevBlinkinLedDriver ledDriver;

    public Lights(RevBlinkinLedDriver ledDriver) {
        this.ledDriver = ledDriver;
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
    }

    public void drivingToBackdropPattern() {
        ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        ledDriver.setPattern(pattern);
    }

}
