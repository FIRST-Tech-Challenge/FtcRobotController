package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.org.rustlib.commandsystem.Command;
import org.firstinspires.ftc.teamcode.subsystems.Lights;

public class FlashLights extends Command {

    private final Lights lights;
    private final int period;
    private static final int minPeriod = 1000; // The maximum allowable frequency is 1 Hz
    private final RevBlinkinLedDriver.BlinkinPattern pattern;

    public FlashLights(Lights lights, int period, RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.lights = lights;
        this.period = Math.max(minPeriod, period);
        this.pattern = pattern;
        addRequirements(lights);
    }

    public FlashLights(Lights lights, int period) {
        this(lights, period, RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    @Override
    public void execute() {
        if (Math.sin(timeSinceInitialized() * 2 * Math.PI / period) > 0) {
            lights.setPattern(pattern);
        } else {
            lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
