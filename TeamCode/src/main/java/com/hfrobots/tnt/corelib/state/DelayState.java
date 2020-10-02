package com.hfrobots.tnt.corelib.state;


import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.corelib.state.State;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class DelayState extends State {
    private long startTime = 0;
    private long thresholdTimeMs;

    public DelayState(String name, Telemetry telemetry, long val, TimeUnit unit) {
        super(name, telemetry);
        thresholdTimeMs = unit.toMillis(val);
    }

    public DelayState(String name, Telemetry telemetry, long numberOfSeconds) {
        super(name, telemetry);
        thresholdTimeMs = TimeUnit.SECONDS.toMillis(numberOfSeconds);
    }

    @Override
    public void resetToStart() {
        startTime = 0;
    }

    @Override
    public void liveConfigure(NinjaGamePad gamePad) {

    }

    @Override
    public State doStuffAndGetNextState() {
        if (startTime == 0) {
            startTime = System.currentTimeMillis();
            return this;
        }

        long now = System.currentTimeMillis();
        long elapsedMs = now - startTime;

        if (elapsedMs > thresholdTimeMs) {
            return nextState;
        }

        if (telemetry != null) {
            telemetry.addData("04", "Delay: %s %d of %d ms", name, elapsedMs, thresholdTimeMs);
        }

        return this;
    }
}
