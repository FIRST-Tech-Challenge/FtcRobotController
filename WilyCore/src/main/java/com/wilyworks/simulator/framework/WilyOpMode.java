package com.wilyworks.simulator.framework;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.wilyworks.simulator.WilyCore;

public abstract class WilyOpMode extends OpMode {

    abstract public void runOpMode() throws InterruptedException;

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void waitForStart() {
        WilyCore.render();
        while (!isStarted())
            sleep(30);
    }

    public final boolean opModeIsActive() {
        if (WilyCore.status.state == WilyCore.State.INITIALIZED)
            WilyCore.render();
        return WilyCore.status.state != WilyCore.State.STOPPED;
    }

    public final boolean isStarted() {
        WilyCore.render();
        return WilyCore.status.state == WilyCore.State.STARTED;
    }
    public final boolean isStopRequested() {
        return WilyCore.status.state == WilyCore.State.STOPPED;
    }

}
