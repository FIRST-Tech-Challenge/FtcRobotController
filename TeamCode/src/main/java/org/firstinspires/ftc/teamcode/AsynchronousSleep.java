package org.firstinspires.ftc.teamcode;

public class AsynchronousSleep {
    private boolean waits = false;
    private long startTime = -1;
    private long ms = -1;

    public void AsynchronousSleep() {}

    public void wait(int ms) {
        if (!waits) {
            startTime = System.currentTimeMillis();
            waits = true;
            this.ms = ms;
        }
    }

    public void update() {
        waits = ((System.currentTimeMillis() - startTime) <= ms);
    }

    public boolean isReady() {
        return !waits && startTime > 0;
    }

    public void reset() {
        waits = false;
        startTime = -1;
    }

    public boolean isBusy() {
        return waits;
    } // is waiting?
}
