package org.firstinspires.ftc.teamcode;

public class Delay {
    private long currentTime;
    private final long delayDuration = 500;
    public boolean open;

    public Delay() {
        currentTime = System.currentTimeMillis();
        open = false;
    }

    public boolean delay() {
        if (System.currentTimeMillis() - currentTime >= delayDuration) {
            currentTime = System.currentTimeMillis();
            return true;
        } else {
            return false;
        }
    }
}
