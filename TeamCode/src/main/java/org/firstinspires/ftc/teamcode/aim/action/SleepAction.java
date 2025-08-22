package org.firstinspires.ftc.teamcode.aim.action;

public class SleepAction extends Action {
    private final long timeout;
    private long expiryTime;

    public SleepAction(String name, long millSecs) {
        super(name);
        this.timeout = millSecs;
        this.expiryTime = 0;
    }

    @Override
    public boolean run() {
        if (this.expiryTime == 0) {
            this.expiryTime = System.currentTimeMillis() + this.timeout;
        }
        long currentTime = System.currentTimeMillis();
        return (currentTime >= expiryTime);
    }
}
