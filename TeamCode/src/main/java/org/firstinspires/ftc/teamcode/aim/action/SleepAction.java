package org.firstinspires.ftc.teamcode.aim.action;

public class SleepAction implements Action {
    private final long timeout;
    private long expiryTime;

    /**
     * @param expiryTime The absolute time (in milliseconds since epoch) at which this action expires.
     *                   For example, System.currentTimeMillis() + 5000 would create an action
     *                   that expires 5 seconds from now.
     */
    public SleepAction(long millSecs) {
        this.timeout = millSecs;
        this.expiryTime = 0;
    }

    /**
     * Checks whether the current system time has passed the expiryTime.
     *
     * @return true  if the current time is greater than or equal to the expiryTime
     *         false otherwise
     */
    @Override
    public boolean run() {
        if (this.expiryTime == 0) {
            this.expiryTime = System.currentTimeMillis() + this.timeout;
        }
        long currentTime = System.currentTimeMillis();
        return (currentTime >= expiryTime);
    }
}
