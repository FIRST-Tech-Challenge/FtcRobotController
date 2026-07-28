package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

/** A non-blocking wait that completes after its configured duration. */
public class WaitStep implements AutoStep {
    private final double durationSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started;

    public WaitStep(double durationSeconds) {
        validateDuration(durationSeconds);
        this.durationSeconds = durationSeconds;
    }

    @Override public void start() { timer.reset(); started = true; }
    @Override public void update() { }
    @Override public boolean isFinished() { return started && timer.seconds() >= durationSeconds; }
    @Override public void stop() { }
    @Override public String getName() { return "Wait"; }

    static void validateDuration(double durationSeconds) {
        if (Double.isNaN(durationSeconds) || Double.isInfinite(durationSeconds)
                || durationSeconds < 0.0) {
            throw new IllegalArgumentException("An autonomous duration must be a non-negative number.");
        }
    }
}
