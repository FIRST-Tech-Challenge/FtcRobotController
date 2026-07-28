package org.firstinspires.ftc.teamcode.common.autonomous;

/** One non-blocking action in an {@link AutoSequence}. */
public interface AutoStep {
    void start();
    void update();
    boolean isFinished();
    void stop();
    String getName();
}
