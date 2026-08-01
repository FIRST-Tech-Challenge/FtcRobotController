package org.firstinspires.ftc.teamcode.common.autonomous;

/**
 * The small public robot API needed by baseline timed autonomous steps.
 *
 * <p>Team-specific robot compositions implement this interface, keeping shared autonomous steps
 * independent of Team A without introducing a broad abstraction hierarchy.</p>
 */
public interface AutonomousRobotControl {
    void drive(double forward, double strafe, double rotate);
    void enableManualDrive();
    void disableDrive();
    void startIntake();
    void stopIntake();
}
