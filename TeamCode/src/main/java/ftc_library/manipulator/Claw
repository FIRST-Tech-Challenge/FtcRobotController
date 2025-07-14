package ftc_library.manipulator;

import ftc_library.hardware.RobotHardware;

/**
 * Claw provides open, close, and half-open actions for a servo-based claw.
 */
public class Claw {
    private final RobotHardware hw;

    public Claw(RobotHardware hardware) {
        this.hw = hardware;
    }

    /**
     * Opens the claw fully.
     */
    public void open() {
        hw.wrist.setPosition(1.0); // Adjust as needed for your servo
    }

    /**
     * Closes the claw fully.
     */
    public void close() {
        hw.wrist.setPosition(0.0); // Adjust as needed for your servo
    }

    /**
     * Sets the claw to a half-open position.
     */
    public void halfOpen() {
        hw.wrist.setPosition(0.5); // Adjust as needed for your servo
    }
}
