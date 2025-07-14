package ftc_library.autonomous;

import ftc_library.hardware.RobotHardware;

/**
 * BaseAutonomous provides utility methods for timed movements in autonomous mode.
 * Extend this class for your custom autonomous routines.
 */
public class BaseAutonomous {
    protected final RobotHardware hw;

    public BaseAutonomous(RobotHardware hardware) {
        this.hw = hardware;
    }

    /**
     * Moves the robot in the X (strafe) direction for a set time.
     * @param power Power to apply (-1.0 to 1.0)
     * @param ms    Duration in milliseconds
     */
    public void moveXForTime(double power, long ms) throws InterruptedException {
        hw.moveX(power);
        Thread.sleep(ms);
        hw.stopMoving();
    }

    /**
     * Moves the robot in the Y (forward/back) direction for a set time.
     * @param power Power to apply (-1.0 to 1.0)
     * @param ms    Duration in milliseconds
     */
    public void moveYForTime(double power, long ms) throws InterruptedException {
        hw.moveY(power);
        Thread.sleep(ms);
        hw.stopMoving();
    }

    /**
     * Stops all drive motors.
     */
    public void stopMoving() {
        hw.stopMoving();
    }
}
