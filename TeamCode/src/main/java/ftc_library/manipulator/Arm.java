package ftc_library.manipulator;

import ftc_library.hardware.RobotHardware;

/**
 * Arm controls both vertical and horizontal movement of a two-axis arm mechanism.
 */
public class Arm {
    private final RobotHardware hw;

    public Arm(RobotHardware hardware) {
        this.hw = hardware;
    }

    /**
     * Moves the vertical arm up or down.
     * @param power Positive for up, negative for down
     */
    public void moveVertical(double power) {
        hw.verticalArm.setPower(RobotHardware.armPowerCurve(power));
    }

    /**
     * Moves the horizontal arm forward or backward.
     * @param power Positive for forward, negative for backward
     */
    public void moveHorizontal(double power) {
        hw.horizontalArm.setPower(RobotHardware.armPowerCurve(power));
    }

    /**
     * Stops both arm motors.
     */
    public void stop() {
        hw.verticalArm.setPower(0);
        hw.horizontalArm.setPower(0);
    }
}
