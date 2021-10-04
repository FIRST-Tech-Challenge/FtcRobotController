package org.firstinspires.ftc.teamcode.core.movement.api;

public interface Movement {
    /**
     * Drives with a certain power on each of the wheels.
     * @param frontLeftPower power for the front left wheel
     * @param frontRightPower power for the front right wheel
     * @param backRightPower power for the back right wheel.
     * @param backLeftPower power for the back left wheel.
     */
    void drivePower(double frontLeftPower, double frontRightPower, double backRightPower, double backLeftPower);
}
