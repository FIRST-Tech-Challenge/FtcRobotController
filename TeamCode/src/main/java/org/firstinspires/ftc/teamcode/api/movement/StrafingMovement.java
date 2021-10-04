package org.firstinspires.ftc.teamcode.api.movement;

public interface StrafingMovement extends Movement {
    /**
     * Drives with Drive, Rotate and Strafe.
     * @param drive Movement forwards (positive) and backwards. (negative)
     * @param rotate Rotation left (positive) and right. (negative)
     * @param strafe Movement left (positive) and right. (negative)
     */
    void driveDRS(double drive, double rotate, double strafe);

    /**
     * Drives the robot forwards (positive) and backwards. (negative)
     * @param velocity speed and direction.
     */
    void drive(double velocity);

    /**
     * Rotates the robot left (positive) and right. (negative)
     * @param velocity speed and direction.
     */
    void rotate(double velocity);

    /**
     * Moves the robot left (positive) and right. (negative)
     * @param velocity speed and direction.
     */
    void strafe(double velocity);
}
