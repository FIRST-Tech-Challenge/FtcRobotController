package org.firstinspires.ftc.teamcode.NewStuff;

public class DrivetrainPowers {
    public final double fLeftPower;
    public final double fRightPower;
    public final double bLeftPower;
    public final double bRightPower;

    /**
     * Defines the powers needed by the various drivetrain motors
     * @param fLeftPower
     * @param fRightPower
     * @param bLeftPower
     * @param bRightPower
     */
    public DrivetrainPowers(double fLeftPower, double fRightPower, double bLeftPower, double bRightPower) {
        this.fLeftPower = fLeftPower;
        this.fRightPower = fRightPower;
        this.bLeftPower = bLeftPower;
        this.bRightPower = bRightPower;
    }
}