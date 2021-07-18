package com.bravenatorsrobotics.core;

import com.bravenatorsrobotics.drive.AbstractDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * The type Robot specifications is used to specify the robot's physical shape
 * for the Operation Mode.
 */
public class RobotSpecifications {

    /**
     * The name of the robot motors.
     */
    public final String[] robotMotors;
    /**
     * The selected drive type. (eg. four wheel, two wheel).
     */
    public final Class<? extends AbstractDrive> driveType;

    /**
     * How many ticks it takes to turn the motor once
     */
    public final double ticksPerMotorRev;
    /**
     * The gearing between the motor and the wheel. (Don't count the in motor gearing).
     */
    public final double driveGearReduction;
    /**
     * The Wheel diameter inches.
     */
    public final double wheelDiameterInches;
    /**
     * The Pivot diameter inches. (Distance between wheels on the same side from axle to axle).
     */
    public final double pivotDiameterInches;

    /**
     * If the drive system should use velocity. (Recommended).
     */
    public boolean useVelocity = false;
    /**
     * The maximum amount of velocity the drive system motors can spin at. (Run the max velocity test to figure out).
     */
    public int maxVelocity; // Run the max velocity test to figure out

    /**
     * The Zero power behavior.
     */
    public DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    /**
     * Instantiates a new Robot specifications.
     *
     * @param robotMotors         the name of the robot motors
     * @param driveType           the selected drive type. (eg. four wheel, two wheel)
     * @param ticksPerMotorRev    how many ticks it takes to turn the motor once
     * @param driveGearReduction  the gearing between the motor and the wheel. (Don't count the in motor gearing)
     * @param wheelDiameterInches the wheel diameter inches
     * @param pivotDiameterInches the pivot diameter inches  (Distance between wheels on the same side from axle to axle)
     */
    public RobotSpecifications(String[] robotMotors, Class<? extends AbstractDrive> driveType,
                               double ticksPerMotorRev, double driveGearReduction, double wheelDiameterInches, double pivotDiameterInches) {
        this.robotMotors = robotMotors;
        this.driveType = driveType;

        this.ticksPerMotorRev = ticksPerMotorRev;
        this.driveGearReduction = driveGearReduction;
        this.wheelDiameterInches = wheelDiameterInches;
        this.pivotDiameterInches = pivotDiameterInches;
    }
}
