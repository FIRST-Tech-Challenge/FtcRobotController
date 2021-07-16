package com.bravenatorsrobotics.core;

import com.bravenatorsrobotics.drive.AbstractDrive;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotSpecifications {

    public final String[] robotMotors;
    public final Class<? extends AbstractDrive> driveType;

    public final double ticksPerMotorRev;
    public final double driveGearReduction;
    public final double wheelDiameterInches;

    public boolean useVelocity = false;
    public int maxVelocity; // Run the max velocity test to figure out

    public DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    public RobotSpecifications(String[] robotMotors, Class<? extends AbstractDrive> driveType,
                               double ticksPerMotorRev, double driveGearReduction, double wheelDiameterInches) {
        this.robotMotors = robotMotors;
        this.driveType = driveType;

        this.ticksPerMotorRev = ticksPerMotorRev;
        this.driveGearReduction = driveGearReduction;
        this.wheelDiameterInches = wheelDiameterInches;
    }
}
