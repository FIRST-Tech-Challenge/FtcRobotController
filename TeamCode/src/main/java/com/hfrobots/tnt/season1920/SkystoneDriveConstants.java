package com.hfrobots.tnt.season1920;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.hfrobots.tnt.corelib.drive.mecanum.DriveConstants;

@Config
public class SkystoneDriveConstants extends DriveConstants {

    private static DriveConstraints DRIVE_CONSTRAINTS = new DriveConstraints(80, 30.0, 0.0,
            Math.toRadians(180.0), Math.toRadians(180.0), 0.0);
    private static PIDCoefficients TRANSLATIONAL_PID_COEFFICIENTS = new PIDCoefficients(4.2D, 0, 0);
    private static PIDCoefficients HEADING_PID_COEFFICIENTS = new PIDCoefficients(0.295D, 0, 0);

    @Override
    public DriveConstraints getBaseConstraints() {
        return DRIVE_CONSTRAINTS;
    }

    @Override
    public PIDCoefficients getTranslationalPID() {
        return TRANSLATIONAL_PID_COEFFICIENTS;
    }

    @Override
    public PIDCoefficients getHeadingPid() {
        return HEADING_PID_COEFFICIENTS;
    }
}
