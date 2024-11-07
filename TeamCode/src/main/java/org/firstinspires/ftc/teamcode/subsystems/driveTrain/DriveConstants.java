package org.firstinspires.ftc.teamcode.subsystems.driveTrain;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class DriveConstants {
    public static final double TICKS_PER_REV = 2000;

    public static final double GEAR_RATIO = 1;
    public static final double WHEEL_RADIUS = 0.045;
    public static final double WHEELS_DISTANCE = 0.19;

    public static final double METERS_PER_REV = (Math.PI * 2) * WHEEL_RADIUS;
    public static final double METERS_PER_TICK = (METERS_PER_REV / (TICKS_PER_REV * GEAR_RATIO));

    public static final double Ks = 0;

    public static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static final RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
}
