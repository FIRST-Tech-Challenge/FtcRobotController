package org.firstinspires.ftc.teamcode.utils;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public final class Constants {
    public static final String MAIN_GROUP = "KronBot";

    // Drivetrain
    public static double CONTROLLER_DEADZONE = 0.05;
    public static double MANUAL_SPEED = 1.0;
    public static double PATH_MAX_POWER = 0.8;
    public static double BLUE_BASKET_COMMON_X = 85.5;
    public static double BLUE_BASKET_1_Y = 53;
    public static double BLUE_BASKET_2_Y = 87;

    public static double RED_BASKET_COMMON_X = 58.5;
    public static double RED_BASKET_1_Y = 53;
    public static double RED_BASKET_2_Y = 87;

    public static double AIM_KP = 1.3;
    public static double AIM_KI = 0.2;
    public static double AIM_KD = 0.1;
    public static double AIM_DEADBAND_RAD = Math.toRadians(1.5);
    public static double AIM_INTEGRAL_LIMIT = 0.5;

    // Assisted-driving poses.
    public static double LEFT_START_X = 9.0;
    public static double RIGHT_START_X = 135.0;
    public static double START_Y = 9.0;
    public static double START_HEADING_DEG = 90.0;
    public static double CENTER_X = 72.0;
    public static double CENTER_Y = 72.0;
    public static double CENTER_HEADING_DEG = 90.0;

    // Optional intake.
    public static double INTAKE_POWER = 0.3;

    public static final int LIMELIGHT_DEFAULT_PIPELINE = 3;
    public static final int LIMELIGHT_ALTERNATE_PIPELINE = 4;

    private Constants() {}

    public static double applyDeadzone(double value) {
        double magnitude = Math.abs(value);
        if (magnitude <= CONTROLLER_DEADZONE) return 0.0;
        double normalized = (magnitude - CONTROLLER_DEADZONE)
                / (1.0 - CONTROLLER_DEADZONE);
        return Math.copySign(normalized * MANUAL_SPEED, value);
    }
}
