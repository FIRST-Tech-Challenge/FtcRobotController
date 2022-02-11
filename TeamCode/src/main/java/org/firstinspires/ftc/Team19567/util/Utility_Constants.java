package org.firstinspires.ftc.Team19567.util;

public final class Utility_Constants {
    public static final double PPR_RATIO = 1.95509087; //ratio of old encoder values to current encoder values
    public static final double INTAKE_SPEED = 1.0; //speed to set the intake motor to
    public static final double TURN_SENSITIVITY = 0.9; //driver-controlled sensitivity for turning
    public static final double STRAFE_SENSITIVITY = 1.0; //driver-controlled sensitivity for strafing
    public static final double MAX_SENSITIVITY = 1.0; //driver-controller overall sensitivity, basically
    public static final double SLOWMODE_MULT = 0.3; //speed multiplier during SlowMode
    //
    public static final double _firstLevelPos = 870; //original encoder values for first level of alliance hub/shared hub
    public static final double _secondLevelPos = 750; //original encoder values for second level of alliance hub/shared hub
    public static final double _thirdLevelPos = 600; //original encoder values for third level of alliance hub/shared hub
    //Tune the previous values, not these values
    public static final int FIRST_LEVEL_POS = (int)(_firstLevelPos*PPR_RATIO);
    public static final int SECOND_LEVEL_POS = (int)(_secondLevelPos*PPR_RATIO);
    public static final int THIRD_LEVEL_POS = (int)(_thirdLevelPos*PPR_RATIO);
    //
    public static final int MILLI_ACC = 1300; //Milliseconds after carousel has been engaged to begin accelerating the carousel
    public static final int MILLI_FINAL = 2000; //Milliseconds after carousel has been engaged to reach the final speed
    public static final int MILLI_END = 2500; //Milliseconds after carousel has been engaged to stop spinning the carousel
    public static final int DEBOUNCE_TIME = 100; //Minimum duration in milliseconds between certain presses (e.g. SlowMode, Carousel Engagement)
    public static final double INIT_POWER = 0.6; //Initial power for carousel
    public static final double ACC_COEFFICIENT = 50000; //Rate at which to accelerate by (reciprocal of)
    public static final double FINAL_POWER = 1.0; //Final power for carousel
    public static final double FLICKER_TIME = 750; //Time in milliseconds for the flicker to release one freight
}