package org.firstinspires.ftc.Team19567.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class Utility_Constants {
    //GENERAL_CONSTANTS
    public static double PPR_RATIO = 1.95509087; //ratio of old encoder values to current encoder values
    public static double INTAKE_SPEED = 1.0; //speed to set the intake motor to
    //MULTIPLIERS
    public static double TURN_SENSITIVITY = 0.9; //driver-controlled sensitivity for turning
    public static double STRAFE_SENSITIVITY = 1.0; //driver-controlled sensitivity for strafing
    public static double MAX_SENSITIVITY = 1.0; //driver-controller overall sensitivity, basically
    public static double SLOWMODE_MULT = 0.3; //speed multiplier during SlowMode
    //INTERNAL ARM VALUES
    private static double _firstLevelPos = 870; //original encoder values for first level of alliance hub/shared hub
    private static double _secondLevelPos = 750; //original encoder values for second level of alliance hub/shared hub
    private static double _thirdLevelPos = 580; //original encoder values for third level of alliance hub/shared hub
    private static double _maxPos = 1000; //original encoder values for maximum rotatable position of the arm
    //EXTERNAL ARM VALUES
    //Tune the previous values, not these values
    public static int FIRST_LEVEL_POS = (int)(_firstLevelPos*PPR_RATIO);
    public static int SECOND_LEVEL_POS = (int)(_secondLevelPos*PPR_RATIO);
    public static int THIRD_LEVEL_POS = (int)(_thirdLevelPos*PPR_RATIO);
    public static int MAX_POS = (int)(_maxPos*PPR_RATIO);
    //CAROUSEL POWERS
    public static double INIT_POWER = -0.6; //Initial power for carousel
    public static double FINAL_POWER = -1.0; //Final power for carousel
    //ARM POWERS
    public static double FIRST_LEVEL_POWER= 0.45;
    public static double SECOND_LEVEL_POWER = 0.525;
    public static double THIRD_LEVEL_POWER = 0.65;
    public static double GOING_DOWN_POWER = 0.3;
    //POSITIONS
    public static double RELEASE_SERVO_DEFAULT = 0.71;
    public static double BALANCE_SERVO_DEFAULT = 0.09;
    //TIMINGS
    public static int MILLI_ACC = 1100; //Milliseconds after carousel has been engaged to begin accelerating the carousel
    public static int MILLI_FINAL = 1100; //Milliseconds after carousel has been engaged to reach the final speed
    public static int MILLI_END = 1400; //Milliseconds after carousel has been engaged to stop spinning the carousel
    public static double FLICKER_TIME = 100; //Time in milliseconds for the flicker to release one freight
    //COEFFICIENTS
    public static double BALANCE_COEFFICIENT = 1300.1; //Balancing coefficient for ARM ENCODER VALUES
    public static double POTENTIOMETER_COEFFICIENT = 3.8; //Balancing Coefficient for POTENTIOMETER VOLTAGE VALUES
    public static double ACC_COEFFICIENT = 80000; //Rate at which to accelerate by (reciprocal of)
    public static double POW_COEFFICIENT = 1.2; //Exponent at which the balancing increases by
    //THRESHOLDS
    public static double DISTANCE_SENSOR_THRESHOLD = 80; //Specifies maximum distance in MM for distance sensor to detect freight (cannot be less than 50)
    public static double FORCE_SENSOR_THRESHOLD = 0.01; //Specifies minimum awmount of detected voltage change to detect freight (should be barely > 0)
    public static int DEBOUNCE_TIME = 250; //Minimum duration in milliseconds between certain presses (e.g. SlowMode, Carousel Engagement)
    public static int INTAKE_TIME = 500; //Milliseconds to run the intake in reverse after a freight has been detected
    //MISC
    public static Gamepad.RumbleEffect END_GAME_RUMBLE = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();
    public static Gamepad.RumbleEffect BOX_SECURED_RUMBLE = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 800)  //  Rumble right motor 100% for one whole second
            .build();
}