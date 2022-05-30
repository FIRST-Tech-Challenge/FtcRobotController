package org.firstinspires.ftc.Team19567.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * INSANELY useful file containing all of the utility constants (e.g. timeouts, multipliers, thresholds) used throughout the entire repository.
 */

@Config //Tunable using FTC Dashboard, making life 1000x easier
public class Utility_Constants {
    /** Ratio of old encoder values to current encoder values (after motor change) */
    public static double PPR_RATIO = 1.95509087;
    /** Speed to set the intake motor to when intaking freight */
    public static double INTAKE_SPEED = 1.0;
    /** Speed to eject freight at */
    public static double EJECTION_SPEED = -0.6;
    /** Driver-controlled sensitivity for turning */
    public static double TURN_SENSITIVITY = 0.9;
    /** Driver-controlled sensitivity for strafing */
    public static double STRAFE_SENSITIVITY = 1.0;
    /** Driver-controller overall sensitivity, basically */
    public static double MAX_SENSITIVITY = 1.0;
    /** Speed multiplier during SlowMode */
    public static double SLOWMODE_MULT = 0.3;
    /** Sensitivity for release servo during manual operation */
    public static double SERVO_SENSITIVITY = 0.05;
    /** Original encoder values for first level of alliance hub/shared hub */
    private static double _firstLevelPos = 820;
    /** Original encoder values for second level of alliance hub */
    private static double _secondLevelPos = 700;
    /** Original encoder values for third level of alliance hub */
    private static double _thirdLevelPos = 550;
    /** Original encoder values for maximum rotatable position of the arm */
    private static double _maxPos = 1000;
    //Tune the previous values, NOT these values!
    /** Actual encoder values for first level of alliance hub/shared hub; do not tune this! */
    public static int FIRST_LEVEL_POS = (int)(_firstLevelPos*PPR_RATIO);
    /** Actual encoder values for second level of alliance hub; do not tune this! */
    public static int SECOND_LEVEL_POS = (int)(_secondLevelPos*PPR_RATIO);
    /** Actual encoder values for third level of alliance hub; do not tune this! */
    public static int THIRD_LEVEL_POS = (int)(_thirdLevelPos*PPR_RATIO);
    /** Actual encoder values for maximum rotatable position of the arm; do not tune this! */
    public static int MAX_POS = (int)(_maxPos*PPR_RATIO);
    /** Initial power for carousel */
    public static double INIT_POWER = -0.6;
    /** Final power for carousel */
    public static double FINAL_POWER = -1.0;
    /** Arm power for first level of alliance/shared hub */
    public static double FIRST_LEVEL_POWER= 0.45;
    /** Arm power for second level of alliance hub */
    public static double SECOND_LEVEL_POWER = 0.5;
    /** Arm power for third level of alliance hub */
    public static double THIRD_LEVEL_POWER = 0.65;
    /** Arm power when resetting/going down */
    public static double GOING_DOWN_POWER = 0.3;
    /** Default (reset) position of the release servo */
    public static double RELEASE_SERVO_DEFAULT = 0.71;
    /** Default (reset) position of the balance servo */
    public static double BALANCE_SERVO_DEFAULT = 0.04;
    /** Milliseconds after carousel has been engaged to begin accelerating the carousel */
    public static int MILLI_ACC = 1100;
    /** Milliseconds after carousel has been engaged to reach the final speed */
    public static int MILLI_FINAL = 1100;
    /** Milliseconds after carousel has been engaged to stop spinning the carousel */
    public static int MILLI_END = 1400;
    /** Time in milliseconds for the flicker to release one freight */
    public static double FLICKER_TIME = 100;
    /** Balancing coefficient for ARM ENCODER VALUES */
    public static double BALANCE_COEFFICIENT = 850;
    /** Balancing Coefficient for POTENTIOMETER VOLTAGE VALUES */
    public static double POTENTIOMETER_COEFFICIENT = 3.8;
    /** (Reciprocal of the) rate at which to accelerate by */
    public static double ACC_COEFFICIENT = 80000;
    /** Exponent at which the balancing increases by */
    public static double POW_COEFFICIENT = 1.2;
    /** Specifies maximum distance in MM for distance sensor to detect freight (cannot be less than 50) */
    public static double DISTANCE_SENSOR_THRESHOLD = 80;
    /** Specifies minimum awmount of detected voltage change to detect freight (should be barely > 0) */
    public static double FORCE_SENSOR_THRESHOLD = 0.01;
    /** Minimum duration in milliseconds between certain presses (e.g. SlowMode, Carousel Engagement) */
    public static int DEBOUNCE_TIME = 250;
    /** Milliseconds to run the intake in reverse after a freight has been detected */
    public static int INTAKE_TIME = 500;
    /** Rumble effect for time signals */
    public static Gamepad.RumbleEffect END_GAME_RUMBLE = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();
    /** Rumble effect that plays when a box is intaked */
    public static Gamepad.RumbleEffect BOX_SECURED_RUMBLE = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 800)  //  Rumble right motor 100% for one whole second
            .build();
}