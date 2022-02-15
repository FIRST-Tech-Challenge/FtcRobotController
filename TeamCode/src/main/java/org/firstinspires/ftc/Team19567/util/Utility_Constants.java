package org.firstinspires.ftc.Team19567.util;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

@Config
public class Utility_Constants {
    public static double PPR_RATIO = 1.95509087; //ratio of old encoder values to current encoder values
    public static double INTAKE_SPEED = 1.0; //speed to set the intake motor to
    public static double TURN_SENSITIVITY = 0.9; //driver-controlled sensitivity for turning
    public static double STRAFE_SENSITIVITY = 1.0; //driver-controlled sensitivity for strafing
    public static double MAX_SENSITIVITY = 1.0; //driver-controller overall sensitivity, basically
    public static double SLOWMODE_MULT = 0.3; //speed multiplier during SlowMode
    //
    public static double _firstLevelPos = 870; //original encoder values for first level of alliance hub/shared hub
    public static double _secondLevelPos = 750; //original encoder values for second level of alliance hub/shared hub
    public static double _thirdLevelPos = 600; //original encoder values for third level of alliance hub/shared hub
    //Tune the previous values, not these values
    public static int FIRST_LEVEL_POS = (int)(_firstLevelPos*PPR_RATIO);
    public static int SECOND_LEVEL_POS = (int)(_secondLevelPos*PPR_RATIO);
    public static int THIRD_LEVEL_POS = (int)(_thirdLevelPos*PPR_RATIO);
    //
    public static int MILLI_ACC = 1100; //Milliseconds after carousel has been engaged to begin accelerating the carousel
    public static int MILLI_FINAL = 1100; //Milliseconds after carousel has been engaged to reach the final speed
    public static int MILLI_END = 1400; //Milliseconds after carousel has been engaged to stop spinning the carousel
    public static int DEBOUNCE_TIME = 500; //Minimum duration in milliseconds between certain presses (e.g. SlowMode, Carousel Engagement)
    public static double INIT_POWER = -0.55; //Initial power for carousel
    public static double ACC_COEFFICIENT = 80000; //Rate at which to accelerate by (reciprocal of)
    public static double FINAL_POWER = -1.0; //Final power for carousel
    public static double FLICKER_TIME = 400; //Time in milliseconds for the flicker to release one freight
    public static Gamepad.RumbleEffect END_GAME_RUMBLE = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();

    public static Gamepad.RumbleEffect BOX_SECURED_RUMBLE = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 1000)  //  Rumble right motor 100% for one whole second
            .build();
}