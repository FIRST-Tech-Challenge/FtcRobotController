package org.firstinspires.ftc.Team19567.util;

/**
 * Enum describing all of the possible states the robot can be in during a TeleOP program. <br>
 * Never really used, since Roadrunner in TeleOP was never really used. <br>
 * Actually useful thing, more on enums <a href="https://docs.oracle.com/javase/tutorial/java/javaOO/enum.html">here</a>. <br>
 */
public enum TELEOP_STATE {
    /**
     * Roadrunner is driving the robot somewhere
     */
    AUTOMATION_ROADRUNNER_MOVEMENT,
    /**
     * The robot is delivering the freight by rotating the flicker
     */
    AUTOMATION_FLICKER,
    /**
     * The driver has control of the robot (standard TeleOP)
     */
    DRIVER_CONTROL
}
