package org.firstinspires.ftc.teamcode.robot.config;

/*
 * Holds the left intake servo's settings for every opmode, and remembers where the servo is so
 * each opmode leaves it where the last one did. A static variable keeps its value when one opmode
 * stops and another starts, but goes back to false when the robot restarts. So after a restart the
 * servo goes to START_POSITION at INIT.
 *
 * ROTATION_DEGREES is how far the servo turns from its start position. Make it negative to turn
 * the other way. SERVO_RANGE_DEGREES is the servo's full travel, which is what positions 0 to 1
 * cover. TURNED_POSITION has to stay between 0 and 1, so from a start position of 0.5 and a 300
 * degree servo, ROTATION_DEGREES can be at most 150 either way.
 */
public class LeftServoMemory {
    public static final double ROTATION_DEGREES = 135;
    public static final double SERVO_RANGE_DEGREES = 300;
    public static final double START_POSITION = 0.5;
    public static final double TURNED_POSITION = START_POSITION + ROTATION_DEGREES / SERVO_RANGE_DEGREES;

    public static boolean turned = false;

    public static double position() {
        return turned ? TURNED_POSITION : START_POSITION;
    }
}
