package org.firstinspires.ftc.teamcode.types;

/**
 * Which alliance the robot is playing for this match.
 *
 * <p>The robot figures this out by itself: a switch reads which alliance badge is
 * mounted, during INIT. Drivers never pick a red or blue OpMode. See
 * docs/decisions/0001-one-opmode-alliance-from-hardware.md.
 */
public enum Alliance {
    RED,
    BLUE,
    /** The switch has not been read, is not installed, or reads both or neither. */
    UNKNOWN
}
