package com.arcrobotics.ftclib.gamepad;

/**
 * An enumerator for the different keys on the gamepad, including bumpers,
 * buttons, and triggers.
 */

public class GamepadKeys {

    public enum Button {
        Y, X, A, B, LEFT_BUMPER, RIGHT_BUMPER, BACK,
        START, DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        LEFT_STICK_BUTTON, RIGHT_STICK_BUTTON
    }

    public enum Trigger {
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

}
