package org.firstinspires.ftc.teamcode.tatooine.utils.gamepads;

/**
 * A utility class containing enums that represent various gamepad inputs:
 * <ul>
 *   <li>Buttons (e.g., CROSS, CIRCLE)</li>
 *   <li>Triggers (LEFT_TRIGGER, RIGHT_TRIGGER)</li>
 *   <li>Stick axes (LEFT_STICK_X, RIGHT_STICK_X, etc.)</li>
 * </ul>
 * These enums can be used with classes like {@code EasyGamepad} to refer
 * to specific inputs in a concise and readable way.
 */
public final class GamepadKeys {

    /**
     * Represents the standard face buttons, bumpers, and D-pad inputs on a typical gamepad,
     * plus START and BACK buttons and stick buttons.
     */
    public enum Button {
        CROSS,
        TRIANGLE,
        CIRCLE,
        SQUARE,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        BACK,
        START,
        DPAD_UP,
        DPAD_DOWN,
        DPAD_LEFT,
        DPAD_RIGHT,
        LEFT_STICK_BUTTON,
        RIGHT_STICK_BUTTON
    }

    /**
     * Represents the triggers on the gamepad: left or right.
     */
    public enum Trigger {
        LEFT_TRIGGER,
        RIGHT_TRIGGER
    }

    /**
     * Represents the individual axes of the two analog sticks.
     * For example, {@code LEFT_STICK_X} indicates the x-axis (horizontal) of the left stick.
     */
    public enum Stick {
        LEFT_STICK_X,
        RIGHT_STICK_X,
        LEFT_STICK_Y,
        RIGHT_STICK_Y
    }

    /**
     * Private constructor to prevent instantiation.
     */
    private GamepadKeys() {
        // This class is not meant to be instantiated.
    }
}
