package org.firstinspires.ftc.teamcode.tatooine.utils.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * A utility class that simplifies checking button presses, stick movements,
 * and triggers for a single gamepad. It also tracks the previous state of the
 * gamepad to detect changes or newly pressed buttons.
 *
 * <p>Usage:
 * <pre>
 *     EasyGamepad egp = new EasyGamepad(gamepad1);
 *
 *     // In your loop:
 *     egp.update(gamepad1);
 *
 *     if (egp.justPressedButton(GamepadKeys.Button.CIRCLE)) {
 *         // Circle was pressed this frame, but not last frame
 *     }
 *
 *     double leftY = egp.getStick(GamepadKeys.Stick.LEFT_STICK_Y);
 * </pre>
 */
public class EasyGamepad {

    // -------------------------------------------------------------------------
    // Fields
    // -------------------------------------------------------------------------

    /** The current state of the gamepad. */
    private Gamepad gamepad;

    /** The previous state of the gamepad (to detect new presses or state changes). */
    private Gamepad prevGamepad;

    // -------------------------------------------------------------------------
    // Constructor
    // -------------------------------------------------------------------------

    /**
     * Creates a new EasyGamepad instance with an initial gamepad reference.
     * A default empty {@link Gamepad} is assigned to {@code prevGamepad} to avoid null checks.
     *
     * @param gamepad The current gamepad reference (e.g., {@code gamepad1}).
     */
    public EasyGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.prevGamepad = new Gamepad(); // start with an empty state
    }

    // -------------------------------------------------------------------------
    // Updating
    // -------------------------------------------------------------------------

    /**
     * Updates the EasyGamepad with a fresh reference to the new gamepad state each loop.
     * Call this method at the start of your loop to ensure all methods use the latest state.
     *
     * @param gamepad The updated gamepad reference (usually the same gamepad1 or gamepad2).
     */
    public void update(Gamepad gamepad) {
        this.prevGamepad = this.gamepad;
        this.gamepad = gamepad;
    }

    // -------------------------------------------------------------------------
    // Button Queries
    // -------------------------------------------------------------------------

    /**
     * Checks if a given button on the gamepad is currently pressed.
     *
     * @param button The button enum (e.g. {@code GamepadKeys.Button.CROSS}).
     * @return {@code true} if the button is pressed, {@code false} otherwise.
     */
    public boolean getButton(GamepadKeys.Button button) {
        switch (button) {
            case CROSS:
                return gamepad.cross;
            case CIRCLE:
                return gamepad.circle;
            case TRIANGLE:
                return gamepad.triangle;
            case SQUARE:
                return gamepad.square;
            case LEFT_BUMPER:
                return gamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper;
            case DPAD_UP:
                return gamepad.dpad_up;
            case DPAD_DOWN:
                return gamepad.dpad_down;
            case DPAD_LEFT:
                return gamepad.dpad_left;
            case DPAD_RIGHT:
                return gamepad.dpad_right;
            case BACK:
                return gamepad.back;
            case START:
                return gamepad.start;
            case LEFT_STICK_BUTTON:
                return gamepad.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gamepad.right_stick_button;
            default:
                return false;
        }
    }

    /**
     * Checks if a button was just pressed this frame (i.e., it's pressed now
     * but was not pressed in the previous state).
     *
     * @param button The button to check.
     * @return {@code true} if the button was pressed this frame, otherwise {@code false}.
     */
    public boolean justPressedButton(GamepadKeys.Button button) {
        switch (button) {
            case CROSS:
                return gamepad.cross && !prevGamepad.cross;
            case CIRCLE:
                return gamepad.circle && !prevGamepad.circle;
            case TRIANGLE:
                return gamepad.triangle && !prevGamepad.triangle;
            case SQUARE:
                return gamepad.square && !prevGamepad.square;
            case LEFT_BUMPER:
                return gamepad.left_bumper && !prevGamepad.left_bumper;
            case RIGHT_BUMPER:
                return gamepad.right_bumper && !prevGamepad.right_bumper;
            default:
                return false;
        }
    }

    // -------------------------------------------------------------------------
    // Stick Queries
    // -------------------------------------------------------------------------

    /**
     * Retrieves the current value of a specified stick axis (e.g. left_x, right_y).
     * Negative is automatically applied to Y axes if desired (e.g., for forward=negative).
     *
     * @param stick The enum representing which stick axis to read.
     * @return The numeric value of the stick axis in [-1.0, 1.0].
     */
    public double getStick(GamepadKeys.Stick stick) {
        switch (stick) {
            case LEFT_STICK_X:
                return gamepad.left_stick_x;
            case LEFT_STICK_Y:
                return -gamepad.left_stick_y; // Negative to invert typical up/down orientation
            case RIGHT_STICK_X:
                return gamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return -gamepad.right_stick_y;
            default:
                return 0;
        }
    }

    /**
     * Checks if the stick value changed from the previous state to the current state.
     *
     * @param stick The stick axis to check.
     * @return {@code true} if the stick's value has changed, otherwise {@code false}.
     */
    public boolean stateChangedStick(GamepadKeys.Stick stick) {
        switch (stick) {
            case LEFT_STICK_X:
                return gamepad.left_stick_x != prevGamepad.left_stick_x;
            case LEFT_STICK_Y:
                return gamepad.left_stick_y != prevGamepad.left_stick_y;
            case RIGHT_STICK_X:
                return gamepad.right_stick_x != prevGamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return gamepad.right_stick_y != prevGamepad.right_stick_y;
            default:
                return false;
        }
    }

    // -------------------------------------------------------------------------
    // Trigger Queries
    // -------------------------------------------------------------------------

    /**
     * Retrieves the current value of the specified trigger.
     *
     * @param trigger The enum representing which trigger to read.
     * @return The trigger value in the range [0.0, 1.0].
     */
    public double getTrigger(GamepadKeys.Trigger trigger) {
        switch (trigger) {
            case LEFT_TRIGGER:
                return gamepad.left_trigger;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger;
            default:
                return 0;
        }
    }

    /**
     * Checks if the trigger value changed from the previous state to the current state.
     *
     * @param trigger The trigger to check.
     * @return {@code true} if the trigger's value has changed, otherwise {@code false}.
     */
    public boolean stateChangedTriggers(GamepadKeys.Trigger trigger) {
        switch (trigger) {
            case LEFT_TRIGGER:
                return gamepad.left_trigger != prevGamepad.left_trigger;
            case RIGHT_TRIGGER:
                return gamepad.right_trigger != prevGamepad.right_trigger;
            default:
                return false;
        }
    }
}
