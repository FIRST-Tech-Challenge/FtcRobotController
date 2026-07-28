package org.firstinspires.ftc.teamcode.core.input;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Tracks one FTC gamepad's buttons and analog controls for TeleOp mappings.
 *
 * <p>Create this class with a non-null gamepad reference. Call {@link #update()} exactly once at
 * the beginning of each TeleOp loop, then use the query methods while mapping input to public
 * robot methods. The first update records the current button states without reporting presses or
 * releases, so buttons held when the OpMode starts do not create an unexpected edge event.</p>
 */
public class InputManager {
    private final Gamepad gamepad;

    private boolean a;
    private boolean b;
    private boolean x;
    private boolean y;
    private boolean leftBumper;
    private boolean rightBumper;

    private boolean previousA;
    private boolean previousB;
    private boolean previousX;
    private boolean previousY;
    private boolean previousLeftBumper;
    private boolean previousRightBumper;

    private double leftStickX;
    private double leftStickY;
    private double rightStickX;
    private double rightStickY;
    private double leftTrigger;
    private double rightTrigger;
    private boolean hasUpdated;

    /**
     * Creates an input manager for one FTC gamepad.
     *
     * @param gamepad the non-null gamepad that FTC updates each loop
     */
    public InputManager(Gamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("Input manager needs a gamepad.");
        }
        this.gamepad = gamepad;
    }

    /**
     * Captures the gamepad's current state for one TeleOp loop.
     *
     * <p>Call exactly once before reading this manager's button or analog queries for a loop.</p>
     */
    public void update() {
        if (hasUpdated) {
            copyCurrentButtonsToPrevious();
        }

        readCurrentGamepadState();

        if (!hasUpdated) {
            copyCurrentButtonsToPrevious();
            hasUpdated = true;
        }
    }

    public boolean isAHeld() {
        return a;
    }

    public boolean wasAJustPressed() {
        return a && !previousA;
    }

    public boolean wasAJustReleased() {
        return !a && previousA;
    }

    public boolean isBHeld() {
        return b;
    }

    public boolean wasBJustPressed() {
        return b && !previousB;
    }

    public boolean wasBJustReleased() {
        return !b && previousB;
    }

    public boolean isXHeld() {
        return x;
    }

    public boolean wasXJustPressed() {
        return x && !previousX;
    }

    public boolean wasXJustReleased() {
        return !x && previousX;
    }

    public boolean isYHeld() {
        return y;
    }

    public boolean wasYJustPressed() {
        return y && !previousY;
    }

    public boolean wasYJustReleased() {
        return !y && previousY;
    }

    public boolean isLeftBumperHeld() {
        return leftBumper;
    }

    public boolean wasLeftBumperJustPressed() {
        return leftBumper && !previousLeftBumper;
    }

    public boolean wasLeftBumperJustReleased() {
        return !leftBumper && previousLeftBumper;
    }

    public boolean isRightBumperHeld() {
        return rightBumper;
    }

    public boolean wasRightBumperJustPressed() {
        return rightBumper && !previousRightBumper;
    }

    public boolean wasRightBumperJustReleased() {
        return !rightBumper && previousRightBumper;
    }

    public double getLeftStickX() {
        return leftStickX;
    }

    public double getLeftStickY() {
        return leftStickY;
    }

    public double getRightStickX() {
        return rightStickX;
    }

    public double getRightStickY() {
        return rightStickY;
    }

    public double getLeftTrigger() {
        return leftTrigger;
    }

    public double getRightTrigger() {
        return rightTrigger;
    }

    private void copyCurrentButtonsToPrevious() {
        previousA = a;
        previousB = b;
        previousX = x;
        previousY = y;
        previousLeftBumper = leftBumper;
        previousRightBumper = rightBumper;
    }

    private void readCurrentGamepadState() {
        a = gamepad.a;
        b = gamepad.b;
        x = gamepad.x;
        y = gamepad.y;
        leftBumper = gamepad.left_bumper;
        rightBumper = gamepad.right_bumper;
        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightStickX = gamepad.right_stick_x;
        rightStickY = gamepad.right_stick_y;
        leftTrigger = gamepad.left_trigger;
        rightTrigger = gamepad.right_trigger;
    }
}
