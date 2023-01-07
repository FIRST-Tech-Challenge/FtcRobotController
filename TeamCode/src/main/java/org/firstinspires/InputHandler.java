package org.firstinspires;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputHandler {
    private Gamepad gamepad;

    // Holds 2 frames of each input
    // [frame 1, frame 2]
    private boolean[] a;
    private boolean[] b;
    private boolean[] x;
    private boolean[] y;
    private boolean[] dpadUp;
    private boolean[] dpadDown;
    private boolean[] dpadLeft;
    private boolean[] dpadRight;
    private boolean[] start;
    private boolean[] back;
    private boolean[] rightBumper;
    private boolean[] leftBumper;
    private boolean[] leftStickButton;
    private boolean[] rightStickButton;

    public void update() {
        a[1] = a[0];
        b[1] = b[0];
        x[1] = x[0];
        y[1] = y[0];
        dpadUp[1] = dpadUp[0];
        dpadDown[1] = dpadDown[0];
        dpadLeft[1] = dpadLeft[0];
        dpadRight[1] = dpadRight[0];
        rightBumper[1] = rightBumper[0];
        leftBumper[1] = leftBumper[0];
        leftStickButton[1] = leftStickButton[0];
        rightStickButton[1] = rightStickButton[0];
        a[0] = gamepad.a;
        b[0] = gamepad.b;
        x[0] = gamepad.x;
        y[0] = gamepad.y;
        dpadUp[0] = gamepad.dpad_up;
        dpadDown[0] = gamepad.dpad_down;
        dpadLeft[0] = gamepad.dpad_left;
        dpadRight[0] = gamepad.dpad_right;
        start[0] = gamepad.start;
        back[0] = gamepad.back;
        rightBumper[0] = gamepad.right_bumper;
        leftBumper[0] = gamepad.left_bumper;
        leftStickButton[0] = gamepad.left_stick_button;
        rightStickButton[0] = gamepad.right_stick_button;
    }

    public InputHandler(Gamepad gamepad) {
        this.gamepad = gamepad;
        a = new boolean[]{false, false};
        b = new boolean[]{false, false};
        x = new boolean[]{false, false};
        y = new boolean[]{false, false};
        dpadUp = new boolean[]{false, false};
        dpadDown = new boolean[]{false, false};
        dpadLeft = new boolean[]{false, false};
        dpadRight = new boolean[]{false, false};
        start = new boolean[]{false, false};
        back = new boolean[]{false, false};
        rightBumper = new boolean[]{false, false};
        leftBumper = new boolean[]{false, false};
        leftStickButton = new boolean[]{false, false};
        rightStickButton = new boolean[]{false, false};
    }

    public enum Digital {
        A, B, X, Y, DPAD_UP, DPAD_DOWN, DPAD_RIGHT, DPAD_LEFT, START, BACK, RIGHT_BUMPER, RIGHT_STICK_BUTTON, LEFT_BUMPER, LEFT_STICK_BUTTON;
    }
    public enum Analogue {
        LEFT_STICK_X, LEFT_STICK_Y, RIGHT_STICK_X, RIGHT_STICK_Y, LEFT_TRIGGER, RIGHT_TRIGGER;
    }

    public double getAnalogueInput(Analogue stick) {
        switch (stick) {
            case LEFT_STICK_X: return gamepad.left_stick_x;
            case LEFT_STICK_Y: return gamepad.left_stick_y;
            case RIGHT_STICK_X: return gamepad.right_stick_x;
            case RIGHT_STICK_Y: return gamepad.right_stick_y;
            case LEFT_TRIGGER: return gamepad.left_trigger;
            case RIGHT_TRIGGER: return gamepad.right_trigger;
            default: return 0.0;
        }
    }
    public boolean justPressed(Digital button) {
        switch (button) {
            case A: return a[0] && !a[1];
            case B: return b[0] && !b[1];
            case X: return x[0] && !x[1];
            case Y: return y[0] && !y[1];
            case DPAD_UP: return dpadUp[0] && !dpadUp[1];
            case DPAD_DOWN: return dpadDown[0] && !dpadDown[1];
            case DPAD_LEFT: return dpadLeft[0] && !dpadLeft[1];
            case DPAD_RIGHT: return dpadRight[0] && !dpadRight[1];
            case START: return start[0] && !start[1];
            case BACK: return back[0] && !back[1];
            case LEFT_BUMPER: return leftBumper[0] && !leftBumper[1];
            case RIGHT_BUMPER: return rightBumper[0] && !rightBumper[1];
            case LEFT_STICK_BUTTON: return leftStickButton[0] && !leftStickButton[1];
            case RIGHT_STICK_BUTTON: return rightStickButton[0] && !rightStickButton[1];
            default: return false;
        }
    }
    public boolean justReleased(Digital button) {
        switch (button) {
            case A: return a[1] && !a[0];
            case B: return b[1] && !b[0];
            case X: return x[1] && !x[0];
            case Y: return y[1] && !y[0];
            case DPAD_UP: return dpadUp[1] && !dpadUp[0];
            case DPAD_DOWN: return dpadDown[1] && !dpadDown[0];
            case DPAD_LEFT: return dpadLeft[1] && !dpadLeft[0];
            case DPAD_RIGHT: return dpadRight[1] && !dpadRight[0];
            case START: return start[1] && !start[0];
            case BACK: return back[1] && !back[0];
            case LEFT_BUMPER: return leftBumper[1] && !leftBumper[0];
            case RIGHT_BUMPER: return rightBumper[1] && !rightBumper[0];
            case LEFT_STICK_BUTTON: return leftStickButton[1] && !leftStickButton[0];
            case RIGHT_STICK_BUTTON: return rightStickButton[1] && !rightStickButton[0];
            default: return false;
        }
    }
    public boolean isPressed(Digital button) {
        switch (button) {
            case A: return a[0];
            case B: return b[0];
            case X: return x[0];
            case Y: return y[0];
            case DPAD_UP: return dpadUp[0];
            case DPAD_DOWN: return dpadDown[0];
            case DPAD_LEFT: return dpadLeft[0];
            case DPAD_RIGHT: return dpadRight[0];
            case START: return start[0];
            case BACK: return back[0];
            case LEFT_BUMPER: return leftBumper[0];
            case RIGHT_BUMPER: return rightBumper[0];
            case LEFT_STICK_BUTTON: return leftStickButton[0];
            case RIGHT_STICK_BUTTON: return rightStickButton[0];
            default: return false;
        }
    }



}
