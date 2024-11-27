package org.firstinspires.ftc.teamcode.tatooine.utils.gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;

public class EasyGamepad {
    boolean triangleTriggered = false;
    boolean circleTriggered = false;
    boolean crossTriggered = false;

    boolean squareTriggered = false;

    boolean dPadUpTriggered = false;

    boolean dPadDownTriggered = false;

    boolean dPadRightTriggered = false;

    boolean dPadLeftTriggered = false;

    boolean startTriggered = false;
    boolean backTriggered = false;
    Gamepad gamepad;
    Gamepad prevGamepad;

    public EasyGamepad(Gamepad gamepad) {
        this.prevGamepad = this.gamepad;
        this.gamepad = gamepad;
    }

    public void update(Gamepad gamepad) {
        this.prevGamepad = this.gamepad;
        this.gamepad = gamepad;
    }

    public boolean getButton(GamepadKeys.Button button) {
        boolean buttonValue;
        switch (button) {
            case CROSS:
                buttonValue = gamepad.cross;
                break;
            case CIRCLE:
                buttonValue = gamepad.circle;
                break;
            case TRIANGLE:
                buttonValue = gamepad.triangle;
                break;
            case SQUARE:
                buttonValue = gamepad.square;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper;
                break;
            case DPAD_UP:
                buttonValue = gamepad.dpad_up;
                break;
            case DPAD_DOWN:
                buttonValue = gamepad.dpad_down;
                break;
            case DPAD_LEFT:
                buttonValue = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonValue = gamepad.dpad_right;
                break;
            case BACK:
                buttonValue = gamepad.back;
                break;
            case START:
                buttonValue = gamepad.start;
                break;
            case LEFT_STICK_BUTTON:
                buttonValue = gamepad.left_stick_button;
                break;
            case RIGHT_STICK_BUTTON:
                buttonValue = gamepad.right_stick_button;
                break;
            default:
                buttonValue = false;
                break;
        }
        return buttonValue;
    }

    public double getStick(GamepadKeys.Stick stick) {
        double stickValue;
        switch (stick) {
            case LEFT_STICK_X:
                stickValue = gamepad.left_stick_x;
                break;
            case LEFT_STICK_Y:
                stickValue = -gamepad.left_stick_y;
                break;
            case RIGHT_STICK_X:
                stickValue = gamepad.right_stick_x;
                break;
            case RIGHT_STICK_Y:
                stickValue = -gamepad.right_stick_y;
                break;
            default:
                stickValue = 0;
                break;
        }
        return stickValue;
    }

    public double getTrigger(GamepadKeys.Trigger trigger) {
        double triggerValue;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger;
                break;
            default:
                triggerValue = 0;
        }
        return triggerValue;
    }

    public boolean stateJustChangeButton(GamepadKeys.Button button) {
        boolean buttonValue;
        switch (button) {
            case CROSS:
                buttonValue = gamepad.cross && !prevGamepad.cross;
                break;
            case CIRCLE:
                buttonValue = gamepad.circle && !prevGamepad.circle;
                break;
            case TRIANGLE:
                buttonValue = gamepad.triangle && !prevGamepad.triangle;
                break;
            case SQUARE:
                buttonValue = gamepad.square && !prevGamepad.square;
                break;
            case LEFT_BUMPER:
                buttonValue = gamepad.left_bumper && !prevGamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonValue = gamepad.right_bumper && !prevGamepad.right_bumper;
                break;
            default:
                buttonValue = false;
                break;
        }
        return buttonValue;
    }

    public boolean stateJustChangeStick(GamepadKeys.Stick stick) {
        boolean stickValue;
        switch (stick) {
            case LEFT_STICK_X:
                stickValue = gamepad.left_stick_x != prevGamepad.left_stick_x;
                break;
            case LEFT_STICK_Y:
                stickValue = gamepad.left_stick_y != prevGamepad.left_stick_y;
                break;
            case RIGHT_STICK_X:
                stickValue = gamepad.right_stick_x != prevGamepad.right_stick_x;
                break;
            case RIGHT_STICK_Y:
                stickValue = gamepad.right_stick_y != prevGamepad.right_stick_y;
                break;
            default:
                stickValue = false;
                break;
        }
        return stickValue;
    }

    public boolean stateJustChangedTriggered(GamepadKeys.Trigger trigger) {
        boolean triggerValue;
        switch (trigger) {
            case LEFT_TRIGGER:
                triggerValue = gamepad.left_trigger != prevGamepad.left_trigger;
                break;
            case RIGHT_TRIGGER:
                triggerValue = gamepad.right_trigger != prevGamepad.right_trigger;
                break;
            default:
                triggerValue = false;
                break;
        }
        return triggerValue;
    }
    public boolean buttonTriggered(GamepadKeys.Button button) {
        boolean buttonValue;
        switch (button) {
            case CROSS:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.CROSS) && getButton(GamepadKeys.Button.CROSS) && crossTriggered;
                break;
            case CIRCLE:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.CIRCLE) && getButton(GamepadKeys.Button.CIRCLE) && circleTriggered;
                break;
            case TRIANGLE:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.TRIANGLE) && getButton(GamepadKeys.Button.TRIANGLE) && triangleTriggered;
                break;
            case SQUARE:
                buttonValue = buttonValue = stateJustChangeButton(GamepadKeys.Button.SQUARE) && getButton(GamepadKeys.Button.SQUARE) && squareTriggered;
                break;
            case BACK:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.BACK) && getButton(GamepadKeys.Button.BACK) && backTriggered;
                break;
            case START:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.START) && getButton(GamepadKeys.Button.START) && startTriggered;
                break;
            case DPAD_UP:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.DPAD_UP) && getButton(GamepadKeys.Button.DPAD_UP) && dPadUpTriggered;
                break;
            case DPAD_DOWN:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.DPAD_DOWN) && getButton(GamepadKeys.Button.DPAD_DOWN) && dPadDownTriggered;
                break;
            case DPAD_LEFT:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.DPAD_LEFT) && getButton(GamepadKeys.Button.DPAD_LEFT) && dPadLeftTriggered;
                break;
            case DPAD_RIGHT:
                buttonValue = stateJustChangeButton(GamepadKeys.Button.DPAD_RIGHT) && getButton(GamepadKeys.Button.DPAD_RIGHT) && dPadRightTriggered;
                break;
            default:
                buttonValue = false;
                break;
        }
        return buttonValue;
    }
}