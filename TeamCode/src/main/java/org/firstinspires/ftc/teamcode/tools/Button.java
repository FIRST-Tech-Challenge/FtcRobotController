package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Button {
    private boolean buttonState;
    private boolean buttonPreviousState;
    private final NAME buttonName;

    public enum NAME {
        A, B, X, Y, DPAD_DOWN, DPAD_UP, DPAD_LEFT, DPAD_RIGHT,
        LEFT_BUMPER, RIGHT_BUMPER, LEFT_TRIGGER, RIGHT_TRIGGER
    }

    public Button(Gamepad gamepad, NAME buttonName) {
        this.buttonName = buttonName;
        buttonState = false;//  = generateButtonFunctionMap(gamepad).get(buttonName);
        buttonPreviousState = false; //generateButtonFunctionMap(gamepadPreviousState).get(buttonName);
        this.updateButton(gamepad);


    }
    public void updateButton(Gamepad gamepad) {

        buttonPreviousState = buttonState;
        switch (buttonName) {
            case A:
                buttonState = gamepad.a;
                break;
            case B:
                buttonState = gamepad.b;
                break;
            case X:
                buttonState = gamepad.x;
                break;
            case Y:
                buttonState = gamepad.y;
                break;
            case DPAD_DOWN:
                buttonState = gamepad.dpad_down;
                break;
            case DPAD_UP:
                buttonState = gamepad.dpad_up;
                break;
            case DPAD_LEFT:
                buttonState = gamepad.dpad_left;
                break;
            case DPAD_RIGHT:
                buttonState = gamepad.dpad_right;
                break;
            case LEFT_BUMPER:
                buttonState = gamepad.left_bumper;
                break;
            case RIGHT_BUMPER:
                buttonState = gamepad.right_bumper;
                break;
            case LEFT_TRIGGER:
                buttonState = gamepad.left_trigger >= 0.5;
                break;
            case RIGHT_TRIGGER:
                buttonState = gamepad.right_trigger >= 0.5;
                break;
        }
    }


    public boolean Pressed() {
        return !buttonPreviousState && buttonState;
    }
    public boolean Released() {
        return buttonPreviousState && !buttonState;
    }
    public boolean On() {
        return buttonState;
    }
    public boolean Off() {
        return !buttonState;
    }
}