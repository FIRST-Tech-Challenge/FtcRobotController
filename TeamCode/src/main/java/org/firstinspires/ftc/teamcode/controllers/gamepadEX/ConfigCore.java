package org.firstinspires.ftc.teamcode.team7786.controller.gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
atm this is just a config class, im trying to find a way to make this extensible so yea lets hope that
works




 */
public class ConfigCore {
    GamepadEx gamepadEx;
    StandardButton aButton, XButton, dpadUpButton, dpadDownButton;
    ToggleButton bButton;
    VariableInput leftStickXButton, leftStickYButton, rightStickXButton, rightStickYButton;
    VariableInputButton rightTriggerButton;

    public boolean a, b, x, dpad_up, dpad_down, right_trigger_button;
    public float left_stick_x, left_stick_y, right_stick_x, right_stick_y;

    /**
     * @param gamepad the gamepad, usually gamepad1 or gamepad2
     */
    public ConfigCore(Gamepad gamepad) {
        this.gamepadEx = new GamepadEx(gamepad);

        aButton = gamepadEx.getAButton();
        XButton = gamepadEx.getXButton();
        dpadUpButton = gamepadEx.getDpad_upButton();
        dpadDownButton = gamepadEx.getDpad_DownButton();

        bButton = gamepadEx.getBButtonToggled();

        leftStickXButton = gamepadEx.getLeftStickX();
        leftStickYButton = gamepadEx.getLeftStickY();

        rightStickXButton = gamepadEx.getRightStickX();
        rightStickYButton = gamepadEx.getRightStickY();

        rightTriggerButton = gamepadEx.getRightTriggerButton(0.5);
    }

    /*
    then you define each button that will be used by calling the factory method from
    the gamepad plus class like this I recommend grouping them by button type for readability
     */





    /*
    from here you will make a method for each button, trying to keep in line with the
    method names given in the gamepad class even if this violates your naming conventions
    this will allow the improved gamepad to be almost a drop in replacement for the standard gamepad
     */

    //Standard buttons
    public boolean aButton() {
        return aButton.pressed();
    }

    public boolean xButton() {
        return XButton.pressed();
    }

    public boolean dpad_upButton() {
        return dpadUpButton.pressed();
    }

    public boolean dpad_downButton() {
        return dpadDownButton.pressed();
    }

    //Toggle Button
    public boolean bButton() {
        return bButton.getState();
    }

    //Variable Inputs
    public float left_stick_xButton() {
        return leftStickXButton.getPosition();
    }

    public float left_stick_yButton() {
        return leftStickYButton.getPosition();
    }

    public float right_stick_xButton() {
        return rightStickXButton.getPosition();
    }

    public float right_stick_yButton() {
        return rightStickYButton.getPosition();
    }

    public boolean right_triggerButton() {
        return rightTriggerButton.pressed();
    }


}
